/**
  ******************************************************************************
  * @file    arm_pwm.c
  * @brief   角度 → 脉宽 → CCR 的实现（T1 的核心逻辑）
  ******************************************************************************
  * @note    整个换算链就三行公式，别被代码行数吓到：
  *
  *            ① 角度夹进该关节的 [deg_min, deg_max]
  *
  *            ② 线性插值成脉宽：
  *                 us = us_at_deg_min
  *                    + (deg - deg_min) × (us_at_deg_max - us_at_deg_min)
  *                                       ÷ (deg_max - deg_min)
  *
  *            ③ 脉宽夹进舵机可接受范围 [SERVO_US_NOMINAL_MIN, SERVO_US_NOMINAL_MAX]
  *               再乘 ARM_PWM_TICKS_PER_US 得到 CCR
  *
  *          公式里刻意用 int32 做中间量、最后才转 uint16：
  *          如果直接用 uint16 算减法，us_at_deg_max < us_at_deg_min（装反的关节）
  *          会下溢成一个巨大的正数，舵机瞬间冲到底。
  ******************************************************************************
  */

#include "main.h"        /* HAL、htim2 所在的头文件 */
#include "arm_cfg.h"
#include "arm_pwm.h"

/* main.c 里定义的定时器句柄，这里只声明不定义 */
extern TIM_HandleTypeDef htim2;

/* 关节编号 → TIM2 通道。顺序必须和 g_arm_joint[] / 引脚表一致。
 *
 * TIM2 走**全重映射**（main.c 的 MX_GPIO_Init() 里写
 * __HAL_AFIO_REMAP_TIM2_ENABLE()），通道号不变，只是出脚变了：
 *
 *     J1 TIM2_CH1 → PA15      J2 TIM2_CH2 → PB3
 *     J3 TIM2_CH3 → PB10      J4 TIM2_CH4 → PB11
 *
 * PA0/PA1 是按键和 LED，不能用；改脚必须同步改这里和 arm_cfg.c 的对照表。 */
static const uint32_t s_channel[ARM_JOINT_NUM] =
{
  TIM_CHANNEL_1,   /* J1 → PA15（全重映射） */
  TIM_CHANNEL_2,   /* J2 → PB3  */
  TIM_CHANNEL_3,   /* J3 → PB10 */
  TIM_CHANNEL_4,   /* J4 → PB11 */
};

/* ------------------------------------------------------------------------- */
/* 内部：把 CCR 直接写进比较寄存器                                            */
/* ------------------------------------------------------------------------- */
static void arm_write_ccr(uint8_t joint, uint16_t ccr)
{
  __HAL_TIM_SET_COMPARE(&htim2, s_channel[joint], ccr);
}

/* ------------------------------------------------------------------------- */
/* 上电第一拍该给多少 µs                                                      */
/* ------------------------------------------------------------------------- */

/* ★ 为什么速度型关节不能按 deg_home 反算：
 *
 *   J1 是 360° 连续旋转舵机，没有“开环角度”这回事 —— `arm_deg_to_us()`
 *   只是拿标定表对 (deg_min..deg_max) 做线性插值。J1 的 deg_home = 0
 *   落在区间中间，插值出来是一个**非零转速**：上电第一拍就会转，
 *   等 5ms 后的 vel_tick() 才抓得住。
 *
 *   速度型关节的正确上电状态是“停转”：给 us_stop，剩下的交给闭环去追
 *   deg_home。5ms 后 vel_tick() 会把目标拉到 deg_home，所以“上电自动回零”
 *   这个行为没有变，变的只是第一拍不再窜出去。 */
static uint16_t power_on_us(uint8_t j)
{
  if (g_arm_joint[j].kind == ARM_JOINT_VEL)
  {
    return (g_arm_joint[j].us_stop != 0u) ? g_arm_joint[j].us_stop : SERVO_US_HOME;
  }

  return arm_deg_to_us(j, (float)g_arm_joint[j].deg_home);
}

/* ------------------------------------------------------------------------- */
/* 初始化：四路先给各自的上电脉宽，再启动 PWM                                 */
/* ------------------------------------------------------------------------- */
void arm_init(void)
{
  uint8_t j;

  /* 第一步：先把比较值写好。必须在 HAL_TIM_PWM_Start() **之前**，
   * 否则通道一启动就会按寄存器里的旧值（可能是 0）输出一小段，
   * 舵机会抖一下甚至冲向极限。
   *
   * 为什么不能四路统一写 1500：每个关节的零位不一定是 1500µs ——
   * J4 夹爪零位是 1480µs（见 arm_cfg.c）。统一写 1500 的话，
   * 上电瞬间夹爪先被拉到 ≈84°，再弹回 60°，肉眼就是“上电抽一下”，
   * 带载还会顶一下工件。
   *
   * 所以按 power_on_us() 逐关节给：位置型按 deg_home 反算，第一拍就是
   * 正确位置；速度型给 us_stop，第一拍不转（理由见 power_on_us()）。 */
  for (j = 0; j < ARM_JOINT_NUM; j++)
  {
    arm_write_ccr(j/*#语法糖标签#*/, ARM_PWM_US_TO_CCR(power_on_us(j)));
  }

  /* 第二步：启动四个通道 */
  for (j = 0; j < ARM_JOINT_NUM; j++)
  {
    (void)HAL_TIM_PWM_Start(&htim2, s_channel[j]);
  }

  /* 第三步：按各自的上电脉宽再走一遍
   * （位置型 = deg_home 反算；速度型 = us_stop，理由见 power_on_us()） */
  for (j = 0; j < ARM_JOINT_NUM; j++)
  {
    arm_set_us(j, power_on_us(j));
  }
}

/* ------------------------------------------------------------------------- */
/* 逻辑角度 → 脉宽（µs）                                                      */
/* ------------------------------------------------------------------------- */
uint16_t arm_deg_to_us(uint8_t joint, float deg)
{
  if (joint >= ARM_JOINT_NUM)
  {
    return SERVO_US_HOME;          /* 编号非法：给个中位，别输出奇怪的脉宽 */
  }

  const arm_joint_cfg_t *c = &g_arm_joint[joint];

  /* ① 角度先夹进该关节允许范围 */
  if (deg < (float)c->deg_min) { deg = (float)c->deg_min; }
  if (deg > (float)c->deg_max) { deg = (float)c->deg_max; }

  const int32_t span_deg = (int32_t)c->deg_max - (int32_t)c->deg_min;
  const int32_t span_us  = (int32_t)c->us_at_deg_max - (int32_t)c->us_at_deg_min;

  int32_t us;

  /* ② 线性插值。span_deg == 0 说明标定表写错了，退回下限值，不做除零 */
  if (span_deg == 0)
  {
    us = (int32_t)c->us_at_deg_min;
  }
  else
  {
    us = (int32_t)c->us_at_deg_min
       + (int32_t)(((float)span_us / (float)span_deg) * (deg - (float)c->deg_min));
  }

  /* ③ 夹进舵机可接受范围。正常情况下这一步是空操作：
   *    角度已经夹在 [deg_min, deg_max] 内，插值结果必然落在标定表两端点之间。
   *    留着它只防一种情况：标定表里填了 3000 这种超出舵机手册的值。 */
  if (us < (int32_t)SERVO_US_NOMINAL_MIN) { us = (int32_t)SERVO_US_NOMINAL_MIN; }
  if (us > (int32_t)SERVO_US_NOMINAL_MAX) { us = (int32_t)SERVO_US_NOMINAL_MAX; }

  return (uint16_t)us;
}

/* ------------------------------------------------------------------------- */
/* 逻辑角度 → CCR                                                             */
/* ------------------------------------------------------------------------- */
uint16_t arm_deg_to_ccr(uint8_t joint, float deg)
{
  return ARM_PWM_US_TO_CCR(arm_deg_to_us(joint, deg));
}

/* ------------------------------------------------------------------------- */
/* 设置关节角度                                                               */
/* ------------------------------------------------------------------------- */
void arm_set_deg(uint8_t joint, float deg)
{
  if (joint >= ARM_JOINT_NUM)
  {
    return;
  }
  arm_write_ccr(joint, arm_deg_to_ccr(joint, deg));
}

/* ------------------------------------------------------------------------- */
/* 直接按脉宽设置（标定时用，跳过角度换算）                                    */
/* ------------------------------------------------------------------------- */
void arm_set_us(uint8_t joint, uint16_t us)
{
  if (joint >= ARM_JOINT_NUM)
  {
    return;
  }
  /* 同样夹舵机可接受范围，乱填也不会烧舵机（但挡不住"顶到机械限位"，
   * 那要靠标定表把端点留在机械限位内侧 40~50µs） */
  if (us < SERVO_US_NOMINAL_MIN) { us = SERVO_US_NOMINAL_MIN; }
  if (us > SERVO_US_NOMINAL_MAX) { us = SERVO_US_NOMINAL_MAX; }

  arm_write_ccr(joint, ARM_PWM_US_TO_CCR(us));
}

/* ------------------------------------------------------------------------- */
/* 读回当前角度（由 CCR 反算，调试/回帧用）                                    */
/* ------------------------------------------------------------------------- */
float arm_get_deg(uint8_t joint)
{
  if (joint >= ARM_JOINT_NUM)
  {
    return 0.0f;
  }

  const arm_joint_cfg_t *c = &g_arm_joint[joint];

  const uint16_t ccr = (uint16_t)__HAL_TIM_GET_COMPARE(&htim2, s_channel[joint]);
  const int32_t  us  = (int32_t)ARM_PWM_CCR_TO_US(ccr);

  const int32_t span_us  = (int32_t)c->us_at_deg_max - (int32_t)c->us_at_deg_min;
  const int32_t span_deg = (int32_t)c->deg_max - (int32_t)c->deg_min;

  if (span_us == 0)
  {
    return (float)c->deg_min;
  }

  float deg = (float)c->deg_min
            + ((float)(us - (int32_t)c->us_at_deg_min) * (float)span_deg / (float)span_us);

  if (deg < (float)c->deg_min) { deg = (float)c->deg_min; }
  if (deg > (float)c->deg_max) { deg = (float)c->deg_max; }

  return deg;
}

/* ==========================================================================
 * 调试接口（对应上位机 0x31 / 0x32 调试帧）
 * ========================================================================== */

/* ------------------------------------------------------------------------- */
/* 读回当前脉宽（µs）                                                         */
/* ------------------------------------------------------------------------- */
uint16_t arm_get_us(uint8_t joint)
{
  if (joint >= ARM_JOINT_NUM)
  {
    return 0u;
  }

  /* 直接读比较寄存器再换算回 µs —— 读的是芯片里的真实值，
   * 不是软件缓存的"我以为写进去的值"。这样上位机看到的就是硬件现状。 */
  const uint16_t ccr = (uint16_t)__HAL_TIM_GET_COMPARE(&htim2, s_channel[joint]);

  return ARM_PWM_CCR_TO_US(ccr);
}

/* ------------------------------------------------------------------------- */
/* 开/关单路 PWM 输出                                                         */
/* ------------------------------------------------------------------------- */
void arm_output_enable(uint8_t joint, uint8_t enable)
{
  if (joint >= ARM_JOINT_NUM)
  {
    return;
  }

  if (enable != 0u)
  {
    (void)HAL_TIM_PWM_Start(&htim2, s_channel[joint]);
  }
  else
  {
    (void)HAL_TIM_PWM_Stop(&htim2, s_channel[joint]);
  }
}
