/**
  ******************************************************************************
  * @file    pid_selftest.c
  * @brief   PID 模块 PC 端单元测试（在电脑上跑，不需要 STM32 / ST-Link）
  ******************************************************************************
  * @note    编译运行（gcc 用 MinGW 那个，本机没有 Linux 环境）：
  *
  *            gcc -std=c11 -Wall -Wextra -Wpedantic -O2 -ffp-contract=off ^
  *                -I Core/Inc tests/pid_selftest.c Core/Src/arm_pid.c ^
  *                -o tests/pid_selftest.exe
  *            tests\pid_selftest.exe
  *
  *          退出码 0 = 全部通过，1 = 有失败。
  *
  *          -ffp-contract=off：禁止编译器把 `a + b*c` 融合成 fma。
  *          第 1 组测的是"与旧代码**逐位**相同"，一旦有一边被融合、
  *          另一边没融合，末尾几位就会差 1 ulp，那条断言会莫名其妙地挂。
  *          （Cortex-M3 没有硬件 FPU，固件那边全是软件浮点，本来就不会融合；
  *            这个开关是给 PC 上的 x86 编译器加的保险。）
  *
  * @note    第 1 组是**硬指标**（plan.md §11.4）：
  *          ki = kd = 0 时，PID 的输出必须与 2026-11-13 之前 arm_joint.c
  *          里那段纯 P 代码逐位相同 —— 这不是风格问题，而是"加了模块、
  *          固件行为一个 bit 都不许变"的回归证据。
  ******************************************************************************
  */

#include <stdio.h>
#include <string.h>

#include "arm_pid.h"

static int g_pass = 0;
static int g_fail = 0;

static void check(int cond, const char *name)
{
  if (cond)
  {
    g_pass++;
    printf("  [PASS] %s\n", name);
  }
  else
  {
    g_fail++;
    printf("  [FAIL] %s\n", name);
  }
}

/* 浮点按位比较：把 float 当 4 个字节看，一位都不能差
 * （不用 union / 不直接强转指针，避免 strict-aliasing 和 -Wpedantic 的抱怨） */
static uint32_t u32_of(float f)
{
  uint32_t x;

  memcpy(&x, &f, sizeof x);

  return x;
}

static int bits_equal(float a, float b)
{
  return (u32_of(a) == u32_of(b)) ? 1 : 0;
}

/* ==========================================================================
 * 被测参数的"出厂值"—— 与 arm_cfg.h §五 的当前值一致
 * ========================================================================== */
#define T_KP      3.0f      /* ARM_BASE_KP_US_PER_DEG  */
#define T_UMAX    300.0f    /* ARM_BASE_U_MAX_US       */
#define T_DB      1.5f      /* ARM_BASE_DEADBAND_DEG   */
#define T_DT      0.005f    /* 200Hz                   */
#define T_DMAX  1000.0f     /* ARM_BASE_D_MAX_DPS      */

/* --------------------------------------------------------------------------
 * 参考实现：**照抄** 2026-11-13 之前 arm_joint.c 的 vel_tick() 里那几行
 *
 *     u = u_ff + (ARM_BASE_KP_US_PER_DEG * err);
 *     if (u >  U_MAX) u =  U_MAX;
 *     if (u < -U_MAX) u = -U_MAX;
 *     if (|err| < DEADBAND) u = 0;
 *
 * 一个字都别"顺手优化"—— 它的价值就在于它是**旧代码本身**。
 * -------------------------------------------------------------------------- */
static float ref_legacy_p(float err, float ff)
{
  float u;
  float ae = (err >= 0.0f) ? err : -err;   /* 旧代码就是这么算绝对值的 */

  u = ff + (T_KP * err);

  if (u > T_UMAX)  { u = T_UMAX;  }
  if (u < -T_UMAX) { u = -T_UMAX; }

  if (ae < T_DB)
  {
    u = 0.0f;
  }

  return u;
}

/* 把 PID 实例配成"与旧代码等价"的样子：只有 P + 限幅 + 死区 */
static void setup_legacy(arm_pid_t *p)
{
  p->kp       = T_KP;
  p->ki       = 0.0f;
  p->kd       = 0.0f;
  p->dt_s     = T_DT;
  p->out_max  = T_UMAX;
  p->i_max    = 0.0f;
  p->deadband = T_DB;
  p->d_tau_s  = 0.0f;
  p->d_max    = T_DMAX;
  arm_pid_reset(p);
}

/* 通用配置 */
static void setup(arm_pid_t *p, float kp, float ki, float kd,
                  float out_max, float i_max, float deadband, float tau)
{
  p->kp       = kp;
  p->ki       = ki;
  p->kd       = kd;
  p->dt_s     = T_DT;
  p->out_max  = out_max;
  p->i_max    = i_max;
  p->deadband = deadband;
  p->d_tau_s  = tau;
  p->d_max    = T_DMAX;
  arm_pid_reset(p);
}

/* ==========================================================================
 * 1. 逐位回归：ki = kd = 0 时必须与旧纯 P 代码逐位相同
 * ========================================================================== */
static void test_bit_identical(void)
{
  static const float ff_set[] = { 0.0f, 12.5f, -12.5f, 299.75f, -300.0f, 1000.0f };
  arm_pid_t p;
  int       mism = 0;
  int       n    = 0;
  int       i;
  int       k;
  float     err;
  float     u_pid;
  float     u_ref;
  uint16_t  us_pid;
  uint16_t  us_ref;

  setup_legacy(&p);

  /* 误差从 -602.8° 扫到 +602.8°，步长 0.137（故意不是整数，
   * 保证扫到一大堆无法精确表示的浮点数 —— 逐位比较才有意义） */
  for (i = -4400; i <= 4400; i++)
  {
    err = (float)i * 0.137f;

    for (k = 0; k < (int)(sizeof ff_set / sizeof ff_set[0]); k++)
    {
      u_pid = arm_pid_step(&p, err, 0.0f, ff_set[k]);
      u_ref = ref_legacy_p(err, ff_set[k]);
      n++;

      if (!bits_equal(u_pid, u_ref))
      {
        if (mism == 0)
        {
          printf("         首个不一致: err=%.6f ff=%.3f pid=%.9g(0x%08lX) ref=%.9g(0x%08lX)\n",
                 (double)err, (double)ff_set[k], (double)u_pid,
                 (unsigned long)u32_of(u_pid),
                 (double)u_ref, (unsigned long)u32_of(u_ref));
        }
        mism++;
      }

      /* 连"写进 CCR 之前那一步"也比：us = (uint16_t)(us_stop + u) */
      us_pid = (uint16_t)(1500.0f + u_pid);
      us_ref = (uint16_t)(1500.0f + u_ref);
      if (us_pid != us_ref)
      {
        mism++;
      }
    }
  }

  printf("  （扫描 %d 组，逐位比较）\n", n);
  check(mism == 0, "ki=kd=0 时输出与旧纯 P 代码逐位相同（含 1500+u 转 uint16）");

  /* meas 不该影响纯 P 的结果 */
  {
    arm_pid_t a;
    arm_pid_t b;
    int       same = 1;

    setup_legacy(&a);
    setup_legacy(&b);

    for (i = 0; i < 200; i++)
    {
      err = (float)i - 100.0f;
      if (!bits_equal(arm_pid_step(&a, err, 0.0f, 0.0f),
                      arm_pid_step(&b, err, 123.456f, 0.0f)))
      {
        same = 0;
        break;
      }
    }
    check(same == 1, "ki=kd=0 时 meas 不参与运算（换个数结果不变）");
  }
}

/* ==========================================================================
 * 2. 死区必须清积分（plan.md §11.4「死区和积分的死结」）
 * ========================================================================== */
static void test_deadband_clears_integral(void)
{
  arm_pid_t p;
  arm_pid_t fresh;
  float     u;
  int       i;

  setup(&p, T_KP, 2.0f, 0.0f, T_UMAX, 100.0f, T_DB, 0.0f);

  /* 先在死区外用大误差喂 1500 拍，把积分喂到限幅（每拍 +10×0.005 = 0.05，
   * 限幅 = i_max/ki = 50，所以要 1000 拍以上） */
  for (i = 0; i < 1500; i++)
  {
    (void)arm_pid_step(&p, 10.0f, 0.0f, 0.0f);
  }

  check((p.integ > 1.0f) && (bits_equal(p.ki * p.integ, p.i_max)),
        "死区外 1500 拍后积分已经喂到 i_max");

  /* 走进死区一次（误差 0.5° < 1.5°） */
  u = arm_pid_step(&p, 0.5f, 0.0f, 0.0f);
  check(bits_equal(u, 0.0f), "死区内输出为 0（含前馈）");
  check(bits_equal(p.integ, 0.0f), "★ 死区内积分被清零（不是只在输出端清 u）");

  /* 出了死区，必须和"全新控制器"给出同一个数 —— 证明没有把旧账带出来 */
  setup(&fresh, T_KP, 2.0f, 0.0f, T_UMAX, 100.0f, T_DB, 0.0f);
  check(bits_equal(arm_pid_step(&p, 10.0f, 0.0f, 0.0f),
                   arm_pid_step(&fresh, 10.0f, 0.0f, 0.0f)),
        "★ 走出死区第一拍与全新控制器逐位相同（没有攒下的旧积分）");

  /* 死区边界：|e| = 1.5 不算进死区（判据是严格小于） */
  {
    arm_pid_t q;
    setup(&q, T_KP, 0.0f, 0.0f, T_UMAX, 0.0f, T_DB, 0.0f);
    check(bits_equal(arm_pid_step(&q, T_DB, 0.0f, 0.0f), T_KP * T_DB),
          "死区判据是 |e| < DB，正好等于 DB 时不进死区");
  }
}

/* ==========================================================================
 * 3. 积分单独限幅：|Ki·∫| 永远不超过 i_max
 * ========================================================================== */
static void test_integral_clamp(void)
{
  arm_pid_t p;
  float     u = 0.0f;
  int       i;

  setup(&p, T_KP, 1.0f, 0.0f, 1000.0f, 50.0f, 0.0f, 0.0f);

  for (i = 0; i < 2000; i++)
  {
    u = arm_pid_step(&p, 100.0f, 0.0f, 0.0f);   /* 误差一直同号，最坏的积分场景 */
  }

  check(bits_equal(p.ki * p.integ, 50.0f), "积分项被 i_max 卡住且正好等于 i_max");
  check(bits_equal(u, (T_KP * 100.0f) + 50.0f), "输出 = P + I(clamp)，逐位对得上");
  check(u < 1000.0f, "本例没有触发输出限幅（证明是积分限幅在起作用）");
}

/* ==========================================================================
 * 4. 抗饱和冻结：输出顶死时不再攒积分
 * ========================================================================== */
static void test_anti_windup(void)
{
  arm_pid_t p;
  float     u;
  int       i;

  setup(&p, T_KP, 1.0f, 0.0f, T_UMAX, 0.0f, 0.0f, 0.0f);   /* i_max = 0 → 不限 */

  for (i = 0; i < 1000; i++)
  {
    (void)arm_pid_step(&p, 500.0f, 0.0f, 0.0f);  /* P 项就有 1500µs，必顶死 */
  }

  check(bits_equal(p.integ, 0.0f), "★ 输出顶死 1000 拍，积分纹丝不动（冻结生效）");

  /* 误差一反向，输出应立刻离开限幅，而不是被积分顶着继续满幅 */
  u = arm_pid_step(&p, -10.0f, 0.0f, 0.0f);
  check((u > -30.1f) && (u < -29.9f), "误差反向后输出立刻回到 ≈-30µs（无积分拖尾）");
  check(arm_pid_saturated(&p) == 0u, "此时饱和标志已清零");

  /* 反向时积分该正常累加（它在帮忙刹车，不能冻） */
  check(p.integ < 0.0f, "误差反向时积分恢复累加（方向为负）");
}

/* ==========================================================================
 * 5. D 项对「测量」微分：目标跳变不产生 setpoint kick
 * ========================================================================== */
static void test_derivative_on_measurement(void)
{
  arm_pid_t p_kd;
  arm_pid_t p_p;
  float     u1;
  float     u2;
  int       i;

  setup(&p_kd, T_KP, 0.0f, 5.0f, 1000.0f, 0.0f, 0.0f, 0.05f);
  setup(&p_p,  T_KP, 0.0f, 0.0f, 1000.0f, 0.0f, 0.0f, 0.0f);

  /* 先跑几拍让 D 有历史，测量值一直不动 */
  for (i = 0; i < 5; i++)
  {
    (void)arm_pid_step(&p_kd, 0.0f, 90.0f, 0.0f);
    (void)arm_pid_step(&p_p,  0.0f, 90.0f, 0.0f);
  }

  /* 目标从 90° 瞬跳到 180°：误差从 0 变 90，但**测量没动** */
  u1 = arm_pid_step(&p_kd, 90.0f, 90.0f, 0.0f);
  u2 = arm_pid_step(&p_p,  90.0f, 90.0f, 0.0f);

  check(bits_equal(u1, u2), "★ 目标瞬跳 90° 时，带 Kd 与不带 Kd 输出完全相同");
  check(bits_equal(u1, 270.0f), "输出就是纯 P 的 3×90（D 一点没踢）");
}

/* ==========================================================================
 * 6. D 项低通：把 AS5600 的 1 LSB 量化抖动压下去
 * ========================================================================== */
static void test_derivative_lowpass(void)
{
  arm_pid_t p_raw;
  arm_pid_t p_lpf;
  float     meas;
  float     raw_max = 0.0f;
  float     lpf_max = 0.0f;
  float     d;
  int       i;

  setup(&p_raw, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f);    /* tau = 0，全通 */
  setup(&p_lpf, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.05f);  /* tau = 50ms */

  for (i = 0; i < 200; i++)
  {
    /* 模拟"读数在两格之间来回跳"：±1 LSB = ±0.0879° */
    meas = ((i & 1) != 0) ? 0.0879f : 0.0f;

    (void)arm_pid_step(&p_raw, 0.0f, meas, 0.0f);
    (void)arm_pid_step(&p_lpf, 0.0f, meas, 0.0f);

    d = (p_raw.d_state >= 0.0f) ? p_raw.d_state : -p_raw.d_state;
    if (d > raw_max) { raw_max = d; }
    d = (p_lpf.d_state >= 0.0f) ? p_lpf.d_state : -p_lpf.d_state;
    if (d > lpf_max) { lpf_max = d; }
  }

  printf("        1 LSB@200Hz：不滤波 %.2f °/s，tau=50ms 后 %.2f °/s\n",
         (double)raw_max, (double)lpf_max);

  check(raw_max > 17.0f && raw_max < 18.0f, "不滤波时 1 LSB 被放大成 ≈17.6 °/s（理论 0.0879/0.005）");
  check(lpf_max < raw_max * 0.25f, "★ 50ms 低通把量化噪声压到 1/4 以下");
  check(lpf_max > 0.0f, "低通只是压小，不是把微分整个抹成 0");
}

/* ==========================================================================
 * 7. 复位：清状态不清参数
 * ========================================================================== */
static void test_reset(void)
{
  arm_pid_t p;
  arm_pid_t fresh;
  int       i;

  setup(&p, T_KP, 2.0f, 3.0f, T_UMAX, 100.0f, 0.0f, 0.05f);

  for (i = 0; i < 50; i++)
  {
    (void)arm_pid_step(&p, 20.0f, (float)i, 0.0f);
  }

  arm_pid_reset(&p);
  check(bits_equal(p.integ, 0.0f) && bits_equal(p.d_state, 0.0f) &&
        (p.started == 0u) && (p.sat == 0u), "arm_pid_reset 清干净了状态");
  check(bits_equal(p.kp, T_KP) && bits_equal(p.ki, 2.0f) && bits_equal(p.kd, 3.0f),
        "arm_pid_reset 不动参数（kp/ki/kd 原样保留）");

  setup(&fresh, T_KP, 2.0f, 3.0f, T_UMAX, 100.0f, 0.0f, 0.05f);
  check(bits_equal(arm_pid_step(&p, 20.0f, 0.0f, 0.0f),
                   arm_pid_step(&fresh, 20.0f, 0.0f, 0.0f)),
        "复位后第一拍与全新实例逐位相同");

  /* 空指针 */
  check(bits_equal(arm_pid_step(0, 10.0f, 0.0f, 0.0f), 0.0f), "p == 0 时返回 0 而不是崩");
  arm_pid_reset(0);                    /* 能安全返回就够了，没有返回值可断言 */
  check(1, "arm_pid_reset(0) 安全返回");
  check(arm_pid_saturated(0) == 0u, "arm_pid_saturated(0) 返回 0");
}

/* ==========================================================================
 * 8. 限幅开关：out_max <= 0 表示不限幅
 * ========================================================================== */
static void test_out_max_switch(void)
{
  arm_pid_t p;

  setup(&p, 100.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
  check(bits_equal(arm_pid_step(&p, 45.0f, 0.0f, 0.0f), 4500.0f),
        "out_max = 0 表示不限幅（4500µs 原样输出）");
  check(arm_pid_saturated(&p) == 0u, "不限幅时 sat 恒为 0");

  setup(&p, 100.0f, 0.0f, 0.0f, T_UMAX, 0.0f, 0.0f, 0.0f);
  check(bits_equal(arm_pid_step(&p, 45.0f, 0.0f, 0.0f), T_UMAX),
        "限幅生效时输出正好等于 out_max");
  check(arm_pid_saturated(&p) == 1u, "限幅生效时 sat = 1");
}

/* ==========================================================================
 * 9. 追斜坡仿真：为什么"光有 P"不够（对应 arm_joint.c 头部那段注释）
 *
 *    被控对象（连续舵机的极简模型）：转速 = 1.0 × 输出(µs)，角度 = ∫转速dt
 *    也就是 g = 1 (°/s) / µs —— 正是 arm_joint.c 里"Kp = 3 追 180°/s
 *    会稳定落后 60°"那个假设。
 *
 *    纯 P  ：稳态滞后 = v / (g·Kp) = 180 / 3 = 60°
 *    前馈+P：前馈直接补上"匀速该出多大力"，滞后 → 0
 *    P+I   ：积分自己把 60° 的账还掉，滞后 → 0
 * ========================================================================== */
#define SIM_G     1.0f       /* (°/s) per µs  */
#define SIM_V     180.0f     /* 目标速度 °/s  */
#define SIM_STEPS 20000      /* 100 秒 @200Hz（P+I 的慢极点时间常数约 5.6s，留足 17τ） */
#define SIM_DT    0.005f

static float sim_lag(arm_pid_t *p, uint8_t use_ff)
{
  /* 被控对象的**积分**用 double 记，只把误差/测量喂给 PID 时转成 float。
   *
   * 为什么不用 float 记：跑 100 秒后角度已经到 18000°，float32 在这个量级
   * 上一个 ulp 就是 0.002°，一拍一拍加 20000 次会累积出接近 1° 的**记账误差** ——
   * 那是测试脚本自己的数值问题，会冒充成"控制器有稳态误差"，把人带沟里。
   * （这条本身就是个好例子：定点/浮点累加的误差和量级有关，别拿它当算法问题。） */
  double r     = 0.0;        /* 目标角 */
  double theta = 0.0;        /* 实际角 */
  double omega;
  double lag   = 0.0;        /* 最后一拍控制器**看到**的误差 */
  float  ff    = 0.0f;
  float  u;
  int    i;

  if (use_ff != 0u)
  {
    ff = SIM_V * 1.0f;       /* us_per_dps = 1.0 µs/(°/s) */
  }

  for (i = 0; i < SIM_STEPS; i++)
  {
    r    += (double)SIM_V * (double)SIM_DT;
    lag   = r - theta;                       /* 采样时刻的误差：目标已推进、测量还没动 */
    u     = arm_pid_step(p, (float)lag, (float)theta, ff);
    omega = (double)SIM_G * (double)u;
    theta = theta + (omega * (double)SIM_DT);
  }

  /* ★ 返回的是**循环里最后一拍算出来的那个误差**，不是 `r - theta`。
   *
   *   两者差整整一拍的行程（180°/s × 5ms = 0.9°），因为循环里 r 先推进、
   *   theta 后推进：循环结束时 theta 比 r 多积分了一拍。
   *   第一版就是这么写的，结果三种方案全部偏 −0.9°，差点被当成控制器 bug ——
   *   记在这儿，免得下次又去怀疑 PID。 */
  return (float)lag;
}

static void test_ramp_tracking(void)
{
  arm_pid_t p;
  float     lag_p;
  float     lag_ff;
  float     lag_i;

  /* ① 纯 P */
  setup(&p, T_KP, 0.0f, 0.0f, T_UMAX, 0.0f, 0.0f, 0.0f);
  lag_p = sim_lag(&p, 0u);

  /* ② 前馈 + P */
  setup(&p, T_KP, 0.0f, 0.0f, T_UMAX, 0.0f, 0.0f, 0.0f);
  lag_ff = sim_lag(&p, 1u);

  /* ③ P + I */
  setup(&p, T_KP, 0.5f, 0.0f, T_UMAX, 300.0f, 0.0f, 0.0f);
  lag_i = sim_lag(&p, 0u);

  printf("        追 180°/s 的稳态滞后：纯P %.3f°，前馈+P %.3f°，P+I %.3f°\n",
         (double)lag_p, (double)lag_ff, (double)lag_i);

  check((lag_p > 58.0f) && (lag_p < 62.0f),
        "纯 P 追 180°/s 稳定落后 ≈60°（理论 v/(g·Kp) = 60）");
  check((lag_ff > -0.5f) && (lag_ff < 0.5f),
        "前馈 + P 把稳态滞后消到 0 附近（这就是 vel_tick() 里 u_ff 的作用）");
  check((lag_i > -0.5f) && (lag_i < 0.5f),
        "P + I 也把稳态滞后消到 0 附近（积分还的是同一笔账）");
}

/* ==========================================================================
 * 10. 微分合理性上限（d_max）：测量值跨 ±180 接缝跳 360° 时必须丢掉这一拍
 *
 *     场景就是底座真会发生的事：手臂从 +180 走到 -179.5（跨接缝），
 *     meas 在**一拍之内**跳了 -359.5°，除以 5ms = +71900°/s。
 *     没有这道闸，低通只把它削到 9%，之后要 255ms 才衰减完 ——
 *     这 255ms 输出一直顶在限幅上（闭环仿真实测冲过目标 3.07°）。
 *
 *     注意符号：d_raw = -(meas - prev)/dt = **负的测量速度**。
 *     所以手臂往负方向走（179.5 → 179.0）时 d_raw 是 +100，不是 -100。
 * ========================================================================== */
static void test_derivative_jump_guard(void)
{
  arm_pid_t p;
  float u;
  float a;

  /* 只留 D（kp = ki = 0，out_max = 0 不限幅），这样 u 就是 d_term 本身 */
  setup(&p, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f);   /* d_tau_s = 0，直通 */
  p.d_max = 1000.0f;

  (void)arm_pid_step(&p, 0.0f, 179.5f, 0.0f);            /* 第 1 拍只记 prev_meas */

  u = arm_pid_step(&p, 0.0f, 179.0f, 0.0f);              /* 走 0.5°/拍 = 100°/s */
  check(bits_equal(u, 100.0f), "正常运动 0.5°/拍（100°/s）不被拦");

  u = arm_pid_step(&p, 0.0f, 180.0f, 0.0f);              /* 反向 1°/拍 = 200°/s */
  check(bits_equal(u, -200.0f), "正常运动 1°/拍（200°/s）不被拦");

  /* ★ 接缝：+180 → -179.5，一拍跳 -359.5° ⇒ +71900°/s */
  u = arm_pid_step(&p, 0.0f, -179.5f, 0.0f);
  check(bits_equal(u, 0.0f), "跨接缝这一拍被丢掉：d_term = 0（不是 71900）");
  check(bits_equal(p.d_state, 0.0f), "丢的同时清掉 d_state，假速度不会拖尾");

  /* 下一拍照常工作 —— 证明只丢一拍，不是把 D 永久关掉 */
  u = arm_pid_step(&p, 0.0f, -179.0f, 0.0f);
  check(bits_equal(u, -100.0f), "接缝之后第一拍就恢复正常（-100°/s）");

  /* d_max = 0 表示不检查：同样的跳变原样送出去（改动前的行为，留作对照） */
  setup(&p, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f);
  p.d_max = 0.0f;
  (void)arm_pid_step(&p, 0.0f, 179.5f, 0.0f);
  u = arm_pid_step(&p, 0.0f, -179.5f, 0.0f);
  a = (u < 0.0f) ? -u : u;
  check((a > 70000.0f) && (a < 74000.0f), "d_max = 0 时不检查（71800 原样输出，对照用）");

  /* 滤波器开着也拦得住 —— 说明检查发生在低通**之前** */
  setup(&p, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.05f);
  p.d_max = 1000.0f;
  (void)arm_pid_step(&p, 0.0f, 179.5f, 0.0f);
  u = arm_pid_step(&p, 0.0f, -179.5f, 0.0f);
  check(bits_equal(u, 0.0f), "d_tau_s = 0.05 时同样在低通之前拦下");
  check(bits_equal(p.d_state, 0.0f), "滤波状态下 d_state 也停在 0，没有 6545 的拖尾");
}

/* ========================================================================== */
int main(void)
{
  printf("=== arm_pid selftest ===\n\n");

  printf("[1] 逐位回归：ki = kd = 0 ≡ 旧纯 P 代码\n");
  test_bit_identical();

  printf("\n[2] 死区与积分\n");
  test_deadband_clears_integral();

  printf("\n[3] 积分限幅\n");
  test_integral_clamp();

  printf("\n[4] 抗饱和（冻结法）\n");
  test_anti_windup();

  printf("\n[5] D 项对测量微分（无 setpoint kick）\n");
  test_derivative_on_measurement();

  printf("\n[6] D 项低通抑制量化噪声\n");
  test_derivative_lowpass();

  printf("\n[7] 复位 / 空指针\n");
  test_reset();

  printf("\n[8] 输出限幅开关\n");
  test_out_max_switch();

  printf("\n[9] 追斜坡仿真（P / 前馈+P / P+I）\n");
  test_ramp_tracking();

  printf("\n[10] 微分合理性上限（接缝 360° 跳变）\n");
  test_derivative_jump_guard();

  printf("\n=== %d passed, %d failed ===\n", g_pass, g_fail);

  return (g_fail == 0) ? 0 : 1;
}
