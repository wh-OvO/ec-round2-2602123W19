/**
  ******************************************************************************
  * @file    arm_joint.c
  * @brief   关节执行层：把"底座闭环"和"其余三路直通"藏在同一个接口后面
  ******************************************************************************
  * @note    **本文件只干两件事**
  *
  *          1) 位置型关节（J2/J3/J4）：转手交给 arm_pwm.c，一行逻辑都不加。
  *          2) 速度型关节（J1 底座）：跑位置闭环，输出的是**转速**而不是角度。
  *
  *          上层（插补器 / 协议层）永远只说"我要到多少度"，不关心底下怎么实现。
  *
  * @note    **底座闭环的控制律**
  *
  *              脉宽 = us_stop + 前馈 + Kp × 误差 + Ki·∫误差 − Kd·d(测量)/dt
  *                     └──────────────────┬──────────────────┘
  *                                    arm_pid.c 负责
  *
  *            误差 = wrap180(目标角 − 实测角)     ← 必须绕到 ±180，否则 359° 和 0°
  *                                                  之间会被算成"差 359 度"
  *
  *          为什么必须有前馈：插补器每 5ms 把目标往前推一点，底座在追一个
  *          **斜坡**。纯比例控制追斜坡必然有稳态误差 e = ω / Kp ——
  *          底座 180°/s、Kp = 3 时，运动中会稳定落后 60°，肉眼可见。
  *          前馈把"跟上这个速度该出多大力"直接算好，Kp 只负责收尾。
  *          （这 60° 不是嘴上说的：tests/pid_selftest.c 第 9 组仿真出来的
  *           就是 60.000°，加了前馈之后是 0.000°。）
  *
  *          P/I/D 三项由 arm_pid.c 算，本文件只负责
  *          "读角度 → 算误差 → 限位判断 → 给前馈 → 写脉宽"。
  *
  * @note    **为什么读不到角度就停转，而不是"保持上一次的脉宽"**
  *
  *          上一次的脉宽是个**非零转速**的指令。失去了角度反馈还继续发这个指令，
  *          底座就会一直转，转到线被拧断或者撞上什么东西为止。
  *          停下来是唯一安全的选择，同时点亮故障位让上位机看到。
  ******************************************************************************
  */

#include "arm_cfg.h"
#include "arm_pwm.h"
#include "arm_joint.h"
#include "arm_pid.h"
#include "as5600.h"

/* ------------------------------------------------------------------------- */
/* 编译期护栏：PID 的积分/微分都按 dt 算，这里的 dt 是从插补节拍推出来的。       */
/* 哪天有人把 ARM_INTERP_TICK_MS 从 5 改成 10 而忘了改 ARM_BASE_DT_S，         */
/* I 和 D 的数值会**悄悄**差一倍（不改编译结果、不报错，只是控制变得莫名其妙）。 */
/* 与其等到上板调不出来，不如在这里直接编不过。                                  */
/* ------------------------------------------------------------------------- */
#if (ARM_INTERP_TICK_MS != 5u)
#  error "改 ARM_INTERP_TICK_MS 必须同步改 arm_cfg.h §五 的 ARM_BASE_DT_S"
#endif

/* ---- 状态 ---- */
static float   s_tgt_deg[ARM_JOINT_NUM];   /* 每个关节的当前目标角（度） */
static float   s_ff_dps[ARM_JOINT_NUM];    /* 插补器给的目标速度（度/秒），前馈用 */
static uint8_t s_manual[ARM_JOINT_NUM];    /* 1 = 被 0x31 手动占用，闭环别抢 */
static arm_pid_t s_pid[ARM_JOINT_NUM];     /* 每个关节一个 PID 实例（只有 VEL 会真用） */

static uint8_t s_base_fault;               /* 1 = AS5600 读不到 */
static float   s_base_last_deg;            /* 最近一次**成功**读到的机械角（度），读失败时对外就报它 */
static float   s_base_err;                 /* 最近一次闭环误差的绝对值（度），给状态位用 */
static uint8_t s_read_fail_cnt;            /* 底座**连续**读失败次数，见 BASE_READ_FAIL_RESET_CNT */

/* ------------------------------------------------------------------------- */
/* 角度归一化：把一个角度绕进 (-180, +180]                                    */
/* ------------------------------------------------------------------------- */
static float wrap180(float a)
{
  while (a > 180.0f)
  {
    a -= 360.0f;
  }
  while (a <= -180.0f)
  {
    a += 360.0f;
  }
  return a;
}

/* ------------------------------------------------------------------------- */
/* 底座实测角：把 AS5600 的原始读数换算到"机械坐标系"                          */
/*                                                                            */
/* 这是**全工程唯一**该读 AS5600 的地方。别在别的文件里再直接调                */
/* as5600_read_deg()，否则同一个角度会出现两种坐标系，调起来没完没了。         */
/*                                                                            */
/* 返回值和 g_arm_joint[ARM_JOINT_BASE].deg_min..deg_max 是**同一个坐标**：    */
/*     home(0°) → 报 ≈0     下限位 → 报 ≈-179     上限位 → 报 ≈+179            */
/* 所以 vel_tick() 里可以直接算 `s_tgt_deg[joint] - meas`，不需要再加减偏置。  */
/* J1 当前是 −180 ~ +145 / home = 0。                                       */
/* ------------------------------------------------------------------------- */
/* 诊断开关：0 = 正常（走 ENC_SIGN/ENC_ZERO 标定）
 *            1 = 直通输出 AS5600 原始磁角度（0~360°）
 * 正常标定完必须改回 0。 */
#define ARM_BASE_DIAG_RAW   0

static uint8_t base_read_deg(float *deg)
{
  const arm_joint_cfg_t *c = &g_arm_joint[ARM_JOINT_BASE];
  float raw    = 0.0f;
  float center;

  if (as5600_read_deg(&raw) == 0u)
  {
    /* ★ 读失败必须**在这里**就置故障：status / move_start / 0x34 都会在
     * vel_tick 之前读角度，若等 vel_tick 再置位，它们会拿到上次的值却查不到
     * 故障，把坏读数当真的用。故障位与“这一次读成功没有”同步。 */
    s_base_fault = 1u;
    return 0u;                       /* 读失败：*deg 保持不动，调用方自己判断 */
  }

  /* 位宽补正：万一换到"真只有 8 位"的传感器，用这个把满量程临时拉回来。
   * SHIFT = 0 时这一行是 ×1，不影响正常芯片（详见 arm_cfg.h 的
   * ARM_BASE_RAW_SHIFT 注释）。 */
  raw *= (float)(1u << ARM_BASE_RAW_SHIFT);

  /* 区间中点，把 wrap180 的 (−180,+180] 平移到标定表的区间上。
   * J1 是 −180~+145 ⇒ center = **−17.5**；接缝落在 CENTER±180×SCALE
   * = −257.5/+222.5，离两端各 77.5°。 */
  center = 0.5f * ((float)c->deg_min + (float)c->deg_max);

  /* 机械角 = wrap180( SIGN × (原始角 − ZERO) ) × SCALE + CENTER
   *
   *   减 ZERO    磁铁装在哪，AS5600 的零点就在哪，跟机械角度无关
   *   乘 SIGN    磁铁装反 / 舵机反向时两者方向相反，不乘就变正反馈
   *   绕 ±180    减完可能跑到 0~360 之外，绕回来才好算误差
   *   + CENTER   ★ 把 0 从"行程中点"平移回"机械角 0"
   *
   * ★ CENTER 必须和目标坐标系一致：home 处读数若停在 0，而 s_tgt_deg[0]
   *   是别的值 → 误差恒定 → u 顶到限幅 → 底座全速转过去才停 → 撞限位。
   *   改 CENTER 就要同时改 deg_home，否则上电第一拍就是满误差。
   *
   * ★ SCALE：实测「上位机目标/实际都是 90，但手臂现实里转了 120」——
   *   编码器**少读**了：1° 读数对应 1.3333° 真实转角。
   *   所以 wrap180 之后再乘 SCALE 换成真实角，最后加 CENTER。
   *
   *   ⚠️ SCALE 必须乘在 wrap180 **之后**：折返要在"原始磁角度"空间里做，
   *      先缩放再折返的话接缝会跑到 ±180/SCALE 上去，整个标定都错位。
   *   接了 SCALE 之后接缝落在 CENTER ± 180×SCALE：
   *      −17.5 ± 240 = **−257.5 / +222.5**，离行程 [−180, +145] 两头各剩 77.5°。
   */
#if ARM_BASE_DIAG_RAW
  /* ===== 临时诊断 =====
   * 直通输出 AS5600 的**原始磁角度**，上位机「实际」直接显示 0~360°。
   *   磁铁转一圈 → 读数扫过 0~360  → 传感器好的，问题在坐标系标定
   *               → 读数扫不动     → 传感器 / 磁铁 / 总线的问题
   * ⚠️ 诊断期间**别发 0x20 目标**（闭环拿到的是未映射的原始角，方向会是乱的）。 */
  (void)center;
  *deg = raw;
#else
  *deg = wrap180(ARM_BASE_ENC_SIGN * (raw - ARM_BASE_ENC_ZERO_DEG))
         * ARM_BASE_DEG_SCALE + center;
#endif

  s_base_last_deg = *deg;            /* 只有读成功才更新"最后一次好值" */
  s_base_fault    = 0u;
  return 1u;
}

/* ------------------------------------------------------------------------- */
/* 速度型关节的闭环：一拍                                                      */
/* ------------------------------------------------------------------------- */

/* 连续读失败到几次，才认为"反馈真的断了"、可以作废积分/微分历史。
 *
 * ★ 为什么不是 1 次（见 tests/pid_loop_sim.c 场景⑥）：
 *   I²C 上偶发一次 NACK 是完全正常的（干扰、时钟拉伸）。如果一拍没读到就
 *   arm_pid_reset() 把积分清零，那么"每 2 秒失败一次"这种很轻的毛病，
 *   会让积分永远攒不起来 —— 仿真实测：40 秒后还停在 +7.86°，而正常应为 +1.49°。
 *   表现出来就是"I 项开了跟没开一样"，最难查的一种病。
 *   3 次 = 15ms 连续失败才算真断线，单次抖动就这样被滤掉了。
 * ⚠️ 以后要开 Kd(_US_PER_DPS != 0)，这里得改回 1 次：微分用 meas 的差值，
 *    中间漏了几拍会让 (meas - prev) 变成"跨越几百毫秒的位移 ÷ 5ms"，
 *    算出偏大的假速度。现在 Kd = 0，没有这个问题。 */
#define BASE_READ_FAIL_RESET_CNT   3u

static void vel_tick(uint8_t joint)
{
  const arm_joint_cfg_t *c = &g_arm_joint[joint];
  float    meas = 0.0f;
  float    err;
  float    u;
  float    u_ff = 0.0f;
  uint16_t us_stop;

  if (s_manual[joint] != 0u)
  {
    arm_pid_reset(&s_pid[joint]);    /* 手动期间不攒积分，交还时从零开始 */
    return;                          /* 手动模式：脉宽由 0x31 说了算，闭环让开 */
  }

  /* ---- ① 读角度（已换算到机械坐标系）。读不到就停转，别猜 ---- */
  if (base_read_deg(&meas) == 0u)
  {
    s_base_fault = 1u;               /* 故障位立刻置上：上位机这一帧就该看到 */

    if (s_read_fail_cnt < 0xFFu)
    {
      s_read_fail_cnt++;
    }

    /* 只有**连续**失败够多次才作废历史，单次 NACK 不动积分（见上面宏的注释） */
    if (s_read_fail_cnt == BASE_READ_FAIL_RESET_CNT)
    {
      arm_pid_reset(&s_pid[joint]);
    }

    us_stop = (c->us_stop != 0u) ? c->us_stop : SERVO_US_MID;
    (void)arm_set_us(joint, us_stop);
    return;
  }
  s_base_fault   = 0u;
  s_read_fail_cnt = 0u;

  /* ---- ② 误差：直接相减，不绕 ±180 --------------------------------------
   *
   * 底座行程是**一条弧**（−180 ~ +145），不是圆环。圆环才需要 wrap，弧不需要；
   * “抄近路”绕的那个方向在机械上不存在（会撞柱子）。
   * wrap180 的接缝（CENTER±180×SCALE = −257.5/+222.5）本来就在行程外，
   * 它不该出现在误差里。
   *
   * 读数先夹到“行程 ± ARM_LIMIT_OVERSHOOT_DEG”，再直接减：
   *   目标 = 145、实测 = 150 → err = −5 → 往回拉 ✓
   *   目标 = −180、实测 = −185 → err = +5 → 往回拉 ✓
   * 若把读数夹回行程内，这两种会算成 err = 0，停在限位外侧回不来、
   * bit3 还报“已到位”。±10° 带宽离接缝仍有 67° 以上。 */
  {
    float m = meas;

    if (m < ((float)c->deg_min - ARM_LIMIT_OVERSHOOT_DEG))
    {
      m = (float)c->deg_min - ARM_LIMIT_OVERSHOOT_DEG;
    }
    if (m > ((float)c->deg_max + ARM_LIMIT_OVERSHOOT_DEG))
    {
      m = (float)c->deg_max + ARM_LIMIT_OVERSHOOT_DEG;
    }

    err = s_tgt_deg[joint] - m;
  }
  s_base_err = (err >= 0.0f) ? err : -err;

  /* ---- ③ 软件限位：这里不重复判停 ---------------------------------------
   * 目标已由 proto_app.c 的 store_q_x10() 钳进 [deg_min, deg_max]，
   * err 永远指向行程内侧；滑行过冲由 PID 自己拉回。
   * 原判据“读数顶在限位上且误差朝外 → 停”两个分支都不可达
   * （目标 ≤ deg_max、读数 ≥ deg_max ⇒ err ≤ 0），这里不需要重复判停。 */

  /* ---- ④ PID 一拍 ------------------------------------------------------
   *
   * P/I/D、限幅、死区全在 arm_pid.c：加了 I 之后，死区必须在积分器那一侧
   * 处理（死区里只清输出、不清积分，走出死区那一下会窜出去），
   * 这种耦合不该散在调用方。 */
#if ARM_BASE_FF_ENABLE
  u_ff = s_ff_dps[joint] * c->us_per_dps;   /* 度/秒 × µs每度每秒 = µs */
#else
  u_ff = 0.0f;                              /* 还没标定 us_per_dps，先只用 P */
#endif

  u = arm_pid_step(&s_pid[joint], err, meas, u_ff);

  /* ---- ④b 最小出力补偿（静摩擦 / 死区补偿）------------------------------
   *
   * 为什么必须有这一段：误差 9° 时 u = 3×9 = 27µs，正好等于舵机死区；
   * 误差再小一点，u 就整段落在死区里 —— **P 项在 9° 以下恒定不出力**。
   * 于是"接近目标还有 10°"就再也走不动，只能干等积分慢慢攒（Ki=1.0 要 2.6s），
   * 现场看到的就是"接近目标就没劲了 / 发不出力"。
   *
   * 补偿规则：误差还在软件死区外，输出就不许小于 U_KICK，方向跟着误差走。
   * 参数怎么来的见 arm_cfg.h 的 ARM_BASE_U_KICK_US。
   *
   * ⚠️ 只补"该走"的情况：|err| <= DEADBAND 时 arm_pid_step() 已经按死区返回 0，
   *    这里不能把它再拉起来，否则目标附近会一直蹭、嗡嗡响。 */
  if ((s_base_err > ARM_BASE_DEADBAND_DEG) &&
      (u > -ARM_BASE_U_KICK_US) && (u < ARM_BASE_U_KICK_US))
  {
    u = (err > 0.0f) ? ARM_BASE_U_KICK_US : -ARM_BASE_U_KICK_US;
  }

  /* ---- ⑤ 写脉宽：us_stop ± u ---- */
  us_stop = (c->us_stop != 0u) ? c->us_stop : SERVO_US_MID;
  (void)arm_set_us(joint, (uint16_t)((float)us_stop + u));
}

/* ------------------------------------------------------------------------- */
/* 对外                                                                      */
/* ------------------------------------------------------------------------- */
void arm_joint_init(void)
{
  uint8_t j;

  for (j = 0u; j < ARM_JOINT_NUM; j++)
  {
    s_tgt_deg[j] = (float)g_arm_joint[j].deg_home;
    s_ff_dps[j]  = 0.0f;
    s_manual[j]  = 0u;

    /* PID 参数：全部来自 arm_cfg.h §五，这里只负责"搬过来"。
     * 四个关节都配，但只有 kind == ARM_JOINT_VEL 的那个会真跑。 */
    s_pid[j].kp       = ARM_BASE_KP_US_PER_DEG;
    s_pid[j].ki       = ARM_BASE_KI_US_PER_DEG_S;
    s_pid[j].kd       = ARM_BASE_KD_US_PER_DPS;
    s_pid[j].dt_s     = ARM_BASE_DT_S;
    s_pid[j].out_max  = ARM_BASE_U_MAX_US;
    s_pid[j].i_max    = ARM_BASE_I_MAX_US;
    s_pid[j].deadband = ARM_BASE_DEADBAND_DEG;
    s_pid[j].d_tau_s  = ARM_BASE_D_TAU_S;
    s_pid[j].d_max    = ARM_BASE_D_MAX_DPS;
    arm_pid_reset(&s_pid[j]);
  }

  s_base_fault = 0u;
  s_base_err   = 0.0f;

  if (as5600_init() == 0u)
  {
    /* 探测不到编码器，只有“当前配置里真有速度型关节”时才算故障。
     * J1 是 VEL，所以探不到 AS5600 就是底座故障；没有 VEL 关节时
     * 不点亮 bit4，其余三路照常工作。 */
    for (j = 0u; j < ARM_JOINT_NUM; j++)
    {
      if (g_arm_joint[j].kind == ARM_JOINT_VEL)
      {
        s_base_fault = 1u;
        break;
      }
    }
  }
}

void arm_joint_set_traj(uint8_t joint, float deg, float v_dps)
{
  if (joint >= (uint8_t)ARM_JOINT_NUM)
  {
    return;
  }

  s_tgt_deg[joint] = deg;
  s_ff_dps[joint]  = v_dps;

  if (g_arm_joint[joint].kind == ARM_JOINT_POS)
  {
    /* 位置型：立即生效，函数返回时舵机已经在动了 */
    (void)arm_set_deg(joint, deg);
  }
  /* 速度型：只记下目标，等 arm_joint_tick() 每一拍去追 */
}

void arm_joint_set_deg(uint8_t joint, float deg)
{
  arm_joint_set_traj(joint, deg, 0.0f);
}

float arm_joint_get_deg(uint8_t joint)
{
  float d = 0.0f;

  if (joint >= (uint8_t)ARM_JOINT_NUM)
  {
    return 0.0f;
  }

  if (g_arm_joint[joint].kind == ARM_JOINT_VEL)
  {
    if (base_read_deg(&d) != 0u)
    {
      return d;                      /* 真实测量值（已是机械坐标系） */
    }

    /* ★★★ 读失败返回「上一次成功值」，故障位已在 base_read_deg() 置上 ★★★
     *
     * **任何情况下都不返回目标值**：那会把“读失败”彻底藏起来
     * （磁铁在转、上位机“实际”纹丝不动）。
     * 也不用 −100/−200 哨兵：−100 是合法角度，0x10 里看不出异常；
     * move_start 会把它当插补起点 → 第一拍误差 ~100° → u 顶到 ±200µs
     * 全速反甩，是真实的错误动作，不只是显示问题。
     *
     * 返回 s_base_last_deg；故障由 bit2 / base_fault 表达，原始磁角度看 0x30。
     * 代价：单拍 NACK 时 0x10 的 act 最多旧 5ms，可接受。 */
    return s_base_last_deg;
  }

  return arm_get_deg(joint);         /* 位置型：CCR 反算 */
}

float arm_joint_get_deg_cached(void)
{
  /* 只给 50Hz 状态帧用：不碰 I²C，拿 vel_tick() 最近一次读成功的值。
   * 目的很具体 —— 上报不能占 5ms 插补拍的预算（一次读超时就是 2ms）。 */
  return s_base_last_deg;
}

uint16_t arm_joint_get_us(uint8_t joint)
{
  return arm_get_us(joint);
}

void arm_joint_tick(uint32_t now)
{
  uint8_t j;

  (void)now;                         /* 目前不用时间戳；将来要做速度估算时会用到 */

  for (j = 0u; j < ARM_JOINT_NUM; j++)
  {
    if (g_arm_joint[j].kind == ARM_JOINT_VEL)
    {
      vel_tick(j);
    }
  }
}

void arm_joint_set_manual(uint8_t joint, uint8_t on)
{
  if (joint >= (uint8_t)ARM_JOINT_NUM)
  {
    return;
  }

  const uint8_t prev = s_manual[joint];
  const uint8_t next = (on != 0u) ? 1u : 0u;

  /* ★ 状态**没变**时直接返回，别碰 PID。
   *
   * move_start() 每收到一条 0x20 都会调 set_manual(j, 0) 表示“交还闭环”。
   * 若无条件 reset，每发一次目标就清一次积分，I 项等于没开 ——
   * 现象是“Ki 调大了稳态误差也不掉”，极难查。
   *
   * 只在进/出手动那一刻清：
   *   进手动 → 丢掉闭环攒的积分；
   *   出手动 → 交还后第一拍从零开始，不接着手动前的旧账算。
   * 重复设成同一个值不再动积分。 */
  if (prev == next)
  {
    return;
  }

  s_manual[joint] = next;

  arm_pid_reset(&s_pid[joint]);

  if (s_manual[joint] != 0u)
  {
    s_ff_dps[joint] = 0.0f;          /* 手动时不留前馈，免得交还闭环时突然一冲 */
  }
}

void arm_joint_hold_position(void)
{
  uint8_t j;

  for (j = 0u; j < ARM_JOINT_NUM; j++)
  {
    if (g_arm_joint[j].kind == ARM_JOINT_VEL)
    {
      float d = 0.0f;

      /* 把目标改成"我现在实际在哪"，闭环于是原地不动 */
      if (base_read_deg(&d) != 0u)
      {
        s_tgt_deg[j] = d;
      }
      s_ff_dps[j] = 0.0f;
      arm_pid_reset(&s_pid[j]);      /* 目标刚被改写成实测值，旧的积分没有意义了 */
    }
    /* 位置型不需要处理：CCR 不被改写，舵机自己就保持住了 */
  }
}

uint8_t arm_joint_base_fault(void)
{
  return s_base_fault;
}

uint8_t arm_joint_moving(void)
{
  /* 本工程只有底座一个是速度型关节，所以只需要看它。
   * 将来如果加了第二个闭环关节，这里要改成遍历，别只判底座。 */
  if (g_arm_joint[ARM_JOINT_BASE].kind != ARM_JOINT_VEL)
  {
    return 0u;                       /* 不是闭环关节，没有"没到位"这回事 */
  }
  if (s_base_fault != 0u)
  {
    return 0u;                       /* 故障了就不再报"运动中"，让 bit2/bit4 去说 */
  }

  return (uint8_t)((s_base_err > ARM_MOVING_STILL_DEG) ? 1u : 0u);
}
