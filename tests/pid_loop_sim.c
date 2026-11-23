/* ============================================================================
 * pid_loop_sim.c —— 闭环仿真（PC）
 *
 * 把 Core/Src/arm_pid.c **原样**编进来，后面接一个按实测参数写的舵机模型，
 * 跑完整闭环，看中间过程。
 *
 * 为什么板上跑过了还要仿：
 *   板上只能看到"最后停在哪"和"上位机刷新的那 50Hz 快照"，
 *   看不到积分攒到多少、看不到过冲、更看不到接缝那一下发生了什么。
 *   仿真能把每一拍都打出来 —— 而且它验的是**同一份 arm_pid.c**。
 *
 * 舵机模型（参数来自 2026-11-18 实测，见 plan.md §8）：
 *   · 死区：|u| ≤ DB 不动。空载实测 10µs/20µs，带载 27µs 仍推不动 → DB = 27
 *   · 死区外：速度 ∝ (|u| − DB)。实测 50µs（1550）≈ 180°/s → K = 180/23 ≈ 7.83 °/s/µs
 *   · 一阶滞后 60ms（舵机内部响应，不是瞬时）
 *   · 50Hz PWM：脉宽每 20ms 才被舵机采样一次，这 20ms 内保持不变
 *
 * 编译（和单测一样的命令）：
 *   gcc -std=c11 -Wall -Wextra -O2 -ffp-contract=off -I Core/Inc \
 *       tests/pid_loop_sim.c Core/Src/arm_pid.c -o tests/pid_loop_sim.exe
 * ========================================================================== */

#include <stdio.h>
#include "arm_pid.h"

/* ---- 舵机模型参数（改这里就是改被控对象） -------------------------------- */
#define SIM_DT_S      0.005f      /* 闭环节拍 5ms = 200Hz */
#define SERVO_DB_US   27.0f       /* 带载死区 */
#define SERVO_K       7.83f       /* deg/s per µs（死区之外） */
#define SERVO_TAU_S   0.06f       /* 一阶滞后 */
#define PWM_PERIOD_TICKS 4        /* 50Hz PWM：每 4 拍（20ms）采样一次脉宽 */
#define SERVO_US_STOP 1500.0f

static float wrap180(float a)
{
  while (a > 180.0f)   { a -= 360.0f; }
  while (a <= -180.0f) { a += 360.0f; }
  return a;
}

/* 微分合理性上限。1000 = arm_cfg.h 的 ARM_BASE_D_MAX_DPS；
 * 场景⑤b 会临时改成 0，用来复现"加这道闸之前"的行为做对照。 */
static float g_d_max = 1000.0f;

/* 舵机能转多快（°/s）—— 模型里 (u−DB)·K 是**线性**的，没有饱和，
 * 但这只对 u 小的时候成立：真舵机到 200°/s 左右就上不去了。
 * 默认 1e9 = 不饱和，这样 ①②③… 那些老场景的结果**逐位不变**；
 * 硬要改 u_max 的 ⑧ 组会临时设成 200。 */
static float g_v_max = 1e9f;

/* 输出限幅。arm_cfg.h 的 ARM_BASE_U_MAX_US 现在是 50（标定期的临时值），
 * ⑧ 组会把它拨到 300 做对照。 */
static float g_out_max = 50.0f;

/* 一个被控对象实例：位置 + 当前速度 + 上一拍保持的脉宽 */
typedef struct
{
  float deg;
  float w_dps;
  float u_hold;
} plant_t;

static void plant_step(plant_t *pl, float u_cmd, uint32_t tick)
{
  /* 舵机只在 PWM 周期边界采样脉宽 */
  if ((tick % PWM_PERIOD_TICKS) == 0u)
  {
    pl->u_hold = u_cmd;
  }

  float d = pl->u_hold;
  float w_tgt = 0.0f;

  if      (d >  SERVO_DB_US) { w_tgt =  (d - SERVO_DB_US) * SERVO_K; }
  else if (d < -SERVO_DB_US) { w_tgt =  (d + SERVO_DB_US) * SERVO_K; }

  /* 转速上限。g_v_max = 1e9 时这两行什么都不做（老场景不受影响） */
  if (w_tgt >  g_v_max) { w_tgt =  g_v_max; }
  if (w_tgt < -g_v_max) { w_tgt = -g_v_max; }

  /* 一阶滞后 */
  const float alpha = SIM_DT_S / (SERVO_TAU_S + SIM_DT_S);
  pl->w_dps += alpha * (w_tgt - pl->w_dps);

  pl->deg = wrap180(pl->deg + (pl->w_dps * SIM_DT_S));
}

/* 跑一个场景。返回"进入 ±1.5° 死区"的时刻（秒），没进去返回 -1 */
static float run(const char *title, float tgt, float start, float kp, float ki,
                 float kd, float out_max, float seconds, uint8_t verbose)
{
  arm_pid_t p;
  plant_t   pl;
  uint32_t  k;
  const uint32_t n = (uint32_t)(seconds / SIM_DT_S);
  float     t_in_zone = -1.0f;

  p.kp       = kp;
  p.ki       = ki;
  p.kd       = kd;
  p.dt_s     = SIM_DT_S;
  p.out_max  = out_max;
  p.i_max    = 60.0f;
  p.deadband = 1.5f;
  p.d_tau_s  = 0.05f;
  p.d_max    = g_d_max;
  arm_pid_reset(&p);

  pl.deg    = start;
  pl.w_dps  = 0.0f;
  pl.u_hold = 0.0f;

  printf("\n===== %s =====\n", title);
  printf("  Kp=%.2f Ki=%.2f Kd=%.2f  out_max=%.0f  目标=%+.1f° 起点=%+.1f°\n",
         (double)kp, (double)ki, (double)kd, (double)out_max, (double)tgt, (double)start);

  if (verbose != 0u)
  {
    printf("   t[s]   meas     err      u     i_term   speed[deg/s]\n");
  }

  for (k = 0u; k < n; k++)
  {
    const float err = wrap180(tgt - pl.deg);
    const float u   = arm_pid_step(&p, err, pl.deg, 0.0f);

    if ((verbose != 0u) && ((k % 200u) == 0u))
    {
      printf("  %5.1f  %+7.2f  %+7.2f  %+6.1f  %+7.1f   %+8.1f\n",
             (double)((float)k * SIM_DT_S), (double)pl.deg, (double)err, (double)u,
             (double)(p.integ * p.ki), (double)pl.w_dps);
    }

    if ((t_in_zone < 0.0f) && (err < 1.5f) && (err > -1.5f))
    {
      t_in_zone = (float)k * SIM_DT_S;
    }

    plant_step(&pl, u, k);
  }

  printf("  → 最终 %.2f°（误差 %+.2f°），%s\n", (double)pl.deg,
         (double)wrap180(tgt - pl.deg),
         (t_in_zone >= 0.0f) ? "已进死区" : "**没进死区**");
  if (t_in_zone >= 0.0f)
  {
    printf("  → 首次进入 ±1.5° 用时 %.2f s\n", (double)t_in_zone);
  }
  return t_in_zone;
}

/* 参数扫描：不打印过程，只给指标。
 *   t2      第一次进 ±2° 的时刻（太久 = 用户会觉得"卡住不动"）
 *   ovs     过冲（穿过目标后跑到反方向的最大偏差）
 *   fin     20 秒后的残余误差（死区 1.5° 以内算合格） */
static void sweep_one(float kp, float ki, float tgt, float start, float secs)
{
  arm_pid_t p;
  plant_t   pl;
  uint32_t  k;
  const uint32_t n = (uint32_t)(secs / SIM_DT_S);
  const int   init_side = (wrap180(tgt - start) > 0.0f) ? 1 : -1;
  float t2  = -1.0f;
  float ovs = 0.0f;
  float fin;
  int   crossed = 0;

  p.kp = kp; p.ki = ki; p.kd = 0.0f; p.dt_s = SIM_DT_S;
  p.out_max = g_out_max; p.i_max = 60.0f; p.deadband = 1.5f; p.d_tau_s = 0.05f;
  p.d_max   = g_d_max;
  arm_pid_reset(&p);

  pl.deg = start; pl.w_dps = 0.0f; pl.u_hold = 0.0f;

  for (k = 0u; k < n; k++)
  {
    const float err = wrap180(tgt - pl.deg);
    const float u   = arm_pid_step(&p, err, pl.deg, 0.0f);
    const float ae  = (err < 0.0f) ? -err : err;
    const int   s   = (err > 0.0f) ? 1 : -1;

    if ((t2 < 0.0f) && (ae < 2.0f))
    {
      t2 = (float)k * SIM_DT_S;
    }

    /* 第一次跑到目标对面 ⇒ 从这之后的最大偏差就是"过冲/振荡幅度" */
    if ((crossed == 0) && (s != init_side) && (ae > 0.2f))
    {
      crossed = 1;
    }
    if ((crossed != 0) && (ae > ovs))
    {
      ovs = ae;
    }

    plant_step(&pl, u, k);
  }

  fin = wrap180(tgt - pl.deg);
  fin = (fin < 0.0f) ? -fin : fin;

  printf("  Kp %5.1f  Ki %4.1f | 进±2° %6.2fs | 过冲 %5.2f° | 20s 残差 %5.2f°\n",
         (double)kp, (double)ki, (double)t2, (double)ovs, (double)fin);
}

int main(void)
{
  printf("============================================================\n");
  printf(" 闭环仿真（arm_pid.c 原样 + 舵机模型：死区 %.0fus，%.1f deg/s/µs）\n",
         (double)SERVO_DB_US, (double)SERVO_K);
  printf("============================================================\n");

  /* ① 现场这一档：目标 0，实测停在 +9（纯 P 的死区残差） */
  (void)run("① 现场：0° 目标，从 +9° 出发（Ki=0.2，就是刚烧进去的）",
            0.0f, 9.0f, 3.0f, 0.2f, 0.0f, 50.0f, 40.0f, 1u);

  /* ② 纯 P 对照：应该停住不动（证明 9° 是 P 的死区残差，不是 bug） */
  (void)run("② 对照：同一起点，但 Ki=0（纯 P）",
            0.0f, 9.0f, 3.0f, 0.0f, 0.0f, 50.0f, 40.0f, 1u);

  /* ③ 大角度阶跃：看超调 */
  (void)run("③ 阶跃 90°：从 +9° 到 +90°",
            90.0f, 9.0f, 3.0f, 0.2f, 0.0f, 50.0f, 20.0f, 1u);

  /* ③b Ki 加大 5 倍：0.2 太慢（① 用了 27s），看 1.0 会不会冲 */
  (void)run("③b 同①，但 Ki=1.0（5 倍）",
            0.0f, 9.0f, 3.0f, 1.0f, 0.0f, 50.0f, 40.0f, 1u);

  /* ④ 真跨接缝：+175 → -175，最短路径是 +10°，要穿过 +180/-180 */
  (void)run("④ 跨接缝：目标 -175°，起点 +175°（要走 +10°，穿过接缝）",
            -175.0f, 175.0f, 3.0f, 0.2f, 0.0f, 50.0f, 15.0f, 1u);

  /* ⑤ 同上 + 开 D + d_max 生效：接缝那一拍被丢掉，应该和 ④ 一样平稳 */
  (void)run("⑤ 跨接缝 + Kd=1.0 + d_max=1000（这道闸已生效）",
            -175.0f, 175.0f, 3.0f, 0.2f, 1.0f, 50.0f, 15.0f, 1u);

  /* ⑤b 对照：把闸关掉，复现"加闸之前"的样子 */
  g_d_max = 0.0f;
  (void)run("⑤b 同上但 d_max=0（改动前：假速度没人接）",
            -175.0f, 175.0f, 3.0f, 0.2f, 1.0f, 50.0f, 15.0f, 1u);
  g_d_max = 1000.0f;

  /* ⑥ 积分被反复清零：模拟"偶发一次读失败 → arm_pid_reset"，看 I 还能不能攒起来 */
  {
    arm_pid_t p;
    plant_t   pl;
    uint32_t  k;
    const float tgt = 0.0f;

    p.kp = 3.0f; p.ki = 0.2f; p.kd = 0.0f; p.dt_s = SIM_DT_S;
    p.out_max = 50.0f; p.i_max = 60.0f; p.deadband = 1.5f; p.d_tau_s = 0.05f;
    arm_pid_reset(&p);

    pl.deg = 9.0f; pl.w_dps = 0.0f; pl.u_hold = 0.0f;

    printf("\n===== ⑥ 每 2 秒来一次「读失败」（arm_pid_reset）=====\n");
    printf("  Ki 还是 0.2，但积分每 2000ms 被清一次\n");
    printf("   t[s]   meas     err      u     i_term\n");

    for (k = 0u; k < 8000u; k++)
    {
      const float err = wrap180(tgt - pl.deg);

      if ((k != 0u) && ((k % 400u) == 0u))
      {
        arm_pid_reset(&p);          /* ← 一次 I²C 读失败，vel_tick 里就是这么干的 */
      }

      const float u = arm_pid_step(&p, err, pl.deg, 0.0f);

      if ((k % 400u) == 0u)
      {
        printf("  %5.1f  %+7.2f  %+7.2f  %+6.1f  %+7.1f\n",
               (double)((float)k * SIM_DT_S), (double)pl.deg, (double)err,
               (double)u, (double)(p.integ * p.ki));
      }

      plant_step(&pl, u, k);
    }
    printf("  → 40 秒后还停在 %+.2f°（没 reset 的话 ① 是 +1.49°）\n", (double)pl.deg);
  }

  /* ⑦ 参数扫描：0° 目标、从 +9° 出发（就是现场那个残差） */
  printf("\n===== ⑦ 参数扫描（目标 0°，起点 +9°，20 秒）=====\n");
  sweep_one( 3.0f, 0.0f, 0.0f, 9.0f, 20.0f);
  sweep_one( 3.0f, 0.2f, 0.0f, 9.0f, 20.0f);
  sweep_one( 3.0f, 0.5f, 0.0f, 9.0f, 20.0f);
  sweep_one( 3.0f, 1.0f, 0.0f, 9.0f, 20.0f);
  sweep_one( 3.0f, 2.0f, 0.0f, 9.0f, 20.0f);
  sweep_one( 6.0f, 0.0f, 0.0f, 9.0f, 20.0f);
  sweep_one( 6.0f, 1.0f, 0.0f, 9.0f, 20.0f);
  sweep_one(10.0f, 0.0f, 0.0f, 9.0f, 20.0f);
  sweep_one(10.0f, 1.0f, 0.0f, 9.0f, 20.0f);

  printf("\n===== ⑦b 大角度（目标 90°，起点 0°，20 秒）—— 看高 Kp 会不会振 =====\n");
  sweep_one( 3.0f, 1.0f, 90.0f, 0.0f, 20.0f);
  sweep_one( 6.0f, 1.0f, 90.0f, 0.0f, 20.0f);
  sweep_one(10.0f, 1.0f, 90.0f, 0.0f, 20.0f);

  /* ⑧ u_max 从 50 改回 300 —— 50 是标定"怕填错符号飞车"时的临时值。
   *
   *    真正的区别不是"更有力"，而是**比例带变宽**：
   *        比例带 = u_max / Kp  =  50/3 ≈ ±16.7°  →  300/3 = ±100°
   *    误差 20° 时，u_max=50 已经顶满（bang-bang），u_max=300 才给 60µs。
   *    所以 50 的时候大角度阶跃根本不是 PID，是开关控制。
   *
   *    ★ 必须同时把 v_max 打开：模型里 (u−DB)·K 是线性的，300µs 会算出
   *      (300−27)×7.83 ≈ 2100°/s 这种不存在的速度。真舵机到 200 就上不去了。 */
  printf("\n===== ⑧ u_max 50 vs 300（v_max=200 已打开）=====\n");
  g_v_max = 200.0f;

  printf("\n--- ⑧a 大角度：目标 90°，起点 0°，Ki=1.0（看 u_max 到多大开始过冲）---\n");
  {
    static const float um_list[] = { 50.0f, 60.0f, 80.0f, 100.0f, 150.0f, 300.0f };
    unsigned i;

    for (i = 0u; i < (unsigned)(sizeof(um_list) / sizeof(um_list[0])); i++)
    {
      printf("  u_max=%.0f", (double)um_list[i]);
      g_out_max = um_list[i];
      sweep_one( 3.0f, 1.0f, 90.0f, 0.0f, 20.0f);
    }
  }
  g_out_max = 300.0f;
  sweep_one( 3.0f, 0.2f, 90.0f, 0.0f, 20.0f);
  sweep_one( 3.0f, 2.0f, 90.0f, 0.0f, 20.0f);
  sweep_one( 3.0f, 4.0f, 90.0f, 0.0f, 20.0f);

  printf("\n--- ⑧b 小角度：目标 0°，起点 +9°（就是现场那 9° 残差）---\n");
  sweep_one( 3.0f, 0.2f, 0.0f, 9.0f, 20.0f);
  sweep_one( 3.0f, 1.0f, 0.0f, 9.0f, 20.0f);
  sweep_one( 3.0f, 2.0f, 0.0f, 9.0f, 20.0f);

  g_out_max = 50.0f;
  g_v_max   = 1e9f;

  return 0;
}
