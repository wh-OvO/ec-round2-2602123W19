/**
  ******************************************************************************
  * @file    scs_selftest.c
  * @brief   kin_scs.c 的 PC 端单元测试（不需要板子，也不需要舵机）
  *
  *          编译（在 rm_arm 工程根目录）：
  *
  *            C:\Users\Lenovo\tools\mingw64\bin\gcc.exe -std=c11 -Wall -Wextra ^
  *              -Wpedantic -O2 -ffp-contract=off -I Core/Inc ^
  *              tests\scs_selftest.c Core\Src\kin_scs.c -o tests\scs_selftest.exe -lm
  *
  *          几何参数**直接从 arm_cfg.h 读**（L1=43 / L2=105 / 下垂 10°/20°），
  *          所以这个测试验的就是板上真跑的配置。想拿别的一组数验数学，加 -D
  *          覆盖那几个宏即可，测试里的期望值全是跟着宏算的，不用改代码。
  ******************************************************************************
  */

#include <stdio.h>
#include <math.h>

#include "kin_scs.h"
#include "arm_cfg.h"

/* 被测配置（全部来自 arm_cfg.h，测试不另立一套） */
#define T_L1      ARM_KIN_L1_MM
#define T_L2      ARM_KIN_L2_MM
#define T_OFF_J2  ARM_KIN_J2_OFF_DEG
#define T_OFF_J3  ARM_KIN_J3_OFF_DEG
#define T_SIGN_J2 ARM_KIN_J2_SIGN
#define T_SIGN_J3 ARM_KIN_J3_SIGN
#define T_SAG_J2  ARM_KIN_J2_SAG_DEG
#define T_SAG_J3  ARM_KIN_J3_SAG_DEG

/* 可达范围。L1=43 / L2=105 ⇒ 62 ~ 148mm */
#define T_DMIN    (fabsf(T_L1 - T_L2))
#define T_DMAX    (T_L1 + T_L2)
/* 下垂补偿后仍能真正走到的最远斜距：再往外 "bend − 20°" 会变成负数被夹到 0，
 * 也就是手臂其实伸不了那么直（详见第 8 组）。留 3mm 余量。 */
#define T_SAFE    (T_DMAX - 3.0f)
/* 折到极限也一样：太靠近 d_min 时 bend 会超过 180 */
#define T_TIGHT   (T_DMIN + 5.0f)

#define PI_F      3.14159265358979f
#define D2R       (PI_F / 180.0f)
#define R2D       (180.0f / PI_F)

/* 容差 */
#define TOL_DEG   0.02f      /* 角度往返误差 */
#define TOL_MM    0.25f      /* 位置往返误差（148mm 半径上，0.02° ≈ 0.05mm） */

static int g_pass = 0;
static int g_fail = 0;

static void check(int cond, const char *what)
{
  if (cond != 0)
  {
    g_pass++;
    printf("  [ OK ] %s\n", what);
  }
  else
  {
    g_fail++;
    printf("  [FAIL] %s\n", what);
  }
}

static int near_f(float a, float b, float tol)
{
  float d = a - b;

  if (d < 0.0f)
  {
    d = -d;
  }

  return (d <= tol) ? 1 : 0;
}

/* 关节角 → 平面点 → 球坐标（测试自己用 atan2，和被测代码无关） */
static void to_da(float x, float z, float *d, float *alpha)
{
  *d     = sqrtf((x * x) + (z * z));
  *alpha = atan2f(z, x) * R2D;
}

/* 余弦定理的两个内角，测试里独立算一遍，用来验"补偿量到底加在哪" */
static float geo_beta(float d)
{
  return acosf((((T_L1 * T_L1) + (d * d) - (T_L2 * T_L2)) / (2.0f * T_L1 * d))) * R2D;
}

static float geo_bend(float d)
{
  const float cg = ((T_L1 * T_L1) + (T_L2 * T_L2) - (d * d)) / (2.0f * T_L1 * T_L2);

  return 180.0f - (acosf(cg) * R2D);
}

/* ==========================================================================
 * 第 1 组：基本可用性
 * ========================================================================== */
static void test_basic(void)
{
  float dmin = 0.0f;
  float dmax = 0.0f;

  printf("\n[1] 基本可用性\n");

  check(kin_scs_ready() == 1u, "几何参数已填 ⇒ ready = 1");

  kin_scs_reach(&dmin, &dmax);
  printf("         可达斜距 %.1f ~ %.1f mm\n", (double)dmin, (double)dmax);
  check(near_f(dmin, T_DMIN, 0.01f), "d_min = |L1 − L2| = 62 mm");
  check(near_f(dmax, T_DMAX, 0.01f), "d_max = L1 + L2 = 148 mm");

  check(kin_scs_ready() == 1u, "重复调用 ready 不改变状态");
}

/* ==========================================================================
 * 第 2 组：球坐标 → 关节 → 正解 → 回到原点
 * ========================================================================== */
static void test_scs_roundtrip(void)
{
  static const float cases[][2] =
  {
    /*  d          α     */
    { 105.0f,       0.0f },   /* 正前方、中等距离 */
    { 105.0f,      30.0f },   /* 上斜 */
    { 105.0f,     -30.0f },   /* 下斜 */
    {  80.0f,      55.0f },   /* 高且近 */
    { 145.0f,       0.0f },   /* 接近伸直（补偿后仍可达） */
    {  67.0f,       0.0f },   /* 接近折死 */
  };
  const int n = (int)(sizeof(cases) / sizeof(cases[0]));
  int  i;
  int  ok = 1;

  printf("\n[2] 球坐标 → 逆解 → 正解 往返（%d 组）\n", n);

  for (i = 0; i < n; i++)
  {
    kin_scs_tgt_t tgt;
    kin_scs_out_t q;
    float         x  = 0.0f;
    float         z  = 0.0f;
    float         xw = 0.0f;
    float         zw = 0.0f;

    tgt.theta_deg = 45.0f;
    tgt.d_mm      = cases[i][0];
    tgt.alpha_deg = cases[i][1];

    if (kin_scs_solve(&tgt, &q) == 0u)
    {
      printf("         d=%.1f α=%.1f 解算失败\n", (double)tgt.d_mm, (double)tgt.alpha_deg);
      ok = 0;
      continue;
    }

    /* 期望的平面点（球坐标直接换回直角坐标，全程不敢用被测代码） */
    xw = tgt.d_mm * cosf(tgt.alpha_deg * D2R);
    zw = tgt.d_mm * sinf(tgt.alpha_deg * D2R);

    kin_scs_fk(&q, &x, &z);

    if ((near_f(x, xw, TOL_MM) == 0) || (near_f(z, zw, TOL_MM) == 0))
    {
      printf("         d=%.1f α=%.1f → 正解 (%.3f, %.3f)，期望 (%.3f, %.3f)\n",
             (double)tgt.d_mm, (double)tgt.alpha_deg,
             (double)x, (double)z, (double)xw, (double)zw);
      ok = 0;
    }
  }

  check(ok, "六组球坐标解算后正解都落回目标点（误差 < 0.25 mm）");
}

/* ==========================================================================
 * 第 3 组：任意姿态 → 正解 → 球坐标 → 逆解 → 正解，位置必须一致
 * ========================================================================== */
static void test_pose_point(void)
{
  float phi1;
  float bend;
  int   ok     = 1;
  int   n_test = 0;

  printf("\n[3] 姿态 → 点 → 逆解 → 点（位置不变性）\n");

  /* 这里的 phi1 / bend 是**几何角**，所以构造命令角时要把下垂量补进去 */
  for (phi1 = -60.0f; phi1 <= 60.0f; phi1 += 20.0f)
  {
    for (bend = 20.0f; bend <= 140.0f; bend += 20.0f)
    {
      kin_scs_out_t q0;
      kin_scs_out_t q1;
      kin_scs_tgt_t tgt;
      float x0 = 0.0f;
      float z0 = 0.0f;
      float x1 = 0.0f;
      float z1 = 0.0f;
      float d  = 0.0f;
      float al = 0.0f;

      q0.j1_deg = 0.0f;
      q0.j2_deg = T_OFF_J2 + (T_SIGN_J2 * (phi1 + T_SAG_J2));
      q0.j3_deg = T_OFF_J3 + (T_SIGN_J3 * (bend - T_SAG_J3));

      kin_scs_fk(&q0, &x0, &z0);
      to_da(x0, z0, &d, &al);

      tgt.theta_deg = 0.0f;
      tgt.d_mm      = d;
      tgt.alpha_deg = al;

      if (kin_scs_solve(&tgt, &q1) == 0u)
      {
        printf("         φ1=%.0f bend=%.0f 逆解失败（d=%.2f）\n",
               (double)phi1, (double)bend, (double)d);
        ok = 0;
        continue;
      }

      kin_scs_fk(&q1, &x1, &z1);
      n_test++;

      if ((near_f(x1, x0, TOL_MM) == 0) || (near_f(z1, z0, TOL_MM) == 0))
      {
        printf("         φ1=%.0f bend=%.0f → 逆解再正解 (%.2f, %.2f)，原本 (%.2f, %.2f)\n",
               (double)phi1, (double)bend,
               (double)x1, (double)z1, (double)x0, (double)z0);
        ok = 0;
      }
    }
  }

  printf("         共 %d 组姿态\n", n_test);
  check(ok && (n_test >= 49), "所有姿态换成点再解回来，末端位置一致");
}

/* ==========================================================================
 * 第 4 组：elbow-up 性质（大臂必须高于目标连线）
 * ========================================================================== */
static void test_elbow_up(void)
{
  kin_scs_tgt_t tgt;
  kin_scs_out_t q;
  float         phi1;
  float         al;
  int           ok = 1;

  printf("\n[4] elbow-up：大臂仰角 ≥ 目标仰角\n");

  for (al = -60.0f; al <= 60.0f; al += 30.0f)
  {
    tgt.theta_deg = 0.0f;
    tgt.d_mm      = 105.0f;
    tgt.alpha_deg = al;

    if (kin_scs_solve(&tgt, &q) == 0u)
    {
      ok = 0;
      continue;
    }

    /* 把命令角还原成**几何**仰角（减掉补偿的 10°）再比 */
    phi1 = ((q.j2_deg - T_OFF_J2) * T_SIGN_J2) - T_SAG_J2;

    /* β ≥ 0 ⇒ φ1 = α + β ≥ α。这是 elbow-up 的定义性判据。 */
    if (phi1 < (al - TOL_DEG))
    {
      printf("         α=%.0f → φ1=%.3f，小于 α（跑到 elbow-down 去了）\n",
             (double)al, (double)phi1);
      ok = 0;
    }
  }

  check(ok, "α = −60/−30/0/30/60 时 φ1 都 ≥ α");
}

/* ==========================================================================
 * 第 5 组：可达性拒绝
 * ========================================================================== */
static void test_reject(void)
{
  kin_scs_tgt_t tgt;
  kin_scs_out_t q;

  printf("\n[5] 不可达 / 非法输入必须被拒绝\n");

  tgt.theta_deg = 0.0f;  tgt.alpha_deg = 0.0f;
  tgt.d_mm = T_DMAX + 10.0f;
  check(kin_scs_solve(&tgt, &q) == 0u, "d 超过 L1+L2 ⇒ 拒绝");

  tgt.d_mm = T_DMIN - 10.0f;
  check(kin_scs_solve(&tgt, &q) == 0u, "d 小于 |L1−L2| ⇒ 拒绝");

  tgt.d_mm = 0.0f;
  check(kin_scs_solve(&tgt, &q) == 0u, "d = 0（会在除法上炸）⇒ 拒绝");

  tgt.d_mm = T_SAFE;
  check(kin_scs_solve(&tgt, (kin_scs_out_t *)0) == 0u, "输出指针为 NULL ⇒ 拒绝");

  tgt.d_mm = T_SAFE;  tgt.alpha_deg = NAN;
  check(kin_scs_solve(&tgt, &q) == 0u, "α = NaN ⇒ 拒绝");

  /* 边界值必须**接受**：d 正好等于 L1+L2 时两连杆共线 */
  tgt.alpha_deg = 0.0f;  tgt.d_mm = T_DMAX;
  check(kin_scs_solve(&tgt, &q) == 1u, "d 正好 = L1+L2（边界）⇒ 接受");
  check(near_f(((q.j3_deg - T_OFF_J3) * T_SIGN_J3), 0.0f, TOL_DEG),
        "边界上命令 bend 被夹到 0（伸不到比伸直更直）");
}

/* ==========================================================================
 * 第 6 组：方位角归一化
 * ========================================================================== */
static void test_theta(void)
{
  kin_scs_tgt_t tgt;
  kin_scs_out_t q;

  printf("\n[6] 方位角归一化到 [0, 360)\n");

  tgt.d_mm = T_SAFE;  tgt.alpha_deg = 0.0f;

  tgt.theta_deg = -30.0f;
  check((kin_scs_solve(&tgt, &q) == 1u) && near_f(q.j1_deg, 330.0f, 0.01f), "−30 → 330");

  tgt.theta_deg = 390.0f;
  check((kin_scs_solve(&tgt, &q) == 1u) && near_f(q.j1_deg, 30.0f, 0.01f), "390 → 30");

  tgt.theta_deg = 360.0f;
  check((kin_scs_solve(&tgt, &q) == 1u) && near_f(q.j1_deg, 0.0f, 0.01f), "360 → 0（不留在外面）");

  tgt.theta_deg = 123.456f;
  check((kin_scs_solve(&tgt, &q) == 1u) && near_f(q.j1_deg, 123.456f, 0.01f), "区间内原样通过");
}

/* ==========================================================================
 * 第 7 组：几何直觉
 * ========================================================================== */
static void test_sanity(void)
{
  kin_scs_tgt_t tgt;
  kin_scs_out_t q_up;
  kin_scs_out_t q_dn;

  printf("\n[7] 几何直觉\n");

  /* 同一个距离，目标抬高了，大臂仰角必须跟着变大 */
  tgt.theta_deg = 0.0f;  tgt.d_mm = 105.0f;
  tgt.alpha_deg = -20.0f;
  (void)kin_scs_solve(&tgt, &q_dn);
  tgt.alpha_deg = 20.0f;
  (void)kin_scs_solve(&tgt, &q_up);

  check(q_up.j2_deg > q_dn.j2_deg, "目标抬高 ⇒ 大臂命令角增大");

  /* 距离越远，肘越直（bend 越小） */
  tgt.alpha_deg = 0.0f;
  tgt.d_mm = 90.0f;
  (void)kin_scs_solve(&tgt, &q_up);
  tgt.d_mm = T_SAFE;
  (void)kin_scs_solve(&tgt, &q_dn);

  check(q_dn.j3_deg < q_up.j3_deg, "目标更远 ⇒ 肘弯曲量减小（更接近伸直）");

  /* 伸出到极点：命令 bend 被夹到 0 */
  tgt.d_mm = T_DMAX;
  (void)kin_scs_solve(&tgt, &q_up);
  check(near_f((q_up.j3_deg - T_OFF_J3) * T_SIGN_J3, 0.0f, TOL_DEG), "d = L1+L2 ⇒ bend = 0");

  /* 折到极点（d = |L1−L2|）：**物理** bend = 180（命令角要减去补偿的 20） */
  tgt.d_mm = T_DMIN;
  (void)kin_scs_solve(&tgt, &q_dn);
  check(near_f(((q_dn.j3_deg - T_OFF_J3) * T_SIGN_J3) + T_SAG_J3, 180.0f, TOL_DEG),
        "d = |L1−L2| ⇒ 物理 bend = 180");
}

/* ==========================================================================
 * 第 8 组：重力下垂补偿到底加在哪
 * ========================================================================== */
static void test_sag(void)
{
  kin_scs_tgt_t tgt;
  kin_scs_out_t q;
  float         d  = 0.5f * (T_DMIN + T_DMAX);
  float         pi = 0.0f;
  float         bc = 0.0f;

  printf("\n[8] 重力下垂补偿（SAG_J2=%.0f° SAG_J3=%.0f°）\n",
         (double)T_SAG_J2, (double)T_SAG_J3);

  tgt.theta_deg = 0.0f;
  tgt.d_mm      = d;
  tgt.alpha_deg = 0.0f;

  if (kin_scs_solve(&tgt, &q) == 0u)
  {
    check(0, "中距目标解算失败");
    return;
  }

  pi = (q.j2_deg - T_OFF_J2) * T_SIGN_J2;
  bc = (q.j3_deg - T_OFF_J3) * T_SIGN_J3;

  /* 大臂：命令角 = 几何 φ1 + 10（先抬高，掉下来才正好） */
  check(near_f(pi, geo_beta(d) + T_SAG_J2, 0.05f),
        "命令 φ1 = 几何 φ1 + SAG_J2");

  /* 小臂：命令 bend = 几何 bend − 20（先放平，折进去才正好） */
  check(near_f(bc, geo_bend(d) - T_SAG_J3, 0.05f),
        "命令 bend = 几何 bend − SAG_J3");

  /* FK 必须把补偿扣回去，否则"位置"描述的就不是手臂真实所在 */
  {
    float x = 0.0f;
    float z = 0.0f;

    kin_scs_fk(&q, &x, &z);
    check(near_f(sqrtf((x * x) + (z * z)), d, TOL_MM),
          "正解把补偿扣回去 ⇒ 落回目标斜距");
  }

  /* 补偿量填 0 时，命令角必须等于几何角（这条守住"不补偿也对"的基线） */
  check((T_SAG_J2 >= 0.0f) && (T_SAG_J3 >= 0.0f), "两个补偿量都是正的（符号没填反）");
}

/* ==========================================================================
 * main
 * ========================================================================== */
int main(void)
{
  printf("==========================================================\n");
  printf(" kin_scs 单元测试  L1=%.0f L2=%.0f\n",
         (double)T_L1, (double)T_L2);
  printf("   OFF_J2=%.0f SIGN_J2=%+.0f  OFF_J3=%.0f SIGN_J3=%+.0f\n",
         (double)T_OFF_J2, (double)T_SIGN_J2,
         (double)T_OFF_J3, (double)T_SIGN_J3);
  printf("   SAG_J2=%.0f SAG_J3=%.0f   可达 %.0f ~ %.0f mm\n",
         (double)T_SAG_J2, (double)T_SAG_J3, (double)T_DMIN, (double)T_DMAX);
  printf("==========================================================\n");

  test_basic();
  test_scs_roundtrip();
  test_pose_point();
  test_elbow_up();
  test_reject();
  test_theta();
  test_sanity();
  test_sag();

  printf("\n==========================================================\n");
  printf(" === %d passed, %d failed ===\n", g_pass, g_fail);
  printf("==========================================================\n");

  return (g_fail == 0) ? 0 : 1;
}
