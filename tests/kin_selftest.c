/**
  ******************************************************************************
  * @file    kin_selftest.c
  * @brief   arm_kin.c 的 PC 端单元测试（不需要板子，也不需要舵机）
  *
  *          编译（在 rm_arm 工程根目录）：
  *
  *            C:\Users\Lenovo\tools\mingw64\bin\gcc.exe -std=c11 -Wall -Wextra ^
  *              -Wpedantic -O2 -ffp-contract=off -I Core/Inc ^
  *              -DARM_KIN_L1_MM=150.0f -DARM_KIN_L2_MM=120.0f ^
  *              -DARM_KIN_J2_OFF_DEG=90.0f -DARM_KIN_J3_OFF_DEG=0.0f ^
  *              tests\kin_selftest.c Core\Src\arm_kin.c -o tests\kin_selftest.exe -lm
  *
  *          为什么几何参数用 -D 传：arm_cfg.h 里那几个宏真的还是 0（未标定），
  *          而测试要验的是**数学**，不是那几个数。用 -D 覆盖成一组整数值，
  *          数学对不对就能验；等真机标定完，把同样的数填进 arm_cfg.h 即可。
  *
  *          OFF_J2 故意取 90（而不是 0），就是为了让"偏移 + 符号"这段代码
  *          真的被跑到 —— 全填 0 的话映射是个恒等式，写错了也测不出来。
  ******************************************************************************
  */

#include <stdio.h>
#include <math.h>

#include "arm_kin.h"
#include "arm_cfg.h"

/* 测试用的几何参数（和编译命令里的 -D 必须一致） */
#define T_L1      150.0f
#define T_L2      120.0f
#define T_OFF_J2  90.0f
#define T_OFF_J3  0.0f
#define T_SIGN    1.0f

/* 容差 */
#define TOL_DEG   0.02f      /* 角度往返误差 */
#define TOL_MM    0.25f      /* 位置往返误差（150mm 半径上，0.02° ≈ 0.05mm） */

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

/* 关节角 → 平面点 → 极坐标（测试自己用 atan2，和被测代码无关） */
static void to_polar(float x, float z, float *d, float *alpha)
{
  *d     = sqrtf((x * x) + (z * z));
  *alpha = atan2f(z, x) * (180.0f / 3.14159265358979f);
}

/* ==========================================================================
 * 第 1 组：基本可用性
 * ========================================================================== */
static void test_basic(void)
{
  float dmin = 0.0f;
  float dmax = 0.0f;

  printf("\n[1] 基本可用性\n");

  check(arm_kin_ready() == 1u, "几何参数已填 ⇒ ready = 1");

  arm_kin_reach(&dmin, &dmax);
  check(near_f(dmin, fabsf(T_L1 - T_L2), 0.01f), "d_min = |L1 − L2| = 30 mm");
  check(near_f(dmax, T_L1 + T_L2, 0.01f),         "d_max = L1 + L2 = 270 mm");

  check(arm_kin_ready() == 1u, "重复调用 ready 不改变状态");
}

/* ==========================================================================
 * 第 2 组：极坐标 → 关节 → 正解 → 回到原点
 * ========================================================================== */
static void test_polar_roundtrip(void)
{
  static const float cases[][2] =
  {
    /*  d      α     */
    { 200.0f,   0.0f },   /* 正前方 */
    { 200.0f,  30.0f },   /* 上斜 */
    { 200.0f, -30.0f },   /* 下斜 */
    { 150.0f,  60.0f },   /* 高且近 */
    { 270.0f,   0.0f },   /* 完全伸直（边界） */
    {  31.0f,   0.0f },   /* 几乎折死（边界附近） */
  };
  const int n = (int)(sizeof(cases) / sizeof(cases[0]));
  int  i;
  int  ok = 1;

  printf("\n[2] 极坐标 → 逆解 → 正解 往返（%d 组）\n", n);

  for (i = 0; i < n; i++)
  {
    arm_kin_polar_t  tgt;
    arm_kin_joints_t q;
    float            x  = 0.0f;
    float            z  = 0.0f;
    float            xw = 0.0f;
    float            zw = 0.0f;

    tgt.theta_deg = 45.0f;
    tgt.d_mm      = cases[i][0];
    tgt.alpha_deg = cases[i][1];

    if (arm_kin_solve_polar(&tgt, &q) == 0u)
    {
      printf("         d=%.1f α=%.1f 解算失败\n", (double)tgt.d_mm, (double)tgt.alpha_deg);
      ok = 0;
      continue;
    }

    /* 期望的平面点（极坐标直接换回直角坐标，全程不敢用被测代码） */
    xw = tgt.d_mm * cosf(tgt.alpha_deg * (3.14159265358979f / 180.0f));
    zw = tgt.d_mm * sinf(tgt.alpha_deg * (3.14159265358979f / 180.0f));

    arm_kin_fk(&q, &x, &z);

    if ((near_f(x, xw, TOL_MM) == 0) || (near_f(z, zw, TOL_MM) == 0))
    {
      printf("         d=%.1f α=%.1f → 正解 (%.3f, %.3f)，期望 (%.3f, %.3f)\n",
             (double)tgt.d_mm, (double)tgt.alpha_deg,
             (double)x, (double)z, (double)xw, (double)zw);
      ok = 0;
    }
  }

  check(ok, "六组极坐标解算后正解都落回目标点（误差 < 0.25 mm）");
}

/* ==========================================================================
 * 第 3 组：任意姿态 → 正解 → 极坐标 → 逆解 → 正解，位置必须一致
 * ========================================================================== */
static void test_pose_point(void)
{
  float phi1;
  float bend;
  int   ok     = 1;
  int   n_test = 0;

  printf("\n[3] 姿态 → 点 → 逆解 → 点（位置不变性）\n");

  for (phi1 = -70.0f; phi1 <= 70.0f; phi1 += 20.0f)
  {
    for (bend = 0.0f; bend <= 160.0f; bend += 20.0f)
    {
      arm_kin_joints_t q0;
      arm_kin_joints_t q1;
      arm_kin_polar_t  tgt;
      float x0 = 0.0f;
      float z0 = 0.0f;
      float x1 = 0.0f;
      float z1 = 0.0f;
      float d  = 0.0f;
      float al = 0.0f;

      q0.j1_deg = 0.0f;
      q0.j2_deg = T_OFF_J2 + (T_SIGN * phi1);
      q0.j3_deg = T_OFF_J3 + (T_SIGN * bend);

      arm_kin_fk(&q0, &x0, &z0);
      to_polar(x0, z0, &d, &al);

      tgt.theta_deg = 0.0f;
      tgt.d_mm      = d;
      tgt.alpha_deg = al;

      if (arm_kin_solve_polar(&tgt, &q1) == 0u)
      {
        printf("         φ1=%.0f bend=%.0f 逆解失败\n", (double)phi1, (double)bend);
        ok = 0;
        continue;
      }

      arm_kin_fk(&q1, &x1, &z1);
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
  check(ok && (n_test > 50), "所有姿态换成点再解回来，末端位置一致");
}

/* ==========================================================================
 * 第 4 组：elbow-up 性质（大臂必须高于目标连线）
 * ========================================================================== */
static void test_elbow_up(void)
{
  arm_kin_polar_t  tgt;
  arm_kin_joints_t q;
  float            phi1;
  int              ok = 1;
  float            al;

  printf("\n[4] elbow-up：大臂仰角 ≥ 目标仰角\n");

  for (al = -60.0f; al <= 60.0f; al += 30.0f)
  {
    tgt.theta_deg = 0.0f;
    tgt.d_mm      = 200.0f;
    tgt.alpha_deg = al;

    if (arm_kin_solve_polar(&tgt, &q) == 0u)
    {
      ok = 0;
      continue;
    }

    phi1 = (q.j2_deg - T_OFF_J2) * T_SIGN;

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
  arm_kin_polar_t  tgt;
  arm_kin_joints_t q;

  printf("\n[5] 不可达 / 非法输入必须被拒绝\n");

  tgt.theta_deg = 0.0f;  tgt.d_mm = 275.0f;  tgt.alpha_deg = 0.0f;
  check(arm_kin_solve_polar(&tgt, &q) == 0u, "d = 275 > L1+L2 = 270 ⇒ 拒绝");

  tgt.d_mm = 25.0f;
  check(arm_kin_solve_polar(&tgt, &q) == 0u, "d = 25 < |L1−L2| = 30 ⇒ 拒绝");

  tgt.d_mm = 0.0f;
  check(arm_kin_solve_polar(&tgt, &q) == 0u, "d = 0（会在除法上炸）⇒ 拒绝");

  tgt.d_mm = 200.0f;
  check(arm_kin_solve_polar(&tgt, (arm_kin_joints_t *)0) == 0u, "输出指针为 NULL ⇒ 拒绝");

  tgt.theta_deg = 0.0f;  tgt.d_mm = 200.0f;  tgt.alpha_deg = NAN;
  check(arm_kin_solve_polar(&tgt, &q) == 0u, "α = NaN ⇒ 拒绝");

  /* 边界值必须**接受**：d 正好等于 L1+L2 时两连杆共线 */
  tgt.alpha_deg = 0.0f;  tgt.d_mm = 270.0f;
  check(arm_kin_solve_polar(&tgt, &q) == 1u, "d 正好 = 270（边界）⇒ 接受");
  check(near_f((q.j3_deg - T_OFF_J3) * T_SIGN, 0.0f, TOL_DEG), "边界上 bend = 0（两连杆共线）");
}

/* ==========================================================================
 * 第 6 组：方位角归一化
 * ========================================================================== */
static void test_theta(void)
{
  arm_kin_polar_t  tgt;
  arm_kin_joints_t q;

  printf("\n[6] 方位角归一化到 [0, 360)\n");

  tgt.d_mm = 200.0f;  tgt.alpha_deg = 0.0f;

  tgt.theta_deg = -30.0f;
  check((arm_kin_solve_polar(&tgt, &q) == 1u) && near_f(q.j1_deg, 330.0f, 0.01f), "−30 → 330");

  tgt.theta_deg = 390.0f;
  check((arm_kin_solve_polar(&tgt, &q) == 1u) && near_f(q.j1_deg, 30.0f, 0.01f), "390 → 30");

  tgt.theta_deg = 360.0f;
  check((arm_kin_solve_polar(&tgt, &q) == 1u) && near_f(q.j1_deg, 0.0f, 0.01f), "360 → 0（不留在外面）");

  tgt.theta_deg = 123.456f;
  check((arm_kin_solve_polar(&tgt, &q) == 1u) && near_f(q.j1_deg, 123.456f, 0.01f), "区间内原样通过");
}

/* ==========================================================================
 * 第 7 组：几何直觉
 * ========================================================================== */
static void test_sanity(void)
{
  arm_kin_polar_t  tgt;
  arm_kin_joints_t q_up;
  arm_kin_joints_t q_dn;

  printf("\n[7] 几何直觉\n");

  /* 同一个距离，目标抬高了，大臂仰角必须跟着变大 */
  tgt.theta_deg = 0.0f;  tgt.d_mm = 200.0f;
  tgt.alpha_deg = -20.0f;
  (void)arm_kin_solve_polar(&tgt, &q_dn);
  tgt.alpha_deg = 20.0f;
  (void)arm_kin_solve_polar(&tgt, &q_up);

  check(q_up.j2_deg > q_dn.j2_deg, "目标抬高 ⇒ 大臂命令角增大");

  /* 距离越远，肘越直（bend 越小） */
  tgt.alpha_deg = 0.0f;
  tgt.d_mm = 200.0f;
  (void)arm_kin_solve_polar(&tgt, &q_up);
  tgt.d_mm = 260.0f;
  (void)arm_kin_solve_polar(&tgt, &q_dn);

  check(q_dn.j3_deg < q_up.j3_deg, "目标更远 ⇒ 肘弯曲量减小（更接近伸直）");

  /* 伸出到极点时 bend 必须为 0 */
  tgt.d_mm = 270.0f;
  (void)arm_kin_solve_polar(&tgt, &q_up);
  check(near_f((q_up.j3_deg - T_OFF_J3) * T_SIGN, 0.0f, TOL_DEG), "d = L1+L2 ⇒ bend = 0");

  /* 折到极点（d = |L1−L2|）时 bend 必须为 180 */
  tgt.d_mm = 30.0f;
  (void)arm_kin_solve_polar(&tgt, &q_dn);
  check(near_f((q_dn.j3_deg - T_OFF_J3) * T_SIGN, 180.0f, TOL_DEG), "d = |L1−L2| ⇒ bend = 180");
}

/* ==========================================================================
 * main
 * ========================================================================== */
int main(void)
{
  printf("==========================================================\n");
  printf(" arm_kin 单元测试  L1=%.0f L2=%.0f OFF_J2=%.0f OFF_J3=%.0f\n",
         (double)T_L1, (double)T_L2, (double)T_OFF_J2, (double)T_OFF_J3);
  printf("==========================================================\n");

  test_basic();
  test_polar_roundtrip();
  test_pose_point();
  test_elbow_up();
  test_reject();
  test_theta();
  test_sanity();

  printf("\n==========================================================\n");
  printf(" === %d passed, %d failed ===\n", g_pass, g_fail);
  printf("==========================================================\n");

  return (g_fail == 0) ? 0 : 1;
}
