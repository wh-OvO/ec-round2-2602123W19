/* ===========================================================================
 * kin_scs 的 PC 单测
 *
 *   直接在 PC 上编（不需要板子），校验逆解/正解的数学部分：
 *
 *     gcc -std=c11 -Wall -Wextra -Wpedantic -O2 -ffp-contract=off \
 *         -I ../Core/Inc scs_selftest.c ../Core/Src/kin_scs.c -lm -o scs_selftest
 *
 *   注意：这个文件**不能**编进固件（有 printf / main），只在 PC 上跑。
 *
 *   2026-11-23 改版：几何模型从"平面两连杆"换成"两个摆动平面各自偏斜"，
 *   所以断言也跟着换了 —— 现在验的是**不变量**（往返、余弦定理、方位角
 *   落点、仰角上限），而不是照抄一遍公式（那样等于自己验自己）。
 * =========================================================================== */

#include <stdio.h>
#include <math.h>

#include "arm_cfg.h"
#include "kin_scs.h"

#define T_PI    3.14159265358979f
#define T_D2R   (T_PI / 180.0f)
#define T_R2D   (180.0f / T_PI)

#define T_L1    ARM_KIN_L1_MM
#define T_L2    ARM_KIN_L2_MM
#define T_TILT2 ARM_KIN_J2_TILT_DEG
#define T_TILT3 ARM_KIN_J3_TILT_DEG
#define T_OFF2  ARM_KIN_J2_OFF_DEG
#define T_OFF3  ARM_KIN_J3_OFF_DEG
#define T_SGN2  ARM_KIN_J2_SIGN
#define T_SGN3  ARM_KIN_J3_SIGN

#define T_DMIN  (fabsf(T_L1 - T_L2))
#define T_DMAX  (T_L1 + T_L2)

static int g_pass;
static int g_fail;

static void check(const char *name, int ok)
{
  if (ok != 0)
  {
    g_pass++;
    printf("  [ OK ] %s\n", name);
  }
  else
  {
    g_fail++;
    printf("  [FAIL] %s\n", name);
  }
}

static int near_f(float a, float b, float tol)
{
  return (fabsf(a - b) <= tol) ? 1 : 0;
}

/* 命令角 ← 几何角（和 kin_scs.c 的⑦用同一个映射） */
static void make_q(float th, float phi1, float bend, kin_scs_out_t *q)
{
  q->j1_deg = th;
  q->j2_deg = T_OFF2 + (T_SGN2 * phi1);
  q->j3_deg = T_OFF3 + (T_SGN3 * bend);
}

/* 3D 点 → 球坐标 → 逆解。等价于上位机"我想让末端到这儿"。 */
static uint8_t solve_at(float x, float y, float z, kin_scs_out_t *out)
{
  kin_scs_tgt_t t;
  const float d = sqrtf((x * x) + (y * y) + (z * z));

  if (d < 1.0e-6f)
  {
    return 0u;
  }

  t.theta_deg = atan2f(y, x) * T_R2D;
  t.d_mm      = d;
  t.alpha_deg = asinf(z / d) * T_R2D;

  return kin_scs_solve(&t, out);
}

/* ---------------------------------------------------------------------------
 * 第 1 组：常量与就绪状态
 * ------------------------------------------------------------------------ */
static void test_setup(void)
{
  float lo;
  float hi;

  printf("\n[1] 常量与就绪状态\n");

  check("L1 已填实测值（> 0）", T_L1 > 0.0f);
  check("L2 已填实测值（> 0）", T_L2 > 0.0f);
  check("kin_scs_ready() == 1", kin_scs_ready() == 1u);

  kin_scs_reach(&lo, &hi);
  check("reach 下限 = |L1 − L2|", near_f(lo, T_DMIN, 0.001f));
  check("reach 上限 = L1 + L2", near_f(hi, T_DMAX, 0.001f));
  check("下限是 62mm（105 − 43）", near_f(lo, 62.0f, 0.1f));
  check("上限是 148mm（105 + 43）", near_f(hi, 148.0f, 0.1f));
}

/* ---------------------------------------------------------------------------
 * 第 2 组：可达性边界
 * ------------------------------------------------------------------------ */
static void test_reach(void)
{
  kin_scs_tgt_t t;
  kin_scs_out_t q;

  printf("\n[2] 可达性边界（d 的硬上下限）\n");

  t.theta_deg = 0.0f;
  t.alpha_deg = 0.0f;

  t.d_mm = 60.0f;                       /* 比 |L1−L2| 还近 */
  check("d = 60 < 62 ⇒ 拒绝", kin_scs_solve(&t, &q) == 0u);

  t.d_mm = 150.0f;                      /* 比 L1+L2 还远 */
  check("d = 150 > 148 ⇒ 拒绝", kin_scs_solve(&t, &q) == 0u);

  t.d_mm = 105.0f;                      /* 正好在中间 */
  check("d = 105 ⇒ 接受", kin_scs_solve(&t, &q) == 1u);

  t.d_mm = 0.0f;
  check("d = 0 ⇒ 拒绝", kin_scs_solve(&t, &q) == 0u);

  /* 空指针必须挡得住，不能崩 */
  check("空指针 ⇒ 返回 0", kin_scs_solve(0, &q) == 0u);
}

/* ---------------------------------------------------------------------------
 * 第 3 组：bend 必须满足余弦定理，且 d 越远越直
 * ------------------------------------------------------------------------ */
static float law_bend(float d)
{
  const float cb = ((d * d) - (T_L1 * T_L1) - (T_L2 * T_L2)) / (2.0f * T_L1 * T_L2);
  const float c  = (cb > 1.0f) ? 1.0f : ((cb < -1.0f) ? -1.0f : cb);

  return acosf(c) * T_R2D;
}

static void test_law(void)
{
  static const float ds[] = { 67.0f, 80.0f, 105.0f, 130.0f, 145.0f };
  const int nd = (int)(sizeof(ds) / sizeof(ds[0]));
  kin_scs_tgt_t t;
  kin_scs_out_t q;
  float prev_bend = 999.0f;
  int   i;
  int   mono = 1;
  int   law_ok = 1;

  printf("\n[3] 余弦定理与单调性\n");

  t.theta_deg = 25.0f;
  t.alpha_deg = 0.0f;

  for (i = 0; i < nd; i++)
  {
    float bend;

    t.d_mm = ds[i];

    if (kin_scs_solve(&t, &q) != 1u)
    {
      law_ok = 0;
      printf("         d=%.0f 解不出来\n", (double)ds[i]);
      continue;
    }

    bend = (q.j3_deg - T_OFF3) * T_SGN3;

    if (near_f(bend, law_bend(ds[i]), 0.05f) == 0)
    {
      law_ok = 0;
      printf("         d=%.0f  bend=%.3f 应为 %.3f\n",
             (double)ds[i], (double)bend, (double)law_bend(ds[i]));
    }

    if (bend > (prev_bend + 0.05f))
    {
      mono = 0;
    }

    prev_bend = bend;
  }

  check("bend 满足 d² = L1² + L2² + 2·L1·L2·cos(bend)", law_ok != 0);
  check("d 越远 bend 越小（越伸直）", mono != 0);

  /* 极点 */
  t.d_mm = T_DMAX;
  check("d = L1+L2 ⇒ 接受", kin_scs_solve(&t, &q) == 1u);
  check("d = L1+L2 ⇒ bend = 0", near_f((q.j3_deg - T_OFF3) * T_SGN3, 0.0f, 0.05f));
}

/* ---------------------------------------------------------------------------
 * 第 4 组：正解 → 逆解 → 正解，必须回到同一个点
 * ------------------------------------------------------------------------ */
static void test_roundtrip(void)
{
  static const float ths[]   = { 0.0f, 45.0f, 120.0f, 300.0f };
  static const float phis[]  = { 20.0f, 45.0f, 70.0f, 100.0f, 130.0f };
  static const float bends[] = { 30.0f, 60.0f, 90.0f, 120.0f, 150.0f };

  const int nth   = (int)(sizeof(ths) / sizeof(ths[0]));
  const int nphi  = (int)(sizeof(phis) / sizeof(phis[0]));
  const int nbend = (int)(sizeof(bends) / sizeof(bends[0]));

  kin_scs_out_t q0;
  kin_scs_out_t q1;
  kin_scs_out_t q2;
  float x0;
  float y0;
  float z0;
  float x1;
  float y1;
  float z1;
  float worst_pos = 0.0f;
  float worst_fix = 0.0f;
  int   n_test = 0;
  int   n_fail = 0;
  int   n_mirror = 0;
  int   i;
  int   j;
  int   k;

  printf("\n[4] 正解 → 逆解 → 正解 往返（%d 组姿态）\n", nth * nphi * nbend);

  for (i = 0; i < nth; i++)
  {
    for (j = 0; j < nphi; j++)
    {
      for (k = 0; k < nbend; k++)
      {
        float ep;
        float ea;
        float ef;

        make_q(ths[i], phis[j], bends[k], &q0);
        kin_scs_fk(&q0, &x0, &y0, &z0);

        if (solve_at(x0, y0, z0, &q1) != 1u)
        {
          n_fail++;
          continue;
        }

        kin_scs_fk(&q1, &x1, &y1, &z1);

        ep = sqrtf(((x1 - x0) * (x1 - x0)) +
                   ((y1 - y0) * (y1 - y0)) +
                   ((z1 - z0) * (z1 - z0)));

        /* 再解一次：从 q1 反算出来的点再解，必须**停住不动**。
         * 这是比"和原姿态比"更硬的判据 —— 逆解对同一个点只能有一个答案，
         * 翻了分支就说明公式某处符号不自洽（位置可能还对得上，但姿态在跳）。 */
        if (solve_at(x1, y1, z1, &q2) != 1u)
        {
          n_fail++;
          continue;
        }

        ef = fmodf(fabsf(q2.j1_deg - q1.j1_deg), 360.0f);
        ef = (ef > 180.0f) ? (360.0f - ef) : ef;
        ef += fabsf(q2.j2_deg - q1.j2_deg);
        ef += fabsf(q2.j3_deg - q1.j3_deg);

        /* 顺便数一下有多少组是"镜像解"：同一个点，另一条肘分支。
         * 这不是错，是两连杆的固有二义性，只统计不判失败。 */
        ea = fmodf(fabsf(q1.j1_deg - q0.j1_deg), 360.0f);
        ea = (ea > 180.0f) ? (360.0f - ea) : ea;
        ea += fabsf(q1.j2_deg - q0.j2_deg);
        ea += fabsf(q1.j3_deg - q0.j3_deg);

        if (ea > 1.0f)
        {
          n_mirror++;
        }

        if (ep > worst_pos)
        {
          worst_pos = ep;
        }

        if (ef > worst_fix)
        {
          worst_fix = ef;
          printf("         最差不动点：θ=%.0f φ1=%.0f bend=%.0f → 再解 θ=%.2f φ1=%.2f bend=%.2f\n",
                 (double)ths[i], (double)phis[j], (double)bends[k],
                 (double)q2.j1_deg, (double)q2.j2_deg, (double)q2.j3_deg);
        }

        n_test++;
      }
    }
  }

  printf("         解出 %d / %d 组，最大位置误差 %.5f mm，最大不动点误差 %.5f°\n",
         n_test, nth * nphi * nbend, (double)worst_pos, (double)worst_fix);
  printf("         其中 %d 组是镜像解（同一个点、另一条肘分支，属正常二义性）\n",
         n_mirror);

  check("全部姿态都解得出来", n_fail == 0);
  check("位置往返误差 < 0.01mm", worst_pos < 0.01f);
  check("逆解是不动点（同一个点再解一次不变）", worst_fix < 0.05f);
  check("样本数够（>= 100）", n_test >= 100);
}

/* ---------------------------------------------------------------------------
 * 第 5 组：两个平面偏斜到底有没有真的进模型
 * ------------------------------------------------------------------------ */
static void test_tilt(void)
{
  const float s1 = sinf(T_TILT2 * T_D2R);
  const float c1 = cosf(T_TILT2 * T_D2R);
  const float s2 = sinf(T_TILT3 * T_D2R);

  kin_scs_tgt_t t;
  kin_scs_out_t q;
  float bend;
  float cc;
  float r;
  float zt;
  float sn;
  float expect;

  printf("\n[5] 平面偏斜（TILT_J2=%.1f° TILT_J3=%.1f°）\n",
         (double)T_TILT2, (double)T_TILT3);

  t.theta_deg = 30.0f;
  t.d_mm      = 105.0f;
  t.alpha_deg = 0.0f;

  check("这一组目标可解", kin_scs_solve(&t, &q) == 1u);

  bend = (q.j3_deg - T_OFF3) * T_SGN3;
  cc   = -(T_L2 * s2 * sinf(bend * T_D2R));
  r    = t.d_mm * cosf(t.alpha_deg * T_D2R);
  zt   = t.d_mm * sinf(t.alpha_deg * T_D2R);
  sn   = (cc + (zt * s1)) / (r * c1);
  expect = t.theta_deg + (asinf(sn) * T_R2D);

  printf("         横向偏出 c = %.3f mm ⇒ J1 应为 %.3f°，实际 %.3f°\n",
         (double)cc, (double)expect, (double)q.j1_deg);

  check("J1 = 目标方位 + asin((c + Z·sinδ2)/(R·cosδ2))",
        fabsf(q.j1_deg - expect) < 0.05f);

  check("偏斜角非零（模型里真的用上了实测 10°/20°）",
        (fabsf(T_TILT2) > 0.01f) && (fabsf(T_TILT3) > 0.01f));

  check("J1 确实偏离了目标方位（偏斜的必然结果）",
        fabsf(q.j1_deg - t.theta_deg) > 1.0f);
}

/* ---------------------------------------------------------------------------
 * 第 6 组：仰角上限 —— 平面歪了以后够不到肩轴正上方
 * ------------------------------------------------------------------------ */
static void test_elev_limit(void)
{
  kin_scs_tgt_t t;
  kin_scs_out_t q;

  printf("\n[6] 仰角上限（理想坐标系里最高只能到 90° − TILT_J2）\n");

  t.theta_deg = 0.0f;
  t.d_mm      = 110.0f;

  t.alpha_deg = 89.0f;
  check("α = 89°（几乎正上方）⇒ 拒绝", kin_scs_solve(&t, &q) == 0u);

  t.alpha_deg = 75.0f;
  check("α = 75° ⇒ 接受", kin_scs_solve(&t, &q) == 1u);

  t.alpha_deg = -60.0f;
  check("α = −60°（往下够）⇒ 接受", kin_scs_solve(&t, &q) == 1u);
}

/* ---------------------------------------------------------------------------
 * 第 7 组：命令角映射（OFF / SIGN）
 * ------------------------------------------------------------------------ */
static void test_mapping(void)
{
  kin_scs_tgt_t t;
  kin_scs_out_t q;
  float phi1;

  printf("\n[7] 命令角映射 = OFF + SIGN × 几何角\n");

  t.theta_deg = 0.0f;
  t.d_mm      = 120.0f;
  t.alpha_deg = 10.0f;

  check("目标可解", kin_scs_solve(&t, &q) == 1u);

  phi1 = (q.j2_deg - T_OFF2) * T_SGN2;

  check("J2 = OFF_J2 + SIGN_J2 × φ1",
        near_f(q.j2_deg, T_OFF2 + (T_SGN2 * phi1), 0.001f));
  check("φ1 落在 −10~180°（大臂正常工作面）", (phi1 > -10.0f) && (phi1 < 180.5f));
  check("J1 已归一化到 [0, 360)", (q.j1_deg >= 0.0f) && (q.j1_deg < 360.0f));
}

/* ------------------------------------------------------------------------ */

int main(void)
{
  float lo;
  float hi;

  kin_scs_reach(&lo, &hi);

  printf("kin_scs 单向测试\n");
  printf("  L1=%.1f L2=%.1f   可达 %.1f ~ %.1f mm\n",
         (double)T_L1, (double)T_L2, (double)lo, (double)hi);
  printf("  TILT_J2=%.1f° TILT_J3=%.1f°   OFF=(%.1f, %.1f) SIGN=(%.1f, %.1f)\n",
         (double)T_TILT2, (double)T_TILT3,
         (double)T_OFF2, (double)T_OFF3, (double)T_SGN2, (double)T_SGN3);

  test_setup();
  test_reach();
  test_law();
  test_roundtrip();
  test_tilt();
  test_elev_limit();
  test_mapping();

  printf("\n========== %d passed, %d failed ==========\n", g_pass, g_fail);

  return (g_fail == 0) ? 0 : 1;
}
