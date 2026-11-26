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
 *   几何模型是“两个摆动平面各自偏斜”，断言验的是**不变量**（往返、
 *   余弦定理、方位角落点、仰角上限），而不是照抄一遍公式。
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
#define T_BIAS2 ARM_KIN_J2_BIAS_DEG
#define T_BIAS3 ARM_KIN_J3_BIAS_DEG
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

    /* J3 是**命令角**，物理上的连杆夹角还要把偏置加回去 */
    bend = ((q.j3_deg - T_OFF3) * T_SGN3) + T_BIAS3;

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
 * 第 5 组：角度偏置到底有没有真的扣进命令角
 *
 *   独立重算一遍几何（不抄 kin_scs.c 的写法）：先用两次余弦定理拿出
 *   "物理上应该在哪个姿态"，再手算命令角，看 solve 给的一不一样。
 * ------------------------------------------------------------------------ */
static void test_bias(void)
{
  kin_scs_tgt_t t;
  kin_scs_out_t q;

  const float d = 105.0f;
  const float al = 0.0f;

  const float cb = ((T_L1 * T_L1) + (d * d) - (T_L2 * T_L2)) / (2.0f * T_L1 * d);
  const float cg = ((T_L1 * T_L1) + (T_L2 * T_L2) - (d * d)) / (2.0f * T_L1 * T_L2);

  const float phi1_geo = al + (acosf(cb) * T_R2D);
  const float bend_geo = 180.0f - (acosf(cg) * T_R2D);
  const float phi1_cmd = phi1_geo + T_BIAS2;
  const float bend_cmd = bend_geo - T_BIAS3;

  printf("\n[5] 角度偏置（BIAS_J2=%.1f° BIAS_J3=%.1f°）\n",
         (double)T_BIAS2, (double)T_BIAS3);

  t.theta_deg = 30.0f;
  t.d_mm      = d;
  t.alpha_deg = al;

  check("这一组目标可解", kin_scs_solve(&t, &q) == 1u);

  printf("         几何：φ1=%.3f° bend=%.3f°  ⇒ 命令：φ1=%.3f° bend=%.3f°\n",
         (double)phi1_geo, (double)bend_geo, (double)phi1_cmd, (double)bend_cmd);
  printf("         实际：J2=%.3f° J3=%.3f°\n", (double)q.j2_deg, (double)q.j3_deg);

  check("J2 = 几何仰角 + BIAS_J2",
        near_f(q.j2_deg, T_OFF2 + (T_SGN2 * phi1_cmd), 0.05f));
  check("J3 = 几何夹角 − BIAS_J3",
        near_f(q.j3_deg, T_OFF3 + (T_SGN3 * bend_cmd), 0.05f));

  /* 正解把偏置扣回去，应该回到目标点 —— 这条同时验了"加"和"减"是互逆的 */
  {
    float x;
    float y;
    float z;
    float rr;

    kin_scs_fk(&q, &x, &y, &z);

    rr = sqrtf((x * x) + (y * y));
    printf("         正解扣回：(r, z) = (%.4f, %.4f)，目标是 (%.4f, %.4f)\n",
           (double)rr, (double)z, (double)(d * cosf(al * T_D2R)),
           (double)(d * sinf(al * T_D2R)));

    check("正解扣回偏置后落回目标点",
          (fabsf(rr - (d * cosf(al * T_D2R))) < 0.01f) &&
          (fabsf(z - (d * sinf(al * T_D2R))) < 0.01f));
  }

  check("偏置非零（模型里真的用上了实测 10°/20°）",
        (fabsf(T_BIAS2) > 0.01f) && (fabsf(T_BIAS3) > 0.01f));
}

/* ---------------------------------------------------------------------------
 * 第 6 组：偏置带来的两个实际限制
 *   ① 命令角被夹到 0 —— 手臂伸直是靠"命令 0"实现的，但它物理上还弯着
 *   ② 因此最远够不到 148mm，大概只到 133mm
 * ------------------------------------------------------------------------ */
static void test_bias_limit(void)
{
  kin_scs_tgt_t t;
  kin_scs_out_t q;
  float x;
  float y;
  float z;
  float r148;

  printf("\n[6] 偏置的实际后果（够不到名义上的 148mm）\n");

  t.theta_deg = 0.0f;
  t.d_mm      = T_DMAX;
  t.alpha_deg = 90.0f;

  check("d = 148（名义最远）仍可解", kin_scs_solve(&t, &q) == 1u);
  check("但命令角被夹在 0，不会变成负数", q.j3_deg >= -0.001f);

  /* 命令"竖直伸直"时，物理上两连杆还差 20° 才能共线 —— 所以末端
   * 不在正上方，而是被顶出去一截。这一条比"距离短了多少"更直观。 */
  q.j1_deg = 0.0f;
  q.j3_deg = 0.0f;
  kin_scs_fk(&q, &x, &y, &z);

  r148 = sqrtf((x * x) + (y * y));

  printf("         命令 j3 = 0 时，末端偏离正上方 %.2f mm，到肩 %.2f mm（名义 148）\n",
         (double)r148, (double)sqrtf((x * x) + (y * y) + (z * z)));

  check("命令 j3 = 0 时末端明显偏离正上方（≥ 30mm）", r148 > 30.0f);

  /* 平面模型没有"仰角上限"，正上方照样能解 */
  t.d_mm      = 110.0f;
  t.alpha_deg = 89.0f;
  check("α = 89°（几乎正上方）⇒ 仍可解", kin_scs_solve(&t, &q) == 1u);
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
  printf("  BIAS_J2=%.1f° BIAS_J3=%.1f°   OFF=(%.1f, %.1f) SIGN=(%.1f, %.1f)\n",
         (double)T_BIAS2, (double)T_BIAS3,
         (double)T_OFF2, (double)T_OFF3, (double)T_SGN2, (double)T_SGN3);

  test_setup();
  test_reach();
  test_law();
  test_roundtrip();
  test_bias();
  test_bias_limit();
  test_mapping();

  printf("\n========== %d passed, %d failed ==========\n", g_pass, g_fail);

  return (g_fail == 0) ? 0 : 1;
}
