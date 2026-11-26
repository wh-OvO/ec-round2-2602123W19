/* ===========================================================================
 * kin_ccs（笛卡尔逆解）的 PC 单测 + 与 kin_scs（球坐标）的对照
 *
 *   gcc -std=c11 -Wall -Wextra -Wpedantic -O2 -ffp-contract=off \
 *       -I ../Core/Inc ccs_selftest.c ../Core/Src/kin_ccs.c \
 *       ../Core/Src/kin_scs.c -lm -o ccs_selftest
 *
 *   两个库的定位：kin_scs 是**验收路径**（上位机发 θ/d/α），kin_ccs 是
 *   拿来对比的独立实现（上位机直接发 x/y/z）。既然是同一台手臂的两种
 *   描述，几何模型必须完全一致 —— 所以这里最硬的断言是"同一个目标，
 *   两个库解出来的三个关节角必须一样"。
 *
 *   两个库用的是同一套“两个摆动平面各自偏斜”模型。
 * =========================================================================== */

#include <stdio.h>
#include <math.h>

#include "arm_cfg.h"
#include "kin_ccs.h"
#include "kin_scs.h"

#define T_PI   3.14159265358979f
#define T_D2R  (T_PI / 180.0f)
#define T_R2D  (180.0f / T_PI)

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

/* 角度差，考虑整圈 */
static float ang_diff(float a, float b)
{
  float d = fmodf(fabsf(a - b), 360.0f);

  return (d > 180.0f) ? (360.0f - d) : d;
}

static void cfg_from_macros(kin_ccs_cfg_t *c)
{
  c->l1_mm      = ARM_KIN_L1_MM;
  c->l2_mm      = ARM_KIN_L2_MM;
  c->j2_off_deg = ARM_KIN_J2_OFF_DEG;
  c->j2_sign    = ARM_KIN_J2_SIGN;
  c->j3_off_deg = ARM_KIN_J3_OFF_DEG;
  c->j3_sign    = ARM_KIN_J3_SIGN;
  c->j2_bias_deg = ARM_KIN_J2_BIAS_DEG;
  c->j3_bias_deg = ARM_KIN_J3_BIAS_DEG;
}

/* ---------------------------------------------------------------------------
 * 第 1 组：setup 的参数校验
 * ------------------------------------------------------------------------ */
static void test_setup(void)
{
  kin_ccs_cfg_t c;

  printf("\n[1] setup 参数校验\n");

  cfg_from_macros(&c);
  check("正常参数 ⇒ setup 成功", kin_ccs_setup(&c) == 1u);
  check("setup 之后 ready", kin_ccs_ready() == 1u);

  cfg_from_macros(&c);
  c.l1_mm = 0.0f;
  check("L1 = 0 ⇒ 拒绝", kin_ccs_setup(&c) == 0u);
  check("拒绝之后 ready == 0", kin_ccs_ready() == 0u);

  cfg_from_macros(&c);
  c.j2_sign = 0.0f;
  check("SIGN = 0 ⇒ 拒绝", kin_ccs_setup(&c) == 0u);

  cfg_from_macros(&c);
  check("空指针 ⇒ 拒绝", kin_ccs_setup(0) == 0u);

  /* 恢复成可用状态，后面几组要用 */
  cfg_from_macros(&c);
  (void)kin_ccs_setup(&c);
}

/* ---------------------------------------------------------------------------
 * 第 2 组：和 kin_scs 必须解出同一组关节角
 * ------------------------------------------------------------------------ */
static void test_agree(void)
{
  static const float ths[] = { 0.0f, 40.0f, 120.0f, 250.0f, 330.0f };
  static const float ds[]  = { 70.0f, 90.0f, 110.0f, 130.0f };
  static const float als[] = { -40.0f, -10.0f, 20.0f, 50.0f };

  const int nth = (int)(sizeof(ths) / sizeof(ths[0]));
  const int nd  = (int)(sizeof(ds) / sizeof(ds[0]));
  const int nal = (int)(sizeof(als) / sizeof(als[0]));

  int i;
  int j;
  int k;
  int n = 0;
  int n_bad = 0;
  float worst = 0.0f;

  printf("\n[2] 与 kin_scs 的一致性（%d 组目标）\n", nth * nd * nal);

  for (i = 0; i < nth; i++)
  {
    for (j = 0; j < nd; j++)
    {
      for (k = 0; k < nal; k++)
      {
        const float th = ths[i] * T_D2R;
        const float al = als[k] * T_D2R;
        const float x  = ds[j] * cosf(al) * cosf(th);
        const float y  = ds[j] * cosf(al) * sinf(th);
        const float z  = ds[j] * sinf(al);

        kin_scs_tgt_t t;
        kin_scs_out_t a;
        kin_ccs_out_t b;

        t.theta_deg = ths[i];
        t.d_mm      = ds[j];
        t.alpha_deg = als[k];

        if ((kin_scs_solve(&t, &a) != 1u) || (kin_ccs_solve(x, y, z, &b) != 1u))
        {
          continue;    /* 两个都拒了是正常的（够不到） */
        }

        n++;

        {
          const float e = ang_diff(a.j1_deg, b.j1_deg) +
                          ang_diff(a.j2_deg, b.j2_deg) +
                          ang_diff(a.j3_deg, b.j3_deg);

          if (e > 0.02f)
          {
            n_bad++;
            printf("         θ=%.0f d=%.0f α=%.0f：scs(%.2f,%.2f,%.2f) vs ccs(%.2f,%.2f,%.2f)\n",
                   (double)ths[i], (double)ds[j], (double)als[k],
                   (double)a.j1_deg, (double)a.j2_deg, (double)a.j3_deg,
                   (double)b.j1_deg, (double)b.j2_deg, (double)b.j3_deg);
          }

          if (e > worst)
          {
            worst = e;
          }
        }
      }
    }
  }

  printf("         两边都能解的 %d 组，最大关节角差 %.4f°\n", n, (double)worst);

  check("样本数够（>= 40）", n >= 40);
  check("最大关节角差 < 0.02°", n_bad == 0);
}

/* ---------------------------------------------------------------------------
 * 第 3 组：笛卡尔往返（solve → fk → 回到同一个点）
 * ------------------------------------------------------------------------ */
static void test_roundtrip(void)
{
  static const float pts[][3] = {
    { 100.0f,    0.0f,   0.0f },
    {  70.0f,   70.0f,  30.0f },
    { -50.0f,   80.0f,  50.0f },
    {   0.0f, -110.0f, -20.0f },
    {  60.0f,  -60.0f,  70.0f },
    { -90.0f,  -40.0f,  40.0f },
  };

  const int np = (int)(sizeof(pts) / sizeof(pts[0]));
  int i;
  int n = 0;
  float worst = 0.0f;

  printf("\n[3] 笛卡尔往返\n");

  for (i = 0; i < np; i++)
  {
    kin_ccs_out_t q;
    float x1;
    float y1;
    float z1;

    if (kin_ccs_solve(pts[i][0], pts[i][1], pts[i][2], &q) != 1u)
    {
      printf("         点 (%6.1f,%6.1f,%6.1f) 解不出\n",
             (double)pts[i][0], (double)pts[i][1], (double)pts[i][2]);
      continue;
    }

    kin_ccs_fk(&q, &x1, &y1, &z1);

    {
      const float e = sqrtf(((x1 - pts[i][0]) * (x1 - pts[i][0])) +
                            ((y1 - pts[i][1]) * (y1 - pts[i][1])) +
                            ((z1 - pts[i][2]) * (z1 - pts[i][2])));

      if (e > worst)
      {
        worst = e;
      }

      n++;
    }
  }

  printf("         解出 %d / %d 个点，最大往返误差 %.5f mm\n", n, np, (double)worst);

  check("全部点都能解", n == np);
  check("往返误差 < 0.01mm", worst < 0.01f);
}

/* ---------------------------------------------------------------------------
 * 第 4 组：该拒的要拒
 * ------------------------------------------------------------------------ */
static void test_reject(void)
{
  kin_ccs_out_t q;

  printf("\n[4] 拒绝非法/够不到的目标\n");

  check("原点 ⇒ 拒绝", kin_ccs_solve(0.0f, 0.0f, 0.0f, &q) == 0u);
  check("太远（200mm）⇒ 拒绝", kin_ccs_solve(200.0f, 0.0f, 0.0f, &q) == 0u);
  check("太近（30mm）⇒ 拒绝", kin_ccs_solve(30.0f, 0.0f, 0.0f, &q) == 0u);
  check("空指针 ⇒ 拒绝", kin_ccs_solve(100.0f, 0.0f, 0.0f, 0) == 0u);
  check("器臂正上方 110mm ⇒ 也能解（平面模型没有仰角上限）",
        kin_ccs_solve(0.0f, 0.0f, 110.0f, &q) == 1u);
}

/* ---------------------------------------------------------------------------
 * 第 5 组：两个库的正解也必须一致
 * ------------------------------------------------------------------------ */
static void test_fk_agree(void)
{
  kin_ccs_out_t q;
  kin_scs_out_t s;
  float x1;
  float y1;
  float z1;
  float x2;
  float y2;
  float z2;
  float e;

  printf("\n[5] 两个库的正解一致\n");

  q.j1_deg = 40.0f;
  q.j2_deg = 75.0f;
  q.j3_deg = 100.0f;

  s.j1_deg = q.j1_deg;
  s.j2_deg = q.j2_deg;
  s.j3_deg = q.j3_deg;

  kin_ccs_fk(&q, &x1, &y1, &z1);
  kin_scs_fk(&s, &x2, &y2, &z2);

  e = sqrtf(((x1 - x2) * (x1 - x2)) +
            ((y1 - y2) * (y1 - y2)) +
            ((z1 - z2) * (z1 - z2)));

  printf("         ccs(%.4f, %.4f, %.4f)  scs(%.4f, %.4f, %.4f)  差 %.6f mm\n",
         (double)x1, (double)y1, (double)z1,
         (double)x2, (double)y2, (double)z2, (double)e);

  check("两条正解路径完全重合（< 0.001mm）", e < 0.001f);
}

/* ------------------------------------------------------------------------ */

int main(void)
{
  kin_ccs_cfg_t c;

  cfg_from_macros(&c);

  printf("kin_ccs 单向测试（含与 kin_scs 的对照）\n");
  printf("  L1=%.1f L2=%.1f  BIAS=(%.1f°, %.1f°)\n",
         (double)c.l1_mm, (double)c.l2_mm,
         (double)c.j2_bias_deg, (double)c.j3_bias_deg);

  test_setup();
  test_agree();
  test_roundtrip();
  test_reject();
  test_fk_agree();

  printf("\n========== %d passed, %d failed ==========\n", g_pass, g_fail);

  return (g_fail == 0) ? 0 : 1;
}
