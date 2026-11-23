/**
  ******************************************************************************
  * @file    ccs_selftest.c
  * @brief   kin_ccs.c（笛卡尔逆解）的 PC 端单元测试 + 与 kin_scs.c 的对照
  ******************************************************************************
  * @note    不需要板子、不需要舵机，直接 PC 上编：
  *
  *            gcc -std=c11 -Wall -Wextra -Wpedantic -O2 -ffp-contract=off ^
  *              -I Core/Inc ^
  *              -DARM_KIN_L1_MM=150.0f -DARM_KIN_L2_MM=120.0f ^
  *              -DARM_KIN_J2_OFF_DEG=90.0f -DARM_KIN_J3_OFF_DEG=0.0f ^
  *              tests\ccs_selftest.c Core\Src\kin_ccs.c Core\Src\kin_scs.c ^
  *              -o tests\ccs_selftest.exe -lm
  *
  *          ★ 这个文件是**测试**，可以同时 include 两个库；
  *            两个库本身（kin_scs.c / kin_ccs.c）是互不相干的。
  ******************************************************************************
  * @note    **对照的意义**
  *
  *          同一副手臂、同一组几何参数，喂同一个末端点：
  *            - SCS 走 (θ, d, α)
  *            - CCS 走 (x, y, z)，自己先转成球坐标
  *          两边解出来的 J1/J2/J3 必须一致。这验证了"CCS = SCS + 4 步换算"
  *          这句话，也说明 SCS 那套没什么魔法。
  ******************************************************************************
  */

#include "kin_ccs.h"
#include "kin_scs.h"
#include "arm_cfg.h"

#include <math.h>
#include <stdio.h>

#define PI_F        3.14159265358979f
#define D2R         (PI_F / 180.0f)
#define R2D         (180.0f / PI_F)

static int g_pass;
static int g_fail;

static void check(int cond, const char *what)
{
  if (cond)
  {
    g_pass++;
    printf("  [ok]   %s\n", what);
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

  return d <= tol;
}

/* 用 arm_cfg.h 里那同一组宏装配 CCS 的参数 —— 保证两个库拿到的几何完全一样 */
static void ccs_cfg_from_macros(kin_ccs_cfg_t *c)
{
  c->l1_mm     = ARM_KIN_L1_MM;
  c->l2_mm     = ARM_KIN_L2_MM;
  c->j2_off_deg = ARM_KIN_J2_OFF_DEG;
  c->j2_sign    = ARM_KIN_J2_SIGN;
  c->j3_off_deg = ARM_KIN_J3_OFF_DEG;
  c->j3_sign    = ARM_KIN_J3_SIGN;
  c->j2_sag_deg = ARM_KIN_J2_SAG_DEG;
  c->j3_sag_deg = ARM_KIN_J3_SAG_DEG;
}

/* 可达斜距区间（L1=43 / L2=105 ⇒ 62 ~ 148mm），测试点按比例取，
 * 这样换几何参数不用改测试 */
#define CCS_DMIN  (fabsf(ARM_KIN_L1_MM - ARM_KIN_L2_MM))
#define CCS_DMAX  (ARM_KIN_L1_MM + ARM_KIN_L2_MM)

static float d_at(float frac)
{
  return CCS_DMIN + (frac * (CCS_DMAX - CCS_DMIN));
}

/* ------------------------------------------------------------------------- */
/* 1. setup / ready / reach                                                   */
/* ------------------------------------------------------------------------- */
static void test_setup(void)
{
  kin_ccs_cfg_t c;
  float dmin = 0.0f;
  float dmax = 0.0f;

  printf("\n[1] setup / ready / reach\n");

  check(kin_ccs_setup((const kin_ccs_cfg_t *)0) == 0u, "cfg = NULL ⇒ setup 失败");

  ccs_cfg_from_macros(&c);
  c.l1_mm = 0.0f;
  check(kin_ccs_setup(&c) == 0u, "L1 = 0 ⇒ setup 失败");
  check(kin_ccs_ready() == 0u, "setup 失败后 ready 必须回到 0");

  ccs_cfg_from_macros(&c);
  c.j2_sign = 0.5f;
  check(kin_ccs_setup(&c) == 0u, "SIGN = 0.5（不是 ±1）⇒ setup 失败");

  ccs_cfg_from_macros(&c);
  check(kin_ccs_setup(&c) == 1u, "参数合法 ⇒ setup 成功");
  check(kin_ccs_ready() == 1u, "setup 成功后 ready = 1");

  kin_ccs_reach(&dmin, &dmax);
  check(near_f(dmin, fabsf(c.l1_mm - c.l2_mm), 0.001f), "d_min = |L1−L2|");
  check(near_f(dmax, c.l1_mm + c.l2_mm, 0.001f), "d_max = L1+L2");

  /* 非法 setup 之后必须真的解不出来（不能还留着上一轮的参数在用） */
  ccs_cfg_from_macros(&c);
  c.l2_mm = -1.0f;
  (void)kin_ccs_setup(&c);
  {
    kin_ccs_out_t q;
    check(kin_ccs_solve(100.0f, 0.0f, 50.0f, &q) == 0u,
          "setup 失败后 solve 一律拒绝");
  }

  ccs_cfg_from_macros(&c);
  (void)kin_ccs_setup(&c);
}

/* ------------------------------------------------------------------------- */
/* 2. 正解 / 逆解往返                                                          */
/* ------------------------------------------------------------------------- */
static void test_roundtrip(void)
{
  static const float thetas[] = { 0.0f, 30.0f, 90.0f, 137.5f, 200.0f, 275.0f, 359.0f };
  static const float df[]     = { 0.05f, 0.30f, 0.50f, 0.72f, 0.95f };  /* d = d_at(f) */
  static const float alphas[] = { -60.0f, -20.0f, 0.0f, 25.0f, 60.0f };

  int nt = (int)(sizeof(thetas) / sizeof(thetas[0]));
  int nd = (int)(sizeof(df) / sizeof(df[0]));
  int na = (int)(sizeof(alphas) / sizeof(alphas[0]));
  int i;
  int j;
  int k;
  int solved = 0;
  int rejected = 0;
  float worst = 0.0f;

  printf("\n[2] solve → fk 往返（%d 个点）\n", nt * nd * na);

  for (i = 0; i < nt; i++)
  {
    for (j = 0; j < nd; j++)
    {
      for (k = 0; k < na; k++)
      {
        const float th = thetas[i] * D2R;
        const float r0 = d_at(df[j]) * cosf(alphas[k] * D2R);
        const float z0 = d_at(df[j]) * sinf(alphas[k] * D2R);
        const float x0 = r0 * cosf(th);
        const float y0 = r0 * sinf(th);

        kin_ccs_out_t q;
        float x1 = 0.0f;
        float y1 = 0.0f;
        float z1 = 0.0f;
        float e;

        if (kin_ccs_solve(x0, y0, z0, &q) == 0u)
        {
          rejected++;
          continue;
        }

        kin_ccs_fk(&q, &x1, &y1, &z1);

        e = sqrtf(((x1 - x0) * (x1 - x0)) +
                  ((y1 - y0) * (y1 - y0)) +
                  ((z1 - z0) * (z1 - z0)));

        if (e > worst)
        {
          worst = e;
        }

        solved++;
      }
    }
  }

  printf("       解出 %d 个，拒绝 %d 个，最大往返误差 %.4f mm\n",
         solved, rejected, (double)worst);

  check(solved == (nt * nd * na), "全部点都解出来了（没有误拒）");
  check(worst < 0.5f, "往返误差 < 0.5 mm");
}

/* ------------------------------------------------------------------------- */
/* 3. 该拒绝的必须拒绝                                                         */
/* ------------------------------------------------------------------------- */
static void test_reject(void)
{
  kin_ccs_out_t q;
  const float l1 = ARM_KIN_L1_MM;
  const float l2 = ARM_KIN_L2_MM;

  printf("\n[3] 非法输入\n");

  check(kin_ccs_solve(0.0f, 0.0f, 0.0f, &q) == 0u, "目标在原点 ⇒ 拒绝");
  check(kin_ccs_solve(l1 + l2 + 30.0f, 0.0f, 0.0f, &q) == 0u,
        "超出 L1+L2 ⇒ 拒绝");
  check(kin_ccs_solve(fabsf(l1 - l2) - 20.0f, 0.0f, 0.0f, &q) == 0u,
        "小于 |L1−L2| ⇒ 拒绝");
  check(kin_ccs_solve(NAN, 0.0f, 50.0f, &q) == 0u, "x = NaN ⇒ 拒绝");
  check(kin_ccs_solve(100.0f, 0.0f, 50.0f, (kin_ccs_out_t *)0) == 0u,
        "输出指针为 NULL ⇒ 拒绝");
  check(kin_ccs_solve(1.0e9f, 0.0f, 0.0f, &q) == 0u, "野值 1e9 ⇒ 拒绝");
}

/* ------------------------------------------------------------------------- */
/* 4. 角度归一化 / elbow-up                                                    */
/* ------------------------------------------------------------------------- */
static void test_angle(void)
{
  kin_ccs_out_t q;

  printf("\n[4] 方位角归一化 + elbow-up\n");

  /* (x < 0, y = 0) ⇒ θ = 180° */
  if (kin_ccs_solve(-100.0f, 0.0f, 50.0f, &q) != 0u)
  {
    check(near_f(q.j1_deg, 180.0f, 0.01f), "x < 0, y = 0 ⇒ θ = 180");
  }
  else
  {
    check(0, "x < 0, y = 0 ⇒ θ = 180（解算失败）");
  }

  /* (x = 0, y < 0) ⇒ θ = 270 */
  if (kin_ccs_solve(0.0f, -100.0f, 50.0f, &q) != 0u)
  {
    check(near_f(q.j1_deg, 270.0f, 0.01f), "x = 0, y < 0 ⇒ θ = 270");
  }
  else
  {
    check(0, "x = 0, y < 0 ⇒ θ = 270（解算失败）");
  }

  /* θ 必须落在 [0, 360)，不能出负的 */
  if (kin_ccs_solve(100.0f, -0.001f, 50.0f, &q) != 0u)
  {
    check((q.j1_deg >= 0.0f) && (q.j1_deg < 360.0f), "j1 恒在 [0, 360)");
  }
  else
  {
    check(0, "j1 恒在 [0, 360)（解算失败）");
  }

  /* elbow-up：同一点和 elbow-down 比，大臂的仰角应该更大 */
  {
    kin_scs_tgt_t tgt;
    kin_scs_out_t up;
    float bend;
    float phi1;

    tgt.theta_deg = 0.0f;
    tgt.d_mm      = d_at(0.5f);
    tgt.alpha_deg = 0.0f;

    if (kin_scs_solve(&tgt, &up) != 0u)
    {
      /* 命令角 → **几何**角：先按 SIGN 反变换（±1，1/SIGN == SIGN），
       * 再把④里补的下垂量扣回去 —— 这一步不能省，否则比的是"命令角" */
      phi1 = ((up.j2_deg - ARM_KIN_J2_OFF_DEG) * ARM_KIN_J2_SIGN) - ARM_KIN_J2_SAG_DEG;
      bend = ((up.j3_deg - ARM_KIN_J3_OFF_DEG) * ARM_KIN_J3_SIGN) + ARM_KIN_J3_SAG_DEG;
      check(phi1 > tgt.alpha_deg, "elbow-up：φ1 > α（大臂抬到 OP 线上方）");
      check((bend >= 0.0f) && (bend <= 180.0f), "bend 在 [0, 180]");
    }
    else
    {
      check(0, "elbow-up 判据（SCS 解算失败）");
    }
  }
}

/* ------------------------------------------------------------------------- */
/* 5. 和 SCS 逐点对照                                                          */
/* ------------------------------------------------------------------------- */
static void test_compare(void)
{
  static const float thetas[] = { 0.0f, 30.0f, 90.0f, 137.5f, 200.0f, 275.0f, 359.0f };
  static const float df[]     = { 0.05f, 0.30f, 0.50f, 0.72f, 0.95f };  /* d = d_at(f) */
  static const float alphas[] = { -60.0f, -20.0f, 0.0f, 25.0f, 60.0f };

  int nt = (int)(sizeof(thetas) / sizeof(thetas[0]));
  int nd = (int)(sizeof(df) / sizeof(df[0]));
  int na = (int)(sizeof(alphas) / sizeof(alphas[0]));
  int i;
  int j;
  int k;
  int pairs = 0;
  int mismatch = 0;
  float worst = 0.0f;
  int printed = 0;

  printf("\n[5] SCS vs CCS 逐点对照（%d 个点）\n", nt * nd * na);
  printf("       θ       d       α |        SCS J1/J2/J3      |        CCS J1/J2/J3      |  Δmax\n");
  printf("     ----------------------------------------------------------------------------------\n");

  for (i = 0; i < nt; i++)
  {
    for (j = 0; j < nd; j++)
    {
      for (k = 0; k < na; k++)
      {
        kin_scs_tgt_t tgt;
        kin_scs_out_t p;
        kin_ccs_out_t q;
        float r;
        float z;
        float th;
        float e;

        tgt.theta_deg = thetas[i];
        tgt.d_mm      = d_at(df[j]);
        tgt.alpha_deg = alphas[k];

        if (kin_scs_solve(&tgt, &p) == 0u)
        {
          continue;
        }

        /* SCS 的正解是**平面**的 (r, z)，要自己转成 3D 才能喂给 CCS */
        kin_scs_fk(&p, &r, &z);
        th = p.j1_deg * D2R;

        if (kin_ccs_solve(r * cosf(th), r * sinf(th), z, &q) == 0u)
        {
          continue;
        }

        e = fabsf(q.j2_deg - p.j2_deg);
        if (fabsf(q.j3_deg - p.j3_deg) > e)
        {
          e = fabsf(q.j3_deg - p.j3_deg);
        }
        if (fabsf(q.j1_deg - p.j1_deg) > e)
        {
          e = fabsf(q.j1_deg - p.j1_deg);
        }

        pairs++;
        if (e > worst)
        {
          worst = e;
        }
        if (e > 0.01f)
        {
          mismatch++;
        }

        /* 只打前 8 行，别把屏幕刷满 */
        if (printed < 8)
        {
          printf("   %6.1f  %6.1f  %6.1f | %7.3f %7.3f %7.3f | %7.3f %7.3f %7.3f | %.2e\n",
                 (double)tgt.theta_deg, (double)tgt.d_mm, (double)tgt.alpha_deg,
                 (double)p.j1_deg, (double)p.j2_deg, (double)p.j3_deg,
                 (double)q.j1_deg, (double)q.j2_deg, (double)q.j3_deg,
                 (double)e);
          printed++;
        }
      }
    }
  }

  printf("     ----------------------------------------------------------------------------------\n");
  printf("       对照 %d 点，最大偏差 %.6f°，不一致(>0.01°) %d 个\n",
         pairs, (double)worst, mismatch);

  check(pairs == (nt * nd * na), "SCS / CCS 都能解（同一批点没人掉队）");
  check(worst < 0.01f, "两套解算结果一致（偏差 < 0.01°）");
}

/* ------------------------------------------------------------------------- */
int main(void)
{
  kin_ccs_cfg_t c;

  ccs_cfg_from_macros(&c);
  (void)kin_ccs_setup(&c);

  printf("=== kin_ccs 单元测试 + kin_scs 对照 ===\n");
  printf(" L1=%.1f L2=%.1f OFF_J2=%.1f SIGN_J2=%.0f OFF_J3=%.1f SIGN_J3=%.0f\n",
         (double)c.l1_mm, (double)c.l2_mm,
         (double)c.j2_off_deg, (double)c.j2_sign,
         (double)c.j3_off_deg, (double)c.j3_sign);

  test_setup();
  test_roundtrip();
  test_reject();
  test_angle();
  test_compare();

  printf("\n=== %d passed, %d failed ===\n", g_pass, g_fail);

  return (g_fail == 0) ? 0 : 1;
}
