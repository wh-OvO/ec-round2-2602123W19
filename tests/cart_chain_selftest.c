/**
  ******************************************************************************
  * @file    cart_chain_selftest.c
  * @brief   0x34 逆解**整条链**的 PC 单测：笛卡尔 → 球坐标 → 关节角 → 正解
  *
  *          这是三个模块拼起来之后的联合测试：
  *
  *              x,y,z ──[kin_c2s]──> θ,d,α ──[kin_scs_solve]──> J1,J2,J3
  *                                                              │
  *              x,y,z <──────[kin_scs_fk]──────────────────────┘
  *
  *          单模块的测试（c2s_selftest / scs_selftest）证明各自没算错，
  *          但**拼接口处**的约定仍可能错：单位、坐标系原点、j1 的
  *          0~360 / −180~180、夹爪算不算在 L2 里…… 这些只有往返一次才看得出来。
  *          所以判据是"解出来的姿态再正解回去，必须回到原来那个点"。
  *
  *          编译（在 rm_arm 目录下）：
  *            gcc -std=c11 -Wall -Wextra -Wpedantic -O2 -ffp-contract=off
  *                -I Core/Inc tests/cart_chain_selftest.c
  *                Core/Src/kin_c2s.c Core/Src/kin_scs.c -lm
  *                -o tests/cart_chain_selftest.exe
  ******************************************************************************
  */

#include <math.h>
#include <stdio.h>

#include "arm_cfg.h"
#include "kin_c2s.h"
#include "kin_scs.h"

#define T_PI   3.14159265358979f
#define T_D2R  (T_PI / 180.0f)

static int g_pass = 0;
static int g_fail = 0;

static void ck_f(const char *name, float got, float want, float tol)
{
  if (fabsf(got - want) <= tol)
  {
    g_pass++;
  }
  else
  {
    g_fail++;
    printf("  FAIL  %-30s got %10.4f  want %10.4f  (diff %.4f)\n",
           name, (double)got, (double)want, (double)fabsf(got - want));
  }
}

static void ck_u(const char *name, unsigned got, unsigned want)
{
  if (got == want)
  {
    g_pass++;
  }
  else
  {
    g_fail++;
    printf("  FAIL  %-30s got %u  want %u\n", name, got, want);
  }
}

/* 球坐标 → 笛卡尔（测试里自己算一遍，不借用被测代码） */
static void sph_to_xyz(float th, float d, float al, float *x, float *y, float *z)
{
  const float rr = d * cosf(al * T_D2R);

  *x = rr * cosf(th * T_D2R);
  *y = rr * sinf(th * T_D2R);
  *z = d * sinf(al * T_D2R);
}

int main(void)
{
  static const float thetas[] = { -170.0f, -120.0f, -30.0f, 0.0f,
                                   45.0f,   90.0f, 150.0f, 179.0f };
  static const float dists[]  = { 70.0f, 90.0f, 110.0f, 130.0f, 146.0f };
  static const float alphas[] = { -20.0f, 0.0f, 20.0f, 45.0f, 60.0f };

  float d_lo = 0.0f;
  float d_hi = 0.0f;
  int   solved = 0;
  int   rejected = 0;
  size_t i;
  size_t j;
  size_t k;

  printf("=== 0x34 逆解整链单测：xyz → (θ,d,α) → 关节角 → 正解回 xyz ===\n");

  /* ---- ① 可达区间 ---- */
  kin_scs_reach(&d_lo, &d_hi);
  printf("  可达斜距 d = %.2f ~ %.2f mm\n", (double)d_lo, (double)d_hi);
  ck_f("d 下限 = |L1-L2|", d_lo,
       fabsf(ARM_KIN_L1_MM - ARM_KIN_L2_MM), 1e-3f);
  ck_f("d 上限 = L1+L2", d_hi, ARM_KIN_L1_MM + ARM_KIN_L2_MM, 1e-3f);

  /* ---- ② 网格往返：8 × 5 × 5 = 200 个点 ----
   *    每个点走完整条链，解出来的姿态再正解回去必须回到原点位。 */
  for (i = 0u; i < (sizeof(thetas) / sizeof(thetas[0])); i++)
  {
    for (j = 0u; j < (sizeof(dists) / sizeof(dists[0])); j++)
    {
      for (k = 0u; k < (sizeof(alphas) / sizeof(alphas[0])); k++)
      {
        const float th0 = thetas[i];
        const float d0  = dists[j];
        const float al0 = alphas[k];
        float x0, y0, z0;
        float x1, y1, z1;
        float x2, y2, z2;
        float th1 = 0.0f;
        float d1  = 0.0f;
        float al1 = 0.0f;
        float j1w;
        uint8_t st;
        kin_c2s_in_t  in;
        kin_scs_tgt_t tgt;
        kin_scs_out_t sol;

        sph_to_xyz(th0, d0, al0, &x0, &y0, &z0);

        /* 原点已经是"肩轴心"，不需要任何平移 —— 这正是要验的约定之一 */
        in.x_mm = x0;
        in.y_mm = y0;
        in.z_mm = z0;

        st = kin_c2s_deg(&in, 0.0f, &th1, &d1, &al1);
        if (st != KIN_C2S_OK)
        {
          /* 只有正好落在底座轴上才会走到这里，本组点的 θ 都不是 0/180 的
           * 极端组合时不该出现；出现就说明变换有问题。 */
          g_fail++;
          printf("  FAIL  c2s 状态异常 th=%.0f d=%.0f al=%.0f\n",
                 (double)th0, (double)d0, (double)al0);
          continue;
        }

        ck_f("往返 θ", th1, th0, 0.01f);
        ck_f("往返 d", d1, d0, 0.01f);
        ck_f("往返 α", al1, al0, 0.01f);

        tgt.theta_deg = th1;
        tgt.d_mm      = d1;
        tgt.alpha_deg = al1;

        if (kin_scs_solve(&tgt, &sol) == 0u)
        {
          rejected++;          /* 几何上无解（elbow-up 够不到），不算错 */
          continue;
        }
        solved++;

        /* 接线时必须做的事：j1 从 0~360 绕回 −180~180 */
        j1w = kin_scs_wrap180(sol.j1_deg);
        if ((j1w < -180.0f) || (j1w >= 180.0f))
        {
          g_fail++;
          printf("  FAIL  wrap180 越界 j1=%.3f → %.3f\n",
                 (double)sol.j1_deg, (double)j1w);
        }
        else
        {
          g_pass++;
        }

        /* 绕回前后必须是同一个物理方向（差 360 的整数倍） */
        ck_f("wrap 前后同向", sol.j1_deg - j1w,
             floorf((sol.j1_deg - j1w) / 360.0f + 0.5f) * 360.0f, 1e-2f);

        sol.j1_deg = j1w;        /* 用绕回后的角度做正解 */

        kin_scs_fk(&sol, &x2, &y2, &z2);

        /* 判据：正解回到原点位（1e-2 mm 级；float 只有 7 位有效数字） */
        x1 = x0; y1 = y0; z1 = z0;
        ck_f("往返 x", x2, x1, 0.05f);
        ck_f("往返 y", y2, y1, 0.05f);
        ck_f("往返 z", z2, z1, 0.05f);
      }
    }
  }
  printf("  网格：解出 %d 个 / 几何无解 %d 个\n", solved, rejected);

  /* ---- ③ 不可达：必须被判出来，而不是解出一堆胡说 ----
   *    用手算的边界值，不依赖 kin_scs 自己的输出。 */
  {
    const float below = fabsf(ARM_KIN_L1_MM - ARM_KIN_L2_MM) - 5.0f;
    const float above = ARM_KIN_L1_MM + ARM_KIN_L2_MM + 5.0f;
    kin_scs_tgt_t t;
    kin_scs_out_t o;

    t.theta_deg = 0.0f; t.d_mm = below; t.alpha_deg = 0.0f;
    ck_u("d 太近 → 拒绝", (unsigned)kin_scs_solve(&t, &o), 0u);

    t.d_mm = above;
    ck_u("d 太远 → 拒绝", (unsigned)kin_scs_solve(&t, &o), 0u);
  }

  /* ---- ④ 奇异一：目标在底座转轴上 → SING_BASE，θ 沿用当前角 ---- */
  {
    kin_c2s_in_t  in;
    kin_scs_tgt_t t;
    kin_scs_out_t sol;
    float th = 0.0f;
    float d  = 0.0f;
    float al = 0.0f;
    float x, y, z;

    in.x_mm = 0.0f;
    in.y_mm = 0.0f;
    in.z_mm = 100.0f;

    ck_u("正上方 → SING_BASE",
         (unsigned)kin_c2s_deg(&in, -35.0f, &th, &d, &al),
         (unsigned)KIN_C2S_SING_BASE);
    ck_f("正上方 θ 沿用当前角", th, -35.0f, 1e-4f);
    ck_f("正上方 d", d, 100.0f, 1e-3f);
    ck_f("正上方 α", al, 90.0f, 1e-3f);

    /* 沿用当前角之后必须能真的解出来，而且正解回到 (0,0,100) */
    t.theta_deg = th;
    t.d_mm      = d;
    t.alpha_deg = al;
    ck_u("正上方能解", (unsigned)kin_scs_solve(&t, &sol), 1u);
    kin_scs_fk(&sol, &x, &y, &z);
    ck_f("正上方正解 x", x, 0.0f, 0.5f);
    ck_f("正上方正解 y", y, 0.0f, 0.5f);
    ck_f("正上方正解 z", z, 100.0f, 0.5f);
  }

  /* ---- ⑤ 奇异二：目标落在肩轴心上 → 必须失败（α 无定义）---- */
  {
    kin_c2s_in_t in;
    float th = 0.0f;
    float d  = 0.0f;
    float al = 0.0f;

    in.x_mm = 0.0f;
    in.y_mm = 0.0f;
    in.z_mm = 0.0f;
    ck_u("肩轴心上 → FAIL",
         (unsigned)kin_c2s_deg(&in, 0.0f, &th, &d, &al), (unsigned)KIN_C2S_FAIL);
  }

  /* ---- ⑥ wrap180 的边界（接线全靠它）---- */
  ck_f("wrap180(0)",     kin_scs_wrap180(0.0f),      0.0f,    1e-4f);
  ck_f("wrap180(179.9)", kin_scs_wrap180(179.9f),  179.9f,    1e-3f);
  ck_f("wrap180(180)",   kin_scs_wrap180(180.0f), -180.0f,    1e-4f);
  ck_f("wrap180(330)",   kin_scs_wrap180(330.0f),  -30.0f,    1e-4f);
  ck_f("wrap180(359)",   kin_scs_wrap180(359.0f),   -1.0f,    1e-4f);
  ck_f("wrap180(-180)",  kin_scs_wrap180(-180.0f),-180.0f,    1e-4f);
  ck_f("wrap180(-179)",  kin_scs_wrap180(-179.0f),-179.0f,    1e-4f);
  ck_f("wrap180(540)",   kin_scs_wrap180(540.0f), -180.0f,    1e-4f);
  ck_f("wrap180(-540)",  kin_scs_wrap180(-540.0f),-180.0f,    1e-4f);

  printf("%d passed, %d failed\n", g_pass, g_fail);
  return (g_fail == 0) ? 0 : 1;
}
