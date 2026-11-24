/**
  ******************************************************************************
  * @file    c2s_selftest.c
  * @brief   kin_c2s 的 PC 端单测（不需要板子）
  *
  *          编译（在 rm_arm 目录下）：
  *            gcc -std=c11 -Wall -Wextra -Wpedantic -O2 -ffp-contract=off
  *                -I Core/Inc tests/c2s_selftest.c Core/Src/kin_c2s.c -lm
  *                -o tests/c2s_selftest.exe
  *
  *          ⚠️ 必须带 -lm：本模块用了 sqrtf / atan2f。
  ******************************************************************************
  */

#include "kin_c2s.h"

#include <math.h>
#include <stdio.h>

static int g_pass = 0;
static int g_fail = 0;

static void check_f(const char *name, float got, float want, float tol)
{
  const float diff = fabsf(got - want);

  if (diff <= tol)
  {
    g_pass++;
  }
  else
  {
    g_fail++;
    printf("  FAIL  %-26s got %10.5f  want %10.5f  (diff %.5f)\n",
           name, (double)got, (double)want, (double)diff);
  }
}

static void check_u(const char *name, unsigned got, unsigned want)
{
  if (got == want)
  {
    g_pass++;
  }
  else
  {
    g_fail++;
    printf("  FAIL  %-26s got %u  want %u\n", name, got, want);
  }
}

/* 喂一组进去：先核对状态码，正常/奇异时再核对三个出参 */
static void one(const char *tag, uint8_t want_st,
                float x, float y, float z, float cur_theta,
                float w_theta, float w_d, float w_alpha)
{
  const kin_c2s_in_t in = { x, y, z };
  float th = 0.0f;
  float d  = 0.0f;
  float al = 0.0f;
  const uint8_t st = kin_c2s_deg(&in, cur_theta, &th, &d, &al);
  char name[64];

  check_u(tag, (unsigned)st, (unsigned)want_st);
  if (st != want_st)
  {
    return;
  }
  if ((st != KIN_C2S_OK) && (st != KIN_C2S_SING_BASE))
  {
    return;                     /* 失败时三个出参不保证有值，别核对 */
  }

  (void)snprintf(name, sizeof(name), "%s theta", tag);
  check_f(name, th, w_theta, 1e-3f);

  (void)snprintf(name, sizeof(name), "%s d", tag);
  check_f(name, d, w_d, 1e-3f);

  (void)snprintf(name, sizeof(name), "%s alpha", tag);
  check_f(name, al, w_alpha, 1e-3f);
}

int main(void)
{
  kin_c2s_in_t in;
  float th;
  float d;
  float al;

  printf("=== kin_c2s 单测：笛卡尔 → 球坐标 ===\n");

  /* ---- ① 三条坐标轴，各自正反向 ---- */
  one("+x 轴", KIN_C2S_OK,
      100.0f,   0.0f,   0.0f,  0.0f,    0.0f, 100.0f,   0.0f);
  one("+y 轴", KIN_C2S_OK,
        0.0f, 100.0f,   0.0f,  0.0f,   90.0f, 100.0f,   0.0f);
  one("-x 轴", KIN_C2S_OK,
     -100.0f,   0.0f,   0.0f,  0.0f,  180.0f, 100.0f,   0.0f);
  one("-y 轴", KIN_C2S_OK,
        0.0f,-100.0f,   0.0f,  0.0f,  -90.0f, 100.0f,   0.0f);

  /* ---- ② 竖直方向（z 轴）：底座奇异，θ 必须原样沿用传进来的当前角 ----
   *    这三个是**奇异但不是错误**的用例：状态码必须是 SING_BASE，
   *    而且 d/α 照样得对。 */
  one("+z 正上方（θ 沿用 37）", KIN_C2S_SING_BASE,
        0.0f, 0.0f, 100.0f, 37.0f, 37.0f, 100.0f,  90.0f);
  one("-z 正下方（θ 沿用 37）", KIN_C2S_SING_BASE,
        0.0f, 0.0f,-100.0f, 37.0f, 37.0f, 100.0f, -90.0f);
  one("+z 正上方（当前角 0）", KIN_C2S_SING_BASE,
        0.0f, 0.0f, 100.0f,  0.0f,  0.0f, 100.0f,  90.0f);
  /* 当前角给负数也得原样沿用，不能被吞成 0 */
  one("+z 正上方（θ 沿用 -120）", KIN_C2S_SING_BASE,
        0.0f, 0.0f, 100.0f,-120.0f,-120.0f, 100.0f, 90.0f);

  /* ---- ③ 斜面 45°、3-4-5 勾股 ---- */
  one("x-z 45°", KIN_C2S_OK,
      100.0f,   0.0f, 100.0f,  0.0f,   0.0f, 141.421356f, 45.0f);
  one("3-4-5", KIN_C2S_OK,
       60.0f,  80.0f,   0.0f,  0.0f,  53.130102f, 100.0f,  0.0f);
  one("3-4-5 + z=100", KIN_C2S_OK,
       60.0f,  80.0f, 100.0f,  0.0f,  53.130102f, 141.421356f, 45.0f);

  /* ---- ④ 负的仰角（目标在肩轴水平面以下）---- */
  one("俯 30°", KIN_C2S_OK,
       86.60254f, 0.0f, -50.0f, 0.0f, 0.0f, 100.0f, -30.0f);

  /* ---- ⑤ 奇点 ②：目标落在肩轴心上 → 必须判失败 ---- */
  in.x_mm = 0.0f; in.y_mm = 0.0f; in.z_mm = 0.0f;
  th = 0.0f; d = 0.0f; al = 0.0f;
  check_u("d=0 判失败",
          (unsigned)kin_c2s_deg(&in, 0.0f, &th, &d, &al), (unsigned)KIN_C2S_FAIL);

  /* ---- ⑥ 空指针 → 必须判失败，不能崩 ---- */
  check_u("xyz 为空",   (unsigned)kin_c2s_deg(0, 0.0f, &th, &d, &al),
          (unsigned)KIN_C2S_FAIL);
  check_u("theta 为空", (unsigned)kin_c2s_deg(&in, 0.0f, 0, &d, &al),
          (unsigned)KIN_C2S_FAIL);
  check_u("d 为空",     (unsigned)kin_c2s_deg(&in, 0.0f, &th, 0, &al),
          (unsigned)KIN_C2S_FAIL);
  check_u("alpha 为空", (unsigned)kin_c2s_deg(&in, 0.0f, &th, &d, 0),
          (unsigned)KIN_C2S_FAIL);

  printf("%d passed, %d failed\n", g_pass, g_fail);
  return (g_fail == 0) ? 0 : 1;
}
