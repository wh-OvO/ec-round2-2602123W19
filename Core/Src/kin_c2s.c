/**
  ******************************************************************************
  * @file    kin_c2s.c
  * @brief   笛卡尔 → 球坐标：x,y,z → θ,d,α（见 kin_c2s.h 头注释）
  ******************************************************************************
  * @note    **四行数学，为什么值得一个文件**
  *
  *              r = √(x² + y²)          水平投影半径
  *              θ = atan2(y, x)         方位角
  *              d = √(r² + z²)          斜距（肩轴心 → 末端）
  *              α = atan2(z, r)         仰角
  *
  *          单独成文件不是为了"少写几行"，是为了**能被单测逐点钉死**。
  *          这段一旦算错（单位、正负、坐标系原点），kin_scs 会把一个错误的
  *          球坐标解成一堆"看着像对"的关节角 —— 那种错误在板子上极难发现：
  *          手臂会动、会到位、就是位置差一截。
  *
  *          所以 tests/c2s_selftest.c 对 9 个点做了数值校验，包括两个奇点。
  ******************************************************************************
  */

#include "kin_c2s.h"

#include <math.h>

/* 角度换算。不用 M_PI：C11 标准里 <math.h> 不保证定义它（-Wpedantic 会报）。 */
#define C2S_PI      3.14159265358979323846f
#define C2S_RAD2DEG (180.0f / C2S_PI)

/* 判"是否落在轴上"的阈值，单位 mm。
 * 0.001mm = 1µm，比手臂的机械分辨率（几十 µm 量级）小得多，
 * 所以它只会命中**数学上真的在轴上**的情况，不会误伤正常目标。 */
#define C2S_EPS_MM  0.001f

uint8_t kin_c2s_deg(const kin_c2s_in_t *xyz,
                    float               cur_theta_deg,
                    float              *theta_deg,
                    float              *d_mm,
                    float              *alpha_deg)
{
  float r;
  float d;
  uint8_t st;

  if ((xyz == 0) || (theta_deg == 0) || (d_mm == 0) || (alpha_deg == 0))
  {
    return KIN_C2S_FAIL;
  }

  /* NaN / Inf 守卫：协议层是 int16，正常进不来；但 0x34 的调用链里还有
   * cur_theta（浮点）和以后可能的其它入口。NaN 会一路穿过 sqrtf/atan2f，
   * 最后变成“解出来的关节角也是 NaN”，而 NaN 与任何数比较都为假，
   * 限位、可达性、奇异判定会全部放行 —— 必须在入口挡住。 */
  if ((isfinite(xyz->x_mm) == 0) || (isfinite(xyz->y_mm) == 0) ||
      (isfinite(xyz->z_mm) == 0) || (isfinite(cur_theta_deg) == 0))
  {
    return KIN_C2S_FAIL;
  }

  /* ① 水平投影半径 */
  r = sqrtf((xyz->x_mm * xyz->x_mm) + (xyz->y_mm * xyz->y_mm));

  /* ② 斜距：肩轴心 → 末端。注意是 r 和 z 勾股，不是 x/y/z 三维勾股 ——
   *    结果一样，但这样写只用两次开方，而且顺手把 r 留给第 ④ 步。 */
  d = sqrtf((r * r) + (xyz->z_mm * xyz->z_mm));

  /* ③ d ≈ 0：目标点落在肩轴心上，仰角无定义 → 判失败。
   *    这一步必须在算 atan2 之前，否则 atan2(0,0) 会静默返回 0，
   *    解出一个"α = 0"的假目标。 */
  if (d < C2S_EPS_MM)
  {
    return KIN_C2S_FAIL;
  }

  /* ④ 方位角 θ：r ≈ 0 时末端在底座轴正上/正下方，θ 无定义。
   *    沿用**当前**底座角，而不是 0 —— 否则目标每路过一次正上方，
   *    底座就会自己弹回 0°，看起来就是"抖一下"。
   *    这种情况**不算失败**：位置是明确的，只是底座那一维没有唯一解。
   *    单独返回 KIN_C2S_SING_BASE，让上层能如实告诉上位机。 */
  if (r < C2S_EPS_MM)
  {
    *theta_deg = cur_theta_deg;
    st = KIN_C2S_SING_BASE;
  }
  else
  {
    *theta_deg = atan2f(xyz->y_mm, xyz->x_mm) * C2S_RAD2DEG;
    st = KIN_C2S_OK;
  }

  *d_mm      = d;
  *alpha_deg = atan2f(xyz->z_mm, r) * C2S_RAD2DEG;

  return st;
}
