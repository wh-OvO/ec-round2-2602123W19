/**
  ******************************************************************************
  * @file    kin_ccs.c
  * @brief   传统笛卡尔逆解的实现（推导、坐标系约定见 kin_ccs.h）
  ******************************************************************************
  * @note    这个文件**不 include arm_cfg.h、不 include kin_scs.h**，
  *          几何参数全靠 kin_ccs_setup() 喂进来。
  *          故意保持独立，就是为了能和 kin_scs.c 并排对照。
  ******************************************************************************
  */

#include "kin_ccs.h"

#include <math.h>

/* ---------------------------------------------------------------------------
 * 私有常量（不从任何地方借）
 * ------------------------------------------------------------------------ */

#define CART_PI       3.14159265358979f
#define CART_RAD2DEG  (180.0f / CART_PI)
#define CART_DEG2RAD  (CART_PI / 180.0f)

/* 距离容差（mm）。够到边界时 acos 的入参会因为浮点误差略微超出 [-1, 1]。 */
#define CART_D_EPS    0.01f

/* 输入合法性上限。超过这个量级只会是上位机发错或者内存被踩了。 */
#define CART_MAG_MAX  1.0e6f

/* ---------------------------------------------------------------------------
 * 私有状态
 * ------------------------------------------------------------------------ */

static kin_ccs_cfg_t s_cfg;
static uint8_t            s_ready = 0u;

/* ---------------------------------------------------------------------------
 * 私有工具
 * ------------------------------------------------------------------------ */

/* acos 的入参必须落在 [-1, 1]：手算出来的 cos 在边界上可能是 1.0000001 */
static float cart_clamp_unit(float v)
{
  if (v > 1.0f)
  {
    return 1.0f;
  }

  if (v < -1.0f)
  {
    return -1.0f;
  }

  return v;
}

/* NaN / 无穷检查。v != v 是 NaN 唯一可靠的判据，不引 isfinite()。 */
static uint8_t cart_sane(float v)
{
  if (v != v)
  {
    return 0u;
  }

  if ((v > CART_MAG_MAX) || (v < -CART_MAG_MAX))
  {
    return 0u;
  }

  return 1u;
}

/* 归一化到 [0, 360)。360 要折成 0 —— 同一个位置，但差一整个量程。 */
static float cart_norm360(float deg)
{
  while (deg < 0.0f)
  {
    deg += 360.0f;
  }

  while (deg >= 360.0f)
  {
    deg -= 360.0f;
  }

  return deg;
}

/* ---------------------------------------------------------------------------
 * 接口实现
 * ------------------------------------------------------------------------ */

uint8_t kin_ccs_setup(const kin_ccs_cfg_t *cfg)
{
  /* 先无条件清成"未就绪"：中途 return 时绝不会留着上一轮的旧参数在跑。 */
  s_ready = 0u;

  if (cfg == 0)
  {
    return 0u;
  }

  if ((cfg->l1_mm > 0.0f) == 0)
  {
    return 0u;
  }

  if ((cfg->l2_mm > 0.0f) == 0)
  {
    return 0u;
  }

  /* SIGN 只允许 ±1：它是"舵机装反了没有"，不是比例系数。 */
  if ((cfg->j2_sign != 1.0f) && (cfg->j2_sign != -1.0f))
  {
    return 0u;
  }

  if ((cfg->j3_sign != 1.0f) && (cfg->j3_sign != -1.0f))
  {
    return 0u;
  }

  s_cfg = *cfg;
  s_ready = 1u;

  return 1u;
}

uint8_t kin_ccs_ready(void)
{
  return s_ready;
}

void kin_ccs_reach(float *d_min, float *d_max)
{
  const float lo = fabsf(s_cfg.l1_mm - s_cfg.l2_mm);

  if (d_min != 0)
  {
    *d_min = lo;
  }

  if (d_max != 0)
  {
    *d_max = s_cfg.l1_mm + s_cfg.l2_mm;
  }
}

uint8_t kin_ccs_solve(float x_mm, float y_mm, float z_mm,
                           kin_ccs_out_t *out)
{
  const float l1 = s_cfg.l1_mm;
  const float l2 = s_cfg.l2_mm;

  float r;        /* 水平投影：底座轴 → 末端 */
  float d;        /* 斜距：肩轴心 → 末端 */
  float theta;    /* 方位角 */
  float zt;       /* 高度（以肩为零点，向上为正） */
  float cb;       /* cos β：大臂 与 OP 连线的夹角 */
  float cg;       /* cos γ：两连杆之间的内角 */
  float beta;
  float gamma;
  float bend;     /* 两连杆之间的几何夹角（bend = 180° − γ） */
  float phi1;     /* 大臂几何仰角 */
  float bend_c;   /* 扣掉偏置之后的命令角 */

  if (out == 0)
  {
    return 0u;
  }

  /* 没 setup 就不解。宁可"什么都不动"，也不要拿 0 当连杆长度算出一堆 NaN。 */
  if (s_ready == 0u)
  {
    return 0u;
  }

  if ((cart_sane(x_mm) == 0u) || (cart_sane(y_mm) == 0u) || (cart_sane(z_mm) == 0u))
  {
    return 0u;
  }

  /* ---- ① 3D 直角坐标 → 球坐标 -------------------------------------------
   *
   *   ★ 这一段就是**球坐标版省掉的全部东西**，两次开方 + 两次 atan2：
   *
   *       r = √(x² + y²)        水平投影
   *       θ = atan2(y, x)       方位角，(−180, 180]
   *       d = √(r² + z²)        斜距
   *       α = atan2(z, r)       仰角，(−180, 180]
   *
   *   ⚠️ x = y = 0 时 atan2 无定义（newlib 返回 0，不报错），
   *      θ 会静默取 0。所以下面 d < eps 时直接拒绝。
   */
  r = sqrtf((x_mm * x_mm) + (y_mm * y_mm));

  theta = (r > CART_D_EPS) ? (atan2f(y_mm, x_mm) * CART_RAD2DEG) : 0.0f;

  d = sqrtf((r * r) + (z_mm * z_mm));

  /* 高度直接用 z —— 肩轴心就是原点，不用再算一次仰角。
   * 球坐标版那边是 z = d·sin(α)，同一个量。 */
  zt = z_mm;

  /* 目标就在肩轴心上：方向无定义，逆解不存在。 */
  if (d < CART_D_EPS)
  {
    return 0u;
  }

  /* ---- ② 可达性：|L1 − L2| ≤ d ≤ L1 + L2 ------------------------------- */
  if (d < (fabsf(l1 - l2) - CART_D_EPS))
  {
    return 0u;
  }

  if (d > ((l1 + l2) + CART_D_EPS))
  {
    return 0u;
  }

  /* ---- ③ 平面两连杆：两次余弦定理 ---------------------------------------
   *
   *   到这里为止和球坐标版**一模一样** —— 差别全在 ①（怎么拿 r/θ/z）。
   *   三角形三边是 L1、L2、d：
   *
   *     β = ∠(大臂, OP)      cos β = (L1² + d² − L2²) / (2·L1·d)
   *     γ = ∠(大臂, 小臂)    cos γ = (L1² + L2² − d²) / (2·L1·L2)
   *
   *   γ 是**内角**，弯得越厉害它越小；要的"折角"是补角 bend = 180° − γ。
   */
  cb = ((l1 * l1) + (d * d) - (l2 * l2)) / (2.0f * l1 * d);
  cg = ((l1 * l1) + (l2 * l2) - (d * d)) / (2.0f * l1 * l2);

  beta  = acosf(cart_clamp_unit(cb)) * CART_RAD2DEG;
  gamma = acosf(cart_clamp_unit(cg)) * CART_RAD2DEG;

  /* 大臂仰角 = 目标仰角 + β（elbow-up，和球坐标版同一个分支）。
   * 目标仰角这里用 atan2(z, r) 反推 —— 球坐标版是入口直接给 α。 */
  phi1 = (atan2f(zt, r) * CART_RAD2DEG) + beta;
  bend = 180.0f - gamma;

  /* ---- ④ 扣掉臂本身的角度偏置 -------------------------------------------
   *
   *   和 kin_scs.c 的 ④ 必须**逐字一致**，否则两个库解出来的不是同一个姿态，
   *   对照就白做了。偏置来自 arm_cfg.h，这里是从 cfg 里读的。
   */
  bend_c = bend - s_cfg.j3_bias_deg;

  if (bend_c < 0.0f)
  {
    bend_c = 0.0f;
  }

  /* ---- ⑤ 几何角 → 舵机命令角 -------------------------------------------
   *
   *      命令角 = OFF + SIGN × 几何角
   */
  out->j1_deg = cart_norm360(theta);
  out->j2_deg = s_cfg.j2_off_deg + (s_cfg.j2_sign * (phi1 + s_cfg.j2_bias_deg));
  out->j3_deg = s_cfg.j3_off_deg + (s_cfg.j3_sign * bend_c);

  return 1u;
}

void kin_ccs_fk(const kin_ccs_out_t *q,
                     float *x_mm, float *y_mm, float *z_mm)
{
  const float l1 = s_cfg.l1_mm;
  const float l2 = s_cfg.l2_mm;

  float phi1;
  float bend;
  float theta;
  float rr;
  float zz;

  if ((q == 0) || (x_mm == 0) || (y_mm == 0) || (z_mm == 0))
  {
    return;
  }

  /* 命令角 → 几何角。SIGN 是 ±1，所以 1/SIGN == SIGN。
   * BIAS 这里减（解算里加），两边互逆。 */
  phi1  = ((q->j2_deg - s_cfg.j2_off_deg) * s_cfg.j2_sign) - s_cfg.j2_bias_deg;
  bend  = ((q->j3_deg - s_cfg.j3_off_deg) * s_cfg.j3_sign) + s_cfg.j3_bias_deg;
  theta = q->j1_deg * CART_DEG2RAD;

  /* 平面两连杆正解：大臂在 φ1，小臂在 φ1 − bend。 */
  rr = (l1 * cosf(phi1 * CART_DEG2RAD)) +
       (l2 * cosf((phi1 - bend) * CART_DEG2RAD));

  zz = (l1 * sinf(phi1 * CART_DEG2RAD)) +
       (l2 * sinf((phi1 - bend) * CART_DEG2RAD));

  *x_mm = rr * cosf(theta);
  *y_mm = rr * sinf(theta);
  *z_mm = zz;
}
