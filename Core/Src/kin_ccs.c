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

  const float s1 = sinf(s_cfg.j2_tilt_deg * CART_DEG2RAD);
  const float c1 = cosf(s_cfg.j2_tilt_deg * CART_DEG2RAD);
  const float s2 = sinf(s_cfg.j3_tilt_deg * CART_DEG2RAD);
  const float c2 = cosf(s_cfg.j3_tilt_deg * CART_DEG2RAD);

  float r;        /* 水平投影：底座轴 → 末端 */
  float d;        /* 斜距：肩轴心 → 末端 */
  float theta;    /* 方位角（最后会被平面偏斜修正） */
  float zt;       /* 高度（以肩为零点，向上为正） */
  float cb;
  float bend;
  float cc;       /* 末端横向偏出摆动平面多少 mm */
  float lat;      /* cc + zt·s1 */
  float sn;       /* sin(θ − φ) */
  float a;        /* 平面内：沿"前方"的分量 */
  float bt;       /* 平面内：沿"平面自己的上"分量 */
  float ap;
  float bp;
  float psi;
  float phi1;

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

  /* ---- ③ 余弦定理直接给 bend ---------------------------------------------
   *
   *     d² = L1² + L2² + 2·L1·L2·cos(bend)
   *
   *   bend 是**两根连杆之间的夹角**，跟两个摆动平面歪不歪完全无关 ——
   *   绕连杆自身轴线怎么扭都不改变这个夹角。所以平面偏斜一步都不用进来。
   *
   *   到这里为止和球坐标版**一模一样** —— 差别全在 ①。
   */
  cb = ((d * d) - (l1 * l1) - (l2 * l2)) / (2.0f * l1 * l2);

  bend = acosf(cart_clamp_unit(cb)) * CART_RAD2DEG;

  /* ---- ④ 小臂的横向偏出 -------------------------------------------------
   *
   *   c = −L2 · sin(TILT_J3) · sin(bend)
   *
   *   bend = 0 时横偏也是 0（伸直没有"出平面"这回事），弯到 90° 时最大。
   *   符号约定和 kin_scs.c 的④必须**一模一样**，否则两个库算出来的末端
   *   会镜像对称，对比就白做了。
   */
  cc = -(l2 * s2 * sinf(bend * CART_DEG2RAD));

  /* ---- ⑤ 底座补转 -------------------------------------------------------
   *
   *     sin(θ − φ) = ( c + Z·sin(TILT_J2) ) / ( R·cos(TILT_J2) )
   *
   *   R 是目标的水平半径、Z 是高度。右端超出 ±1 就是"这套几何根本到不了"。
   */
  lat = cc + (zt * s1);

  if (r < CART_D_EPS)
  {
    if ((lat > CART_D_EPS) || (lat < -CART_D_EPS))
    {
      return 0u;
    }

    a = 0.0f;
  }
  else
  {
    sn = lat / (r * c1);

    if ((sn > 1.0f) || (sn < -1.0f))
    {
      return 0u;
    }

    theta += asinf(sn) * CART_RAD2DEG;

    a = r * sqrtf(1.0f - (sn * sn));
  }

  bt = (zt + (cc * s1)) / c1;

  /* ---- ⑥ 平面内两连杆 → 大臂仰角 ---------------------------------------
   *
   *   A = L1 + L2·cos(bend)           沿大臂方向
   *   B = L2·sin(bend)·cos(TILT_J3)   垂直大臂方向（只剩落在平面里的那部分）
   *
   *   φ1 = atan2(bt, a) + atan2(B, A)    ← 加号 = elbow-up
   */
  ap  = l1 + (l2 * cosf(bend * CART_DEG2RAD));
  bp  = l2 * sinf(bend * CART_DEG2RAD) * c2;
  psi = atan2f(bp, ap) * CART_RAD2DEG;

  phi1 = (atan2f(bt, a) * CART_RAD2DEG) + psi;

  /* ---- ⑦ 几何角 → 舵机命令角 -------------------------------------------
   *
   *      命令角 = OFF + SIGN × 几何角
   */
  out->j1_deg = cart_norm360(theta);
  out->j2_deg = s_cfg.j2_off_deg + (s_cfg.j2_sign * phi1);
  out->j3_deg = s_cfg.j3_off_deg + (s_cfg.j3_sign * bend);

  return 1u;
}

void kin_ccs_fk(const kin_ccs_out_t *q,
                     float *x_mm, float *y_mm, float *z_mm)
{
  const float s1 = sinf(s_cfg.j2_tilt_deg * CART_DEG2RAD);
  const float c1 = cosf(s_cfg.j2_tilt_deg * CART_DEG2RAD);
  const float s2 = sinf(s_cfg.j3_tilt_deg * CART_DEG2RAD);
  const float c2 = cosf(s_cfg.j3_tilt_deg * CART_DEG2RAD);

  float phi1;
  float bend;
  float theta;
  float ap;
  float bp;
  float cp;
  float a;
  float bt;
  float c;
  float u;
  float p1;

  if ((q == 0) || (x_mm == 0) || (y_mm == 0) || (z_mm == 0))
  {
    return;
  }

  /* 命令角 → 几何角。SIGN 是 ±1，所以 1/SIGN == SIGN。 */
  phi1  = (q->j2_deg - s_cfg.j2_off_deg) * s_cfg.j2_sign;
  bend  = (q->j3_deg - s_cfg.j3_off_deg) * s_cfg.j3_sign;
  theta = q->j1_deg * CART_DEG2RAD;

  /* 平面内两连杆合成一个矢量，再拆成 (沿大臂, 垂直大臂, 出平面) */
  ap = s_cfg.l1_mm + (s_cfg.l2_mm * cosf(bend * CART_DEG2RAD));
  bp = s_cfg.l2_mm * sinf(bend * CART_DEG2RAD) * c2;
  cp = s_cfg.l2_mm * sinf(bend * CART_DEG2RAD) * s2;

  /* 转到"这个摆动平面"的坐标里：(a 前, bt 上, c 法向) */
  p1 = phi1 * CART_DEG2RAD;
  a  = (ap * cosf(p1)) + (bp * sinf(p1));
  bt = (ap * sinf(p1)) - (bp * cosf(p1));
  c  = -cp;

  /* 再转回世界坐标（基向量同 kin_scs_fk，两边必须完全一致） */
  u = (bt * s1) + (c * c1);

  *x_mm = (a * cosf(theta)) + (u * sinf(theta));
  *y_mm = (a * sinf(theta)) - (u * cosf(theta));
  *z_mm = (bt * c1) - (c * s1);
}
