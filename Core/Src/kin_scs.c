/**
  ******************************************************************************
  * @file    kin_scs.c
  * @brief   球坐标逆解的实现（推导见 kin_scs.h 文件头）
  ******************************************************************************
  */

#include "kin_scs.h"
#include "arm_cfg.h"

#include <math.h>

/* ---------------------------------------------------------------------------
 * 常量
 * ------------------------------------------------------------------------ */

#define KIN_PI        3.14159265358979f
#define KIN_DEG2RAD   (KIN_PI / 180.0f)
#define KIN_RAD2DEG   (180.0f / KIN_PI)

/* 距离容差（mm）。够到边界时 acos 的入参会因为浮点误差略微超出 [-1, 1]，
 * 所以两头都放宽一点点，再把这几个百分之一毫米的误差吃掉。 */
#define KIN_D_ESP     0.01f

/* 输入合法性上限。超过这个数量级的数只会是上位机发错了或者内存被踩了，
 * 直接当非法输入拒绝，免得后面的 while 归一化转上几亿圈。 */
#define KIN_MAG_MAX   1.0e6f

/* ---------------------------------------------------------------------------
 * 内部工具
 * ------------------------------------------------------------------------ */

/* acos 的入参必须落在 [-1, 1]：手算出来的 cos 值在边界上可能是 1.0000001 */
static float clamp_unit(float v)
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

/* NaN / 无穷 的检查。写成 v != v 是因为 NaN 不等于自己 —— 不需要 isfinite()，
 * 也就不用引 math.h 里那些可能把 FLASH 撑大的宏。 */
static uint8_t is_sane(float v)
{
  if (v != v)
  {
    return 0u;
  }

  if ((v > KIN_MAG_MAX) || (v < -KIN_MAG_MAX))
  {
    return 0u;
  }

  return 1u;
}

/* 把角度归一化到 [0, 360)。注意 360 本身要变成 0 —— 它和 0 是同一个位置，
 * 但差一整个量程，留在外面会让限位判断误以为超程。 */
static float norm360(float deg)
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

uint8_t kin_scs_ready(void)
{
  if ((ARM_KIN_L1_MM > 0.0f) && (ARM_KIN_L2_MM > 0.0f))
  {
    return 1u;
  }

  return 0u;
}

void kin_scs_reach(float *d_min, float *d_max)
{
  const float l1 = ARM_KIN_L1_MM;
  const float l2 = ARM_KIN_L2_MM;
  const float lo = (l1 > l2) ? (l1 - l2) : (l2 - l1);

  if (d_min != 0)
  {
    *d_min = lo;
  }

  if (d_max != 0)
  {
    *d_max = l1 + l2;
  }
}

uint8_t kin_scs_solve(const kin_scs_tgt_t *tgt, kin_scs_out_t *out)
{
  const float l1 = ARM_KIN_L1_MM;
  const float l2 = ARM_KIN_L2_MM;

  /* 两个摆动平面的偏斜角。三角函数每次解算只算一遍（一次移动只调用一次），
   * 所以不为了省这点时间去搞静态缓存。 */
  const float s1 = sinf(ARM_KIN_J2_TILT_DEG * KIN_DEG2RAD);
  const float c1 = cosf(ARM_KIN_J2_TILT_DEG * KIN_DEG2RAD);
  const float s2 = sinf(ARM_KIN_J3_TILT_DEG * KIN_DEG2RAD);
  const float c2 = cosf(ARM_KIN_J3_TILT_DEG * KIN_DEG2RAD);

  float d;      /* 肩 → 目标的斜距 */
  float r;      /* 目标的水平距离（绕肩轴量） */
  float zt;     /* 目标的高度（以肩轴为零点，向上为正） */
  float cb;     /* 余弦定理里的 cos(bend) */
  float bend;   /* 几何弯曲量（度） */
  float cc;     /* 末端横向偏出摆动平面多少 mm */
  float lat;    /* cc + zt·s1，下面要用两次 */
  float sn;     /* sin(θ − φ)，即底座要补转的那个角的正弦 */
  float theta;  /* 修正后的底座角 */
  float a;      /* 平面内：沿"前方"的分量 */
  float bt;     /* 平面内：沿"平面自己的上"分量 */
  float ap;     /* 平面内两连杆合成的"沿大臂"分量 */
  float bp;     /* 平面内两连杆合成的"垂直大臂"分量 */
  float psi;    /* 上面那个合矢量相对大臂的夹角 */
  float phi1;   /* 大臂的几何仰角（度） */

  if ((tgt == 0) || (out == 0))
  {
    return 0u;
  }

  /* ---- ① 参数体检 -------------------------------------------------------
   * 没标定就直接拒绝。宁可"什么都不动"，也不要拿 0 去当连杆长度算出
   * 一堆 NaN 再写进 CCR —— 那会让舵机瞬间冲向某个随机角度。 */
  if (kin_scs_ready() == 0u)
  {
    return 0u;
  }

  if ((is_sane(tgt->theta_deg) == 0u) ||
      (is_sane(tgt->d_mm)      == 0u) ||
      (is_sane(tgt->alpha_deg) == 0u))
  {
    return 0u;
  }

  d = tgt->d_mm;

  /* ---- ② 可达性：|L1 − L2| ≤ d ≤ L1 + L2 -------------------------------
   * 这是两连杆的硬边界。d 太远 = 伸直也够不着；d 太近 = 折到极限也靠不上。
   *
   * ⚠️ 注意这里的 d 是**肩到目标的直线距离**，不是水平距离。
   *    两个平面歪不歪**不影响**这条边界 —— 因为绕着连杆自身轴线转，
   *    末端到肩的距离不变（后面③会再确认一次）。
   *
   * 所以平面偏斜只吃掉"方向"，不吃"距离"：斜距范围还是 62~148，
   * 但理想仰角上限会从 90° 掉到 90° − TILT_J2。 */
  if (d < ((fabsf(l1 - l2)) - KIN_D_ESP))
  {
    return 0u;
  }

  if (d > ((l1 + l2) + KIN_D_ESP))
  {
    return 0u;
  }

  /* d 趋近 0 时下面要除以 d。只有 L1 == L2 才可能走到这里。 */
  if (d < KIN_D_ESP)
  {
    return 0u;
  }

  /* 球坐标 → 水平半径 + 高度。肩是原点，所以直接就是这两个分量。 */
  r  = d * cosf(tgt->alpha_deg * KIN_DEG2RAD);
  zt = d * sinf(tgt->alpha_deg * KIN_DEG2RAD);

  /* ---- ③ 余弦定理直接给 bend -------------------------------------------
   *
   *   关键一步：**bend 是两根连杆之间的夹角**，跟两个摆动平面歪不歪、
   *   歪多少**完全无关**。绕连杆自身轴线怎么扭都不改变两连杆的夹角。
   *   所以平面偏斜一步都不用进这条公式：
   *
   *     d² = L1² + L2² + 2·L1·L2·cos(bend)
   *     cos(bend) = (d² − L1² − L2²) / (2·L1·L2)
   *
   *   d = L1+L2（伸直）→ cos = 1 → bend = 0；
   *   d = |L1−L2|（折死）→ cos = −1 → bend = 180°。定义域正好对上。
   *
   *   用 cos 不用 sin：acos 直接给 [0, π]，不用判象限，边界上也不会出岔子。 */
  cb = ((d * d) - (l1 * l1) - (l2 * l2)) / (2.0f * l1 * l2);
  bend = acosf(clamp_unit(cb)) * KIN_RAD2DEG;

  /* ---- ④ 平面偏斜修正：小臂的横向偏出 -----------------------------------
   *
   *   小臂的摆动平面相对大臂平面歪了 TILT_J3。这意味着弯肘的时候，
   *   末端不是留在原平面里，而是**斜着出去**：
   *
   *     c = −L2 · sin(TILT_J3) · sin(bend)
   *
   *   bend = 0（伸直）时 c = 0 —— 伸直的手臂没有"出平面"这回事；
   *   bend = 90° 时偏出最多，等于 L2·sin(TILT_J3)。
   *   按 105mm / 20° 算，最大偏出 36mm —— 这不是小数，必须算进来。
   *
   *   负号：以笛卡尔 x 向前、y 向左、z 向上，TILT_J3 > 0 时末端偏到 **−y** 侧
   *   （俯视看是顺时针那侧）。实测反了就改 arm_cfg.h 里那个角的符号。 */
  cc = -(l2 * s2 * sinf(bend * KIN_DEG2RAD));

  /* ---- ⑤ 底座补转：把横向偏移"转"回去 ----------------------------------
   *
   *   手臂是歪的，末端天生偏在一边。想让它落到指定方位 φ 上，底座就不能
   *   正好停在 φ，得偏一点。偏多少由几何直接解出来：
   *
   *     目标点 P 在"θ 方位那个平面"的坐标系里 = (a, bt, c)
   *     其中 c = cos(TILT_J2)·R·sin(θ−φ) − sin(TILT_J2)·Z
   *
   *   令它等于手臂能给出来的 c（④那条），解出
   *
   *     sin(θ − φ) = ( c + Z·sin(TILT_J2) ) / ( R·cos(TILT_J2) )
   *
   *   右端超出 ±1 → 这个目标点根本不在这套几何能达到的曲面上，拒绝。
   *   典型情况：目标贴着肩轴正上方（R≈0），而手臂因为 J2 歪了 10°，
   *   最高只能指到 80°。 */
  lat = cc + (zt * s1);

  if (r < KIN_D_ESP)
  {
    /* 目标正好在肩轴正上方/正下方：θ 无定义，只有偏斜恰好抵消时才有解 */
    if ((lat > KIN_D_ESP) || (lat < -KIN_D_ESP))
    {
      return 0u;
    }

    theta = tgt->theta_deg;
    a     = 0.0f;
  }
  else
  {
    sn = lat / (r * c1);

    if ((sn > 1.0f) || (sn < -1.0f))
    {
      return 0u;
    }

    theta = tgt->theta_deg + (asinf(sn) * KIN_RAD2DEG);

    /* a = R·cos(θ−φ)。用 sqrt 而不是再算一次 cos：|θ−φ| ≤ 90°，
     * 所以 cos 一定是正的那个根。 */
    a = r * sqrtf(1.0f - (sn * sn));
  }

  /* 平面内"上"方向的分量。cos(TILT_J2) 这一除是坐标变换带来的，
   * 不是修正量 —— 偏斜角为 0 时它等于 1，退化成 bt = Z。 */
  bt = (zt + (cc * s1)) / c1;

  /* ---- ⑥ 平面内两连杆 → 大臂仰角 ---------------------------------------
   *
   *   现在 (a, bt) 已经是"落在这个摆动平面里"的坐标了，剩下的就是经典
   *   平面两连杆。把两根连杆合成一个矢量：
   *
   *     A = L1 + L2·cos(bend)          ← 沿大臂方向的分量
   *     B = L2·sin(bend)·cos(TILT_J3)  ← 垂直大臂的分量
   *
   *   B 里那个 cos(TILT_J3) 就是"小臂歪出去了一部分，剩下的才落在平面里"。
   *
   *     ψ  = atan2(B, A)
   *     φ1 = atan2(bt, a) + ψ          ← 加号 = elbow-up，大臂抬到 OP 线上方
   *
   *   换成 elbow-down 只要把加号改成减号，但那样大臂会压低、重力力臂变大，
   *   而且和"解只有一个"的假设冲突，所以本模块不提供。 */
  ap  = l1 + (l2 * cosf(bend * KIN_DEG2RAD));
  bp  = l2 * sinf(bend * KIN_DEG2RAD) * c2;
  psi = atan2f(bp, ap) * KIN_RAD2DEG;

  phi1 = (atan2f(bt, a) * KIN_RAD2DEG) + psi;

  /* ---- ⑦ 几何量 → 舵机命令角 -------------------------------------------
   *
   *   两者之间只差一个"安装偏移 + 方向"：
   *
   *       命令角 = ARM_KIN_Jn_OFF_DEG + ARM_KIN_Jn_SIGN × 几何角
   *
   *   SIGN 只能是 +1 或 −1，对应舵机装反。跟 arm_cfg.c 里用
   *   us_at_deg_min/max 对调来处理装反是一个套路：量出来填数，逻辑不动。
   *
   *   注意 J1 输出的是**修正后**的 theta，不是上位机发进来的方位角 ——
   *   差了那个 asin(·)。这是这套几何的必然结果，不是笔误。 */
  out->j1_deg = norm360(theta);
  out->j2_deg = ARM_KIN_J2_OFF_DEG + (ARM_KIN_J2_SIGN * phi1);
  out->j3_deg = ARM_KIN_J3_OFF_DEG + (ARM_KIN_J3_SIGN * bend);

  return 1u;
}

void kin_scs_fk(const kin_scs_out_t *q, float *x_mm, float *y_mm, float *z_mm)
{
  const float l1 = ARM_KIN_L1_MM;
  const float l2 = ARM_KIN_L2_MM;

  const float s1 = sinf(ARM_KIN_J2_TILT_DEG * KIN_DEG2RAD);
  const float c1 = cosf(ARM_KIN_J2_TILT_DEG * KIN_DEG2RAD);
  const float s2 = sinf(ARM_KIN_J3_TILT_DEG * KIN_DEG2RAD);
  const float c2 = cosf(ARM_KIN_J3_TILT_DEG * KIN_DEG2RAD);

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
  float p2;
  float p3;

  if ((q == 0) || (x_mm == 0) || (y_mm == 0) || (z_mm == 0))
  {
    return;
  }

  /* 命令角 → 几何角。SIGN 是 ±1，所以 1/SIGN == SIGN。 */
  phi1  = (q->j2_deg - ARM_KIN_J2_OFF_DEG) * ARM_KIN_J2_SIGN;
  bend  = (q->j3_deg - ARM_KIN_J3_OFF_DEG) * ARM_KIN_J3_SIGN;
  theta = q->j1_deg * KIN_DEG2RAD;

  /* 平面内两连杆合成一个矢量，再拆成 (沿大臂, 垂直大臂) */
  ap  = l1 + (l2 * cosf(bend * KIN_DEG2RAD));
  bp  = l2 * sinf(bend * KIN_DEG2RAD) * c2;
  cp  = l2 * sinf(bend * KIN_DEG2RAD) * s2;

  /* 转到"这个摆动平面"的坐标里：(a 前, bt 上, c 法向) */
  p1 = phi1 * KIN_DEG2RAD;
  a  = (ap * cosf(p1)) + (bp * sinf(p1));
  bt = (ap * sinf(p1)) - (bp * cosf(p1));
  c  = -cp;

  /* 再转回世界坐标：θ 绕竖轴，TILT_J2 是平面自身的倾斜。
   * 三个基向量是
   *     前  = (cosθ,             sinθ,             0   )
   *     上  = ( s1·sinθ,        −s1·cosθ,         c1  )
   *     法向 = ( c1·sinθ,        −c1·cosθ,        −s1  )
   * 所以直接按分量展开，不需要构造矩阵。 */
  u  = (bt * s1) + (c * c1);
  p2 = theta;
  p3 = (bt * c1) - (c * s1);

  *x_mm = (a * cosf(p2)) + (u * sinf(p2));
  *y_mm = (a * sinf(p2)) - (u * cosf(p2));
  *z_mm = p3;
}
