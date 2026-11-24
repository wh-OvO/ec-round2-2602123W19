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

/* 把 j1_deg 从 norm360 的 **0~360** 绕回机械坐标系的 **−180 ~ +180**。
 *
 * 为什么必须有这一步（2026-11-24 接 0x34 时补的，差点踩雷）：
 *   kin_scs_solve() 内部 out->j1_deg = norm360(theta)，输出 0~360；
 *   而 arm_cfg.c 里 J1 是 deg_min = −180、deg_max = +135。
 *   不绕的后果：θ = 330°（其实就是 −30°）→ j1 = 330° → 被钳到 135°，
 *   底座反着跑一大截，而且**不报任何错**，看起来就是"逆解算错了"。
 *
 * 为什么用**左闭右开** [−180, 180)：+180 要绕成 −180。
 *   两者物理上是同一个方向，但本机 deg_max 只有 +135，+180 根本够不到，
 *   而 −180 是够得到的 —— 绕成 −180 才是限位闸能接受的表示。 */
float kin_scs_wrap180(float deg)
{
  while (deg >= 180.0f)
  {
    deg -= 360.0f;
  }

  while (deg < -180.0f)
  {
    deg += 360.0f;
  }

  return deg;
}

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

  float d;      /* 肩 → 目标的斜距 */
  float cb;     /* 余弦定理：cos β，β 是"大臂"与"肩-目标连线"的夹角 */
  float cg;     /* 余弦定理：cos γ，γ 是两根连杆之间的**内角** */
  float beta;   /* 上面那个 β（度） */
  float gamma;  /* 上面那个 γ（度） */
  float phi1;   /* 大臂的**几何**仰角（度） */
  float bend;   /* 两根连杆之间的**几何**夹角（度）。0 = 伸直 */
  float theta;  /* 底座方位角（度） */
  float phi_c;  /* 换算成舵机命令角之后的大臂仰角 */
  float bend_c; /* 换算成舵机命令角之后的连杆夹角 */

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
   * 两连杆的硬边界。d 太远 = 伸直也够不着；d 太近 = 折到极限也靠不上。
   * 注意 d 是**肩到目标的直线距离**，不是水平距离。 */
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

  /* 平面两连杆：θ 只决定这个平面朝哪，先原样留着；α 直接就是
   * "肩-末端连线"相对水平面的仰角，③ 里的 β 要加在它上面。 */

  /* ---- ③ 平面两连杆：两次余弦定理 ---------------------------------------
   *
   *   这只手是**一个平面里的两连杆**（大臂 L1 + 小臂 L2），所以三角形
   *   一画就出来了：肩 O、肘 E、末端 P，三边是 L1、L2、d。
   *
   *     β = ∠(大臂, OP)      cos β = (L1² + d² − L2²) / (2·L1·d)
   *     γ = ∠(大臂, 小臂)    cos γ = (L1² + L2² − d²) / (2·L1·L2)
   *
   *   注意 γ 是**内角**（两连杆夹出来的那个角），弯得越厉害 γ 越小：
   *     d = L1+L2（伸直）→ cos γ = 1  → γ = 0   → bend = 0
   *     d = |L1−L2|（折死）→ cos γ = −1 → γ = 180 → bend = 180
   *   所以我们要的"弯曲量"是补角：bend = 180° − γ。
   *
   *   ⚠️ bend 和 γ 别搞混：γ 是"两根连杆张开多大"，bend 是"折了多少"。
   *      后面 FK 里写的是 `小臂角 = 大臂角 − bend`，用的是 bend。
   *
   *   用 cos 不用 sin：acos 直接给 [0, π]，不用判象限，边界也不会出岔子。 */
  cb = ((l1 * l1) + (d * d) - (l2 * l2)) / (2.0f * l1 * d);
  cg = ((l1 * l1) + (l2 * l2) - (d * d)) / (2.0f * l1 * l2);

  beta  = acosf(clamp_unit(cb)) * KIN_RAD2DEG;
  gamma = acosf(clamp_unit(cg)) * KIN_RAD2DEG;

  /* 大臂仰角 = 目标仰角 + β（elbow-up：肘抬到 OP 连线上方）。
   * 想换 elbow-down 就把加号改成减号，但那样大臂压低、重力力臂变大，
   * 而且和"解只有一个"的假设冲突，所以本模块不提供。 */
  phi1 = tgt->alpha_deg + beta;
  bend = 180.0f - gamma;

  theta = tgt->theta_deg;

  /* ---- ④ 几何角 → 舵机命令角（扣掉臂本身的角度偏置）---------------------
   *
   *   实测这手是歪的：命令它竖直，大臂实际只有 80°；命令两连杆共线，
   *   小臂实际还多折 20°。见 arm_cfg.h 里 ARM_KIN_J2/J3_BIAS_DEG 那段。
   *
   *   所以想让**物理**摆到 φ1 / bend，命令就得预先偏回去：
   *
   *       命令仰角 = φ1    + BIAS_J2     （手臂实际比命令低，命令就抬高点）
   *       命令夹角 = bend  − BIAS_J3     （手臂实际比命令多折，命令就放直点）
   *
   *   这两个偏置也进 FK（反过来扣），保证"解 → 正解 → 再解"闭环。 */
  phi_c  = phi1 + ARM_KIN_J2_BIAS_DEG;
  bend_c = bend - ARM_KIN_J3_BIAS_DEG;

  /* 偏置把命令角压成负数就没意义了：手臂再怎么放直也只能到 bend = 0
   * （物理上还留着那 20° 的折角），够不着"笔直"。夹住，代价是最远
   * 只到 ~133mm 而不是 148mm —— 这是这台手本身的限制，不是解算的问题。 */
  if (bend_c < 0.0f)
  {
    bend_c = 0.0f;
  }

  /* ---- ⑤ 输出 -----------------------------------------------------------
   *
   *   几何角 → 舵机角的最后一道换算只有"安装偏移 + 方向"：
   *
   *       命令角 = ARM_KIN_Jn_OFF_DEG + ARM_KIN_Jn_SIGN × 几何角
   *
   *   SIGN 只能是 +1 或 −1，对应舵机装反。跟 arm_cfg.c 里用
   *   us_at_deg_min/max 对调来处理装反是一个套路：量出来填数，逻辑不动。
   *
   *   ⚠️ 注意 ④ 里已经把 BIAS 折进 phi_c / bend_c 了，这里**不要再加一次**。 */
  out->j1_deg = norm360(theta);
  out->j2_deg = ARM_KIN_J2_OFF_DEG + (ARM_KIN_J2_SIGN * phi_c);
  out->j3_deg = ARM_KIN_J3_OFF_DEG + (ARM_KIN_J3_SIGN * bend_c);

  return 1u;
}

void kin_scs_fk(const kin_scs_out_t *q, float *x_mm, float *y_mm, float *z_mm)
{
  const float l1 = ARM_KIN_L1_MM;
  const float l2 = ARM_KIN_L2_MM;

  float phi1;   /* 大臂几何仰角（度） */
  float bend;   /* 连杆几何夹角（度） */
  float th;     /* 底座方位角（弧度） */
  float r1;     /* 末端水平距离 */
  float z;      /* 末端高度 */

  if ((q == 0) || (x_mm == 0) || (y_mm == 0) || (z_mm == 0))
  {
    return;
  }

  /* 命令角 → 几何角。SIGN 是 ±1，所以 1/SIGN == SIGN。
   * BIAS 在这里是**减**（解算里是加），两边正好互逆。 */
  phi1 = ((q->j2_deg - ARM_KIN_J2_OFF_DEG) * ARM_KIN_J2_SIGN) - ARM_KIN_J2_BIAS_DEG;
  bend = ((q->j3_deg - ARM_KIN_J3_OFF_DEG) * ARM_KIN_J3_SIGN) + ARM_KIN_J3_BIAS_DEG;

  /* 平面两连杆正解：大臂在 φ1，小臂在 φ1 − bend（elbow-up 时小臂比大臂低）。
   * bend = 0 时两段共线，退回一根 L1+L2 的直线。 */
  r1 = (l1 * cosf(phi1 * KIN_DEG2RAD)) +
       (l2 * cosf((phi1 - bend) * KIN_DEG2RAD));

  z  = (l1 * sinf(phi1 * KIN_DEG2RAD)) +
       (l2 * sinf((phi1 - bend) * KIN_DEG2RAD));

  /* 绕竖轴转到底座方位上。y 分量是底座转出来的，不是摆出来的 ——
   * 整条手臂始终躺在这个竖直平面里。 */
  th = q->j1_deg * KIN_DEG2RAD;

  *x_mm = r1 * cosf(th);
  *y_mm = r1 * sinf(th);
  *z_mm = z;
}
