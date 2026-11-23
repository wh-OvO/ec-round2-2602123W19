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

  float d;
  float cb;
  float cg;
  float beta;
  float gamma;
  float phi1;
  float bend;

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
   * 注意这里**只**判几何可达，不管关节有没有超程 —— 那是调用方的事。 */
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

  /* ---- ③ 余弦定理 -------------------------------------------------------
   *
   *   三角形 O(肩) - E(肘) - P(末端)，三边 L1 / L2 / d：
   *
   *     β（肩处，OP 与大臂的夹角）  cos β = (L1² + d² − L2²) / (2·L1·d)
   *     γ（肘处，两连杆的内夹角）    cos γ = (L1² + L2² − d²) / (2·L1·L2)
   *
   *   用 cos 而不是 sin 的好处：acos 能直接给出 [0, π] 的角，
   *   不用判断象限，也就不会在 d 接近边界时因为 sin 变号出岔子。 */
  cb = ((l1 * l1) + (d * d) - (l2 * l2)) / (2.0f * l1 * d);
  cg = ((l1 * l1) + (l2 * l2) - (d * d)) / (2.0f * l1 * l2);

  beta  = acosf(clamp_unit(cb)) * KIN_RAD2DEG;
  gamma = acosf(clamp_unit(cg)) * KIN_RAD2DEG;

  /* ---- ④ 取 elbow-up 解 ------------------------------------------------
   *
   *   大臂的**绝对仰角** φ1 = α + β  ← 加号就是"肘抬到 OP 线上方"
   *   换成 elbow-down 只要把加号改减号，但那样大臂会压低、重力力臂变大，
   *   而且和"解只有一个"的假设冲突，所以本模块不提供。
   *
   *   肘的**弯曲量** bend = 180° − γ：两连杆共线时 γ = 180°、bend = 0。 */
  phi1 = tgt->alpha_deg + beta;
  bend = 180.0f - gamma;

  /* ---- ⑤ 重力下垂补偿（11-23 实测：大臂低 10°、小臂多折 20°）-----------
   *
   *   上面两步算出来的是**几何角** —— 手臂"应该"摆成什么样。
   *   可真挂在那儿，重力会把大臂再压低 SAG_J2、把小臂再折进 SAG_J3，
   *   所以命令角得先把这个量补回去（不是加在下垂那一侧，是反着提前补）：
   *
   *       命令 φ1   = 几何 φ1   + SAG_J2      ← 先抬高，掉下来才正好
   *       命令 bend = 几何 bend − SAG_J3      ← 先放平，折进去才正好
   *
   *   ⚠️ J3 这条最容易搞反（"外歪"到底是往折的方向还是往开的方向）。
   *      判据只有实测：命令 J3 折到某个角，看小臂实际停在哪边。
   *      反了就改 arm_cfg.h 里 SAG_J3 的符号，**代码不用动**。
   *
   *   补偿后 bend < 0 说明"这个姿态手臂本来就得伸直，再放平就是反向折了"，
   *   机械上做不到 —— 夹到 0，让手臂保持伸直，那 20mm 的误差留在那儿。
   *   ⚠️ 这是**几何意义上的**夹紧（bend 的定义域本来只有 [0,180]），
   *      不是关节限位；关节超程仍然由调用方拒绝。 */
  phi1 += ARM_KIN_J2_SAG_DEG;
  bend -= ARM_KIN_J3_SAG_DEG;

  if (bend < 0.0f)
  {
    bend = 0.0f;
  }

  /* ---- ⑥ 几何量 → 舵机命令角 -------------------------------------------
   *
   *   两者之间只差一个"安装偏移 + 方向"：
   *
   *       命令角 = ARM_KIN_Jn_OFF_DEG + ARM_KIN_Jn_SIGN × 几何角
   *
   *   SIGN 只能是 +1 或 −1，对应舵机装反。跟 arm_cfg.c 里用
   *   us_at_deg_min/max 对调来处理装反是一个套路：量出来填数，逻辑不动。 */
  out->j1_deg = norm360(tgt->theta_deg);
  out->j2_deg = ARM_KIN_J2_OFF_DEG + (ARM_KIN_J2_SIGN * phi1);
  out->j3_deg = ARM_KIN_J3_OFF_DEG + (ARM_KIN_J3_SIGN * bend);

  return 1u;
}

void kin_scs_fk(const kin_scs_out_t *q, float *x_mm, float *z_mm)
{
  float phi1;
  float bend;
  float a1;
  float a2;
  float x;
  float z;

  if ((q == 0) || (x_mm == 0) || (z_mm == 0))
  {
    return;
  }

  /* 命令角 → 几何角。SIGN 是 ±1，所以 1/SIGN == SIGN。 */
  phi1 = (q->j2_deg - ARM_KIN_J2_OFF_DEG) * ARM_KIN_J2_SIGN;
  bend = (q->j3_deg - ARM_KIN_J3_OFF_DEG) * ARM_KIN_J3_SIGN;

  /* 再把⑤里补的下垂量扣回去，还原成**真实的几何姿态**。
   * ⚠️ 正解描述的是"舵机停在命令角时手臂**实际**在哪"，所以扣回去是对的
   *    —— 这样 solve → fk 才有意义（描述同一个物理姿态）。 */
  phi1 -= ARM_KIN_J2_SAG_DEG;
  bend += ARM_KIN_J3_SAG_DEG;

  a1 = phi1 * KIN_DEG2RAD;
  a2 = (phi1 - bend) * KIN_DEG2RAD;

  x = (ARM_KIN_L1_MM * cosf(a1)) + (ARM_KIN_L2_MM * cosf(a2));
  z = (ARM_KIN_L1_MM * sinf(a1)) + (ARM_KIN_L2_MM * sinf(a2));

  *x_mm = x;
  *z_mm = z;
}
