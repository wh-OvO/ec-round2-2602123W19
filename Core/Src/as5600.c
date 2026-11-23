/**
  ******************************************************************************
  * @file    as5600.c
  * @brief   AS5600 磁编码器驱动（硬件 I²C1，100kHz）
  ******************************************************************************
  * @note    历史：2026-11-09 之前 I²C1 没打开时这里是个占位实现。
  *          T4 把 MPU6050 接上同一条总线后，AS5600 也一并换成真实现了 ——
  *          两颗器件地址不同（AS5600 = 0x36，MPU6050 = 0x68），不冲突。
  *
  *          现在即使 AS5600 没接，也只是 as5600_init() 返回 0；
  *          只有 J1 的 kind 改成 ARM_JOINT_VEL 时才算底座故障（见 arm_joint.c）。
  *          （2026-11-13 起 J1 已经是 VEL，所以现在没磁铁 = 底座故障。）
  ******************************************************************************
  * @note    AS5600 寄存器速查（数据手册里都能查，别背）：
  *
  *            0x0B  STATUS    bit5 = MD（检测到磁铁） bit4 = ML（磁场太弱）
  *                            bit3 = MH（磁场太强）
  *            0x0C  RAW ANGLE bit3:0 = 角度 bit11:8   ←★ 右对齐，别按左对齐拼
  *            0x0D  RAW ANGLE bit7:0 = 角度 bit7:0
  *            0x0E  ANGLE     bit3:0 = 角度 bit11:8（**缩放后**：过了 ZPOS/MPOS，
  *                                    本工程不用，见下面 AS5600_REG_RAW_H 的注释）
  *            0x0F  ANGLE     bit7:0 = 角度 bit7:0
  *
  *          ★ 正确的拼法（2026-11-18 修，这是"转一圈只动 22°"的元凶）：
  *                raw = ((buf[0] & 0x0F) << 8) | buf[1]
  *            以前写的是 (buf[0] << 4) | (buf[1] >> 4) —— 那是**左对齐**
  *            （16 位 SPI 编码器）的拆法，对右对齐的数据**恰好等价于 raw >> 4**，
  *            白丢低 4 位 → 读数最大只有 255 个计数 → 255 × 0.0879 = 22.4°。
  *
  *          12 位 = 0~4095 对应 0~360°：
  *            角度(度) = 原始值 × 360 / 4096 = 原始值 × 0.087890625
  *
  *          I²C 器件地址 0x36，HAL 里要左移一位 → 0x6C（见下面的 AS5600_ADDR）
  ******************************************************************************
  */

#include "main.h"           /* HAL / I2C_HandleTypeDef */
#include "as5600.h"
#include <stdio.h>          /* 只为了上电那三行诊断打印，见 as5600_init() */

/* hi2c1 定义在 main.c（CubeMX 生成），这里只声明 */
extern I2C_HandleTypeDef hi2c1;

/* 1 = 真正走 I²C 读芯片；0 = 占位，永远返回"不在线"。
 * I²C1 已在 T0 由 CubeMX 生成（PB6/PB7），2026-11-15 起降到 100kHz，所以现在恒为 1。 */
#define AS5600_HW_READY       1

/* 12 位满量程 */
#define AS5600_RAW_MAX        4096u
#define AS5600_DEG_PER_LSB    (360.0f / (float)AS5600_RAW_MAX)

/* 单次 I²C 事务的超时。**必须短**：底座闭环 5ms 一个节拍（`ARM_INTERP_TICK_MS`），
 * 一次读阻塞 2ms 就已经吃掉四成节拍了，再久整个控制周期就崩了。 */
#define AS5600_I2C_TIMEOUT_MS 2u

#define AS5600_REG_STATUS     0x0Bu
/* ★ 用 RAW ANGLE(0x0C)，不要用 ANGLE(0x0E)。
 *   0x0E 是**缩放后**的角度，会被芯片内部 ZPOS(0x01)/MPOS(0x02) 二次加工：
 *       ANGLE = (RAW - ZPOS) * 4095 / (MPOS - ZPOS)
 *   本工程从没写过这两个寄存器，但这颗 clone 的 OTP 里可能残留了值
 *   （之前已实测 0x0B 的 bit0~2 / bit6 是乱码），一旦 ZPOS≈MPOS 就会把
 *   整圈压缩成一小段 —— 现象正是"转一圈读数只动几度"。
 *   0x0C 是 CORDIC 算完的原始角，**完全不经过 ZPOS/MPOS**，这才是 1:1 的。 */
#define AS5600_REG_RAW_H      0x0Cu   /* RAW ANGLE 高 8 位（不缩放） */
#define AS5600_REG_ANGLE_H    0x0Eu   /* ANGLE 高 8 位（缩放后）—— 本工程不用 */
#define AS5600_ADDR           (0x36u << 1)   /* HAL 要求已左移的 8 位地址 */

/* STATUS(0x0B) 的位定义（2026-11-17 实测核对过，别按旧注释填）：
 *   bit5 MD = 检测到磁铁      ← 只有这一位参与"能不能测"的判定
 *   bit4 ML = 磁场太弱（AGC 已到最大增益）
 *   bit3 MH = 磁场太强（AGC 已到最小增益）
 * 注意：ML=1 只是"读数质量差"，芯片仍能出角度；MD=0 才是"没有磁铁"。
 * 实测：同一颗磁铁 远 → 0x13(MD=0,ML=1)，叠一块 → 0x33(MD=1,ML=1)，
 *       贴脸 → 0x67(MD=1,ML=0)，可见 MD 由场强决定。 */
#define AS5600_STATUS_MD      0x20u

/* ---- 不在线时的重新探测节奏（2026-11-17 加）----------------------------
 *
 * 修的是这个坑：原来 s_present 只在 as5600_init() 里置一次 1，上电瞬间
 * 只要 MD=0（磁铁还没装、气隙偏大、或就是抖了一下），s_present 就**永远**
 * 是 0 —— as5600_read_deg() 每拍都直接 return 0u，底座闭环全程报故障，
 * 必须按 RESET 才能解开，而 RESET 会让 USB 重枚举、上位机掉线。
 * 但"有没有磁铁"这件事**是会变的**（磁铁会被装上去、气隙会被调小），
 * 所以判据必须能自己恢复，不能一次判死。
 *
 * 为什么不能每拍都探测：底座闭环 200 Hz，一次 I²C 事务最坏要等
 * AS5600_I2C_TIMEOUT_MS = 2 ms，200 次/秒 = 40% 的 CPU 全耗在超时上。
 * 100 ms 探一次 → 最坏 2% CPU，而恢复延迟只有 0.1 s，人眼无感。 */
#define AS5600_PROBE_INTERVAL_MS  100u

/* 连续读角度失败多少次才判"掉线"。
 * 200 Hz × 20 拍 = 100 ms —— 和上面的探测间隔对齐。
 * 门槛设这么高的理由：偶发一次 NACK（杜邦线抖一下）不该让底座停转；
 * 真要判掉线，就得是"连续 100 ms 都读不到"这种硬故障。 */
#define AS5600_FAIL_LIMIT         20u

static uint8_t  s_present;          /* 1 = 磁铁在场，可测角度 */
static uint32_t s_last_probe_ms;    /* 上次探测 STATUS 的时刻 */
static uint8_t  s_fail_cnt;         /* 连续读角度失败的次数 */

#if AS5600_HW_READY
/* 读 STATUS 判 MD。1 = 前面有磁铁（可测），0 = 没应答 / 没磁铁。
 *
 * 单独抽成函数是因为它有了**两个**调用点：开机初始化、掉线后的定期重探。
 * 两边判据必须一模一样，否则会出现"开机认了、重探不认"的鬼故事。 */
/* 探测，**把两种完全不同的故障分开报**。
 *
 *   2 = 芯片应答，MD=1（有磁铁）→ 正常
 *   1 = 芯片应答，但 MD=0 → 芯片是好的，是**磁铁**太远/偏了/没装
 *   0 = 芯片根本不应答   → 接线 / 地址 / 供电 / 上拉
 *
 * ★ 2026-11-21 为什么要拆：原来只有一个返回值 1/0，把 "MD=0" 和 "芯片没应答"
 *   都压成 0。现场只看到"iic 又坏了、底座不动"，可这两个的修法完全相反 ——
 *   一个是拧螺丝挪磁铁，一个是查线。三个值分开之后，上电串口一眼就能看出来。 */
static uint8_t probe_magnet_ex(void)
{
  uint8_t status = 0xFFu;

  if (HAL_I2C_Mem_Read(&hi2c1, AS5600_ADDR, AS5600_REG_STATUS,
                       I2C_MEMADD_SIZE_8BIT, &status, 1u,
                       AS5600_I2C_TIMEOUT_MS) != HAL_OK)
  {
    return 0u;                         /* 芯片没应答：没接 / 地址错 / 上拉没接 */
  }

  /* 能读寄存器不等于能测角度：MD=0 说明芯片前面没有磁铁，
   * 这时 ANGLE 寄存器给的是噪声，当成"不在线"处理更安全。 */
  return (uint8_t)(((status & AS5600_STATUS_MD) != 0u) ? 2u : 1u);
}

/* 老接口：只要"能不能用"，不要原因。掉线重探那条路用这个。 */
static uint8_t probe_magnet(void)
{
  return (uint8_t)((probe_magnet_ex() == 2u) ? 1u : 0u);
}
#endif

uint8_t as5600_init(void)
{
  s_present  = 0u;
  s_fail_cnt = 0u;

  /* 把"上次探测时刻"往前拨一个间隔，让第一次 as5600_read_deg()
   * 立刻就有资格重探 —— 否则上电到第一次重探之间会白瞎 100 ms。 */
  s_last_probe_ms = HAL_GetTick() - AS5600_PROBE_INTERVAL_MS;

#if AS5600_HW_READY
  {
    const uint8_t r = probe_magnet_ex();

    s_present = (uint8_t)((r == 2u) ? 1u : 0u);

    /* 这三行是"底座不动"的唯一线索，别删。见 probe_magnet_ex() 的注释。 */
    if (r == 0u)
    {
      printf("[T2] AS5600 无应答 (addr 0x36) —— 查接线 / 供电 / 上拉\r\n");
    }
    else if (r == 1u)
    {
      printf("[T2] AS5600 有应答但 STATUS.MD=0 —— 芯片没坏，是**磁铁**没到位\r\n");
    }
    else
    {
      printf("[T2] AS5600 在线 (MD=1)\r\n");
    }
  }
#endif

  return s_present;
}

uint8_t as5600_read_deg(float *deg)
{
  if (deg == 0)
  {
    return 0u;
  }

#if AS5600_HW_READY
  {
    uint8_t  buf[2];
    uint16_t raw;

    /* ---- ① 不在线：按 100 ms 的节奏重探，别一拍一次地刷总线 ---- */
    if (s_present == 0u)
    {
      const uint32_t now = HAL_GetTick();

      /* 无符号相减对溢出安全：tick 绕回 0 时差值语义不变 */
      if ((uint32_t)(now - s_last_probe_ms) < AS5600_PROBE_INTERVAL_MS)
      {
        return 0u;                     /* 还没到下次探测时刻 */
      }

      s_last_probe_ms = now;

      if (probe_magnet() == 0u)
      {
        return 0u;
      }

      /* 磁铁回来了：**本拍**就接着往下读，不用再等下一个节拍 */
      s_present  = 1u;
      s_fail_cnt = 0u;
    }

    /* ---- ② 读角度 ---- */
    /* 一次连续读 2 字节。AS5600 的 12 位角度是**右对齐**分在两个字节里的：
     *   0x0C 的 bit3:0 = 角度 bit11:8   （bit7:4 恒 0）
     *   0x0D           = 角度 bit7:0
     * ★ 2026-11-18 修：原来这里写的是 (buf[0]<<4)|(buf[1]>>4)，那是
     *   **左对齐**（16 位 SPI 编码器）的拼法，对右对齐的数据正好等价于 A>>4，
     *   白白丢掉低 4 位 → 读数最大只有 255 个计数 → 22.4°（"转一圈只动 22°"的元凶）。 */
    if (HAL_I2C_Mem_Read(&hi2c1, AS5600_ADDR, AS5600_REG_RAW_H,
                         I2C_MEMADD_SIZE_8BIT, buf, 2u,
                         AS5600_I2C_TIMEOUT_MS) != HAL_OK)
    {
      /* 连续失败够多才判掉线。判掉之后 ① 会按 100 ms 重探 ——
       * 总线恢复 / 换了根线就能自己活过来，不必再按 RESET。 */
      if (s_fail_cnt < 0xFFu)
      {
        s_fail_cnt++;
      }
      if (s_fail_cnt >= AS5600_FAIL_LIMIT)
      {
        s_present = 0u;
      }
      return 0u;                       /* 读失败：调用者必须保持上一次的脉宽，别乱动 */
    }

    s_fail_cnt = 0u;

    /* 右对齐拼接：0x0C 只用低 4 位（bit11:8），0x0D 是 bit7:0（全 8 位） */
    raw = (uint16_t)((((uint16_t)buf[0] & 0x0Fu) << 8) | (uint16_t)buf[1]);
    *deg = (float)raw * AS5600_DEG_PER_LSB;
    return 1u;
  }
#else
  (void)deg;                           /* 占位实现：不改 *deg，只报告失败 */
  return 0u;
#endif
}

uint8_t as5600_present(void)
{
  return s_present;
}
