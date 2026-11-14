# rm_arm — STM32F103C8T6 四轴机械臂主控

长理机甲大师电控二轮考核工程。四个关节（底座 / 肩 / 肘 / 夹爪）的角度控制、多轴插补、
USB CDC 上报与上位机联调全部在本仓库内完成。

- 对应任务：**T0** 板级自检 · **T1** 单轴 PWM · **T2** 多轴联动与插补 · **T3** 标定与限位保护 · **T4** I²C 姿态检测 · **T5a** CDC 虚拟串口
- 选做项 **T5b（Custom HID）/ T6（四态状态机）/ A1（逆解）/ A2（自制手柄）** 未实现，默认砍
- **AI 使用记录见 [AI.MD](AI.MD)**（按任务书附件三规范）

---

## 1. 硬件

| 项 | 型号 / 参数 |
|---|---|
| 主控 | 野火 **STM32F103C8T6** 核心板（双 USB 款），Cortex-M3 @ 72 MHz，**无 FPU** |
| 时钟 | HSE 8 MHz × 9 = 72 MHz，USB 48 MHz，FLASH_LATENCY_2 |
| 舵机 J1 底座 | SG90 改 **360° 连续旋转**（无内置位置环，靠 AS5600 外挂闭环） |
| 舵机 J2 / J3 | MG90S（位置型，内置 PID） |
| 舵机 J4 夹爪 | SG90（位置型） |
| PWM | TIM2 四通道全重映射，50 Hz，脉宽 500~2500 µs，步进 0.5 µs |
| 姿态传感器 | MPU6050（I²C1，3.3 V） |
| 角度传感器 | AS5600 磁编码器（I²C1，3.3 V）—— 用于 J1 位置闭环 |
| 调试串口 | USART1，板载 CH340C，115200 8N1 |

### 1.1 接线表（引脚唯一事实来源）

| 用途 | 引脚 | 外设 | 备注 |
|---|---|---|---|
| J1 底座 PWM | **PA15** | TIM2_CH1 | 全重映射；出厂是 JTDI，靠 `SWJ_NOJTAG` 释放 |
| J2 大臂 PWM | **PB3** | TIM2_CH2 | 全重映射；出厂是 JTDO |
| J3 小臂 PWM | **PB10** | TIM2_CH3 | 装反了对调 µs 值即可，不改代码 |
| J4 夹爪 PWM | **PB11** | TIM2_CH4 | 行程 0~120°，见 `arm_cfg.c` |
| I²C1 SCL | **PB6** | I²C1 400 kHz | MPU6050 与 AS5600 共用 |
| I²C1 SDA | **PB7** | I²C1 400 kHz | 同上 |
| RGB 灯 | **PA1 / PA2 / PA3** | GPIO 输出 | 板载，**低电平点亮** |
| 按键 K1 / K2 | **PA0 / PC13** | GPIO 输入 | **高电平有效**（板上有外部下拉） |
| USART1 TX / RX | **PA9 / PA10** | USART1 | 板载 CH340C，插上即可 `printf` |
| USB DM / DP | **PA11 / PA12** | USB FS | 与串口口是两个独立 Type-C |
| SWDIO / SWCLK | **PA13 / PA14** | SWD | ST-Link 只接这两根 + GND，**别接它的 3.3 V** |

> ⚠️ **供电**：舵机必须用独立电源（12 V → MP1584 → 5 V），与主控共地，
> **禁止**从板子的 3.3 V / 5 V 引脚取电驱动舵机。详见 [`docs/接线与实物数据流.md`](docs/接线与实物数据流.md)。

---

## 2. 工具链版本

| 工具 | 版本 |
|---|---|
| STM32CubeMX | 6.12.1-RC4 |
| STM32CubeCLT | 1.22.0 |
| arm-none-eabi-gcc | 14.3.1 |
| CMake | 4.3.1 |
| Ninja | 1.13.2 |
| STM32CubeProgrammer | 2.23.0 |

工具链文件 `cmake/gcc-arm-none-eabi.cmake` 由 CubeMX 生成，**未手改**（任务书 2.2）。

---

## 3. 构建

### 3.1 构建预设（`CMakePresets.json`）

| 预设 | 说明 |
|---|---|
| **`Debug`** | 默认，`-Og`。**FLASH 41096 B / 64 KB = 62.71%**，RAM 8248 B / 20 KB = 40.27%（2026-11-13 全量构建实测） |
| `Release` | `-O2`，**FLASH 38692 B / 64 KB = 59.04%**，RAM 同上。用于最终烧录 |
| `RelWithDebInfo` / `MinSizeRel` | 保留预设，未做验收 |

### 3.2 命令

`arm-none-eabi-` 必须在 PATH 里。Windows 下（本机实际使用的路径）：

```powershell
$env:PATH = 'D:\STM32CubeCLT_1.22.0\CMake\bin;' + `
            'D:\STM32CubeCLT_1.22.0\Ninja\bin;' + `
            'D:\STM32CubeCLT_1.22.0\GNU-tools-for-STM32\bin;' + $env:PATH

cmake --preset Debug
cmake --build --preset Debug
```

产物：`build/Debug/rm_arm.elf`（另有 `.hex` / `.bin` / `.map`）。

> **必须在工程目录内执行**，否则找不到 `CMakePresets.json`。
> `.gitignore` 已排除 `build/` 与全部编译产物，干净目录克隆后可直接构建。

### 3.3 烧录

```powershell
STM32_Programmer_CLI -c port=SWD -w build\Debug\rm_arm.elf -v -rst
```

---

## 4. 协议实现说明

定长二进制帧，**不用 printf 走 USB**。帧头 `0xA5`，CRC-8（多项式 `0x07`，初值 `0x00`，
不反射，输出不异或）。多字节字段小端序。

```
+------+------+------+-----------+--------+
| SOF  | type | len  |  payload  |  CRC8  |
| 0xA5 |  1B  |  1B  |  len 字节 |   1B   |
+------+------+------+-----------+--------+
```

### 4.1 帧类型

| 类型 | 名称 | 方向 | 载荷长度 | 说明 |
|---|---|---|---|---|
| `0x10` | 状态上报 | 设备 → 上位机 | 19 B | 四轴角度（0.1°）+ 状态位 + 计数，20~50 Hz |
| `0x20` | 目标角度 | 上位机 → 设备 | 11 B | 四轴目标（0.1°）+ 插补时长 + profile |
| `0x21` | 模式切换 | 上位机 → 设备 | 1 B | 手动 / 自动 / 回零 |
| `0x22` | 急停 | 上位机 → 设备 | 1 B | |
| `0x23` | 心跳 | 双向 | 1 B | 1 Hz，超时进入故障 |
| `0x24` | 版本查询 | 双向 | 2 B | 连接时 |
| `0x30` | 传感器原始数据 | 双向 | 1 / 17 B | MPU6050 六轴 + 温度 + pitch + 状态 |
| `0x31` | 单路脉宽设定 | 上位机 → 设备 | 4 B | 调试用：绕开映射直接写 CCR |
| `0x32` | 脉宽回读 | 双向 | 1 / 9 B | |
| `0x33` | 陀螺零偏标定 | 双向 | 1 / 9 B | |

### 4.2 实现分层

```
proto_link.c     CDC 收发（环形缓冲，独占 hUsbDeviceFS）
      ↓
proto_frame.c    CRC8 / 组帧 / 流式解帧      ← 纯逻辑，可在 PC 上单测
      ↓
proto_app.c      协议状态机 + 插补器（5 ms 一拍）
      ↓
arm_joint.c      按 kind 分叉：位置型直通 / 速度型闭环
      ↓                  ↓
arm_pwm.c          as5600.c / mpu6050.c
角度 ↔ 脉宽 ↔ CCR      I²C 传感器
```

关键约定：**`arm_pwm.c` 是纯执行层，永远不知道 I²C 传感器的存在**。
新增业务逻辑加在 `arm_joint.c` 及以上，不要往执行层塞。

### 4.3 插补

- 插补周期 **5 ms（200 Hz）**，与 20 ms 的状态帧**解耦**（`ARM_INTERP_TICK_MS` vs `APP_REPORT_MS`）
- 三档速度规划：`00` 阶跃 / `01` 梯形 / `02` S 曲线
- 四轴按同一进度 `s` 走，**同时出发、同时到达**
- 数值验证（峰均比、二阶差分连续性）见 [`docs/T2_插补增量数值验证.md`](docs/T2_插补增量数值验证.md)

---

## 5. 目录结构

```
Core/Inc, Core/Src    业务代码（CubeMX 生成 + USER CODE 区内的实现）
  ├─ main.c           入口 / 自检 / 日志 / 按键
  ├─ arm_cfg.c/h      四关节标定表 + 所有可调参数（**标定就是改这个文件**）
  ├─ arm_pwm.c/h      角度 ↔ 脉宽 ↔ CCR
  ├─ arm_joint.c/h    关节执行层（kind 分叉、底座闭环）
  ├─ arm_pid.c/h      通用位置式 PID（零依赖，可 PC 单测）
  ├─ as5600.c/h       底座磁编码器
  ├─ mpu6050.c/h      姿态传感器
  ├─ proto_link.c     CDC 链路层
  ├─ proto_frame.c    CRC8 / 组帧 / 解帧
  └─ proto_app.c      协议状态机 + 插补器
Drivers/, Middlewares/, USB_DEVICE/   CubeMX 生成，未改
cmake/                工具链文件（未手改）与 CubeMX 子工程
tests/                PC 端单元测试（宿主机 gcc 直接编译）
docs/                 各专题说明文档
```

---

## 6. 测试

协议层与 PID 层**不依赖 HAL**，可在电脑上用 gcc 直接编译运行：

```powershell
# 协议（CRC8 / 组帧 / 解帧）
gcc -I Core/Inc tests/proto_selftest.c Core/Src/proto_frame.c -o tests/proto_selftest.exe
.\tests\proto_selftest.exe

# PID
gcc -std=c11 -Wall -Wextra -Wpedantic -O2 -ffp-contract=off -I Core/Inc `
    tests/pid_selftest.c Core/Src/arm_pid.c -o tests/pid_selftest.exe
.\tests\pid_selftest.exe
```

| 测试 | 条数 | 内容 |
|---|---|---|
| `tests/proto_selftest.c` | 28 | CRC8 向量、组帧、流式解帧逐字节喂入、参数校验 |
| `tests/pid_selftest.c` | 32 | **逐位回归**（`Ki=Kd=0` 时与旧纯 P 代码 52806 组输出完全一致）、死区清积分、积分限幅、抗饱和冻结、D 低通、无 setpoint kick、斜坡跟踪仿真 |

退出码 `0` = 全过。

---

## 7. 文档索引

| 文档 | 内容 |
|---|---|
| [`docs/接线与实物数据流.md`](docs/接线与实物数据流.md) | 引脚总表、供电规矩、数据流图 |
| [`docs/T1_角度脉宽映射_说明.md`](docs/T1_角度脉宽映射_说明.md) | 角度 → 脉宽换算与标定方法 |
| [`docs/T2_速度曲线实现说明.md`](docs/T2_速度曲线实现说明.md) | 三档速度规划实现 |
| [`docs/T2_插补器逐行导读.md`](docs/T2_插补器逐行导读.md) | 插补器逐行讲解 |
| [`docs/T2_插补增量数值验证.md`](docs/T2_插补增量数值验证.md) | 峰均比 / 二阶差分定量验证 |
| [`docs/T4_MPU6050_说明.md`](docs/T4_MPU6050_说明.md) | MPU6050 驱动与 0x30/0x33 |
| [`docs/AS5600_编码器导读.md`](docs/AS5600_编码器导读.md) | AS5600 原理与接线 |
| [`docs/调试帧_0x31_0x32_标定说明.md`](docs/调试帧_0x31_0x32_标定说明.md) | 手动写脉宽的标定流程 |

---

## 8. 演示视频

> 待录制。按任务书 5.2，按 T0～T5a 及选做项顺序拍摄，总长 5 分钟内，
> 须同时拍到机械臂动作与上位机画面；T4 须拍出拔线进入安全态的过程。
>
> 链接：＿＿＿＿（录制后回填）

---

## 9. 已知限制（如实记录）

1. **底座 J1 仍按位置型（`ARM_JOINT_POS`）运行**：360° 舵机 + AS5600 尚未安装，
   `arm_joint.c` 的速度闭环分支（`vel_tick()`）**当前不执行**。PID 模块已就绪且
   `Ki = Kd = 0`，对现有行为零影响，等硬件到位后标定 `ARM_BASE_ENC_ZERO_DEG` /
   `ARM_BASE_ENC_SIGN` 再翻成速度型。
2. **J2/J3/J4 是开环**：模拟位置舵机内部自带位置环，MCU 只负责"发对数"，
   不构成外部闭环（也不应该在外部再套一层位置环）。
3. **J4 夹爪分辨率**：该路 1 µs 脉宽 ≈ 1.2°，协议 0.1° 精度在这一路无效。
4. **合测**：四路舵机同时驱动需要独立电源（MP1584 → 5 V），USB 供电只够空载单路。

---

## 10. AI 使用记录

**见 [AI.MD](AI.MD)** —— 含工具与版本、逐条使用记录、未采用的方案、本人声明。

自 2026-11-13 起的提交，message 末尾带 `AI:` / `Verified:` 两行；
检索：`git log --grep="^AI:"`。
