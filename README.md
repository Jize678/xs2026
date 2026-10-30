# 智能车（微缩负压循迹车）STM32F103C8T6 固件

本工程是开源智能车项目（微缩车模 / stm32c8t6minicar）的固件，
**已按本车模实际 PCB 的引脚分配表（`表格_20260919.csv`，原件见 `docs/` 目录）做过适配**。

本文档记录：实车引脚分配、与原始开源工程的区别、改动内容、编译烧录方法和调试要点。
后续换板、查线、交接都看这一份即可。

---

## 一、硬件平台

| 模块 | 型号 / 说明 |
|------|-------------|
| MCU | STM32F103C8T6（LQFP48，64KB Flash / 20KB RAM），外部晶振 8MHz，SYSCLK 72MHz |
| 电机驱动 | 双路有刷电机 + **两路半桥（单向）** 驱动：每路只有 PWM + 公共使能，**没有方向脚，只能前进** |
| 编码器 | 两路 AB 相增量编码器，用 TIM3 / TIM4 的硬件编码器接口 |
| 循迹 | 16 路光电（灰度）传感器 → 16 通道模拟开关（S0~S3 选通）→ ADC1_IN1 |
| 姿态 | 无（LSM6DSR 六轴 IMU **未安装**；驱动源码 `LSM6DSR_Config.c` 保留在工程里但不被调用，控制环里没有任何 IMU 采样） |
| 显示 | 0.96" OLED（软件 I2C） |
| 负压风机 | 无刷电调（ESC）信号 → TIM2_CH4，50Hz 舵机 PWM（1000~2000us） |
| 人机 | 4 个按键 + 2 颗指示灯（接在 RGB-0/RGB-1 脚上）+ 2 路预留车灯输出（RGB-2/RGB-3 脚） |
| 调试 | SWD（PA13/PA14）+ USART2（PA2/PA3，115200） |

---

## 二、实车引脚分配表

数据来源：`表格_20260919.csv`（本车模现在的引脚分配）。
表中“工程内定义”一列即 `User/Board_Config.h` 中的宏，代码里不再出现写死的端口号。

> 完整版（按功能分组、复用通道、定时器/外设占用、极性、JTAG 复用坑）见 [`引脚分布表.md`](引脚分布表.md)。

| 引脚号 | 芯片引脚 | 网络 / 功能 | 工程内定义 | 说明 |
|--------|----------|-------------|------------|------|
| 2  | PC13-TAMPER-RTC | KEY4 | `KEY_K4_PORT/PIN` | 按键，低电平有效，内部上拉 |
| 3  | PC14-OSC32_IN | KEY3 | `KEY_K3_PORT/PIN` | 按键，低电平有效，内部上拉 |
| 4  | PC15-OSC32_OUT | —— | —— | 未连接（图中绿叉 NC） |
| 5  | PD0-OSC_IN | OSCIN | —— | 外部晶振输入 |
| 6  | PD1-OSC_OUT | OSCOUT | —— | 外部晶振输出 |
| 10 | PA0-WKUP | ADC_BAT | `ADC_BAT_PORT/PIN/CHANNEL` | ADC1_IN0，电池电压检测 |
| 11 | PA1 | ADC_IN | `ADC_IN_PORT/PIN/CHANNEL` | ADC1_IN1，模拟开关输出（灰度） |
| 12 | PA2 | USART2_TX | `UART2_TX_PORT/PIN` | 串口 2 发送 |
| 13 | PA3 | USART2_RX | `UART2_RX_PORT/PIN` | 串口 2 接收 |
| 14 | PA4 | KEY2 | `KEY_K2_PORT/PIN` | 按键 |
| 15 | PA5 | KEY1 | `KEY_K1_PORT/PIN` | 按键 |
| 16 | PA6 | ENC1B | `ENC1_B_PIN` | 编码器 1 B 相 → TIM3_CH1 |
| 17 | PA7 | ENC1A | `ENC1_A_PIN` | 编码器 1 A 相 → TIM3_CH2 |
| 18 | PB0 | S0 | `MUX_S0_PIN` | 模拟开关地址位 S0（最低位） |
| 19 | PB1 | S1 | `MUX_S1_PIN` | 模拟开关地址位 S1 |
| 20 | PB2 | S2 | `MUX_S2_PIN` | 模拟开关地址位 S2 |
| 21 | PB10 | S3 | `MUX_S3_PIN` | 模拟开关地址位 S3（最高位） |
| 22 | PB11 | FAN_PWM | `FAN_PWM_PORT/PIN` | 电调信号 → TIM2_CH4（部分重映射 2，50Hz / 1~2ms） |
| 25 | PB12 | GYRO_NSS | `GYRO_NSS_PORT/PIN` | 陀螺片选：I²C 模式下必须保持高（未装则空置） |
| 26 | PB13 | GYRO_SCK | `GYRO_SCK_PORT/PIN` | 陀螺 SCL（当前固件用位操作 I²C，不再用 SPI2） |
| 27 | PB14 | GYRO_MISO | —— | 空置（SPI 时代的 MISO，I²C 下不用） |
| 28 | PB15 | GYRO_MOSI | `GYRO_MOSI_PORT/PIN` | 陀螺 SDA（当前固件用位操作 I²C） |
| 29 | PA8 | —— | —— | 空出（原 R_DIR，两路半桥后不用） |
| 30 | PA9 | R_PWM | `R_PWM_PORT/PIN` | 右电机 PWM → TIM1_CH2 |
| 31 | PA10 | —— | —— | 空出（原 L_DIR，改两路半桥后不用） |
| 32 | PA11 | L_PWM | `L_PWM_PORT/PIN` | 左电机 PWM → TIM1_CH4 |
| 33 | PA12 | MOTOREN | `MOTOREN_PORT/PIN` | 电机驱动使能 |
| 34 | PA13 | SDIO | —— | SWDIO，调试口，不做 GPIO |
| 37 | PA14 | SCLK | —— | SWCLK，调试口，不做 GPIO |
| 38 | PA15 | RGB-0 | `RGB0_PORT/PIN` | **LED0 指示灯**（高电平点亮） |
| 39 | PB3 | RGB-1 | `RGB1_PORT/PIN` | **LED1 指示灯**（高电平点亮） |
| 40 | PB4 | IIC_D | `OLED_SDA_PORT/PIN` | OLED I2C 数据 |
| 41 | PB5 | IIC_C | `OLED_SCL_PORT/PIN` | OLED I2C 时钟 |
| 42 | PB6 | ENC2B | `ENC2_B_PIN` | 编码器 2 B 相 → TIM4_CH1 |
| 43 | PB7 | ENC2A | `ENC2_A_PIN` | 编码器 2 A 相 → TIM4_CH2 |
| 45 | PB8 | RGB-2 | `RGB2_PORT/PIN` | **预留车灯输出**（已引出，暂不使用） |
| 46 | PB9 | RGB-3 | `RGB3_PORT/PIN` | **预留车灯输出**（已引出，暂不使用） |

### 16 路模拟开关真值表

S3 为最高位，其余为地址低位；S3S2S1S0 = 0000~1111 依次选通 Y0~Y15，
代码中 `MuxADC_SetSelectLines()` 就是按这个顺序写地址线的。

| S3 | S2 | S1 | S0 | 选通 |
|----|----|----|----|------|
| L | L | L | L | Y0 |
| L | L | L | H | Y1 |
| … | … | … | … | … |
| H | H | H | H | Y15 |

---

## 三、运行方式与串口指令

**上电默认是待机（IDLE）**：电机不使能、风扇不转、闭环不动作，车完全静止，
只保留传感器采样与 OLED 显示（第 2 行寻线位置、第 3 行电池电压、第 4 行左右轮速度和），
方便上电时先做检查。

指令通过 USART2（PA2/PA3，115200 8N1）发送，**发单个字符**即可，回车、换行会被忽略
（`l` / `r` / `f` 这三条带数字的指令例外，数字后面用回车结束）。

> 完整速查（含每条指令的回显格式、波形帧通道、常见问题）见 [`串口指令表.md`](串口指令表.md)。

| 指令 | 模式 | 执行内容 | LED0 / LED1 指示 |
|------|------|----------|------------------|
| `0` | IDLE 待机 | 两个电机 PWM 归零 + 使能拉低、风扇停、两个 PID 复位 | 两颗灯同步闪（0.5s 亮 / 0.5s 灭） |
| `1` | TRACK 闭环循迹 | 发车闭环：风扇 95%（`CAR_TRACK_FAN_DUTY`）、使能电机、**串级 PID**——位置环（外环，16 路寻线）算出转向量→换算成左右目标速度→左右**各自一个**速度环（内环，编码器反馈）→各自占空比 | LED0 常亮；丢线时 LED1 快闪 |
| `2` | MOTOR_TEST 电机测试 | 左右电机按**固定测试占空比 30%**（`CAR_TEST_DUTY = 3000` / 10000）：**向前 2 秒 → 停车 2 秒**（`CAR_TEST_FWD_MS` / `CAR_TEST_STOP_MS`），不开风扇；跑满 4 秒自动回到待机 `0` | 两颗灯交替闪，4 秒后恢复待机闪 |

调试指令（排查硬件/引脚用，不影响运行模式）：

| 指令 | 作用 |
|------|------|
| `3` | 两颗指示灯常亮（强制，用来确认灯与引脚） |
| `4` | 两颗指示灯常灭 |
| `5` | 串口回发 PA15 / PB3 的引脚配置字、ODR（软件写的电平）、IDR（引脚真实电平） |
| `l<0~100>` | **左轮单独测试**：只驱动左轮向前，按该占空比转 **2 秒**（`CAR_WHEEL_TEST_MS`），到点自动停车回待机；右轮不转。例如 `l30` 回车 = 左轮 30% 转 2 秒 |
| `r<0~100>` | **右轮单独测试**：只驱动右轮向前，其余同上。例如 `r10` 回车 = 右轮 10% 转 2 秒 |
| `f<0~100>` | 风扇/电调测试：先压 **1.5 秒最低油门**等电调解锁（`CAR_FAN_ARM_MS`，这段电调不转），再按该油门跑 **5 秒**（`CAR_FAN_TEST_MS`），期间两颗灯同频快闪（80ms 亮/80ms 灭）；电机保持禁用，到点自动停风扇回待机。`f` 必须带数字，`f0` = 只闪灯不吹风 |
| `ff<0~100>` | **延时风扇测试**：收到后先空等 **5 秒**（`CAR_FAN_DELAY_MS`；期间电机禁用、风扇保持最低油门、两颗灯 **40ms 亮/40ms 灭超快闪**，`LED_ST_FAN_FAST`），再用该油门跑 5 秒（`CAR_FAN_TEST_MS`），到点自动回待机。适合"先按下录像、再起风"的场景。`ff` 必须带数字，可写 `ff50` 或 `ff 50` |
| `bmq` | **编码器实时监视（开关）**：发一次进入，串口任务每 20ms（`APP_ENC_MON_PERIOD_MS`）打印一行 `enc L=… R=… \| vL=… vR=… \| T=…/…`（左/右原始计数、换算成 mm/s 的速度、上电以来累计），再发一次退出。不改运行模式、不动电机风扇；监视期间 `w` 的波形帧自动暂停。抬手转轮子即可验证 A/B 相、计数方向与是否丢脉冲 |
| `p` | 风机 PWM 诊断：打印 TIM2 的 `PSC/ARR/CCR4/CNT`、PB11 复用配置、TIM2 重映射位（判断"电调不转"是没信号还是信号制式不对） |
| `g` | **灰度 16 路实时监视（开关）**：发一次进入，串口任务每 100ms（`APP_GRAY_MON_PERIOD_MS`）打印一帧——2 行原始 ADC（0-7 / 8-15）+ 2 行黑白判定（0/1，按当前各路阈值）+ 1 行 `pos/found/idx`；再发一次退出（回 `[gray] OFF`）。只读，不动电机与模式；监视期间 `w` 的波形帧自动暂停。用来核对模拟开关地址线（`S0~S3` = PB0/PB1/PB2/PB10，地址值=通道号）到 `Y0~Y15` 的对应关系，以及各路阈值还准不准 |
| `testl<0~4000>` | **左轮速度环标定**：只闭环左轮，目标速度 = **mm/s**（直道基准 2110，上限 4000），10 秒后自动回待机；`s` 可随时打断。期间波形帧暂停，串口任务每 100ms（`APP_SPEED_TEST_PERIOD_MS`）打一行 `[t] T=… F=… E=… P=+… I=+… U=…`（目标 / 反馈 / 误差 / P 增量 / I 增量 / 占空比），目标与反馈都是 mm/s。用来单独看速度环的阶跃响应、超调与饱和 | `left speed test 1500 (10s)` |
| `testr<0~4000>` | **右轮速度环标定**：同上，闭环右轮 | `right speed test 150 (10s)` |
| `mc` | **推车测距（开关，临时标定用）**：第一次发开始清零累计（同时投递 `IDLE`，电机不动），之后每秒打一行 `[mc] t=… L=… R=…`；手推 1 米后再发一次结束，打印两轮净计数 / 绝对值累计 / `cnt/m` / `um/cnt` / 来回拖动量。计数由控制任务在 `ABEncoder_UpdateSpeed()` 之后累加（`g_meas_*`，仅开关打开时），用来把“每米多少 counts”标定出来，之后就能把速度量纲换成 mm/s | `[mc] start, push the car 1m…` |
| `tune` | **速度环调参模式（临时）**：`tune` 进模式（停车、风扇停）→ `vl<mm/s>` / `vr<mm/s>` 设左右目标 → `! <ms> l|r|all` 定时闭环（如 `! 1000 l`）：**不吹负压**（进模式时风扇已回最低油门），发令后闭环立即开始计时，到点自动停、目标清零 → 期间每 20ms（`APP_TUNE_PERIOD_MS = 20`，50Hz）打一行 CSV `ms,tL,fL,uL,tR,fR,uR`；`kpl0.6` / `kil0.5` / `kdl0.05`（`pr/ir/dr` 为右轮）在线改增益，`kq` 查当前参数，`hlt` 临时停车（目标清零、留在模式里），`0` 退出。目标为 0 的一侧不积分，可只调一侧；增益只存 RAM，复位回默认 | `[tune] enter: ! start ...` / `5932,1000,981,4120,1000,1005,3980` |
| `s` | **一键停车**：投递 IDLE 让控制任务停车（与按键 K2 同一条路径），回显 `[stop] send IDLE`；原来的栈水位挪到了 `z` |
| `w` / `?` / `z` | 波形帧开关 / 状态行 / 栈与堆水位（见第十节） |

发 `5` 之后回发的三行里：
`cfg:PA15=0x3 PB3=0x3` 表示两个脚都配成了推挽输出；
`IDR=1 ODR=1` 表示引脚真的被拉高了（这时灯还不亮就是灯那侧的问题）；
`ODR=1` 但 `IDR=0` 说明引脚被别的外设（最常见是 JTAG）占着，软件的写没生效。

发 `0`/`1`/`2` 可以随时退出调试指示、回到正常模式指示。

实现与注意事项：

- 指令在**串口任务**里解析（`App/app_serial.c`），动作由**控制任务**在 2ms 节拍里执行（`Car_Control_Update()`），
  模式定义与指令字符都在 `User/Car_Control.h`。
- `1` 会一直跑到收到 `0` 为止。原来的“0.5s 收不到串口就自动停车”保护已取消
  （否则发完 `1` 立刻就会被停掉），停车请发 `0`。
- 连续约 1s 找不到线（丢线保护）会自动回到待机，需要重新发 `1` 才动车。
- 每次切换模式都会复位速度环和位置环，避免上一次的积分残留。
- 按键：K1 等同于串口 `1`（发车），K2 等同于串口 `0`（停车），K3/K4 暂未分配（原来是切 RGB 颜色，现在颜色灯没有了）。
- 需要串口回显（调试用）时，把 `User/Car_Control.h` 里的 `CAR_CTRL_ECHO` 改成 1，
  每收到一条有效指令就会回发 `mode=x`；用 VOFA+ 画图时请保持 0。

### 两颗指示灯（LED0 / LED1）的闪灯设计

板子上 RGB-0、RGB-1 这两个网络接的其实是**两颗独立的 LED，高电平点亮**；
RGB-2、RGB-3 是引出来给车灯用的，现在不用。所以指示灯按"运行状态"来设计，
而不是按颜色：闪灯逻辑在 `User/Led_Status.c`，节奏参数都是 `Led_Status.h` 里的宏。

| 状态 | LED0（RGB-0 / PA15） | LED1（RGB-1 / PB3） | 含义 |
|------|----------------------|---------------------|------|
| 待机 IDLE | 同步闪：0.5s 亮 / 0.5s 灭 | 同步闪：与 LED0 一起亮、一起灭 | 已上电、等指令 |
| 上电瞬间（自检） | 亮 0.5s（常亮） | 亮 0.5s（常亮） | 用来确认灯与极性正常 |
| 循迹 TRACK | 常亮 | 在线上：灭；丢线：快闪（120ms 周期） | 正在跑 + 找线提示 |
| 电机测试 TEST | 250ms 交替闪 | 250ms 交替闪（与 LED0 反相） | 正在进行电机测试 |
| 异常停车 ALARM | 快闪（160ms 周期） | 快闪（与 LED0 同步） | 例如丢线保护停车，等新指令 |

想改手感就改 `User/Led_Status.h` 里的：

| 宏 | 作用 |
|----|------|
| `LED_IDLE_ON_MS` / `LED_IDLE_PERIOD_MS` | 待机同步闪的亮灯时长 / 周期（两颗灯一起） |
| `LED_LOST_ON_MS` / `LED_LOST_MS` | 丢线提示的亮灯时长 / 周期 |
| `LED_TEST_HALF_MS` | 电机测试交替闪的半周期 |
| `LED_ALARM_HALF_MS` | 异常停车快闪的半周期 |
| `LED_IDLE_BLINK_EN` | 待机不要闪灯时改成 0（两灯全灭） |
| `LED_BOOT_SELFTEST_EN` / `LED_BOOT_SELFTEST_MS` | 上电自检开关 / 自检亮灯时长 |

预留车灯将来接上线以后，可以直接用底层接口控制：

```c
RGB_SetChannel(LAMP0_CH, 1);   /* RGB-2 / PB8 输出点亮电平 */
RGB_SetChannel(LAMP1_CH, 1);   /* RGB-3 / PB9 输出点亮电平 */
```

点亮电平在 `User/RGB_Led.h` 的 `LED_ACTIVE_HIGH` 里统一配置（现在是 1，即高电平点亮），
`User/RGB_Led.c` 里的 `rgb_active_high[]` 也可以逐路单独改；
如果将来车灯驱动是低电平点亮，把对应那一位改成 0 就行。

**指示灯不亮时怎么查**（按顺序）：

1. 上电瞬间两颗灯应同时**常亮 0.5s**（上电自检）。这一段是主循环里直接做的，不依赖中断，
   如果连这一段都不亮，说明是**极性或接线**问题：先把 `User/RGB_Led.h` 的 `LED_ACTIVE_HIGH`
   改成 0 再烧一次（低电平点亮），仍然不亮就量一下 PA15/PB3 到 LED 的线路。
2. 自检亮、之后待机不闪，说明 2ms 中断没在跑：这时候 OLED 也不会刷新、串口发 `2` 电机也不转，
   需要查中断/初始化（正常情况应该是 OLED 有显示、发 `1`/`2` 有反应）。
3. 闪灯节奏不如意：改 `User/Led_Status.h` 里的节奏宏，不用动逻辑。

---

## 四、与原始开源工程的引脚差异（本次适配内容）

把实车引脚表和原始工程代码逐脚对照后，**只有编码器那一处是真正的引脚差异**，其余引脚完全一致；
另外实车这几个“RGB 脚”接的不是 RGB 灯，用法需要重新设计：

| 项目 | 原始开源工程 | 本车模实车 | 本次处理 |
|------|--------------|------------|----------|
| 编码器 1（右轮）A/B 相 | 右轮 A 相接在 CH1：PA6 = ENC1A、PA7 = ENC1B | 右轮 B 相接在 CH1：PA6 = ENC1B、PA7 = ENC1A | A/B 互换会让计数方向相反；**实测（2026-10-29 手推）：前进时左轮原始计数为负、右轮为正**，故 `ENC_L_SIGN = -1`、`ENC_R_SIGN = +1` |
| RGB-0 / RGB-1 的器件 | 当作 RGB 灯的一路，共阳低电平点亮 | **两颗独立的 LED，高电平点亮** | 驱动极性改成高电平点亮；改由 `Led_Status.c` 按运行模式控制闪灯 |
| RGB-2 / RGB-3 的用途 | 当作 RGB 灯的一路 | **引出给车灯用，现在不用** | 仍初始化为输出并保持熄灭，提供 `RGB_SetChannel()` 供将来使用 |

关于编码器方向的说明（结论有依据，但仍请按“第七节第 1 条”实测确认）：

- 原工程固件里右轮速度取 `+TIM3`、左轮速度取 `-TIM4`；原开源车模是 `PA6 = ENC1A / PB6 = ENC2B`，即右侧 A 相在 CH1、左侧 B 相在 CH1。
  配套的原开源车模原理图网名为 `PA6 = ENC1A / PA7 = ENC1B`、`PB6 = ENC2B / PB7 = ENC2A`——
  正好对应“右轮 A 相在 CH1（正计数）、左轮 B 相在 CH1（要取负）”。
- 本车模两路编码器都是 **B 相接在 CH1**（PA6 = ENC1B、PB6 = ENC2B），按此推断两侧都该取负。
- 但 2026-10-29 手推实测（`mc` 指令）推到的结果是：**两轮原始计数方向相反**，前进时左轮原始为负、右轮原始为正；
  所以只需把右轮翻过来 ⇒ 最终 `ENC_L_SIGN = -1`、`ENC_R_SIGN = +1`（`User/ABEncoder.h`），两轮前进时都为正值。

配套的代码改动：

1. **新增 `User/Board_Config.h`**：全工程引脚集中定义（唯一来源），并附带完整引脚表注释。
   电机、编码器、ADC、按键、OLED、指示灯、陀螺仪、串口、风扇各模块都改为引用这里的宏。
2. **`RGB_Led.h / RGB_Led.c`**：把原来“RGB 共阳低电平点亮 + 颜色枚举”的写法改成
   4 路通用输出：`LED0_CH`(PA15) / `LED1_CH`(PB3) 是两颗指示灯，`LAMP0_CH`(PB8) / `LAMP1_CH`(PB9) 是预留车灯；
   点亮电平逐路可配（`rgb_active_high[]`，现为全部高电平点亮）。
3. **新增 `User/Led_Status.c/.h`**：指示灯状态机，按运行模式与丢线情况产生闪灯图案
   （待机同步闪、运行常亮、丢线快闪、测试交替闪、异常快闪），节奏都是一组宏，方便调。
4. **`ABEncoder.h / ABEncoder.c`**：写明 A/B 相与定时器通道的对应关系，
   新增方向补偿宏 `ENC_R_SIGN` / `ENC_L_SIGN`（现为 `ENC_L_SIGN = -1` / `ENC_R_SIGN = +1`，保证“车轮前进 = 速度为正”）。
5. **`main.c`**：按键 K1 = 发车（同 `1`）、K2 = 停车（同 `0`）、K3/K4 暂未分配；
   OLED 第 1 行显示当前模式（IDLE / TRACK / TEST / ALARM）。
6. 其余模块（`Motor_ctr.c`、`M3PWM.c`、`ADC_get.c`、`Key_Scan.c`、`OLED.c`、
   `Uart_Config.c`、`LSM6DSR_Config.c/.h`）只把写死的端口/引脚换成了 `Board_Config.h` 的宏，逻辑未变。

编译验证：Keil MDK（ARMCC V5.06 update 6）全编译 **0 Error / 0 Warning**。

---

## 五、代码结构与占用的外设

| 文件 | 功能 | 引脚 / 外设 |
|------|------|-------------|
| `User/Board_Config.h` | 引脚集中定义（唯一来源） | —— |
| `App/app_main.c` | 启动入口：硬件初始化 → 创建队列/任务 → `vTaskStartScheduler()` | 全部外设 |
| `App/app_ctrl.c` | **2ms 控制任务**：编码器、ADC、寻线、丢线保护、PID、指示灯状态机 | TIM3/TIM4、ADC1、TIM1、TIM2 |
| `App/app_keys.c` | 10ms 按键任务：K1 发车 / K2 停车 / K3 波形帧开关 | PA5/PA4/PC14/PC13 |
| `App/app_display.c` | 50ms OLED 任务：模式 / 位置 / 电池 / 轮速 | PB4/PB5（软 I2C） |
| `App/app_serial.c` | 串口任务：指令解析（队列投递）+ 50Hz 波形帧 | PA2/PA3，USART2 |
| `App/app_state.c/.h` | 指令队列、事件组、任务句柄、共享采样变量 | —— |
| `App/app_config.h` | 任务优先级 / 栈 / 周期 / 队列深度 | —— |
| `System/Delay.c` | 微秒延时（DWT 周期计数器，带 nop 兜底） | —— |
| `User/Motor_ctr.c` | 双电机方向 + PWM + 使能 | PA8/PA9/PA10/PA11/PA12，TIM1 |
| `User/M3PWM.c` | 负压风机 **电调** 信号（50Hz 舵机 PWM，1000~2000us） | PB11，TIM2_CH4 |
| `User/ABEncoder.c` | 两路编码器测速 | PA6/PA7（TIM3）、PB6/PB7（TIM4） |
| `User/ADC_get.c` | 16 路灰度扫描 + 电池电压 | PB0/PB1/PB2/PB10（地址）、PA1/PA0（ADC1） |
| `User/BlackPoint_Finder.c` | 16 路归一化寻线、重心插值定位 | 由 ADC 模块供数 |
| `User/PID_Controller.c` | **串级 PID**：位置环（外环）→ 左右两个速度环（内环） | 由电机/编码器/寻线模块供数 |
| `User/Car_Control.c` | 运行模式控制：串口 `0/1/2` 指令、发车、电机测试、停车 | 由电机/风扇模块执行 |
| `User/Key_Scan.c` | 4 按键扫描（非阻塞，含事件队列） | PA5/PA4/PC14/PC13 |
| `User/Led_Status.c` | 两颗指示灯的闪灯状态机（按运行模式） | 通过 `RGB_Led.c` 驱动 |
| `User/RGB_Led.c` | 4 路输出的底层驱动：2 颗指示灯 + 2 路预留车灯（点亮电平逐路可配） | PA15 / PB3 / PB8 / PB9 |
| `User/OLED.c` | 0.96" OLED 驱动（软件 I2C） | PB4/PB5 |
| `User/LSM6DSR_Config.c` | LSM6DSR 驱动（**未启用**：陀螺没装，工程里没有任何调用） | PB12~PB15（位操作 I²C，备用） |
| `User/pose.c` | 姿态解算（四元数/互补滤波，备用） | 由 IMU 供数 |
| `User/Uart_Config.c` | 串口 2（115200）：中断收 + 环形缓冲 + 发送互斥 + printf 重定向 | PA2/PA3，USART2 |
| `User/stm32f10x_it.c` | 中断向量：只保留内核异常（HardFault 等）；SVC/PendSV/SysTick 由 FreeRTOS 接管 | —— |

---

## 六、引脚集中定义：`User/Board_Config.h`

以后改板只需要动这一个文件，例如把风扇 PWM 挪到别的脚：

```c
#define FAN_PWM_PORT          GPIOB
#define FAN_PWM_PIN           GPIO_Pin_11   /* 引脚 22 -> TIM2_CH4 */
```

改完重新编译即可，业务代码（`M3PWM.c` 等）不用改。
需要同时改定时器/重映射时，也可以用同一个文件里的 `FAN_PWM_TIM`、`FAN_PWM_REMAP` 等宏。

---

## 七、编译与烧录

1. 环境：Keil MDK5（ARM Compiler V5.06 update 6，C99），器件包 `Keil.STM32F1xx_DFP`。
2. 打开 `Project.uvprojx`，Target 选择 `Target 1`（器件 STM32F103C8，IROM 0x08000000/64KB，IRAM 0x20000000/20KB）。
   工程分 6 组：`Start` / `Library` / `System` / `User` / `App` / `Middlewares`（后两组是 FreeRTOS 改造新增的）。
3. 编译：`Project → Build`，输出 `Objects\Project.axf`。命令行等价写法：
   `"<Keil安装目录>\UV4\UV4.exe" -r "<工程绝对路径>\Project.uvprojx" -j0 -o "<日志路径>"`。
   注意：用 PowerShell 的 `Start-Process` 调 UV4 时，**整条参数要作为一个字符串**传进去；
   拆成数组会把带空格的路径拆散，UV4 直接以退出码 41 退出且不写日志。
4. 下载：J-Link / SWD 接 PA13(SWDIO)、PA14(SWCLK)、GND、3V3。
   代码里调用了 `GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE)`（关闭 JTAG、保留 SWD），
   所以 PA15/PB3/PB4 可以当普通 GPIO 用，同时不影响 SWD 调试和下载。

---

## 八、上电自检与调试要点

1. **电机方向自检（先做这一步）**
   把车架空，串口发 `2`：两个轮子应同时**向前**转（30% 占空比，持续 2 秒后自动停），两颗指示灯交替闪；
   发 `0` 停车，指示灯回到待机（两颗灯同步闪，0.5s 亮 / 0.5s 灭）。
   若某个轮子转向相反，把该电机的两根线对调即可。
2. **测速方向自检**
   架空状态下手动将两个轮子向前转，OLED 第 4 行显示的是 `speed_left + speed_right`，应为**正值**。
   若为负值，说明该侧编码器方向反了：把 `User/ABEncoder.h` 里对应的
   `ENC_R_SIGN`（右轮）或 `ENC_L_SIGN`（左轮）改成相反的符号再编译。
   （本车模两路编码器都是 B 相接在 TIM3/TIM4 的 CH1 上，与常见“A 相接 CH1”接法相反，因此这里做了取反。）
3. **电调 / 负压风机自检（新）**
   先发 `p` 看信号制式：最后一行必须是 `RESULT: PASS`（即 `mode=SERVO 50Hz`、`PSC=71`、`ARR=19999`、
   `TIM2_REMAP=2`、`CRH[15:12]=0xB`、`CCR4` 在 1000~2000、`CNT` 在变，六项都 OK）。确认无误后发 `f50`：前 1.5 秒是解锁段（**电调不转是正常的**），
   这时应听到电调"嘀—嘀"两声确认；随后 5 秒按 50% 油门吹风，两灯同频快闪；5 秒后自动停。
   完全没有解锁声 → 查信号线有没有接到 **PB11**、电调地与主板地是否共地、电调是否要求更高电平（3.3V 一般够）。
4. **发车 / 停车**：串口发 `1`（或按 K1）发车，LED0 常亮、风扇 95%、电机使能；
   发 `0`（或按 K2）停车，回到待机。K3/K4 暂未分配。
5. **看灯判断状态**：上电自检两颗灯常亮 0.5s = 灯路正常；两颗灯同步闪 = 待机等指令；LED0 常亮 = 正在循迹；
   LED1 快闪 = 当前丢线在找线；两灯交替闪 = 电机测试；两灯同频快闪 = 异常停车（丢线保护）或风扇测试。
6. **OLED 显示**：第 1 行 = 当前模式（IDLE / TRACK / TEST / ALARM），
   第 2 行 = 寻线位置（×10），第 3 行 = 电池电压（×100），第 4 行 = 左右轮速度和。
7. **电池电压换算**：`BDI_V = ADC(PA0) × 0.004265…`，其中系数由分压电阻决定
   （上游原理图 `V_OUT = (R7/R6 + 1) × Vref`，Vref = 0.8V），换板后需要重新标定。
8. **保护逻辑**：连续 500 次（约 1s）没找到线 → 自动回到待机（禁电机、风扇停），
   并且指示灯变成两灯快闪的“异常停车”，提示是丢线停的车；
   需要重新发 `1` 才动车；串口相关的自动停车保护已取消。
9. **灰度标定**：`BlackPoint_Finder.c` 里 16 路的 `min_value`（白底）/`max_value`（最黑）是实测值，
   换传感器或换高度后要重新标定；阈值判定为归一化值的 20%。
10. **IMU 未安装（2026-10-29 起）**：固件里已经没有任何 IMU 采样——控制环不读陀螺，
    波形帧回到 5 通道（CH3 陀螺 Z 恒 0），串口 `hx` / `i` / `x` / `c` 四条诊断指令已删除。
    `User/LSM6DSR_Config.c` 只是留在工程里备查（位操作 I²C 的实测结论都写在里面），
    重新装传感器时再接回 `App_CtrlTask`。

---

## 九、注意事项

- **PC13/PC14/PC15 当作按键和 NC 使用**，所以本车模没有 32.768kHz 晶振，
  不要启用 LSE / RTC，也不要使用 `BKP` 相关功能。
- 定时器已全部占用：TIM1 = 电机 PWM，TIM2 = 电调信号，TIM3/TIM4 = 两路编码器。
  如需再加 PWM 输出，要考虑重映射或换用软件方案。
- **PB11 上挂的是无刷电调（ESC），必须给 50Hz、1000~2000us 的舵机信号**，给 17kHz 占空比 PWM 它是
  完全不理的——但上电瞬间照样会用电机绕组鸣叫报电池节数，非常容易误判成"信号正常"。电调还要在
  最低油门（1ms）保持约 1 秒才会"嘀—嘀"解锁，解锁前推油门不转，所以 `M3PWM_Init()` 末尾和 `f`
  指令里都有 1.5s 的最低油门保持（`FAN_ARM_DELAY_MS` / `CAR_FAN_ARM_MS`）。
- **控制周期是 2ms**（`App/app_config.h` 的 `APP_CTRL_PERIOD_MS`，由 `vTaskDelayUntil` 保证）。
  业务代码在 `App/app_ctrl.c` 的 `App_CtrlTask` 里，**不再跑在 `SysTick_Handler`**——
  `SysTick` 归 FreeRTOS 调度器用（1kHz tick）。控制任务里完成：IMU 读取与转换、编码器测速、
  16 路 ADC 扫描（每路切换后 `Delay_us(3)` 并丢弃一次采样）、寻线定位、丢线保护、PID、模式执行。
  循环体目前约 0.5ms，往里加东西要注意别挤爆 2ms 节拍。
- PID 参数就是按 2ms 周期调的，想改成 1ms 需要重新调 PID。
- **速度环调参只许单侧电机通电**（2026-10-30 定）：单轮驱动时车走固定半径圆环、行程有界，
  不需要跑道也不会冲出去；两轮同时驱动就是直行，会一直加速到撞东西。所以调参只用
  `vl<目标>` + `! <ms> l`，**禁止 `1`（TRACK）、`2`（MOTOR_TEST）、K1 和任何 `r…`/`vr…` 指令**。
  详见 `速度环调参说明.md` §9.0。
- 初始化顺序：`App_HardwareInit()`（全部外设）→ `App_StateInit()`（队列/事件组）→
  `App_CreateTasks()` → `vTaskStartScheduler()`。外设都在调度器启动前初始化完，
  任务里不会碰到没初始化的 ADC/定时器。
- 指示灯引脚（PA15 / PB3）依赖 `GPIO_Remap_SWJ_JTAGDisable` 关闭 JTAG。
  因为风扇的 TIM2 部分重映射、OLED 初始化都会去写 `AFIO->MAPR`，而 `SWJ_CFG` 是只写位，
  所以 `App/app_main.c` 里的 `App_HardwareInit()` 在所有外设初始化完成后再调用一次 `RGB_EnsureUsable()`（关 JTAG + 重新配成推挽输出），
  确保这两条腿一定是普通 GPIO。以后再加涉及重映射的外设，也要放在这一个调用之前。
- **RGB-0 / RGB-1 上的两颗指示灯是高电平点亮**（和常见的共阳 RGB 相反），
  点亮电平在 `User/RGB_Led.c` 的 `rgb_active_high[]` 里逐路配置；
  RGB-2 / RGB-3 是预留车灯，初始化后保持熄灭，将来接上驱动电路再用 `RGB_SetChannel()` 控制。
- PA15 / PB3 / PB4 在初始化时必须先关闭 JTAG（`GPIO_Remap_SWJ_JTAGDisable`）才能当普通 GPIO；
  关掉 JTAG 不影响 SWD 下载调试。
- **电机只有前进一个方向**（2026-10-28 起）：驱动换成**两路半桥**后，方向脚（PA8 / PA10）
  已经从固件和接线里彻底去掉，这两个脚现在是空脚。代码里任何负速度（PID 混控后某一侧算出负值）
  都在 `Motor_SetSpeedClamped()` 里按 **0** 处理——也就是说极限打舵时是**一侧停转**，不是倒转。
  换驱动前留下的 `MOTOR_DIR_LEVEL_SWAP` 以及"右电机不转"那个坑已随之作废，不用再管。
- **编码器必须可用才能发车**：速度环的反馈来自编码器。编码器没接/坏了的话，发 `1` 后反馈速度恒为 0，
  速度环会在十几毫秒内把占空比加到上限，车会直接窜出去。上赛道前务必确认手推车轮时速度有正负变化。
- `Objects/`、`Listings/` 是 Keil 的编译产物，可随时删除重建（有 `keilkill.bat`）。

---

## 十、FreeRTOS 改造（2026-10-20）

### 10.1 结果

- 内核：FreeRTOS **V10.5.1**，只取 `tasks.c / queue.c / list.c / timers.c / event_groups.c`；
  移植层用 `portable/RVDS/ARM_CM3`（兼容 ARMCC AC5），内存用 `heap_4.c`，堆 **7KB**。
- 编译：**0 Error / 0 Warning**；ROM **30.16KB / 64KB**，RAM **8.93KB / 20KB**（含 FreeRTOS 堆 7KB）。
- 行为不变：上电自检、待机心跳、`0`~`5` 指令、K1/K2、OLED 四行、丢线约 1s 回待机、
  切模式复位两个环、风扇 95%、占空比限幅，全部与原实现一致。

### 10.2 任务划分

| 任务 | 优先级 | 栈(words) | 周期 | 职责 |
|------|-------|----------|------|------|
| `App_CtrlTask` | 4 | 256 | 2ms | IMU / 编码器 / ADC / 寻线 / 丢线保护 / PID / 电机 / 风扇 / 指示灯状态机 |
| `App_KeysTask` | 3 | 128 | 10ms | `Key_Scan_Update`，K1 发车、K2 停车、K3 波形帧开关 |
| `App_DisplayTask` | 2 | 256 | 50ms | OLED 四行刷新 |
| `App_SerialTask` | 2 | 288 | 事件驱动 | 指令解析、50Hz 波形帧 |
| `IDLE` / 定时器服务 | 0 / 3 | 128 | — | 空闲任务、软件定时器 |

### 10.3 任务间通信

- **指令队列**（8 深，`AppCmd_t`）：串口/按键任务 → 控制任务，统一在 2ms 节拍开头处理。
- **事件组** `g_app_events`：`LINE_FOUND` / `LINE_LOST` / `ALARM` / `WAVE_ON`，串口任务据此决定发不发帧。
- **任务通知**：`USART2_IRQHandler` 收到字节后 `vTaskNotifyGiveFromISR()` 唤醒串口任务，
  不再在主循环里轮询环形缓冲。
- **递归互斥量**：串口发送加锁，`printf` 与波形帧不会互相截断。
- **软件定时器**：控制任务看门狗 20ms，控制任务每拍 `xTimerReset()`；一旦卡死自动断电机 +
  停风扇 + 进 ALARM。

### 10.4 与原实现的差异

| 项 | 原实现 | 现实现 | 原因 |
|----|--------|--------|------|
| 2ms 业务 | `SysTick_Handler` 里全做完 | `App_CtrlTask` + `vTaskDelayUntil` | 中断只做最小工作，周期由内核保证 |
| 串口接收 | 主循环轮询环形缓冲 | 中断通知 + 阻塞等待 | 没数据时 CPU 不空转 |
| 串口发送 | 阻塞等 TXE，无保护 | 阻塞 + 递归互斥量 | 波形帧和调试信息不交错 |
| `Delay_us` | 6 个 `nop` 空转（依赖 -O1） | DWT `CYCCNT` 计数 + nop 兜底 | 优化等级和抢占都不再影响精度 |
| 待机分支 | 每 2ms 重写电机/风扇寄存器 | 只在 `ApplyMode()` 切模式时写 | 去掉 500 次/秒无谓寄存器写 |
| 栈/堆监控 | 无 | `configCHECK_FOR_STACK_OVERFLOW=2` + `z` 指令 | 便于长期跑车排查 |

### 10.5 新增串口指令与波形帧

- `w` / `W`：开关 50Hz 波形帧。格式 `0x00 0x00` + 5 个 `float`（寻线位置/10、左轮速、右轮速、
  陀螺 Z、电池电压）+ 尾部 `0x00 0x00 0x80 0x7F`（float NaN），共 24 字节，115200 下约 4.8KB/s。
  陀螺没装，CH3 恒为 0；只想看别的量时改 `App/app_serial.c` 的 `Serial_SendWave()`
  （通道表就在那 5 行里），通道数改了要同步 `App/app_config.h` 的 `APP_WAVE_CHANNELS`。
- `l<0~100>` / `r<0~100>`：单轮测试。只驱动一侧轮子（另一侧 0 占空比）向前跑 2 秒，
  用来分辨"不转"到底是哪一侧的问题（电机 / 驱动 / 接线 / 编码器），也用来找电机起动死区。
- `f<0~100>`：负压风扇/电调测试：1.5 秒最低油门解锁（电调会"嘀—嘀"两声）→ 按该油门跑 5 秒 → 停。
  两灯同频快闪，不带电机转，用来单独验证风道、电调供电与油门标定。
- `ff<0~100>`：延时风扇测试：先空等 5 秒（`CAR_FAN_DELAY_MS`，电机禁用、风扇最低油门、
  两灯 40ms/40ms 超快同频闪），再按该油门跑 5 秒后自动停。等待期就是留给操作者
  “按下录像 / 手离开桨叶”的时间；超快闪（`LED_ST_FAN_FAST`）和普通 `f` 的 80ms 快闪能一眼区分。
- `bmq`：编码器实时监视开关，解析器里用 `s_word_state` 单独匹配这个三字符指令（不区分大小写，
  不用回车）。进入后每 20ms 打一行 `enc L=… R=… | vL=… vR=… | T=…/…`，只读不写外设，
  所以循迹跑着也能看；监视期间自动暂停 `w` 的二进制帧，避免文本行和 JustFloat 帧互相打断。
- `p`：风机 PWM 诊断。`mode=SERVO 50Hz`、`ARR=19999`、`CCR4=1000~2000`、`CNT` 在跳变，
  才说明"50Hz 舵机信号真的送到了 PB11"；`TIM2_REMAP=2` 才说明 CH4 确实映射到了 PB11。
- `?`：回一条状态行（模式、位置、电池、轮速、指令队列剩余、事件组标志）。
- `z`：回四个任务的栈高水位（words，越小越危险）和剩余堆字节数（原来挂在 `s` 上）。
- `s`：**一键停车**，投递 `APP_CMD_MODE(CAR_MODE_IDLE)`，与按键 K2 同一条路径。

### 10.6 目录变化

```
App/                    新增：app_main.c / app_ctrl.c / app_keys.c / app_display.c /
                              app_serial.c / app_state.c / app_state.h / app_config.h
Middlewares/FreeRTOS/   新增：内核 + portable/RVDS/ARM_CM3 + heap_4.c + FreeRTOSConfig.h
Backup/                 User_main.c.legacy：旧版 main.c（已移出工程，留作对照）
User/main.c             已删除（内容迁到 App/app_main.c）
User/stm32f10x_it.c     只保留内核异常；SVC/PendSV/SysTick 由 port.c 提供
User/Uart_Config.c      重写：中断收 + 环形缓冲 + 发送互斥 + 任务通知
User/Car_Control.c      删除 Car_Control_Task()，指令解析搬到 App/app_serial.c；待机分支不再每拍写寄存器
System/Delay.c          改用 DWT 周期计数器（带 nop 兜底）
```

### 10.7 已知取舍

- 串口仍是阻塞发送（按需求保留），最坏情况串口任务占用二三十毫秒；它在优先级 2，不影响 2ms 控制任务。
- OLED 50ms 刷新，显示有 0~50ms 延迟；软 I2C 仍用 `Delay_us` 做时序。
- 陀螺仪未安装（2026-10-29 起）：控制环里没有 IMU 采样，`LSE6DSR_data` 恒 0，PID 里的陀螺阻尼项（2800×gz）等于没开。
- 编码器缺失时发车仍会窜车（原逻辑保留），上赛道前必须手推车轮确认速度有正负。

---

### 10.8 速度/距离单位与里程标定（2026-10-29）

原来速度环的目标和反馈都是“编码器每 2ms 的计数增量”（counts/2ms），直道基准写死 230。
现在统一改成物理单位 **mm/s**，counts 只在 `speed_left` / `speed_right` 里保留做诊断。

**标定**：手推车走 1 米，测 5 次（左右轮各一个），共 10 个数；去掉最大值、最小值后取平均
→ **54476 counts/m**（定义在 `User/ABEncoder.h` 的 `ENC_COUNTS_PER_METER`）。

| 换算 | 值 |
|-|-|
| 1 count | 18.357 µm |
| 1 count/2ms | 9.1784 mm/s |
| 直道基准 230 counts/2ms | 2110 mm/s（≈2.1 m/s，7.6 km/h） |
| 弯道最慢 130 counts/2ms | 1190 mm/s |

**代码落点**：
- `User/ABEncoder.h`：`ENC_COUNTS_PER_METER` / `ENC_MM_PER_COUNT` / `ENC_COUNTS_TO_MM_S`；
  `ABEncoder.c` 里 `speed_left_mm_s` / `speed_right_mm_s` 由计数乘系数得到；
- `User/PID_Controller.c`：`PID_BASE_FAST 2110`、`PID_BASE_SLOW 1190`（mm/s），斜坡 `4.6 mm/s / 2ms`；
  速度环增益改成 **0.60 / 0.56 / 0.63**（单位 duty/(mm/s)，等效于旧 5.5/5.1/5.8 duty/(count·2ms⁻¹)）；
- `User/Car_Control.h`：`CAR_SPEED_TEST_MAX` 400 → 4000（mm/s）；
- 串口：`?` 的 `spd=`、`bmq` 的 `vL/vR`、`testl/testr` 的目标、波形帧 CH1/CH2、OLED 速度行都是 mm/s。

**验证**：车架起来发 `testl1000`，应看到 `[t] T=1000 F≈1000`；或再推 1 米发 `mc`，
结束报告里的 `vs cal` 应接近 `100%`（偏离说明标定或轮径变了）。


## 十一、变更记录

| 日期 | 内容 |
|------|------|
| 2026-09-19 | 依据实车引脚表（`表格_20260919.csv`）适配：新增 `Board_Config.h` 集中管理引脚；RGB 由 3 路扩展为 4 路（新增 PB9/RGB-3）；编码器 1 的 A/B 互换做方向补偿；`main.c` 改用枚举常量；全工程编译 0 Error / 0 Warning |
| 2026-09-19 | 增加串口指令控制：新增 `Car_Control.c/.h`，上电进入待机（不跑车），`1` 运行闭环循迹、`2` 双电机 20% 正转测试、`0` 停止；取消“0.5s 无串口自动停车”保护，丢线保护改为回到待机 |
| 2026-09-19 | 按实车改指示灯：RGB-0/RGB-1 实为两颗**高电平点亮**的 LED（RGB-2/RGB-3 是预留车灯输出），新增 `Led_Status.c/.h` 按运行模式设计闪灯（待机心跳 / 运行常亮 + 丢线快闪 / 测试交替闪 / 异常快闪）；K1 发车、K2 停车，OLED 第 1 行显示模式 |
| 2026-09-19 | 待机指示改为 **PA15 与 PB3 两颗灯同步闪**（原来只有 PA15 心跳闪） |
| 2026-09-19 | 待机闪灯改成看得清的 0.5s 亮 / 0.5s 灭节奏，并新增上电自检（两颗灯常亮 0.5s）与 `LED_ACTIVE_HIGH` 极性开关，便于排查灯不亮的问题 |
| 2026-09-19 | 排查指示灯不亮：新增 `RGB_EnsureUsable()`（初始化最后再关一次 JTAG 并重配 PA15/PB3），新增调试指令 `3`/`4`/`5`（两灯常亮 / 常灭 / 回发引脚配置与 IDR、ODR） |
| 2026-09-19 | 修丢线保护 bug：原来待机/电机测试也计时，导致台架上发 `1`/`2` 立刻被打回待机；现在只在循迹(TRACK)模式计时且切模式清零。`2` 电机测试改为**跑满 1 秒自动回到待机**；`5` 诊断增加电机状态一行并改为纯 ASCII |
| 2026-09-19 | 电机测试按实车标定：占空比 20% → **30%**，持续时间 1 秒 → **2 秒** |
| 2026-09-19 | 试过把两个方向脚电平整体取反（`MOTOR_DIR_LEVEL_SWAP=1`）让车向前，实测**右电机不转**（右驱动侧把 PA8=0 当不使能），已改回 **0** 恢复原方向；两轮向前应改为**对调右电机的两根电机线** |
| 2026-09-19 | `2` 电机测试改成**时序测试**：向前 2 秒 → 向后 2 秒 → 自动回待机 `0`（`CAR_TEST_FWD_MS` / `CAR_TEST_BWD_MS` 可调），占空比仍为 30% |
| 2026-10-20 | **迁移到 FreeRTOS V10.5.1**：2ms 业务从 `SysTick_Handler` 迁到 `App_CtrlTask`（`vTaskDelayUntil`）；新增 `App/` 任务层（控制 / 按键 / 显示 / 串口 / 状态）与 `Middlewares/FreeRTOS`；`Uart_Config.c` 改中断通知 + 环形缓冲 + 发送互斥；`Delay_us` 改 DWT 周期计数；新增 `w`（波形帧）/ `?`（状态）/ `s`（栈与堆水位）指令；编译 0 Error / 0 Warning，ROM 30.16KB、RAM 8.93KB |
| 2026-10-20 | 删除 `User/main.c`（移到 `Backup/User_main.c.legacy`），入口改为 `App/app_main.c`；删除 `Car_Control_Task()`，指令解析搬到 `App/app_serial.c`；待机分支不再每 2ms 重复写电机/风扇寄存器；控制任务看门狗 20ms，卡死自动断电机 |
| 2026-10-20 | 串口新增运行时调试指令：`d<0~100>` 改电机测试占空比（测试模式中立即生效）、`j<0~100>` 立即点动 2 秒、`f<0~100>` 风扇跑 5 秒（新增 `LED_ST_FAN_TEST`，两灯同频快闪，到点自动停）；数字输入用"字母 + 数字 + 回车"状态机，`d`/`j` 只发字母回车时按查询处理 |
| 2026-10-22 | 电机调试指令改版：**取消 `d`（改测试占空比）和 `j`（点动）**，改为 **`l<0~100>` / `r<0~100>` 单轮测试**——只驱动一侧轮子向前跑 2 秒（`CAR_WHEEL_TEST_MS`），另一侧保持不转，到点自动回待机。`2` 电机测试的占空比恢复为固定 30%（`CAR_TEST_DUTY`），不再运行时可调；新增 `Car_Control_WheelRun()` 与 `Car_Wheel_t`，移除 `Car_Control_Jog/SetTestDuty/GetTestDuty` 与 `s_jog_ticks/s_test_duty`。编译 0 Error / 0 Warning |
| 2026-10-23 | 新增串口指令 **`ff<0~100>` 延时风扇测试**：收到后先空等 5 秒（`CAR_FAN_DELAY_MS`），再用该油门吹 5 秒（`CAR_FAN_TEST_MS`）后自动停；等待 + 吹风全程两颗灯**超快同频闪**（新增 `LED_ST_FAN_FAST`、`LED_FAN_FAST_HALF_MS = 40`，与 `f` 的 80ms 快闪区分）。新增 `APP_CMD_FAN_DELAY_TEST`、`Car_Control_FanDelayedTest()`、`s_fdelay_ticks/s_frun_ticks/s_fdelay_duty`（`ApplyMode` 切模式时清零，可被 `0/1/2` 打断）；数字输入态里第二个 `f` 切换为延时版，数字前的空格被忽略（`ff 50` 可用）。编译 0 Error / 0 Warning，ROM 32.44KB、RAM 8.94KB |
| 2026-10-23 | 新增串口指令 **`bmq`（编码器实时监视开关）**：进入后 serial 任务每 20ms（`APP_ENC_MON_PERIOD_MS`）打印 `enc L/R`（TIM4/TIM3 原始计数）、`vL/vR`（每 2ms 增量）、`T`（累计总计数），再发一次退出；进入/退出各回一行 `[enc] ON` / `[enc] OFF`。新增 `APP_CMD_ENC_MON` 与 `g_enc_mon`（ctrl 任务翻转、serial 任务消费），三字符指令用 `s_word_state` 状态机匹配（不匹配的字符仍按单字符指令解析）；监视期间自动暂停波形帧。编译 0 Error / 0 Warning，ROM 32.80KB、RAM 8.94KB |
| 2026-10-24 | **修正灰度模拟开关地址线（重要）**：实车 `S0~S3` = PB0 / PB1 / PB2 / PB10（`S0` 为最低位，`S3` 最高位，地址值 = 通道号），原文档与代码误写成 `S0=PB10、S1=PB2、S2=PB1、S3=PB0`——两根地址线对调，导致**每路灰度实际采样到的是 `Y(bitrev(n))`**（通道 n ↔ n 的位反转），16 路顺序被彻底打乱。已改 `Board_Config.h` 的 `MUX_S0..S3_PIN` 为 PB0/PB1/PB2/PB10，`ADC_get.c/.h`、本 README、`引脚分布表.md` 的引脚表同步更正。**连带注意**：`BlackPoint_Finder_Init()` 里 16 组 `min/max` 是按旧（错乱）映射标定的，若当时的物理顺序不是现在这样，需要按物理位置重新核一遍阈值。新增串口指令 **`g`**（灰度 16 路诊断，只读，不动电机与模式）。编译 0 Error / 0 Warning，Code=31864 / RO-data=1908 / RW-data=352 / ZI-data=8800。注：`docs/表格_20260919.csv`（厂商原始引脚表）第 18~21 行原为 `PB0=S3 … PB10=S0`（这次错误的源头），已同步改为 `PB0=S0 … PB10=S3`，全工程的引脚资料现在是一致的 |
| 2026-10-24 | `g` 由“打一次就完”改为**开关式实时打印**：发 `g` 进入（回 `[gray] ON` 并立刻打一帧），之后串口任务每 100ms（`APP_GRAY_MON_PERIOD_MS`）打一帧 16 路原始值 + 黑白判定 + 寻线结果；再发 `g` 退出（回 `[gray] OFF`）。开关是 `app_serial.c` 内的静态变量 `s_gray_mon`，不进指令队列、不打扰 2ms 控制任务；监视期间自动暂停 `w` 的波形帧（与 `bmq` 一致）。串口任务无字节时按 20ms 超时醒来，所以实际节拍是 100ms。编译 0 Error / 0 Warning，Code=31984 / RO-data=1976 / RW-data=352 / ZI-data=8800 |
| 2026-10-28 | **驱动改为两路半桥（单向），全面去掉电机方向脚**：删除 `Motor_SetDirection()`、`MOTOR_DIR_FORWARD/BACKWARD`、`MOTOR_DIR_LEVEL_SWAP`、`R_DIR_PORT/PIN`、`L_DIR_PORT/PIN` 以及 PA8/PA10 的 GPIO 初始化；`Motor_SetSpeedWithDirection()` 改名 `Motor_SetSpeedClamped()`，负速度一律按 **0**（极限打舵时一侧停转，不再倒转）；`2` 电机测试第 2 段由“向后 2 秒”改为“停车 2 秒”（`CAR_TEST_BWD_MS` → `CAR_TEST_STOP_MS`）；`5` 诊断不再打印 DIR 电平。PA8/PA10 现在是空脚。本 README、`引脚分布表.md`、`串口指令表.md`、`docs/表格_20260919.csv` 同步更新。全量重编译 0 Error / 0 Warning，Code=31788 / RO-data=1896 / RW-data=348 / ZI-data=8804（比上一版少 196 字节，就是删掉的方向逻辑） |
| 2026-10-28 | 新增**车头朝向（绕 Z 轴角度）**与串口指令 **`hx`**（开关式实时打印）：`App_CtrlTask` 每 2ms 读完 IMU 后调 `App_YawUpdate()`，对 `LSE6DSR_data.gz_rads` 去零偏积分成 `g_yaw_deg`（`±180°`，`0°` = 上电车头方向，逆时针为正，折回 `(-180, 180]`）；零偏 `g_gyro_bias_z` 只在“两轮编码器都 ≈0 且 `\|ωz\| < 0.15 rad/s` 连续 0.5s”时以 ≈2.5s 时间常数在线学习，跑动时冻结，避免把慢速转弯吃掉。串口侧新增静态开关 `s_yaw_mon`（`hx` 两字符状态机，`h` 后面不是 `x` 时不吞字符）+ `Serial_SendYaw()`，每 100ms（`APP_YAW_MON_PERIOD_MS`）打印 `yaw/wz/bias`，用 0.1° 定点整数拼串（microlib 下不引入 `%f` 浮点格式化）；`hx` 打开期间波形帧自动暂停。编译 0 Error / 0 Warning，Code=32632 / RO-data=1896 / RW-data=368 / ZI-data=8800 |
| 2026-10-28 | `w` 波形帧从 5 通道扩到 **6 通道**：新增 CH5 = 车头朝向 × 0.1°（`±1800` 即 `±180°`，`0` = 上电方向，VOFA+ 里除以 10），帧长 24 → 28 字节；`APP_WAVE_CHANNELS` 5 → 6，新增 `APP_WAVE_YAW_SCALE = 0.1f`；`?` 状态行追加 `yaw10=`（定点整数，不引入 `%f`）。旧通道编号不变，嫌电池电压没用就把 `Serial_SendWave()` 里 `ch[4]` 换掉。编译 0 Error / 0 Warning，Code=32624 / RO-data=1964 / RW-data=368 / ZI-data=8800 |
| 2026-10-28 | 陀螺排查工具：新增串口指令 **`i`**（一次打印 `init`、`WHO_AM_I`、`CTRL1_XL/CTRL2_G/CTRL3_C`、`STATUS_REG` + 6 轴原始值 + 12 字节 hex + `gz_rads/零偏/朝向`）；`App/app_main.c` 把 `LSM6DSR_Init()` 的返回值存进 `g_imu_init_ok`（之前直接丢弃，IMU 初始化失败完全无声）；`hx` 行尾追加 `raw N`（陀螺 Z 原始 LSB），手转车身时这个数不变即可判定传感器没有出数。编译 0 Error / 0 Warning，Code=33064 / RO-data=2104 / RW-data=368 / ZI-data=8800 |
| 2026-10-29 | 现场排障：用蓝牙串口（COM12）实测 `i` 全部读到 `0xFF`、`bb2 cslo_up=1 cslo_dn=0`（CS 拉低+打时钟后仍无人驱动），用 ST-Link 直连确认目标 3.2V、Device ID 0x410/64KB 正常，并**直接烧录**了本机构建（Code=34388）。新增串口指令 **`c`**：把 CS(PB12) 长拉低 15 秒并停掉所有 SPI 事务（新增 `g_imu_cs_hold`，`LSM6DSR_ReadData()` 见到即直接返回，否则每 2ms 一次读会把 CS 拉回去、万用表量到高低混合的假电平），用于万用表量模块 CS 焊盘。另把 `Project.uvprojx` 的 **CreateHexFile 改为 1**（原来只出 `.axf`，`Objects\` 里没有 hex，ST-Link 没法直接烧），现在每次 Build 都会生成 `Objects\Project.hex` |
| 2026-10-29 | 用户实测四根信号线都通、`bb miso_idle=1 who=0xF0 who_sw=0xFF`，且 `gz=63`（40442 urad/s）与同帧 hex 里的 `gz=0` 自相矛盾 → 判定**每次读到的都是浮空线随机值**。为此新增 **`bb2` 三态判定**：`LSM6DSR_BitBangSelfTest()` 里把 CS 拉低并打一个字节时钟，让芯片 SDO 进入驱动状态，再用上拉/下拉各读一次 MISO（`g_imu_bb_cslo_up/dn`），`i` 打 `[i] bb2 cslo_up=? cslo_dn=?`：`1/1` 或 `0/0` = 芯片活着在驱动（问题在协议侧），`1/0` = 线上没人驱动（没供电 / CS 没到芯片 / 芯片不回应）。原理上 `miso_idle` 说明不了芯片死活——CS 空闲时 SDO 本来就是高阻。编译 0 Error / 0 Warning，Code=34112 / RO-data=2104 / RW-data=376 / ZI-data=8800 |
| 2026-10-29 | 实车第二次 `i` 读到 `who=0xF0`、`c1=0xFF c2=0xFE c3=0xFF st=0xFF`（上一轮全 `0x00`）——总线从“被钉在低”变成“悬空/被拉高”，两次之间硬件被动过。为此 `i` 再加一行 **`pins cs/sck/miso/mosi`**（直接读 PB12/13/14/15 的 IDR）：可用“把线头碰 GND/3.3V 再发 `i` 看电平跟不跟”的办法定位断线；同时修 `App_YawUpdate()`：`g_imu_init_ok == 0` 时**直接不积分**（yaw 保持 0），避免把静态零偏积分成假朝向（现场曾出现 `yaw10=313` 这种莫名 31.3°）。编译 0 Error / 0 Warning，Code=33876 / RO-data=2104 / RW-data=372 / ZI-data=8804 |
| 2026-10-29 | 实车 `i` 显示 `who=0x00`、`c1/c2/c3/st` 全 `0x00`、12 字节里只有 1 个字节非零 → 陀螺/加速度计都没在应答，MISO 基本被钉在低电平；之前看到的“`gz=1957 urad/s`、`yaw +0.3°`”只是代码里那条静态零偏常量经积分得到的假值。为此在 **`LSM6DSR_Init()` 失败分支**里加了一次**位操作自检** `LSM6DSR_BitBangSelfTest()`（绕过 SPI2 外设，用 GPIO 手打模式 0 时序）：先把 MISO 配成上拉输入读空闲电平（判断线上有没有人驱动），再用“正常接线”和“MISO/MOSI 互换”两种接法各读一次 `WHO_AM_I`，最后调 `HardwareSPI_Init()` 恢复外设配置；结果存 `g_imu_bb_miso_idle / g_imu_bb_who / g_imu_bb_who_swap`，由 `i` 的第 5 行 `[i] bb ...` 打印。另修：`SPI_Init()` 后补 `SPI_NSSInternalSoftwareConfig(SPI2, SPI_NSSInternalSoft_Set)`（StdPeriph 只置 SSM 不置 SSI，主模式有 MODF 隐患）；`WHO_AM_I` 由读 1 次改为**重试 3 次**（间隔 10ms）。编译 0 Error / 0 Warning，Code=33760 / RO-data=2104 / RW-data=372 / ZI-data=8804 |
| 2026-10-29 | **决定不装陀螺仪**，固件不再依赖 IMU：`App_CtrlTask` 里整段 IMU 处理（`ReadData`/`ReadGyro`/`ConvertToPhysics`/`App_YawUpdate`）用 `if(g_imu_init_ok != 0u)` 包起来，传感器不存在时**一次总线操作都不做**，`yaw` 恒 0、波形第 6 通道恒 0，其余功能不受影响。同时把 `User/LSM6DSR_Config.c` 按 I²C 方案重写（去掉硬件 SPI2 的死代码，只留位操作 I²C + 总线互斥 + 总线恢复 + 串口 `i`/`x`/`c` 用的诊断），README/串口指令表/引脚表标注“陀螺仪未安装”。编译 0 Error / 0 Warning，Code=35148 / RO-data=2172 / RW-data=392 / ZI-data=8800 |
| 2026-10-29 | **删除陀螺仪相关的全部串口功能**（用户要求）：删掉 `hx`（朝向实时打印）、`i`/`x`/`c` 三条诊断指令，以及 `App_YawUpdate()`、`g_yaw_deg`/`g_yaw_rate_dps`/`g_gyro_bias_z`、`g_imu_init_ok`、`APP_YAW_*`、`APP_IMU_FULL_PERIOD_TICKS`；`w` 波形帧回到 **5 通道 / 24 字节**（去掉 CH5 朝向，删 `APP_WAVE_YAW_SCALE`）；`?` 状态行去掉 `yaw10=`，`g` 灰度监视不受影响；`App/app_main.c` 不再调用 `LSM6DSR_Init()`，`User/LSM6DSR_Config.c` 自此没有任何调用方（仅留作备用）。串口实测：`?` 正常、`i`/`hx` 无响应、`g` 开关正常、波形 5×24 字节按 50Hz 输出。编译 0 Error / 0 Warning，Code=30764 / RO-data=1896 / RW-data=348 / ZI-data=8804 |
| 2026-10-29 | 串口指令调整（用户要求）：**`s` 改成一键停车**（投递 `CAR_MODE_IDLE`，回显 `[stop] send IDLE`；`0` 仍可停车），原来的栈水位打印挪到 **`z`**；新增 **`k`** 开关**蓝牙链路看门狗**：`App/app_serial.c` 增加 `s_last_rx_tick` / `s_link_seen` / `s_link_wd_en` / `s_link_stop_latched` 四个静态量，`Serial_HandleByte()` 开头对任何字节续命，串口任务主循环调用 `Serial_LinkWatchdog()`——超过 `APP_LINK_TIMEOUT_MS`（`App/app_config.h`，2000ms）没收到字节且 `star_car != 0` 时，自动投递 IDLE 保护性停车并打印 `[link] no byte for 2000ms -> protective stop`；`star_car == 0` 只解锁不动作（待机调参不会误停），上电后从未收到字节则整个看门狗不启用（按键 K1 发车不受影响）。实测：临时把超时改成 300ms 烧录，发 `1` 后不再发任何字节，0.3s 即打印并回 IDLE；改回 2000ms 后 `?` / `s` / `k` / `z` 全部正常。编译 0 Error / 0 Warning，Code=31104 / RO-data=1896 / RW-data=360 / ZI-data=8800 |
| 2026-10-29 | **蓝牙保护改为看模块 STATE 脚**（用户要求：删掉 `k`，“蓝牙心跳的意思是检测蓝牙能不能连接，而不是用户不发送就停车”）：删除 `k` 指令、`#define APP_LINK_TIMEOUT_MS`、`Serial_LinkWatchdog()` 及四个心跳静态量；`User/Board_Config.h` 新增 `BT_STATE_PORT/PIN`（PA8），`Uart2_Init()` 把 PA8 配成输入下拉，新增 `Uart2_BtConnected()`；`App/app_serial.c` 改为 `Serial_BtLinkProtect()`，每轮读一次链接状态，只在 **1→0 下降沿**且 `star_car != 0` 时投递 IDLE 并打印 `[link] BT lost -> protective stop`（状态变化各打一行 `[link] BT connected/disconnected`）。没接线时 PA8 恒 0、无下降沿，K1 发车不受影响。烧录实测：`k` 已无响应、`s` 回 `[stop] send IDLE`、`z` 回 `stk(ctrl=200 keys=100 disp=212 ser=218) heap=1328`、开机不刷链路日志。编译 0 Error / 0 Warning，Code=31076 / RO-data=1896 / RW-data=352 / ZI-data=8800 |
| 2026-10-29 | **取消蓝牙自动停车**（用户反馈：模块没有 STATE/LINK 脚）：删除上一版刚加的 `Serial_BtLinkProtect()`、`s_bt_link_was`/`s_bt_lost_latched`、`Uart2_BtConnected()`、`Uart2_Init()` 里 PA8 的下拉初始化，以及 `User/Board_Config.h` 的 `BT_STATE_PORT/PIN`；PA8 恢复为空脚。**停车只保留串口 `s`（以及按键 K2、指令 `0`）**，串口不再因蓝牙断开擅自停车，也不会周期性输出任何内容。烧录实测：`?` 正常、`s` 回 `[stop] send IDLE`、`z` 回栈水位、静置 4 秒无输出。编译 0 Error / 0 Warning，Code=30820 / RO-data=1896 / RW-data=348 / ZI-data=8804 |
| 2026-10-29 | **控制环改为串级 PID（位置环 → 左右两个独立速度环）**：`User/PID_Controller.c` 的 `g_speed_pid` 拆成 `g_speed_pid_l` / `g_speed_pid_r`（旧版是“平均速度单环 + duty 域差速叠加”，无法补偿两轮差异）。位置环常数不变（Kp 198 / Ki 0 / Kd 2280、陀螺项 2800×gz 随陀螺停用而为 0，输出仍限 ±9000）；位置环输出改解释为“转向量”，按 `steer_speed = steer × base / 9000` 换成速度差（满舵 = ±base），左目标 = base + steer_speed、右目标 = base − steer_speed，两者下限 0（半桥不能倒转）；`base` 仍是 `230 − min(\|pos−8\|,3)/3 × 100`（130~230），起步斜坡 0.5/2ms 保留。两个速度环各用自己的编码器 `speed_left`/`speed_right` 反馈，增量式 PI（5.5/5.1/5.8、±8000）→ `Motor_SetSpeedClamped()`。`Car_Control.c` 三处 `SpeedPID_Reset(&g_speed_pid)` 改为左右各一次。转向极性与旧版保持同一套（Y0 在左/右未上板确认，若越修越偏把左右目标的加减号对调）。编译 0 Error / 0 Warning，Code=30924 / RO-data=1896 / RW-data=348 / ZI-data=8844 |
| 2026-10-29 | 新增串口指令 **`testl` / `testr`（单轮速度环标定）**：`Car_Control` 增加 `CAR_MODE_SPEED_TEST` 模式（`CAR_SPEED_TEST_MS 10000`、`CAR_SPEED_TEST_MAX 400`）与 `Car_Control_SpeedTest()` / `Car_Control_GetSpeedTest()`，模式内只闭环指定一侧（`g_speed_pid_l` 或 `g_speed_pid_r`）、另一侧强制 0、风扇不起压，超时自动回 `IDLE`，任何切模式指令都会取消标定；`SpeedPID_Controller_t` 增加 `last_target` / `last_feedback` / `last_p_term` / `last_i_term` 调试快照（`SpeedPID_Calculate()` 里顺手记录，不参与控制）；`AppCmd_t` 增加 `int16_t arg16` 与 `App_PostCmdArg()`（目标值 0~400 超出 `uint8_t`）；`app_serial.c` 用 `s_test_state` 前缀状态机匹配 `testl` / `testr`，匹配后复用 `s_num` 收数字，`Serial_SendSpeedTest()` 每 100ms 打 `[t] T/F/E/P/I/U`；`CAR_MODE_SPEED_TEST` 期间暂停 `w` 波形帧。串口实测：`testl`（不带数字）回用法提示、`testl0` 每 100ms 打印一行、`s` 立即停车。编译 0 Error / 0 Warning，Code=31924 / RO-data=1896 / RW-data=360 / ZI-data=8872 |
| 2026-10-29 | 新增**推车测距**串口指令 **`mc`（开关）**：`App/app_ctrl.c` 定义 `g_meas_on` / `g_meas_net_l` / `g_meas_net_r` / `g_meas_abs_l` / `g_meas_abs_r`，在 `ABEncoder_UpdateSpeed()` 之后把每 2ms 增量累加进去（净计数带符号、另记绝对值累计以判断来回拖）；`App/app_serial.c` 用 `s_mc_state` 匹配 `mc`，第一次进入时清零并投递 `IDLE`（保证电机不动）、每秒打印累计，第二次打印结果（`cnt/m`、`um/cnt`、来回拖动量、两轮符号不一致告警）；测量期间暂停波形帧。烧录实测：start/1Hz 行/stop/无位移告警均正常。编译 0 Error / 0 Warning，Code=32912 / RO-data=1896 / RW-data=388 / ZI-data=8876 |
| 2026-10-29 | **里程标定**：手推车 1 米、左右轮各测 5 次共 10 个数，去掉最大值和最小值后取平均得 **54476 counts/m**，写入 `User/ABEncoder.h` 的 `ENC_COUNTS_PER_METER`（另给出 `ENC_MM_PER_COUNT` ≈ 18.357 µm/count、`ENC_COUNTS_TO_MM_S` ≈ 9.1784 mm/s per count·2ms⁻¹；`ENC_SAMPLE_MS` 必须与控制周期 2ms 一致） |
| 2026-10-29 | **速度量纲从 counts/2ms 改成 mm/s**：`ABEncoder.c` 新增 `speed_left_mm_s` / `speed_right_mm_s`；`PID_Controller.c` 位置环基准 230/130 → **2110/1190 mm/s**、斜坡 0.5 → **4.6 mm/s per 2ms**、速度环增益 5.5/5.1/5.8 → **0.60/0.56/0.63**（duty per mm/s，数学等效）；`Car_Control` 的 `testl/testr` 目标与 `CAR_SPEED_TEST_MAX`（400 → 4000）改用 mm/s，反馈取 `speed_*_mm_s`；串口 `?` 的 `spd=`、`bmq` 的 `vL/vR`、波形帧 CH1/CH2、OLED 速度行同步改成 mm/s；`mc` 结束报告新增 `dist:`（按标定折成 mm）与 `vs cal:`（与 54476 的百分比偏差）两行。编译 0 Error / 0 Warning，Code=33128 / RO-data=1896 / RW-data=396 / ZI-data=8876 |
| 2026-10-30 | 新增**速度环调参模式** `tune`（临时调试用，不进循迹逻辑）：`Car_Control` 增加 `CAR_MODE_SPEED_TUNE` 与左右独立目标 `Car_Control_TuneSetTarget()`、在环增益 `Car_Control_TuneSetGain()`（内部调 `SpeedPID_SetParam()`）、快照 `Car_Control_TuneSnapshot()`（目标/反馈/输出）；2ms 里两侧各自闭环，目标为 0 的一侧输出 0 且复位该侧环（不积分）；`App/app_state.h` 增加 `APP_CMD_TUNE_ENTER/START/HALT/TARGET/GAIN`（复用 `arg16` 传 mm/s 与 milli 增益）；`app_serial.c` 增加 `tune` / `!` / `hlt` / `vl` / `vr` / `kpl..kdr`（支持 3 位小数）/ `kq` 的匹配与打印，运行中每 100ms 输出 CSV 并暂停波形帧（`hlt` 后停止打印；`hlt` 匹配器放在 `testl` 之前，避免末尾 `t` 被抢）。编译 0 Error / 0 Warning，Code=35588 / RO-data=2128 / RW-data=420 / ZI-data=9196 |
| 2026-10-30 | `!` 改为**定时闭环**：! <ms> l / ! <ms> r / ! <ms> all（1~30000ms），到点自动停车并清目标（留在 tune 内）；运行中再发 ! 直接忽略；单侧运行时另一侧目标强制 0。Car_Control_TuneRun() 用 2ms 控制拍倒计时（CAR_TUNE_RUN_MS_MAX），Car_TuneSnapshot_t 增加 side/run_ms；串口新增 ! 五态解析器 + [tune] run/stop/busy 回显，Serial_SendTune 只在运行中打印。另新增 速度环调参说明.md 操作规范。编译 0 Error / 0 Warning，Code=36328 / RO-data=2116 / RW-data=436 / ZI-data=9196 |
| 2026-10-30 | 调参闭环接入**负压风扇**：`Car_Control.h` 新增 `CAR_TUNE_FAN_DUTY 500`（= 1500µs 油门，M3PWM 量程 0~1000）与 `CAR_TUNE_FAN_ARM_MS 500`；`Car_Control.c` 新增静态 `s_tune_arm_ticks`，`Car_Control_TuneRun()` 收到 `!` 后置锁臂 tick、风扇压最低油门、**闭环计时暂不启动**，`Car_Control_TuneUpdate()` 新增锁臂段（期间电机两侧占空比 0、不解锁电机，锁臂结束后 `M3PWM_SetDutyCycle(500)` + `Motor_Enable()` 并从这一刻起计时），`Car_Control_TuneStop()`（到点 / `hlt` / `0` / 切模式）把风扇同步归 0，`Car_TuneSnapshot_t.running` 在锁臂期间仍为 0（CSV 从运动开始才打印）；CSV 周期 `APP_TUNE_PERIOD_MS` 100 → **20ms（50Hz）**；`ApplyMode()` 增加清 `s_spd_ticks`、`TuneSetTarget()` 增加 0~`CAR_TUNE_MAX` 限幅。文档同步：`README.md`、`串口指令表.md`、`速度环调参说明.md`。编译 0 Error / 0 Warning，Code=36432 / RO-data=2116 / RW-data=440 / ZI-data=9192；烧录校验和 0x0037F088。备注：本次改动用 PowerShell 脚本时发生变量名大小写冲突，把 `User/Car_Control.c` 写坏，已按原逻辑重建并用事故前保存的 `Car_Control.o` 比对恢复（20 个函数、98 个节的名称与大小完全一致，主镜像 Code 也回到 36328） |
| 2026-10-30 | **回退**“`!` 带负压百分比”那一版（判为过度设计）：`!` 恢复为 `! <ms> <l\|r\|all>` 五态解析，负压固定 50%（`CAR_TUNE_FAN_DUTY = 500`），锁臂 1.5s 后起风并开始闭环。删除 `APP_CMD_TUNE_FAN`、`App_PostCmdArg8()`（`AppCmd_t.arg8` 一并去掉）、`Car_Control_TuneSetFan()`、`s_tune_fan_duty`、串口 `s_ex_side/s_ex_pct` 与 7 态解析器。编译 0 Error / 0 Warning，Code=36400 / RO-data=2116 / RW-data=440 / ZI-data=9192（与回退前 36436 相差的 36 字节来自串口提示串变短） |
| 2026-10-30 | **删掉调参模式里的负压风扇**（`!` 与此无关，只做定时闭环）：移除 `CAR_TUNE_FAN_DUTY` / `CAR_TUNE_FAN_ARM_MS` 两个宏、静态 `s_tune_arm_ticks`、`Car_Control_TuneRun()` 里的锁臂 tick 与 `M3PWM_SetDutyCycle(0)`、`Car_Control_TuneUpdate()` 的锁臂段（不再有“等电调解锁”的 1.5s 空转，`!` 之后立即使能电机并计时）、`Car_Control_TuneStop()` 的风扇归零；`Car_TuneSnapshot_t.running` 简化回 `s_tune_run != 0`。`!` 的解析与 `vl/vr/kp..` 全部不变。编译 0 Error / 0 Warning，Code=36300 / RO-data=2116 / RW-data=436 / ZI-data=9196 |
| 2026-10-30 | `速度环调参说明.md` 增补 **§9 AI 自动调参流程**（固定配方：500 mm/s、单侧、`! 1500`、组间 ≥2.6s、单步 ≤8%、3 次取中位；采数口径：50Hz CSV、有效窗口、块均；决策规则、停止条件、交付物、禁止事项）、**§10 最终指标要求**（① 上升 100~300ms ② 上升过程偏差 ≤10% ③ 稳态 ≤±2%，均按 100ms 块平均判定）与 **§11 已知坑**，并写明 1 count/2ms = 9.1784 mm/s 的量化地板 |
| 2026-10-30 | **接入 Git 版本管理**：本目录 `git init`（分支 `main`）并推送到 <https://github.com/Jize678/xs2026>，首个提交 `9350d1b`（153 个文件、约 13.7MB）；新增 `.gitignore` 忽略 Keil 编译产物（`Objects/`、`Listings/`、`*.o /*.d /*.crf /*.axf /*.hex /*.map`）、IDE 用户文件（`*.uvguix.*`、`*.uvoptx`、`JLinkLog.txt`、`JLinkSettings.ini`、`EventRecorderStub.scvd`、`DebugConfig/`）与临时日志 `build*.log`；`ld/` 下的 ST 参考 PDF 随库保存；本地设 `core.autocrlf=false` 保持原始换行、`core.quotepath=false` 正常显示中文文件名，作者身份用 GitHub noreply 邮箱 |
| 2026-10-30 | **修掉速度环 D 项的结构性缺陷**：`SpeedPID_Calculate()` 里 `p_term` 与 `d_term` 原来共用同一个差分 `e(k)−e(k−1)`，`kd` 实际是"第二个 kp"（Δe 总增益 = `kp+kd` = 1.23，见 `速度环调参记录.md` §5）。本次 `d_term` 改为标准二阶差分 `kd*[e(k)−2e(k−1)+e(k−2)]`，`SpeedPID_Controller_t` 新增 `last_error2`（`Init`/`Reset` 清零，更新顺序必须是 `last_error2 = last_error; last_error = error;`）；默认增益按"等效保真"重标为 **`kp=1.23 / ki=0.56 / kd=0.00`**，Δe 总增益仍为 1.23 故手感不变；同时修正 `PID_Controller.h` 头部过期的 counts 量纲框图（230/130 → 2110/1190 mm/s）与 `PositionPID_Calculate()` 里"左轮减速、右轮加速"的错注释（代码实际是左目标变大 = 右转）。文档同步：`串口指令表.md`、`速度环调参说明.md` §5/§9 的默认增益、`速度环调参记录.md` 新增 §13（旧数据换算表，并注明 §12 的负压锁臂已作废）。编译 0 Error / 0 Warning，Code=36324 / RO-data=2116 / RW-data=436 / ZI-data=9204 |
| 2026-10-30 | **测速加 5 拍滑窗（A2 降噪）**：`User/ABEncoder.h` 新增 `ENC_SPEED_WINDOW 5`（置 1 即退回瞬时差分，可做 A/B 对比），`User/ABEncoder.c` 在 `ABEncoder_UpdateSpeed()` 里为左右各加一个 5 深环形缓冲，`speed_left/right_mm_s` 改为"最近 5 拍计数和 / 5"（量纲仍是 counts per 2ms，`ENC_COUNTS_TO_MM_S` 与里程标定都不变），另存单拍瞬时值 `speed_left/right_mm_s_raw` 供诊断；上电前 4 拍按实际已填样本数取平均，避免被"不存在的历史"拉低。收益：500 mm/s 下量化分辨率 1.84% → **0.37%**、随机噪声 ÷√5；代价是 (N−1)/2 拍 = **4 ms 群延迟**（20 Hz 处约 29° 相位），**控制拍仍是 2ms**，两者不要混为一谈。`bmq` 的 `vL/vR`、`tune` 的 `fL/fR`、波形帧 CH1/CH2、OLED 速度行同步变为滑窗值。编译 0 Error / 0 Warning，Code=36536 / RO-data=2116 / RW-data=448 / ZI-data=9224 |
| 2026-10-30 | **调参安全规则入档：只许单侧电机通电**。`速度环调参说明.md` 新增 **§9.0 硬安全规则**（单轮驱动 → 固定半径圆环 → 行程有界；禁止 `1`/`2`/K1/任何 `r…`、`vr…`、`testr…`；只读指令 `?`/`kq`/`bmq`/`g`/`z` 不限；本版固件无单侧开环命令故不做开环验证），§9.7 禁止事项同步改写；`tools/tune_auto.ps1` 增加白名单安全门 `Assert-SafeCmd()`（不在 `?/kq/k[lri d]/0/hlt/tune/vl/vr/bmq/g/z/!` 清单里的指令直接抛错拒发）与 `Do-Group` 的 `-Side r` 拦截。起因：当日调参中 AI 为验证测速噪声发了 `2`（MOTOR_TEST 两轮 30%），右轮跟着转了 2 秒，违反"只动左轮"的约定 |
| 2026-10-30 | **速度环实车调参完成，定版增益写入固件** `kp=0.200 / ki=0.080 / kd=0.040`（左右相同，`User/PID_Controller.c` 的 `PID_Init()`）。条件：单侧闭环、1000 mm/s、`! 4000`、4 s、无负压、每组 5 reps 取中位。结果——左轮 稳态 999.9 / 误差 −0.01% / t90 200 ms / t90 后偏离 4.1% / sd 19.7；右轮 稳态 999.0 / 误差 −0.10% / t90 100 ms / 偏离 2.97% / sd 11.4；**两侧三项指标全部通过**。关键经验：旧默认 1.23 大到让输出长期钳在 0（`u_mean = −2489`），积分对着饱和累积造成 +16% 的假偏差；先把 kp 砍到 0.6、ki 砍到 0.2 脱离饱和，再 0.467→0.300→0.200 把偏离从 28.2% 压到 4.1%；8% 步进在饱和区里几乎看不出效果，容易误判成"纹波与增益无关"。遗留：500 mm/s 时左轮偏离 10.35%（占空比 6.5% 贴静摩擦门槛），常跑低速需单独整定。新增 `PID调参轨迹.txt`（逐次参数+指标+判定）、`tools/tune_auto.ps1`（六子命令 runner）与 `tune_data/`（原始 CSV，不入库）；`速度环调参说明.md` 新增 §9.8 工具说明、§9.0 改为"一次只许一侧、左右都行"、§10.3 换成定版实测；`速度环调参记录.md` 新增 §14。编译 0 Error / 0 Warning，Code=36540 / RO-data=2116 / RW-data=448 / ZI-data=9224 |
