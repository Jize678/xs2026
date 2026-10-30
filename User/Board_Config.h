/**
  ******************************************************************************
  * @file    Board_Config.h
  * @brief   车模硬件引脚分配 —— 全工程唯一来源
  *
  * 数据来源：现在的车模引脚分配表（表格_20260919.csv）
  * 芯片型号：STM32F103C8T6 (LQFP48)，外部晶振 8MHz，SYSCLK 72MHz
  *
  * 说明：
  *   1. 工程内所有外设模块（电机、编码器、ADC、按键、OLED、RGB、陀螺仪、
  *      串口、风扇）都只引用本文件里的宏，不再在各自的 .c/.h 里写死端口和引脚。
  *   2. 车模改板时，只需要修改本文件，其余业务代码不动。
  *
  * 引脚分配表
  *   引脚号 | 芯片引脚        | 网络 / 功能 | 说明
  *   -------+-----------------+-------------+---------------------------------
  *     2    | PC13-TAMPER-RTC | KEY4        | 按键（低电平有效，内部上拉）
  *     3    | PC14-OSC32_IN   | KEY3        | 按键（低电平有效，内部上拉）
  *     4    | PC15-OSC32_OUT  | ——          | 未连接（图中绿叉 NC）
  *     5    | PD0-OSC_IN      | OSCIN       | 外部晶振输入
  *     6    | PD1-OSC_OUT     | OSCOUT      | 外部晶振输出
  *    10    | PA0-WKUP        | ADC_BAT     | 模拟电源 / ADC 电池检测
  *    11    | PA1             | ADC_IN      | ADC 输入（16 路灰度模拟开关输出）
  *    12    | PA2             | USART2_TX   | 串口 2 发送
  *    13    | PA3             | USART2_RX   | 串口 2 接收
  *    14    | PA4             | KEY2        | 按键
  *    15    | PA5             | KEY1        | 按键
  *    16    | PA6             | ENC1B       | 编码器 1 B 相
  *    17    | PA7             | ENC1A       | 编码器 1 A 相
  *    18    | PB0             | S0          | 模拟开关地址位 S0（最低位）
  *    19    | PB1             | S1          | 模拟开关地址位 S1
  *    20    | PB2             | S2          | 模拟开关地址位 S2
  *    21    | PB10            | S3          | 模拟开关地址位 S3（最高位）
  *    22    | PB11            | FAN_PWM     | 风扇 PWM（TIM2_CH4）
  *    25    | PB12            | GYRO_NSS    | 陀螺仪 SPI 片选
  *    26    | PB13            | GYRO_SCK    | 陀螺仪 SPI 时钟
  *    27    | PB14            | GYRO_MISO   | 陀螺仪 SPI MISO
  *    28    | PB15            | GYRO_MOSI   | 陀螺仪 SPI MOSI
  *    29    | PA8             | ——          | 空出（原 R_DIR，两路半桥后不用）
  *    30    | PA9             | R_PWM       | 右电机 PWM（TIM1_CH2）
  *    31    | PA10            | ——          | 空出（原 L_DIR，改两路半桥后不用）
  *    32    | PA11            | L_PWM       | 左电机 PWM（TIM1_CH4）
  *    33    | PA12            | MOTOREN     | 电机驱动使能
  *    34    | PA13            | SDIO        | SWDIO（调试口，不做 GPIO）
  *    37    | PA14            | SCLK        | SWCLK（调试口，不做 GPIO）
  *    38    | PA15            | RGB-0       | RGB 数据通道 0
  *    39    | PB3             | RGB-1       | RGB 数据通道 1
  *    40    | PB4             | IIC_D       | OLED I2C 数据
  *    41    | PB5             | IIC_C       | OLED I2C 时钟
  *    42    | PB6             | ENC2B       | 编码器 2 B 相
  *    43    | PB7             | ENC2A       | 编码器 2 A 相
  *    45    | PB8             | RGB-2       | RGB 数据通道 2
  *    46    | PB9             | RGB-3       | RGB 数据通道 3
  ******************************************************************************
  */

#ifndef __BOARD_CONFIG_H
#define __BOARD_CONFIG_H

#include "stm32f10x.h"

/*==============================================================================
 * 一、按键 KEY1 ~ KEY4
 *   KEY1 -> PA5   KEY2 -> PA4   KEY3 -> PC14  KEY4 -> PC13
 *   低电平有效，MCU 内部上拉
 *============================================================================*/
#define KEY_K1_PORT           GPIOA
#define KEY_K1_PIN            GPIO_Pin_5      /* 引脚 15 */
#define KEY_K1_RCC            RCC_APB2Periph_GPIOA

#define KEY_K2_PORT           GPIOA
#define KEY_K2_PIN            GPIO_Pin_4      /* 引脚 14 */
#define KEY_K2_RCC            RCC_APB2Periph_GPIOA

#define KEY_K3_PORT           GPIOC
#define KEY_K3_PIN            GPIO_Pin_14     /* 引脚 3  */
#define KEY_K3_RCC            RCC_APB2Periph_GPIOC

#define KEY_K4_PORT           GPIOC
#define KEY_K4_PIN            GPIO_Pin_13     /* 引脚 2  */
#define KEY_K4_RCC            RCC_APB2Periph_GPIOC

/*==============================================================================
 * 二、ADC：电池检测 ADC_BAT(PA0) + 灰度输入 ADC_IN(PA1)
 *   16 路灰度传感器经模拟开关（S0~S3 选通）后接到 PA1，PA0 单独测电池电压
 *============================================================================*/
#define ADC_BAT_PORT          GPIOA
#define ADC_BAT_PIN           GPIO_Pin_0      /* 引脚 10 */
#define ADC_BAT_CHANNEL       ADC_Channel_0   /* ADC1_IN0 */

#define ADC_IN_PORT           GPIOA
#define ADC_IN_PIN            GPIO_Pin_1      /* 引脚 11 */
#define ADC_IN_CHANNEL        ADC_Channel_1   /* ADC1_IN1 */

#define ADC_GPIO_RCC          RCC_APB2Periph_GPIOA

/* 模拟开关地址线（全部在 GPIOB 上）
 *   2026-10-24 按实车更正：原来的表把 S0/S3 与 S1/S2 接反了。
 *   S0 -> PB0  (引脚 18)   S1 -> PB1  (引脚 19)
 *   S2 -> PB2  (引脚 20)   S3 -> PB10 (引脚 21)
 *
 * 真值表（16 路灰度输出）：
 *   S3 S2 S1 S0 = 0000 0001 ... 1111  ->  输出 AS = Y0 Y1 ... Y15
 *   即地址 = 通道号的二进制，S0 最低位、S3 最高位。
 *   地址线不是 PB0~PB3 连续排列，S3 跑到了 PB10。
 */
#define MUX_GPIO_PORT         GPIOB
#define MUX_GPIO_CLK          RCC_APB2Periph_GPIOB
#define MUX_S0_PIN            GPIO_Pin_0      /* 引脚 18 */
#define MUX_S1_PIN            GPIO_Pin_1      /* 引脚 19 */
#define MUX_S2_PIN            GPIO_Pin_2      /* 引脚 20 */
#define MUX_S3_PIN            GPIO_Pin_10     /* 引脚 21 */

/*==============================================================================
 * 三、串口 2（调试 / 上位机）
 *   PA2 -> USART2_TX   PA3 -> USART2_RX
 *============================================================================*/
#define UART2_TX_PORT         GPIOA
#define UART2_TX_PIN          GPIO_Pin_2      /* 引脚 12 */
#define UART2_RX_PORT         GPIOA
#define UART2_RX_PIN          GPIO_Pin_3      /* 引脚 13 */
#define UART2_GPIO_RCC        RCC_APB2Periph_GPIOA

/*==============================================================================
 * 四、编码器（TIM3 / TIM4 硬件编码器接口）
 *   ENC1（右轮）: PA6 = ENC1B -> TIM3_CH1，PA7 = ENC1A -> TIM3_CH2
 *   ENC2（左轮）: PB6 = ENC2B -> TIM4_CH1，PB7 = ENC2A -> TIM4_CH2
 *
 *   注意：本车模两路编码器都是 B 相接在 CH1、A 相接在 CH2。
 *   与“A 相接 CH1”的常规接法相比，计数器方向相反，
 *   方向补偿见 ABEncoder.h 里的 ENC_R_SIGN / ENC_L_SIGN。
 *============================================================================*/
#define ENC1_GPIO_PORT        GPIOA
#define ENC1_A_PIN            GPIO_Pin_7      /* ENC1A 引脚 17 -> TIM3_CH2 */
#define ENC1_B_PIN            GPIO_Pin_6      /* ENC1B 引脚 16 -> TIM3_CH1 */
#define ENC1_TIM              TIM3
#define ENC1_TIM_RCC          RCC_APB1Periph_TIM3

#define ENC2_GPIO_PORT        GPIOB
#define ENC2_A_PIN            GPIO_Pin_7      /* ENC2A 引脚 43 -> TIM4_CH2 */
#define ENC2_B_PIN            GPIO_Pin_6      /* ENC2B 引脚 42 -> TIM4_CH1 */
#define ENC2_TIM              TIM4
#define ENC2_TIM_RCC          RCC_APB1Periph_TIM4

#define ENC_GPIO_RCC          (RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB)

/*==============================================================================
 * 五、电机驱动（TIM1，两路半桥 / 单向）
 *   驱动只认 PWM，没有方向脚，电机只能往前走。
 *   R_PWM -> PA9  (TIM1_CH2)
 *   L_PWM -> PA11 (TIM1_CH4)
 *   MOTOREN -> PA12
 *   原方向脚 PA8 / PA10 已空出，不再使用（也不要再接线）。
 *============================================================================*/
#define MOTOREN_PORT          GPIOA
#define MOTOREN_PIN           GPIO_Pin_12     /* 引脚 33 */

#define R_PWM_PORT            GPIOA
#define R_PWM_PIN             GPIO_Pin_9      /* 引脚 30 -> TIM1_CH2 */
#define L_PWM_PORT            GPIOA
#define L_PWM_PIN             GPIO_Pin_11     /* 引脚 32 -> TIM1_CH4 */

#define MOTOR_GPIO_RCC        RCC_APB2Periph_GPIOA
#define MOTOR_TIM             TIM1
#define MOTOR_TIM_RCC         RCC_APB2Periph_TIM1

/*==============================================================================
 * 六、风扇 PWM（TIM2_CH4，需要 TIM2 部分重映射 2）
 *   FAN_PWM -> PB11
 *============================================================================*/
#define FAN_PWM_PORT          GPIOB
#define FAN_PWM_PIN           GPIO_Pin_11     /* 引脚 22 -> TIM2_CH4 */
#define FAN_PWM_GPIO_RCC      RCC_APB2Periph_GPIOB
#define FAN_PWM_TIM           TIM2
#define FAN_PWM_TIM_RCC       RCC_APB1Periph_TIM2
#define FAN_PWM_REMAP         GPIO_PartialRemap2_TIM2

/*==============================================================================
 * 七、陀螺仪 LSM6DSR（SPI2 硬件 SPI，CS 软件控制）
 *   GYRO_NSS -> PB12   GYRO_SCK  -> PB13
 *   GYRO_MISO-> PB14   GYRO_MOSI -> PB15
 *============================================================================*/
#define GYRO_NSS_PORT         GPIOB
#define GYRO_NSS_PIN          GPIO_Pin_12     /* 引脚 25 */
#define GYRO_SCK_PORT         GPIOB
#define GYRO_SCK_PIN          GPIO_Pin_13     /* 引脚 26 */
#define GYRO_MISO_PORT        GPIOB
#define GYRO_MISO_PIN         GPIO_Pin_14     /* 引脚 27 */
#define GYRO_MOSI_PORT        GPIOB
#define GYRO_MOSI_PIN         GPIO_Pin_15     /* 引脚 28 */
#define GYRO_GPIO_RCC         RCC_APB2Periph_GPIOB
#define GYRO_SPI              SPI2
#define GYRO_SPI_RCC          RCC_APB1Periph_SPI2

/*==============================================================================
 * 八、OLED（软件 I2C）
 *   IIC_D -> PB4（数据）   IIC_C -> PB5（时钟）
 *   注意：PB4 默认是 JTAG 的 NJTRST，需要在 OLED_Init 里关掉 JTAG
 *============================================================================*/
#define OLED_SDA_PORT         GPIOB
#define OLED_SDA_PIN          GPIO_Pin_4      /* 引脚 40 / IIC_D */
#define OLED_SCL_PORT         GPIOB
#define OLED_SCL_PIN          GPIO_Pin_5      /* 引脚 41 / IIC_C */
#define OLED_GPIO_RCC         RCC_APB2Periph_GPIOB

/*==============================================================================
 * 九、RGB 灯（共阳极：低电平点亮）
 *   RGB-0 -> PA15   RGB-1 -> PB3   RGB-2 -> PB8   RGB-3 -> PB9
 *
 *   注意：PA15 / PB3 默认是 JTAG 的 JTDI / JTDO，
 *        需要关闭 JTAG（保留 SWD）后才能当普通 GPIO 用。
 *   本车模这 4 个脚接的不是 RGB 灯：
 *     RGB-0 / RGB-1 是两颗指示灯（高电平点亮，闪灯逻辑见 Led_Status.c）；
 *     RGB-2 / RGB-3 是引出给车灯用的预留输出，现在保持熄灭。
 *============================================================================*/
#define RGB0_PORT             GPIOA
#define RGB0_PIN              GPIO_Pin_15     /* 引脚 38 */
#define RGB1_PORT             GPIOB
#define RGB1_PIN              GPIO_Pin_3      /* 引脚 39 */
#define RGB2_PORT             GPIOB
#define RGB2_PIN              GPIO_Pin_8      /* 引脚 45 */
#define RGB3_PORT             GPIOB
#define RGB3_PIN              GPIO_Pin_9      /* 引脚 46 */

#define RGB_GPIO_RCC          (RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB)

#define RGB_CHANNEL_COUNT     4               /* 通道总数 RGB-0 ~ RGB-3 */

#endif /* __BOARD_CONFIG_H */
