#ifndef __M3PWM_H
#define __M3PWM_H

#include "stm32f10x.h"

/* ============================ 输出模式选择 ============================
 * FAN_PWM_MODE_SERVO = 1（默认，PB11 接的是【无刷电调 ESC】）
 *     输出 50Hz 标准舵机信号：周期 20ms，高电平脉宽 1000~2000us
 *         0%   -> 1000us（最低油门，停转）
 *         100% -> 2000us（满油门）
 *     电调只认这种信号。上电后它会先用电机绕组发声报"电池节数"，
 *     再等最低油门保持约 1s 才"嘀—嘀"确认解锁，之后才响应油门。
 *
 * FAN_PWM_MODE_SERVO = 0（PB11 接【有刷风机 + MOSFET】）
 *     输出 17kHz 高速 PWM，按占空比 0~1000/1000 调速（原来的做法）。
 *     注意：17kHz 对电调完全无效 —— 电调会一直不解锁、电机不转，
 *     但上电瞬间的鸣叫仍会发生，很容易误判成"信号没问题"。
 *
 * 两种模式引脚、定时器、重映射完全相同，只差时基和"占空比 -> CCR"的换算。
 */
#define FAN_PWM_MODE_SERVO    1

#define FAN_SERVO_MIN_US      1000u    /* 最低油门脉宽 */
#define FAN_SERVO_MAX_US      2000u    /* 最高油门脉宽 */
#define FAN_SERVO_PSC         71u      /* 72MHz/(71+1) = 1MHz，计数 1 个单位 = 1us */
#define FAN_SERVO_ARR         19999u   /* 20000us = 20ms = 50Hz */

#define FAN_FAST_PSC          0u
#define FAN_FAST_ARR          4234u    /* 72MHz/4235 ≈ 17kHz */

/* 上电解锁保持时间：M3PWM_Init() 把油门压到最低并保持这么久，等电调自检解锁。
 * 这段时间内推油门是没用的（电调还没解锁），所以初始化里等一次，后面随便发指令。 */
#define FAN_ARM_DELAY_MS      1500u

/* 兼容旧名字（原来写在 17kHz 版本里，工程里还有地方引用） */
#define PWM_FREQUENCY_17KHZ   37000
#define PWM_DUTY_CYCLE_50     50

/* 函数声明 */
void     M3PWM_Init(void);                    /* 初始化 + 上电解锁等待 */
void     M3PWM_SetDutyCycle(uint16_t duty);   /* 油门 0~1000（千分比），两种模式语义一致 */
void     M3PWM_SetPulseUs(uint16_t us);       /* 直接给脉宽 1000~2000us（仅舵机模式） */
uint16_t M3PWM_GetPulseUs(void);              /* 当前高电平脉宽（us），非舵机模式返回 0 */
uint16_t M3PWM_GetDuty(void);                 /* 当前油门 0~1000 */
void     M3PWM_Start(void);                   /* 启动定时器输出 */
void     M3PWM_Stop(void);                    /* 停定时器（引脚回到无效电平） */
void     M3PWM_SetFrequency(uint32_t freq);   /* 仅 FAN_PWM_MODE_SERVO=0 时有效 */
void     M3PWM_PrintDiag(void);               /* 打印 TIM2/GPIO/AFIO 寄存器，确认引脚真在输出 */

#endif
