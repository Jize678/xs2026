#ifndef __RGB_LED_H__
#define __RGB_LED_H__

#include "stm32f10x.h"
#include "Board_Config.h"

/* 本车模这 4 个脚的实际用途（引脚表里的网络名还是 RGB-0 ~ RGB-3，但接的不是 RGB 灯）：
 *   RGB-0 (PA15) -> LED0 指示灯，高电平点亮
 *   RGB-1 (PB3)  -> LED1 指示灯，高电平点亮
 *   RGB-2 (PB8)  -> 预留车灯输出（已引出，现在不用，将来接车灯）
 *   RGB-3 (PB9)  -> 预留车灯输出（已引出，现在不用，将来接车灯）
 *
 * 两颗指示灯的闪灯逻辑在 Led_Status.c 里，本文件只负责按逻辑亮灭控制引脚。
 */
#define LED0_CH				0		/* RGB-0 -> PA15 指示灯 */
#define LED1_CH				1		/* RGB-1 -> PB3  指示灯 */
#define LAMP0_CH			2		/* RGB-2 -> PB8  预留车灯 */
#define LAMP1_CH			3		/* RGB-3 -> PB9  预留车灯 */

/* 点亮电平总开关：1 = 高电平点亮（本车模两颗指示灯就是这样），0 = 低电平点亮
 * 判断方法：上电自检时两颗灯应该同时亮 0.5s，
 * 如果自检时完全不亮，把这里改成 0 再试一次，就能确认是不是极性写反了。 */
#define LED_ACTIVE_HIGH		1

void RGB_Init(void);
void RGB_EnsureUsable(void);                        /* 初始化末尾再确认一次：关JTAG + 配成输出 */
void RGB_SetChannel(uint8_t channel, uint8_t on);   /* channel: 0~3，on: 0=灭 1=亮（按逻辑，不管电平） */
void RGB_AllOff(void);

/* 两颗指示灯（高电平点亮） */
void LED0_Set(uint8_t on);
void LED1_Set(uint8_t on);

#endif
