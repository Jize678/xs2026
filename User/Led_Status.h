/**
  ******************************************************************************
  * @file    Led_Status.h
  * @brief   两颗指示灯（RGB-0/PA15、RGB-1/PB3，高电平点亮）的状态指示设计
  *
  * 闪灯图案（由运行模式决定）：
  *   IDLE  待机    : LED0 与 LED1 同步闪（亮 0.5s、灭 0.5s，表示"在线待命"）
  *   TRACK 循迹运行: LED0 常亮（运行中）；
  *                   LED1 在线上时灭，丢线时快闪（提示正在找线）
  *   TEST  电机测试: LED0 / LED1 交替闪（250ms 一换，一眼能和别的方式区分）
  *   ALARM 异常停车: 两颗灯同频快闪（例如丢线保护停车，等新指令）
  *   FAN   风扇测试: 两颗灯同频快闪（串口 'f' 起风扇，5 秒后自动停）
  *   FAN+  延时风扇: 两颗灯超快同频闪（串口 'ff'，先等 5 秒，再吹 5 秒）
  *
  * 节奏参数都在下面，改宏就能调手感；待机不想要闪灯时把
  * LED_IDLE_BLINK_EN 改成 0 即可（两灯全灭）。
  ******************************************************************************
  */

#ifndef __LED_STATUS_H
#define __LED_STATUS_H

#include "stm32f10x.h"

typedef enum
{
	LED_ST_IDLE = 0,     /* 待机：LED0 与 LED1 同步闪（0.5s 亮 / 0.5s 灭） */
	LED_ST_TRACK,        /* 循迹运行：LED0 常亮，丢线时 LED1 快闪 */
	LED_ST_TEST,         /* 电机测试：两灯交替闪 */
	LED_ST_ALARM,        /* 异常停车：两灯快闪 */
	LED_ST_FORCE_ON,     /* 调试：两灯常亮（串口 '3'） */
	LED_ST_FORCE_OFF,    /* 调试：两灯常灭（串口 '4'） */
	LED_ST_FAN_TEST,     /* 风扇测试：两灯同频快闪（串口 'f'，5 秒） */
	LED_ST_FAN_FAST      /* 延时风扇测试：两灯超快同频闪（串口 'ff'，先等 5s 再吹 5s） */
} Led_State_t;

/* ---- 闪灯节奏（单位 ms） ---- */
#define LED_TICK_MS              2      /* 调用周期，与 SysTick 一致（2ms） */
#define LED_IDLE_ON_MS           500    /* 待机同步闪：亮的时长（两颗灯一起） */
#define LED_IDLE_PERIOD_MS       1000   /* 待机同步闪：周期 */
#define LED_LOST_ON_MS           40     /* 丢线提示：每次亮多久 */
#define LED_LOST_MS              120    /* 丢线提示：周期 */
#define LED_TEST_HALF_MS         250    /* 电机测试：交替的半周期 */
#define LED_ALARM_HALF_MS        80     /* 异常停车：快闪半周期 */
#define LED_FAN_FAST_HALF_MS     40     /* 延时风扇测试：超快闪半周期 */

/* 待机时是否要闪灯：1 = 两颗灯同步闪（默认），0 = 两灯全灭 */
#define LED_IDLE_BLINK_EN        1

/* 上电自检：上电后两颗灯同时亮一段时间，用来确认灯和极性没问题 */
#define LED_BOOT_SELFTEST_EN     1
#define LED_BOOT_SELFTEST_MS     500

void       Led_Status_Init(void);                    /* 上电初始化（两灯全灭） */
void       Led_Status_SelfTest(void);                /* 上电自检：两颗灯同时亮一下 */
void       Led_Status_SetState(Led_State_t st);      /* 切换指示状态 */
Led_State_t Led_Status_GetState(void);               /* 读取当前指示状态 */
void       Led_Status_SetLineFound(uint8_t found);   /* 循迹时喂入"是否在线上" */
void       Led_Status_Update(void);                  /* 2ms 周期调用，刷新两颗灯 */

#endif /* __LED_STATUS_H */
