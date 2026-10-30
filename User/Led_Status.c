#include "Led_Status.h"
#include "RGB_Led.h"
#include "Delay.h"

static Led_State_t s_state = LED_ST_IDLE;
static uint32_t    s_tick = 0;          /* 状态内的计时（单位 ms） */
static uint8_t     s_line_found = 0;    /* 循迹时是否找到线 */

void Led_Status_Init(void)
{
	s_state = LED_ST_IDLE;
	s_tick = 0;
	s_line_found = 0;
	RGB_AllOff();
}

/* 上电自检：两颗指示灯同时亮 LED_BOOT_SELFTEST_MS 毫秒后再灭，
 * 用来确认"引脚 → 灯 → 极性"这一路是通的。 */
void Led_Status_SelfTest(void)
{
#if LED_BOOT_SELFTEST_EN
	LED0_Set(1);
	LED1_Set(1);
	Delay_ms(LED_BOOT_SELFTEST_MS);
	LED0_Set(0);
	LED1_Set(0);
	Delay_ms(100);
#endif
}

void Led_Status_SetState(Led_State_t st)
{
	if(st != s_state)
	{
		s_state = st;
		s_tick = 0;      /* 换状态时重新计时，保证图案从"亮"开始 */
	}
}

Led_State_t Led_Status_GetState(void)
{
	return s_state;
}

void Led_Status_SetLineFound(uint8_t found)
{
	s_line_found = found ? 1 : 0;
}

void Led_Status_Update(void)
{
	uint8_t led0 = 0;
	uint8_t led1 = 0;

	s_tick += LED_TICK_MS;

	switch(s_state)
	{
		case LED_ST_TRACK:
			/* 运行中：LED0 常亮；丢线时 LED1 快闪 */
			led0 = 1;
			led1 = s_line_found ? 0 : (((s_tick % LED_LOST_MS) < LED_LOST_ON_MS) ? 1 : 0);
			break;

		case LED_ST_TEST:
			/* 电机测试：两灯交替闪 */
			led0 = (((s_tick % (2 * LED_TEST_HALF_MS)) < LED_TEST_HALF_MS) ? 1 : 0);
			led1 = led0 ? 0 : 1;
			break;

		case LED_ST_ALARM:
			/* 异常停车：两灯同频快闪 */
			led0 = (((s_tick % (2 * LED_ALARM_HALF_MS)) < LED_ALARM_HALF_MS) ? 1 : 0);
			led1 = led0;
			break;

		case LED_ST_FAN_TEST:
			/* 风扇测试：两灯同频快闪（与异常停车同节奏，但语义是"风扇在跑"） */
			led0 = (((s_tick % (2 * LED_ALARM_HALF_MS)) < LED_ALARM_HALF_MS) ? 1 : 0);
			led1 = led0;
			break;

		case LED_ST_FAN_FAST:
			/* 延时风扇测试（串口 'ff'）：两灯同频超快闪，比快闪再快一倍，
			 * 从"开始等 5 秒"一直闪到"吹完 5 秒自动停" */
			led0 = (((s_tick % (2 * LED_FAN_FAST_HALF_MS)) < LED_FAN_FAST_HALF_MS) ? 1 : 0);
			led1 = led0;
			break;

		case LED_ST_FORCE_ON:
			/* 调试：两灯常亮 */
			led0 = 1;
			led1 = 1;
			break;

		case LED_ST_FORCE_OFF:
			/* 调试：两灯常灭 */
			led0 = 0;
			led1 = 0;
			break;

		case LED_ST_IDLE:
		default:
			/* 待机：两颗灯同步闪（亮 LED_IDLE_ON_MS，灭剩下的时间） */
#if LED_IDLE_BLINK_EN
			led0 = (((s_tick % LED_IDLE_PERIOD_MS) < LED_IDLE_ON_MS) ? 1 : 0);
			led1 = led0;
#else
			led0 = 0;
			led1 = 0;
#endif
			break;
	}

	LED0_Set(led0);
	LED1_Set(led1);
}
