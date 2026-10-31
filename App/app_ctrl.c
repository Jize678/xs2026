/**
  ******************************************************************************
  * @file    app_ctrl.c
  * @brief   2ms 控制任务 —— 原 SysTick 中断里的全部实时工作搬到这里
  *
  *  每个周期（严格 2ms，vTaskDelayUntil）：
  *    1. 取指令队列里的模式切换 / LED 调试请求（非阻塞）
  *    2. 读双编码器、扫 16 路灰度 + 电池
  *    3. 寻线解算 → position_get，更新事件组里的在/丢线状态
  *    4. 丢线保护（TRACK 模式下连续 1s 丢线 → 回 IDLE + ALARM）
  *    5. 按当前模式执行：PID 闭环 / 电机测试时序 / 待机
  *    6. 刷新指示灯状态机（LED_TICK_MS = 2，与任务周期一致）
  *    7. 喂控制任务看门狗
  *
  *  说明：这个任务独占电机、风扇、LED 的写权限，其它任务只投递指令。
  ******************************************************************************
  */

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "app_config.h"
#include "app_state.h"

#include "ABEncoder.h"
#include "ADC_get.h"
#include "BlackPoint_Finder.h"
#include "PID_Controller.h"
#include "Car_Control.h"
#include "Led_Status.h"
#include "Motor_ctr.h"
#include "M3PWM.h"

/*------------------------------------------------------------------------------
 * 共享采样状态的定义（声明在 app_state.h，PID_Controller.c 里 extern 引用）
 *----------------------------------------------------------------------------*/
int16_t  position_get = 0;
float    BDI_V        = 0.0f;
float    add_angle    = 0.0f;
float    add_angle_num = 0.0f;
/* 推车测距（串口 mc 开关，见 App_SerialTask）：ctrl 任务在 ABEncoder_UpdateSpeed 之后累加，
 * serial 任务只读。volatile 是因为两边在不同任务里读写，单变量读写本身是原子的。 */
volatile uint8_t  g_meas_on    = 0;
volatile int32_t  g_meas_net_l = 0;
volatile int32_t  g_meas_net_r = 0;
volatile uint32_t g_meas_abs_l = 0;
volatile uint32_t g_meas_abs_r = 0;

uint8_t  star_car     = 0;
BlackPointResult_t result_BlackPoint;

/* 控制任务看门狗（一次性软件定时器，20ms 超时） */
static TimerHandle_t s_ctrl_wdt = NULL;

/* 丢线保护计数：500 × 2ms ≈ 1s */
#define APP_LOST_LIMIT   500u

/*------------------------------------------------------------------------------
 * 看门狗超时：控制任务超过 20ms 没有喂狗，说明实时环已经失效
 * 这里直接紧急停车并把指示灯切到 ALARM，等人工发指令恢复
 *----------------------------------------------------------------------------*/
static void App_CtrlWdtCallback(TimerHandle_t xTimer)
{
	(void)xTimer;

	star_car = 0;
	Motor_StopAll();
	Motor_Disable();
	M3PWM_SetDutyCycle(0);
	Led_Status_SetState(LED_ST_ALARM);
}

void App_CtrlWatchdogInit(void)
{
	s_ctrl_wdt = xTimerCreate("ctrl_wdt",
	                          pdMS_TO_TICKS(APP_CTRL_WDT_MS),
	                          pdFALSE,      /* 一次性：每次由控制任务复位 */
	                          NULL,
	                          App_CtrlWdtCallback);
	configASSERT(s_ctrl_wdt != NULL);
}

/*------------------------------------------------------------------------------
 * 处理指令队列：只有本任务会真正执行模式切换
 *----------------------------------------------------------------------------*/
static void App_ProcessCommands(void)
{
	AppCmd_t cmd;

	while(xQueueReceive(g_app_cmd_queue, &cmd, 0) == pdPASS)
	{
		switch(cmd.type)
		{
			case APP_CMD_MODE:
				Car_Control_ApplyMode((Car_Mode_t)cmd.arg);
				break;

			case APP_CMD_LED_FORCE_ON:
				Led_Status_SetState(LED_ST_FORCE_ON);
				break;

			case APP_CMD_LED_FORCE_OFF:
				Led_Status_SetState(LED_ST_FORCE_OFF);
				break;

			case APP_CMD_WAVE_TOGGLE:
				g_wave_enable = (uint8_t)(g_wave_enable ? 0u : 1u);
				if(g_wave_enable)
				{
					xEventGroupSetBits(g_app_events, APP_EVT_WAVE_ON);
				}
				else
				{
					xEventGroupClearBits(g_app_events, APP_EVT_WAVE_ON);
				}
				break;

			case APP_CMD_WHEEL_LEFT:
				Car_Control_WheelRun(CAR_WHEEL_LEFT, (uint16_t)cmd.arg * 100u);
				break;

			case APP_CMD_WHEEL_RIGHT:
				Car_Control_WheelRun(CAR_WHEEL_RIGHT, (uint16_t)cmd.arg * 100u);
				break;

			case APP_CMD_FAN_TEST:
				Car_Control_FanTest((uint16_t)cmd.arg * 10u);
				break;

			case APP_CMD_FAN_DELAY_TEST:
				/* 'ff'：先空等 CAR_FAN_DELAY_MS，再按占空比吹 CAR_FAN_TEST_MS */
				Car_Control_FanDelayedTest((uint16_t)cmd.arg * 10u);
				break;

			case APP_CMD_ENC_MON:
				/* 'bmq'：只翻转串口监视开关，不碰电机/风扇/模式 */
				g_enc_mon = (uint8_t)(g_enc_mon ? 0u : 1u);
				break;

			case APP_CMD_SPEED_TEST_L:
				/* 'testl<0~400>'：左轮速度环标定 */
				Car_Control_SpeedTest(CAR_WHEEL_LEFT, cmd.arg16);
				break;

			case APP_CMD_SPEED_TEST_R:
				/* 'testr<0~400>'：右轮速度环标定 */
				Car_Control_SpeedTest(CAR_WHEEL_RIGHT, cmd.arg16);
				break;

			case APP_CMD_TUNE_ENTER:
				/* tune：进入调参模式（停车、风扇停、两环复位） */
				Car_Control_TuneEnter();
				break;

			case APP_CMD_TUNE_RUN:
				/* ! <ms> <l|r|all>：定时闭环 */
				Car_Control_TuneRun(cmd.arg, (uint16_t)cmd.arg16);
				break;

			case APP_CMD_TUNE_HALT:
				/* hlt：目标清零并停车，留在调参模式里 */
				Car_Control_TuneHalt();
				break;

			case APP_CMD_TUNE_TARGET:
				/* vl/vr：arg = 左右轮，arg16 = 目标 mm/s */
				Car_Control_TuneSetTarget((cmd.arg == 0u) ? CAR_WHEEL_LEFT : CAR_WHEEL_RIGHT,
				                          cmd.arg16);
				break;

			case APP_CMD_TUNE_GAIN:
				/* kl/il/dl/pt…：arg = 轮*3 + 项，arg16 = 增益×1000 */
				Car_Control_TuneSetGain((cmd.arg >= 3u) ? CAR_WHEEL_RIGHT : CAR_WHEEL_LEFT,
				                        (Car_TuneTerm_t)(cmd.arg % 3u),
				                        (float)(cmd.arg16) / 1000.0f);
				break;

			case APP_CMD_TUNE_FAN:
				/* tf：arg = 百分比 0~100，只设设定值，下次 `!` 实验才吹 */
				Car_Control_TuneSetFan((uint16_t)cmd.arg * 10u);
				break;

			case APP_CMD_TUNE_RAMP:
				/* tr：arg16 = 软起动时长 ms（0 = 不软起动），下次 `!` 生效 */
				Car_Control_TuneSetRamp((cmd.arg16 > 0) ? (uint16_t)cmd.arg16 : 0u);
				break;

			default:
				break;
		}
	}
}

/*------------------------------------------------------------------------------
 * 丢线保护：只在 TRACK 模式下计时，切模式时清零重新数
 *----------------------------------------------------------------------------*/
static void App_LostLineGuard(uint8_t found)
{
	static uint32_t  lose_time = 0;
	static Car_Mode_t last_mode = CAR_MODE_IDLE;

	if(Car_Control_GetMode() != last_mode)
	{
		last_mode = Car_Control_GetMode();
		lose_time = 0;
	}

	if(Car_Control_GetMode() != CAR_MODE_TRACK)
	{
		return;
	}

	if(found)
	{
		if(lose_time > 0u)
		{
			lose_time--;
		}
	}
	else
	{
		lose_time++;
		if(lose_time > APP_LOST_LIMIT)
		{
			lose_time = APP_LOST_LIMIT;
			Car_Control_ApplyMode(CAR_MODE_IDLE);
			Led_Status_SetState(LED_ST_ALARM);
		}
	}
}

/*------------------------------------------------------------------------------
 * 事件组状态发布（只在状态变化时写，避免每 2ms 无谓地进出临界区）
 *----------------------------------------------------------------------------*/
static void App_PublishLineState(uint8_t found)
{
	static uint8_t last_found = 0xFFu;    /* 0xFF：强制第一次发布 */

	if(found == last_found)
	{
		return;
	}
	last_found = found;

	if(found)
	{
		xEventGroupSetBits(g_app_events, APP_EVT_LINE_FOUND);
		xEventGroupClearBits(g_app_events, APP_EVT_LINE_LOST);
	}
	else
	{
		xEventGroupSetBits(g_app_events, APP_EVT_LINE_LOST);
		xEventGroupClearBits(g_app_events, APP_EVT_LINE_FOUND);
	}
}

/*------------------------------------------------------------------------------
 * 控制任务主体
 *----------------------------------------------------------------------------*/
void App_CtrlTask(void *argument)
{
	TickType_t last_wake = xTaskGetTickCount();
	uint8_t    last_alarm = 0xFFu;
	(void)argument;

	for(;;)
	{
		/* 1. 指令（模式切换 / LED 调试 / 波形开关） */
		App_ProcessCommands();

		/* 2. 传感器：双编码器 → 16 路灰度 + 电池（陀螺仪没装，不再有 IMU 采样） */

		ABEncoder_UpdateSpeed();

	/* 推车测距：把每 2ms 的增量累加进去（只在 mc 打开时） */
	if(g_meas_on != 0u)
	{
		int16_t dl = (int16_t)speed_left;
		int16_t dr = (int16_t)speed_right;

		g_meas_net_l += dl;
		g_meas_net_r += dr;
		g_meas_abs_l += (uint32_t)((dl >= 0) ? dl : -dl);
		g_meas_abs_r += (uint32_t)((dr >= 0) ? dr : -dr);
	}
		MuxADC_SampleAll();
		BDI_V = (float)g_mux_adc_values[16] * 0.00426508726f;

		/* 3. 寻线：找到线才刷新位置，找不到时保持上一次位置（与原逻辑一致） */
		BlackPoint_Finder_Search(g_mux_adc_values, &result_BlackPoint);
		Led_Status_SetLineFound(result_BlackPoint.found);

		if(result_BlackPoint.found)
		{
			position_get = (int16_t)(result_BlackPoint.precise_position * 10.0f);
		}
		App_PublishLineState(result_BlackPoint.found);

		/* 4. 丢线保护 */
		App_LostLineGuard(result_BlackPoint.found);

		/* 5. 执行当前模式的动作（PID / 电机测试 / 待机） */
		Car_Control_Update();

		/* 6. 指示灯状态机，周期 2ms 与 LED_TICK_MS 一致 */
		Led_Status_Update();

		/* 7. 发布 ALARM 位 */
		{
			uint8_t alarm = (Led_Status_GetState() == LED_ST_ALARM) ? 1u : 0u;
			if(alarm != last_alarm)
			{
				last_alarm = alarm;
				if(alarm)
				{
					xEventGroupSetBits(g_app_events, APP_EVT_ALARM);
				}
				else
				{
					xEventGroupClearBits(g_app_events, APP_EVT_ALARM);
				}
			}
		}

		/* 8. 喂看门狗（20ms 超时，2ms 周期有 10 倍余量） */
		if(s_ctrl_wdt != NULL)
		{
			xTimerReset(s_ctrl_wdt, 0);
		}

		vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(APP_CTRL_PERIOD_MS));
	}
}
