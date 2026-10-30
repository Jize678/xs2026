/**
  ******************************************************************************
  * @file    app_display.c
  * @brief   OLED 显示任务（50ms 刷新）
  *
  *  显示内容（与原主循环一致，4 行 × 16 列 8x16 字库）：
  *    第 1 行：运行模式 / ALARM
  *    第 2 行：寻线位置 position_get（0~150，×10 存储）
  *    第 3 行：电池电压 BDI_V × 100
  *    第 4 行：左右轮速度之和（编码器 2ms 差分）
  *
  *  软 I2C 全屏刷新约 20~30ms，所以周期放到 50ms：显示延迟无感，
  *  同时给控制任务（优先级更高）留出足够的 CPU。
  ******************************************************************************
  */

#include "FreeRTOS.h"
#include "task.h"
#include "app_config.h"
#include "app_state.h"

#include "OLED.h"
#include "Car_Control.h"
#include "Led_Status.h"
#include "ABEncoder.h"

void App_DisplayTask(void *argument)
{
	TickType_t last_wake = xTaskGetTickCount();
	(void)argument;

	for(;;)
	{
		/* 第 1 行：模式；异常停车时优先显示 ALARM */
		if(Led_Status_GetState() == LED_ST_ALARM)
		{
			OLED_ShowString(1, 1, (char *)"ALARM");
		}
		else
		{
			OLED_ShowString(1, 1, (char *)Car_Control_GetModeName());
		}

		/* 第 2 行：寻线位置（0~150） */
		OLED_ShowSignedNum(2, 1, (int32_t)position_get, 5);

		/* 第 3 行：电池电压 ×100，例如 12.34V 显示 1234 */
		OLED_ShowSignedNum(3, 1, (int32_t)(BDI_V * 100.0f), 5);

		/* 第 4 行：左右轮速度之和 */
		OLED_ShowSignedNum(4, 1, (int32_t)(speed_left_mm_s + speed_right_mm_s), 5);

		vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(APP_DISPLAY_PERIOD_MS));
	}
}
