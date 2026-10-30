/**
  ******************************************************************************
  * @file    app_keys.c
  * @brief   按键任务（10ms 轮询）
  *
  *  Key_Scan 的语义是“释放沿才产生事件”（按下 → 松开 才算一次），所以这里
  *  每 10ms 调一次 Key_Scan_Update()，再把事件翻译成指令投递给控制任务。
  *  不在本任务里直接改电机/模式，避免与 2ms 控制任务抢外设。
  *
  *  按键功能：
  *    K1 (PA5)  → 循迹发车 TRACK
  *    K2 (PA4)  → 停车回 IDLE
  *    K3 (PC14) → 切换波形帧输出（VOFA+ JustFloat）
  *    K4 (PC13) → 预留（不动）
  ******************************************************************************
  */

#include "FreeRTOS.h"
#include "task.h"
#include "app_config.h"
#include "app_state.h"
#include "Key_Scan.h"

void App_KeysTask(void *argument)
{
	TickType_t   last_wake = xTaskGetTickCount();
	Key_Event_t *ev;
	(void)argument;

	for(;;)
	{
		Key_Scan_Update();

		while((ev = Key_GetEvent()) != NULL)
		{
			switch(ev->key_id)
			{
				case KEY_K1:
					App_PostCmd(APP_CMD_MODE, (uint8_t)CAR_MODE_TRACK);
					break;

				case KEY_K2:
					App_PostCmd(APP_CMD_MODE, (uint8_t)CAR_MODE_IDLE);
					break;

				case KEY_K3:
					App_PostCmd(APP_CMD_WAVE_TOGGLE, 0);
					break;

				case KEY_K4:
				default:
					/* 预留：暂不绑定动作 */
					break;
			}
		}

		vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(APP_KEYS_PERIOD_MS));
	}
}
