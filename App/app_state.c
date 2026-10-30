/**
  ******************************************************************************
  * @file    app_state.c
  * @brief   任务间共享对象的创建与访问
  ******************************************************************************
  */

#include "app_state.h"
#include "app_config.h"

QueueHandle_t      g_app_cmd_queue = NULL;
EventGroupHandle_t g_app_events    = NULL;

TaskHandle_t g_task_ctrl    = NULL;
TaskHandle_t g_task_keys    = NULL;
TaskHandle_t g_task_display = NULL;
TaskHandle_t g_task_serial  = NULL;

volatile uint8_t g_wave_enable = 0;
volatile uint8_t g_enc_mon     = 0;

void App_StateInit(void)
{
	g_app_cmd_queue = xQueueCreate(APP_CMD_QUEUE_LEN, sizeof(AppCmd_t));
	g_app_events    = xEventGroupCreate();

	/* 堆只有 7KB，创建失败说明 configTOTAL_HEAP_SIZE 或栈配置需要调整 */
	configASSERT(g_app_cmd_queue != NULL);
	configASSERT(g_app_events != NULL);
}

BaseType_t App_PostCmd(AppCmdType_t type, uint8_t arg)
{
	return App_PostCmdArg(type, arg, 0);
}

BaseType_t App_PostCmdArg(AppCmdType_t type, uint8_t arg, int16_t arg16)
{
	AppCmd_t cmd;

	if(g_app_cmd_queue == NULL)
	{
		return pdFAIL;
	}

	cmd.type  = type;
	cmd.arg   = arg;
	cmd.arg16 = arg16;

	/* 不允许在中断里调用：需要从 ISR 投递时请用 xQueueSendFromISR */
	return xQueueSend(g_app_cmd_queue, &cmd, 0);
}
