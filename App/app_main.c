/**
  ******************************************************************************
  * @file    app_main.c
  * @brief   FreeRTOS 版主入口：硬件初始化 → 创建 IPC/任务 → 启动调度器
  *
  *  与旧版 User\main.c 的差异：
  *    - 不再有 while(1) 超级循环，也没有 SysTick 里的业务代码；
  *    - SysTick 由 FreeRTOS 接管（1ms tick），系统时基来自内核；
  *    - 初始化顺序、引脚、限幅、指令语义全部保持不变；
  *    - 新增 App_CtrlWatchdogInit()：控制任务 20ms 不喂狗就紧急停车。
  *
  *  启动耗尽：启动文件里 Stack_Size = 0x400（1KB）只给 main() 用，
  *  调度器启动前的初始化调用链都在这 1KB 上跑，任务栈另由 heap 分配。
  ******************************************************************************
  */

#include "FreeRTOS.h"
#include "task.h"

#include "app_config.h"
#include "app_state.h"

#include "misc.h"                /* NVIC_PriorityGroupConfig */
#include "Delay.h"
#include "RGB_Led.h"
#include "Led_Status.h"
#include "OLED.h"
#include "M3PWM.h"
#include "Motor_ctr.h"
#include "ABEncoder.h"
#include "BlackPoint_Finder.h"
#include "ADC_get.h"
#include "Key_Scan.h"
#include "Uart_Config.h"
#include "PID_Controller.h"
#include "Car_Control.h"

/*------------------------------------------------------------------------------
 * 硬件初始化（顺序与原 main.c 保持一致）
 *----------------------------------------------------------------------------*/
static void App_HardwareInit(void)
{
	/* 指示灯：先把 JTAG 关掉，PA15/PB3 才能当普通 GPIO 用 */
	RGB_Init();
	Led_Status_SelfTest();          /* 上电自检：两灯亮 500ms（阻塞 0.5s） */

	OLED_Init();

	M3PWM_Init();                   /* 负压风机 TIM2_CH4 */
	Motor_Init();                   /* 双电机 TIM1_CH2/CH4 + 方向/使能 */
	M3PWM_Start();                  /* 风机先起转（占空比 0，等 TRACK 再给 950） */

	ABEncoder_Init();
	BlackPoint_Finder_Init();       /* 16 路灰度标定表 */
	MuxADC_Init();                  /* 模拟开关 + ADC1 校准 */

	Key_Scan_Init();
	Uart2_Init(115200);             /* USART2 中断优先级 5，可调用 FromISR */

	PID_Init();                     /* 控制器参数 + 限幅 */
	Car_Control_Init();             /* 进入 IDLE：电机禁用、风扇 0、两环复位 */

	/* 所有外设初始化完、SysTick 启动前，再关一次 JTAG 并重配四路指示灯：
	 * 别的外设可能重写 AFIO->MAPR，而 SWJ_CFG 是只写位，读回来是 0。 */
	RGB_EnsureUsable();
}

/*------------------------------------------------------------------------------
 * 任务创建（内存不够时 configASSERT 会在启动阶段停住，便于排查）
 *----------------------------------------------------------------------------*/
static void App_CreateTasks(void)
{
	BaseType_t ok;

	ok = xTaskCreate(App_CtrlTask,    "ctrl",   APP_STACK_CTRL,    NULL, APP_PRIO_CTRL,    &g_task_ctrl);
	configASSERT(ok == pdPASS);
	ok = xTaskCreate(App_KeysTask,    "keys",   APP_STACK_KEYS,    NULL, APP_PRIO_KEYS,    &g_task_keys);
	configASSERT(ok == pdPASS);
	ok = xTaskCreate(App_DisplayTask, "disp",   APP_STACK_DISPLAY, NULL, APP_PRIO_DISPLAY, &g_task_display);
	configASSERT(ok == pdPASS);
	ok = xTaskCreate(App_SerialTask,  "serial", APP_STACK_SERIAL,  NULL, APP_PRIO_SERIAL,  &g_task_serial);
	configASSERT(ok == pdPASS);
}

/*------------------------------------------------------------------------------
 * 内核钩子
 *----------------------------------------------------------------------------*/
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
	(void)xTask;
	(void)pcTaskName;

	/* 栈溢出：停掉调度器（保持中断关闭）等调试器/复位 */
	taskDISABLE_INTERRUPTS();
	for(;;)
	{
	}
}

void vApplicationMallocFailedHook(void)
{
	/* pvPortMalloc 失败（堆 7KB 不够）：同上 */
	taskDISABLE_INTERRUPTS();
	for(;;)
	{
	}
}

/*------------------------------------------------------------------------------
 * main
 *----------------------------------------------------------------------------*/
int main(void)
{
	/* 4 位全给抢占优先级：FreeRTOS 的 BASEPRI 屏蔽要求分组为 4 */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

	/* DWT 周期计数器做微秒延时（ADC 模拟开关切换、OLED 软 I2C 用到） */
	Delay_Init();

	App_HardwareInit();

	/* 队列 / 事件组 / 软件定时器 */
	App_StateInit();
	App_CtrlWatchdogInit();

	App_CreateTasks();

	/* 启动调度器：正常情况下不返回 */
	vTaskStartScheduler();

	/* 只有堆不足、调度器起不来时才会走到这里 */
	Motor_StopAll();
	Motor_Disable();
	M3PWM_SetDutyCycle(0);

	for(;;)
	{
	}
}
