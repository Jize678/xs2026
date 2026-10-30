/**
  ******************************************************************************
  * @file    stm32f10x_it.c
  * @brief   中断向量实现（FreeRTOS 版）
  *
  *  与旧版相比删掉了：
  *    - SVC_Handler / PendSV_Handler / SysTick_Handler
  *      → 这三个由 FreeRTOS 内核（port.c）通过 FreeRTOSConfig.h 里的
  *        vPortSVCHandler / xPortPendSVHandler / xPortSysTickHandler 映射接管；
  *    - 2ms SysTick 里的全部业务代码（IMU/编码器/ADC/寻线/PID/LED）
  *      → 已迁移到 App\app_ctrl.c 的 2ms 控制任务。
  *
  *  保留：内核异常处理（HardFault 等），便于调试时断点定位。
  ******************************************************************************
  */

#include "stm32f10x_it.h"

/* 内核异常：FreeRTOS 没有 hook 覆盖这些，出错时停在原地等调试器 */
void NMI_Handler(void)
{
	while(1)
	{
	}
}

void HardFault_Handler(void)
{
	while(1)
	{
	}
}

void MemManage_Handler(void)
{
	while(1)
	{
	}
}

void BusFault_Handler(void)
{
	while(1)
	{
	}
}

void UsageFault_Handler(void)
{
	while(1)
	{
	}
}
