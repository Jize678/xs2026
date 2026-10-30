#include "Uart_Config.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "misc.h"
#include <stdio.h>
#include "Board_Config.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#ifndef USART2_RX_BUFFER_SIZE
#define USART2_RX_BUFFER_SIZE 128
#endif

static volatile uint8_t  s_usart2_rx_buffer[USART2_RX_BUFFER_SIZE];
static volatile uint16_t s_usart2_rx_head = 0;
static volatile uint16_t s_usart2_rx_tail = 0;

/* 发送互斥（递归：SendString -> SendBuf 会嵌套加锁） */
static SemaphoreHandle_t s_tx_mutex = NULL;

/* 收到字节时唤醒的任务（由串口任务在启动时注册） */
static TaskHandle_t s_rx_notify_task = NULL;

/*------------------------------------------------------------------------------
 * 内部工具
 *----------------------------------------------------------------------------*/

/* 调度器未启动（上电初始化阶段的 printf）或互斥量还没建好时，锁退化为空操作，
 * 此时本来也没有并发，不会出问题。 */
static int Uart2_CanLock(void)
{
	if(s_tx_mutex == NULL)
	{
		return 0;
	}
	if(xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED)
	{
		return 0;
	}
	return 1;
}

static void Uart2_PutChar(uint8_t ch)
{
	while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
	USART_SendData(USART2, ch);
}

/*------------------------------------------------------------------------------
 * 初始化
 *----------------------------------------------------------------------------*/
void Uart2_Init(uint32_t baudrate)
{
	GPIO_InitTypeDef gpio;
	USART_InitTypeDef usart;
	NVIC_InitTypeDef nvic;

	if(baudrate == 0) baudrate = 115200;

	/* 发送互斥量：递归，允许 SendString 内部再调 SendBuf */
	s_tx_mutex = xSemaphoreCreateRecursiveMutex();

	RCC_APB2PeriphClockCmd(UART2_GPIO_RCC | RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

	// PA2: TX 复用推挽
	gpio.GPIO_Pin = UART2_TX_PIN;
	gpio.GPIO_Speed = GPIO_Speed_50MHz;
	gpio.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(UART2_TX_PORT, &gpio);

	// PA3: RX 浮空输入
	gpio.GPIO_Pin = UART2_RX_PIN;
	gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(UART2_RX_PORT, &gpio);

	// USART2 参数
	USART_StructInit(&usart);
	usart.USART_BaudRate = baudrate;
	usart.USART_WordLength = USART_WordLength_8b;
	usart.USART_StopBits = USART_StopBits_1;
	usart.USART_Parity = USART_Parity_No;
	usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	usart.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART2, &usart);

	// 使能接收中断，环形缓冲
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);

	/* 优先级 5 = configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY：
	 * 数值大于等于它（优先级更低）才能调用 FreeRTOS 的 FromISR API。
	 * 原来的 2 会高于 BASEPRI 屏蔽线，在中断里调用 FromISR 会触发断言。 */
	nvic.NVIC_IRQChannel = USART2_IRQn;
	nvic.NVIC_IRQChannelPreemptionPriority = 5;
	nvic.NVIC_IRQChannelSubPriority = 0;
	nvic.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic);

	USART_Cmd(USART2, ENABLE);
}

void Uart2_SetRxNotifyTask(void *task)
{
	s_rx_notify_task = (TaskHandle_t)task;
}

void Uart2_TxLock(void)
{
	if(Uart2_CanLock())
	{
		(void)xSemaphoreTakeRecursive(s_tx_mutex, portMAX_DELAY);
	}
}

void Uart2_TxUnlock(void)
{
	if(Uart2_CanLock())
	{
		(void)xSemaphoreGiveRecursive(s_tx_mutex);
	}
}

/*------------------------------------------------------------------------------
 * 接收中断：入环形缓冲 + 唤醒串口任务
 *----------------------------------------------------------------------------*/
void USART2_IRQHandler(void)
{
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
	{
		uint8_t data = (uint8_t)USART_ReceiveData(USART2);
		uint16_t next = (uint16_t)((s_usart2_rx_head + 1) % USART2_RX_BUFFER_SIZE);

		if(next != s_usart2_rx_tail)
		{
			s_usart2_rx_buffer[s_usart2_rx_head] = data;
			s_usart2_rx_head = next;
		}
		// 否则丢弃字节以避免覆盖

		if(s_rx_notify_task != NULL)
		{
			BaseType_t woken = pdFALSE;
			vTaskNotifyGiveFromISR(s_rx_notify_task, &woken);
			portYIELD_FROM_ISR(woken);
		}
	}
}

/*------------------------------------------------------------------------------
 * 发送
 *----------------------------------------------------------------------------*/
void Uart2_SendByte(uint8_t byte)
{
	Uart2_TxLock();
	Uart2_PutChar(byte);
	while(USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);
	Uart2_TxUnlock();
}

void Uart2_SendBuf(const uint8_t *buf, uint16_t len)
{
	if(buf == 0 || len == 0) return;

	Uart2_TxLock();
	while(len--)
	{
		Uart2_PutChar(*buf++);
	}
	while(USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);
	Uart2_TxUnlock();
}

void Uart2_SendString(const char *str)
{
	if(!str) return;

	Uart2_TxLock();
	while(*str)
	{
		Uart2_PutChar((uint8_t)*str++);
	}
	while(USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);
	Uart2_TxUnlock();
}

/*------------------------------------------------------------------------------
 * 接收（由串口任务调用）
 *----------------------------------------------------------------------------*/
uint8_t Uart2_ReadByteBlocking(void)
{
	while(s_usart2_rx_head == s_usart2_rx_tail);
	uint8_t ch = s_usart2_rx_buffer[s_usart2_rx_tail];
	s_usart2_rx_tail = (uint16_t)((s_usart2_rx_tail + 1) % USART2_RX_BUFFER_SIZE);
	return ch;
}

int Uart2_BytesAvailable(void)
{
	if(s_usart2_rx_head >= s_usart2_rx_tail)
		return (int)(s_usart2_rx_head - s_usart2_rx_tail);
	else
		return (int)(USART2_RX_BUFFER_SIZE - (s_usart2_rx_tail - s_usart2_rx_head));
}

// printf 重定向：单字节阻塞发送，不加锁（多行输出请由调用方用 Uart2_TxLock 包住）
int fputc(int ch, FILE *f)
{
	(void)f;
	Uart2_PutChar((uint8_t)ch);
	return ch;
}
