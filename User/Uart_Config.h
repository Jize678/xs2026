#ifndef __UART_CONFIG_H__
#define __UART_CONFIG_H__

#include "stm32f10x.h"
#include <stdint.h>

// USART2: PA2 (TX), PA3 (RX)
//
// FreeRTOS 版新增：
//   - USART2 中断优先级 = 5（= configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY），
//     因此可以在中断里安全调用 vTaskNotifyGiveFromISR()；
//   - 发送串行化：TX 递归互斥量，printf 与波形帧不会互相打断；
//   - RX 任务通知：中断收到字节后 wakup 串口任务，不再靠轮询/阻塞读。

void Uart2_Init(uint32_t baudrate);
void Uart2_SendByte(uint8_t byte);
void Uart2_SendBuf(const uint8_t *buf, uint16_t len);
void Uart2_SendString(const char *str);
uint8_t Uart2_ReadByteBlocking(void);
int Uart2_BytesAvailable(void);

/* 发送互斥（递归）：整段多行输出时用，避免被波形帧/其它任务插队 */
void Uart2_TxLock(void);
void Uart2_TxUnlock(void);

/* 注册“收到字节时唤醒哪个任务”（传 xTaskGetCurrentTaskHandle() 的值） */
void Uart2_SetRxNotifyTask(void *task);

#endif
