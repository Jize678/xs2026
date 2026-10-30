#ifndef __DELAY_H
#define __DELAY_H

#include <stdint.h>

/* 启动时调用一次：使能 DWT 周期计数器。
 * 不调用时 Delay_us 会退回 nop 空转（旧行为）。 */
void Delay_Init(void);

void Delay_us(uint32_t us);
void Delay_ms(uint32_t ms);
void Delay_s(uint32_t s);

#endif
