#include "stm32f10x.h"
#include "Delay.h"

/*
 * 微秒延时：优先用 Cortex-M3 的 DWT 周期计数器（72MHz 下 1µs = 72 个周期），
 * 不受编译优化等级影响 —— 原来的 6 次 __nop() 空转只有经过实测校准才准。
 *
 * 这里直接用寄存器地址访问 DWT：工程里的 core_cm3.h 没有导出 DWT 定义
 * （CMSIS 在部分发行版里把 DWT/ITM 放在条件编译块里），而这三组寄存器
 * 在 Cortex-M3 上是固定地址，直接访问既简单又不依赖头文件版本。
 *
 * 若 DWT 不可用（某些调试器/低功耗配置下 CYCCNT 不计数），
 * 自动退回原来的 nop 空转逻辑，保证还能跑。
 */

/* Cortex-M3 私有外设总线上的三个寄存器 */
#define DEMCR_REG          ( *( volatile uint32_t * ) 0xE000EDFCUL )  /* Debug Exception and Monitor Control */
#define DWT_CTRL_REG       ( *( volatile uint32_t * ) 0xE0001000UL )  /* DWT Control */
#define DWT_CYCCNT_REG     ( *( volatile uint32_t * ) 0xE0001004UL )  /* DWT Cycle Counter */

#define DEMCR_TRCENA       ( 1UL << 24 )   /* 使能跟踪/调试单元 */
#define DWT_CTRL_CYCCNTENA ( 1UL << 0 )    /* 使能周期计数器 */

static uint8_t  s_dwt_ok   = 0;
static uint32_t s_us_ticks = 72;           /* 每微秒的系统周期数 */

void Delay_Init(void)
{
	DEMCR_REG      |= DEMCR_TRCENA;
	DWT_CYCCNT_REG  = 0;
	DWT_CTRL_REG   |= DWT_CTRL_CYCCNTENA;

	s_us_ticks = SystemCoreClock / 1000000u;
	if(s_us_ticks == 0)
	{
		s_us_ticks = 72;
	}

	s_dwt_ok = ( ( DWT_CTRL_REG & DWT_CTRL_CYCCNTENA ) != 0 ) ? 1 : 0;
}

/**
  * @brief  微秒级延时
  * @param  xus 延时时长（µs），内部按 60ms 分段，避免 xus * 72 溢出
  * @retval 无
  */
void Delay_us(uint32_t xus)
{
	if(s_dwt_ok)
	{
		while(xus > 60000u)
		{
			uint32_t start = DWT_CYCCNT_REG;
			while( ( DWT_CYCCNT_REG - start ) < ( 60000u * s_us_ticks ) );
			xus -= 60000u;
		}

		{
			uint32_t start = DWT_CYCCNT_REG;
			uint32_t ticks = xus * s_us_ticks;
			while( ( DWT_CYCCNT_REG - start ) < ticks );
		}
	}
	else
	{
		/* 兜底：与旧版一致（-O1 下每 “1µs” 6 个 nop） */
		while(xus--)
		{
			for(uint16_t i = 6; i > 0; i--)
			{
				__nop();
			}
		}
	}
}

/**
  * @brief  毫秒级延时
  * @param  xms 延时时长，范围：0~4294967295
  * @retval 无
  */
void Delay_ms(uint32_t xms)
{
	while(xms--)
	{
		Delay_us(1000);
	}
}

/**
  * @brief  秒级延时
  * @param  xs 延时时长，范围：0~4294967295
  * @retval 无
  */
void Delay_s(uint32_t xs)
{
	while(xs--)
	{
		Delay_ms(1000);
	}
}
