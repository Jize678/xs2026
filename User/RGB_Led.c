#include "RGB_Led.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"

// 通道号与引脚的对应关系全部来自 Board_Config.h
typedef struct
{
	GPIO_TypeDef *port;
	uint16_t      pin;
} RGB_Pin_t;

static const RGB_Pin_t rgb_pin[RGB_CHANNEL_COUNT] =
{
	{ RGB0_PORT, RGB0_PIN },   /* RGB-0 -> PA15  LED0 指示灯 */
	{ RGB1_PORT, RGB1_PIN },   /* RGB-1 -> PB3   LED1 指示灯 */
	{ RGB2_PORT, RGB2_PIN },   /* RGB-2 -> PB8   预留车灯 */
	{ RGB3_PORT, RGB3_PIN },   /* RGB-3 -> PB9   预留车灯 */
};

/* 每路的点亮电平：1 = 高电平点亮，0 = 低电平点亮
 * 本车模的 LED0/LED1 是高电平点亮；
 * 预留车灯 RGB-2/RGB-3 将来接上驱动电路后，如果变成低电平点亮，把对应的 1 改成 0 即可。
 */
static const uint8_t rgb_active_high[RGB_CHANNEL_COUNT] =
{
	LED_ACTIVE_HIGH, LED_ACTIVE_HIGH,   /* LED0 / LED1 指示灯 */
	LED_ACTIVE_HIGH, LED_ACTIVE_HIGH    /* 预留车灯 */
};

void RGB_SetChannel(uint8_t channel, uint8_t on)
{
	uint8_t level;

	if(channel >= RGB_CHANNEL_COUNT)
		return;

	if(on)
		level = rgb_active_high[channel];                  /* 点亮电平 */
	else
		level = rgb_active_high[channel] ? 0 : 1;          /* 熄灭电平 */

	GPIO_WriteBit(rgb_pin[channel].port, rgb_pin[channel].pin, level ? Bit_SET : Bit_RESET);
}

void LED0_Set(uint8_t on)
{
	RGB_SetChannel(LED0_CH, on);
}

void LED1_Set(uint8_t on)
{
	RGB_SetChannel(LED1_CH, on);
}

void RGB_AllOff(void)
{
	uint8_t i;
	for(i = 0; i < RGB_CHANNEL_COUNT; i++)
	{
		RGB_SetChannel(i, 0);
	}
}

void RGB_Init(void)
{
	GPIO_InitTypeDef gpio;
	uint8_t i;

	RCC_APB2PeriphClockCmd(RGB_GPIO_RCC | RCC_APB2Periph_AFIO, ENABLE);

	// 释放PB3、PB4、PA15复用功能：关闭JTAG但保留SWD，J-Link可用
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);

	// 四路都配置为推挽输出
	gpio.GPIO_Speed = GPIO_Speed_50MHz;
	gpio.GPIO_Mode = GPIO_Mode_Out_PP;
	for(i = 0; i < RGB_CHANNEL_COUNT; i++)
	{
		gpio.GPIO_Pin = rgb_pin[i].pin;
		GPIO_Init(rgb_pin[i].port, &gpio);
	}

	RGB_AllOff();
}

/* 初始化末尾再调用一次：
 * 其它外设（比如风扇的 TIM2 部分重映射）也会调用 GPIO_PinRemapConfig() 去改 AFIO->MAPR，
 * 而 AFIO->MAPR 里的 SWJ_CFG 是"只写"位（读回来是 0），一旦被重新写入，
 * 就有把 JTAG 重新打开的风险 —— PA15/PB3 一旦被 JTAG 占用，软件怎么写都不亮。
 * 所以在所有外设初始化完成、SysTick 启动之前，再关一次 JTAG 并把 4 路重新配成推挽输出，
 * 确保指示灯的 PA15/PB3 一定处于普通 GPIO 状态。
 */
void RGB_EnsureUsable(void)
{
	GPIO_InitTypeDef gpio;
	uint8_t i;

	RCC_APB2PeriphClockCmd(RGB_GPIO_RCC | RCC_APB2Periph_AFIO, ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);

	gpio.GPIO_Speed = GPIO_Speed_50MHz;
	gpio.GPIO_Mode = GPIO_Mode_Out_PP;
	for(i = 0; i < RGB_CHANNEL_COUNT; i++)
	{
		gpio.GPIO_Pin = rgb_pin[i].pin;
		GPIO_Init(rgb_pin[i].port, &gpio);
	}

	RGB_AllOff();
}
