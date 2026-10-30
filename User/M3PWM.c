#include "M3PWM.h"
#include "Board_Config.h"
#include "Delay.h"
#include <stdio.h>

static uint16_t g_duty = 0;      /* 0 ~ 1000 千分比 */

static uint16_t M3PWM_Clamp(uint16_t duty)
{
	return (duty > 1000u) ? 1000u : duty;
}

/**
  * @brief  风机 PWM 初始化（PB11 / TIM2_CH4，部分重映射2）
  * @note   舵机模式：50Hz、1000~2000us；初始化末尾保持最低油门 FAN_ARM_DELAY_MS
  *         等电调解锁，否则上电后立刻推油门电调不会响应（听起来就是"不转"）。
  */
void M3PWM_Init(void)
{
	GPIO_InitTypeDef        GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef       TIM_OCInitStructure = {0};   /* 显式清零：不设的字段不要带栈上垃圾 */

	RCC_APB2PeriphClockCmd(FAN_PWM_GPIO_RCC | RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB1PeriphClockCmd(FAN_PWM_TIM_RCC, ENABLE);

	/* TIM2 部分重映射2：CH4 -> PB11（默认 CH4 在 PA3，会和 USART2_RX 抢脚） */
	GPIO_PinRemapConfig(FAN_PWM_REMAP, ENABLE);

	GPIO_InitStructure.GPIO_Pin   = FAN_PWM_PIN;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(FAN_PWM_PORT, &GPIO_InitStructure);

#if FAN_PWM_MODE_SERVO
	TIM_TimeBaseStructure.TIM_Period    = FAN_SERVO_ARR;   /* 20ms -> 50Hz */
	TIM_TimeBaseStructure.TIM_Prescaler = FAN_SERVO_PSC;   /* 1MHz -> 1us 分辨率 */
#else
	TIM_TimeBaseStructure.TIM_Period    = FAN_FAST_ARR;    /* 17kHz */
	TIM_TimeBaseStructure.TIM_Prescaler = FAN_FAST_PSC;
#endif
	TIM_TimeBaseStructure.TIM_ClockDivision     = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode       = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(FAN_PWM_TIM, &TIM_TimeBaseStructure);

	TIM_OCInitStructure.TIM_OCMode       = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState  = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
	TIM_OCInitStructure.TIM_OCPolarity   = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OCNPolarity  = TIM_OCNPolarity_High;
	TIM_OCInitStructure.TIM_OCIdleState  = TIM_OCIdleState_Reset;
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;
#if FAN_PWM_MODE_SERVO
	TIM_OCInitStructure.TIM_Pulse = FAN_SERVO_MIN_US;      /* 先给最低油门，电调才肯解锁 */
#else
	TIM_OCInitStructure.TIM_Pulse = 0;
#endif
	TIM_OC4Init(FAN_PWM_TIM, &TIM_OCInitStructure);

	TIM_Cmd(FAN_PWM_TIM, ENABLE);

	g_duty = 0;
	M3PWM_SetDutyCycle(0);

#if (FAN_PWM_MODE_SERVO && (FAN_ARM_DELAY_MS > 0))
	Delay_ms(FAN_ARM_DELAY_MS);    /* 保持最低油门，等电调自检解锁（会"嘀—嘀"两声） */
#endif
}

/**
  * @brief  设油门（0~1000 千分比）
  * @note   舵机模式换算成 1000~2000us 脉宽；0 不是"没有信号"，而是最低油门 1000us，
  *         这样电调一直保持解锁状态，随时推油门都能立刻响应。
  */
void M3PWM_SetDutyCycle(uint16_t duty)
{
	duty   = M3PWM_Clamp(duty);
	g_duty = duty;

#if FAN_PWM_MODE_SERVO
	M3PWM_SetPulseUs((uint16_t)(FAN_SERVO_MIN_US + duty));
#else
	TIM_SetCompare4(FAN_PWM_TIM, (uint16_t)(((uint32_t)(FAN_PWM_TIM->ARR + 1) * duty) / 1000u));
#endif
}

/**
  * @brief  直接指定高电平脉宽（1000~2000us），舵机模式下用来做精细标定
  */
void M3PWM_SetPulseUs(uint16_t us)
{
#if FAN_PWM_MODE_SERVO
	if(us < FAN_SERVO_MIN_US) us = FAN_SERVO_MIN_US;
	if(us > FAN_SERVO_MAX_US) us = FAN_SERVO_MAX_US;

	g_duty = (uint16_t)(us - FAN_SERVO_MIN_US);
	TIM_SetCompare4(FAN_PWM_TIM, us);
#else
	(void)us;
#endif
}

uint16_t M3PWM_GetPulseUs(void)
{
#if FAN_PWM_MODE_SERVO
	return (uint16_t)TIM_GetCapture4(FAN_PWM_TIM);
#else
	return 0;
#endif
}

uint16_t M3PWM_GetDuty(void)
{
	return g_duty;
}

void M3PWM_Start(void)
{
	TIM_Cmd(FAN_PWM_TIM, ENABLE);
}

void M3PWM_Stop(void)
{
	TIM_Cmd(FAN_PWM_TIM, DISABLE);
}

/**
  * @brief  改 PWM 频率（只在"有刷风机"模式下有意义）
  */
void M3PWM_SetFrequency(uint32_t freq)
{
#if FAN_PWM_MODE_SERVO
	(void)freq;    /* 舵机信号必须 50Hz，不允许改 */
#else
	uint32_t psc, arr;

	if(freq < 1u)       freq = 1u;
	if(freq > 1000000u) freq = 1000000u;

	psc = 72000000u / (freq * 1000u);
	if(psc > 0u) psc -= 1u;
	if(psc > 65535u) psc = 65535u;

	arr = 72000000u / (freq * (psc + 1u));
	if(arr > 0u) arr -= 1u;
	if(arr > 65535u) arr = 65535u;

	TIM_PrescalerConfig(FAN_PWM_TIM, (uint16_t)psc, TIM_PSCReloadMode_Immediate);
	TIM_SetAutoreload(FAN_PWM_TIM, (uint16_t)arr);

	M3PWM_SetDutyCycle(g_duty);
#endif
}

/**
  * @brief  打印风机 PWM 的实测状态，并对每一项给出 OK / NG 判定
  * @note   串口 'p' 调用。检查五件事：
  *         1) 时基是不是 50Hz 那套（PSC=71、ARR=19999）；
  *         2) PB11 是否被配成复用推挽（CRH[15:12] = 0xB）；
  *         3) TIM2_REMAP 是否为 10b（=2，部分重映射2 → CH4 在 PB11）；
  *         4) CCR4 是否在 1000~2000（= 高电平微秒数）；
  *         5) 计数器是否真的在走（间隔 2ms 采样两次 CNT）。
  *         五项都 OK 才打印 PASS —— 此时 PB11 上一定是标准 50Hz 舵机信号。
  */
void M3PWM_PrintDiag(void)
{
	uint32_t psc    = FAN_PWM_TIM->PSC;
	uint32_t arr    = FAN_PWM_TIM->ARR;
	uint32_t ccr    = FAN_PWM_TIM->CCR4;
	uint32_t cnt_a  = FAN_PWM_TIM->CNT;
	uint32_t crh    = GPIOB->CRH;
	uint32_t mapr   = AFIO->MAPR;
	uint32_t remap  = (mapr >> 8) & 0x3u;
	uint32_t nibble = (crh >> 12) & 0xFu;
	uint32_t cnt_b;
	uint8_t  ok_hz, ok_remap, ok_gpio, ok_ccr, ok_run;

	Delay_ms(2);                     /* 让计数器至少走过一个 2ms 窗口 */
	cnt_b = FAN_PWM_TIM->CNT;

#if FAN_PWM_MODE_SERVO
	ok_hz = (psc == FAN_SERVO_PSC) && (arr == FAN_SERVO_ARR);
	ok_ccr = (ccr >= FAN_SERVO_MIN_US) && (ccr <= FAN_SERVO_MAX_US);
#else
	ok_hz = (psc == FAN_FAST_PSC) && (arr == FAN_FAST_ARR);
	ok_ccr = (ccr <= arr);
#endif
	ok_remap = (remap == 0x2u);      /* 10b = 部分重映射2 -> CH4 = PB11 */
	ok_gpio  = (nibble == 0xBu);     /* 1011b = 复用推挽 50MHz */
	ok_run   = (cnt_b != cnt_a);

	printf("[fan] mode=%s  duty=%u/1000\r\n",
	       FAN_PWM_MODE_SERVO ? "SERVO 50Hz" : "FAST 17kHz", (unsigned int)g_duty);
	printf("[fan] PSC=%lu ARR=%lu  %s\r\n",
	       (unsigned long)psc, (unsigned long)arr, ok_hz ? "OK" : "NG");
	printf("[fan] PB11 CRH[15:12]=0x%lX  %s (want 0xB = AF_PP 50MHz)\r\n",
	       (unsigned long)nibble, ok_gpio ? "OK" : "NG");
	printf("[fan] TIM2_REMAP=%lu  %s (want 2 = TIM2 partial remap 2)\r\n",
	       (unsigned long)remap, ok_remap ? "OK" : "NG");
	printf("[fan] CCR4=%lu  %s (want 1000~2000us)\r\n",
	       (unsigned long)ccr, ok_ccr ? "OK" : "NG");
	printf("[fan] CNT %lu -> %lu  %s\r\n",
	       (unsigned long)cnt_a, (unsigned long)cnt_b, ok_run ? "OK" : "NG");

	if(ok_hz && ok_remap && ok_gpio && ok_ccr && ok_run)
	{
		printf("[fan] RESULT: PASS, PB11 outputs 50Hz servo pulses (%luus)\r\n", (unsigned long)ccr);
	}
	else
	{
		printf("[fan] RESULT: FAIL, check the NG lines above\r\n");
	}
}
