#include "BlackPoint_Finder.h"

/*------------------------------------------------------------------------------
 * 16 路灰度寻线（2026-11-13 改版：固定阈值）
 *   · 赛道：蓝底 + 白线。逐路阈值（s_line_threshold[16]：默认 1999，
 *         p3 单独 1500、p6 单独 1000、p15 单独 900）：
 *         小于 → 白（线）      大于等于 → 蓝（底色）
 *     与旧的白底黑线同向——线始终是 ADC 更小的那一路，所以“找线”依旧是
 *     找最小值，只是判定换成一条全局阈值，不再需要 16 组 min/max。
 *   · 顺序：sensor 0 = 最左（p0），sensor 15 = 最右（p15）。
 *   · 位置：0~15 的浮点数（连续段加权重心），App 层乘 10 存进 position_get。
 *   · 只在 CtrlTask（2 ms）里调用，s_weight 用文件作用域是为了省控制任务的栈
 *     （原来 16 个 float 的归一化数组放在栈上）。
 *----------------------------------------------------------------------------*/
static float  s_weight[SENSOR_COUNT];   /* 每路“白度” = 阈值 − ADC，蓝底为 0 */

/* 逐路阈值表（2026-11-14 起支持单路覆盖）：
 *   p3  = LINE_THRESHOLD_ADC_P3 (1500)
 *   p6  = LINE_THRESHOLD_ADC_P6 (1000)
 *   p15 = LINE_THRESHOLD_ADC_P15 (900)
 *   其余 13 路 = LINE_THRESHOLD_ADC(1999)。
 * 用 const 表而不是运行时填，是因为 IsLine() 可能在 Init() 之前被调用。 */
static const uint16_t s_line_threshold[SENSOR_COUNT] =
{
	/* p0    p1    p2    p3    p4    p5    p6    p7  */
	LINE_THRESHOLD_ADC, LINE_THRESHOLD_ADC, LINE_THRESHOLD_ADC, LINE_THRESHOLD_ADC_P3,
	LINE_THRESHOLD_ADC, LINE_THRESHOLD_ADC, LINE_THRESHOLD_ADC_P6, LINE_THRESHOLD_ADC,
	/* p8    p9    p10   p11   p12   p13   p14   p15 */
	LINE_THRESHOLD_ADC, LINE_THRESHOLD_ADC, LINE_THRESHOLD_ADC, LINE_THRESHOLD_ADC,
	LINE_THRESHOLD_ADC, LINE_THRESHOLD_ADC, LINE_THRESHOLD_ADC, LINE_THRESHOLD_ADC_P15
};

/* 上一次找到的线位置（初始化为中间） */
static uint8_t last_position = SENSOR_COUNT / 2;

/* 上一次的精确位置（初始化为中间） */
static float last_precise_position = (float)(SENSOR_COUNT / 2);

/**
 * @brief 初始化寻线模块
 * @note 固定阈值后没有标定表要填了，只把“上一次位置”复位到中间
 */
void BlackPoint_Finder_Init(void)
{
	last_position = SENSOR_COUNT / 2;
	last_precise_position = (float)(SENSOR_COUNT / 2);
}

/**
 * @brief 判断指定传感器是否压到线（白线）
 * @return 1=线（白，ADC < 阈值），0=底色（蓝）
 */
uint8_t BlackPoint_Finder_IsLine(uint8_t sensor_idx, uint16_t adc_value)
{
	if(sensor_idx >= SENSOR_COUNT)
		return 0;

	return (adc_value < s_line_threshold[sensor_idx]) ? 1u : 0u;
}

/**
 * @brief 寻线主函数：固定阈值 + 连续段加权重心
 * @param adc_values: ADC 值数组（0 = 最左）
 * @param result: 输出结果结构体指针
 * @return 精确位置（0~15）；没看到线时返回上一次的位置
 */
float BlackPoint_Finder_Search(volatile uint16_t *adc_values, BlackPointResult_t *result)
{
	uint8_t i;
	uint8_t i0;
	uint8_t left;
	uint8_t right;
	float   total;
	float   pos_sum;
	float   precise;

	if((adc_values == NULL) || (result == NULL))
	{
		if(result != NULL)
		{
			result->found = 0;
			result->position = last_position;
			result->precise_position = last_precise_position;
		}
		return last_precise_position;
	}

	/* 1. 逐路算“白度”：ADC 低于阈值的深度，越白越大；蓝底记 0。
	 *    权重在阈值处连续降到 0，所以重心对 ADC 是连续的，不会因为某一路
	 *    刚好跨过阈值而跳变。 */
	i0 = 0;
	for(i = 0; i < SENSOR_COUNT; i++)
	{
		uint16_t adc = adc_values[i];
		uint16_t thr = s_line_threshold[i];

		s_weight[i] = (adc < thr)
		            ? (float)(thr - adc)
		            : 0.0f;

		if(s_weight[i] > s_weight[i0])
		{
			i0 = i;
		}
	}

	/* 2. 最白的一路都没过阈值（或只擦了个边）⇒ 这一拍没看到线：
	 *    位置沿用上一次，丢线计时交给 App 层的 App_LostLineGuard()。 */
	if(s_weight[i0] < (float)LINE_MIN_DEPTH_ADC)
	{
		result->found = 0;
		result->position = last_position;
		result->precise_position = last_precise_position;
		return last_precise_position;
	}

	/* 3. 只统计最白那一路所在的**连续段**：远处某一路偶发掉到阈值以下，
	 *    不会被算进来把重心拖偏。 */
	left  = i0;
	right = i0;

	while((left > 0u) && (s_weight[left - 1u] > 0.0f))
	{
		left--;
	}
	while((right < (SENSOR_COUNT - 1u)) && (s_weight[right + 1u] > 0.0f))
	{
		right++;
	}

	/* 4. 段内加权重心：白线只压住一路时结果就是那一路；压在两路之间时
	 *    按两路白度比例插值，得到 0~15 之间的连续位置。 */
	total   = 0.0f;
	pos_sum = 0.0f;

	for(i = left; i <= right; i++)
	{
		total   += s_weight[i];
		pos_sum += s_weight[i] * (float)i;
	}

	precise = (total > 0.0f) ? (pos_sum / total) : (float)i0;

	/* 限制在有效范围内 */
	if(precise < 0.0f)
		precise = 0.0f;
	if(precise > (float)(SENSOR_COUNT - 1))
		precise = (float)(SENSOR_COUNT - 1);

	/* 找到线，更新结果 */
	result->found = 1;
	result->position = i0;
	result->precise_position = precise;
	last_position = i0;
	last_precise_position = precise;

	return precise;
}

/**
 * @brief 获取上一次找到的线位置
 */
uint8_t BlackPoint_Finder_GetLastPosition(void)
{
	return last_position;
}

/**
 * @brief 重置上一次位置为中间位置
 */
void BlackPoint_Finder_ResetLastPosition(void)
{
	last_position = SENSOR_COUNT / 2;
}
