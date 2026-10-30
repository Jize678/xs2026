#include "BlackPoint_Finder.h"
#include <math.h>

// 传感器配置数组
static SensorConfig_t sensor_config[SENSOR_COUNT];

// 上一次找到的黑点位置（初始化为中间位置）
static uint8_t last_position = SENSOR_COUNT / 2;

// 上一次找到的精确位置（初始化为中间位置）
static float last_precise_position = (float)(SENSOR_COUNT / 2);

/**
 * @brief 初始化寻点模块
 * @note 设置默认的最小值和最大值参数
 */
void BlackPoint_Finder_Init(void)
{
	
//	// 初始化所有传感器的默认配置
//	// 默认值：最小值=100（白纸），最大值=3500（黑点）
//	// 实际使用时需要通过校准或调参来设置正确的值
//	for(i = 0; i < SENSOR_COUNT; i++)
//	{
//		sensor_config[i].min_value = 100;   // 默认最小值（白纸）
//		sensor_config[i].max_value = 3500;  // 默认最大值（黑点）
//	}
			sensor_config[0].min_value = 200;sensor_config[0].max_value = 1400;
			sensor_config[1].min_value = 200;sensor_config[1].max_value = 1700;
			sensor_config[2].min_value = 320;sensor_config[2].max_value = 1660;
			sensor_config[3].min_value = 200;sensor_config[3].max_value = 2100;
			sensor_config[4].min_value = 200;sensor_config[4].max_value = 2000;
			sensor_config[5].min_value = 200;sensor_config[5].max_value = 1770;
			sensor_config[6].min_value = 150;sensor_config[6].max_value = 1770;
			sensor_config[7].min_value = 200;sensor_config[7].max_value = 2220;
			sensor_config[8].min_value = 200;sensor_config[8].max_value = 2250;
			sensor_config[9].min_value = 200;sensor_config[9].max_value = 2100;
			sensor_config[10].min_value = 200;sensor_config[10].max_value = 1700;
			sensor_config[11].min_value = 200;sensor_config[11].max_value = 1760;
			sensor_config[12].min_value = 200;sensor_config[12].max_value = 1980;
			sensor_config[13].min_value = 200;sensor_config[13].max_value = 2530;
			sensor_config[14].min_value = 220;sensor_config[14].max_value = 2310;
			sensor_config[15].min_value = 230;sensor_config[15].max_value = 1790;
	// 初始化上一次位置为中间
	last_position = SENSOR_COUNT / 2;
	last_precise_position = (float)(SENSOR_COUNT / 2);
}

/**
 * @brief 设置指定传感器的最小值和最大值
 */
void BlackPoint_Finder_SetSensorConfig(uint8_t sensor_idx, uint16_t min_value, uint16_t max_value)
{
	if(sensor_idx < SENSOR_COUNT)
	{
		sensor_config[sensor_idx].min_value = min_value;
		sensor_config[sensor_idx].max_value = max_value;
	}
}

/**
 * @brief 获取指定传感器的配置
 */
void BlackPoint_Finder_GetSensorConfig(uint8_t sensor_idx, uint16_t *min_value, uint16_t *max_value)
{
	if(sensor_idx < SENSOR_COUNT && min_value != NULL && max_value != NULL)
	{
		*min_value = sensor_config[sensor_idx].min_value;
		*max_value = sensor_config[sensor_idx].max_value;
	}
}

/**
 * @brief 判断指定传感器是否为黑点
 * @return 1=黑点，0=白点
 */
uint8_t BlackPoint_Finder_IsBlackPoint(uint8_t sensor_idx, uint16_t adc_value)
{
	uint16_t min_val, max_val;
	uint16_t threshold;
	
	if(sensor_idx >= SENSOR_COUNT)
		return 0;
	
	min_val = sensor_config[sensor_idx].min_value;
	max_val = sensor_config[sensor_idx].max_value;
	
	// 计算阈值：(max - min) * 0.2 + min
	// 如果ADC值小于等于阈值，则认为是黑点
	threshold = min_val + (uint16_t)((max_val - min_val) * BLACK_POINT_THRESHOLD_PERCENT);
	
	// 黑点：ADC值小（接近最小值）
	// 白点：ADC值大（接近最大值）
	if(adc_value <= threshold)
		return 1;  // 是黑点
	else
		return 0;  // 是白点
}

/**
 * @brief 寻点主函数：遍历所有点找归一化最小值，用左右两点插值提高精度
 * @param adc_values: ADC值数组
 * @param result: 输出结果结构体指针
 * @return 返回找到的黑点精确位置（浮点数），如果未找到则保持上一个精确位置
 */
float BlackPoint_Finder_Search(volatile uint16_t *adc_values, BlackPointResult_t *result)
{
	uint8_t i;
	float normalized_values[SENSOR_COUNT];  // 归一化值数组
	float sum = 0.0f;                       // 归一化值总和
	float average = 0.0f;                   // 归一化值平均值
	float min_normalized = 1.0f;            // 归一化最小值（初始化为最大）
	uint8_t min_index = 0;                  // 最小值对应的传感器索引
	float precise_pos = 0.0f;               // 精确位置
	float all_pos_sum = 0;
	
	if(adc_values == NULL || result == NULL)
	{
		// 参数无效，返回上一个精确位置
		if(result != NULL)
		{
			result->found = 0;
			result->position = last_position;
			result->precise_position = last_precise_position;
		}
		return last_precise_position;
	}
	
	// 第一步：对所有点进行归一化处理
	// 归一化公式：normalized = (adc_value - min_value) / (max_value - min_value)
	// 归一化后，越接近0表示越黑，越接近1表示越白
	for(i = 0; i < SENSOR_COUNT; i++)
	{
		uint16_t min_val = sensor_config[i].min_value;
		uint16_t max_val = sensor_config[i].max_value;
		uint16_t adc_val = adc_values[i];
		
		// 防止除零
		if(max_val > min_val)
		{
			// 归一化：0表示最黑，1表示最白
			normalized_values[i] = (float)(adc_val - min_val) / (float)(max_val - min_val);
			
			// 限制范围在0-1之间
			if(normalized_values[i] < 0.0f)
				normalized_values[i] = 0.0f;
			if(normalized_values[i] > 1.0f)
				normalized_values[i] = 1.0f;
		}
		else
		{
			// 如果max_val <= min_val，设为0（视为最黑）
			normalized_values[i] = 0.0f;
		}
		
		sum += normalized_values[i];
	}
	
	// 第二步：找到归一化后的最小值
	for(i = 0; i < SENSOR_COUNT; i++)
	{
		all_pos_sum += normalized_values[i];
		if(normalized_values[i] < min_normalized)
		{
			min_normalized = normalized_values[i];
			min_index = i;
		}
	}
	
	// 第三步：计算平均值
	average = sum / (float)SENSOR_COUNT;
	
	float other_sum = 0.0f;
	uint8_t other_count = 0;
	
	for(i = 0; i < SENSOR_COUNT; i++)
	{
		// 排除找到的点及其左右两点
		// 排除 min_index 本身
		if(i == min_index)
			continue;
		
		// 排除 min_index - 1（如果存在）
		if(min_index > 0 && i == min_index - 1)
			continue;
		// 排除 min_index + 1（如果存在）
		if(min_index < SENSOR_COUNT - 1 && i == min_index + 1)
			continue;
		// 其他点参与计算
		other_sum += normalized_values[i];
		other_count++;
	}
	
	// 计算剩余点的平均值
	float other_average = 0.0f;
	if(other_count > 0)
	{
		other_average = other_sum / (float)other_count;
	}
	// 第四步：验证最小值是否小于归一化值的20%
	// 条件：min_normalized < 0.2（归一化值的20%）
	if(min_normalized / other_average > 0.30f)
	{
		// 不满足条件，未找到黑点
		result->found = 0;
		result->position = last_position;
		result->precise_position = last_precise_position;
		return last_precise_position;
	}
	
	// 第四步半：验证除找到点及其左右两点外的其他点平均值是否大于60%
	// 排除 min_index-1, min_index, min_index+1 这三个点，计算剩余点的平均值
	
	// 判断：其他点的平均值必须大于60%（0.6）
	if(other_average <= 0.70f)
	{
		// 不满足条件，未找到黑点
		result->found = 0;
		result->position = last_position;
		result->precise_position = last_precise_position;
		return last_precise_position;
	}
	
	// 第五步：使用改进的重心插值算法，保证分布均匀
	// 使用三个点（左、中、右）进行重心计算，权重基于归一化值的倒数
	// 改进权重函数，使结果分布更均匀，避免聚集
	precise_pos = (float)min_index;  // 初始化为最小值位置
	
	// 获取三个点的归一化值
	float left_val = (min_index > 0) ? normalized_values[min_index - 1] : 1.0f;
	float center_val = normalized_values[min_index];
	float right_val = (min_index < SENSOR_COUNT - 1) ? normalized_values[min_index + 1] : 1.0f;
	
	// 计算权重：使用改进的权重函数，避免结果聚集
	// 使用 (1.0 - normalized_value) 的平方作为权重，使权重分布更平滑
	float left_weight = 0.0f;
	float center_weight = 0.0f;
	float right_weight = 0.0f;
	
	if(min_index > 0)
	{
		// 左侧权重：归一化值越小（越黑），权重越大
		float dark_factor = 1.0f - left_val;
		left_weight = dark_factor * dark_factor + 0.01f;  // 平方+小常数，避免为0
	}
	
	// 中心权重：归一化值越小（越黑），权重越大
	float center_dark = 1.0f - center_val;
	center_weight = center_dark * center_dark + 0.01f;
	
	if(min_index < SENSOR_COUNT - 1)
	{
		// 右侧权重：归一化值越小（越黑），权重越大
		float right_dark = 1.0f - right_val;
		right_weight = right_dark * right_dark + 0.01f;
	}
	
	// 计算加权平均位置（重心）
	float total_weight = left_weight + center_weight + right_weight;
	
	if(total_weight > 0.001f)
	{
		float left_pos = (min_index > 0) ? (float)(min_index - 1) : (float)min_index;
		float center_pos = (float)min_index;
		float right_pos = (min_index < SENSOR_COUNT - 1) ? (float)(min_index + 1) : (float)min_index;
		
		precise_pos = (left_pos * left_weight + center_pos * center_weight + right_pos * right_weight) / total_weight;
	}
	
	// 限制精确位置在有效范围内
	if(precise_pos < 0.0f)
		precise_pos = 0.0f;
	if(precise_pos > (float)(SENSOR_COUNT - 1))
		precise_pos = (float)(SENSOR_COUNT - 1);
	
	// 找到黑点，更新结果
	result->found = 1;
	result->position = min_index;
	result->precise_position = precise_pos;
	last_position = min_index;
	last_precise_position = precise_pos;
	
	return precise_pos;
}

/**
 * @brief 获取上一次找到的黑点位置
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

