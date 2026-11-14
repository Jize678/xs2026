#ifndef __BLACKPOINT_FINDER_H__
#define __BLACKPOINT_FINDER_H__

#include "stm32f10x.h"
#include <stdint.h>
#include <float.h>
#include "stdio.h"

// 传感器数量（可根据实际硬件调整）
#define SENSOR_COUNT 16

/* 寻线阈值（2026-11-13 起改为**固定阈值**，不再逐路标定 min/max）：
 *     adc <  LINE_THRESHOLD_ADC → 白色（线）
 *     adc >= LINE_THRESHOLD_ADC → 蓝色（赛道底色）
 * 赛道是蓝底白线。实测白色读数低于蓝色读数（与旧的白底黑线同向：线 = ADC 小），
 * 所以“找线”仍然是“找读数更低的那一路”，只是判定从“逐路归一化 + 20% 比较”
 * 换成了一条全局阈值，16 组 min/max 标定表随之删除。
 * 传感器顺序：0 = 最左（p0），15 = 最右（p15），位置沿 ADC 数组下标递增。 */
#define LINE_THRESHOLD_ADC     1999
/* 单路阈值覆盖：p3、p6、p15（最右）的装配/贴合与其它路不同，阈值单独标定
 * （2026-11-14 用户实测要求）：
 *     p3  → LINE_THRESHOLD_ADC_P3  = 1500
 *     p6  → LINE_THRESHOLD_ADC_P6  = 1000
 *     p15 → LINE_THRESHOLD_ADC_P15 = 900
 * 其余 13 路仍用 LINE_THRESHOLD_ADC。实际判定用的是 BlackPoint_Finder.c 里的
 * s_line_threshold[16] 表（改这里只改宏，表里的引用会自动跟着变）。 */
#define LINE_THRESHOLD_ADC_P3  1500
#define LINE_THRESHOLD_ADC_P6  1000
#define LINE_THRESHOLD_ADC_P15 900
/* 防抖余量：最白的一路至少要低于阈值这么多，才认为这一拍真的压到线了。
 * 只防“整片底色都在阈值边缘抖动”被误判成有线；正常白线读数远低于它。 */
#define LINE_MIN_DEPTH_ADC     100

/**
 * @brief 寻点结果结构体
 * @param found: 这一拍是否看到线（1=看到，0=没看到，位置沿用上一次）
 * @param position: 线最强的那一路（传感器索引 0~SENSOR_COUNT-1）
 * @param precise_position: 重心插值后的连续位置（0~15，例如 7.5 表示线在 7、8 之间）
 */
typedef struct
{
	uint8_t found;           // 是否看到线
	uint8_t position;        // 线位置（整数路号）
	float precise_position;  // 精确位置（重心插值）
} BlackPointResult_t;

/**
 * @brief 初始化寻线模块（只是把“上一次位置”复位到中间）
 */
void BlackPoint_Finder_Init(void);

/**
 * @brief 寻线主函数：按固定阈值找“最白”的一路，再对连续段做加权重心插值
 * @param adc_values: ADC 值数组（长度至少为 SENSOR_COUNT，0=最左）
 * @param result: 输出结果结构体指针
 * @return 返回找到的精确位置（0~SENSOR_COUNT-1 的浮点数）；
 *         没看到线时保持上一次的精确位置并原样返回
 * @note 算法流程：
 *       1. 每路白度 = 阈值 − ADC（蓝底 / 超过阈值记 0）
 *       2. 找白度最大的一路；它低于阈值不够深（LINE_MIN_DEPTH_ADC）⇒ 没看到线
 *       3. 以它为中心向两侧扩展出连续的“白”段，段外的孤立噪声不参与
 *       4. 段内按白度加权求重心 → 连续位置（精度远好于整格）
 */
float BlackPoint_Finder_Search(volatile uint16_t *adc_values, BlackPointResult_t *result);

/**
 * @brief 判断某一路是否压到线（白线）
 * @param sensor_idx: 传感器索引（0~SENSOR_COUNT-1），0 = 最左
 * @param adc_value: ADC值
 * @return 1=线（白，adc < 阈值），0=底色（蓝）
 * @note 旧名 `BlackPoint_Finder_IsBlackPoint` 是白底黑线时代的叫法；
 *       2026-11-13 改成蓝底白线后语义反了——"点"指的是线本身，而线是白的。
 *       这里一并改名，避免读代码的人把 1 当成"黑"。
 */
uint8_t BlackPoint_Finder_IsLine(uint8_t sensor_idx, uint16_t adc_value);

/**
 * @brief 获取上一次找到的线位置
 * @return 线位置（0~SENSOR_COUNT-1），如果从未找到则返回中间位置
 */
uint8_t BlackPoint_Finder_GetLastPosition(void);

/**
 * @brief 重置上一次位置为中间位置
 */
void BlackPoint_Finder_ResetLastPosition(void);

#endif // __BLACKPOINT_FINDER_H__
