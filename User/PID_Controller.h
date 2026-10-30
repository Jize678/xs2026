#ifndef __PID_CONTROLLER_H__
#define __PID_CONTROLLER_H__

#include "stm32f10x.h"
#include "Motor_ctr.h"
#include "ABEncoder.h"
#include "BlackPoint_Finder.h"

/**
 * @brief PID 控制器使用说明（串级：位置环 → 速度环）
 *
 * 控制架构：
 * 1. 位置环（外环 / 方向环）：输入线位置（0~15，中心 8.0），
 *    输出「转向量」steer（±9000，沿用旧的 duty 量纲）。
 * 2. 速度环（内环）：**左右轮各一个**，输入目标速度、反馈各自的编码器速度
 *    （单位：每 2ms 的计数增量），输出各自的占空比。
 *
 * 数据流（每 2ms 一拍，PID_Control_Update()）：
 *    base  = 230 − min(|pos−8|,3)/3 × 100            // 偏差越大越慢，130~230
 *    steer = PositionPID_Calculate(pos)               // ±9000
 *    diff  = steer × base / 9000                      // 满舵 = ±base
 *    左目标 = base + diff  (≥0)
 *    右目标 = base − diff  (≥0)
 *    duty_L = SpeedPID_Calculate(&g_speed_pid_l, 左目标, speed_left)
 *    duty_R = SpeedPID_Calculate(&g_speed_pid_r, 右目标, speed_right)
 *
 * 极性（与旧版一致，未改）：
 *    error = pos − 8 > 0（线在偏 8 这一侧）→ steer > 0 → 左目标变大、右目标变小
 *    实际往哪边修正取决于 Y0~Y15 的安装方向，现场若发现越修越偏，把
 *    PID_Controller.c 里左右目标的加减号对调即可。
 */

// ==================== 速度环PID参数 ====================
// 增量式PID参数（用于电机速度控制）
typedef struct
{
	float kp;           // 比例系数
	float ki;           // 积分系数
	float kd;           // 微分系数
	float output_max;   // 输出上限（占空比，0-10000）
	float output_min;   // 输出下限（占空比；可为负，但下发到电机时负值按 0 处理）
	float integral_max; // 积分限幅（防止积分饱和）
	float integral_min; // 积分下限
} SpeedPID_Param_t;

// 速度环PID控制器结构体
typedef struct
{
	SpeedPID_Param_t param;  // PID参数
	float last_error;        // 上一次误差
	float last_output;       // 上一次输出
	float integral;          // 积分项累积

	/* 调试快照：最近一次 SpeedPID_Calculate() 的输入与各项增量（只读，
	 * 给串口 testl/testr 打印用，不参与控制） */
	float last_target;       // 最近一次的目标速度
	float last_feedback;     // 最近一次的反馈速度
	float last_p_term;       // 最近一次的 P 增量
	float last_i_term;       // 最近一次的 I 增量
} SpeedPID_Controller_t;

// ==================== 位置环PID参数 ====================
// 位置式PID参数（用于寻点位置控制）
typedef struct
{
	float kp;           // 比例系数
	float ki;           // 积分系数
	float kd;           // 微分系数
	float gyro_kd;
	float output_max;   // 输出上限（偏差值，叠加到速度环输出）
	float output_min;   // 输出下限（偏差值，叠加到速度环输出）
	float integral_max; // 积分限幅
	float integral_min; // 积分下限
	float target_position; // 目标位置（寻点中心位置，通常为SENSOR_COUNT/2）
} PositionPID_Param_t;

// 位置环PID控制器结构体
typedef struct
{
	PositionPID_Param_t param;  // PID参数
	float last_error;            // 上一次误差
	float integral;              // 积分项累积
} PositionPID_Controller_t;

// ==================== 速度环PID函数 ====================
/**
 * @brief 初始化速度环PID控制器
 * @param controller: 速度环PID控制器指针
 * @param kp: 比例系数
 * @param ki: 积分系数
 * @param kd: 微分系数
 * @param output_max: 输出上限（占空比，0-10000）
 * @param output_min: 输出下限（占空比，-10000到0）
 */
void SpeedPID_Init(SpeedPID_Controller_t *controller, float kp, float ki, float kd, 
                   float output_max, float output_min);

/**
 * @brief 速度环PID计算（增量式）
 * @param controller: 速度环PID控制器指针
 * @param target_speed: 目标速度（编码器值）
 * @param current_speed: 当前速度（编码器值）
 * @return 输出值（占空比，正值为前进；负值在 Motor_SetSpeedClamped() 里按 0 处理）
 */
float SpeedPID_Calculate(SpeedPID_Controller_t *controller, float target_speed, float current_speed);

/**
 * @brief 设置速度环PID参数
 * @param controller: 速度环PID控制器指针
 * @param kp: 比例系数
 * @param ki: 积分系数
 * @param kd: 微分系数
 */
void SpeedPID_SetParam(SpeedPID_Controller_t *controller, float kp, float ki, float kd);

/**
 * @brief 重置速度环PID控制器（清零积分项和历史值）
 * @param controller: 速度环PID控制器指针
 */
void SpeedPID_Reset(SpeedPID_Controller_t *controller);

// ==================== 位置环PID函数 ====================
/**
 * @brief 初始化位置环PID控制器
 * @param controller: 位置环PID控制器指针
 * @param kp: 比例系数
 * @param ki: 积分系数
 * @param kd: 微分系数
 * @param output_max: 输出上限（偏差值，叠加到速度环输出）
 * @param output_min: 输出下限（偏差值，叠加到速度环输出）
 * @param target_position: 目标位置（寻点中心位置，通常为SENSOR_COUNT/2）
 */
void PositionPID_Init(PositionPID_Controller_t *controller, float kp, float ki, float kd, float gyro_kd,
                      float output_max, float output_min, float target_position);

/**
 * @brief 位置环PID计算（位置式）
 * @param controller: 位置环PID控制器指针
 * @param current_position: 当前位置（寻点位置，左边为0）
 * @return 输出偏差值（直接叠加到速度环输出上，左轮减、右轮加）
 * @note 位置偏右时输出为正，左轮减速、右轮加速；位置偏左时输出为负，左轮加速、右轮减速
 */
float PositionPID_Calculate(PositionPID_Controller_t *controller, float current_position);

/**
 * @brief 设置位置环PID参数
 * @param controller: 位置环PID控制器指针
 * @param kp: 比例系数
 * @param ki: 积分系数
 * @param kd: 微分系数
 */
void PositionPID_SetParam(PositionPID_Controller_t *controller, float kp, float ki, float kd);

/**
 * @brief 设置位置环目标位置
 * @param controller: 位置环PID控制器指针
 * @param target_position: 目标位置（寻点中心位置，通常为SENSOR_COUNT/2）
 */
void PositionPID_SetTarget(PositionPID_Controller_t *controller, float target_position);

/**
 * @brief 重置位置环PID控制器（清零积分项和历史值）
 * @param controller: 位置环PID控制器指针
 */
void PositionPID_Reset(PositionPID_Controller_t *controller);

// ==================== 电机控制辅助函数 ====================
/**
 * @brief 把浮点速度写进电机（两路半桥只能前进，负值按 0 处理）
 * @param motor_id: 电机编号（MOTOR_L 或 MOTOR_R）
 * @param speed_target: 目标速度（0~10000）；< 0 一律当 0
 * @note  没有方向脚了，这里只做限幅 + 取整
 */
void Motor_SetSpeedClamped(uint8_t motor_id, float speed_target);
void PID_Init(void);
void PID_Control_Update(void);

/* 两个环的实例（Car_Control.c 切换模式时需要复位它们） */
extern SpeedPID_Controller_t   g_speed_pid_l;   /* 内环：左轮 */
extern SpeedPID_Controller_t   g_speed_pid_r;   /* 内环：右轮 */
extern PositionPID_Controller_t g_position_pid;
#endif // __PID_CONTROLLER_H__
