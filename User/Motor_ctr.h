#ifndef __MOTOR_CTR_H
#define __MOTOR_CTR_H

#include "stm32f10x.h"

/* 电机驱动：两路半桥（单向）
 *   每路只有 PWM + 公共使能，**没有方向脚** —— 电机只能往前走。
 *   占空比 0~10000（MOTOR_DUTY_MAX），0 = 停。
 *   代码里算出的负速度（例如 PID 混控后某侧为负）一律按 0 处理，
 *   见 PID_Controller.c 的 Motor_SetSpeedClamped()。
 */

// 电机相关宏定义
#define MOTOR_FREQUENCY_17KHZ    17000   // PWM频率17KHz
#define MOTOR_DUTY_MAX           10000   // 最大占空比值
#define MOTOR_DUTY_MIN           0       // 最小占空比值

// 电机编号定义
#define MOTOR_R                  1       // 电机1（右）
#define MOTOR_L                  2       // 电机2（左）

// 函数声明
void Motor_Init(void);                           // 电机初始化函数
void Motor_Enable(void);                         // 使能电机
void Motor_Disable(void);                        // 禁用电机
void Motor_SetSpeed(uint8_t motor_id, uint16_t duty);  // 设置电机速度 (0-10000，只有正转)
void Motor_Stop(uint8_t motor_id);               // 停止指定电机
void Motor_StopAll(void);                        // 停止所有电机
void Motor_SetFrequency(uint32_t freq);          // 设置PWM频率

#endif
