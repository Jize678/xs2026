#include "PID_Controller.h"
#include "LSM6DSR_Config.h"
#include <math.h>
 extern int16_t position_get;
// ==================== 速度环PID实现 ====================

/**
 * @brief 初始化速度环PID控制器
 */
void SpeedPID_Init(SpeedPID_Controller_t *controller, float kp, float ki, float kd, 
                   float output_max, float output_min)
{
	if(controller == NULL) return;
	
	controller->param.kp = kp;
	controller->param.ki = ki;
	controller->param.kd = kd;
	controller->param.output_max = output_max;
	controller->param.output_min = output_min;
	controller->param.integral_max = output_max * 0.5f;  // 积分限幅设为输出上限的一半
	controller->param.integral_min = output_min * 0.5f;
	
	controller->last_error = 0.0f;
	controller->last_output = 0.0f;
	controller->integral = 0.0f;
	controller->last_target = 0.0f;
	controller->last_feedback = 0.0f;
	controller->last_p_term = 0.0f;
	controller->last_i_term = 0.0f;
}

/**
 * @brief 速度环PID计算（增量式）
 * 增量式PID公式：Δu(k) = Kp*[e(k)-e(k-1)] + Ki*e(k) + Kd*[e(k)-2*e(k-1)+e(k-2)]
 * 简化版：Δu(k) = Kp*[e(k)-e(k-1)] + Ki*e(k) + Kd*[e(k)-e(k-1)]
 * 输出：u(k) = u(k-1) + Δu(k)
 */
float SpeedPID_Calculate(SpeedPID_Controller_t *controller, float target_speed, float current_speed)
{
	if(controller == NULL) return 0.0f;
	
	float error = target_speed - current_speed;
	float delta_output = 0.0f;
	float new_output = 0.0f;
	
	// 比例项：Kp * (e(k) - e(k-1))
	float p_term = controller->param.kp * (error - controller->last_error);
	
	// 积分项：Ki * e(k)
	float i_term = controller->param.ki * error;
	
	// 微分项：Kd * (e(k) - e(k-1))
	float d_term = controller->param.kd * (error - controller->last_error);
	 
	// 计算增量输出
	delta_output = p_term + i_term + d_term;
	
	// 新的输出 = 上一次输出 + 增量
	new_output = controller->last_output + delta_output;
	
	// 输出限幅
	if(new_output > controller->param.output_max)
		new_output = controller->param.output_max;
	else if(new_output < controller->param.output_min)
		new_output = controller->param.output_min;
	
	// 调试快照（串口 testl/testr 打印用，不影响控制）
	controller->last_target   = target_speed;
	controller->last_feedback = current_speed;
	controller->last_p_term   = p_term;
	controller->last_i_term   = i_term;

	// 保存当前误差和输出
	controller->last_error = error;
	controller->last_output = new_output;
	
	return new_output;
}

/**
 * @brief 设置速度环PID参数
 */
void SpeedPID_SetParam(SpeedPID_Controller_t *controller, float kp, float ki, float kd)
{
	if(controller == NULL) return;
	
	controller->param.kp = kp;
	controller->param.ki = ki;
	controller->param.kd = kd;
}

/**
 * @brief 重置速度环PID控制器
 */
void SpeedPID_Reset(SpeedPID_Controller_t *controller)
{
	if(controller == NULL) return;
	
	controller->last_error = 0.0f;
	controller->last_output = 0.0f;
	controller->integral = 0.0f;
	controller->last_target = 0.0f;
	controller->last_feedback = 0.0f;
	controller->last_p_term = 0.0f;
	controller->last_i_term = 0.0f;
}

// ==================== 位置环PID实现 ====================

/**
 * @brief 初始化位置环PID控制器
 */
void PositionPID_Init(PositionPID_Controller_t *controller, float kp, float ki, float kd, float gyro_kd,
                      float output_max, float output_min, float target_position)
{
	if(controller == NULL) return;
	
	controller->param.kp = kp;
	controller->param.ki = ki;
	controller->param.kd = kd;
	controller->param.gyro_kd = gyro_kd;
	controller->param.output_max = output_max;
	controller->param.output_min = output_min;
	controller->param.integral_max = output_max * 0.5f;
	controller->param.integral_min = output_min * 0.5f;
	controller->param.target_position = target_position;
	
	controller->last_error = 0.0f;
	controller->integral = 0.0f;
}

/**
 * @brief 位置环PID计算（位置式）
 * 位置式PID公式：u(k) = Kp*e(k) + Ki*Σe(k) + Kd*[e(k)-e(k-1)]
 * 误差定义：error = current_position - target_position
 * 输出：位置偏右时为正（左轮减速、右轮加速），位置偏左时为负（左轮加速、右轮减速）
 */
float PositionPID_Calculate(PositionPID_Controller_t *controller, float current_position)
{
	if(controller == NULL) return 0.0f;
	
	// 误差 = 当前位置 - 目标位置
	// 位置偏右（current_position > target_position）时，error为正，输出为正，左轮减速、右轮加速
	float error = current_position - controller->param.target_position;
	float output = 0.0f;
	
	// 比例项：Kp * e(k)
	float p_term = controller->param.kp * error;
	
	// 积分项：Ki * Σe(k)
	controller->integral += error;
	
	// 积分限幅（防止积分饱和）
	if(controller->integral > controller->param.integral_max)
		controller->integral = controller->param.integral_max;
	else if(controller->integral < controller->param.integral_min)
		controller->integral = controller->param.integral_min;
	
	float i_term = controller->param.ki * controller->integral;
	
	// 微分项：Kd * [e(k) - e(k-1)]
	float d_term = controller->param.kd * (error - controller->last_error);
	float gyro_term = controller->param.gyro_kd * LSE6DSR_data.gz_rads;
	if(gyro_term >= 7000)
	{
		gyro_term = 7000;
	}
	else if(gyro_term <= -7000)
	{
		gyro_term = -7000;
	}
	// 输出偏差值（直接叠加到速度环输出）
	output = p_term + i_term + d_term - gyro_term;
	
	// 输出限幅
	if(output > controller->param.output_max)
		output = controller->param.output_max;
	else if(output < controller->param.output_min)
		output = controller->param.output_min;
	
	// 保存当前误差
	controller->last_error = error;
	
	return output;
}

/**
 * @brief 设置位置环PID参数
 */
void PositionPID_SetParam(PositionPID_Controller_t *controller, float kp, float ki, float kd)
{
	if(controller == NULL) return;
	
	controller->param.kp = kp;
	controller->param.ki = ki;
	controller->param.kd = kd;
}

/**
 * @brief 设置位置环目标位置
 */
void PositionPID_SetTarget(PositionPID_Controller_t *controller, float target_position)
{
	if(controller == NULL) return;
	
	controller->param.target_position = target_position;
}

/**
 * @brief 重置位置环PID控制器
 */
void PositionPID_Reset(PositionPID_Controller_t *controller)
{
	if(controller == NULL) return;
	
	controller->last_error = 0.0f;
	controller->integral = 0.0f;
}

// ==================== 电机控制辅助函数 ====================

/**
 * @brief 把浮点速度写进电机（两路半桥只能前进）
 * @note  速度 <= 0 时直接给 0 占空比（不再有倒车）；> MOTOR_DUTY_MAX 时限幅
 */
void Motor_SetSpeedClamped(uint8_t motor_id, float speed_target)
{
	float v = speed_target;

	if(v <= 0.0f)
	{
		Motor_SetSpeed(motor_id, 0);
		return;
	}

	if(v > (float)MOTOR_DUTY_MAX)
	{
		v = (float)MOTOR_DUTY_MAX;
	}

	Motor_SetSpeed(motor_id, (uint16_t)v);
}

/*==============================================================================
 * 闭环架构（串级）：位置环（外环）→ 速度环（内环，左右各一个）→ 电机
 *
 *   线位置 position_get/10  (0~15，中心 8.0)
 *        │
 *        ▼  PositionPID_Calculate()            常数未改：Kp 198 / Ki 0 / Kd 2280
 *   转向量 steer  (±9000，本质是 duty 量纲的"舵量")
 *        │  steer_speed = steer × base / 9000    ← 满舵 = ±基准速度
 *        ▼
 *   左目标 = base + steer_speed
 *   右目标 = base − steer_speed        （两者都限到 ≥ 0，半桥不能倒转）
 *        │
 *        ▼  SpeedPID_Calculate(&g_speed_pid_l, 左目标, speed_left_mm_s)
 *           SpeedPID_Calculate(&g_speed_pid_r, 右目标, speed_right_mm_s)
 *   duty_L / duty_R (0~8000) → Motor_SetSpeedClamped()
 *
 * base = 2110 − min(|pos−8|,3)/3 × 920   （1190~2110 mm/s，偏差越大越慢）
 *
 * 单位：速度环的目标和反馈都是 **mm/s**（由 ABEncoder 按 54476 counts/m 标定换算好），
 *       直道基准 2110 mm/s ≈ 2.1 m/s（旧写法 230 counts/2ms）。方向环常数沿用旧的 duty 量纲，
 *============================================================================*/

#define PID_BASE_FAST      2110.0f  /* 居中时的基准速度（mm/s）≈ 2.11 m/s，旧写法 230 counts/2ms */
#define PID_BASE_SLOW      1190.0f  /* 偏差 >= 3 格时的基准速度（mm/s），旧写法 130 counts/2ms */
#define PID_STEER_FULL     9000.0f  /* 方向环输出上限，对应 ±基准速度 的差速 */
#define PID_RAMP_STEP      4.6f     /* 起步斜坡：每 2ms 加 4.6 mm/s（≈ 旧 0.5 counts/2ms），0→2110 约 0.92s */

SpeedPID_Controller_t g_speed_pid_l;    /* 内环：左轮 */
SpeedPID_Controller_t g_speed_pid_r;    /* 内环：右轮 */
PositionPID_Controller_t g_position_pid;/* 外环：线位置 */

void PID_Init(void)
{
	/* 内环 ×2：左右各自一个增量式速度环；参数暂时相同，状态各自独立。
	 * 增益单位 = duty / (mm/s)：0.60/0.56/0.63 等效于旧写法 5.5/5.1/5.8 duty/(count·2ms⁻¹)，
	 * 换算系数 1 count/2ms ≈ 9.1784 mm/s。*/
	SpeedPID_Init(&g_speed_pid_l, 0.60f, 0.56f, 0.63f, 8000.0f, -8000.0f);
	SpeedPID_Init(&g_speed_pid_r, 0.60f, 0.56f, 0.63f, 8000.0f, -8000.0f);

	/* 外环：线位置 → 转向量（输出 ±9000，换算见文件顶部的框图） */
	PositionPID_Init(&g_position_pid, 198.0f, 0.0f, 2280.0f, 2800.0f, 9000.0f, -9000.0f, 8.0f);
}

extern uint8_t star_car;

void PID_Control_Update(void)
{
	static uint8_t first_set = 0;    /* 起步斜坡是否跑完 */
	static float   ramp_base = 0;    /* 斜坡里的基准速度 */

	float current_position;
	float base_target;
	float base_speed;
	float steer;
	float steer_speed;
	float target_left, target_right;
	float duty_left, duty_right;

	/* 1. 当前位置：0~15，中心 8.0 */
	current_position = (float)position_get / 10.0f;

	/* 2. 外环：位置 → 转向量（±9000） */
	steer = PositionPID_Calculate(&g_position_pid, current_position);

	/* 3. 基准速度：偏差越大越慢 */
	base_target = PID_BASE_FAST
	            - (fmin(fabs(current_position - 8.0f), 3.0f) / 3.0f)
	              * (PID_BASE_FAST - PID_BASE_SLOW);

	if(star_car)
	{
		/* 起步斜坡：基准速度从 0 爬到 base_target，避免一上来就满舵 */
		if((first_set == 0u) && (ramp_base < base_target))
		{
			ramp_base += PID_RAMP_STEP;
		}
		else
		{
			ramp_base = base_target;
			first_set = 1u;
		}
		base_speed = ramp_base;

		/* 4. 转向量换算成"速度差"：满舵(±9000) = ±基准速度 */
		steer_speed = steer * (base_speed / PID_STEER_FULL);

		/* 5. 串级：外环把目标速度分别发给左右内环 */
		target_left  = base_speed + steer_speed;
		target_right = base_speed - steer_speed;
		if(target_left  < 0.0f) { target_left  = 0.0f; }
		if(target_right < 0.0f) { target_right = 0.0f; }

		/* 6. 内环：各用各的编码器反馈，各算各的占空比 */
		duty_left  = SpeedPID_Calculate(&g_speed_pid_l, target_left,  speed_left_mm_s);
		duty_right = SpeedPID_Calculate(&g_speed_pid_r, target_right, speed_right_mm_s);
	}
	else
	{
		/* 没发车：清干净，下次发车从 0 重新爬坡 */
		ramp_base  = 0.0f;
		first_set  = 0u;
		SpeedPID_Reset(&g_speed_pid_l);
		SpeedPID_Reset(&g_speed_pid_r);
		duty_left  = 0.0f;
		duty_right = 0.0f;
	}

	/* 7. 下发到电机 */
	Motor_SetSpeedClamped(MOTOR_L, duty_left);
	Motor_SetSpeedClamped(MOTOR_R, duty_right);
}
