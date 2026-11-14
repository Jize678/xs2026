#include "PID_Controller.h"
#include "Car_Control.h"   /* 循迹也按档选增益：Car_Control_TrackApply() */
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
	
	controller->last_error  = 0.0f;
	controller->last_error2 = 0.0f;
	controller->last_output = 0.0f;
	controller->integral = 0.0f;
	controller->last_target = 0.0f;
	controller->last_feedback = 0.0f;
	controller->last_p_term = 0.0f;
	controller->last_i_term = 0.0f;
}

/**
 * @brief 速度环PID计算（增量式，标准式）
 * 增量式PID公式：Δu(k) = Kp*[e(k)-e(k-1)] + Ki*e(k) + Kd*[e(k)-2*e(k-1)+e(k-2)]
 * 输出：u(k) = u(k-1) + Δu(k)
 *
 * 历史：2026-10-30 之前 P 项与 D 项共用同一个 Δe，kd 等效于"第二个 kp"
 * （见 速度环调参记录.md §5，Δe 增益实为 kp+kd=1.23）。本版把 D 项恢复为
 * 真正的二阶差分 e(k)-2e(k-1)+e(k-2)，默认增益随之按"等效保真"重标：
 * kp 0.60→1.23、kd 0.63→0，Δe 总增益仍为 1.23，车的手感不变。
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
	// 积分冻结（2026-11-04）：误差超出 ±PID_I_FREEZE_MM_S 时只按 PID_I_TAIL_SCALE
	// 降权积分，落在窗口内才全速积分。阶跃段不再用 ki·e 连乘去搭输出，改由前馈
	// 把占空比一次挪到位，调节时间和超调才有可能分开达标。
	float i_gain = controller->param.ki;
	if(fabsf(error) > PID_I_FREEZE_MM_S)
	{
		i_gain *= PID_I_TAIL_SCALE;
	}
	float i_term = i_gain * error;
	
	// 微分项：Kd * [e(k) - 2*e(k-1) + e(k-2)]
	float d_term = controller->param.kd * (error - 2.0f * controller->last_error + controller->last_error2);
	 
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

	// 保存当前误差和输出（last_error2 必须比 last_error 晚一拍更新）
	controller->last_error2 = controller->last_error;
	controller->last_error  = error;
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
	
	controller->last_error  = 0.0f;
	controller->last_error2 = 0.0f;
	controller->last_output = 0.0f;
	controller->integral = 0.0f;
	controller->last_target = 0.0f;
	controller->last_feedback = 0.0f;
	controller->last_p_term = 0.0f;
	controller->last_i_term = 0.0f;
}

/**
 * @brief 前馈跳变：把 last_output（增量式的积分器状态）一次性挪 duty_delta
 * @note  只改输出状态，不动 last_error/last_error2，所以下一拍的 P/D 项仍然
 *        按“上一拍误差”算，跳变不会被误当成一个大误差而再叠一次。
 */
void SpeedPID_Bump(SpeedPID_Controller_t *controller, float duty_delta)
{
	float out;

	if(controller == NULL) return;

	out = controller->last_output + duty_delta;
	if(out > controller->param.output_max)
	{
		out = controller->param.output_max;
	}
	else if(out < controller->param.output_min)
	{
		out = controller->param.output_min;
	}
	controller->last_output = out;
}

// ==================== 位置环PID实现 ====================

/**
 * @brief 初始化位置环PID控制器
 */
void PositionPID_Init(PositionPID_Controller_t *controller, float kp, float ki, float kd, float yaw_kd,
                      float output_max, float output_min, float target_position)
{
	if(controller == NULL) return;
	
	controller->param.kp = kp;
	controller->param.ki = ki;
	controller->param.kd = kd;
	controller->param.yaw_kd = yaw_kd;
	controller->param.output_max = output_max;
	controller->param.output_min = output_min;
	controller->param.integral_max = output_max * 0.5f;
	controller->param.integral_min = output_min * 0.5f;
	controller->param.target_position = target_position;
	
	controller->last_error = 0.0f;
	controller->integral = 0.0f;
}

/*------------------------------------------------------------------------------
 * 航向角估计（2026-11-14：陀螺没装，用两路编码器差速积分出航向角）
 *
 *   ω = (v_R − v_L) / L          [rad/s]，左转为正（与陀螺 Z 轴同符号）
 *   θ = ∫ ω dt                   [rad]，进循迹/复位时清零
 *
 *   微分驱动不打滑时 θ 严格等于整车转过的角度：θ = (d_R − d_L) / L。
 *   所以它量的是"车自己转了多少度"，正是原来陀螺项要的那个量；
 *   差别在于它不再独立于转向指令（详见 README §10.7）。
 *----------------------------------------------------------------------------*/
#define WHEEL_BASE_MM        130.0f   /* 两驱动轮中心距（实测） */
#define PID_YAW_KD           100.0f   /* 航向角项增益：舵量/rad，0 = 关闭该项。
                                       * 含义：满量程航向 1 rad（57°）只给 100 舵量（半格）。
                                       * 2026-11-14 上车试调由 500 降到 100（500 时"顺转向加舵"
                                       * 太强，弯道上越跑越偏），文档已同步。 */
#define PID_YAW_LPF          0.35f    /* 角速度一阶低通（2 ms 一拍 ⇒ ≈6 ms） */
#define PID_YAW_HEAD_MAX     1.0f     /* 航向角积分上限 ±1 rad（≈57°），防弯道累积跑飞 */
#define PID_YAW_TERM_MAX     7000.0f  /* 与旧 gyro_term 的限幅保持一致 */
#define PID_TICK_S           0.002f   /* 控制周期 2 ms，用于积分 */

static float s_yaw_rate = 0.0f;       /* 滤波后的角速度估计 rad/s */
static float s_heading  = 0.0f;       /* 航向角积分 rad（左转为正） */

float PID_EstimateYawRate(void)
{
	float raw = (speed_right_mm_s - speed_left_mm_s) / WHEEL_BASE_MM;

	s_yaw_rate += PID_YAW_LPF * (raw - s_yaw_rate);

	/* 积分成航向角；限幅防止弯道上一路累加 */
	s_heading += s_yaw_rate * PID_TICK_S;
	if(s_heading > PID_YAW_HEAD_MAX)
		s_heading = PID_YAW_HEAD_MAX;
	else if(s_heading < -PID_YAW_HEAD_MAX)
		s_heading = -PID_YAW_HEAD_MAX;

	return s_yaw_rate;
}

float PID_GetHeading(void)
{
	return s_heading;
}

void PID_ResetYawEst(void)
{
	s_yaw_rate = 0.0f;
	s_heading  = 0.0f;
}

/**
 * @brief 位置环PID计算（位置式）
 * 位置式PID公式：u(k) = Kp*e(k) + Ki*Σe(k) + Kd*[e(k)-e(k-1)]
 * 误差定义：error = current_position - target_position
 * 输出：位置偏右时为正（左目标变大、右目标变小，即左快右慢 = 右转），位置偏左时为负
 */
float PositionPID_Calculate(PositionPID_Controller_t *controller, float current_position)
{
	if(controller == NULL) return 0.0f;
	
	// 误差 = 当前位置 - 目标位置
	// 位置偏右（current_position > target_position）时，error为正，输出为正 → 左目标变大、右目标变小
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
	/* 航向角前馈（结构完全沿用原来的陀螺项：output = p + i + d − gyro_term）。
	 * gyro_term 现在 = yaw_kd · θ，θ 是两路编码器差速积分出来的航向角（rad，左转为正）。
	 * 注意 θ 是"进循迹以来累计转过的角度"，弯道上会一直往同一边累积，
	 * 所以增益只能给小的，积分本身另有 ±PID_YAW_HEAD_MAX 的限幅。 */
	(void)PID_EstimateYawRate();   /* 每 2 ms 更新一次 ω 低通与 θ 积分 */
	float gyro_term = controller->param.yaw_kd * PID_GetHeading();
	if(gyro_term >= PID_YAW_TERM_MAX)
	{
		gyro_term = PID_YAW_TERM_MAX;
	}
	else if(gyro_term <= -PID_YAW_TERM_MAX)
	{
		gyro_term = -PID_YAW_TERM_MAX;
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
 *   线位置 position_get/10  (0~15，中心 7.5 = 16 路几何中点)
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
 * base = 1100 − min(|pos−7.5|,3)/3 × 300   （800~1100 mm/s，偏差越大越慢）
 *
 * 单位：速度环的目标和反馈都是 **mm/s**（由 ABEncoder 按 54476 counts/m 标定换算好）。
 *       基准速度的来历：2110/1190（-2026-11-13）→ 700/400（2026-11-13 场地变小先求稳）
 *       → **1100/800（2026-11-14 用户上车试调）**。三次都按满 3 格线性铺开，每格恒为
 *       100 mm/s，所以"灰度窜一格 = 差速跳 100"这个前提一直成立。
 *       方向环常数沿用旧的 duty 量纲，未重标。
 *============================================================================*/

#define PID_POS_CENTER     7.5f     /* 线位置中心：16 路 p0..p15，几何中点是 7.5 */
#define PID_BASE_FAST      1100.0f   /* 居中时的基准速度（mm/s）：2110 →(11-13) 700 →(11-14 试调) 1100 */
#define PID_BASE_SLOW      800.0f   /* 偏差 >= 3 格时的基准速度（mm/s）：1190 →(11-13) 400 →(11-14) 800，每格仍 100 */
#define PID_STEER_FULL     9000.0f  /* 方向环输出上限，对应 ±基准速度 的差速 */
#define PID_RAMP_STEP      4.6f     /* 起步斜坡：每 2ms 加 4.6 mm/s，0→1100 约 0.48s（0→700 时代 0.30s） */

SpeedPID_Controller_t g_speed_pid_l;    /* 内环：左轮 */
SpeedPID_Controller_t g_speed_pid_r;    /* 内环：右轮 */
PositionPID_Controller_t g_position_pid;/* 外环：线位置 */

void PID_Init(void)
{
	/* 内环 ×2：左右各自一个增量式速度环；两侧参数目前相同，但状态各自独立。
	 * 增益单位 = duty / (mm/s)。2026-10-30 起 D 项改为标准二阶差分，增益按"等效保真"重标：
	 * kp 0.60→1.23、kd 0.63→0，Δe 总增益仍为 1.23（等效旧写法 5.5/5.1/5.8 duty/(count·2ms⁻¹)，
	 * 换算系数 1 count/2ms ≈ 9.1784 mm/s）；需要真微分时用 tune 模式的 kdl/kdr 在线加。
	 *
	 * 2026-10-30 21:00 车测定版：左右各跑 1000 mm/s 阶跃（单侧、无负压），三项指标全过
	 *   左轮 稳态 999.9 / 误差 -0.01% / t90 200 ms / t90 后偏离 4.1%
	 *   右轮 稳态 999.0 / 误差 -0.10% / t90 100 ms / t90 后偏离 2.97%
	 * 定版值 0.200 / 0.080 / 0.040 —— 直接来自 速度环调参说明.md §9 流程，轨迹见 PID调参轨迹.txt。
	 * 旧版 1.23/0.56/0.00 在同一工况下误差 +12~16%、t90 后偏离 48~60%，已淘汰。*/
	SpeedPID_Init(&g_speed_pid_l, 0.200f, 0.080f, 0.040f, 8000.0f, -8000.0f);
	SpeedPID_Init(&g_speed_pid_r, 0.200f, 0.080f, 0.040f, 8000.0f, -8000.0f);

	/* 外环：线位置 → 转向量（输出 ±9000，换算见文件顶部的框图）。
	 * 目标位置 = 16 路的中点 7.5（p0 最左、p15 最右），不是 8.0。 */
	//																	p				i				d			前馈系数
	PositionPID_Init(&g_position_pid, 198.0f, 0.0f, 2280.0f, PID_YAW_KD, 9000.0f, -9000.0f, PID_POS_CENTER);
}

extern uint8_t star_car;

/* 起步斜坡的状态：放在文件作用域，好让 PID_ResetRamp() 能清掉。
 * 原来是函数内 static，只在"上电后第一次发车"有效 —— 之后每次进 TRACK
 * 都因为 first_set 已经是 1 而直接跳到位，等于没有软起动。 */
static uint8_t s_ramp_done = 0;    /* 起步斜坡是否跑完 */
static float   s_ramp_base = 0.0f; /* 斜坡里的基准速度 */

void PID_ResetRamp(void)
{
	s_ramp_done = 0u;
	s_ramp_base = 0.0f;
	PID_ResetYawEst();   /* 进/出循迹时角速度估计也要从 0 起 */
}

void PID_Control_Update(void)
{

	float current_position;
	float base_target;
	float base_speed;
	float steer;
	float steer_speed;
	float target_left, target_right;
	float duty_left, duty_right;

	/* 1. 当前位置：0~15，中心 PID_POS_CENTER(7.5) */
	current_position = (float)position_get / 10.0f;

	/* 2. 外环：位置 → 转向量（±9000） */
	steer = PositionPID_Calculate(&g_position_pid, current_position);

	/* 3. 基准速度：偏差越大越慢（7.5 = 16 路中点） */
	base_target = PID_BASE_FAST
	            - (fmin(fabs(current_position - PID_POS_CENTER), 3.0f) / 3.0f)
	              * (PID_BASE_FAST - PID_BASE_SLOW);

	if(star_car)
	{
		/* 起步斜坡：基准速度从 0 爬到 base_target，避免一上来就满舵 */
		if((s_ramp_done == 0u) && (s_ramp_base < base_target))
		{
			s_ramp_base += PID_RAMP_STEP;
		}
		else
		{
			s_ramp_base = base_target;
			s_ramp_done = 1u;
		}
		base_speed = s_ramp_base;

		/* 4. 转向量换算成"速度差"：满舵(±9000) = ±基准速度 */
		steer_speed = steer * (base_speed / PID_STEER_FULL);

		/* 5. 串级：外环把目标速度分别发给左右内环 */
		target_left  = base_speed + steer_speed;
		target_right = base_speed - steer_speed;
		if(target_left  < 0.0f) { target_left  = 0.0f; }
		if(target_right < 0.0f) { target_right = 0.0f; }

		/* 6. 分段增益：目标一变就按新目标选档（v4 定版，实现在 Car_Control.c） */
		Car_Control_TrackApply(0u, target_left,  &g_speed_pid_l);
		Car_Control_TrackApply(1u, target_right, &g_speed_pid_r);

		/* 7. 内环：各用各的编码器反馈，各算各的占空比 */
		duty_left  = SpeedPID_Calculate(&g_speed_pid_l, target_left,  speed_left_mm_s);
		duty_right = SpeedPID_Calculate(&g_speed_pid_r, target_right, speed_right_mm_s);
	}
	else
	{
		/* 没发车：清干净，下次发车从 0 重新爬坡 */
		PID_ResetRamp();
		Car_Control_TrackReset();
		SpeedPID_Reset(&g_speed_pid_l);
		SpeedPID_Reset(&g_speed_pid_r);
		duty_left  = 0.0f;
		duty_right = 0.0f;
	}

	/* 8. 下发到电机 */
	Motor_SetSpeedClamped(MOTOR_L, duty_left);
	Motor_SetSpeedClamped(MOTOR_R, duty_right);
}
