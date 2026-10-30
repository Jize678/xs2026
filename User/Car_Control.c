#include "Car_Control.h"
#include "Board_Config.h"
#include "Motor_ctr.h"
#include "M3PWM.h"
#include "Led_Status.h"
#include "PID_Controller.h"
#include "ABEncoder.h"
#include <stdio.h>

/* 闭环运行标志（PID_Controller.c 用它判断是否发车） */
extern uint8_t star_car;

static Car_Mode_t s_mode = CAR_MODE_IDLE;
static uint16_t   s_test_ticks = 0;     /* 电机测试计时（单位：CAR_CTRL_TICK_MS） */
static uint8_t    s_test_phase = 0;     /* 电机测试阶段：0=向前，1=停车 */
static uint16_t   s_wheel_ticks = 0;    /* 单轮测试剩余 tick，0 = 未测试 */
static uint16_t   s_fan_ticks = 0;      /* 风扇测试剩余 tick，0 = 未测试 */
static uint16_t   s_fan_arm_ticks = 0;  /* 风扇测试里“压最低油门等解锁”的剩余 tick */
static uint16_t   s_fan_duty = 0;       /* 风扇测试目标油门 0~1000 */
static uint16_t   s_fdelay_ticks = 0;   /* 'ff' 延时风扇：先空等的剩余 tick */
static uint16_t   s_frun_ticks   = 0;   /* 'ff' 延时风扇：吹风的剩余 tick */
static uint16_t   s_fdelay_duty  = 0;   /* 'ff' 延时风扇目标油门 0~1000 */

/* 速度环标定（testl / testr）：单轮按目标速度闭环，另一侧停 */
static Car_Wheel_t s_spd_wheel  = CAR_WHEEL_LEFT;
static int16_t     s_spd_target = 0;
static uint16_t    s_spd_ticks  = 0;

/* 速度环调参（tune）：左右目标独立，s_tune_run=1 表示已发 ! 开始闭环 */
static int16_t     s_tune_tgt_l = 0;
static int16_t     s_tune_tgt_r = 0;
static uint8_t     s_tune_run   = 0;
static uint16_t    s_tune_ms     = 0;    /* 本次 ! 的定时（ms），0 = 未开始 */
static uint8_t     s_tune_side   = 2;    /* 本次 ! 的侧别（Car_TuneSide_t） */
static uint32_t    s_tune_left_ms  = 0;  /* 本次 ! 剩余时长（ms），2ms 控制拍里递减 */

/**
  * @brief  单轮测试：只让一侧轮子转，CAR_WHEEL_TEST_MS 毫秒后自动回待机
  * @param  wheel: CAR_WHEEL_LEFT（左轮）/ CAR_WHEEL_RIGHT（右轮）
  * @param  duty : 0 ~ MOTOR_DUTY_MAX(10000)；串口按百分比给，进来前已 ×100
  *
  * 为什么要单轮：双轮一起转时，“某个轮子不转”根本分不清是电机、驱动、
  * 接线还是编码器的问题。这个指令只驱动一侧、方向固定向前，另一侧保持
  * 0 占空比，方便听声音 / 看轮子 / 量电流。
  *
  * 进入前先 ApplyMode(CAR_MODE_IDLE)：停掉循迹、电机测试、风扇和旧单轮测试，
  * 保证任何时候只有这一个动作在跑。期间发 '0'/'1'/'2' 会立刻打断。
  */
void Car_Control_WheelRun(Car_Wheel_t wheel, uint16_t duty)
{
	uint8_t motor = (wheel == CAR_WHEEL_RIGHT) ? MOTOR_R : MOTOR_L;

	if(duty > MOTOR_DUTY_MAX)
	{
		duty = MOTOR_DUTY_MAX;
	}

	Car_Control_ApplyMode(CAR_MODE_IDLE);   /* 先停干净（同时清掉旧单轮测试） */

	Motor_SetSpeed(motor, duty);
	Motor_Enable();
	Led_Status_SetState(LED_ST_TEST);

	s_wheel_ticks = (uint16_t)(CAR_WHEEL_TEST_MS / CAR_CTRL_TICK_MS);
}

/**
  * @brief  风扇测试：负压风扇按给定占空比跑 CAR_FAN_TEST_MS 毫秒，期间两颗灯同频快闪
  * @param  duty: 0 ~ 1000（M3PWM 量程）；0 时只闪灯不吹风，方便单看灯
  *
  * 执行期间电机保持禁用（先 ApplyMode(IDLE) 停干净），模式名仍显示 IDLE，
  * 时间到自动停风扇、回待机闪灯。期间发 0/1/2 会立刻打断。
  */
void Car_Control_FanTest(uint16_t duty)
{
	if(duty > 1000u)
	{
		duty = 1000u;
	}

	Car_Control_ApplyMode(CAR_MODE_IDLE);   /* 停电机、清掉旧的点动/测试 */
	M3PWM_SetDutyCycle(0);                  /* 先压最低油门，等电调解锁 */
	Led_Status_SetState(LED_ST_FAN_TEST);

	s_fan_duty      = duty;
	s_fan_arm_ticks = (uint16_t)(CAR_FAN_ARM_MS / CAR_CTRL_TICK_MS);
	s_fan_ticks     = (uint16_t)(CAR_FAN_TEST_MS / CAR_CTRL_TICK_MS);
}

/**
  * @brief  延时风扇测试：先空等 CAR_FAN_DELAY_MS（只闪灯、风扇压最低油门），
  *         再用给定占空比吹 CAR_FAN_TEST_MS，到点自动停风扇回待机
  * @param  duty: 0 ~ 1000（M3PWM 量程）
  *
  * 串口 'ff' 指令专用。等待的 5 秒里有三个作用：
  *   1) 电调早已在最低油门上完成解锁，推油门即转，不依赖 M3PWM_Init 的延时；
  *   2) 给操作者留出“按下录制 / 手离开桨叶”的时间；
  *   3) 两颗指示灯用超快同频闪（LED_ST_FAN_FAST）提示“倒计时中”。
  *
  * 期间电机保持禁用；发 0/1/2 会立刻打断（ApplyMode 会清掉本测试的计时）。
  */
void Car_Control_FanDelayedTest(uint16_t duty)
{
	if(duty > 1000u)
	{
		duty = 1000u;
	}

	Car_Control_ApplyMode(CAR_MODE_IDLE);   /* 停电机、清掉旧的测试 */
	M3PWM_SetDutyCycle(0);                  /* 等待期保持最低油门，等电调解锁 */
	Led_Status_SetState(LED_ST_FAN_FAST);   /* 超快闪，和普通 'f' 区分开 */

	s_fdelay_duty  = duty;
	s_fdelay_ticks = (uint16_t)(CAR_FAN_DELAY_MS / CAR_CTRL_TICK_MS);
	s_frun_ticks   = (uint16_t)(CAR_FAN_TEST_MS / CAR_CTRL_TICK_MS);
}

Car_Mode_t Car_Control_GetMode(void)
{
	return s_mode;
}

const char *Car_Control_GetModeName(void)
{
	switch(s_mode)
	{
		case CAR_MODE_TRACK:      return "TRACK";
		case CAR_MODE_MOTOR_TEST: return "TEST ";
		case CAR_MODE_SPEED_TEST: return "SPEED";
		case CAR_MODE_SPEED_TUNE: return "TUNE ";
		case CAR_MODE_IDLE:
		default:                  return "IDLE ";
	}
}

/**
  * @brief  上电初始化：进入待机（电机禁用、风扇停、闭环复位）
  */
void Car_Control_Init(void)
{
	s_mode = CAR_MODE_IDLE;
	star_car = 0;

	Led_Status_Init();
	Motor_StopAll();
	Motor_Disable();
	M3PWM_SetDutyCycle(0);
	Led_Status_SetState(LED_ST_IDLE);
	SpeedPID_Reset(&g_speed_pid_l);
	SpeedPID_Reset(&g_speed_pid_r);
	PositionPID_Reset(&g_position_pid);
}

/**
  * @brief  切换运行模式
  * @param  mode: CAR_MODE_IDLE / CAR_MODE_TRACK / CAR_MODE_MOTOR_TEST / CAR_MODE_SPEED_TUNE
  */
void Car_Control_ApplyMode(Car_Mode_t mode)
{
	s_mode = mode;
	star_car = 0;                       /* 先退出闭环，再按新模式下动作 */
	s_tune_run   = 0;                   /* 切模式时退出调参闭环 */
	s_tune_tgt_l = 0;
	s_tune_tgt_r = 0;
	s_test_ticks = 0;                   /* 切模式时把电机测试的计时清零 */
	s_test_phase = 0;
	s_wheel_ticks = 0;                  /* 切模式时取消单轮测试 */
	s_spd_ticks   = 0;                  /* 切模式时取消速度环标定 */
	s_fan_ticks  = 0;                   /* 切模式时取消风扇测试 */
	s_fan_arm_ticks = 0;
	s_fan_duty   = 0;
	s_fdelay_ticks = 0;                 /* 切模式时取消延时风扇测试 */
	s_frun_ticks   = 0;
	s_fdelay_duty  = 0;

	switch(mode)
	{
		case CAR_MODE_TRACK:
			/* 发车：风扇起压 + 使能电机 + 闭环循迹（LED0 常亮、丢线时 LED1 快闪） */
			SpeedPID_Reset(&g_speed_pid_l);
			SpeedPID_Reset(&g_speed_pid_r);
			PositionPID_Reset(&g_position_pid);
			Led_Status_SetState(LED_ST_TRACK);
			M3PWM_SetDutyCycle(CAR_TRACK_FAN_DUTY);
			Motor_Enable();
			star_car = 1;
			break;

		case CAR_MODE_SPEED_TUNE:
			/* 速度环调参：只切模式并停干净，等串口发 ! 才闭环 */
			Motor_StopAll();
			Motor_Disable();
			M3PWM_SetDutyCycle(0);
			SpeedPID_Reset(&g_speed_pid_l);
			SpeedPID_Reset(&g_speed_pid_r);
			Led_Status_SetState(LED_ST_TEST);
			break;

		case CAR_MODE_MOTOR_TEST:
			/* 电机测试：双电机按 CAR_TEST_DUTY（30%）正向转，风扇不转（两灯交替闪） */
			Led_Status_SetState(LED_ST_TEST);
			M3PWM_SetDutyCycle(0);
			Motor_SetSpeed(MOTOR_R, CAR_TEST_DUTY);
			Motor_SetSpeed(MOTOR_L, CAR_TEST_DUTY);
			Motor_Enable();
			break;

		case CAR_MODE_IDLE:
		default:
			/* 停止运行 */
			Motor_StopAll();
			Motor_Disable();
			M3PWM_SetDutyCycle(0);
			Led_Status_SetState(LED_ST_IDLE);
			SpeedPID_Reset(&g_speed_pid_l);
			SpeedPID_Reset(&g_speed_pid_r);
			PositionPID_Reset(&g_position_pid);
			break;
	}
}

/**
  * @brief  速度环标定：只闭环一侧轮子，目标速度由串口 testl / testr 给
  * @param  wheel : CAR_WHEEL_LEFT（左轮）/ CAR_WHEEL_RIGHT（右轮）
  * @param  target: 目标速度，单位 mm/s（ABEncoder 已按 54476 counts/m 标定换算）
  *
  * 用途：把速度环单独拿出来做阶跃实验——给定目标、看反馈怎么爬、超调多少、
  * 稳定多久。另一侧电机保持 0，风扇也不起压（用户要求），所以车不会被推着走。
  * 到时（CAR_SPEED_TEST_MS）自动回待机；期间发 s / 0 立刻打断。
  */
void Car_Control_SpeedTest(Car_Wheel_t wheel, int16_t target)
{
	if(target < 0)
	{
		target = 0;
	}
	if(target > CAR_SPEED_TEST_MAX)
	{
		target = CAR_SPEED_TEST_MAX;
	}

	Car_Control_ApplyMode(CAR_MODE_IDLE);   /* 先停干净，同时清掉旧标定 */

	s_spd_wheel  = wheel;
	s_spd_target = target;
	s_spd_ticks  = (uint16_t)(CAR_SPEED_TEST_MS / CAR_CTRL_TICK_MS);

	SpeedPID_Reset(&g_speed_pid_l);
	SpeedPID_Reset(&g_speed_pid_r);

	s_mode = CAR_MODE_SPEED_TEST;
	M3PWM_SetDutyCycle(0);                  /* 风扇不起压 */
	Motor_SetSpeed(MOTOR_L, 0);
	Motor_SetSpeed(MOTOR_R, 0);
	Motor_Enable();
	Led_Status_SetState(LED_ST_TEST);
}

/**
  * @brief  正在做速度环标定吗（串口任务用它决定要不要打 PID 数据）
  * @param  target: 非 NULL 时回填目标速度
  * @param  wheel : 非 NULL 时回填当前闭环的是哪一侧
  * @return 1 = 正在标定，0 = 没在标定
  */
uint8_t Car_Control_GetSpeedTest(int16_t *target, Car_Wheel_t *wheel)
{
	if(s_mode != CAR_MODE_SPEED_TEST)
	{
		return 0u;
	}
	if(target != NULL)
	{
		*target = s_spd_target;
	}
	if(wheel != NULL)
	{
		*wheel = s_spd_wheel;
	}
	return 1u;
}

/**
  * @brief  速度环标定的每 2ms 动作：闭环一侧、另一侧停
  */
static void Car_Control_SpeedTestUpdate(void)
{
	float duty;

	if(s_spd_ticks == 0u)
	{
		Car_Control_ApplyMode(CAR_MODE_IDLE);
		return;
	}
	s_spd_ticks--;

	if(s_spd_wheel == CAR_WHEEL_LEFT)
	{
		duty = SpeedPID_Calculate(&g_speed_pid_l, (float)s_spd_target, speed_left_mm_s);
		Motor_SetSpeedClamped(MOTOR_L, duty);
		Motor_SetSpeed(MOTOR_R, 0);
	}
	else
	{
		duty = SpeedPID_Calculate(&g_speed_pid_r, (float)s_spd_target, speed_right_mm_s);
		Motor_SetSpeedClamped(MOTOR_R, duty);
		Motor_SetSpeed(MOTOR_L, 0);
	}
}

/**
  * @brief  进入速度环调参模式：切模式并停干净，等串口发 ! 才开始闭环
  */
void Car_Control_TuneEnter(void)
{
	Car_Control_ApplyMode(CAR_MODE_SPEED_TUNE);
}

/**
  * @brief  停车并清目标，但留在调参模式里（hlt 与定时到点共用）
  */
static void Car_Control_TuneStop(void)
{
	s_tune_run   = 0u;
	s_tune_tgt_l = 0;
	s_tune_tgt_r = 0;
	Motor_SetSpeed(MOTOR_L, 0);
	Motor_SetSpeed(MOTOR_R, 0);
	SpeedPID_Reset(&g_speed_pid_l);
	SpeedPID_Reset(&g_speed_pid_r);
}

/**
  * @brief  `! <ms> <l|r|all>`：按当前 vl / vr 目标定时闭环，到点自动停车（留在模式内）
  * @param  side : CAR_TUNE_SIDE_L / R / ALL；只跑一侧时另一侧目标强制 0
  * @param  ms   : 运行时长（1 ~ CAR_TUNE_RUN_MS_MAX）
  * @note   已经在跑的时候再发 ! 直接忽略，等这一组跑完
  */
void Car_Control_TuneRun(uint8_t side, uint16_t ms)
{
	if(s_mode != CAR_MODE_SPEED_TUNE)
	{
		return;
	}
	if(s_tune_run != 0u)
	{
		return;
	}
	if(ms == 0u)
	{
		ms = 1u;
	}
	if(ms > CAR_TUNE_RUN_MS_MAX)
	{
		ms = CAR_TUNE_RUN_MS_MAX;
	}

	if(side == (uint8_t)CAR_TUNE_SIDE_L)
	{
		s_tune_tgt_r = 0;
	}
	else if(side == (uint8_t)CAR_TUNE_SIDE_R)
	{
		s_tune_tgt_l = 0;
	}
	else
	{
		side = (uint8_t)CAR_TUNE_SIDE_ALL;
	}

	SpeedPID_Reset(&g_speed_pid_l);
	SpeedPID_Reset(&g_speed_pid_r);
	s_tune_side     = side;
	s_tune_ms       = ms;
	s_tune_left_ms  = (uint32_t)ms;
	s_tune_run      = 1u;

	/* 调参模式不吹负压：直接使能电机开始闭环计时 */
	Motor_Enable();
}

/**
  * @brief  hlt：临时停车（目标清零、占空比 0），但留在调参模式里
  */
void Car_Control_TuneHalt(void)
{
	if(s_mode != CAR_MODE_SPEED_TUNE)
	{
		return;
	}
	Car_Control_TuneStop();
}

/**
  * @brief  vl / vr：设置某一侧的目标速度（mm/s，0 表示这一侧不闭环）
  */
void Car_Control_TuneSetTarget(Car_Wheel_t wheel, int16_t mm_s)
{
	if(mm_s < 0)
	{
		mm_s = 0;
	}
	if(mm_s > CAR_TUNE_MAX)
	{
		mm_s = CAR_TUNE_MAX;
	}
	if(wheel == CAR_WHEEL_LEFT)
	{
		s_tune_tgt_l = mm_s;
	}
	else
	{
		s_tune_tgt_r = mm_s;
	}
}

/**
  * @brief  在线改某一侧的一个增益（不写 Flash，掉电丢）
  */
void Car_Control_TuneSetGain(Car_Wheel_t wheel, Car_TuneTerm_t term, float value)
{
	SpeedPID_Controller_t *pid = (wheel == CAR_WHEEL_LEFT) ? &g_speed_pid_l : &g_speed_pid_r;

	if(value < 0.0f)
	{
		value = 0.0f;
	}
	if(value > CAR_TUNE_GAIN_MAX)
	{
		value = CAR_TUNE_GAIN_MAX;
	}

	switch(term)
	{
		case CAR_TUNE_KI: pid->param.ki = value; break;
		case CAR_TUNE_KD: pid->param.kd = value; break;
		case CAR_TUNE_KP:
		default:          pid->param.kp = value; break;
	}
}

/**
  * @brief  取调参模式当前状态（串口任务打印 CSV 用，只读）
  */
void Car_Control_TuneSnapshot(Car_TuneSnapshot_t *snap)
{
	if(snap == NULL)
	{
		return;
	}
	snap->running    = (s_tune_run != 0u) ? 1u : 0u;
	snap->side       = s_tune_side;
	snap->run_ms     = s_tune_ms;
	snap->target_l   = s_tune_tgt_l;
	snap->target_r   = s_tune_tgt_r;
	snap->feedback_l = (int16_t)g_speed_pid_l.last_feedback;
	snap->feedback_r = (int16_t)g_speed_pid_r.last_feedback;
	snap->output_l   = (int16_t)g_speed_pid_l.last_output;
	snap->output_r   = (int16_t)g_speed_pid_r.last_output;
}

/**
  * @brief  调参模式的每 2ms 动作：两侧各自闭环（目标为 0 的一侧停、不积分）
  */
static void Car_Control_TuneUpdate(void)
{
	float duty;

	if(s_tune_run != 0u)
	{
		if(s_tune_left_ms <= (uint32_t)CAR_CTRL_TICK_MS)
		{
			Car_Control_TuneStop();      /* 定时到点：停，但留在模式里 */
		}
		else
		{
			s_tune_left_ms -= (uint32_t)CAR_CTRL_TICK_MS;
		}
	}

	if(s_tune_run == 0u)
	{
		Motor_SetSpeed(MOTOR_L, 0);
		Motor_SetSpeed(MOTOR_R, 0);
		return;
	}

	if(s_tune_tgt_l > 0)
	{
		duty = SpeedPID_Calculate(&g_speed_pid_l, (float)s_tune_tgt_l, speed_left_mm_s);
		Motor_SetSpeedClamped(MOTOR_L, duty);
	}
	else
	{
		Motor_SetSpeed(MOTOR_L, 0);
		SpeedPID_Reset(&g_speed_pid_l);
	}

	if(s_tune_tgt_r > 0)
	{
		duty = SpeedPID_Calculate(&g_speed_pid_r, (float)s_tune_tgt_r, speed_right_mm_s);
		Motor_SetSpeedClamped(MOTOR_R, duty);
	}
	else
	{
		Motor_SetSpeed(MOTOR_R, 0);
		SpeedPID_Reset(&g_speed_pid_r);
	}
}

/**
  * @brief  每 2ms 调用一次：执行当前模式（TRACK 循迹 / MOTOR_TEST / SPEED_TEST / SPEED_TUNE / IDLE）
  */
void Car_Control_Update(void)
{
	/* 延时风扇测试（'ff'）最高优先：等待期只闪灯不吹风，时间到才推油门 */
	if(s_fdelay_ticks > 0)
	{
		s_fdelay_ticks--;
		if(s_fdelay_ticks == 0)
		{
			/* 倒计时结束：推油门开始吹。电调此时已经解锁，无需再等 */
			M3PWM_SetDutyCycle(s_fdelay_duty);
		}
		return;
	}

	if(s_frun_ticks > 0)
	{
		s_frun_ticks--;
		if(s_frun_ticks == 0)
		{
			/* 吹完 5 秒：回待机（风扇落回最低油门，灯回待机闪） */
			Car_Control_ApplyMode(CAR_MODE_IDLE);
		}
		return;
	}

	/* 风扇测试优先：期间不跑模式机，时间到自动停风扇回待机 */
	if(s_fan_ticks > 0)
	{
		if(s_fan_arm_ticks > 0)
		{
			/* 解锁阶段：保持最低油门，等电调自检完成，这段时间电调不会转 */
			s_fan_arm_ticks--;
		}
		else
		{
			if(s_fan_ticks == (uint16_t)(CAR_FAN_TEST_MS / CAR_CTRL_TICK_MS))
			{
				/* 刚解锁完：推油门。电调自己会软启动，不需要我们做斜坡 */
				M3PWM_SetDutyCycle(s_fan_duty);
				printf("[fan] armed, throttle %u/1000 (%uus)\r\n",
				       (unsigned int)s_fan_duty, (unsigned int)M3PWM_GetPulseUs());
			}

			s_fan_ticks--;
			if(s_fan_ticks == 0)
			{
				Car_Control_ApplyMode(CAR_MODE_IDLE);
			}
		}
		return;
	}

	/* 单轮测试优先：测试期间不跑模式机，时间到自动回待机 */
	if(s_wheel_ticks > 0)
	{
		s_wheel_ticks--;
		if(s_wheel_ticks == 0)
		{
			Car_Control_ApplyMode(CAR_MODE_IDLE);
		}
		return;
	}

	switch(s_mode)
	{
		case CAR_MODE_TRACK:
			/* 发车闭环：位置环 + 左右两个速度环，内部会下发左右轮 PWM */
			PID_Control_Update();
			break;

		case CAR_MODE_SPEED_TEST:
			/* 速度环标定：单轮按目标速度闭环 */
			Car_Control_SpeedTestUpdate();
			break;

		case CAR_MODE_SPEED_TUNE:
			/* 速度环调参：左右各自闭环，目标/增益都来自串口 */
			Car_Control_TuneUpdate();
			break;

		case CAR_MODE_MOTOR_TEST:
			/* 测试模式：占空比/使能在 ApplyMode 里已经设好，这里只负责时序
			   0~CAR_TEST_FWD_MS      : 向前
			   ~CAR_TEST_TIME_MS      : 停车（两路半桥不能反转）
			   超过 CAR_TEST_TIME_MS  : 停车回待机 */
			s_test_ticks++;
			{
				uint32_t t_ms = (uint32_t)s_test_ticks * CAR_CTRL_TICK_MS;

				if(t_ms > CAR_TEST_TIME_MS)
				{
					Car_Control_ApplyMode(CAR_MODE_IDLE);   /* 4 秒到，自动停车 */
				}
				else if(t_ms > CAR_TEST_FWD_MS && s_test_phase == 0)
				{
					/* 进入第 2 段：两路半桥不能反转，这里直接停车（使能保持不动） */
					s_test_phase = 1;
					Motor_SetSpeed(MOTOR_R, 0);
					Motor_SetSpeed(MOTOR_L, 0);
				}
			}
			break;

		case CAR_MODE_IDLE:
		default:
			/* 待机：停车动作已经在 Car_Control_ApplyMode(IDLE) 里做过一次，
			 * 这里不再每 2ms 重复写电机/风扇寄存器（原来的实现会一直刷）。 */
			break;
	}
}

/*------------------------------------------------------------------------------
 * 串口指令解析已迁移到 App\app_serial.c（serial 任务）：
 *   收到 '0'~'5' / 'w' / '?' 等后通过指令队列投递给控制任务，
 *   由控制任务调用 Car_Control_ApplyMode() 统一执行，避免与本文件的
 *   2ms 状态机并发写电机/指示灯。
 *----------------------------------------------------------------------------*/

/**
  * @brief  调试用：串口回发 PA15 / PB3 的引脚配置与真实电平
  *
  *   CRH/CRL 里 4bit 的配置含义：0x3 / 0x2 = 推挽输出，0x4 = 浮空输入，0x8 = 上/下拉输入
  *   ODR = 软件写出去的电平；IDR = 引脚上真实读到的电平
  *   如果 ODR=1 但 IDR=0，说明这个脚被外设（最常见是 JTAG）占着，软件的写没生效
  *   AFIO_MAPR[27:24]：0x2 = JTAG 已关（SWD 保留），0x0 / 0x1 = JTAG 还开着
  */
void Car_Control_PrintLedDiag(void)
{
	uint16_t idr_a = GPIO_ReadInputData(GPIOA);
	uint16_t idr_b = GPIO_ReadInputData(GPIOB);
	uint16_t odr_a = GPIO_ReadOutputData(GPIOA);
	uint16_t odr_b = GPIO_ReadOutputData(GPIOB);
	uint8_t  cfg_pa15 = (uint8_t)((GPIOA->CRH >> 28) & 0x0F);   /* PA15 -> CRH bit31:28 */
	uint8_t  cfg_pb3  = (uint8_t)((GPIOB->CRL >> 12) & 0x0F);   /* PB3  -> CRL bit15:12 */
	uint8_t  mapr     = (uint8_t)((AFIO->MAPR >> 24) & 0x0F);

	printf("mode=%d cfg:PA15=0x%X PB3=0x%X (0x3=push-pull out)\r\n",
	       (int)s_mode, (unsigned int)cfg_pa15, (unsigned int)cfg_pb3);
	printf("level: PA15 IDR=%d ODR=%d | PB3 IDR=%d ODR=%d\r\n",
	       (idr_a & RGB0_PIN) ? 1 : 0, (odr_a & RGB0_PIN) ? 1 : 0,
	       (idr_b & RGB1_PIN) ? 1 : 0, (odr_b & RGB1_PIN) ? 1 : 0);
	/* 注意：SWJ_CFG 是只写位，原厂芯片读回来通常是 0，所以这一行只作参考 */
	printf("AFIO_MAPR[27:24]=0x%X (0x2=JTAG disabled)\r\n", (unsigned int)mapr);
	printf("motor: MOTOREN=%d R_PWM=%d/1000 L_PWM=%d/1000 FAN=%d/1000\r\n",
	       GPIO_ReadOutputDataBit(MOTOREN_PORT, MOTOREN_PIN) ? 1 : 0,
	       (int)((uint32_t)MOTOR_TIM->CCR2 * 1000 / (MOTOR_TIM->ARR + 1)),
	       (int)((uint32_t)MOTOR_TIM->CCR4 * 1000 / (MOTOR_TIM->ARR + 1)),
	       (int)((uint32_t)FAN_PWM_TIM->CCR4 * 1000 / (FAN_PWM_TIM->ARR + 1)));
}
