/**
  ******************************************************************************
  * @file    Act.c
  * @brief   固定动作（act）：按灰度阵列的形状触发，按动作表顺序执行
  *
  *  数据流（都在 2ms 控制任务里）：
  *    Car_Control_Update() → TRACK 且空闲 → Act_Poll()  检测 4 种触发形状
  *                          → 正在执行     → Act_Update() 跑当前动作
  *
  *  动作表 s_act_plan[] 是“本次比赛/本次跑”的动作序列：每触发一次，游标 +1，
  *  执行表里那一格。触发条件是哪一条都不影响选哪个动作——形状只是“到点了”的
  *  信号。表用完再触发 ⇒ 停车报警（Act_Poll 里处理）。
  *
  *  直角转弯的物理量：
  *    车身绕内侧（刹住的）轮接地点转 90°，外侧轮滚过的弧长 = π/2 × 轮距，
  *    轮距 130 mm ⇒ 204.2 mm。编码器 54476 counts/m ⇒ 11125 counts。
  *    用“左右编码器之差”而不是单侧里程：差 = (s_R − s_L) = 轮距 × 转角，
  *    内侧轮即使被拖动一点也被自然地扣掉（差速里程计的标准做法）。
  ******************************************************************************
  */

#include "Act.h"

#include "ADC_get.h"
#include "BlackPoint_Finder.h"
#include "ABEncoder.h"
#include "Motor_ctr.h"
#include "PID_Controller.h"
#include "Car_Control.h"
#include "Led_Status.h"

/*------------------------------------------------------------------------------
 * 动作表：按执行先后顺序，每触发一次取一格
 *
 * 现在先排两次左转（用户 2026-11-14 指定）。以后按赛道实际顺序改这个数组即可，
 * 动作类型只能是 ACT_TL / ACT_TR（以后有别的动作再加值和处理分支）。
 *----------------------------------------------------------------------------*/
static const uint8_t s_act_plan[] =
{
	ACT_TL,
	ACT_TL
};
#define ACT_PLAN_LEN        (sizeof(s_act_plan) / sizeof(s_act_plan[0]))

/*------------------------------------------------------------------------------
 * 参数
 *----------------------------------------------------------------------------*/
#define ACT_SENSOR_COUNT        16u     /* 灰度路数 */
#define ACT_HALF_COUNT          8u      /* 左/右半区各 8 路（p0~p7 / p8~p15） */

#define ACT_LOST_ON_MAX         4u      /* A：至少 12 路扫不到线 ⇒ 扫到的 ≤4 */
#define ACT_WIDE_ON_MIN         12u     /* D：至少 12 路扫到线 */
#define ACT_HALF_STRONG         6u      /* B/C：半区至少 6 路扫到线 */
#define ACT_HALF_WEAK           1u      /* B/C：另半区至多 1 路扫到线 */

#define ACT_TRIGGER_TICKS       1u      /* 命中 1 拍（2ms）就触发：单次采样符合即认（2026-11-14 用户要求） */
#define ACT_REARM_QUIET_TICKS   5u      /* 动作结束后，条件消失保持 5 拍才重新武装 */

#define ACT_TURN_ANGLE_DEG      90.0f   /* 直角 */
#define ACT_WHEEL_BASE_MM       110.0f  /* act 转角标定值（2026-11-14 用户上车实测定）：真两轮中心距是
                                         * 130 mm（= PID_Controller.c 的 WHEEL_BASE_MM，航向估计仍用它），
                                         * 但刹住内侧轮时车身绕的圆心比轮接地点更靠里、内侧轮还会被拖一点，
                                         * 按 130 算会多转十几度；标成 110 后 90° 与地面标线吻合。
                                         * 只用于本文件 Act_TurnDiffTarget() 的转角目标。 */
#define ACT_OUTER_SPEED_MM_S    1200.0f /* 外侧轮闭环目标（档 2 边界，速度环照样跟得住） */

#define ACT_STRAIGHT_MM         100.0f  /* 直走（ACT_SF）里程，按两轮平均计数判 */
#define ACT_STRAIGHT_SPEED_MM_S 1100.0f /* 直走两轮闭环目标 = 循迹直道基准 */

#define ACT_TIMEOUT_MS          1000u   /* 保护：1s 还没到位 ⇒ 停 + 报警 */
#define ACT_TIMEOUT_TICKS       (ACT_TIMEOUT_MS / 2u)

#define ACT_DEG_TO_RAD          0.01745329252f

/*------------------------------------------------------------------------------
 * 延迟事件信箱（4 格环形；2ms 环里只登记，串口任务打印）
 *----------------------------------------------------------------------------*/
#define ACT_EVENT_SLOTS         4u

static volatile uint8_t s_ev_code[ACT_EVENT_SLOTS];
static volatile uint8_t s_ev_act[ACT_EVENT_SLOTS];
static volatile int32_t s_ev_diff[ACT_EVENT_SLOTS];
static volatile uint16_t s_ev_ms[ACT_EVENT_SLOTS];
static volatile uint8_t s_ev_head = 0u;     /* 生产者（控制任务）写 */
static volatile uint8_t s_ev_tail = 0u;     /* 消费者（串口任务）读 */

/*------------------------------------------------------------------------------
 * 运行状态
 *----------------------------------------------------------------------------*/
static uint8_t  s_running   = 0u;   /* 1 = 正在执行动作 */
static uint8_t  s_act_id    = 0u;   /* 当前动作 */
static uint8_t  s_from_test = 0u;   /* 1 = 串口 '3'/'4'/'5' 台架测试（结束后断电回 IDLE） */
static uint8_t  s_armed     = 0u;   /* 1 = 允许触发（上电/复位后要等条件先消失） */
static uint8_t  s_hold      = 0u;   /* 触发条件连续成立拍数 */
static uint8_t  s_quiet     = 0u;   /* 触发条件连续不成立拍数 */
static uint8_t  s_plan_idx  = 0u;   /* 动作表游标 */

static int32_t  s_cnt_l0    = 0;    /* 动作开始时的左编码器计数 */
static int32_t  s_cnt_r0    = 0;    /* 动作开始时的右编码器计数 */
static int32_t  s_diff_target = 0;  /* 本次动作的判据目标（counts）：转弯 = |左右差|，直走 = 两轮平均里程 */
static uint16_t s_ticks     = 0u;   /* 已执行拍数 */

/*------------------------------------------------------------------------------
 * 内部工具
 *----------------------------------------------------------------------------*/
static void Act_PostEvent(uint8_t code, uint8_t act_id, int32_t diff, uint16_t ms)
{
	uint8_t next = (uint8_t)((s_ev_head + 1u) % ACT_EVENT_SLOTS);

	if(next == s_ev_tail)
	{
		return;                     /* 满了就丢新的，不阻塞控制环 */
	}

	s_ev_code[s_ev_head] = code;
	s_ev_act[s_ev_head]  = act_id;
	s_ev_diff[s_ev_head] = diff;
	s_ev_ms[s_ev_head]   = ms;
	s_ev_head = next;
}

static int32_t Act_TurnDiffTarget(void)
{
	/* 90° 弧长（mm）× counts/mm，四舍五入到整数 counts */
	float mm = (ACT_TURN_ANGLE_DEG * ACT_DEG_TO_RAD) * ACT_WHEEL_BASE_MM;
	float counts = mm * (ENC_COUNTS_PER_METER / 1000.0f);

	if(counts < 1.0f)
	{
		counts = 1.0f;
	}

	return (int32_t)(counts + 0.5f);
}

static int32_t Act_StraightTarget(void)
{
	/* 直走里程（mm）× counts/mm，四舍五入到整数 counts */
	float counts = ACT_STRAIGHT_MM * (ENC_COUNTS_PER_METER / 1000.0f);

	if(counts < 1.0f)
	{
		counts = 1.0f;
	}

	return (int32_t)(counts + 0.5f);
}

/* 读一拍灰度形状：返回 4 种触发条件的位掩码，0 = 都不是 */
static uint8_t Act_ReadPattern(void)
{
	uint8_t  i;
	uint8_t  on_left  = 0u;
	uint8_t  on_right = 0u;
	uint8_t  on_total;
	uint8_t  mask = 0u;

	for(i = 0u; i < ACT_HALF_COUNT; i++)
	{
		if(BlackPoint_Finder_IsLine(i, (uint16_t)g_mux_adc_values[i]))
		{
			on_left++;
		}
	}

	for(i = ACT_HALF_COUNT; i < ACT_SENSOR_COUNT; i++)
	{
		if(BlackPoint_Finder_IsLine(i, (uint16_t)g_mux_adc_values[i]))
		{
			on_right++;
		}
	}

	on_total = (uint8_t)(on_left + on_right);

	if(on_total <= ACT_LOST_ON_MAX)
	{
		mask |= 0x01u;              /* A：几乎全丢线 */
	}

	if(on_total >= ACT_WIDE_ON_MIN)
	{
		mask |= 0x08u;              /* D：一大片白 */
	}

	if((on_left >= ACT_HALF_STRONG) && (on_right <= ACT_HALF_WEAK))
	{
		mask |= 0x02u;              /* B：线压在左半边 */
	}

	if((on_right >= ACT_HALF_STRONG) && (on_left <= ACT_HALF_WEAK))
	{
		mask |= 0x04u;              /* C：线压在右半边 */
	}

	return mask;
}

/* 启动一个动作（不检查是否正在执行，调用前自己保证） */
static void Act_Start(uint8_t act_id, uint8_t from_test)
{
	uint8_t outer_side;

	s_running   = 1u;
	s_act_id    = act_id;
	s_from_test = from_test;
	s_cnt_l0    = left_ecoder_cnt;
	s_cnt_r0    = right_ecoder_cnt;
	s_diff_target = (act_id == ACT_SF) ? Act_StraightTarget() : Act_TurnDiffTarget();
	s_ticks     = 0u;
	s_hold      = 0u;
	s_quiet     = 0u;
	s_armed     = 0u;

	/* 两个速度环都从头开始：内侧轮不再用循迹时的积分/输出 */
	SpeedPID_Reset(&g_speed_pid_l);
	SpeedPID_Reset(&g_speed_pid_r);

	if(act_id == ACT_SF)
	{
		/* 直走：两轮都要闭环，各用自己的档位增益（1100 落在档 1） */
		Car_Control_TrackApply(0u, ACT_STRAIGHT_SPEED_MM_S, &g_speed_pid_l);
		Car_Control_TrackApply(1u, ACT_STRAIGHT_SPEED_MM_S, &g_speed_pid_r);
	}
	else
	{
		/* 转弯：外侧轮用“档 2 升速”那套增益：1200 正好是档 2 边界，先让 Car_Control
		 * 把增益和前馈基准切过去，后面每拍只调 SpeedPID_Calculate() */
		outer_side = (act_id == ACT_TL) ? 1u : 0u;  /* 左转外侧是右轮 */
		Car_Control_TrackApply(outer_side, ACT_OUTER_SPEED_MM_S,
		                       (outer_side == 1u) ? &g_speed_pid_r : &g_speed_pid_l);
	}

	Motor_Enable();

	Act_PostEvent(ACT_EVT_START, act_id, 0, 0u);
}

/* 结束当前动作 */
static void Act_Finish(uint8_t ev_code, int32_t diff)
{
	uint8_t act_id = s_act_id;
	uint8_t test   = s_from_test;
	uint16_t ms    = (uint16_t)(s_ticks * 2u);

	/* 先停干净：两轮占空比 0、电机断电交给调用方决定 */
	Motor_SetSpeed(MOTOR_L, 0u);
	Motor_SetSpeed(MOTOR_R, 0u);
	SpeedPID_Reset(&g_speed_pid_l);
	SpeedPID_Reset(&g_speed_pid_r);

	s_running   = 0u;
	s_from_test = 0u;
	s_ticks     = 0u;
	s_hold      = 0u;
	s_quiet     = 0u;
	s_armed     = 0u;                   /* 必须等触发条件先消失才重新武装 */

	Act_PostEvent(ev_code, act_id, diff, ms);

	if(ev_code == ACT_EVT_TIMEOUT)
	{
		/* 转不到位：宁可停车，也别带着错的角度继续循迹 */
		Motor_Disable();
		if(test == 0u)
		{
			/* ApplyMode(IDLE) 里会把灯设回 LED_ST_IDLE，所以报警灯要在它之后设 */
			Car_Control_ApplyMode(CAR_MODE_IDLE);
		}
		Led_Status_SetState(LED_ST_ALARM);
		return;
	}

	if(test != 0u)
	{
		/* 台架测试：转完断电，留在 IDLE */
		Motor_Disable();
		Car_Control_TrackReset();
		PID_ResetRamp();
		PositionPID_Reset(&g_position_pid);
		return;
	}

	/* 赛道上：立刻回循迹。前馈基准清零（否则会拿转弯时的目标算 bump），
	 * 起步斜坡从 0 重爬（转完不会一上来就满舵）；位置环积分也清零——
	 * 转 90° 后位置误差会突跳一大截，留着旧积分就会先猛打一舵。 */
	Car_Control_TrackReset();
	PID_ResetRamp();
	PositionPID_Reset(&g_position_pid);
}

/*------------------------------------------------------------------------------
 * 对外接口
 *----------------------------------------------------------------------------*/
void Act_Init(void)
{
	s_diff_target = Act_TurnDiffTarget();
	Act_Reset();
}

void Act_Reset(void)
{
	Motor_SetSpeed(MOTOR_L, 0u);
	Motor_SetSpeed(MOTOR_R, 0u);

	s_running   = 0u;
	s_act_id    = 0u;
	s_from_test = 0u;
	s_armed     = 0u;                   /* 第一次触发前要先看到“没有触发条件” */
	s_hold      = 0u;
	s_quiet     = 0u;
	s_plan_idx  = 0u;
	s_ticks     = 0u;
}

void Act_Poll(void)
{
	uint8_t mask;

	if(s_running != 0u)
	{
		return;
	}

	mask = Act_ReadPattern();

	/* 动作刚结束 / 刚进循迹：等条件彻底消失一小会儿，再允许触发 */
	if(s_armed == 0u)
	{
		if(mask == 0u)
		{
			if(++s_quiet >= ACT_REARM_QUIET_TICKS)
			{
				s_armed = 1u;
				s_quiet = 0u;
			}
		}
		else
		{
			s_quiet = 0u;
		}
		return;
	}

	if(mask == 0u)
	{
		s_hold = 0u;
		return;
	}

	if(++s_hold < ACT_TRIGGER_TICKS)
	{
		return;
	}
	s_hold = 0u;

	/* 动作表走完：再遇到触发就停车报警，而不是继续跑 */
	if(s_plan_idx >= (uint8_t)ACT_PLAN_LEN)
	{
		s_armed = 0u;
		Act_PostEvent(ACT_EVT_PLAN_END, 0u, 0, 0u);
		Motor_StopAll();
		Motor_Disable();
		Car_Control_ApplyMode(CAR_MODE_IDLE);   /* 内部会把灯设成 IDLE */
		Led_Status_SetState(LED_ST_ALARM);      /* 再盖成报警，提示“表跑完了” */
		return;
	}

	Act_Start(s_act_plan[s_plan_idx], 0u);
	s_plan_idx++;
}

void Act_Update(void)
{
	int32_t dl;
	int32_t dr;
	int32_t diff;
	int32_t prog;
	uint8_t inner;
	uint8_t outer;
	SpeedPID_Controller_t *pid;
	float feedback;
	float duty;

	if(s_running == 0u)
	{
		return;
	}

	dl = left_ecoder_cnt  - s_cnt_l0;
	dr = right_ecoder_cnt - s_cnt_r0;

	/* 直走：判据是两轮平均里程；两轮各自闭环 1100 mm/s */
	if(s_act_id == ACT_SF)
	{
		prog = (dl + dr) / 2;

		if(prog >= s_diff_target)
		{
			Act_Finish(ACT_EVT_DONE, prog);
			return;
		}

		if(++s_ticks >= ACT_TIMEOUT_TICKS)
		{
			Act_Finish(ACT_EVT_TIMEOUT, prog);
			return;
		}

		duty = SpeedPID_Calculate(&g_speed_pid_l, ACT_STRAIGHT_SPEED_MM_S, speed_left_mm_s);
		Motor_SetSpeedClamped(MOTOR_L, duty);
		duty = SpeedPID_Calculate(&g_speed_pid_r, ACT_STRAIGHT_SPEED_MM_S, speed_right_mm_s);
		Motor_SetSpeedClamped(MOTOR_R, duty);
		return;
	}

	/* 左转时外侧是右轮：差 = 右 − 左；右转镜像 */
	if(s_act_id == ACT_TL)
	{
		diff  = dr - dl;
		inner = MOTOR_L;
		outer = MOTOR_R;
		pid   = &g_speed_pid_r;
		feedback = speed_right_mm_s;
	}
	else
	{
		diff  = dl - dr;
		inner = MOTOR_R;
		outer = MOTOR_L;
		pid   = &g_speed_pid_l;
		feedback = speed_left_mm_s;
	}

	if(diff >= s_diff_target)
	{
		Act_Finish(ACT_EVT_DONE, diff);
		return;
	}

	if(++s_ticks >= ACT_TIMEOUT_TICKS)
	{
		Act_Finish(ACT_EVT_TIMEOUT, diff);
		return;
	}

	/* 内侧轮：占空比 0 且保持使能 = 半桥下管常通 = 短路制动（不是滑行） */
	Motor_SetSpeed(inner, 0u);

	/* 外侧轮：速度环闭环 1200 mm/s */
	duty = SpeedPID_Calculate(pid, ACT_OUTER_SPEED_MM_S, feedback);
	Motor_SetSpeedClamped(outer, duty);
}

uint8_t Act_IsRunning(void)
{
	return s_running;
}

uint8_t Act_TestStart(uint8_t act_id)
{
	if((act_id != ACT_TL) && (act_id != ACT_TR))
	{
		return 0u;
	}

	if(s_running != 0u)
	{
		return 0u;
	}

	Act_Start(act_id, 1u);
	return 1u;
}

uint8_t Act_TakeEvent(uint8_t *act_id, int32_t *diff_counts, uint16_t *elapsed_ms)
{
	uint8_t code = ACT_EVT_NONE;

	if(s_ev_tail == s_ev_head)
	{
		return ACT_EVT_NONE;
	}

	code = s_ev_code[s_ev_tail];

	if(act_id != 0)
	{
		*act_id = s_ev_act[s_ev_tail];
	}

	if(diff_counts != 0)
	{
		*diff_counts = s_ev_diff[s_ev_tail];
	}

	if(elapsed_ms != 0)
	{
		*elapsed_ms = s_ev_ms[s_ev_tail];
	}

	s_ev_tail = (uint8_t)((s_ev_tail + 1u) % ACT_EVENT_SLOTS);
	return code;
}

const char *Act_GetName(uint8_t act_id)
{
	if(act_id == ACT_TL)
	{
		return "act_tl";
	}

	if(act_id == ACT_TR)
	{
		return "act_tr";
	}

	if(act_id == ACT_SF)
	{
		return "act_sf";
	}

	return "act?";
}
