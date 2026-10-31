/**
  ******************************************************************************
  * @file    app_serial.c
  * @brief   串口任务（事件驱动 + 50Hz 波形帧）
  *
  *  收：USART2 RXNE 中断把字节塞进 128B 环形缓冲，并 vTaskNotifyGiveFromISR()
  *      唤醒本任务；本任务解析指令后投递到指令队列，由控制任务执行。
  *      这样中断里只做“入缓冲 + 通知”，不再有 while 阻塞读。
  *
  *  发：printf 仍然走阻塞式 fputc（保持原行为），但由 TxLock/TxUnlock
  *      （递归互斥量）串行化，避免波形帧与调试打印交错成乱码。
  *
  *  指令（与原 Car_Control_Task 完全兼容，另加几个调试指令）：
  *    's' 一键停车 IDLE      '1' 循迹 TRACK        '2' 电机测试
  *    '3' 指示灯常亮         '4' 指示灯常灭        '5' 引脚诊断
  *    'p' 风机 PWM 寄存器诊断                     'w' 切换波形帧
  *    '?' 状态行             'z' 栈水印/剩余堆     'g' 灰度 16 路实时打印（开关）
  *    '0' 仍然也是停车（保留原习惯，一车两键都能停）
  *    （朝向实时打印 'hx' 已随陀螺仪一起移除）
  *    'testl<0~400>' 左轮速度环标定（目标速度=编码器每 2ms 计数）
  *    'testr<0~400>' 右轮速度环标定
  *    'mc' 推车测距开关：第一次开始累计（每秒打一行），第二次停止并算每米计数
  *    'tune' 速度环调参模式：'!' 开始闭环、hlt 临时停车、0 退出；
  *           vl/vr<mm/s> 设左右目标，kpl/kpr 等在线改增益，kq 查参数，
  *           运行时每 100ms 打一行 CSV：ms,tL,fL,uL,tR,fR,uR
  *    'l<0~100>' 左轮单独按该占空比转 2 秒（回车结束）
  *    'r<0~100>' 右轮单独按该占空比转 2 秒（回车结束）
  *    'f<0~100>' 负压风扇按该油门跑 5 秒（先 1.5 秒最低油门等电调解锁，
  *               期间两灯同频快闪，时间到自动停）
  ******************************************************************************
  */

#include "FreeRTOS.h"
#include "task.h"
#include "app_config.h"
#include "app_state.h"

#include "Uart_Config.h"
#include "Car_Control.h"
#include "Led_Status.h"
#include "M3PWM.h"
#include "ABEncoder.h"
#include "PID_Controller.h"
#include "ADC_get.h"
#include "LSM6DSR_Config.h"
#include "stm32f10x_gpio.h"

/* 推车测距 "mc"（开关）：0=空闲 1=已收 m（等 c）；s_mc_start = 开始时刻 */
static uint8_t    s_mc_state = 0;
static TickType_t s_mc_start = 0;

#include <stdio.h>
#include <stdarg.h>
#include <string.h>

/* VOFA+ JustFloat 帧尾：00 00 80 7F（float 的 +inf，作为帧分隔符） */
static const uint8_t s_wave_tail[4] = { 0x00u, 0x00u, 0x80u, 0x7Fu };

/*------------------------------------------------------------------------------
 * 带锁的日志输出（vsnprintf 组包 → 一次发完，避免半行被波形帧插进来）
 *----------------------------------------------------------------------------*/
static void Serial_Log(const char *fmt, ...)
{
	char    buf[96];
	va_list ap;
	int     n;

	va_start(ap, fmt);
	n = vsnprintf(buf, sizeof(buf), fmt, ap);
	va_end(ap);

	if(n > 0)
	{
		Uart2_TxLock();
		Uart2_SendString(buf);
		Uart2_TxUnlock();
	}
}

/*------------------------------------------------------------------------------
 * 50Hz 波形帧：位置、左右轮速、陀螺 Z、电池电压（VOFA+ JustFloat）
 * 帧长 = 5 × 4 + 4 = 24 字节，115200bps 下约 2.1ms 发完
 *----------------------------------------------------------------------------*/
static void Serial_SendWave(void)
{
	float   ch[APP_WAVE_CHANNELS];
	uint8_t buf[APP_WAVE_CHANNELS * 4 + 4];

	ch[0] = (float)position_get / 10.0f;   /* 寻线位置 0~15 */
	ch[1] = speed_left_mm_s;               /* 左轮速度 mm/s */
	ch[2] = speed_right_mm_s;              /* 右轮速度 mm/s */
	ch[3] = LSE6DSR_data.gz_rads;          /* 陀螺 Z（rad/s） */
	ch[4] = BDI_V;                         /* 电池电压（V） */

	memcpy(buf, ch, sizeof(ch));
	memcpy(buf + sizeof(ch), s_wave_tail, sizeof(s_wave_tail));

	Uart2_TxLock();
	Uart2_SendBuf(buf, (uint16_t)sizeof(buf));
	Uart2_TxUnlock();
}

/*------------------------------------------------------------------------------
 * 调试：状态行 + 各任务栈水印 + 剩余堆
 *----------------------------------------------------------------------------*/
static void Serial_SendStatus(void)
{
	EventBits_t bits = xEventGroupGetBits(g_app_events);

	Serial_Log("mode=%s line=%s pos=%d batt=%d.%02dV spd=%d/%d mm/s wave=%d\r\n",
	           Car_Control_GetModeName(),
	           (bits & APP_EVT_LINE_FOUND) ? "ON" : "LOST",
	           (int)position_get,
	           (int)BDI_V,
	           (int)(BDI_V * 100.0f) % 100,
	           (int)speed_left_mm_s,
	           (int)speed_right_mm_s,
	           (int)g_wave_enable);
}

static void Serial_SendStacks(void)
{
	Serial_Log("stk(ctrl=%u keys=%u disp=%u ser=%u) heap=%u\r\n",
	           (unsigned int)uxTaskGetStackHighWaterMark(g_task_ctrl),
	           (unsigned int)uxTaskGetStackHighWaterMark(g_task_keys),
	           (unsigned int)uxTaskGetStackHighWaterMark(g_task_display),
	           (unsigned int)uxTaskGetStackHighWaterMark(g_task_serial),
	           (unsigned int)xPortGetFreeHeapSize());
}

/*------------------------------------------------------------------------------
 * 灰度 16 路一帧：原始值 + 黑白判定 + 当前寻线结果（'g' 开关打开时每周期调用）
 *   只读 g_mux_adc_values / result_BlackPoint，不写任何外设，循迹跑着也能打。
 *   用途：确认模拟开关地址线（S0..S3 = PB0/PB1/PB2/PB10）到 Y0..Y15 的对应关系，
 *         以及各路阈值还准不准——拿一张白纸/黑胶带逐路挡一下，看是不是同一列在变。
 *   注意：CtrlTask 每 2ms 更新一次数组，这里读到的 16 个值可能跨两个采样周期，
 *         做"哪一路最黑"的判断够用，做精确标定请多发几次取稳定值。
 *----------------------------------------------------------------------------*/
static int Serial_AppendUInt(char *buf, int pos, int size, unsigned int v)
{
	int w = snprintf(buf + pos, (size_t)(size - pos), " %4u", v);
	if(w < 0)                     return pos;
	if(pos + w >= size)           return size - 1;
	return pos + w;
}

static void Serial_SendGray(void)
{
	char    buf[96];
	int     n;
	uint8_t i;
	uint8_t base;

	Uart2_TxLock();

	for(base = 0u; base < 16u; base = (uint8_t)(base + 8u))
	{
		n  = snprintf(buf, sizeof(buf), "gray raw %2u-%2u:", (unsigned int)base, (unsigned int)(base + 7u));
		for(i = base; i < (uint8_t)(base + 8u); i++)
		{
			n = Serial_AppendUInt(buf, n, (int)sizeof(buf), (unsigned int)g_mux_adc_values[i]);
		}
		Uart2_SendString(buf);
		Uart2_SendString("\r\n");
	}

	for(base = 0u; base < 16u; base = (uint8_t)(base + 8u))
	{
		n  = snprintf(buf, sizeof(buf), "gray blk %2u-%2u:", (unsigned int)base, (unsigned int)(base + 7u));
		for(i = base; i < (uint8_t)(base + 8u); i++)
		{
			n = Serial_AppendUInt(buf, n, (int)sizeof(buf),
			                      (unsigned int)BlackPoint_Finder_IsBlackPoint(i, g_mux_adc_values[i]));
		}
		Uart2_SendString(buf);
		Uart2_SendString("\r\n");
	}

	Serial_Log("gray pos=%d found=%d idx=%u posx10=%d\r\n",
	           (int)position_get,
	           (int)result_BlackPoint.found,
	           (unsigned int)result_BlackPoint.position,
	           (int)(result_BlackPoint.precise_position * 10.0f));

	Uart2_TxUnlock();
}

/*------------------------------------------------------------------------------
 * 速度环标定数据（testl / testr）：目标 / 反馈 / 误差 / P 增量 / I 增量 / 输出
 * 直接读 SpeedPID 里保存的调试快照，10Hz 打印，不干扰 2ms 控制
 *----------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------
 * 推车测距（串口 "mc" 开关）
 *   第一次 mc：清零、开始累计（同时强制 IDLE，保证电机不动），串口每秒打一行
 *   第二次 mc：停止并打印结果（净计数 / 绝对值累计 / 每米计数 / 每计数多少 um）
 *   计数在控制任务里按 2ms 累加（见 App_CtrlTask），这里只读快照。
 *----------------------------------------------------------------------------*/
static void Serial_MeasStart(void)
{
	g_meas_on    = 0u;
	g_meas_net_l = 0;
	g_meas_net_r = 0;
	g_meas_abs_l = 0u;
	g_meas_abs_r = 0u;

	/* 先停车，避免推车过程中车自己跑 */
	App_PostCmd(APP_CMD_MODE, (uint8_t)CAR_MODE_IDLE);

	s_mc_start = xTaskGetTickCount();
	g_meas_on  = 1u;

	Serial_Log("[mc] start, push the car 1m by hand, then send mc again\r\n");
}

static void Serial_MeasStop(void)
{
	int32_t  nl;
	int32_t  nr;
	uint32_t al;
	uint32_t ar;
	uint32_t ms;
	int32_t  avg;
	uint32_t um;

	g_meas_on = 0u;

	nl = g_meas_net_l;
	nr = g_meas_net_r;
	al = g_meas_abs_l;
	ar = g_meas_abs_r;
	ms = (uint32_t)((xTaskGetTickCount() - s_mc_start) * portTICK_PERIOD_MS);

	Serial_Log("[mc] stop  t=%lu ms\r\n", (unsigned long)ms);
	Serial_Log("[mc] L net=%ld abs=%lu | R net=%ld abs=%lu\r\n",
	           (long)nl, (unsigned long)al, (long)nr, (unsigned long)ar);

	if((nl > 0 && nr < 0) || (nl < 0 && nr > 0))
	{
		Serial_Log("[mc] WARN: two wheels net sign differ, one encoder reversed?\r\n");
	}

	/* 净计数按 1000mm 折算；绝对值累计 - 净绝对值 = 来回拖动的量 */
	avg = ((nl >= 0) ? nl : -nl) + ((nr >= 0) ? nr : -nr);
	avg /= 2;

	if(avg > 0)
	{
		um = (uint32_t)(1000000L / (long)avg);
		Serial_Log("[mc] 1m -> L=%ld cnt, R=%ld cnt, avg=%ld cnt/m\r\n",
		           (long)nl, (long)nr, (long)avg);
		Serial_Log("[mc] 1 cnt = %lu um (0.%03lu mm)\r\n",
		           (unsigned long)um, (unsigned long)(um / 1000u));
		Serial_Log("[mc] back/forth: L=%ld R=%ld cnt (0 = 一路推过去)\r\n",
		           (long)(al - (uint32_t)((nl >= 0) ? nl : -nl)),
		           (long)(ar - (uint32_t)((nr >= 0) ? nr : -nr)));
		/* 按标定值（ENC_COUNTS_PER_METER）把本次净计数折成距离，并给出与标定值的偏差 */
		Serial_Log("[mc] dist: L=%ld mm R=%ld mm (cal %ld cnt/m)\r\n",
		           (long)((long)nl * 1000L / (long)ENC_COUNTS_PER_METER),
		           (long)((long)nr * 1000L / (long)ENC_COUNTS_PER_METER),
		           (long)ENC_COUNTS_PER_METER);
		Serial_Log("[mc] vs cal: L=%ld%% R=%ld%%  (100%% = nominal)\r\n",
		           (long)((long)nl * 100L / (long)ENC_COUNTS_PER_METER),
		           (long)((long)nr * 100L / (long)ENC_COUNTS_PER_METER));
	}
	else
	{
		Serial_Log("[mc] no movement counted, encoder ok?\r\n");
	}
}

/* 测量期间每秒一行实时累计 */
static void Serial_MeasPrint(void)
{
	uint32_t sec = (uint32_t)((xTaskGetTickCount() - s_mc_start) * portTICK_PERIOD_MS) / 1000u;

	Serial_Log("[mc] t=%lu s  L=%ld R=%ld\r\n",
	           (unsigned long)sec, (long)g_meas_net_l, (long)g_meas_net_r);
}


static void Serial_SendSpeedTest(void)
{
	SpeedPID_Controller_t *pid;
	int16_t     target = 0;
	Car_Wheel_t wheel  = CAR_WHEEL_LEFT;

	if(Car_Control_GetSpeedTest(&target, &wheel) == 0u)
	{
		return;
	}

	pid = (wheel == CAR_WHEEL_LEFT) ? &g_speed_pid_l : &g_speed_pid_r;

	Uart2_TxLock();
	Serial_Log("[t] T=%d F=%d E=%d P=%+d I=%+d U=%+d\r\n",
	           (int)(int16_t)pid->last_target,
	           (int)(int16_t)pid->last_feedback,
	           (int)(int16_t)(pid->last_target - pid->last_feedback),
	           (int)pid->last_p_term,
	           (int)pid->last_i_term,
	           (int)pid->last_output);
	Uart2_TxUnlock();
}

/*------------------------------------------------------------------------------
 * 'l' / 'r' / 'f' 的十进制参数输入（回车或任意非数字字符结束）
 *----------------------------------------------------------------------------*/
static uint16_t s_num        = 0;   /* 已输入的数值（0~999） */
static uint8_t  s_num_digits = 0;   /* 已输入的数字个数 */
static uint8_t  s_num_cmd    = 0;   /* 0=无；'L'=左轮；'R'=右轮；'F'=风扇；'D'=延时风扇(ff) */
static uint16_t s_frac       = 0;   /* 小数部分（千分位） */
static uint16_t s_frac_div   = 0;   /* 0=整数态；10/100/1000=已收几位小数 */

/* 多字符指令匹配进度：0=空闲；1=已收 'b'；2=已收 'bm'（用于 'bmq'） */
static uint8_t  s_word_state = 0;

/* 灰度实时打印开关（串口 'g' 翻转，serial 任务自己读写，不进指令队列） */
static uint8_t  s_gray_mon   = 0;
/* 多字符指令 "testl" / "testr" 的匹配进度：0=空闲 1=t 2=te 3=tes 4=test */
static uint8_t  s_test_state = 0;

/* 调参模式的多字符指令进度 */
static uint8_t  s_tune_state = 0;   /* 0=空闲 1=tu 2=tun */
static uint8_t  s_hlt_state  = 0;   /* 0=空闲 1=h 2=hl（hlt） */
static uint8_t  s_k_state    = 0;   /* 0=空闲 1=k 2=kp/ki/kd（等 l/r） */
static uint8_t  s_k_term     = 0;   /* 1=kp 2=ki 3=kd（s_k_state=2 时有效） */
static uint8_t  s_v_state    = 0;   /* 0=空闲 1=v（等 l/r） */
static uint8_t  s_ex_state  = 0;   /* 0=空闲 1=等数字 2=数字 3=等侧别 4=已收 a 5=已收 al */
static uint16_t s_ex_ms     = 0;   /* `! <ms>` 里的时长 */
static uint8_t  s_tune_was  = 0;   /* 上一拍的 running，用来打印 [tune] stop */


/*------------------------------------------------------------------------------
 * 调参模式（tune）：打印当前增益、输出一行 CSV
 *   增益内部是 float，这里一律换成 milli 整数打印（microlib 不支持 %f）
 *----------------------------------------------------------------------------*/
static void Serial_SendTuneParams(void)
{
	Car_TuneSnapshot_t s;
	int lp, li, ld, rp, ri, rd;

	Car_Control_TuneSnapshot(&s);
	lp = (int)(g_speed_pid_l.param.kp * 1000.0f + 0.5f);
	li = (int)(g_speed_pid_l.param.ki * 1000.0f + 0.5f);
	ld = (int)(g_speed_pid_l.param.kd * 1000.0f + 0.5f);
	rp = (int)(g_speed_pid_r.param.kp * 1000.0f + 0.5f);
	ri = (int)(g_speed_pid_r.param.ki * 1000.0f + 0.5f);
	rd = (int)(g_speed_pid_r.param.kd * 1000.0f + 0.5f);

	/* 一行一次 Serial_Log：Serial_Log 自己的缓冲只有 96 字节，拼成一大串会被截断 */
	Serial_Log("[tune] L kp=%d.%03d ki=%d.%03d kd=%d.%03d\r\n",
	           lp / 1000, lp % 1000, li / 1000, li % 1000, ld / 1000, ld % 1000);
	Serial_Log("[tune] R kp=%d.%03d ki=%d.%03d kd=%d.%03d\r\n",
	           rp / 1000, rp % 1000, ri / 1000, ri % 1000, rd / 1000, rd % 1000);
	Serial_Log("[tune] tgt L=%d R=%d mm/s run=%u limit=%d fan=%u/1000 ff=%u.%03u\r\n",
	           (int)s.target_l, (int)s.target_r, (unsigned int)s.running,
	           (int)g_speed_pid_l.param.output_max, (unsigned int)s.fan_duty,
	           (unsigned int)(s.ff_milli / 1000u), (unsigned int)(s.ff_milli % 1000u));
}
/* `! <ms> <l|r|all>` 收齐后：忙就提示，空闲就下发定时闭环 */
static void Serial_TuneRunStart(uint8_t side)
{
	Car_TuneSnapshot_t sn;
	const char *name;

	name = (side == 0u) ? "L" : ((side == 1u) ? "R" : "ALL");
	Car_Control_TuneSnapshot(&sn);
	if((sn.running != 0u) || (sn.arming != 0u))
	{
		Serial_Log("[tune] busy: still running (send hlt to abort)\r\n");
		return;
	}
	App_PostCmdArg(APP_CMD_TUNE_RUN, side, (int16_t)s_ex_ms);
	if(sn.fan_duty > 0u)
	{
		Serial_Log("[tune] run %u ms %s + fan %u/1000 (arm 1.5s)\r\n",
		           (unsigned int)s_ex_ms, name, (unsigned int)sn.fan_duty);
	}
	else
	{
		Serial_Log("[tune] run %u ms %s\r\n", (unsigned int)s_ex_ms, name);
	}
}


/* 每 100ms 一行 CSV：ms,tL,fL,uL,tR,fR,uR（时间用 1ms tick，即毫秒） */
static void Serial_SendTune(void)
{
	static char buf[96];
	Car_TuneSnapshot_t s;

	Car_Control_TuneSnapshot(&s);
	(void)snprintf(buf, sizeof(buf), "%lu,%d,%d,%d,%d,%d,%d\r\n",
		(unsigned long)xTaskGetTickCount(),
		(int)s.target_l, (int)s.feedback_l, (int)s.output_l,
		(int)s.target_r, (int)s.feedback_r, (int)s.output_r);
	Serial_Log("%s", buf);
}

static void Serial_CommitNumber(void)
{
	uint16_t pct = s_num;
	uint8_t  cmd = s_num_cmd;

	s_num_cmd = 0;

	/* 延时风扇测试（ff）：先等 5 秒，再按给定占空比吹 5 秒，随后自动停 */
	if(cmd == 'D')
	{
		if(s_num_digits == 0u)
		{
			Serial_Log("fan+delay: send ff0~ff100 (e.g. ff50)\r\n");
			return;
		}
		if(pct > 100u)
		{
			pct = 100u;
		}
		App_PostCmd(APP_CMD_FAN_DELAY_TEST, (uint8_t)pct);
		Serial_Log("fan %u%% in 5s, run 5s\r\n", (unsigned int)pct);
		return;
	}

	/* 风扇测试：必须带数字，f 回车只提示格式 */
	if(cmd == 'F')
	{
		if(s_num_digits == 0u)
		{
			Serial_Log("fan: send f0~f100 (e.g. f50)\r\n");
			return;
		}
		if(pct > 100u)
		{
			pct = 100u;
		}
		App_PostCmd(APP_CMD_FAN_TEST, (uint8_t)pct);
		Serial_Log("fan %u%% 5s\r\n", (unsigned int)pct);
		return;
	}

	/* 调参模式的负压（tf）：只记设定，等下一次 `!` 实验才解锁并吹 */
	if(cmd == 'P')
	{
		if(s_num_digits == 0u)
		{
			Serial_Log("tune fan: send tf0~tf100 (e.g. tf40), 0 = no fan\r\n");
			return;
		}
		if(pct > 100u)
		{
			pct = 100u;
		}
		App_PostCmd(APP_CMD_TUNE_FAN, (uint8_t)pct);
		if(pct == 0u)
		{
			Serial_Log("[tune] fan off: next ! runs without vacuum\r\n");
		}
		else
		{
			Serial_Log("[tune] fan %u%%: next ! arms 1.5s then runs with vacuum\r\n",
			           (unsigned int)pct);
		}
		return;
	}

	/* 速度环标定：testl0~testl400 / testr0~testr400，单位是编码器每 2ms 计数 */
	if(cmd == 'T' || cmd == 'U')
	{
		if(s_num_digits == 0u)
		{
			Serial_Log("speed: send testl0~testl400 or testr0~testr400\r\n");
			return;
		}
		App_PostCmdArg((cmd == 'T') ? APP_CMD_SPEED_TEST_L : APP_CMD_SPEED_TEST_R,
		               0, (int16_t)pct);
		Serial_Log("%s speed test %u (10s)\r\n",
		           (cmd == 'T') ? "left" : "right", (unsigned int)pct);
		return;
	}

	/* 调参模式的目标阶跃前馈（kff；cmd = 'K'）：千分比，支持小数 */
	if(cmd == 'K')
	{
		uint32_t milli = (uint32_t)s_num * 1000u + (uint32_t)s_frac;

		if(s_num_digits == 0u)
		{
			Serial_Log("ff: kff0.6 = 0.6 duty/(mm/s), kff0 = off\r\n");
			return;
		}
		if(milli > 2000u)
		{
			milli = 2000u;
		}
		App_PostCmdArg(APP_CMD_TUNE_FF, 0, (int16_t)milli);
		Serial_Log("[tune] ff=%d.%03d duty/(mm/s)%s\r\n", (int)(milli / 1000u), (int)(milli % 1000u),
		           (milli == 0u) ? " (off)" : "");
		return;
	}

	/* 调参模式的增益（kpl/kpr 等）：cmd = 'a'..'f'，支持小数 */
	if((cmd >= 'a') && (cmd <= 'f'))
	{
		uint32_t milli  = (uint32_t)s_num * 1000u + (uint32_t)s_frac;
		uint8_t  idx    = (uint8_t)(cmd - 'a');
		uint8_t  wheel  = (uint8_t)(idx / 3u);
		uint8_t  term   = (uint8_t)(idx % 3u);
		static const char *s_term[3] = { "kp", "ki", "kd" };

		if(s_num_digits == 0u)
		{
			Serial_Log("gain: kpl0.6 kil0.5 kdl0.05 kpr0.6 kir0.55 kdr0.06\r\n");
			return;
		}
		if(milli > 32767u)
		{
			milli = 32767u;
		}
		App_PostCmdArg(APP_CMD_TUNE_GAIN, (uint8_t)(wheel * 3u + term), (int16_t)milli);
		Serial_Log("[tune] %s %s=%d.%03d\r\n", (wheel != 0u) ? "R" : "L", s_term[term],
		           (int)(milli / 1000u), (int)(milli % 1000u));
		return;
	}

	/* 调参模式的目标速度（vl / vr，单位 mm/s） */
	if((cmd == 'v') || (cmd == 'w'))
	{
		uint16_t mm = s_num;

		if(s_num_digits == 0u)
		{
			Serial_Log("target: vl<0~4000> or vr<0~4000> (mm/s)\r\n");
			return;
		}
		if(mm > CAR_TUNE_MAX)
		{
			mm = CAR_TUNE_MAX;
		}
		App_PostCmdArg(APP_CMD_TUNE_TARGET, (cmd == 'v') ? 0u : 1u, (int16_t)mm);
		Serial_Log("[tune] %s target %u mm/s\r\n", (cmd == 'v') ? "L" : "R", (unsigned int)mm);
		return;
	}

	/* 单轮测试：l/r 必须带数字，只发 l 回车只提示格式 */
	if(cmd == 'L' || cmd == 'R')
	{
		if(s_num_digits == 0u)
		{
			Serial_Log("wheel: send l0~l100 or r0~r100\r\n");
			return;
		}
		if(pct > 100u)
		{
			pct = 100u;
		}
		App_PostCmd((cmd == 'L') ? APP_CMD_WHEEL_LEFT : APP_CMD_WHEEL_RIGHT, (uint8_t)pct);
		Serial_Log("%s %u%% 2s\r\n", (cmd == 'L') ? "left" : "right", (unsigned int)pct);
	}
}

/*------------------------------------------------------------------------------
 * 单字节指令解析
 *----------------------------------------------------------------------------*/
static void Serial_HandleByte(uint8_t ch)
{

	/* 数字输入态：只收数字，其它字符用来结束输入 */
	if(s_num_cmd != 0u)
	{
		if(ch >= '0' && ch <= '9')
		{
			if(s_frac_div == 0u)
			{
				if(s_num <= 9999u)
				{
					s_num = (uint16_t)(s_num * 10u + (uint16_t)(ch - '0'));
				}
			}
			else if(s_frac_div <= 1000u)
			{
				/* 小数部分按千分位累加（最多 3 位，多余的忽略） */
				s_frac = (uint16_t)(s_frac + (uint16_t)((uint16_t)(ch - '0') * (1000u / s_frac_div)));
				if(s_frac_div < 1000u)
				{
					s_frac_div = (uint16_t)(s_frac_div * 10u);
				}
			}
			s_num_digits++;
		}
		else if((ch == '.') && (s_frac_div == 0u))
		{
			/* 小数点：之后按千分位收小数，支持 kpl0.60 这种写法 */
			s_frac_div = 10u;
		}
		else if(((ch == ' ') || (ch == '\t')) && (s_num_digits == 0u))
		{
			/* 允许 "ff 50" / "f 50" / "l 50" 这种数字前带空格的写法：
			 * 还没输入数字时空格只是分隔符，不当结束符（否则会白提交一次提示） */
		}
		else if((ch == 'f' || ch == 'F') && (s_num_cmd == 'F') && (s_num_digits == 0u))
		{
			/* 'ff'：数字态里再收到一个 f，表示"延时 5 秒再吹"，不是新指令 */
			s_num = 0;
			s_num_digits = 0;
			s_frac = 0;
			s_frac_div = 0;
			s_num_cmd = 'D';
		}
		else
		{
			Serial_CommitNumber();
		}
		return;
	}

	/* 多字符指令 "bmq"：切换编码器实时监视（不区分大小写）。
	 * 匹配错位的字符不吞掉，继续按单字符指令解析，避免打错字后"没反应"。 */
	if(s_word_state == 0u)
	{
		if((ch == 'b') || (ch == 'B'))
		{
			s_word_state = 1;
			return;
		}
	}
	else if(s_word_state == 1u)
	{
		if((ch == 'm') || (ch == 'M'))
		{
			s_word_state = 2;
			return;
		}
		s_word_state = 0;
	}
	else
	{
		if((ch == 'q') || (ch == 'Q'))
		{
			s_word_state = 0;
			App_PostCmd(APP_CMD_ENC_MON, 0);
			return;
		}
		s_word_state = 0;
	}

	/* 多字符指令 "mc"：推车测距开关（不区分大小写） */
	if(s_mc_state == 0u)
	{
		if((ch == 'm') || (ch == 'M'))
		{
			s_mc_state = 1;
			return;
		}
	}
	else
	{
		s_mc_state = 0;
		if((ch == 'c') || (ch == 'C'))
		{
			if(g_meas_on != 0u)
			{
				Serial_MeasStop();
			}
			else
			{
				Serial_MeasStart();
			}
			return;
		}
	}

	/* 多字符指令 "hlt"：调参模式里临时停车（目标清零、留在模式内）。
	 * 必须放在 test 匹配器之前：hlt 的最后一个 't' 不能被 "testl" 的 't' 抢走。 */
	if(s_hlt_state == 0u)
	{
		if((ch == 'h') || (ch == 'H'))
		{
			s_hlt_state = 1;
			return;
		}
	}
	else if(s_hlt_state == 1u)
	{
		if((ch == 'l') || (ch == 'L'))
		{
			s_hlt_state = 2;
			return;
		}
		s_hlt_state = 0;
	}
	else
	{
		s_hlt_state = 0;
		if((ch == 't') || (ch == 'T'))
		{
			App_PostCmd(APP_CMD_TUNE_HALT, 0);
			Serial_Log("[tune] halt\r\n");
			return;
		}
	}

	/* 多字符指令 "testl" / "testr"：速度环标定。匹配成立后转入数字输入态，用 s_num 收目标值；
	 * 匹配失败不吞字符，继续按单字符指令解析。 */
	if(s_test_state == 0u)
	{
		if((ch == 't') || (ch == 'T'))
		{
			s_test_state = 1;
			return;
		}
	}
	else if(s_test_state == 1u)
	{
			if((ch == 'u') || (ch == 'U'))
			{
				s_test_state = 0;
				s_tune_state = 1;
				return;
			}
		if((ch == 'e') || (ch == 'E'))
		{
			s_test_state = 2;
			return;
		}
		/* "tf"：调参模式的负压加持（和 f 一样按百分比，但立刻生效的是下一次 !） */
		if((ch == 'f') || (ch == 'F'))
		{
			s_test_state = 0;
			s_num = 0;
			s_num_digits = 0;
			s_frac = 0;
			s_frac_div = 0;
			s_num_cmd = 'P';
			return;
		}
		s_test_state = 0;
	}
	else if(s_test_state == 2u)
	{
		if((ch == 's') || (ch == 'S'))
		{
			s_test_state = 3;
			return;
		}
		s_test_state = 0;
	}
	else if(s_test_state == 3u)
	{
		if((ch == 't') || (ch == 'T'))
		{
			s_test_state = 4;
			return;
		}
		s_test_state = 0;
	}
	else
	{
		s_test_state = 0;
		if((ch == 'l') || (ch == 'L'))
		{
			s_num = 0;
			s_num_digits = 0;
			s_frac = 0;
			s_frac_div = 0;
			s_num_cmd = 'T';
			return;
		}
		if((ch == 'r') || (ch == 'R'))
		{
			s_num = 0;
			s_num_digits = 0;
			s_frac = 0;
			s_frac_div = 0;
			s_num_cmd = 'U';
			return;
		}
	}

	/* 多字符指令 "tune"：进入速度环调参模式（'tu' 由上面的 test 匹配器接管） */
	if(s_tune_state == 1u)
	{
		if((ch == 'n') || (ch == 'N'))
		{
			s_tune_state = 2;
			return;
		}
		s_tune_state = 0;
	}
	else if(s_tune_state == 2u)
	{
		s_tune_state = 0;
		if((ch == 'e') || (ch == 'E'))
		{
			App_PostCmd(APP_CMD_TUNE_ENTER, 0);
			Serial_Log("[tune] enter: ! <ms> l|r|all | hlt | vl/vr<mm/s> | kpl0.6 | kq | 0 exit\r\n");
			Serial_Log("[tune] tf0~100: vacuum for the next ! (0 = no fan)\r\n");
			Serial_Log("[tune] kff0.6: target-step feedforward, duty per mm/s (0 = off)\r\n");
			return;
		}
	}

	/* 多字符指令 "kq" / "kpl0.6" / "kdl0.05" / "kpr0.6"...：
	 * k + 增益项(p/i/d) + 轮(l/r)，之后进数字态收小数；kq 只打印当前参数。 */
	if(s_k_state == 0u)
	{
		if((ch == 'k') || (ch == 'K'))
		{
			s_k_state = 1;
			return;
		}
	}
	else if(s_k_state == 1u)
	{
		if((ch == 'q') || (ch == 'Q'))
		{
			s_k_state = 0;
			Serial_SendTuneParams();
			return;
		}
		if((ch == 'p') || (ch == 'P'))
		{
			s_k_term  = 1u;
			s_k_state = 2;
			return;
		}
		if((ch == 'i') || (ch == 'I'))
		{
			s_k_term  = 2u;
			s_k_state = 2;
			return;
		}
		if((ch == 'd') || (ch == 'D'))
		{
			s_k_term  = 3u;
			s_k_state = 2;
			return;
		}
		/* kff：目标阶跃前馈系数（kff0.6 = 0.6 占空比/(mm/s)） */
		if((ch == 'f') || (ch == 'F'))
		{
			s_k_state = 3;
			return;
		}
		s_k_state = 0;
	}
	else if(s_k_state == 3u)
	{
		s_k_state = 0;
		if((ch == 'f') || (ch == 'F'))
		{
			s_num = 0;
			s_num_digits = 0;
			s_frac = 0;
			s_frac_div = 0;
			s_num_cmd = 'K';
		}
		return;
	}
	else
	{
		uint8_t base = (uint8_t)('a' + (s_k_term - 1u));

		s_k_state = 0;
		if((ch == 'l') || (ch == 'L'))
		{
			s_num = 0;
			s_num_digits = 0;
			s_frac = 0;
			s_frac_div = 0;
			s_num_cmd = base;
			return;
		}
		if((ch == 'r') || (ch == 'R'))
		{
			s_num = 0;
			s_num_digits = 0;
			s_frac = 0;
			s_frac_div = 0;
			s_num_cmd = (uint8_t)(base + 3u);
			return;
		}
	}

	/* 多字符指令 "vl" / "vr"：设置左右目标速度（mm/s） */
	if(s_v_state == 0u)
	{
		if((ch == 'v') || (ch == 'V'))
		{
			s_v_state = 1;
			return;
		}
	}
	else
	{
		s_v_state = 0;
		if((ch == 'l') || (ch == 'L'))
		{
			s_num = 0;
			s_num_digits = 0;
			s_frac = 0;
			s_frac_div = 0;
			s_num_cmd = 'v';
			return;
		}
		if((ch == 'r') || (ch == 'R'))
		{
			s_num = 0;
			s_num_digits = 0;
			s_frac = 0;
			s_frac_div = 0;
			s_num_cmd = 'w';
			return;
		}
	}

	/* 多字符指令 "! <ms> <l|r|all>"：定时闭环。
	 *   ! 1000 l  左轮；! 1000 r  右轮；! 500 all  两侧一起
	 *   到点自动停车（留在模式内） */
	if(s_ex_state == 0u)
	{
		if(ch == '!')
		{
			s_ex_state = 1u;
			s_ex_ms    = 0;
			return;
		}
	}
	else if(s_ex_state == 1u)
	{
		if((ch >= '0') && (ch <= '9'))
		{
			s_ex_ms    = (uint16_t)(ch - '0');
			s_ex_state = 2u;
			return;
		}
		if((ch == ' ') || (ch == '\t'))
		{
			return;
		}
		s_ex_state = 0u;
		Serial_Log("use: ! <ms> <l|r|all>   e.g. ! 1000 l\r\n");
		return;
	}
	else if(s_ex_state == 2u)
	{
		if((ch >= '0') && (ch <= '9'))
		{
			if(s_ex_ms <= 3000u)          /* 最多 5 位，防止 uint16 溢出 */
			{
				s_ex_ms = (uint16_t)(s_ex_ms * 10u + (uint16_t)(ch - '0'));
			}
			return;
		}
		if((ch == ' ') || (ch == '\t'))
		{
			s_ex_state = 3u;
			return;
		}
		s_ex_state = 0u;
		Serial_Log("use: ! <ms> <l|r|all>   e.g. ! 1000 l\r\n");
		return;
	}
	else if(s_ex_state == 3u)
	{
		if((ch == ' ') || (ch == '\t'))
		{
			return;
		}
		if((ch == 'l') || (ch == 'L'))
		{
			s_ex_state = 0u;
			Serial_TuneRunStart((uint8_t)CAR_TUNE_SIDE_L);
			return;
		}
		if((ch == 'r') || (ch == 'R'))
		{
			s_ex_state = 0u;
			Serial_TuneRunStart((uint8_t)CAR_TUNE_SIDE_R);
			return;
		}
		if((ch == 'a') || (ch == 'A'))
		{
			s_ex_state = 4u;
			return;
		}
		s_ex_state = 0u;
		Serial_Log("use: ! <ms> <l|r|all>   e.g. ! 500 all\r\n");
		return;
	}
	else
	{
		/* state 4：已经收了 a，等 l 凑成 all */
		s_ex_state = 0u;
		if((ch == 'l') || (ch == 'L'))
		{
			Serial_TuneRunStart((uint8_t)CAR_TUNE_SIDE_ALL);
			return;
		}
		Serial_Log("use: ! <ms> <l|r|all>   e.g. ! 500 all\r\n");
		return;
	}

	switch(ch)
	{
		case '0':
			App_PostCmd(APP_CMD_MODE, (uint8_t)CAR_MODE_IDLE);
			break;

		case '1':
			App_PostCmd(APP_CMD_MODE, (uint8_t)CAR_MODE_TRACK);
			break;

		case '2':
			App_PostCmd(APP_CMD_MODE, (uint8_t)CAR_MODE_MOTOR_TEST);
			break;

		case '3':
			App_PostCmd(APP_CMD_LED_FORCE_ON, 0);
			break;

		case '4':
			App_PostCmd(APP_CMD_LED_FORCE_OFF, 0);
			break;

		case '5':
			/* 引脚/电平诊断：多行 printf，整段加锁避免被波形帧打断 */
			Uart2_TxLock();
			Car_Control_PrintLedDiag();
			Uart2_TxUnlock();
			break;

		case 'p':
		case 'P':
			/* 风机 PWM 诊断：TIM2 的 PSC/ARR/CCR4/CNT + PB11 复用配置 + 重映射位 */
			Uart2_TxLock();
			M3PWM_PrintDiag();
			Uart2_TxUnlock();
			break;

		case 'w':
		case 'W':
			App_PostCmd(APP_CMD_WAVE_TOGGLE, 0);
			break;

		case '?':
			Serial_SendStatus();
			break;

		case 's':
		case 'S':
			/* 一键停止：与按键 K2 同一条路径，都是投递 IDLE 让控制任务去停车 */
			App_PostCmd(APP_CMD_MODE, (uint8_t)CAR_MODE_IDLE);
			Serial_Log("[stop] send IDLE\r\n");
			break;

		case 'z':
		case 'Z':
			/* 栈水位/剩余堆（原来挂在 's' 上，'s' 改成停车后挪到这里） */
			Serial_SendStacks();
			break;

		case 'g':
		case 'G':
			/* 灰度实时打印开关：发一次打开、再发一次关闭。
			 * 只翻标志位，实际打印在串口任务主循环里按 APP_GRAY_MON_PERIOD_MS 定时做，
			 * 保证不会因为"一次打一大段"阻塞指令解析。只读共享数据，不动电机/模式。 */
			s_gray_mon = (uint8_t)((s_gray_mon == 0u) ? 1u : 0u);
			break;

		case 'l':
		case 'L':
			s_num = 0;
			s_num_digits = 0;
			s_frac = 0;
			s_frac_div = 0;
			s_num_cmd = 'L';
			break;

		case 'r':
		case 'R':
			s_num = 0;
			s_num_digits = 0;
			s_frac = 0;
			s_frac_div = 0;
			s_num_cmd = 'R';
			break;

		case 'f':
		case 'F':
			s_num = 0;
			s_num_digits = 0;
			s_frac = 0;
			s_frac_div = 0;
			s_num_cmd = 'F';
			break;

		default:
			/* 回车/换行/空格等一律忽略 */
			break;
	}
}

/*------------------------------------------------------------------------------
 * 串口任务
 *----------------------------------------------------------------------------*/
void App_SerialTask(void *argument)
{
	TickType_t last_wave = xTaskGetTickCount();
	TickType_t last_test = 0;
	TickType_t last_mc   = 0;
	TickType_t last_enc  = 0;
	TickType_t last_gray = 0;
	TickType_t last_tune = 0;
	uint8_t    enc_was   = 0;
	uint8_t    gray_was  = 0;
	(void)argument;

	/* 告诉 USART2 中断：收到字节后该唤醒哪个任务 */
	Uart2_SetRxNotifyTask((void *)xTaskGetCurrentTaskHandle());

	for(;;)
	{
		/* 无字节时最多等 20ms，超时后发一帧波形 */
		(void)ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(APP_WAVE_PERIOD_MS));

		/* 把环形缓冲里的字节一次取空 */
		while(Uart2_BytesAvailable() > 0)
		{
			Serial_HandleByte(Uart2_ReadByteBlocking());
		}

		/* 编码器监视（'bmq' 开关）：状态一变就打一行提示，然后按周期实时打印 */
		if(g_enc_mon != enc_was)
		{
			enc_was  = g_enc_mon;
			last_enc = xTaskGetTickCount();
			if(enc_was != 0u)
			{
				Serial_Log("[enc] ON  L/R=cnt vL/vR=spd T=total  (bmq to exit)\r\n");
			}
			else
			{
				Serial_Log("[enc] OFF\r\n");
			}
		}

		if(g_enc_mon != 0u)
		{
			TickType_t now = xTaskGetTickCount();
			if((TickType_t)(now - last_enc) >= pdMS_TO_TICKS(APP_ENC_MON_PERIOD_MS))
			{
				last_enc = now;
				/* 左轮 = TIM4(ENC2)，右轮 = TIM3(ENC1)，与 ABEncoder_UpdateSpeed 的映射一致 */
				Serial_Log("enc L=%d R=%d | vL=%d vR=%d mm/s | T=%ld/%ld\r\n",
				           (int)(int16_t)TIM_GetCounter(ENC2_TIM),
				           (int)(int16_t)TIM_GetCounter(ENC1_TIM),
	           (int)speed_left_mm_s,
	           (int)speed_right_mm_s,
				           (long)left_ecoder_cnt,
				           (long)right_ecoder_cnt);
			}
		}

		/* 灰度监视（'g' 开关）：开关状态一变就打一行提示，打开时先立刻打一帧，
		 * 之后按 APP_GRAY_MON_PERIOD_MS 周期性实时打印。 */
		if(s_gray_mon != gray_was)
		{
			gray_was  = s_gray_mon;
			last_gray = xTaskGetTickCount();
			if(gray_was != 0u)
			{
				Serial_Log("[gray] ON  raw=ADC(black small) blk=0/1  pos=line  (g to exit)\r\n");
				Serial_SendGray();
			}
			else
			{
				Serial_Log("[gray] OFF\r\n");
			}
		}

		if(s_gray_mon != 0u)
		{
			TickType_t now = xTaskGetTickCount();
			if((TickType_t)(now - last_gray) >= pdMS_TO_TICKS(APP_GRAY_MON_PERIOD_MS))
			{
				last_gray = now;
				Serial_SendGray();
			}
		}

		/* 速度环标定（testl / testr）：按 APP_SPEED_TEST_PERIOD_MS 打印 PID 数据 */
		if(Car_Control_GetMode() == CAR_MODE_SPEED_TEST)
		{
			TickType_t now = xTaskGetTickCount();
			if((TickType_t)(now - last_test) >= pdMS_TO_TICKS(APP_SPEED_TEST_PERIOD_MS))
			{
				last_test = now;
				Serial_SendSpeedTest();
			}
		}

		/* 速度环调参（tune）：闭环运行中每 APP_TUNE_PERIOD_MS 打一行 CSV：
		 * ms,tL,fL,uL,tR,fR,uR（目标/反馈 mm/s，输出占空比） */
		if(Car_Control_GetMode() == CAR_MODE_SPEED_TUNE)
		{
			Car_TuneSnapshot_t sn;
			Car_Control_TuneSnapshot(&sn);
			if(sn.running != 0u)
			{
				TickType_t now = xTaskGetTickCount();
				if((TickType_t)(now - last_tune) >= pdMS_TO_TICKS(APP_TUNE_PERIOD_MS))
				{
					last_tune = now;
					Serial_SendTune();
				}
			}
			else
			{
				if(s_tune_was != 0u)
				{
					Serial_Log("[tune] stop\r\n");
				}
				last_tune = xTaskGetTickCount();
			}
			s_tune_was = sn.running;
		}


		/* 推车测距（mc）：每秒打一行累计 */
		if(g_meas_on != 0u)
		{
			TickType_t now = xTaskGetTickCount();
			if((TickType_t)(now - last_mc) >= pdMS_TO_TICKS(1000))
			{
				last_mc = now;
				Serial_MeasPrint();
			}
		}

		/* 波形帧（模式下 50Hz；指令处理优先，不会被拖慢）。
		 * 编码器/灰度/朝向监视期间不发二进制帧，否则文本行和 JustFloat 帧会互相打断。 */
		if((g_wave_enable != 0u) && (g_enc_mon == 0u) && (s_gray_mon == 0u) &&
		   (Car_Control_GetMode() != CAR_MODE_SPEED_TEST) && (g_meas_on == 0u) &&
		   (Car_Control_GetMode() != CAR_MODE_SPEED_TUNE))
		{
			TickType_t now = xTaskGetTickCount();
			if((TickType_t)(now - last_wave) >= pdMS_TO_TICKS(APP_WAVE_PERIOD_MS))
			{
				last_wave = now;
				Serial_SendWave();
			}
		}
		else
		{
			last_wave = xTaskGetTickCount();
		}
	}
}
