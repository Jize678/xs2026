/**
  ******************************************************************************
  * @file    Car_Control.h
  * @brief   运行模式控制（串口单字符指令）
  *
  * 上电默认：IDLE —— 什么也不跑（电机禁用、风扇停、PID 不动作）
  *
  * 串口指令（USART2，115200，发送单个字符即可，回车换行会被忽略）：
  *   '0' -> CAR_MODE_IDLE        停止运行：禁电机、风扇停、闭环复位
  *   '1' -> CAR_MODE_TRACK       发车：现有的闭环循迹（风扇 95% + 使能电机 + 位置环/速度环）
  *   '2' -> CAR_MODE_MOTOR_TEST  电机测试：左右电机 30% 占空比，
  *                               先向前 2 秒，再停车 2 秒，然后自动回到待机(0)
  *
  * 说明：
  *   1. 串口指令由 App\app_serial.c 的 serial 任务解析，投递到指令队列；
  *   2. 控制任务（App\app_ctrl.c，2ms）取指令并调用 Car_Control_Update() 执行当前模式；
  *   3. 原来的“0.5s 收不到串口数据就自动停车”的保护已经取消
  *      （否则发完 '1' 就会立刻被停掉），停车请发 '0'。
  ******************************************************************************
  */

#ifndef __CAR_CONTROL_H
#define __CAR_CONTROL_H

#include "stm32f10x.h"

/* 串口指令字符 */
#define CAR_CMD_STOP          '0'     /* 停止 */
#define CAR_CMD_TRACK         '1'     /* 闭环循迹发车 */
#define CAR_CMD_TEST          '2'     /* 电机测试 */
#define CAR_CMD_LED_ON        '3'     /* 调试：两颗指示灯常亮 */
#define CAR_CMD_LED_OFF       '4'     /* 调试：两颗指示灯常灭 */
#define CAR_CMD_LED_DIAG      '5'     /* 调试：串口回发引脚状态 */

/* 电机测试占空比：30%（Motor_SetSpeed 量程 0~10000） */
#define CAR_TEST_DUTY         3000

/* 电机测试时序：向前 2 秒 -> 停车 2 秒 -> 自动回到待机 IDLE
 * （驱动是两路半桥，只能前进，所以没有"反转"那一段） */
#define CAR_TEST_FWD_MS       2000
#define CAR_TEST_STOP_MS      2000
#define CAR_TEST_TIME_MS      (CAR_TEST_FWD_MS + CAR_TEST_STOP_MS)

/* 单轮测试（串口 'l' / 'r' 指令）持续时间：到点自动停车、回待机 */
#define CAR_WHEEL_TEST_MS     2000

/* 速度环标定（串口 'testl<0~400>' / 'testr<0~400>'）：
 * 目标速度单位 = 编码器每 2ms 的计数增量（与 SpeedPID_Calculate 的反馈同量纲，
 * 满速约 230）。只闭环一侧轮子，另一侧 0 占空比、风扇不起压；
 * 到时自动停车，防止长时间空转转烧电机。 */
#define CAR_SPEED_TEST_MS     10000    /* 速度环标定持续时间（ms） */
#define CAR_SPEED_TEST_MAX    4000     /* 标定目标上限（mm/s），≈ 2× 直道基准 2110 */

/* 风扇测试（串口 'f' 指令）持续时间：到点自动停风扇、回待机 */
#define CAR_FAN_TEST_MS       5000

/* 风扇测试前先压最低油门这么久，等电调"嘀—嘀"解锁（不算在 CAR_FAN_TEST_MS 里） */
#define CAR_FAN_ARM_MS        1500

/* 延时风扇测试（串口 'ff' 指令）：先空等 CAR_FAN_DELAY_MS，再用 CAR_FAN_TEST_MS 吹风 */
#define CAR_FAN_DELAY_MS      5000

/* 速度环调参模式（串口 tune）：进入只切模式，电机不动、风扇不转；
 * 发 ! 才开始闭环，hlt 临时停车（目标清零、但留在模式里），0 退出。
 * 目标单位 mm/s；增益用 SpeedPID_SetParam() 在线改，掉电不保存。 */
#define CAR_TUNE_MAX          4000     /* 目标速度上限（mm/s） */
#define CAR_TUNE_PRINT_MS     100      /* CSV 打印周期（ms） */
#define CAR_TUNE_GAIN_MAX     32.0f    /* 单个增益上限（串口传输上限 32.767） */
#define CAR_TUNE_RUN_MS_MAX   30000    /* `! <ms> <侧>` 的定时上限（ms） */
/* 目标软起动：`!` 开始后，闭环用的目标速度不是一步跳到 vl/vr，而是用这段
 * 时间从 0 线性爬到设定值（斜坡只作用在“目标”上，PID 本身不变）。
 * 加它的原因（2026-10-31 实测）：轮子静止时静摩擦要 u≈1000~1160 才破得动，
 * 而 500 mm/s 稳态只要 u≈320；阶跃起跑时积分在轮子还没转起来之前就把占空比
 * 堆到 24.5%，一破静摩擦就冲到 1200~2000 mm/s，② 低超调指标（≤10%）必挂。
 * 150ms 是“够平顺”和“① 300ms 内到位”之间的折中，改大改小都要重测 tset。 */
#define CAR_TUNE_RAMP_MS      150

/* Car_Control_Update() 的调用周期，等于控制任务周期（2ms） */
#define CAR_CTRL_TICK_MS      2

/* 发车时负压风扇占空比：95%（M3PWM_SetDutyCycle 量程 0~1000） */
#define CAR_TRACK_FAN_DUTY    950

/* 串口回显（调试用）：置 1 时每收到一条有效指令会回发 "mode=x"，
 * 用 VOFA+ 等上位机画图时请保持 0，避免多出文本干扰数据帧。 */
#define CAR_CTRL_ECHO         0

typedef enum
{
	CAR_MODE_IDLE = 0,      /* 上电默认：待机，什么都不跑 */
	CAR_MODE_TRACK,         /* 闭环循迹 */
	CAR_MODE_MOTOR_TEST,    /* 电机测试：双电机 30% 前进 2s -> 停车 2s -> 回待机 */
	CAR_MODE_SPEED_TEST,    /* 速度环标定：单轮按目标速度闭环（串口 testl/testr） */
	CAR_MODE_SPEED_TUNE     /* 速度环调参：左右各一目标、增益可在线改（串口 tune） */
} Car_Mode_t;

/* 单轮测试用哪一侧：串口 'l' = 左轮，'r' = 右轮 */
typedef enum
{
	CAR_WHEEL_LEFT  = 0,
	CAR_WHEEL_RIGHT = 1
} Car_Wheel_t;

/* 调参用哪一项增益 */
typedef enum
{
	CAR_TUNE_KP = 0,
	CAR_TUNE_KI = 1,
	CAR_TUNE_KD = 2
} Car_TuneTerm_t;

/* `! <ms> <侧>` 的侧别：只跑一侧时另一侧目标强制 0 */
typedef enum
{
	CAR_TUNE_SIDE_L   = 0,
	CAR_TUNE_SIDE_R   = 1,
	CAR_TUNE_SIDE_ALL = 2
} Car_TuneSide_t;

/* 调参模式快照（串口任务打印 CSV 用，只读） */
typedef struct
{
	uint8_t running;        /* 1 = 正在闭环 */
	uint8_t side;           /* 本次 ! 的侧别（Car_TuneSide_t） */
	uint16_t run_ms;        /* 本次 ! 的定时（ms），停止后保留显示 */
	uint16_t fan_duty;      /* tf 设定的负压占空比（0~1000），0 = 不带负压 */
	uint8_t  arming;        /* 1 = 正在等电调解锁（1.5s），闭环还没开始 */
	int16_t target_l;       /* 左轮目标 mm/s */
	int16_t target_r;       /* 右轮目标 mm/s */
	int16_t feedback_l;     /* 左轮反馈 mm/s */
	int16_t feedback_r;     /* 右轮反馈 mm/s */
	int16_t output_l;       /* 左轮占空比输出（0~10000） */
	int16_t output_r;       /* 右轮占空比输出（0~10000） */
} Car_TuneSnapshot_t;

void       Car_Control_Init(void);                  /* 上电调用：进入待机 */
void       Car_Control_Update(void);                /* 2ms 控制任务调用：执行当前模式 */
void       Car_Control_ApplyMode(Car_Mode_t mode);  /* 手动切换模式 */
Car_Mode_t Car_Control_GetMode(void);               /* 读取当前模式 */
const char *Car_Control_GetModeName(void);          /* 模式名（定宽 5 字符，给 OLED 用） */
void        Car_Control_PrintLedDiag(void);         /* 串口打印 PA15/PB3 的配置与电平 */
void        Car_Control_WheelRun(Car_Wheel_t wheel, uint16_t duty);
                                                    /* 单轮测试：只转一侧，CAR_WHEEL_TEST_MS 后停 */
void        Car_Control_FanTest(uint16_t duty);     /* 风扇测试：负压风扇跑 CAR_FAN_TEST_MS 毫秒后停 */
void        Car_Control_FanDelayedTest(uint16_t duty);
void        Car_Control_SpeedTest(Car_Wheel_t wheel, int16_t target);
uint8_t     Car_Control_GetSpeedTest(int16_t *target, Car_Wheel_t *wheel);
                                                    /* 正在标定则返回 1，并回填目标速度/轮侧 */

void        Car_Control_TuneEnter(void);                          /* tune：进入调参模式（停车） */
void        Car_Control_TuneRun(uint8_t side, uint16_t ms);
                                                                  /* ! <ms> <l|r|all>：定时闭环，到点自动停 */
void        Car_Control_TuneHalt(void);                           /* hlt：目标清零并停车，留在模式内 */
void        Car_Control_TuneSetTarget(Car_Wheel_t wheel, int16_t mm_s);
                                                                  /* vl/vr：设目标速度（mm/s） */
void        Car_Control_TuneSetGain(Car_Wheel_t wheel, Car_TuneTerm_t term, float value);
                                                                  /* kl/il/dl 等：在线改增益 */
void        Car_Control_TuneSetFan(uint16_t duty);
                                                                  /* tf：设定下次 ! 实验的负压占空比（0~1000，0=不带） */
void        Car_Control_TuneSnapshot(Car_TuneSnapshot_t *snap);   /* 取当前状态（串口打印用） */
                                                    /* 延时风扇：等 CAR_FAN_DELAY_MS 后跑 CAR_FAN_TEST_MS 再停 */

#endif /* __CAR_CONTROL_H */
