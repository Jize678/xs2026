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
/* 目标阶跃前馈（2026-10-31 加）：目标改了 Δ(mm/s)，就把速度环输出直接挪
 * K·Δ 个占空比，不等误差累积。K 用实测静态增益标定：500 mm/s ≈ 693、
 * 1000 ≈ 945、1500 ≈ 1300 占空比 ⇒ 0.5~0.7，取 0.60。
 * 前馈只做“粗顶”，残差仍由 P/I 收，所以稳态精度不变；电池电压、负压大小
 * 会让 K 漂，用串口 `kff<0~2000>`（千分比）在线调，掉电不保存。
 *
 * 2026-10-31 晚：升速、降速分成**两套**（kp/ki/kd + 前馈各一套）。原因是
 * 两个方向的物理不对称——加速要额外扭矩去克服惯性，降速相反：摩擦已经在
 * 帮忙减速，力矩一撤转速掉得比闭环预想的还快，于是先冲过目标再被拉回来。
 *   升速集：kp/ki/kd 用调参值，前馈取 +0.30 左右（实测把超调从 +18% 压到 +8~15%）
 *   降速集：kd 默认 0（D 项在阶跃处等于再补一脚刹车，方向反了），前馈允许负值
 *          （负前馈 = 跳变瞬间多留一点扭矩；实测 ffDn 从 0 加到 +0.6，下冲
 *           从 −8% 单调恶化到 −34%，说明这个方向填反了）
 * 切换按“目标跳变方向”**锁存**，整段用同一套；不按误差符号切——稳态时速度
 * 在目标 ±4% 内抖，按误差切会每几拍翻一次，等于人为制造抖振。 */
#define CAR_TUNE_FF_DEFAULT     0.60f    /* 升速集默认前馈 */
#define CAR_TUNE_FF_DN_DEFAULT  0.00f    /* 降速集默认前馈 */
#define CAR_TUNE_FF_MAX         2.00f    /* 前馈绝对值上限（正负对称） */
#define CAR_TUNE_DN_KD_DEFAULT  0.00f    /* 降速集默认 kd */

/* 分段（增益调度）：速度分 3 档 × 升/降 2 个方向 = 6 组 kp/ki/kd/ff，
 * 每档再按左右轮各存一套（共 12 套）。档位按**这一段的目标速度**选：
 *   档 0：目标 <  600 mm/s（0~0.6 m/s）
 *   档 1：600 ~ 1199 mm/s（0.6~1.2 m/s）
 *   档 2：≥ 1200 mm/s（1.2 m/s 以上）
 * 为什么按目标而不按实测：实测在目标附近抖，按实测切档会在边界反复横跳；
 * 按目标切，一段之内恒定，和“升速/降速两套”用的是同一个锁存原则。 */
#define CAR_TUNE_BAND_COUNT     3        /* 档位数 */
#define CAR_TUNE_BAND1_MM_S     600      /* 档 1 下限 */
#define CAR_TUNE_BAND2_MM_S     1200     /* 档 2 下限 */
/* 调参实验的目标是**阶跃**的：`vl`/`vr` 发下去就是一步到位，PID 看到的是真正的
 * 设定值跳变。2026-10-31 试过的 smoothstep 软起动（`tr<ms>` 可调）已经删除——
 * 它只塑形“0→目标”那一次起跑，而实测需求是“速度阶跃 500↔1000↔1500 能否在
 * 100~300ms 内整定”，起跑抖动不重要；而且中途改目标时它会把新目标按进度打折。 */

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

/* 调参用哪一项参数 */
typedef enum
{
	CAR_TUNE_KP = 0,
	CAR_TUNE_KI = 1,
	CAR_TUNE_KD = 2,
	CAR_TUNE_FF = 3,        /* 该段的前馈系数（占空比/(mm/s)，可为负） */
	CAR_TUNE_TERM_COUNT = 4
} Car_TuneTerm_t;

/* `! <ms> <侧>` 的侧别：只跑一侧时另一侧目标强制 0 */
typedef enum
{
	CAR_TUNE_SIDE_L   = 0,
	CAR_TUNE_SIDE_R   = 1,
	CAR_TUNE_SIDE_ALL = 2
} Car_TuneSide_t;

/* 一套“方向专用”参数：kp/ki/kd + 该方向的阶跃前馈（占空比/(mm/s)，可为负） */
typedef struct
{
	float kp;
	float ki;
	float kd;
	float ff;
} Car_TuneGainSet_t;

/* 调参模式快照（串口任务打印 CSV 用，只读） */
typedef struct
{
	uint8_t running;        /* 1 = 正在闭环 */
	uint8_t side;           /* 本次 ! 的侧别（Car_TuneSide_t） */
	uint16_t run_ms;        /* 本次 ! 的定时（ms），停止后保留显示 */
	uint16_t fan_duty;      /* tf 设定的负压占空比（0~1000），0 = 不带负压 */
	uint8_t  arming;        /* 1 = 正在等电调解锁（1.5s），闭环还没开始 */
	uint8_t  dir_down;      /* 1 = 当前这一段按“降速集”在跑 */
	uint8_t  band;          /* 当前这一段用的档位（0~2） */
	/* 12 套参数（2 轮 × 3 档 × 2 方向）不放进快照：那是 192 字节，串口任务
	 * 的栈只有 288 字节，放进去会溢出。要用参数请调 Car_Control_TuneGetSet()。 */
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
void        Car_Control_TuneSetParam(Car_Wheel_t wheel, uint8_t band, uint8_t down,
                                     Car_TuneTerm_t term, float value);
                                                                  /* k/K + 项 + 档 + 侧 + 值：
                                                                   * 改 [轮][档][方向] 这一套里的某一项 */
void        Car_Control_TuneSetFan(uint16_t duty);
                                                                  /* tf：设定下次 ! 实验的负压占空比（0~1000，0=不带） */
void        Car_Control_TuneSetFFAll(uint8_t down, float duty_per_mm_s);
                                                                  /* kff / KFF（旧别名）：把该方向的 3 档 × 2 轮
                                                                  * 前馈一起设，掉电不保存 */
const Car_TuneGainSet_t *Car_Control_TuneGetSet(uint8_t side, uint8_t band, uint8_t down);
                                                                  /* 只读取一套参数（串口 kq 打印用） */
void        Car_Control_TuneSnapshot(Car_TuneSnapshot_t *snap);   /* 取当前状态（串口打印用） */
                                                    /* 延时风扇：等 CAR_FAN_DELAY_MS 后跑 CAR_FAN_TEST_MS 再停 */

#endif /* __CAR_CONTROL_H */
