/**
  ******************************************************************************
  * @file    app_state.h
  * @brief   任务间共享状态、指令队列与事件组
  *
  *  数据流：
  *    USART2 ISR ──(环形缓冲 + 任务通知)──▶ serial 任务 ──(指令队列)──▶ ctrl 任务
  *    按键 GPIO ──▶ keys 任务 ──(指令队列)──▶ ctrl 任务
  *    ctrl 任务 ──(事件组 / 共享变量)──▶ display 任务、serial 任务
  *
  *  “执行动作”的只有 ctrl 任务：模式切换、电机、风扇、LED 状态机全部在
  *  2ms 控制任务里完成，其它任务只投递指令、读取状态，避免并发写外设。
  ******************************************************************************
  */

#ifndef __APP_STATE_H
#define __APP_STATE_H

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "event_groups.h"
#include "Car_Control.h"
#include "BlackPoint_Finder.h"
#include <stdint.h>

/*------------------------------------------------------------------------------
 * 事件组位定义（ctrl 任务发布，其它任务读取）
 *----------------------------------------------------------------------------*/
#define APP_EVT_LINE_FOUND      ( ( EventBits_t ) 0x0001u )   /* 当前在线上   */
#define APP_EVT_LINE_LOST       ( ( EventBits_t ) 0x0002u )   /* 当前丢线     */
#define APP_EVT_ALARM           ( ( EventBits_t ) 0x0004u )   /* 异常停车     */
#define APP_EVT_WAVE_ON         ( ( EventBits_t ) 0x0008u )   /* 波形帧已开启 */

/*------------------------------------------------------------------------------
 * 指令类型：所有“会改运行状态”的操作都走这条队列
 *----------------------------------------------------------------------------*/
typedef enum
{
	APP_CMD_MODE = 0,        /* arg = Car_Mode_t */
	APP_CMD_LED_FORCE_ON,    /* 指示灯强制常亮（调试） */
	APP_CMD_LED_FORCE_OFF,   /* 指示灯强制常灭（调试） */
	APP_CMD_WAVE_TOGGLE,     /* 切换波形帧输出 */
	APP_CMD_WHEEL_LEFT,      /* arg = 百分比 0~100，左轮单独转 2 秒 */
	APP_CMD_WHEEL_RIGHT,     /* arg = 百分比 0~100，右轮单独转 2 秒 */
	APP_CMD_FAN_TEST,        /* arg = 百分比 0~100，负压风扇跑 5 秒（两灯快闪） */
	APP_CMD_FAN_DELAY_TEST,  /* arg = 百分比 0~100，先等 5 秒再跑 5 秒（两灯超快闪） */
	APP_CMD_ENC_MON,         /* 切换编码器实时监视（串口 'bmq'，不动电机/模式） */
	APP_CMD_SPEED_TEST_L,    /* arg16 = 目标速度，左轮速度环标定（串口 'testl'） */
	APP_CMD_SPEED_TEST_R,    /* arg16 = 目标速度，右轮速度环标定（串口 'testr'） */
	APP_CMD_TUNE_ENTER,       /* tune：进入/复位速度环调参模式（停车） */
	APP_CMD_TUNE_RUN,         /* `! ms side`：定时闭环，arg=侧别，arg16=时长 ms */
	APP_CMD_TUNE_HALT,        /* hlt：目标清零、占空比 0，留在调参模式 */
	APP_CMD_TUNE_TARGET,      /* vl/vr：arg=左右轮，arg16=目标 mm/s */
	APP_CMD_TUNE_GAIN,        /* kl/il/dl…：arg=轮*3+项，arg16=增益×1000 */
	APP_CMD_TUNE_FAN,         /* tf：arg=百分比 0~100，下次 `!` 实验的负压（0=不带） */
	APP_CMD_TUNE_RAMP         /* tr：arg16=软起动时长 ms（0=不软起动），下次 `!` 生效 */
} AppCmdType_t;

typedef struct
{
	AppCmdType_t type;
	uint8_t      arg;     /* 0~255 的参数（百分比等） */
	int16_t      arg16;   /* 需要 0~400 这种量程时用（速度环标定目标） */
} AppCmd_t;

/*------------------------------------------------------------------------------
 * 全局对象
 *----------------------------------------------------------------------------*/
extern QueueHandle_t      g_app_cmd_queue;    /* 指令队列（8 深） */
extern EventGroupHandle_t g_app_events;       /* 运行状态事件组 */

/* 任务句柄（串口 's' 指令打印栈水印时使用） */
extern TaskHandle_t g_task_ctrl;
extern TaskHandle_t g_task_keys;
extern TaskHandle_t g_task_display;
extern TaskHandle_t g_task_serial;

/* 波形帧开关（serial 任务读，任何任务可投递 APP_CMD_WAVE_TOGGLE 切换） */
extern volatile uint8_t g_wave_enable;

/* 编码器实时监视开关（串口 'bmq' 切换；serial 任务读，ctrl 任务写） */
extern volatile uint8_t g_enc_mon;

/* 推车测距（串口 mc）：ctrl 任务每 2ms 累加编码器增量，serial 任务只读 */
extern volatile uint8_t  g_meas_on;                          /* 1 = 正在测 */
extern volatile int32_t  g_meas_net_l;                       /* 左轮净计数（带符号） */
extern volatile int32_t  g_meas_net_r;                       /* 右轮净计数（带符号） */
extern volatile uint32_t g_meas_abs_l;                       /* 左轮绝对值累计（判来回拖） */
extern volatile uint32_t g_meas_abs_r;                       /* 右轮绝对值累计 */

/*------------------------------------------------------------------------------
 * 控制任务维护的采样状态（其它任务只读；均为 32 位以内变量，Cortex-M3 单次读写原子）
 *   position_get : 寻线位置 ×10（0~150），PID 里再 /10
 *   BDI_V        : 电池电压（V）
 *   add_angle    : 陀螺仪 Y 轴角速度积分（rad）
 *   star_car     : 闭环发车标志（PID_Controller.c 引用）
 *----------------------------------------------------------------------------*/
extern int16_t  position_get;
extern float    BDI_V;
extern float    add_angle;
extern float    add_angle_num;
extern uint8_t  star_car;
extern BlackPointResult_t result_BlackPoint;

/*------------------------------------------------------------------------------
 * 接口
 *----------------------------------------------------------------------------*/
void        App_StateInit(void);                            /* 创建队列/事件组（调度器启动前） */
void        App_CtrlWatchdogInit(void);                     /* 创建 20ms 控制任务看门狗（调度器启动前） */
BaseType_t  App_PostCmd(AppCmdType_t type, uint8_t arg);    /* 非阻塞投递指令 */
BaseType_t  App_PostCmdArg(AppCmdType_t type, uint8_t arg, int16_t arg16);
                                                            /* 带 16 位参数的投递（速度环标定目标 0~400） */

/* 任务入口 */
void App_CtrlTask(void *argument);
void App_KeysTask(void *argument);
void App_DisplayTask(void *argument);
void App_SerialTask(void *argument);

#endif /* __APP_STATE_H */
