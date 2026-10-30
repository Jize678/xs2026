#ifndef __ABENCODER_H
#define __ABENCODER_H

#include "stm32f10x.h"
#include "Board_Config.h"

/* 编码器 A/B 相接法与计数方向（依据车模引脚表 + 实车标定）
 *   ENC1（右轮）: PA6 = ENC1B -> TIM3_CH1，PA7 = ENC1A -> TIM3_CH2
 *   ENC2（左轮）: PB6 = ENC2B -> TIM4_CH1，PB7 = ENC2A -> TIM4_CH2
 *
 * 约定：**车轮向前 = speed_left / speed_right 为正**。
 * 2026-10-29 实车标定（手推车前进，串口 mc）：左轮原始计数向前为负、
 * 右轮原始计数向前为正；两边都乘 -1 之后左轮为正、右轮为负，所以右轮改成 +1、左轮保持 -1。
 *
 * 上电自检：把车抬起，手动向前转车轮，speed_left / speed_right 都应为正；
 * 若某一侧为负，把对应宏的符号取反即可（另一侧不要动）。
 */
#define ENC_R_SIGN   (+1)   /* TIM3 / ENC1 / 右轮（实车:向前推原始计数为正,这里不取反） */
#define ENC_L_SIGN   (-1)   /* TIM4 / ENC2 / 左轮 */

/* ---------------- 里程标定 / 单位换算 ----------------
 * 2026-10-29 实车标定：手推车走 1 米，测 5 次（左右轮各一个）共 10 个数，
 * 去掉最大值、最小值后取平均 = 54476 counts，即 1 米对应 54476 个编码器计数。
 * 之后所有速度统一用 mm/s，编码器计数只保留在 speed_left/right 里做诊断用。
 */
#define ENC_COUNTS_PER_METER   54476.0f                                  /* 1 m = 54476 counts */
#define ENC_MM_PER_COUNT       (1000.0f / ENC_COUNTS_PER_METER)          /* ≈ 0.018357 mm/count */
#define ENC_SAMPLE_MS          2                                        /* UpdateSpeed 调用周期（必须 = CAR_CTRL_TICK_MS） */
/* counts/采样周期 -> mm/s：1 count/2ms ≈ 9.1784 mm/s（满速 230 counts/2ms ≈ 2111 mm/s） */
#define ENC_COUNTS_TO_MM_S     (1000.0f / ENC_COUNTS_PER_METER / ((float)ENC_SAMPLE_MS * 0.001f))

void ABEncoder_Init(void);
void ABEncoder_UpdateSpeed(void);
int16_t ABEncoder_GetSpeed_left(void);
int16_t ABEncoder_GetSpeed_right(void);
extern int16_t speed_left,speed_right;
extern float speed_left_mm_s, speed_right_mm_s;   /* 控制用：mm/s（已含 ±1 方向修正） */
extern int32_t left_ecoder_cnt, right_ecoder_cnt;
#endif
