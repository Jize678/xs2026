#include "stm32f10x.h"
#include "ABEncoder.h"

static int16_t last_count_left = 0,last_count_right = 0;
int16_t speed_left = 0,speed_right = 0;
float speed_left_mm_s = 0.0f, speed_right_mm_s = 0.0f;   /* mm/s：滑窗平均后（控制环用这个） */
float speed_left_mm_s_raw = 0.0f, speed_right_mm_s_raw = 0.0f;   /* mm/s：单拍原始差分（仅诊断） */
int32_t left_ecoder_cnt = 0, right_ecoder_cnt = 0;

/* ---- 测速滑窗（A2）：最近 ENC_SPEED_WINDOW 拍原始计数差分 ----
 * 环形缓冲，挤掉最旧的一拍后整窗求和；N=1 时等价于原来的瞬时差分。
 * win_cnt 只在上电后的前 N 拍小于 N，此时按“已填样本数”取平均，
 * 避免把不存在的历史当成 0 拉低读数。 */
static int16_t win_l[ENC_SPEED_WINDOW] = {0};
static int16_t win_r[ENC_SPEED_WINDOW] = {0};
static uint8_t win_idx = 0;
static uint8_t win_cnt = 0;

void ABEncoder_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

    //ENC1: PA6=ENC1B/PA7=ENC1A(TIM3)，ENC2: PB6=ENC2B/PB7=ENC2A(TIM4)
    RCC_APB2PeriphClockCmd(ENC_GPIO_RCC | RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd(ENC1_TIM_RCC | ENC2_TIM_RCC, ENABLE);

    // PA6(ENC1B)/PA7(ENC1A) 浮空输入
    GPIO_InitStructure.GPIO_Pin = ENC1_B_PIN | ENC1_A_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(ENC1_GPIO_PORT, &GPIO_InitStructure);
	    // PB6(ENC2B)/PB7(ENC2A) 浮空输入
    GPIO_InitStructure.GPIO_Pin = ENC2_B_PIN | ENC2_A_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(ENC2_GPIO_PORT, &GPIO_InitStructure);


    // 定时器基本参数 (TIM3 -> ENC1 右轮)
    TIM_TimeBaseStructure.TIM_Period = 0xFFFF;
    TIM_TimeBaseStructure.TIM_Prescaler = 0;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(ENC1_TIM, &TIM_TimeBaseStructure);
		    // 定时器基本参数 (TIM4 -> ENC2 左轮)
    TIM_TimeBaseStructure.TIM_Period = 0xFFFF;
    TIM_TimeBaseStructure.TIM_Prescaler = 0;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(ENC2_TIM, &TIM_TimeBaseStructure);

    // 编码器接口模式
    TIM_EncoderInterfaceConfig(ENC1_TIM, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
    TIM_ClearFlag(ENC1_TIM, TIM_FLAG_Update);
    TIM_SetCounter(ENC1_TIM, 0);
    TIM_Cmd(ENC1_TIM, ENABLE);
		    // 编码器接口模式
    TIM_EncoderInterfaceConfig(ENC2_TIM, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
    TIM_ClearFlag(ENC2_TIM, TIM_FLAG_Update);
    TIM_SetCounter(ENC2_TIM, 0);
    TIM_Cmd(ENC2_TIM, ENABLE);
}

void ABEncoder_UpdateSpeed(void) // 计算速度
{
    int16_t raw_left  = (int16_t)TIM_GetCounter(ENC2_TIM);   // 左轮原始计数
    int16_t raw_right = (int16_t)TIM_GetCounter(ENC1_TIM);   // 右轮原始计数
    int16_t now_left  = (int16_t)(ENC_L_SIGN * raw_left);
    int16_t now_right = (int16_t)(ENC_R_SIGN * raw_right);

    speed_left = now_left - last_count_left;
    last_count_left = now_left;
    speed_right = now_right - last_count_right;
    last_count_right = now_right;
    left_ecoder_cnt += speed_left;
    right_ecoder_cnt += speed_right;

    /* 压入滑窗：覆盖最旧的一拍，然后整窗求和（N 最大 8，代价可忽略） */
    win_l[win_idx] = speed_left;
    win_r[win_idx] = speed_right;
    win_idx++;
    if (win_idx >= ENC_SPEED_WINDOW) { win_idx = 0; }
    if (win_cnt < ENC_SPEED_WINDOW)  { win_cnt++; }

    {
        int32_t sum_l = 0, sum_r = 0;
        uint8_t i;
        for (i = 0; i < win_cnt; i++)
        {
            sum_l += win_l[i];
            sum_r += win_r[i];
        }

        /* 原始瞬时值：单拍差分，仅供诊断/对比，不参与控制 */
        speed_left_mm_s_raw  = (float)speed_left  * ENC_COUNTS_TO_MM_S;
        speed_right_mm_s_raw = (float)speed_right * ENC_COUNTS_TO_MM_S;

        /* 滑窗平均：sum/win_cnt 仍是“每 2ms 的平均计数”，所以标定常数不变。
         * 除法换成乘倒数，省一次软件浮点除法（M3 无 FPU）。 */
        {
            float inv = 1.0f / (float)win_cnt;
            speed_left_mm_s  = (float)sum_l * inv * ENC_COUNTS_TO_MM_S;
            speed_right_mm_s = (float)sum_r * inv * ENC_COUNTS_TO_MM_S;
        }
    }
}

