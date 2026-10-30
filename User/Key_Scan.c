#include "Key_Scan.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "Board_Config.h"

// 按键引脚定义见 Board_Config.h：
//   KEY1 = PA5 (引脚15)  KEY2 = PA4 (引脚14)
//   KEY3 = PC14(引脚3)   KEY4 = PC13(引脚2)

// 按键内部状态结构
typedef struct {
    uint8_t last_state;     // 上次状态
    uint8_t is_pressed;     // 是否处于按下状态（用于检测按下->释放）
} Key_Status_t;

// 按键状态数组
static Key_Status_t key_status[4] = {0};

// 按键事件队列（最多缓存4个事件）
#define KEY_EVENT_BUFFER_SIZE   4
static Key_Event_t key_event_buffer[KEY_EVENT_BUFFER_SIZE];
static uint8_t event_write_index = 0;
static uint8_t event_read_index = 0;
static uint8_t event_count = 0;

// 读取按键原始状态（低电平有效）
static inline uint8_t Key_ReadRaw(Key_ID_t key_id)
{
    uint8_t state = 0;
    
    switch(key_id)
    {
        case KEY_K1:
            state = (GPIO_ReadInputDataBit(KEY_K1_PORT, KEY_K1_PIN) == Bit_RESET) ? 1 : 0;
            break;
        case KEY_K2:
            state = (GPIO_ReadInputDataBit(KEY_K2_PORT, KEY_K2_PIN) == Bit_RESET) ? 1 : 0;
            break;
        case KEY_K3:
            state = (GPIO_ReadInputDataBit(KEY_K3_PORT, KEY_K3_PIN) == Bit_RESET) ? 1 : 0;
            break;
        case KEY_K4:
            state = (GPIO_ReadInputDataBit(KEY_K4_PORT, KEY_K4_PIN) == Bit_RESET) ? 1 : 0;
            break;
        default:
            break;
    }
    
    return state;
}

// 添加事件到队列
static void Key_AddEvent(Key_ID_t key_id)
{
    if(event_count >= KEY_EVENT_BUFFER_SIZE)
    {
        return; // 事件队列已满，丢弃事件
    }
    
    key_event_buffer[event_write_index].key_id = key_id;
    
    event_write_index = (event_write_index + 1) % KEY_EVENT_BUFFER_SIZE;
    event_count++;
}

// 初始化按键扫描
void Key_Scan_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    
    // 使能GPIO时钟
    RCC_APB2PeriphClockCmd(KEY_K1_RCC | KEY_K2_RCC | KEY_K3_RCC | KEY_K4_RCC | RCC_APB2Periph_AFIO, ENABLE);
    
    // 配置K1 (PA5) 和 K2 (PA4) 为上拉输入
    GPIO_InitStructure.GPIO_Pin = KEY_K1_PIN | KEY_K2_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;  // 上拉输入
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(KEY_K1_PORT, &GPIO_InitStructure);
    
    // 配置K3 (PC14) 和 K4 (PC13) 为上拉输入
    GPIO_InitStructure.GPIO_Pin = KEY_K3_PIN;
    GPIO_Init(KEY_K3_PORT, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin = KEY_K4_PIN;
    GPIO_Init(KEY_K4_PORT, &GPIO_InitStructure);
    
    // 初始化状态
    for(uint8_t i = 0; i < 4; i++)
    {
        key_status[i].last_state = Key_ReadRaw((Key_ID_t)(i + 1));
        key_status[i].is_pressed = 0;
    }
    
    // 初始化事件队列
    event_write_index = 0;
    event_read_index = 0;
    event_count = 0;
}

// 按键扫描更新（非阻塞，需要周期性调用）
void Key_Scan_Update(void)
{
    Key_ID_t key_ids[] = {KEY_K1, KEY_K2, KEY_K3, KEY_K4};
    
    for(uint8_t i = 0; i < 4; i++)
    {
        Key_ID_t key_id = key_ids[i];
        Key_Status_t *status = &key_status[i];
        
        // 读取当前状态
        uint8_t current_state = Key_ReadRaw(key_id);
        
        // 检测按下：从释放(0)到按下(1)
        if(current_state == 1 && status->last_state == 0)
        {
            status->is_pressed = 1;  // 标记为按下状态
        }
        
        // 检测释放：从按下(1)到释放(0)，且之前已经按下过
        if(current_state == 0 && status->last_state == 1 && status->is_pressed == 1)
        {
            status->is_pressed = 0;  // 清除按下标记
            Key_AddEvent(key_id);    // 触发按键事件
        }
        
        // 更新上次状态
        status->last_state = current_state;
    }
}

// 获取按键当前状态
uint8_t Key_GetState(Key_ID_t key_id)
{
    if(key_id < KEY_K1 || key_id > KEY_K4)
    {
        return 0;
    }
    
    return Key_ReadRaw(key_id);
}

// 获取按键事件
Key_Event_t* Key_GetEvent(void)
{
    if(event_count == 0)
    {
        return 0;  // 无事件
    }
    
    Key_Event_t *event = &key_event_buffer[event_read_index];
    event_read_index = (event_read_index + 1) % KEY_EVENT_BUFFER_SIZE;
    event_count--;
    
    return event;
}

// 检查是否有按键事件
uint8_t Key_HasEvent(void)
{
    return (event_count > 0) ? 1 : 0;
}

// 清除所有按键事件
void Key_ClearEvent(void)
{
    event_read_index = 0;
    event_write_index = 0;
    event_count = 0;
}
