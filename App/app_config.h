/**
  ******************************************************************************
  * @file    app_config.h
  * @brief   应用层任务配置（优先级 / 栈 / 周期 / 队列深度）
  *
  *  调度设计（FreeRTOS 抢占式，tick = 1ms）：
  *  ┌──────────────┬──────┬────────┬──────────┬──────────────────────────────┐
  *  │ 任务         │ 优先级│ 栈(字) │ 周期     │ 职责                          │
  *  ├──────────────┼──────┼────────┼──────────┼──────────────────────────────┤
  *  │ ctrl         │  4   │  256   │  2 ms    │ IMU/编码器/ADC/寻线/PID/电机  │
  *  │ keys         │  3   │  128   │ 10 ms    │ 按键扫描 → 指令队列           │
  *  │ display      │  2   │  256   │ 50 ms    │ OLED 四行刷新                 │
  *  │ serial       │  2   │  288   │ 事件+20ms│ 串口指令解析 / 波形帧输出     │
  *  │ IDLE         │  0   │  128   │  —       │ 内核空闲任务                  │
  *  │ Tmr Svc      │  3   │  128   │  —       │ 控制任务看门狗（20ms 超时）   │
  *  └──────────────┴──────┴────────┴──────────┴──────────────────────────────┘
  *  优先级数值越大越高；configMAX_PRIORITIES = 6。
  ******************************************************************************
  */

#ifndef __APP_CONFIG_H
#define __APP_CONFIG_H

/*---------------- 任务优先级 ----------------*/
#define APP_PRIO_CTRL           4
#define APP_PRIO_KEYS           3
#define APP_PRIO_DISPLAY        2
#define APP_PRIO_SERIAL         2

/*---------------- 任务栈（单位：字，1 字 = 4 字节） ----------------*/
#define APP_STACK_CTRL          256
#define APP_STACK_KEYS          128
#define APP_STACK_DISPLAY       256
#define APP_STACK_SERIAL        288

/*---------------- 任务周期（ms） ----------------*/
#define APP_CTRL_PERIOD_MS      2       /* 控制环：与原 SysTick 2ms 中断一致 */
#define APP_KEYS_PERIOD_MS      10      /* 按键扫描 */
#define APP_DISPLAY_PERIOD_MS   50      /* OLED 刷新 */
#define APP_WAVE_PERIOD_MS      20      /* 波形帧 50Hz（JustFloat 24 字节） */
#define APP_ENC_MON_PERIOD_MS   20      /* 编码器监视（串口 'bmq'）打印周期，50Hz */
#define APP_GRAY_MON_PERIOD_MS  100     /* 灰度监视（串口 'g'）打印周期，10Hz；
                                         * 一次输出约 280 字节，115200 下占 ~24ms，
                                         * 想更实时可改到 50，但别小于 30 */
#define APP_SPEED_TEST_PERIOD_MS 100  /* 速度环标定（testl/testr）PID 数据打印周期，10Hz */
#define APP_TUNE_PERIOD_MS     20      /* 速度环调参（tune）CSV 打印周期，50Hz */

/*---------------- 控制任务看门狗 ----------------*/
/* 控制任务每 2ms 复位一次；若 20ms 内没有复位（任务卡死/被饿死），
 * 由软件定时器回调紧急停车并点亮 ALARM，避免电机失控。 */
#define APP_CTRL_WDT_MS         20


/*---------------- 队列 ----------------*/
#define APP_CMD_QUEUE_LEN       8       /* 指令队列深度（模式切换/LED 强制等） */

/*---------------- 波形帧 ----------------*/
/* VOFA+ JustFloat 协议：float32 小端 × N + 帧尾 00 00 80 7F */
#define APP_WAVE_CHANNELS       5       /* 位置 / 左速 / 右速 / 陀螺Z / 电池
                                         * （陀螺没装，第 4 通道恒 0） */

#endif /* __APP_CONFIG_H */
