#ifndef __LSM6DSR_CONFIG_H__
#define __LSM6DSR_CONFIG_H__

#include "stm32f10x.h"
#include <stdint.h>
#include "Board_Config.h"

// 选择通信方式：硬件SPI或软件SPI
// 使用硬件SPI时注释掉下面这行，使用软件SPI时保留
// #define USE_SOFTWARE_SPI

// 引脚定义（见 Board_Config.h）：PB12(NSS), PB13(SCK), PB14(MISO), PB15(MOSI)
#define LSM6DSR_CS_PORT    GYRO_NSS_PORT
#define LSM6DSR_CS_PIN     GYRO_NSS_PIN
#define LSM6DSR_SCK_PORT   GYRO_SCK_PORT
#define LSM6DSR_SCK_PIN    GYRO_SCK_PIN
#define LSM6DSR_MISO_PORT  GYRO_MISO_PORT
#define LSM6DSR_MISO_PIN   GYRO_MISO_PIN
#define LSM6DSR_MOSI_PORT  GYRO_MOSI_PORT
#define LSM6DSR_MOSI_PIN   GYRO_MOSI_PIN

// LSM6DSR寄存器地址
#define LSM6DSR_WHO_AM_I       0x0F
#define LSM6DSR_DEVICE_ID      0x6B

#define LSM6DSR_FUNC_CFG       0x01
#define LSM6DSR_INT1_CTRL      0x0D
#define LSM6DSR_INT2_CTRL      0x0E

#define LSM6DSR_CTRL1_XL       0x10  // 加速度计控制寄存器1
#define LSM6DSR_CTRL2_G        0x11  // 陀螺仪控制寄存器2
#define LSM6DSR_CTRL3_C        0x12
#define LSM6DSR_CTRL4_C        0x13
#define LSM6DSR_CTRL5_C        0x14
#define LSM6DSR_CTRL6_C        0x15
#define LSM6DSR_CTRL7_G        0x16
#define LSM6DSR_CTRL8_XL       0x17
#define LSM6DSR_CTRL9_XL       0x18
#define LSM6DSR_CTRL10_C       0x19

#define LSM6DSR_STATUS_REG     0x1E

#define LSM6DSR_OUT_TEMP_L     0x20
#define LSM6DSR_OUT_TEMP_H     0x21

#define LSM6DSR_OUTX_L_GYRO    0x22
#define LSM6DSR_OUTX_H_GYRO    0x23
#define LSM6DSR_OUTY_L_GYRO    0x24
#define LSM6DSR_OUTY_H_GYRO    0x25
#define LSM6DSR_OUTZ_L_GYRO    0x26
#define LSM6DSR_OUTZ_H_GYRO    0x27

#define LSM6DSR_OUTX_L_ACC     0x28
#define LSM6DSR_OUTX_H_ACC     0x29
#define LSM6DSR_OUTY_L_ACC     0x2A
#define LSM6DSR_OUTY_H_ACC     0x2B
#define LSM6DSR_OUTZ_L_ACC     0x2C
#define LSM6DSR_OUTZ_H_ACC     0x2D

typedef struct
{
	int16_t ax;
	int16_t ay;
	int16_t az;
	int16_t gx;
	int16_t gy;
	int16_t gz;
	float ax_g;     // 加速度 X轴 (g)
	float ay_g;     // 加速度 Y轴 (g)
	float az_g;     // 加速度 Z轴 (g)
	float gx_rads;  // 角速度 X轴 (弧度/秒)
	float gy_rads;  // 角速度 Y轴 (弧度/秒)
	float gz_rads;  // 角速度 Z轴 (弧度/秒)
}LSM6DSR_DATA_T;
extern LSM6DSR_DATA_T LSE6DSR_data;
// 函数声明
uint8_t LSM6DSR_Init(void);
uint8_t LSM6DSR_ReadID(void);
void LSM6DSR_ReadData(LSM6DSR_DATA_T *physics);
/* 只读陀螺 6 字节（2ms 控制环用，加速度由 ReadData 低频刷新） */
void LSM6DSR_ReadGyro(LSM6DSR_DATA_T *physics);
void LSM6DSR_ConvertToPhysics(LSM6DSR_DATA_T *physics);
uint8_t LSM6DSR_ReadReg(uint8_t reg);
void LSM6DSR_WriteReg(uint8_t reg, uint8_t value);
void LSM6DSR_ReadRegs(uint8_t reg, uint8_t *buf, uint8_t len);

/* SPI 通路自检（位操作读取，结果供串口 i 指令查看）
 *   g_imu_bb_miso_idle：MISO 上拉后的空闲电平（1=悬空/无人驱动，0=被拉低）
 *   g_imu_bb_who      ：正常接线位操作读到的 WHO_AM_I
 *   g_imu_bb_who_swap ：MISO/MOSI 互换后读到的 WHO_AM_I（防 SDA/SDO 接反）
 *   g_imu_bb_cslo_up/dn：CS 拉低（选中芯片）后，MISO 分别用上拉/下拉读到的电平，
 *                        用来做三态判定：1/1=芯片驱高、0/0=芯片驱低（都说明芯片活着），
 *                        1/0=线上没人驱动（没供电 / CS 没到芯片 / 芯片不回应） */
extern uint8_t g_imu_bb_miso_idle;
extern uint8_t g_imu_bb_who;
extern uint8_t g_imu_bb_who_swap;
extern uint8_t g_imu_bb_cslo_up;
extern uint8_t g_imu_bb_cslo_dn;
/* 总线恢复（最多 40 个 SCL 时钟 + 合法 STOP）的结果：
 *   g_imu_bus_recover_clocks：用了几个时钟 SDA 才放开，0 = 一直没放开
 *   g_imu_bus_sda_idle / g_imu_bus_scl_idle：恢复后两条线的空闲电平，1/1 才算干净 */
extern uint8_t g_imu_bus_sda_idle;
extern uint8_t g_imu_bus_scl_idle;
extern uint8_t g_imu_bus_recover_clocks;
extern uint8_t g_imu_bus_sda_ipu;   /* SDA 换内部上拉读到的电平：1=没人拉低（缺上拉），0=有人按住 */
extern volatile uint8_t g_imu_cs_hold;
void LSM6DSR_BitBangSelfTest(void);

/* I2C 旁路探测（串口 x 指令）：CS 为高时芯片应工作在 I2C 模式，
 * 用 PB13(SCL)/PB15(SDA) 直接扫描总线，判断芯片是否还活着。
 *   g_imu_i2c_ack_6a/6b  ：0x6A / 0x6B 是否有 ACK
 *   g_imu_i2c_who_6a/6b  ：对应地址读回的 WHO_AM_I（正常为 0x6B）
 *   g_imu_i2c_scan_cnt   ：总线上有应答的器件个数（排除上面两个后再看）
 *   g_imu_i2c_scan_first ：第一个有应答的地址 */
extern uint8_t g_imu_i2c_ack_6a;
extern uint8_t g_imu_i2c_ack_6b;
extern uint8_t g_imu_i2c_who_6a;
extern uint8_t g_imu_i2c_who_6b;
extern uint8_t g_imu_i2c_scan_cnt;
extern uint8_t g_imu_i2c_scan_first;
void LSM6DSR_I2cScanTest(void);

#endif
