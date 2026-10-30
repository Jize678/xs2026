#include "LSM6DSR_Config.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "Delay.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include <math.h>
#include <string.h>

/*==============================================================================
 * LSM6DSR 驱动（位操作 I2C）
 *
 * 现状：车上**没有装这颗传感器**。LSM6DSR_Init() 读不到 WHO_AM_I 就返回 0，
 * App_CtrlTask 见到 g_imu_init_ok == 0 会整段跳过 IMU，不做任何总线操作，
 * 朝向 yaw 恒为 0，循迹 / 计时 / 显示都不受影响。本文件保留完整驱动和诊断工具，
 * 以后装上传感器就能直接用（串口 i / x / c 三条指令都是为它准备的）。
 *
 * 为什么是 I2C 而不是 SPI：
 *   原来的硬件 SPI2（PB12 CS / PB13 SCK / PB14 MISO / PB15 MOSI）实测读回
 *   全 0x00 或全 0xFF。用串口 x 指令直接扫总线，在 0x6B 上扫到应答、读回
 *   WHO_AM_I = 0x6B —— 芯片活着，但一直停在 I2C 模式，说明模块上的 CS 焊盘
 *   并没有连到芯片的 CS 脚。于是把线序改成 I2C，复用同一组引脚：
 *       SCL = PB13（原 SCK）
 *       SDA = PB15（原 MOSI）
 *       PB12（CS）保持高电平，PB14（MISO）不用。
 *
 * 为什么位操作而不是硬件 I2C：
 *   I2C1 重映射到 PB8/PB9，这两个脚在本项目里是 RGB-2 / RGB-3；
 *   I2C2 是 PB10/PB11，分别是灰度地址线 S3 和风扇 PWM。硬件 I2C 没有落点。
 *
 * I2C 下不修就会偶发“整帧 0x00”的三件事：
 *   1. 总线互斥：控制任务每 2ms 读一次陀螺，串口任务在 i/x 时也要用总线，
 *      两边同时打时序会把字节搅烂，所以用一把互斥量 s_imu_bus_lock 串行化。
 *   2. 总线恢复：一次事务被中断（MCU 复位、时序错）后从机会一直把 SDA 拉低，
 *      之后所有读都是 0x00、扫描时每个地址都“应答”。上电先补最多 40 个 SCL
 *      时钟让它松手，再补一个合法 STOP。
 *   3. SCL 用推挽：开漏靠上拉爬升，长杜邦线上上升沿太慢，时钟一快就错位。
 *----------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------
 * 通信参数
 *---------------------------------------------------------------------------*/
#define IMU_I2C_ADDR7          0x6Bu                  /* SA0 接地，7 位地址 */
#define IMU_I2C_DELAY()        Delay_us(2)            /* 半周期 2us，约 100kHz */
#define IMU_GYRO_LSB_PER_RADS  1637.022271802352025f  /* ±1000dps：35 mdps/LSB → 1637 LSB/(rad/s) */

/* 静态零偏（LSB），沿用旧版驱动的实测值，用来抵消安装后的系统性偏置 */
#define IMU_GYRO_OFFSET_X      (-3.519f)
#define IMU_GYRO_OFFSET_Y      (12.03f)
#define IMU_GYRO_OFFSET_Z      (3.205f)

LSM6DSR_DATA_T LSE6DSR_data;

/* 诊断结果（串口 i / x 指令会打印） */
uint8_t g_imu_bb_miso_idle;       /* SPI 位操作自检：MISO 上拉后的空闲电平 */
uint8_t g_imu_bb_who;             /* SPI 位操作自检：正常接线的 WHO_AM_I */
uint8_t g_imu_bb_who_swap;        /* SPI 位操作自检：MISO/MOSI 互换后的 WHO_AM_I */
uint8_t g_imu_bb_cslo_up;         /* CS 拉低后 MISO 用上拉读到的电平（三态判定） */
uint8_t g_imu_bb_cslo_dn;         /* CS 拉低后 MISO 用下拉读到的电平 */
uint8_t g_imu_bus_sda_idle;       /* 总线恢复后 SDA 的空闲电平 */
uint8_t g_imu_bus_scl_idle;       /* 总线恢复后 SCL 的空闲电平 */
uint8_t g_imu_bus_recover_clocks; /* 恢复用掉几个时钟 SDA 才放开，0 = 一直没放开 */
uint8_t g_imu_bus_sda_ipu;        /* SDA 切内部上拉读到的电平：1=没人拉低，0=被按住 */
uint8_t g_imu_i2c_ack_6a;         /* 0x6A 是否有 ACK */
uint8_t g_imu_i2c_ack_6b;         /* 0x6B 是否有 ACK */
uint8_t g_imu_i2c_who_6a;         /* 0x6A 读回的 WHO_AM_I */
uint8_t g_imu_i2c_who_6b;         /* 0x6B 读回的 WHO_AM_I */
uint8_t g_imu_i2c_scan_cnt;       /* 总线上有应答的地址个数 */
uint8_t g_imu_i2c_scan_first;     /* 第一个有应答的地址 */

volatile uint8_t g_imu_cs_hold;   /* 1 = 串口 c 指令按住 CS 期间，禁止一切读写 */

static SemaphoreHandle_t s_imu_bus_lock;

/*------------------------------------------------------------------------------
 * 总线互斥
 *   调度器还没起来时（Init 阶段）mutex 可能还是 NULL，此时只有当前任务在跑，
 *   不加锁直接操作即可。
 *---------------------------------------------------------------------------*/
static void Imu_BusLock(void)
{
	if(s_imu_bus_lock != 0)
	{
		(void)xSemaphoreTake(s_imu_bus_lock, portMAX_DELAY);
	}
}

static void Imu_BusUnlock(void)
{
	if(s_imu_bus_lock != 0)
	{
		(void)xSemaphoreGive(s_imu_bus_lock);
	}
}

/*------------------------------------------------------------------------------
 * 引脚配置
 *---------------------------------------------------------------------------*/

/* I2C：SCL 推挽、SDA 开漏、CS 推挽高电平。
 * SDA 必须开漏——读数据和收 ACK 时要让从机把线拉低。 */
static void Imu_I2cPinConfig(void)
{
	GPIO_InitTypeDef gpio;

	RCC_APB2PeriphClockCmd(GYRO_GPIO_RCC | RCC_APB2Periph_AFIO, ENABLE);

	gpio.GPIO_Speed = GPIO_Speed_50MHz;

	gpio.GPIO_Mode = GPIO_Mode_Out_PP;
	gpio.GPIO_Pin  = GYRO_SCK_PIN;          /* SCL */
	GPIO_Init(GYRO_SCK_PORT, &gpio);
	GPIO_SetBits(GYRO_SCK_PORT, GYRO_SCK_PIN);

	gpio.GPIO_Mode = GPIO_Mode_Out_OD;
	gpio.GPIO_Pin  = GYRO_MOSI_PIN;         /* SDA */
	GPIO_Init(GYRO_MOSI_PORT, &gpio);
	GPIO_SetBits(GYRO_MOSI_PORT, GYRO_MOSI_PIN);

	gpio.GPIO_Mode = GPIO_Mode_Out_PP;
	gpio.GPIO_Pin  = GYRO_NSS_PIN;          /* CS：I2C 模式下必须是高 */
	GPIO_Init(GYRO_NSS_PORT, &gpio);
	GPIO_SetBits(GYRO_NSS_PORT, GYRO_NSS_PIN);
}

/* SPI 位操作自检用的方向：SCK/MOSI 推挽、MISO 按参数（上拉/下拉输入） */
static void Imu_SpiPinConfig(GPIOMode_TypeDef miso_mode)
{
	GPIO_InitTypeDef gpio;

	RCC_APB2PeriphClockCmd(GYRO_GPIO_RCC | RCC_APB2Periph_AFIO, ENABLE);

	gpio.GPIO_Speed = GPIO_Speed_50MHz;

	gpio.GPIO_Mode = GPIO_Mode_Out_PP;
	gpio.GPIO_Pin  = GYRO_NSS_PIN | GYRO_SCK_PIN | GYRO_MOSI_PIN;
	GPIO_Init(GPIOB, &gpio);
	GPIO_SetBits(GYRO_NSS_PORT, GYRO_NSS_PIN);
	GPIO_ResetBits(GYRO_SCK_PORT, GYRO_SCK_PIN);

	gpio.GPIO_Mode = miso_mode;
	gpio.GPIO_Pin  = GYRO_MISO_PIN;
	GPIO_Init(GYRO_MISO_PORT, &gpio);
}

/*------------------------------------------------------------------------------
 * I2C 基本时序（调用者负责持锁 + 引脚已配成 I2C 模式）
 *---------------------------------------------------------------------------*/
static void Imu_I2cStart(void)
{
	GPIO_SetBits(GYRO_MOSI_PORT, GYRO_MOSI_PIN);   /* SDA 高 */
	GPIO_SetBits(GYRO_SCK_PORT, GYRO_SCK_PIN);     /* SCL 高 */
	IMU_I2C_DELAY();
	GPIO_ResetBits(GYRO_MOSI_PORT, GYRO_MOSI_PIN); /* SCL 高时 SDA 下降沿 = START */
	IMU_I2C_DELAY();
	GPIO_ResetBits(GYRO_SCK_PORT, GYRO_SCK_PIN);
	IMU_I2C_DELAY();
}

static void Imu_I2cStop(void)
{
	GPIO_ResetBits(GYRO_MOSI_PORT, GYRO_MOSI_PIN); /* SDA 低 */
	IMU_I2C_DELAY();
	GPIO_SetBits(GYRO_SCK_PORT, GYRO_SCK_PIN);     /* SCL 高 */
	IMU_I2C_DELAY();
	GPIO_SetBits(GYRO_MOSI_PORT, GYRO_MOSI_PIN);   /* SCL 高时 SDA 上升沿 = STOP */
	IMU_I2C_DELAY();
}

/* 返回 0 = 收到 ACK，1 = NACK */
static uint8_t Imu_I2cWriteByte(uint8_t byte)
{
	uint8_t i, nack;

	for(i = 0u; i < 8u; i++)
	{
		if((byte & 0x80u) != 0u)
		{
			GPIO_SetBits(GYRO_MOSI_PORT, GYRO_MOSI_PIN);
		}
		else
		{
			GPIO_ResetBits(GYRO_MOSI_PORT, GYRO_MOSI_PIN);
		}
		IMU_I2C_DELAY();
		GPIO_SetBits(GYRO_SCK_PORT, GYRO_SCK_PIN);
		IMU_I2C_DELAY();
		GPIO_ResetBits(GYRO_SCK_PORT, GYRO_SCK_PIN);
		byte <<= 1;
	}
	IMU_I2C_DELAY();

	/* 第 9 个时钟：释放 SDA，从机拉低 = ACK */
	GPIO_SetBits(GYRO_MOSI_PORT, GYRO_MOSI_PIN);
	GPIO_SetBits(GYRO_SCK_PORT, GYRO_SCK_PIN);
	IMU_I2C_DELAY();
	nack = (GPIO_ReadInputDataBit(GYRO_MOSI_PORT, GYRO_MOSI_PIN) != 0u) ? 1u : 0u;
	GPIO_ResetBits(GYRO_SCK_PORT, GYRO_SCK_PIN);
	IMU_I2C_DELAY();

	return nack;
}

/* ack = 0 时回 ACK（还要继续读），1 时回 NACK（最后一字节） */
static uint8_t Imu_I2cReadByte(uint8_t ack)
{
	uint8_t i, byte = 0u;

	GPIO_SetBits(GYRO_MOSI_PORT, GYRO_MOSI_PIN);   /* 主机释放 SDA */
	for(i = 0u; i < 8u; i++)
	{
		GPIO_SetBits(GYRO_SCK_PORT, GYRO_SCK_PIN);
		IMU_I2C_DELAY();
		byte <<= 1;
		if(GPIO_ReadInputDataBit(GYRO_MOSI_PORT, GYRO_MOSI_PIN) != 0u)
		{
			byte |= 1u;
		}
		GPIO_ResetBits(GYRO_SCK_PORT, GYRO_SCK_PIN);
		IMU_I2C_DELAY();
	}

	if(ack == 0u)
	{
		GPIO_ResetBits(GYRO_MOSI_PORT, GYRO_MOSI_PIN);  /* ACK */
	}
	else
	{
		GPIO_SetBits(GYRO_MOSI_PORT, GYRO_MOSI_PIN);    /* NACK */
	}
	GPIO_SetBits(GYRO_SCK_PORT, GYRO_SCK_PIN);
	IMU_I2C_DELAY();
	GPIO_ResetBits(GYRO_SCK_PORT, GYRO_SCK_PIN);
	GPIO_SetBits(GYRO_MOSI_PORT, GYRO_MOSI_PIN);
	IMU_I2C_DELAY();

	return byte;
}

static uint8_t Imu_I2cWrite(uint8_t reg, const uint8_t *buf, uint8_t len)
{
	uint8_t i;

	Imu_I2cStart();
	if(Imu_I2cWriteByte((uint8_t)(IMU_I2C_ADDR7 << 1)) != 0u)
	{
		Imu_I2cStop();
		return 1u;
	}
	if(Imu_I2cWriteByte(reg) != 0u)
	{
		Imu_I2cStop();
		return 1u;
	}
	for(i = 0u; i < len; i++)
	{
		if(Imu_I2cWriteByte(buf[i]) != 0u)
		{
			Imu_I2cStop();
			return 1u;
		}
	}
	Imu_I2cStop();
	return 0u;
}

/* I2C 下连读不需要处理“地址自增位”，寄存器地址由芯片自己递增 */
static uint8_t Imu_I2cRead(uint8_t reg, uint8_t *buf, uint8_t len)
{
	uint8_t i;

	Imu_I2cStart();
	if(Imu_I2cWriteByte((uint8_t)(IMU_I2C_ADDR7 << 1)) != 0u)
	{
		Imu_I2cStop();
		return 1u;
	}
	if(Imu_I2cWriteByte(reg) != 0u)
	{
		Imu_I2cStop();
		return 1u;
	}

	Imu_I2cStart();                                            /* repeated START */
	if(Imu_I2cWriteByte((uint8_t)((IMU_I2C_ADDR7 << 1) | 0x01u)) != 0u)
	{
		Imu_I2cStop();
		return 1u;
	}
	for(i = 0u; i < len; i++)
	{
		buf[i] = Imu_I2cReadByte((i == (uint8_t)(len - 1u)) ? 1u : 0u);
	}
	Imu_I2cStop();
	return 0u;
}

/*------------------------------------------------------------------------------
 * 总线恢复
 *   从机卡住时会把 SDA 一直拉低。补 SCL 时钟让它把残留的字节移完，
 *   再补一个合法 STOP。结果记进全局，串口 i 的 bus 行会打印。
 *---------------------------------------------------------------------------*/
static void Imu_I2cBusRecover(void)
{
	uint8_t i;
	GPIO_InitTypeDef probe;

	GPIO_SetBits(GYRO_MOSI_PORT, GYRO_MOSI_PIN);   /* 释放 SDA */
	GPIO_SetBits(GYRO_SCK_PORT, GYRO_SCK_PIN);
	IMU_I2C_DELAY();

	for(i = 1u; i <= 40u; i++)
	{
		if(GPIO_ReadInputDataBit(GYRO_MOSI_PORT, GYRO_MOSI_PIN) != 0u)
		{
			break;                                  /* 已经放开 */
		}
		GPIO_ResetBits(GYRO_SCK_PORT, GYRO_SCK_PIN);
		IMU_I2C_DELAY();
		GPIO_SetBits(GYRO_SCK_PORT, GYRO_SCK_PIN);
		IMU_I2C_DELAY();
	}
	g_imu_bus_recover_clocks = (GPIO_ReadInputDataBit(GYRO_MOSI_PORT, GYRO_MOSI_PIN) != 0u) ? i : 0u;

	/* 合法 STOP：SCL 低 → SDA 低 → SCL 高 → SDA 高。
	 * 不能直接调 Imu_I2cStop()：它假定进来时 SCL 是低的。 */
	GPIO_ResetBits(GYRO_SCK_PORT, GYRO_SCK_PIN);
	IMU_I2C_DELAY();
	GPIO_ResetBits(GYRO_MOSI_PORT, GYRO_MOSI_PIN);
	IMU_I2C_DELAY();
	GPIO_SetBits(GYRO_SCK_PORT, GYRO_SCK_PIN);
	IMU_I2C_DELAY();
	GPIO_SetBits(GYRO_MOSI_PORT, GYRO_MOSI_PIN);
	IMU_I2C_DELAY();

	g_imu_bus_sda_idle = (GPIO_ReadInputDataBit(GYRO_MOSI_PORT, GYRO_MOSI_PIN) != 0u) ? 1u : 0u;
	g_imu_bus_scl_idle = (GPIO_ReadInputDataBit(GYRO_SCK_PORT, GYRO_SCK_PIN) != 0u) ? 1u : 0u;

	/* 再把 SDA 切成内部上拉输入读一次：区分“缺外部上拉”和“被器件按住” */
	probe.GPIO_Speed = GPIO_Speed_50MHz;
	probe.GPIO_Mode  = GPIO_Mode_IPU;
	probe.GPIO_Pin   = GYRO_MOSI_PIN;
	GPIO_Init(GYRO_MOSI_PORT, &probe);
	Delay_us(100);
	g_imu_bus_sda_ipu = (GPIO_ReadInputDataBit(GYRO_MOSI_PORT, GYRO_MOSI_PIN) != 0u) ? 1u : 0u;

	probe.GPIO_Mode = GPIO_Mode_Out_OD;
	GPIO_Init(GYRO_MOSI_PORT, &probe);
	GPIO_SetBits(GYRO_MOSI_PORT, GYRO_MOSI_PIN);
	Delay_us(50);
}

/*------------------------------------------------------------------------------
 * SPI 位操作自检（串口 i 打印用）
 *   用 GPIO 手打模式 0 时序绕开 SPI2 外设读 WHO_AM_I，用来区分
 *   “STM32 侧外设问题”和“线上没人回应”。当前驱动走 I2C，读不到是正常的。
 *---------------------------------------------------------------------------*/
static uint8_t Imu_SpiTransfer(uint8_t out, GPIO_TypeDef *mosi_port, uint16_t mosi_pin,
                               GPIO_TypeDef *miso_port, uint16_t miso_pin)
{
	uint8_t i, in = 0u;

	for(i = 0u; i < 8u; i++)
	{
		if((out & 0x80u) != 0u)
		{
			GPIO_SetBits(mosi_port, mosi_pin);
		}
		else
		{
			GPIO_ResetBits(mosi_port, mosi_pin);
		}
		out <<= 1;
		Delay_us(1);
		GPIO_SetBits(GYRO_SCK_PORT, GYRO_SCK_PIN);    /* 模式 0：上升沿采样 */
		Delay_us(1);
		in <<= 1;
		if(GPIO_ReadInputDataBit(miso_port, miso_pin) != 0u)
		{
			in |= 1u;
		}
		GPIO_ResetBits(GYRO_SCK_PORT, GYRO_SCK_PIN);
		Delay_us(1);
	}
	return in;
}

static uint8_t Imu_SpiReadReg(uint8_t reg, uint8_t swap)
{
	GPIO_ResetBits(GYRO_NSS_PORT, GYRO_NSS_PIN);
	Delay_us(1);
	if(swap == 0u)
	{
		(void)Imu_SpiTransfer((uint8_t)(reg | 0x80u), GYRO_MOSI_PORT, GYRO_MOSI_PIN,
		                      GYRO_MISO_PORT, GYRO_MISO_PIN);
		return Imu_SpiTransfer(0x00u, GYRO_MOSI_PORT, GYRO_MOSI_PIN,
		                       GYRO_MISO_PORT, GYRO_MISO_PIN);
	}
	/* MISO/MOSI 接反的接法 */
	(void)Imu_SpiTransfer((uint8_t)(reg | 0x80u), GYRO_MISO_PORT, GYRO_MISO_PIN,
	                      GYRO_MOSI_PORT, GYRO_MOSI_PIN);
	return Imu_SpiTransfer(0x00u, GYRO_MISO_PORT, GYRO_MISO_PIN,
	                       GYRO_MOSI_PORT, GYRO_MOSI_PIN);
}

void LSM6DSR_BitBangSelfTest(void)
{
	GPIO_InitTypeDef gpio;

	Imu_BusLock();

	/* 1. MISO 空闲电平（内部上拉） */
	Imu_SpiPinConfig(GPIO_Mode_IPU);
	Delay_us(50);
	g_imu_bb_miso_idle = (GPIO_ReadInputDataBit(GYRO_MISO_PORT, GYRO_MISO_PIN) != 0u) ? 1u : 0u;

	/* 2. 正常接法 / 互换接法各读一次 WHO_AM_I */
	g_imu_bb_who      = Imu_SpiReadReg(LSM6DSR_WHO_AM_I, 0u);
	Delay_us(20);
	g_imu_bb_who_swap = Imu_SpiReadReg(LSM6DSR_WHO_AM_I, 1u);
	GPIO_SetBits(GYRO_NSS_PORT, GYRO_NSS_PIN);
	Delay_us(20);

	/* 3. 三态判定：CS 拉低并打一个字节时钟，让芯片 SDO 进入驱动态，
	 *    再用上拉/下拉各读一次 MISO。
	 *    1/1 或 0/0 = 芯片在驱动（活着）；1/0 = 线上没人驱动。 */
	Imu_SpiPinConfig(GPIO_Mode_IPU);
	GPIO_ResetBits(GYRO_NSS_PORT, GYRO_NSS_PIN);
	if(g_imu_bb_who_swap == LSM6DSR_DEVICE_ID)
	{
		(void)Imu_SpiTransfer(0xFFu, GYRO_MISO_PORT, GYRO_MISO_PIN,
		                      GYRO_MOSI_PORT, GYRO_MOSI_PIN);
	}
	else
	{
		(void)Imu_SpiTransfer(0xFFu, GYRO_MOSI_PORT, GYRO_MOSI_PIN,
		                      GYRO_MISO_PORT, GYRO_MISO_PIN);
	}
	Delay_us(20);
	g_imu_bb_cslo_up = (GPIO_ReadInputDataBit(GYRO_MISO_PORT, GYRO_MISO_PIN) != 0u) ? 1u : 0u;

	gpio.GPIO_Speed = GPIO_Speed_50MHz;
	gpio.GPIO_Mode  = GPIO_Mode_IPD;
	gpio.GPIO_Pin   = GYRO_MISO_PIN;
	GPIO_Init(GYRO_MISO_PORT, &gpio);
	Delay_us(20);
	g_imu_bb_cslo_dn = (GPIO_ReadInputDataBit(GYRO_MISO_PORT, GYRO_MISO_PIN) != 0u) ? 1u : 0u;

	GPIO_SetBits(GYRO_NSS_PORT, GYRO_NSS_PIN);

	/* 用完恢复成 I2C 引脚，否则后面的读会把 SDA 当输出乱打 */
	Imu_I2cPinConfig();

	Imu_BusUnlock();
}

/*------------------------------------------------------------------------------
 * 寄存器访问（对外）
 *   g_imu_cs_hold 置位期间直接返回：串口 c 指令要长时间按住 CS 给万用表量电平
 *---------------------------------------------------------------------------*/
uint8_t LSM6DSR_ReadReg(uint8_t reg)
{
	uint8_t value = 0u;

	if(g_imu_cs_hold != 0u)
	{
		return 0u;
	}

	Imu_BusLock();
	Imu_I2cPinConfig();
	if(Imu_I2cRead(reg, &value, 1u) != 0u)
	{
		value = 0u;
	}
	Imu_BusUnlock();

	return value;
}

void LSM6DSR_WriteReg(uint8_t reg, uint8_t value)
{
	if(g_imu_cs_hold != 0u)
	{
		return;
	}

	Imu_BusLock();
	Imu_I2cPinConfig();
	(void)Imu_I2cWrite(reg, &value, 1u);
	Imu_BusUnlock();
}

void LSM6DSR_ReadRegs(uint8_t reg, uint8_t *buf, uint8_t len)
{
	if((g_imu_cs_hold != 0u) || (buf == 0))
	{
		return;
	}

	Imu_BusLock();
	Imu_I2cPinConfig();
	if(Imu_I2cRead(reg, buf, len) != 0u)
	{
		memset(buf, 0, len);
	}
	Imu_BusUnlock();
}

uint8_t LSM6DSR_ReadID(void)
{
	return LSM6DSR_ReadReg(LSM6DSR_WHO_AM_I);
}

/*------------------------------------------------------------------------------
 * 数据读取
 *   0x22 起连续 12 字节：gx/gy/gz/ax/ay/az，各 16 位小端。
 *---------------------------------------------------------------------------*/
void LSM6DSR_ReadData(LSM6DSR_DATA_T *physics)
{
	uint8_t buf[12];

	if((physics == 0) || (g_imu_cs_hold != 0u))
	{
		return;
	}

	Imu_BusLock();
	Imu_I2cPinConfig();
	if(Imu_I2cRead(LSM6DSR_OUTX_L_GYRO, buf, sizeof(buf)) != 0u)
	{
		memset(buf, 0, sizeof(buf));
	}
	Imu_BusUnlock();

	physics->gx = (int16_t)(((uint16_t)buf[1] << 8) | buf[0]);
	physics->gy = (int16_t)(((uint16_t)buf[3] << 8) | buf[2]);
	physics->gz = (int16_t)(((uint16_t)buf[5] << 8) | buf[4]);
	physics->ax = (int16_t)(((uint16_t)buf[7] << 8) | buf[6]);
	physics->ay = (int16_t)(((uint16_t)buf[9] << 8) | buf[8]);
	physics->az = (int16_t)(((uint16_t)buf[11] << 8) | buf[10]);
}

/* 2ms 控制环只取陀螺 6 字节：位操作 I2C 每字节约 9 个时钟，
 * 12 字节比 6 字节多花一倍时间，加速度不需要这么快。 */
void LSM6DSR_ReadGyro(LSM6DSR_DATA_T *physics)
{
	uint8_t buf[6];

	if((physics == 0) || (g_imu_cs_hold != 0u))
	{
		return;
	}

	Imu_BusLock();
	Imu_I2cPinConfig();
	if(Imu_I2cRead(LSM6DSR_OUTX_L_GYRO, buf, sizeof(buf)) != 0u)
	{
		memset(buf, 0, sizeof(buf));
	}
	Imu_BusUnlock();

	physics->gx = (int16_t)(((uint16_t)buf[1] << 8) | buf[0]);
	physics->gy = (int16_t)(((uint16_t)buf[3] << 8) | buf[2]);
	physics->gz = (int16_t)(((uint16_t)buf[5] << 8) | buf[4]);
}

void LSM6DSR_ConvertToPhysics(LSM6DSR_DATA_T *physics)
{
	if(physics == 0)
	{
		return;
	}

	physics->ax_g = (float)physics->ax / 16384.0f;          /* ±2g */
	physics->ay_g = (float)physics->ay / 16384.0f;
	physics->az_g = (float)physics->az / 16384.0f;

	physics->gx_rads = ((float)physics->gx - IMU_GYRO_OFFSET_X) / IMU_GYRO_LSB_PER_RADS;
	physics->gy_rads = ((float)physics->gy - IMU_GYRO_OFFSET_Y) / IMU_GYRO_LSB_PER_RADS;
	physics->gz_rads = ((float)physics->gz - IMU_GYRO_OFFSET_Z) / IMU_GYRO_LSB_PER_RADS;
}

/*------------------------------------------------------------------------------
 * 初始化
 *   返回 1 = 通信正常且已配置好；0 = 读不到 WHO_AM_I
 *   （车上没装传感器时就是 0，上层据此关掉整段 IMU）
 *---------------------------------------------------------------------------*/
uint8_t LSM6DSR_Init(void)
{
	uint8_t id = 0u;
	uint8_t i;

	if(s_imu_bus_lock == 0)
	{
		s_imu_bus_lock = xSemaphoreCreateMutex();
	}

	Imu_I2cPinConfig();
	Imu_I2cBusRecover();
	Delay_us(500);

	/* WHO_AM_I 重试 3 次：模块刚上电时第一次读常常是 0 */
	for(i = 0u; i < 3u; i++)
	{
		id = LSM6DSR_ReadID();
		if(id == LSM6DSR_DEVICE_ID)
		{
			break;
		}
		Delay_ms(10);
	}

	if(id != LSM6DSR_DEVICE_ID)
	{
		return 0u;
	}

	/* 软复位 */
	LSM6DSR_WriteReg(LSM6DSR_CTRL3_C, 0x01u);
	Delay_ms(20);

	LSM6DSR_WriteReg(LSM6DSR_CTRL3_C, 0x44u);   /* BDU=1（读期间不更新）+ IF_INC=1（地址自增） */
	LSM6DSR_WriteReg(LSM6DSR_CTRL1_XL, 0x20u);  /* 加速度 26Hz / ±2g（不用，只要不掉电） */
	LSM6DSR_WriteReg(LSM6DSR_CTRL2_G, 0x58u);   /* 陀螺 208Hz / ±1000dps → 1637 LSB/(rad/s) */
	LSM6DSR_WriteReg(LSM6DSR_CTRL6_C, 0x00u);   /* 不用 USR 寄存器组 */
	Delay_ms(10);

	return 1u;
}

/*------------------------------------------------------------------------------
 * I2C 总线扫描（串口 x 指令）
 *   把 CS 放高让芯片工作在 I2C 模式，然后逐个地址试探：有 ACK 说明总线上有器件；
 *   再从 0x6A/0x6B 读 WHO_AM_I 确认是不是这颗。
 *   scan_cnt = 0x70（每个地址都应答）是 SDA 被钉在低电平的典型症状。
 *---------------------------------------------------------------------------*/
void LSM6DSR_I2cScanTest(void)
{
	uint8_t addr, nack;

	g_imu_i2c_ack_6a     = 0u;
	g_imu_i2c_ack_6b     = 0u;
	g_imu_i2c_who_6a     = 0u;
	g_imu_i2c_who_6b     = 0u;
	g_imu_i2c_scan_cnt   = 0u;
	g_imu_i2c_scan_first = 0u;

	if(s_imu_bus_lock == 0)
	{
		s_imu_bus_lock = xSemaphoreCreateMutex();
	}

	Imu_BusLock();
	Imu_I2cPinConfig();
	Imu_I2cBusRecover();
	Delay_us(100);

	/* 0x6A / 0x6B 单独确认（SA0 接高接低的板子都有） */
	Imu_I2cStart();
	g_imu_i2c_ack_6a = (Imu_I2cWriteByte((uint8_t)(0x6Au << 1)) == 0u) ? 1u : 0u;
	Imu_I2cStop();
	if(g_imu_i2c_ack_6a != 0u)
	{
		(void)Imu_I2cRead(LSM6DSR_WHO_AM_I, &g_imu_i2c_who_6a, 1u);
	}
	Delay_us(50);

	Imu_I2cStart();
	g_imu_i2c_ack_6b = (Imu_I2cWriteByte((uint8_t)(0x6Bu << 1)) == 0u) ? 1u : 0u;
	Imu_I2cStop();
	if(g_imu_i2c_ack_6b != 0u)
	{
		(void)Imu_I2cRead(LSM6DSR_WHO_AM_I, &g_imu_i2c_who_6b, 1u);
	}
	Delay_us(50);

	/* 扫 0x00~0x6F：有 ACK 就计数并记下第一个地址 */
	for(addr = 0u; addr < 0x70u; addr++)
	{
		Imu_I2cStart();
		nack = Imu_I2cWriteByte((uint8_t)(addr << 1));
		Imu_I2cStop();
		if(nack == 0u)
		{
			if(g_imu_i2c_scan_cnt == 0u)
			{
				g_imu_i2c_scan_first = addr;
			}
			g_imu_i2c_scan_cnt++;
		}
		Delay_us(20);
	}

	Imu_BusUnlock();
}
