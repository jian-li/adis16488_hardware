#ifndef __ADI_H
#define __ADI_H

#include "stm32f1xx_hal.h"
#include "delay.h"

#define G_VALUE 9.8

struct ACC_DATA
{
	int16_t acc_x;
	int16_t acc_y;
	int16_t acc_z;
};

struct GYRO_DATA
{
	int16_t gyro_x;
	int16_t gyro_y;
	int16_t gyro_z;
};

/**************************16488页地址************************************/
// first page
//#define ADIS_REG_PAGE_ID 		 	0x00
#define ADIS_REG_SEQ_CNT 			0x06
#define ADIS_REG_SYS_E_FLAG		0x08
#define ADIS_REG_DIAG_STS			0x0A
#define ADIS_REG_ALM_STS			0x0C
#define ADIS_REG_TEMP_OUT			0x0E
#define ADIS_REG_X_GYRO_LOW  	0x10
#define ADIS_REG_X_GYRO_OUT 	0x12
#define ADIS_REG_Y_GYRO_LOW		0x14
#define ADIS_REG_Y_GYRO_OUT		0x16
#define ADIS_REG_Z_GYRO_LOW		0x18
#define ADIS_REG_Z_GYRO_OUT		0x1A
#define ADIS_REG_X_ACCL_LOW		0x1C
#define ADIS_REG_X_ACCL_OUT		0x1E
#define ADIS_REG_Y_ACCL_LOW		0x20
#define ADIS_REG_Y_ACCL_OUT		0x22
#define ADIS_REG_Z_ACCL_LOW		0x24
#define ADIS_REG_Z_ACCL_OUT		0x26
#define ADIS_REG_X_MAGN_OUT		0x28
#define ADIS_REG_Y_MAGN_OUT		0x2A
#define ADIS_REG_Z_MAGN_OUT		0x2C	
#define ADIS_REG_BAROM_LOW		0x2E
#define ADIS_REG_BAROM_OUT		0x30
#define ADIS_REG_X_DELTANG_LOW	0x40
#define ADIS_REG_X_DELTANG_OUT	0x42
#define ADIS_REG_Y_DELTANG_LOW	0x44
#define ADIS_REG_Y_DELTANG_OUT	0x46
#define ADIS_REG_Z_DELTANG_LOW	0x48
#define ADIS_REG_Z_DELTANG_OUT	0x4A
#define ADIS_REG_X_DELVEL_LOW		0x4C
#define ADIS_REG_X_DELVEL_OUT		0x4E
#define ADIS_REG_Y_DELVEL_LOW		0x50
#define ADIS_REG_Y_DELVEL_OUT		0x52
#define ADIS_REG_Z_DELVEL_LOW		0x54
#define ADIS_REG_Z_DELVEL_OUT		0x56
#define ADIS_REG_TIME_MS_OUT		0x78
#define ADIS_REG_TIME_DH_OUT		0x7A
#define ADIS_REG_TIME_YM_OUT		0x7C
#define ADIS_REG_PROD_ID				0x7E

// second page: page 0x02
// #define ADIS_REG_PAGE_ID				0x00
#define ADIS_REG_X_GYRO_SCALE		0x04
#define ADIS_REG_Y_GYRO_SCALE 	0x06
#define ADIS_REG_Z_GYRO_SCALE		0x08
#define ADIS_REG_X_ACCL_SCALE		0x0A
#define ADIS_REG_Y_ACCL_SCALE		0x0C
#define ADIS_REG_Z_ACCL_SCALE 	0x0E
#define ADIS_REG_XG_BIAS_LOW		0x10
#define ADIS_REG_XG_BIAS_HIGH		0x12
#define ADIS_REG_YG_BIAS_LOW		0x14
#define ADIS_REG_YG_BIAS_HIGH		0x16
#define ADIS_REG_ZG_BIAS_LOW		0x18
#define ADIS_REG_ZG_BIAS_HIGH		0x1A
#define ADIS_REG_XA_BIAS_LOW		0x1C
#define ADIS_REG_XA_BIAS_HIGH		0x1E
#define ADIS_REG_YA_BIAS_LOW		0x20
#define ADIS_REG_YA_BIAS_HIGH		0x22
#define ADIS_REG_ZA_BIAS_LOW		0x24
#define ADIS_REG_ZA_BIAS_HIGH		0x26
#define ADIS_REG_HARD_IRON_X		0x28
#define ADIS_REG_HARD_IRON_Y		0x2A
#define ADIS_REG_HARD_IRON_Z		0x2C
#define ADIS_REG_SOFT_IRON_S11	0x2E
#define ADIS_REG_SOFT_IRON_S12	0x30
#define ADIS_REG_SOFT_IRON_S13 	0x32
#define ADIS_REG_SOFT_IRON_S21	0x34
#define ADIS_REG_SOFT_IRON_S22 	0x36
#define ADIS_REG_SOFT_IRON_S23	0x38
#define ADIS_REG_SOFT_IRON_S31	0x3A
#define ADIS_REG_SOFT_IRON_S32	0x3c
#define ADIS_REG_SOFT_IRON_S33	0x3E
#define ADIS_REG_BR_BIAS_LOW		0x40
#define ADIS_REG_BR_BIAS_HIGH		0x42
#define ADIS_REG_USR_SCR_1			0x74
#define ADIS_REG_USR_SCR_2			0x76
#define ADIS_REG_USR_SCR_3			0x78
#define ADIS_REG_USR_SCR_4			0x7A
#define ADIS_REG_FLSHCNT_LOW		0x7C
#define ADIS_REG_FLSHCNT_HIGH		0x7E

// page 3
// #define ADIS_REG_PAGE_ID				0x00
#define ADIS_REG_GLOB_CMD				0x02
#define ADIS_REG_FNCTIO_CTRL		0x06
#define ADIS_REG_GPIO_CTRL			0x08
#define ADIS_REG_CONFIG					0x0A
#define ADIS_REG_DEC_RATE				0x0C
#define ADIS_REG_NULL_CNFG			0x0E
#define ADIS_REG_SLP_CNT				0x10
#define ADIS_REG_FILTR_BNK0			0x16
#define ADIS_REG_FILTR_BNK1			0x18
#define ADIS_REG_ALM_CNFG_0			0x20
#define ADIS_REG_ALM_CNFG_1			0x22
#define ADIS_REG_ALM_CNFG_2			0x24
#define ADIS_REG_XG_ALM_MAGN		0x28
#define ADIS_REG_YG_ALM_MAGN		0x2A
#define ADIS_REG_ZG_ALM_MAGN		0x2C
#define ADIS_REG_XA_ALM_MAGN		0x2E
#define ADIS_REG_YA_ALM_MAGN		0x30
#define ADIS_REG_ZA_ALM_MAGN		0x32
#define ADIS_REG_XM_ALM_MAGN		0x34
#define ADIS_REG_YM_ALM_MAGN		0x36
#define ADIS_REG_ZM_ALM_MAGN		0x38
#define ADIS_REG_BR_ALM_MAGN		0x3A
#define ADIS_REG_FIRM_REV				0x78
#define ADIS_REG_FIRM_DM				0x7A
#define ADIS_REG_FIRM_Y					0x7C

// page 4
#define ADIS_REG_SERIAL_NUM			0x20

// page 5
// #define ADIS_REG_PAGE_ID				0x00
//#define ADIS_REG_FIR_COEF_Axxx		0x


 /************************SPI接口操作******************************/
#define SET_CS()		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET)	//PA.4->/CS
#define CLR_CS()		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET)

#define	SET_SCL()		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET)	//PA.5->SCLK
#define	CLR_SCL()		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET)

#define SET_SDO()		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET)	 //PA.7->DIN
#define CLR_SDO()		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET)



// Write to reg address to SPI 
void WriteADIInt16(uint16_t regAddr);

// Read data from SPI
int16_t ReadSPIInt16(void);

// read ADI register data
int16_t ReadADIint16(uint16_t RegAddr);

// read ADI register data
int32_t ReadADIint32(uint16_t RegAddrLow, uint16_t RegAddrHigh);

// write control register
void WriteADIControlReg(unsigned char page, unsigned char address);

// init ADIS16488
void ADIS16488_Init(void);

// 
struct ACC_DATA ReadAccData(void);

// 
struct GYRO_DATA ReadGyroData(void);

#endif

//------------------End of File----------------------------
