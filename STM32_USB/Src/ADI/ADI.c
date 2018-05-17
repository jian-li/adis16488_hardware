/********************************************************************
名称：ADI.c
作者：GNC430_Tianz, LiJian 
功能：stm32f103与ADIS16488之间的
      spi配置以及数据交互接口
------------------------------------
 ********************************************************************/

#include "ADI.h"

/********************************************************************
  	
F103与ADIS16488接口：(SPI1) 

	PA4--NSS--CS
	PA5--SCK--SCLK
  PA6--MISO--DOUT
	PA7--MOSI--DIN	
	
*********************************************************************/

/*
 * Init SPI
 */

void Initial_SPI(void)
{
		GPIO_InitTypeDef GPIO_InitStruct;

		/* GPIO Ports Clock Enable */
		__HAL_RCC_GPIOA_CLK_ENABLE();

		/*Configure GPIO pin Output Level */
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);

		/*Configure GPIO pin : SPI_Pins */
		GPIO_InitStruct.Pin = GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

// 
void Init_Sync(void)
{
		// P for ADIS data ready signal
	
		// P for ADIS sync input clock
			
}

void ADIS16488_Init(void)    //SPI初始化
{											 
		// init spi
		Initial_SPI();
		
		// power-on start up time
		delay_us(60000);
	
		// init stm32 and adis synchronization configure
		Init_Sync();
}

/*
 * Write to ADI int 16 data
 * write to the lower 8 bits first, then write to the hight 8 bits
 */
void WriteSPIInt16(uint16_t RegisterData)
{
		unsigned char	lower_byte = 0;
		unsigned char higher_byte = 0;
		unsigned char	i = 0;
	
		lower_byte = RegisterData&0x00ff; //lower bye of reg
		higher_byte = (RegisterData&0xff00)>>8;
	
		SET_CS();
		SET_SCL();
		delay_us(1);
		CLR_CS();	 //bring CS low, select
		delay_us(1);
	
		//write the lower 8 bits
		for(i=0; i<8; i++)
		{
				CLR_SCL();						//write to MISO on rising edge
				if(0x80 == (lower_byte & 0x80))
				{
					SET_SDO();	  //Send one to DIN pin	of ADIS16405
				}
				else
				{
					CLR_SDO();	  //Send zero to DIN pin of ADIS16405
				}
				delay_us(1);
				SET_SCL();
				delay_us(1);
				lower_byte <<= 1;	//Rotate data
		}
		// write the higher 8 bits
		for(i=0; i<8; i++)
		{
				CLR_SCL();						//write to MISO on rising edge
				if(0x80 == (higher_byte & 0x80))
				{
					SET_SDO();	  //Send one to DIN pin	of ADIS16405
				}
				else
				{
					CLR_SDO();	  //Send zero to DIN pin of ADIS16405
				}
				delay_us(1);
				SET_SCL();
				delay_us(1);
				higher_byte <<= 1;	//Rotate data
		}
		delay_us(1);
		
		//
		SET_CS();
}

/*
 * Read From ADI int16 data
 */
int16_t ReadSPIInt16(void)
{
		unsigned char	i = 0;
		int16_t 	ReceiveData = 0;
		char		iTemp = 0;			//8位
		SET_CS();
		SET_SCL();
		delay_us(1);
		CLR_CS();	 //bring CS low, select
		delay_us(1);
		
		//Read data in
		for(i=0; i<16; i++)
		{
				CLR_SCL();					
				delay_us(1);
				ReceiveData <<= 1;		//Rotate data
				iTemp = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6);			//Read DOUT of ADIS16405
				if(iTemp == 1)
				{
						ReceiveData |= 1;	
				}
				SET_SCL();			 //由高变低时读数据
				delay_us(1);
		}
		delay_us(1);
		
		SET_CS();	//bring CS high again
		return ReceiveData;
}

/*
 * Read register data from adi
 */
int16_t ReadADIint16(uint16_t RegAddr)
{
		int16_t 	ReceiveData = 0;
		
		// write the register address
		WriteSPIInt16(RegAddr);
		
		// read the data via spi
		ReceiveData = ReadSPIInt16();
	
		return ReceiveData;	 
}

/*
 * Read 32bit data from ADI 
 */
int32_t ReadADIint32(uint16_t RegAddrLow, uint16_t RegAddrHigh)
{
		int32_t 	ReceiveData = 0; 
		int16_t ReceiveData_low = 0;
		int16_t ReceiveData_high = 0;
		
		// write the register low end address
		WriteSPIInt16(RegAddrLow);
		
		// read the data via spi
		ReceiveData_low = ReadSPIInt16();
	
		// write the register high end address
		WriteSPIInt16(RegAddrHigh);
		
		// read the data via spi
		ReceiveData_high = ReadSPIInt16();
		
		// merge low end and high end 
		ReceiveData = ReceiveData_high <<16 | ReceiveData_low;
	
		return ReceiveData;	 
}

/*
 * Read Acc Data from ADI16488 via SPI
 */
struct ACC_DATA ReadAccData(void)
{
		int16_t acc_x_low_end = 0;
		int16_t acc_x_high_end = 0;
		int16_t acc_y_low_end = 0;
		int16_t acc_y_high_end = 0;
		int16_t acc_z_low_end = 0;
		int16_t acc_z_high_end = 0;
		uint16_t WORD_MAX = 0xffff;
		
		int32_t acc_x_measure = 0;
		int32_t acc_y_measure = 0;
		int32_t acc_z_measure = 0;
	
		struct ACC_DATA acc_data;
	
		// set page_ID to 0x00
		WriteSPIInt16(0x8000);
		
		// read acc_x
		acc_x_high_end = ReadADIint16(ADIS_REG_X_ACCL_OUT);
		acc_x_low_end = ReadADIint16(ADIS_REG_X_ACCL_LOW);
		acc_data.acc_x = acc_x_high_end;//<<16| acc_x_low_end;
		
		// read acc_y 
		acc_y_high_end = ReadADIint16(ADIS_REG_Y_ACCL_OUT);
		acc_y_low_end = ReadADIint16(ADIS_REG_Y_ACCL_LOW);
		acc_data.acc_y = acc_y_high_end;//<<16| acc_y_low_end;
		
		// read acc_z
		acc_z_high_end = ReadADIint16(ADIS_REG_Z_ACCL_OUT);
		acc_z_low_end = ReadADIint16(ADIS_REG_Z_ACCL_LOW);
		acc_data.acc_z = acc_z_high_end;//<<16| acc_z_low_end;

		return acc_data;
}

/*
 * Read Gyro Data from ADI16488 via SPI
 */
struct GYRO_DATA ReadGyroData(void)
{
		int16_t gyro_x_low_end = 0;
		int16_t gyro_x_high_end = 0;
		int16_t gyro_y_low_end = 0;
		int16_t gyro_y_high_end = 0;
		int16_t gyro_z_low_end = 0;
		int16_t gyro_z_high_end = 0;
		uint16_t WORD_MAX = 0xffff;
	
		int32_t gyro_x_measure = 0;
		int32_t gyro_y_measure = 0;
		int32_t gyro_z_measure = 0;
		
		struct GYRO_DATA gyro_data;
	
		// set page_ID to 0x00
		WriteSPIInt16(0x8000);
		
		// read acc_x
		gyro_x_high_end = ReadADIint16(ADIS_REG_X_GYRO_OUT);
		gyro_x_low_end = ReadADIint16(ADIS_REG_X_GYRO_LOW);
		gyro_data.gyro_x = gyro_x_high_end;//<<16| gyro_x_low_end;
		
		// read acc_y 
		gyro_y_high_end = ReadADIint16(ADIS_REG_Y_GYRO_OUT);
		gyro_y_low_end = ReadADIint16(ADIS_REG_Y_GYRO_LOW);
		gyro_data.gyro_y = gyro_y_high_end;//<<16| gyro_y_low_end;
		
		// read acc_z
		gyro_z_high_end = ReadADIint16(ADIS_REG_Z_GYRO_OUT);
		gyro_z_low_end = ReadADIint16(ADIS_REG_Z_GYRO_LOW);
		gyro_data.gyro_z = gyro_z_high_end;//<<16| gyro_z_low_end;
	
		// convert acc raw to m/s^2
//		gyro_data.gyro_x = gyro_x_measure*0.02;///WORD_MAX;
//		gyro_data.gyro_y = gyro_y_high_end*0.02;///WORD_MAX;
//		gyro_data.gyro_z = gyro_z_high_end*0.02;///WORD_MAX;
		
		return gyro_data;
}

//------------------End of File----------------------------
