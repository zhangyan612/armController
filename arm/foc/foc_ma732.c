#include "mt6825.h"
#include "main.h"
#include "spi.h"

#define arraysize         3

#define MT6816
//#define MT6825

uint16_t spi0_send_array[arraysize] = {0x8300, 0x8400, 0x8500 };
uint16_t spi0_receive_array[arraysize]; 

uint32_t MT_angle=0;

void Delay( uint16_t i )
{
   while( i-- );
}

uint16_t SPIx_ReadWriteByte(uint16_t byte)
{
	uint16_t retry = 0;
	while( (SPI1->SR&1<<1) == 0 )//ï¿½ï¿½ï¿½Í»ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ç¿ï¿½
	{
		if( ++retry > 200 )
			return 0;//ï¿½Ó³ï¿½Ò»ï¿½ï¿½Ê±ï¿½ï¿½ó·µ»ï¿½
	}
	SPI1->DR = byte;     //ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
	
	retry = 0;
	while( (SPI1->SR&1<<0) == 0 ) //ï¿½ï¿½ï¿½Õ»ï¿½ï¿½ï¿½ï¿½ï¿½Îªï¿½ï¿½
	{
		if( ++retry > 200 )
			return 0;//ï¿½Ó³ï¿½Ò»ï¿½ï¿½Ê±ï¿½ï¿½ó·µ»ï¿½
	}
	return SPI1->DR;          //ï¿½ï¿½Ò»ï¿½Â»ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ö¾
}



uint32_t ReadValue(uint32_t u32RegValue)
{
	uint32_t u32Data;

	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port,SPI1_CS_Pin,GPIO_PIN_RESET);

	spi0_receive_array[0] = SPIx_ReadWriteByte(spi0_send_array[0]);
	
	spi0_receive_array[1] = SPIx_ReadWriteByte(spi0_send_array[1]);
	
	#ifdef MT6825
	spi0_receive_array[2] = SPIx_ReadWriteByte(spi0_send_array[2]);
	#endif
	
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port,SPI1_CS_Pin,GPIO_PIN_SET);	
	
	#ifdef MT6825
	u32Data=(spi0_receive_array[0]*1024)+((spi0_receive_array[1]&0xfc)*4)+((spi0_receive_array[2]&0xf0)>>4); //for MT6825
	#endif
	
	#ifdef MT6816
	u32Data=(spi0_receive_array[0]*64)+((spi0_receive_array[1]&0xfc)>>2); // for MT6816
	#endif
	
	return(u32Data);
}

uint16_t ReadMA732Angle(void)
{
    uint16_t angle_raw;
    HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
    angle_raw = SPIx_ReadWriteByte(0x0000); // Send dummy data to read
    HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);
    return angle_raw & 0x3FFF; // 14-bit angle
}

