#include "sys.h"
#include "delay.h"
#include "modbus.h"	
#include "crc_16.h"
#include "usart.h"
#include "mcpwm.h"

//////////////////////////////////////////////////////////////////
//¼ÓÈëÒÔÏÂ´úÂë,Ö§³Öprintfº¯Êý,¶ø²»ÐèÒªÑ¡Ôñuse MicroLIB	  
#pragma import(__use_no_semihosting)             
//±ê×¼¿âÐèÒªµÄÖ§³Öº¯Êý                 
struct __FILE 
{ 
	int handle; 

}; 

FILE __stdout;       
//¶¨Òå_sys_exit()ÒÔ±ÜÃâÊ¹ÓÃ°ëÖ÷»úÄ£Ê½    
void _sys_exit(int x) 
{ 
	x = x; 
} 
//ÖØ¶¨Òåfputcº¯Êý 
int fputc(int ch, FILE *f)
{      
	while((USART3->SR&0X40)==0); //Ñ­»··¢ËÍ,Ö±µ½·¢ËÍÍê±Ï   
    
	USART3->DR = (u8) ch;      
	return ch;
}
//½ÓÊÕ×´Ì¬
//bit15£¬	½ÓÊÕÍê³É±êÖ¾
//bit14£¬	½ÓÊÕµ½0x0d
//bit13~0£¬	½ÓÊÕµ½µÄÓÐÐ§×Ö½ÚÊýÄ¿
u16 USART3_RX_STA=0;       //½ÓÊÕ×´Ì¬±ê¼Ç	  
u32 USART3_RX_TIMECHK;
u8 USART3RxBuffer[RXBUFFERSIZE];//HAL¿âÊ¹ÓÃµÄ´®¿Ú½ÓÊÕ»º³å

u16 USART2_RX_STA=0;       //½ÓÊÕ×´Ì¬±ê¼Ç	  
u32 USART2_RX_TIMECHK;
u8 USART2RxBuffer[RXBUFFERSIZE];//HAL¿âÊ¹ÓÃµÄ´®¿Ú½ÓÊÕ»º³å


uint8_t RS485_FrameFlag=0;
uint8_t RS485_TX_EN=0;
uint8_t RS485_RX_CNT=0;
uint8_t RS485_RX_BUFF[USART3_REC_LEN];
uint8_t RS485_TX_BUFF[USART3_REC_LEN];

uint8_t RS232_FrameFlag=0;
uint8_t RS232_TX_EN=0;
uint8_t RS232_RX_CNT=0;
uint8_t RS232_RX_BUFF[USART2_REC_LEN];
uint8_t RS232_TX_BUFF[USART2_REC_LEN];

u16 RS232_Addr=1;
u32 RS232_Baudrate=38400;
u16 RS232_Protocol=0;

u16 RS485_Addr=1;
u16 Modbus_Addr_Base=0;
u32 RS485_Baudrate=38400;
u16 RS485_Protocol=0;
uint16_t startRegAddr=0;
uint16_t RegNum;
uint16_t calCRC;
uint16_t *Modbus_Output_Reg[MODBUS_REG_NUM];
u32 software_version;
  
//³õÊ¼»¯IO ´®¿Ú1 
//bound:²¨ÌØÂÊ
void uart3_init(u32 baudrate)
{	
	//UART ³õÊ¼»¯ÉèÖÃ
	huart3.Instance=USART3;					    //USART3
	huart3.Init.BaudRate=baudrate;				    //²¨ÌØÂÊ
	huart3.Init.WordLength=UART_WORDLENGTH_8B;   //×Ö³¤Îª8Î»Êý¾Ý¸ñÊ½
	huart3.Init.StopBits=UART_STOPBITS_1;	    //Ò»¸öÍ£Ö¹Î»
	huart3.Init.Parity=UART_PARITY_NONE;		    //ÎÞÆæÅ¼Ð£ÑéÎ»
	huart3.Init.HwFlowCtl=UART_HWCONTROL_NONE;   //ÎÞÓ²¼þÁ÷¿Ø
	huart3.Init.Mode=UART_MODE_TX_RX;		    //ÊÕ·¢Ä£Ê½
	HAL_UART_Init(&huart3);					    //HAL_UART_Init()»áÊ¹ÄÜuart3
	
	//HAL_UART_Receive_IT(&huart3, (u8 *)USART3RxBuffer, RXBUFFERSIZE);//¸Ãº¯Êý»á¿ªÆô½ÓÊÕÖÐ¶Ï£º±êÖ¾Î»UART_IT_RXNE£¬²¢ÇÒÉèÖÃ½ÓÊÕ»º³åÒÔ¼°½ÓÊÕ»º³å½ÓÊÕ×î´óÊý¾ÝÁ¿
   __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);
	HAL_UART_Receive_DMA(&huart3, (uint8_t*)RS485_RX_BUFF, 1);     //ÉèÖÃDMA´«Êä£¬½«´®¿Ú1µÄÊý¾Ý°áÔËµ½recvive_buffÖÐ£¬Ã¿´Î255¸ö×Ö½Ú

}

void uart2_init(u32 baudrate)
{	
	//UART ³õÊ¼»¯ÉèÖÃ
	huart2.Instance=USART2;					    //USART2
	huart2.Init.BaudRate=baudrate;				    //²¨ÌØÂÊ
	huart2.Init.WordLength=UART_WORDLENGTH_8B;   //×Ö³¤Îª8Î»Êý¾Ý¸ñÊ½
	huart2.Init.StopBits=UART_STOPBITS_1;	    //Ò»¸öÍ£Ö¹Î»
	huart2.Init.Parity=UART_PARITY_NONE;		    //ÎÞÆæÅ¼Ð£ÑéÎ»
	huart2.Init.HwFlowCtl=UART_HWCONTROL_NONE;   //ÎÞÓ²¼þÁ÷¿Ø
	huart2.Init.Mode=UART_MODE_TX_RX;		    //ÊÕ·¢Ä£Ê½
	HAL_UART_Init(&huart2);					    //HAL_UART_Init()»áÊ¹ÄÜuart3
	
	//HAL_UART_Receive_IT(&huart2, (u8 *)USART2RxBuffer, RXBUFFERSIZE);//¸Ãº¯Êý»á¿ªÆô½ÓÊÕÖÐ¶Ï£º±êÖ¾Î»UART_IT_RXNE£¬²¢ÇÒÉèÖÃ½ÓÊÕ»º³åÒÔ¼°½ÓÊÕ»º³å½ÓÊÕ×î´óÊý¾ÝÁ¿
  __HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);
	HAL_UART_Receive_DMA(&huart2, (uint8_t*)RS232_RX_BUFF, 1);     //ÉèÖÃDMA´«Êä£¬½«´®¿Ú1µÄÊý¾Ý°áÔËµ½recvive_buffÖÐ£¬Ã¿´Î255¸ö×Ö½Ú

}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  char res;
	if(huart->Instance==USART1)
	{
		switch(feedback_type)
		{
			case 4:
				Tamagawa_calCRC=Tamagawa_RX_BUFF[0]^Tamagawa_RX_BUFF[1]^Tamagawa_RX_BUFF[2]^Tamagawa_RX_BUFF[3]^Tamagawa_RX_BUFF[4]^Tamagawa_RX_BUFF[5];
				if(Tamagawa_calCRC==0)
				{
					Tamagawa_count_temp=0;
					tamagawa_angle_32=Tamagawa_RX_BUFF[4]*0x10000+Tamagawa_RX_BUFF[3]*0x100+Tamagawa_RX_BUFF[2];
					tamagawa_angle=tamagawa_angle_32>>2;
					
//					tamagawa_angle_delta_4=(tamagawa_angle-tamagawa_angle_4)%32768;
//					if(Tamagawa_First==10)
//					if((tamagawa_angle_delta_4>2000)||(tamagawa_angle_delta_4<(-2000)))
//						tamagawa_angle=tamagawa_angle_4;
//					
					tamagawa_angle_4=tamagawa_angle;
//					
//					if(tamagawa_angle_delta_3==0)
//						shot=0;
//					if((tamagawa_angle_delta>tamagawa_angle_delta_3)&&(shot==0))
//					{
//						tamagawa_angle_delta_3=tamagawa_angle_delta;
//						if((tamagawa_angle_delta_3>40000)&&(shot==0))
//						{
//							//shot=1;
//							tamagawa_angle=tamagawa_angle_1;
//							tamagawa_angle_delta_1=tamagawa_angle-tamagawa_angle_1;
//							tamagawa_angle_delta_2=tamagawa_angle_1-tamagawa_angle_2;
//							tamagawa_angle_delta_3=tamagawa_angle_2-tamagawa_angle_3;
//							
//							tamagawa_angle_3=tamagawa_angle_2;
//							tamagawa_angle_2=tamagawa_angle_1;
//							tamagawa_angle_1=tamagawa_angle;
//							
//							Tamagawa_RX_BUFF_B[0]=Tamagawa_RX_BUFF[0];
//							Tamagawa_RX_BUFF_B[1]=Tamagawa_RX_BUFF[1];
//							Tamagawa_RX_BUFF_B[2]=Tamagawa_RX_BUFF[2];
//							Tamagawa_RX_BUFF_B[3]=Tamagawa_RX_BUFF[3];
//							Tamagawa_RX_BUFF_B[4]=Tamagawa_RX_BUFF[4];
//							Tamagawa_RX_BUFF_B[5]=Tamagawa_RX_BUFF[5];
//							Tamagawa_RX_BUFF_B[6]=Tamagawa_RX_BUFF[6];
//						}
//					}
//					if(shot==0)
//					{
//						tamagawa_angle_3=tamagawa_angle_2;
//						tamagawa_angle_2=tamagawa_angle_1;
//						tamagawa_angle_1=tamagawa_angle;
//					}
//					
					if(Tamagawa_First<10)
						Tamagawa_First++;
						//tamagawa_angle=Tamagawa_RX_BUFF[3]*0x40+(Tamagawa_RX_BUFF[2]>>2);
				}
				else
				{
					Tamagawa_CRC_count++;
				}
			break;
			case 5:
				Tamagawa_calCRC=Tamagawa_RX_BUFF[0]^Tamagawa_RX_BUFF[1]^Tamagawa_RX_BUFF[2]^Tamagawa_RX_BUFF[3]^Tamagawa_RX_BUFF[4]^Tamagawa_RX_BUFF[5]^Tamagawa_RX_BUFF[6]^Tamagawa_RX_BUFF[7]^Tamagawa_RX_BUFF[8]^Tamagawa_RX_BUFF[9]^Tamagawa_RX_BUFF[10];
				if(Tamagawa_calCRC==0)
				{
					Tamagawa_count_temp=0;
					tamagawa_angle_32=Tamagawa_RX_BUFF[4]*0x10000+Tamagawa_RX_BUFF[3]*0x100+Tamagawa_RX_BUFF[2];
					tamagawa_angle=tamagawa_angle_32>>2;
					tamagawa_multi_turn=Tamagawa_RX_BUFF[6]+Tamagawa_RX_BUFF[7]*0x100+Tamagawa_RX_BUFF[8]*0x10000;
					tamagawa_ENID=Tamagawa_RX_BUFF[5];
					tamagawa_ALMC=Tamagawa_RX_BUFF[9];
					if(Tamagawa_First<10)
						Tamagawa_First++;
						//tamagawa_angle=Tamagawa_RX_BUFF[3]*0x40+(Tamagawa_RX_BUFF[2]>>2);
				}
				else
				{
					Tamagawa_CRC_count++;
				}
			break;
				
		}
	}
	if(huart->Instance==USART2)//Èç¹ûÊÇ´®¿Ú3
	{
			if((USART2_RX_STA&0x8000)==0)//½ÓÊÕÎ´Íê³É
			{
				USART2_RX_TIMECHK=HAL_GetTick();
				RS232_RX_BUFF[USART2_RX_STA&0X3FFF]=USART2RxBuffer[0] ;
				USART2_RX_STA++;
				if(USART2_RX_STA>(USART2_REC_LEN-1))USART2_RX_STA=0;//½ÓÊÕÊý¾Ý´íÎó,ÖØÐÂ¿ªÊ¼½ÓÊÕ	  	 
			}
	}
	if(huart->Instance==USART3)//Èç¹ûÊÇ´®¿Ú3
	{
		if(RS485_TX_EN==0)
			if((USART3_RX_STA&0x8000)==0)//½ÓÊÕÎ´Íê³É
			{
				USART3_RX_TIMECHK=HAL_GetTick();
				RS485_RX_BUFF[USART3_RX_STA&0X3FFF]=USART3RxBuffer[0] ;
				USART3_RX_STA++;
				if(USART3_RX_STA>(USART3_REC_LEN-1))USART3_RX_STA=0;//½ÓÊÕÊý¾Ý´íÎó,ÖØÐÂ¿ªÊ¼½ÓÊÕ	  	 
			}
	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance==USART3)//Èç¹ûÊÇ´®¿Ú3
	{
		Modbus_Solve_485_Disenable();	
			// ÖØÆô¿ªÊ¼DMA´«Êä Ã¿´Î255×Ö½ÚÊý¾Ý
		HAL_UART_Receive_DMA(&huart3, (uint8_t*)RS485_RX_BUFF, USART3_REC_LEN);  
	}		
	if(huart->Instance==USART2)//Èç¹ûÊÇ´®¿Ú2
	{
			// ÖØÆô¿ªÊ¼DMA´«Êä Ã¿´Î255×Ö½ÚÊý¾Ý
		HAL_UART_Receive_DMA(&huart2, (uint8_t*)RS232_RX_BUFF, USART2_REC_LEN);  
	}		

}
void RS485_Process(void)
{
		if(((USART3_RX_STA&0x8000)==0x0)&&(USART3_RX_STA&0X3FFF)>0)
		{
			if((HAL_GetTick()-USART3_RX_TIMECHK)>USART3_RX_TIEMOUT)
			{
				USART3_RX_STA|=0x8000;
				RS485_FrameFlag=1;
				RS485_RX_CNT=USART3_RX_STA&0X3FFF;
				
			}
		}
}

void RS232_Process(void)
{
		if(((USART2_RX_STA&0x8000)==0x0)&&(USART2_RX_STA&0X3FFF)>0)
		{
			if((HAL_GetTick()-USART2_RX_TIMECHK)>USART2_RX_TIEMOUT)
			{
				USART2_RX_STA|=0x8000;
				RS232_FrameFlag=1;
				RS232_RX_CNT=USART2_RX_STA&0X3FFF;
				
			}
		}
}

void Modbus_Solve_485_Enable(void)
{
  HAL_GPIO_WritePin(RS485_EN_GPIO_Port,RS485_EN_Pin,GPIO_PIN_SET);	
}

void Modbus_Solve_485_Disenable(void)
{
	int i;
  //for(i=0;i<3550;i++){}//ÇÐ»»ÑÓÊ±
  HAL_GPIO_WritePin(RS485_EN_GPIO_Port,RS485_EN_Pin,GPIO_PIN_RESET);	
}

void Modbus_Solve_PutString(uint8_t *buf,uint16_t len)
{
		uint16_t i;
	RS485_TX_EN=1;
   Modbus_Solve_485_Enable();
		for(i=0;i<len;i++)
		{
			while((USART3->SR&0X40)==0); //Ñ­»··¢ËÍ,Ö±µ½·¢ËÍÍê±Ï   
			USART3->DR = buf[i]; 
		} 
  Modbus_Solve_485_Disenable();	
	RS485_TX_EN=0;		
}
void RS232_Solve_PutString(uint8_t *buf,uint16_t len)
{
		uint16_t i;
	RS232_TX_EN=1;
		for(i=0;i<len;i++)
		{
			while((USART2->SR&0X40)==0); //Ñ­»··¢ËÍ,Ö±µ½·¢ËÍÍê±Ï   
			USART2->DR = buf[i]; 
		} 
	RS232_TX_EN=0;		
}

void Modbus_Solve_Service(void)
{
	uint16_t recCRC;
	
	if(RS485_FrameFlag==1)
	{
		if(RS485_RX_CNT>5)
		{
			if((RS485_RX_BUFF[0]==RS485_Addr)||(RS485_RX_BUFF[0]==255))
			{
				if((RS485_RX_BUFF[1]==03)||(RS485_RX_BUFF[1]==06)||(RS485_RX_BUFF[1]==16))
				{ 
					startRegAddr=(((uint16_t)RS485_RX_BUFF[2])<<8)|RS485_RX_BUFF[3];
					startRegAddr=startRegAddr-Modbus_Addr_Base;
					if(startRegAddr<MODBUS_REG_NUM)
					{
						calCRC=Get_Crc16(RS485_RX_BUFF,RS485_RX_CNT-2);
						recCRC=(((uint16_t)RS485_RX_BUFF[RS485_RX_CNT-1])<<8)|RS485_RX_BUFF[RS485_RX_CNT-2];                                   
						if(calCRC==recCRC)
						{
							switch(RS485_RX_BUFF[1])
							{   
								case 03: 
								{       
									if(RS485_RX_BUFF[0]!=0)
									{
									 Modbus_03_Solve();
									}
									break;
								}
								case 06: 
								{
									Modbus_06_Solve();
									break;
								}
								case 16: 
								{
									Modbus_16_Solve();
									break;
								} 
								default:
									Modbus_Solve_485_Disenable();	
										// ÖØÆô¿ªÊ¼DMA´«Êä Ã¿´Î255×Ö½ÚÊý¾Ý
									HAL_UART_Receive_DMA(&huart3, (uint8_t*)RS485_RX_BUFF, USART3_REC_LEN);  
									break; 

							}                                          
						} 
						else
						{
							Modbus_Solve_485_Disenable();	
								// ÖØÆô¿ªÊ¼DMA´«Êä Ã¿´Î255×Ö½ÚÊý¾Ý
							HAL_UART_Receive_DMA(&huart3, (uint8_t*)RS485_RX_BUFF, USART3_REC_LEN);   
						}      
					}
					else
					{
						Modbus_Solve_485_Disenable();	
							// ÖØÆô¿ªÊ¼DMA´«Êä Ã¿´Î255×Ö½ÚÊý¾Ý
						HAL_UART_Receive_DMA(&huart3, (uint8_t*)RS485_RX_BUFF, USART3_REC_LEN);   
					}
				}
				else
				{
					Modbus_Solve_485_Disenable();	
						// ÖØÆô¿ªÊ¼DMA´«Êä Ã¿´Î255×Ö½ÚÊý¾Ý
					HAL_UART_Receive_DMA(&huart3, (uint8_t*)RS485_RX_BUFF, USART3_REC_LEN);   
				}
			}
			else
			{
				Modbus_Solve_485_Disenable();	
					// ÖØÆô¿ªÊ¼DMA´«Êä Ã¿´Î255×Ö½ÚÊý¾Ý
				HAL_UART_Receive_DMA(&huart3, (uint8_t*)RS485_RX_BUFF, USART3_REC_LEN);   
			}
			
		}
		else
		{
			Modbus_Solve_485_Disenable();	
				// ÖØÆô¿ªÊ¼DMA´«Êä Ã¿´Î255×Ö½ÚÊý¾Ý
			HAL_UART_Receive_DMA(&huart3, (uint8_t*)RS485_RX_BUFF, USART3_REC_LEN);   
		}

			RS485_FrameFlag=0;
			USART3_RX_STA=0;
			RS485_RX_CNT=0;
			RS485_TX_EN=0;
	}


}

void Modbus_03_Solve(void)
{
	uint8_t i;
	RegNum= (((uint16_t)RS485_RX_BUFF[4])<<8)|RS485_RX_BUFF[5];
	if((startRegAddr+RegNum)<MODBUS_REG_NUM)
	{
		RS485_TX_BUFF[0]=RS485_RX_BUFF[0];
		RS485_TX_BUFF[1]=RS485_RX_BUFF[1];
		RS485_TX_BUFF[2]=RegNum*2;
		for(i=0;i<RegNum;i++)
		{
			RS485_TX_BUFF[3+i*2]=(*Modbus_Output_Reg[startRegAddr+i]>>8)&0xFF;
			RS485_TX_BUFF[4+i*2]=*Modbus_Output_Reg[startRegAddr+i]&0xFF;
		}
		calCRC=Get_Crc16(RS485_TX_BUFF,RegNum*2+3);
		RS485_TX_BUFF[RegNum*2+3]=calCRC&0xFF;
		RS485_TX_BUFF[RegNum*2+4]=(calCRC>>8)&0xFF;
		HAL_UART_Transmit_DMA(&huart3,RS485_TX_BUFF,RegNum*2+5);
	}
	else
	{
		RS485_TX_BUFF[0]=RS485_RX_BUFF[0];
		RS485_TX_BUFF[1]=RS485_RX_BUFF[1]|0x80;
		RS485_TX_BUFF[2]=0x02; 
		HAL_UART_Transmit_DMA(&huart3,RS485_TX_BUFF,3);
	}
}



void Modbus_06_Solve(void)
{
	*Modbus_Output_Reg[startRegAddr]=RS485_RX_BUFF[5];
	*Modbus_Output_Reg[startRegAddr]|=((uint16_t)RS485_RX_BUFF[4])<<8;

	RS485_TX_BUFF[0]=RS485_RX_BUFF[0];
	RS485_TX_BUFF[1]=RS485_RX_BUFF[1];
	RS485_TX_BUFF[2]=RS485_RX_BUFF[2];
	RS485_TX_BUFF[3]=RS485_RX_BUFF[3];
	RS485_TX_BUFF[4]=RS485_RX_BUFF[4];
	RS485_TX_BUFF[5]=RS485_RX_BUFF[5];
	
	calCRC=Get_Crc16(RS485_TX_BUFF,6);
	RS485_TX_BUFF[6]=calCRC&0xFF;			
	RS485_TX_BUFF[7]=(calCRC>>8)&0xFF;	

	if(RS485_RX_BUFF[0]!=0)
	{
		HAL_UART_Transmit_DMA(&huart3,RS485_TX_BUFF,8);
	}

}


void Modbus_16_Solve(void)
{
	uint8_t i;

	RegNum= (((uint16_t)RS485_RX_BUFF[4])<<8)|RS485_RX_BUFF[5];
	if((startRegAddr+RegNum)<MODBUS_REG_NUM)
	{
		for(i=0;i<RegNum;i++)
		{
			*Modbus_Output_Reg[startRegAddr+i]=RS485_RX_BUFF[8+i*2];
			*Modbus_Output_Reg[startRegAddr+i]|=((uint16_t)RS485_RX_BUFF[7+i*2])<<8; 
		}
		
			RS485_TX_BUFF[0]=RS485_RX_BUFF[0];
			RS485_TX_BUFF[1]=RS485_RX_BUFF[1];
			RS485_TX_BUFF[2]=RS485_RX_BUFF[2];
			RS485_TX_BUFF[3]=RS485_RX_BUFF[3];
			RS485_TX_BUFF[4]=RS485_RX_BUFF[4];
			RS485_TX_BUFF[5]=RS485_RX_BUFF[5];
			
			calCRC=Get_Crc16(RS485_TX_BUFF,6);
			RS485_TX_BUFF[6]=calCRC&0xFF;						
			RS485_TX_BUFF[7]=(calCRC>>8)&0xFF;	
		
			if(RS485_RX_BUFF[0]!=0)//
			{
				HAL_UART_Transmit_DMA(&huart3,RS485_TX_BUFF,8);
			}
				
	}
	else
	{
			RS485_TX_BUFF[0]=RS485_RX_BUFF[0];
			RS485_TX_BUFF[1]=RS485_RX_BUFF[1]|0x80;
			RS485_TX_BUFF[2]=0x02; 
			if(RS485_RX_BUFF[0]!=0)
			{
				HAL_UART_Transmit_DMA(&huart3,RS485_TX_BUFF,3);
			}
	}
}


void RS232_Solve_Service(void)
{
	uint16_t recCRC;
	
	if(RS232_FrameFlag==1)
	{
		if(RS232_RX_CNT>5)
			if((RS232_RX_BUFF[0]==RS232_Addr)||(RS232_RX_BUFF[0]==255))
			{
				if((RS232_RX_BUFF[1]==03)||(RS232_RX_BUFF[1]==06)||(RS232_RX_BUFF[1]==16))
				{ 
					startRegAddr=(((uint16_t)RS232_RX_BUFF[2])<<8)|RS232_RX_BUFF[3];
					startRegAddr=startRegAddr-Modbus_Addr_Base;
					if(startRegAddr<MODBUS_REG_NUM)
					{
						calCRC=Get_Crc16(RS232_RX_BUFF,RS232_RX_CNT-2);
						recCRC=(((uint16_t)RS232_RX_BUFF[RS232_RX_CNT-1])<<8)|RS232_RX_BUFF[RS232_RX_CNT-2];                                   
						if(calCRC==recCRC)
						{
							switch(RS232_RX_BUFF[1])
							{   
								case 03: 
								{       
									if(RS232_RX_BUFF[0]!=0)
									{
									 RS232_03_Solve();
									}
									break;
								}
								case 06: 
								{
									RS232_06_Solve();
									break;
								}
								case 16: 
								{
									RS232_16_Solve();
									break;
								}  
								default:
										// ÖØÆô¿ªÊ¼DMA´«Êä Ã¿´Î255×Ö½ÚÊý¾Ý
									HAL_UART_Receive_DMA(&huart2, (uint8_t*)RS232_RX_BUFF, USART2_REC_LEN);  
									break;                                                                   
							}                                          
						}
						else
						{
							// ÖØÆô¿ªÊ¼DMA´«Êä Ã¿´Î255×Ö½ÚÊý¾Ý
							HAL_UART_Receive_DMA(&huart2, (uint8_t*)RS232_RX_BUFF, USART2_REC_LEN);  
						}	
					}
					else
					{
						// ÖØÆô¿ªÊ¼DMA´«Êä Ã¿´Î255×Ö½ÚÊý¾Ý
						HAL_UART_Receive_DMA(&huart2, (uint8_t*)RS232_RX_BUFF, USART2_REC_LEN);  
					}	
				}
				else
				{
					// ÖØÆô¿ªÊ¼DMA´«Êä Ã¿´Î255×Ö½ÚÊý¾Ý
					HAL_UART_Receive_DMA(&huart2, (uint8_t*)RS232_RX_BUFF, USART2_REC_LEN);  
				}	
			}
			else
			{
				// ÖØÆô¿ªÊ¼DMA´«Êä Ã¿´Î255×Ö½ÚÊý¾Ý
				HAL_UART_Receive_DMA(&huart2, (uint8_t*)RS232_RX_BUFF, USART2_REC_LEN);  
			}	

			RS232_FrameFlag=0;
			USART2_RX_STA=0;
			RS232_RX_CNT=0;
			RS232_TX_EN=0;
	}


}

void RS232_03_Solve(void)
{
	uint8_t i;
	RegNum= (((uint16_t)RS232_RX_BUFF[4])<<8)|RS232_RX_BUFF[5];
	if((startRegAddr+RegNum)<MODBUS_REG_NUM)
	{
		RS232_TX_BUFF[0]=RS232_RX_BUFF[0];
		RS232_TX_BUFF[1]=RS232_RX_BUFF[1];
		RS232_TX_BUFF[2]=RegNum*2;
		for(i=0;i<RegNum;i++)
		{
			RS232_TX_BUFF[3+i*2]=(*Modbus_Output_Reg[startRegAddr+i]>>8)&0xFF;
			RS232_TX_BUFF[4+i*2]=*Modbus_Output_Reg[startRegAddr+i]&0xFF;
		}
		calCRC=Get_Crc16(RS232_TX_BUFF,RegNum*2+3);
		RS232_TX_BUFF[RegNum*2+3]=calCRC&0xFF;
		RS232_TX_BUFF[RegNum*2+4]=(calCRC>>8)&0xFF;
		HAL_UART_Transmit_DMA(&huart2,RS232_TX_BUFF,RegNum*2+5);
	}
	else
	{
		RS232_TX_BUFF[0]=RS232_RX_BUFF[0];
		RS232_TX_BUFF[1]=RS232_RX_BUFF[1]|0x80;
		RS232_TX_BUFF[2]=0x02; 
		HAL_UART_Transmit_DMA(&huart2,RS232_TX_BUFF,3);
	}
}



void RS232_06_Solve(void)
{
	*Modbus_Output_Reg[startRegAddr]=RS232_RX_BUFF[5];
	*Modbus_Output_Reg[startRegAddr]|=((uint16_t)RS232_RX_BUFF[4])<<8;

	RS232_TX_BUFF[0]=RS232_RX_BUFF[0];
	RS232_TX_BUFF[1]=RS232_RX_BUFF[1];
	RS232_TX_BUFF[2]=RS232_RX_BUFF[2];
	RS232_TX_BUFF[3]=RS232_RX_BUFF[3];
	RS232_TX_BUFF[4]=RS232_RX_BUFF[4];
	RS232_TX_BUFF[5]=RS232_RX_BUFF[5];
	
	calCRC=Get_Crc16(RS232_TX_BUFF,6);
	RS232_TX_BUFF[6]=calCRC&0xFF;			
	RS232_TX_BUFF[7]=(calCRC>>8)&0xFF;	

	if(RS232_RX_BUFF[0]!=0)
	{
	 HAL_UART_Transmit_DMA(&huart2,RS232_TX_BUFF,8);
	}

}


void RS232_16_Solve(void)
{
	uint8_t i;

	RegNum= (((uint16_t)RS232_RX_BUFF[4])<<8)|RS232_RX_BUFF[5];
	if((startRegAddr+RegNum)<MODBUS_REG_NUM)
	{
		for(i=0;i<RegNum;i++)
		{
			*Modbus_Output_Reg[startRegAddr+i]=RS232_RX_BUFF[8+i*2];
			*Modbus_Output_Reg[startRegAddr+i]|=((uint16_t)RS232_RX_BUFF[7+i*2])<<8; 
		}
		
			RS232_TX_BUFF[0]=RS232_RX_BUFF[0];
			RS232_TX_BUFF[1]=RS232_RX_BUFF[1];
			RS232_TX_BUFF[2]=RS232_RX_BUFF[2];
			RS232_TX_BUFF[3]=RS232_RX_BUFF[3];
			RS232_TX_BUFF[4]=RS232_RX_BUFF[4];
			RS232_TX_BUFF[5]=RS232_RX_BUFF[5];
			
			calCRC=Get_Crc16(RS232_TX_BUFF,6);
			RS232_TX_BUFF[6]=calCRC&0xFF;						
			RS232_TX_BUFF[7]=(calCRC>>8)&0xFF;	
		
			if(RS232_RX_BUFF[0]!=0)//
			{
			 HAL_UART_Transmit_DMA(&huart2,RS232_TX_BUFF,8);
			}
				
	}
	else
	{
			RS232_TX_BUFF[0]=RS232_RX_BUFF[0];
			RS232_TX_BUFF[1]=RS232_RX_BUFF[1]|0x80;
			RS232_TX_BUFF[2]=0x02; 
			if(RS232_RX_BUFF[0]!=0)
			{
			 HAL_UART_Transmit_DMA(&huart2,RS232_TX_BUFF,3);
			}
	}
}


uint16_t Reserve=0;
void Init_Modbus_Addr_List(void)
{
	int i;

	for(i=0;i<MODBUS_REG_NUM;i++)
	{
		Modbus_Output_Reg[i]=(uint16_t*)&Reserve;
	}
	
	Modbus_Output_Reg[10]=(uint16_t*)&store_parameter;
	
	Modbus_Output_Reg[30]=(uint16_t*)&device_temperature;
	Modbus_Output_Reg[31]=(uint16_t*)&vbus_voltage;
	
	Modbus_Output_Reg[40]=(uint16_t*)&Iq_real+1;
	Modbus_Output_Reg[41]=(uint16_t*)&Iq_real;
	Modbus_Output_Reg[42]=(uint16_t*)&real_speed_filter+1;
	Modbus_Output_Reg[43]=(uint16_t*)&real_speed_filter;
	Modbus_Output_Reg[44]=(uint16_t*)&pos_actual+1;
	Modbus_Output_Reg[45]=(uint16_t*)&pos_actual;
	Modbus_Output_Reg[46]=(uint16_t*)&Reserve;
	Modbus_Output_Reg[47]=(uint16_t*)&Reserve;
	Modbus_Output_Reg[48]=(uint16_t*)&Reserve;
	Modbus_Output_Reg[49]=(uint16_t*)&Reserve;
	
	Modbus_Output_Reg[50]=(uint16_t*)&Error_State.all;
	Modbus_Output_Reg[51]=(uint16_t*)&Reserve;
	Modbus_Output_Reg[52]=(uint16_t*)&Reserve;
	Modbus_Output_Reg[53]=(uint16_t*)&Reserve;
	Modbus_Output_Reg[54]=(uint16_t*)&Reserve;
	Modbus_Output_Reg[55]=(uint16_t*)&Reserve;
	
	Modbus_Output_Reg[56]=(uint16_t*)&Driver_IIt_Real+1;
	Modbus_Output_Reg[57]=(uint16_t*)&Driver_IIt_Real;
	Modbus_Output_Reg[58]=(uint16_t*)&Driver_IIt_Real_DC+1;
	Modbus_Output_Reg[59]=(uint16_t*)&Driver_IIt_Real_DC;
	
	Modbus_Output_Reg[60]=(uint16_t*)&operation_mode;
	Modbus_Output_Reg[61]=(uint16_t*)&Reserve;
	Modbus_Output_Reg[62]=(uint16_t*)&control_word.all;
	Modbus_Output_Reg[63]=(uint16_t*)&Reserve;
	Modbus_Output_Reg[64]=(uint16_t*)&target_Iq+1;
	Modbus_Output_Reg[65]=(uint16_t*)&target_Iq;
	Modbus_Output_Reg[66]=(uint16_t*)&target_speed+1;
	Modbus_Output_Reg[67]=(uint16_t*)&target_speed;
	Modbus_Output_Reg[68]=(uint16_t*)&target_position+1;
	Modbus_Output_Reg[69]=(uint16_t*)&target_position;
	Modbus_Output_Reg[70]=(uint16_t*)&Reserve;
	Modbus_Output_Reg[71]=(uint16_t*)&Reserve;
	Modbus_Output_Reg[72]=(uint16_t*)&Reserve;
	Modbus_Output_Reg[73]=(uint16_t*)&Reserve;
	
	Modbus_Output_Reg[74]=(uint16_t*)&Iq_demand+1;
	Modbus_Output_Reg[75]=(uint16_t*)&Iq_demand;
	Modbus_Output_Reg[76]=(uint16_t*)&speed_demand+1;
	Modbus_Output_Reg[77]=(uint16_t*)&speed_demand;
	Modbus_Output_Reg[78]=(uint16_t*)&position_demand+1;
	Modbus_Output_Reg[79]=(uint16_t*)&position_demand;
	
	Modbus_Output_Reg[80]=(uint16_t*)&start_calibrate_hall_phase;
	Modbus_Output_Reg[81]=(uint16_t*)&set_tamagawa_zero;
	
	Modbus_Output_Reg[100]=(uint16_t*)&software_version+1;
	Modbus_Output_Reg[101]=(uint16_t*)&software_version;
	
	Modbus_Output_Reg[120]=(uint16_t*)&RS485_Addr;
	Modbus_Output_Reg[121]=(uint16_t*)&Modbus_Addr_Base;
	Modbus_Output_Reg[122]=(uint16_t*)&RS485_Baudrate+1;
	Modbus_Output_Reg[123]=(uint16_t*)&RS485_Baudrate;
	Modbus_Output_Reg[124]=(uint16_t*)&RS485_Protocol;
	
	Modbus_Output_Reg[130]=(uint16_t*)&operation_mode;
	//Modbus_Output_Reg[132]=(uint16_t*)&control_word;
	Modbus_Output_Reg[131]=(uint16_t*)&Reserve;
	Modbus_Output_Reg[132]=(uint16_t*)&Reserve;
	Modbus_Output_Reg[133]=(uint16_t*)&Reserve;
	Modbus_Output_Reg[134]=(uint16_t*)&target_Iq+1;
	Modbus_Output_Reg[135]=(uint16_t*)&target_Iq;
	Modbus_Output_Reg[136]=(uint16_t*)&target_speed+1;
	Modbus_Output_Reg[137]=(uint16_t*)&target_speed;
	Modbus_Output_Reg[138]=(uint16_t*)&target_position+1;
	Modbus_Output_Reg[139]=(uint16_t*)&target_position;
	
	
	Modbus_Output_Reg[140]=(uint16_t*)&kcp;
	Modbus_Output_Reg[141]=(uint16_t*)&kci;
	Modbus_Output_Reg[142]=(uint16_t*)&Ilim+1;
	Modbus_Output_Reg[143]=(uint16_t*)&Ilim;
	Modbus_Output_Reg[144]=(uint16_t*)&kci_sum_limit+1;
	Modbus_Output_Reg[145]=(uint16_t*)&kci_sum_limit;
	Modbus_Output_Reg[146]=(uint16_t*)&current_in_lpf_a;
	Modbus_Output_Reg[147]=(uint16_t*)&current_out_lpf_a;
	Modbus_Output_Reg[148]=(uint16_t*)&Reserve;
	Modbus_Output_Reg[149]=(uint16_t*)&Reserve;
	
	Modbus_Output_Reg[150]=(uint16_t*)&kvp;
	Modbus_Output_Reg[151]=(uint16_t*)&kvi;
	Modbus_Output_Reg[152]=(uint16_t*)&kvi_sum_limit+1;
	Modbus_Output_Reg[153]=(uint16_t*)&kvi_sum_limit;
	Modbus_Output_Reg[154]=(uint16_t*)&speed_in_lpf_a;
	Modbus_Output_Reg[155]=(uint16_t*)&low_pass_filter_on;
	Modbus_Output_Reg[156]=(uint16_t*)&real_speed_filter_num;
	Modbus_Output_Reg[157]=(uint16_t*)&vel_lim+1;
	Modbus_Output_Reg[158]=(uint16_t*)&vel_lim;
	Modbus_Output_Reg[159]=(uint16_t*)&speed_out_lpf_a;
	
	Modbus_Output_Reg[160]=(uint16_t*)&kpp;
	Modbus_Output_Reg[161]=(uint16_t*)&kpi;
	Modbus_Output_Reg[162]=(uint16_t*)&kpi_sum_limit+1;
	Modbus_Output_Reg[163]=(uint16_t*)&kpi_sum_limit;
	Modbus_Output_Reg[164]=(uint16_t*)&Reserve;
	Modbus_Output_Reg[165]=(uint16_t*)&Reserve;
	Modbus_Output_Reg[166]=(uint16_t*)&position_in_lpf_a;
	Modbus_Output_Reg[167]=(uint16_t*)&position_out_lpf_a;
	Modbus_Output_Reg[168]=(uint16_t*)&Reserve;
	Modbus_Output_Reg[169]=(uint16_t*)&Reserve;
	
	Modbus_Output_Reg[170]=(uint16_t*)&auto_reverse_p_time+1;
	Modbus_Output_Reg[171]=(uint16_t*)&auto_reverse_p_time;
	Modbus_Output_Reg[172]=(uint16_t*)&auto_reverse_n_time+1;
	Modbus_Output_Reg[173]=(uint16_t*)&auto_reverse_n_time;
	Modbus_Output_Reg[174]=(uint16_t*)&auto_p_pos+1;
	Modbus_Output_Reg[175]=(uint16_t*)&auto_p_pos;
	Modbus_Output_Reg[176]=(uint16_t*)&auto_n_pos+1;
	Modbus_Output_Reg[177]=(uint16_t*)&auto_n_pos;
	Modbus_Output_Reg[178]=(uint16_t*)&auto_switch_on;
	Modbus_Output_Reg[179]=(uint16_t*)&Reserve;
	
	
	Modbus_Output_Reg[180]=(uint16_t*)&motor_code;
	Modbus_Output_Reg[181]=(uint16_t*)&poles_num;
	Modbus_Output_Reg[182]=(uint16_t*)&feedback_resolution+1;
	Modbus_Output_Reg[183]=(uint16_t*)&feedback_resolution;
	Modbus_Output_Reg[184]=(uint16_t*)&commutation_mode;
	Modbus_Output_Reg[185]=(uint16_t*)&commutation_time;
	Modbus_Output_Reg[186]=(uint16_t*)&commutation_current+1;
	Modbus_Output_Reg[187]=(uint16_t*)&commutation_current;
	Modbus_Output_Reg[188]=(uint16_t*)&tamagawa_offset;
	Modbus_Output_Reg[189]=(uint16_t*)&tamagawa_dir;
	
	Modbus_Output_Reg[190]=(uint16_t*)&hall_phase_offset;
	Modbus_Output_Reg[191]=(uint16_t*)&hall_phase[1];
	Modbus_Output_Reg[192]=(uint16_t*)&hall_phase[2];
	Modbus_Output_Reg[193]=(uint16_t*)&hall_phase[3];
	Modbus_Output_Reg[194]=(uint16_t*)&hall_phase[4];
	Modbus_Output_Reg[195]=(uint16_t*)&hall_phase[5];
	Modbus_Output_Reg[196]=(uint16_t*)&hall_phase[6];
	Modbus_Output_Reg[197]=(uint16_t*)&ENC_Z_Phase_B;
	Modbus_Output_Reg[198]=(uint16_t*)&phase_dir;
	Modbus_Output_Reg[199]=(uint16_t*)&vel_dir;
	
	Modbus_Output_Reg[200]=(uint16_t*)&motor_rated_current+1;
	Modbus_Output_Reg[201]=(uint16_t*)&motor_rated_current;
	Modbus_Output_Reg[202]=(uint16_t*)&motor_peak_current+1;
	Modbus_Output_Reg[203]=(uint16_t*)&motor_peak_current;
	Modbus_Output_Reg[204]=(uint16_t*)&motor_overload_time+1;
	Modbus_Output_Reg[205]=(uint16_t*)&motor_overload_time;
	Modbus_Output_Reg[206]=(uint16_t*)&Reserve;
	Modbus_Output_Reg[207]=(uint16_t*)&Reserve;
	Modbus_Output_Reg[208]=(uint16_t*)&Reserve;
	Modbus_Output_Reg[209]=(uint16_t*)&Reserve;
	
	Modbus_Output_Reg[210]=(uint16_t*)&gear_factor_a+1;
	Modbus_Output_Reg[211]=(uint16_t*)&gear_factor_a;
	Modbus_Output_Reg[212]=(uint16_t*)&gear_factor_b+1;
	Modbus_Output_Reg[213]=(uint16_t*)&gear_factor_b;
	
	Modbus_Output_Reg[220]=(uint16_t*)&profile_target_position+1;
	Modbus_Output_Reg[221]=(uint16_t*)&profile_target_position;
	Modbus_Output_Reg[222]=(uint16_t*)&profile_speed+1;
	Modbus_Output_Reg[223]=(uint16_t*)&profile_speed;
	Modbus_Output_Reg[224]=(uint16_t*)&profile_acce+1;
	Modbus_Output_Reg[225]=(uint16_t*)&profile_acce;
	Modbus_Output_Reg[226]=(uint16_t*)&profile_dece+1;
	Modbus_Output_Reg[227]=(uint16_t*)&profile_dece;
	Modbus_Output_Reg[228]=(uint16_t*)&searching_speed+1;
	Modbus_Output_Reg[229]=(uint16_t*)&searching_speed;
	Modbus_Output_Reg[230]=(uint16_t*)&motion_out_lpf_a;
	
	
	Modbus_Output_Reg[240]=(uint16_t*)&feedback_type;
	
	Modbus_Output_Reg[250]=(uint16_t*)&over_voltage;
	Modbus_Output_Reg[251]=(uint16_t*)&under_voltage;
	Modbus_Output_Reg[252]=(uint16_t*)&chop_voltage;
	Modbus_Output_Reg[253]=(uint16_t*)&over_temperature;
	Modbus_Output_Reg[254]=(uint16_t*)&Reserve;
	Modbus_Output_Reg[255]=(uint16_t*)&Reserve;
	Modbus_Output_Reg[256]=(uint16_t*)&Driver_IIt_Filter;
	Modbus_Output_Reg[257]=(uint16_t*)&Driver_IIt_Current+1;
	Modbus_Output_Reg[258]=(uint16_t*)&Driver_IIt_Current;
	Modbus_Output_Reg[259]=(uint16_t*)&Driver_IIt_Filter_DC;
	Modbus_Output_Reg[260]=(uint16_t*)&Driver_IIt_Current_DC+1;
	Modbus_Output_Reg[261]=(uint16_t*)&Driver_IIt_Current_DC;
	
	
}

	


