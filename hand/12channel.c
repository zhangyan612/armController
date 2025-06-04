#include "USER.h"


uint8_t  Rx_dat[30],num_dat[30],Moto=0,flag_tim=0;
uint16_t Speed=0;
uint32_t time=0,TimerCnt=0,tim_user[12];
uint32_t Moto_dat[48]={0,0,0,0, //0  1  2  3 
											 0,0,0,0, //4  5  6  7 
											 0,0,0,0, //8  9  10 11                                                                                                 
                       0,0,0,0, //12 13 14 15
											 0,0,0,0, //16 17 18 19
											 0,0,0,0, //20 21 22 23
											 0,0,0,0, //24 25 26 27                                                                                                
                       0,0,0,0, //28 29 30 31
											 0,0,0,0, //32 33 34 35
											 0,0,0,0, //36 37 38 39
											 0,0,0,0, //40 41 42 43                                                                                                  
                       0,0,0,0};//44 45 46 47

void INIT_ALL(void)  //ÏµÍ³³õÊ¼»¯
{
	HAL_TIM_Base_Start_IT(&htim6); //Ê¹ÄÜ¶¨Ê±Æ÷ÖÐ¶Ï
	HAL_TIM_IRQHandler(&htim1);
	HAL_TIM_IRQHandler(&htim2);
	HAL_TIM_IRQHandler(&htim3);
	HAL_TIM_IRQHandler(&htim4);
	HAL_TIM_IRQHandler(&htim5);
	HAL_TIM_IRQHandler(&htim8);
	HAL_TIM_Base_Start_IT(&htim1); 
	HAL_TIM_Base_Start_IT(&htim2); 
	HAL_TIM_Base_Start_IT(&htim3);  
	HAL_TIM_Base_Start_IT(&htim4);
	HAL_TIM_Base_Start_IT(&htim5);
	HAL_TIM_Base_Start_IT(&htim8);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1); 
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2); 
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3); 
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4); 
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1); 
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2); 
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_3); 
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_4); 
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1); 
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2); 
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_3); 
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_4); 
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_1); 
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_2); 
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_3); 
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_4); 
	HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_1); 
	HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_2); 
	HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_3); 
	HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_1); 
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_2); 
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_3); 
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_4);
	PWM_get();
//	HAL_UART_Receive_DMA(&huart3,Rx_dat,11);	
	HAL_UARTEx_ReceiveToIdle_DMA(&huart3,Rx_dat,sizeof(Rx_dat));	
	__HAL_DMA_DISABLE_IT(&hdma_usart3_rx,DMA_IT_HT);
}

void num_data_get(void)
{
	for(uint8_t dat=0;dat<=15;dat++)
	{
		num_dat[dat]=Rx_dat[dat]-'0';
	}
}

void value_get(void)
{
	Moto = (num_dat[1]*10)+num_dat[2];
	Speed = (num_dat[4]*1000)+(num_dat[5]*100)+(num_dat[6]*10)+num_dat[7];
	time = (num_dat[9]*10000)+(num_dat[10]*1000)+(num_dat[11]*100)+(num_dat[12]*10)+num_dat[13];
}

void Moto_dat_get(void)
{
	if (Moto<=12 && Moto>0) {Moto_dat[(Moto-1)*4]=Moto;flag_tim=1;}
	else {printf("µç»úÎ»ºÅÉèÖÃ´íÎó£¡£¡£¡\r\n");flag_tim=0;}
	
	
	if (Speed<2000){
		Moto_dat[Moto*4-3] = 0;
		Moto_dat[Moto*4-2] = 2000-Speed;
	}
	else if (Speed>2000){
		Moto_dat[Moto*4-3] = Speed-2000;
		Moto_dat[Moto*4-2] = 0;
	}
	else if (Speed==2000) Moto_dat[Moto*4-3]=Moto_dat[Moto*4-2]=0;
	else {printf("µç»ú×ªËÙÉèÖÃ´íÎó£¡£¡£¡\r\n");	flag_tim=0;}
	
	
	if (flag_tim==1){
		Moto_dat[Moto*4-1] = time;
		tim_user[Moto-1] = TimerCnt;
		HAL_TIM_Base_Start(&htim4);  //Æô¶¯¶¨Ê±Æ÷
	}
}


void loop(void)
{
	for (uint8_t my_moto_tim=0;my_moto_tim<12;my_moto_tim++)
	{
		if(TimerCnt-tim_user[my_moto_tim]>=Moto_dat[my_moto_tim*4+3])
		{
//			HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);
			Moto_dat[my_moto_tim*4+1]=Moto_dat[my_moto_tim*4+2]=0;
		}
	}
////	HAL_Delay(10);
	PWM_get();
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart,uint16_t Size)
{
	if(huart->Instance == USART3)
	{
		if (Rx_dat[0] == 0x23 && Rx_dat[3] == 0x23 && Rx_dat[8] == 0x23 && Rx_dat[14] == 0x23){
			if (Size == 15){
				num_data_get();
				value_get();
				Moto_dat_get();
				printf("Moto=%d\r\n",Moto);
				printf("Speed=%d\r\n",Speed);
				printf("time=%d\r\n",time);
				printf("\r\n\r\n");
				HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);
				PWM_get();
			}
		}
		else printf("Êý¾ÝÊäÈë¸ñÊ½ÓÐÎó£¬ÇëÖØÐÂÊä³ö£¡£¡£¡\r\n");
//		HAL_UART_Transmit_DMA(&huart3,Rx_dat,Size);
	}
	HAL_UARTEx_ReceiveToIdle_DMA(&huart3,Rx_dat,sizeof(Rx_dat));
	__HAL_DMA_DISABLE_IT(&hdma_usart3_rx,DMA_IT_HT);
}




void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  	if(htim==&htim6)
		{
			TimerCnt++;
//			HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);
		}
	
}

void PWM_get(void)
{
		__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,Moto_dat[5]);
		__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2,Moto_dat[6]);
		__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_3,Moto_dat[1]);
		__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_4,Moto_dat[2]);
		__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,Moto_dat[13]);
		__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,Moto_dat[14]);
		__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3,Moto_dat[9]);
		__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_4,Moto_dat[10]);
		__HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_1,Moto_dat[21]);
		__HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_2,Moto_dat[22]);
		__HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_3,Moto_dat[17]);
		__HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_4,Moto_dat[18]);
		__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,Moto_dat[29]);
		__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2,Moto_dat[30]);
		__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,Moto_dat[25]);
		__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_4,Moto_dat[26]);
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,Moto_dat[37]);
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,Moto_dat[38]);
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,Moto_dat[33]);
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,Moto_dat[34]);
		__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,Moto_dat[45]);
		__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,Moto_dat[46]);
		__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3,Moto_dat[41]);
		__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_4,Moto_dat[42]);
}

