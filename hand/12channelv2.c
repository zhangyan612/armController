#include "USER.h"
#include <string.h>

uint8_t Rx_dat[100] = {0};  // ??????????????
uint8_t num_dat[30] = {0};
uint8_t Moto = 0;
uint8_t flag_tim = 0;
uint16_t Speed = 0;
uint32_t time = 0;
uint32_t TimerCnt = 0;
uint32_t tim_user[12] = {0};

// ??????
uint32_t Moto_dat[48] = {0};  // ????0

void INIT_ALL(void)
{
    // ???????
    HAL_TIM_Base_Start_IT(&htim6);
    HAL_TIM_Base_Start_IT(&htim1);
    HAL_TIM_Base_Start_IT(&htim2);
    HAL_TIM_Base_Start_IT(&htim3);
    HAL_TIM_Base_Start_IT(&htim4);
    HAL_TIM_Base_Start_IT(&htim5);
    HAL_TIM_Base_Start_IT(&htim8);
    
    // ??PWM??
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
    
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
    
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
    
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
    
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_4);
    
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);
    
    PWM_get();  // ???PWM??
    
    // ??DMA??
    HAL_UARTEx_ReceiveToIdle_DMA(&huart3, Rx_dat, sizeof(Rx_dat));
    __HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_HT);
}

void num_data_get(uint8_t* data)
{
    for(uint8_t dat = 0; dat < 15; dat++)
    {
        num_dat[dat] = data[dat] - '0';
    }
}

void value_get(void)
{
    Moto = (num_dat[1] * 10) + num_dat[2];
    Speed = (num_dat[4] * 1000) + (num_dat[5] * 100) + (num_dat[6] * 10) + num_dat[7];
    time = (num_dat[9] * 10000) + (num_dat[10] * 1000) + (num_dat[11] * 100) + (num_dat[12] * 10) + num_dat[13];
}

void Moto_dat_get(void)
{
    if (Moto <= 12 && Moto > 0) {
        Moto_dat[(Moto-1)*4] = Moto;
        flag_tim = 1;
    } else {
        printf("µç»úÎ»ºÅÉèÖÃ´íÎó£¡£¡£¡\r\n");
        flag_tim = 0;
        return;
    }
    
    if (Speed < 2000) {
        Moto_dat[Moto*4-3] = 0;
        Moto_dat[Moto*4-2] = 2000 - Speed;
    } else if (Speed > 2000) {
        Moto_dat[Moto*4-3] = Speed - 2000;
        Moto_dat[Moto*4-2] = 0;
    } else if (Speed == 2000) {
        Moto_dat[Moto*4-3] = 0;
        Moto_dat[Moto*4-2] = 0;
    } else {
        printf("µç»ú×ªËÙÉèÖÃ´íÎó£¡£¡£¡\r\n");
        flag_tim = 0;
        return;
    }
    
    if (flag_tim == 1) {
        Moto_dat[Moto*4-1] = time;
        tim_user[Moto-1] = TimerCnt;
    }
}

void process_motor_command(uint8_t* cmd_data)
{
    // ??????
    if (cmd_data[0] == '#' && cmd_data[3] == '#' && 
        cmd_data[8] == '#' && cmd_data[14] == '#') 
    {
        num_data_get(cmd_data);
        value_get();
        Moto_dat_get();
        
        printf("Moto=%d\r\n", Moto);
        printf("Speed=%d\r\n", Speed);
        printf("time=%d\r\n", time);
        printf("\r\n");
        
        HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    } else {
        printf("??????: %.*s\r\n", 15, cmd_data);
    }
}

void loop(void)
{
    // ???????????
    for (uint8_t my_moto_tim = 0; my_moto_tim < 12; my_moto_tim++)
    {
        if(TimerCnt - tim_user[my_moto_tim] >= Moto_dat[my_moto_tim * 4 + 3])
        {
            // ???????
            Moto_dat[my_moto_tim * 4 + 1] = 0;
            Moto_dat[my_moto_tim * 4 + 2] = 0;
        }
    }
    
    // ??PWM??
    PWM_get();
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if(huart->Instance == USART3)
    {
        uint8_t *start = Rx_dat;
        uint8_t *end;
        
        // ???????????
        while((end = memchr(start, ';', Size - (start - Rx_dat))) != NULL)
        {
            // ?????? (15????)
            if((end - start) == 15)
            {
                process_motor_command(start);
            }
            
            start = end + 1; // ????????
        }
        
        // ????????(??????)
        if((Size - (start - Rx_dat)) >= 15)
        {
            process_motor_command(start);
        }
        
        // ????DMA??
        HAL_UARTEx_ReceiveToIdle_DMA(&huart3, Rx_dat, sizeof(Rx_dat));
        __HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_HT);
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim == &htim6)
    {
        TimerCnt++; // ??????
    }
}

void PWM_get(void)
{
    // ????PWM??????
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, Moto_dat[5]);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, Moto_dat[6]);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, Moto_dat[1]);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, Moto_dat[2]);
    
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, Moto_dat[13]);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, Moto_dat[14]);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, Moto_dat[9]);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, Moto_dat[10]);
    
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, Moto_dat[21]);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, Moto_dat[22]);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, Moto_dat[17]);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, Moto_dat[18]);
    
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, Moto_dat[29]);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, Moto_dat[30]);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, Moto_dat[25]);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, Moto_dat[26]);
    
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, Moto_dat[37]);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, Moto_dat[38]);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, Moto_dat[33]);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, Moto_dat[34]);
    
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, Moto_dat[45]);
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, Moto_dat[46]);
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, Moto_dat[41]);
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, Moto_dat[42]);
}

