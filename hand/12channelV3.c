#include "USER.h"
#include <string.h>
#include <stdlib.h>

uint8_t Rx_dat[100] = {0};
uint8_t num_dat[30] = {0};
uint8_t Moto = 0;
uint8_t flag_tim = 0;
uint16_t Speed = 0;
uint32_t time = 0;
uint32_t TimerCnt = 0;
uint32_t tim_user[12] = {0};
uint32_t Moto_dat[48] = {0};  // ??????

void INIT_ALL(void)
{
    // ?????
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
    
    PWM_get();  // ??PWM??
    
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
        Moto_dat[(Moto-1)*4] = Moto;  // ????ID
        flag_tim = 1;
    } else {
        printf("?????!!!\r\n");
        flag_tim = 0;
        return;
    }
    
    if (Speed < 2000) {
        Moto_dat[Moto*4-3] = 0;          // ??PWM
        Moto_dat[Moto*4-2] = 2000 - Speed; // ??PWM
    } else if (Speed > 2000) {
        Moto_dat[Moto*4-3] = Speed - 2000; // ??PWM
        Moto_dat[Moto*4-2] = 0;          // ??PWM
    } else if (Speed == 2000) {
        Moto_dat[Moto*4-3] = 0;
        Moto_dat[Moto*4-2] = 0;
    } else {
        printf("????!!!\r\n");
        flag_tim = 0;
        return;
    }
    
    if (flag_tim == 1) {
        Moto_dat[Moto*4-1] = time;       // ????
        tim_user[Moto-1] = TimerCnt;     // ??????
    }
}

// ?????? - FIXED VERSION
void stop_motor(uint8_t motor_id)
{
    if (motor_id >= 1 && motor_id <= 12) {
        uint8_t idx = (motor_id-1)*4;
        
        // ??PWM?,?????ID??,??????????
        Moto_dat[idx+1] = 0;         // ??PWM?0
        Moto_dat[idx+2] = 0;         // ??PWM?0
        Moto_dat[idx+3] = 0;         // ?????0
        
        // ????ID,??????
        Moto_dat[idx] = 0;           // ??ID?0
        
        tim_user[motor_id-1] = 0;     // ??????
        
        printf("?? %d ???\r\n", motor_id);
    } else {
        printf("????ID: %d\r\n", motor_id);
    }
}

// ?????? - FIXED to handle both "7" and "07" formats
void process_stop_command(uint8_t* cmd_data)
{
    // ??: #STOP#id1,id2,...,idn#
    uint8_t *id_start = cmd_data + 6;
    uint8_t id_str[20] = {0};
    
    // ?????#
    uint8_t *end_ptr = memchr(id_start, '#', sizeof(Rx_dat) - (id_start - Rx_dat));
    if (!end_ptr) {
        printf("??????: ?????#\r\n");
        return;
    }
    
    // ??ID???
    uint8_t id_len = end_ptr - id_start;
    memcpy(id_str, id_start, id_len);
    id_str[id_len] = '\0';
    
    printf("????: %s\r\n", id_str);
    
    // ???????ID
    char *token = strtok((char*)id_str, ",");
    while (token != NULL) {
        // ??????
        while (*token == ' ') token++;
        
        // ????ID - ?? "7" ? "07" ??
        uint8_t motor_id = 0;
        if (strlen(token) == 1) {
            // ?????: "7"
            motor_id = token[0] - '0';
        } else if (strlen(token) == 2) {
            // ?????: "07" ? "12"
            motor_id = (token[0] - '0') * 10 + (token[1] - '0');
        } else {
            // ??atoi????
            motor_id = atoi(token);
        }
        
        printf("????ID: %s -> %d\r\n", token, motor_id);
        
        if (motor_id >= 1 && motor_id <= 12) {
            stop_motor(motor_id);
        } else {
            printf("????ID: %s(???%d)\r\n", token, motor_id);
        }
        token = strtok(NULL, ",");
    }
    
    // ????PWM?? - THIS IS THE KEY FIX
    PWM_get();
}

void process_motor_command(uint8_t* cmd_data)
{
    // ?????????
    if (memcmp(cmd_data, "#STOP#", 6) == 0) {
        process_stop_command(cmd_data);
        return;
    }
    
    // ?????????
    if (cmd_data[0] == '#' && cmd_data[3] == '#' && 
        cmd_data[8] == '#' && cmd_data[14] == '#') 
    {
        num_data_get(cmd_data);
        value_get();
        Moto_dat_get();
        
        printf("??=%d\r\n", Moto);
        printf("??=%d\r\n", Speed);
        printf("??=%d\r\n", time);
        printf("\r\n");
        
        HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    } else {
        printf("????: %.*s\r\n", 15, cmd_data);
    }
}

void loop(void)
{
    // ???????????
    for (uint8_t my_moto_tim = 0; my_moto_tim < 12; my_moto_tim++)
    {
        // ????????
        if (Moto_dat[my_moto_tim * 4] == 0) continue;
        
        // ??????
        if(TimerCnt - tim_user[my_moto_tim] >= Moto_dat[my_moto_tim * 4 + 3])
        {
            // ???,????
            Moto_dat[my_moto_tim * 4 + 1] = 0;  // ??PWM
            Moto_dat[my_moto_tim * 4 + 2] = 0;  // ??PWM
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
        
        // ??????(????)
        while((end = memchr(start, ';', Size - (start - Rx_dat))) != NULL)
        {
            uint16_t cmd_length = end - start;
            
            // ??????
            if(cmd_length >= 7) {
                process_motor_command(start);
            }
            
            start = end + 1;
        }
        
        // ????????
        if(Size - (start - Rx_dat) >= 7) {
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
        TimerCnt++; // ???????
    }
}

void PWM_get(void)
{
    // ????????PWM???
    // ??1: [1]=??PWM, [2]=??PWM
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, Moto_dat[1]);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, Moto_dat[2]);
    
    // ??2: [5]=??PWM, [6]=??PWM
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, Moto_dat[5]);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, Moto_dat[6]);
    
    // ??3: [9]=??PWM, [10]=??PWM
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, Moto_dat[9]);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, Moto_dat[10]);
    
    // ??4: [13]=??PWM, [14]=??PWM
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, Moto_dat[13]);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, Moto_dat[14]);
    
    // ??5: [17]=??PWM, [18]=??PWM
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, Moto_dat[17]);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, Moto_dat[18]);
    
    // ??6: [21]=??PWM, [22]=??PWM
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, Moto_dat[21]);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, Moto_dat[22]);
    
    // ??7: [25]=??PWM, [26]=??PWM
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, Moto_dat[25]);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, Moto_dat[26]);
    
    // ??8: [29]=??PWM, [30]=??PWM
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, Moto_dat[29]);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, Moto_dat[30]);
    
    // ??9: [33]=??PWM, [34]=??PWM
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, Moto_dat[33]);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, Moto_dat[34]);
    
    // ??10: [37]=??PWM, [38]=??PWM
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, Moto_dat[37]);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, Moto_dat[38]);
    
    // ??11: [41]=??PWM, [42]=??PWM
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, Moto_dat[41]);
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, Moto_dat[42]);
    
    // ??12: [45]=??PWM, [46]=??PWM
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, Moto_dat[45]);
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, Moto_dat[46]);
}

