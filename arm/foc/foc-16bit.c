#include "FOC_H.h"
#include "gpio.h"
#include <stdio.h>
#include <stdlib.h> 
#include "math.h"
#include "main.h"
#include "tim.h"
#include "foc_utils.h"
#include "pid.h"
#include "Kalman.h"
#include "pid.h"
#include "main.h"
#include "usart.h"

// ??????
#define DEBUG_ENABLED 0

// ?????
#if DEBUG_ENABLED
#define DEBUG_PRINTF(...) printf(__VA_ARGS__)
#else
#define DEBUG_PRINTF(...)
#endif

extern float setSpeed;          // ????
extern float setCurrent;         // ????
extern float setPosition ;       // ????

extern ADC_HandleTypeDef hadc1;
extern UART_HandleTypeDef huart1; // ?????UART
extern UART_HandleTypeDef huart2; // ??????UART

float shaft_angle,sensor_offset,zero_electric_angle,electrical_angle;
int pole_pairs=1,pattern=0;
signed char sensor_direction=-1; // ?? signed char ???????
long angle_data, angle_data_prev;// ???????
float full_rotation_offset;      // ????????
extern uint8_t isRunning ;             // ????

DQVoltage_s voltage;                  //!< DQ?????
Kalman kalman_current_Iq_Filter;
DQCurrent_s current;                  //!< DQ?????
// ??????
#define ENCODER_RESOLUTION 128

volatile uint8_t motor_enabled = 0;  // ??????

// ????????(16?)
#define ENCODER_MAX_COUNT 65535
// ????
int32_t rotation_count = 0; // ????

// ???????
int16_t last_count = 0;      // ????????
// float shaft_angle = 0.0f;    // ????(??) // ??????
float Target_speed,shaft_velocity;
int MGT_angle = 0;  // ????
char MGT_angle_rx_flag=0;
char feedback_send_flag;

// --- START: ADDED GLOBAL VARIABLE DEFINITIONS ---
float target_angle = 0.0f; // ???? (??)
int reduction_ratio = 1;   // ???
// --- END: ADDED GLOBAL VARIABLE DEFINITIONS ---


typedef struct{
	float adc_10;
	float adc_11;
} adc_1011;

adc_1011 ADC_1011;

adc_1011 _readADCVoltageInline()
{	
	adc_1011 adc_;
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1,50);
	adc_.adc_10=(float)HAL_ADC_GetValue(&hadc1)*33/40960;
	
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1,50);
	adc_.adc_11=(float)HAL_ADC_GetValue(&hadc1)*33/40960;
	HAL_ADC_Stop (&hadc1);
	
	// DEBUG_PRINTF("ADC: a=%.2fV, b=%.2fV\n", adc_.adc_10, adc_.adc_11);
	return adc_;
}

PhaseCurrent_s getPhaseCurrents(void)
{
	PhaseCurrent_s current;
	ADC_1011=_readADCVoltageInline();
	current.a = (ADC_1011.adc_10 - 1.65)*10; // ?A
	current.b = (ADC_1011.adc_11 - 1.67)*10; // ?B
	current.c = 0; // ?C(????)
	
	// DEBUG_PRINTF("Phase currents: a=%.2fA, b=%.2fA\n", current.a, current.b);
	return current;
}

DQCurrent_s getFOCCurrents(float angle_el)
{
	PhaseCurrent_s current;
	float i_alpha, i_beta;
	float ct,st;
	DQCurrent_s ret;
	
	// 1. ?????
	current = getPhaseCurrents();
	
	// 2. Clarke??
	if(!current.c)
	{
		// ??????,i_a+i_b+i_c=0
		i_alpha = -current.b;  
		i_beta = _1_SQRT3 * current.b + _2_SQRT3 * current.a;
	}
	else
	{
		// ???????
		float mid = (1.f/3) * (current.a + current.b + current.c);
		float a = current.a - mid;
		float b = current.b - mid;
		i_alpha = a;
		i_beta = _1_SQRT3 * a + _2_SQRT3 * b;
	}
	
	// 3. Park??
	ct = _cos(angle_el);
	st = _sin(angle_el);
	ret.d = i_alpha * ct + i_beta * st;
	ret.q = i_beta * ct - i_alpha * st;
	
	// DEBUG_PRINTF("DQ currents: d=%.2f, q=%.2f, angle_el=%.2f\n", ret.d, ret.q, angle_el);
	
	// ??
	ret.q=Kalman_Filter(&kalman_current_Iq_Filter,ret.q);
	ret.q= Slid_Filter(&Filter_ret_q,ret.q,40);
	
	return ret;
}

float Position_KP		=0.15,Position_KI=0.001,Position_KD=0.32;          /* ???PID?? */
float Incremental_KP=1.1,Incremental_KI=0.09,Incremental_KD=0.25;   /* ???PID?? */

float Position_PID(float reality,float target)
{ 	
    static float Bias,Pwm,Last_Bias,Integral_bias=0;
    
    Bias=target-reality;                            // ????
    Integral_bias+=Bias;	                        // ????
    
    if(Integral_bias> 5) Integral_bias = 5;   // ????
    if(Integral_bias<-5) Integral_bias =-5;
    
    Pwm = (Position_KP*Bias)                        // ???
         +(Position_KI*Integral_bias)               // ???
         +(Position_KD*(Bias-Last_Bias));           // ???
    
    Last_Bias=Bias;                                 // ??????
    
    // DEBUG_PRINTF("Position PID: target=%.2f, reality=%.2f, output=%.2f\n", target, reality, Pwm);
    return Pwm;                                     // ??????
}

float Bias_buf;

float Incremental_PID(float reality,float target)
{ 	
	 static float Bias,Pwm,Last_bias=0,Prev_bias=0;
    
	 Bias=target-reality;                                   // ????
     Bias_buf=Bias;
	 Pwm += (Incremental_KP*(Bias-Last_bias))               // ????
           +(Incremental_KI*Bias)                           // ????
           +(Incremental_KD*(Bias-2*Last_bias+Prev_bias));  // ????	
     Prev_bias=Last_bias;                                   // ??????
	 Last_bias=Bias;	                                    // ???????
		
		Pwm=_constrain(Pwm,-3,3);
    
    // DEBUG_PRINTF("Incremental PID: target=%.2f, reality=%.2f, output=%.2f\n", target, reality, Pwm);
	 return Pwm;                                            // ???????
}

float electricalAngle(void)
{
    float angle = _normalizeAngle((shaft_angle + sensor_offset) * pole_pairs - zero_electric_angle);
    // DEBUG_PRINTF("Electrical angle: shaft=%.2f, offset=%.2f, zero=%.2f, result=%.2f\n", 
    //              shaft_angle, sensor_offset, zero_electric_angle, angle);
    return angle;
}

void setPhaseVoltage(float Uq, float Ud, float angle_el)
{
    float Uout;               // ????????
    uint32_t sector;          // ?? (1-6)
    float T0, T1, T2;         // ????????
    float Ta, Tb, Tc;         // ABC?????
    
    // DEBUG_PRINTF("Setting phase voltage: Uq=%.2f, Ud=%.2f, angle_el=%.2f\n", Uq, Ud, angle_el);
    
    // Q?????
    if(Uq > voltage_limit) Uq = voltage_limit;
    if(Uq < -voltage_limit) Uq = -voltage_limit;
    // D?????
    if(Ud > voltage_limit) Ud = voltage_limit;
    if(Ud < -voltage_limit) Ud = -voltage_limit;
    
    // ?Park??
    if(Ud) 
    {
        Uout = _sqrt(Ud * Ud + Uq * Uq) / voltage_power_supply;
        angle_el = _normalizeAngle(angle_el + atan2(Uq, Ud));
    }
    else
    {
        Uout = Uq / voltage_power_supply;
        angle_el = _normalizeAngle(angle_el + _PI_2);
    }
    
    // ???????
    if(Uout > 0.577) Uout = 0.577;
    if(Uout < -0.577) Uout = -0.577;
    
    // ???? (1-6)
    sector = (angle_el / _PI_3) + 1;
    
    // ??T1?T2
    T1 = _SQRT3 * _sin(sector * _PI_3 - angle_el) * Uout;
    T2 = _SQRT3 * _sin(angle_el - (sector - 1.0) * _PI_3) * Uout;
    T0 = 1 - T1 - T2; // ???????
    
    // ???????????
    switch(sector)
    {
        case 1: Ta = T1 + T2 + T0 / 2; Tb = T2 + T0 / 2;      Tc = T0 / 2;            break;
        case 2: Ta = T1 + T0 / 2;      Tb = T1 + T2 + T0 / 2; Tc = T0 / 2;            break;
        case 3: Ta = T0 / 2;           Tb = T1 + T2 + T0 / 2; Tc = T2 + T0 / 2;       break;
        case 4: Ta = T0 / 2;           Tb = T1 + T0 / 2;      Tc = T1 + T2 + T0 / 2;  break;
        case 5: Ta = T2 + T0 / 2;      Tb = T0 / 2;           Tc = T1 + T2 + T0 / 2;  break;
        case 6: Ta = T1 + T2 + T0 / 2; Tb = T0 / 2;           Tc = T1 + T0 / 2;       break;
        default: Ta = 0; Tb = 0; Tc = 0;
    }
    
    // DEBUG_PRINTF("PWM duty: Ta=%.2f, Tb=%.2f, Tc=%.2f\n", Ta, Tb, Tc);
    
    // ??PWM???
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, Ta * PWM_Period); // A?
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, Tb * PWM_Period); // B?
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, Tc * PWM_Period); // C?
}

float getAngle(void)
{
    float d_angle; // ?????

    angle_data =  MGT_angle ; // ????????????? (0-65535)

    // DEBUG_PRINTF("Raw encoder: %d, prev: %ld, rotation: %ld\n", 
    //              MGT_angle, angle_data_prev, rotation_count);
    
    // ??????
    d_angle = (float)(angle_data - angle_data_prev);

    // ???? (????0.8*???)
    if(fabs(d_angle) > (0.8 * 65535.0f)) 
    {
        if (d_angle > 0) { // ?65535???0??,??
            full_rotation_offset -= _2PI;
            rotation_count--;
            // DEBUG_PRINTF("Full rotation detected: CCW, new rotation_count: %ld\n", rotation_count);
        } else { // ?0???65535??,??
            full_rotation_offset += _2PI;
            rotation_count++;
            // DEBUG_PRINTF("Full rotation detected: CW, new rotation_count: %ld\n", rotation_count);
        }
    }

    // ???????????????
    angle_data_prev = angle_data;

    // ????????
    float current_angle = (angle_data * 1.0f / 65535.0f) * _2PI;
    // ??? = ???? + ??????
    float total_angle = full_rotation_offset + current_angle;
    
    // DEBUG_PRINTF("Current angle calc: raw=%.2f rad, offset=%.2f rad, total=%.2f rad\n", 
    //              current_angle, full_rotation_offset, total_angle);
    
    return total_angle;
}

 float current_target_angle = 0.0f;
 float initial_angle = 0.0f;
 char motion_complete = 0;
char test_flag;
 float total_target_angle = 8.0f * (float)_PI; // 8PI rad
 float desired_speed = 2.0f * (float)_PI;      // 2PI rad/s
#define CONTROL_PERIOD 0.1f

/* --- START: MODIFIED/NEW CODE FOR MODBUS ENCODER --- */

// RS485 ????
#define RS485_TX_MODE() HAL_GPIO_WritePin(RS485_EN_GPIO_Port, RS485_EN_Pin, GPIO_PIN_SET)   // ???????
#define RS485_RX_MODE() HAL_GPIO_WritePin(RS485_EN_GPIO_Port, RS485_EN_Pin, GPIO_PIN_RESET) // ???????

// ?????(USART1)??????
#define ENCODER_RESPONSE_SIZE 7
volatile uint8_t encoder_rx_buffer[ENCODER_RESPONSE_SIZE];
volatile uint8_t encoder_rx_idx = 0;
volatile int uart1_rx_timeout_counter = 0; // ???????

// --- START: ADDED MACRO DEFINITIONS ---
#define CMD_BUFFER_SIZE 64
#define CMD_BUFFER_SIZE_B 8   // B???????
// --- END: ADDED MACRO DEFINITIONS ---

// ??????(USART2)??????
volatile char cmd_buffer[CMD_BUFFER_SIZE];
volatile uint8_t cmd_idx = 0;
volatile uint8_t command_ready = 0; // ????????
int uart2_doutntime_rx;

uint8_t rx_byte = 0, rx_byte_uart2 = 0; // ???????

/**
 * @brief  ??Modbus RTU?CRC16???
 * @param  data: ??????????
 * @param  length: ????
 * @retval uint16_t: 16?CRC???
 */
uint16_t crc16(uint8_t *data, uint8_t length) {
    uint16_t crc = 0xFFFF;
    for (uint8_t i = 0; i < length; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

/**
 * @brief  ??Modbus??????????
 */
void Send_Request(void)
{
    uint8_t request_frame[8];
    uint16_t crc;

    // ??Modbus???: [????, ???, ??????, ??????, ??????, ??????]
    request_frame[0] = 0x01; // ????
    request_frame[1] = 0x03; // ??? (0x03: ??????)
    request_frame[2] = 0x00; // ???????? (?? 13)
    request_frame[3] = 0x0D; // ????????
    request_frame[4] = 0x00; // ???????? (?? 1 ?)
    request_frame[5] = 0x01; // ????????

    // ???6????CRC16
    crc = crc16(request_frame, 6);

    // ??CRC??? (????)
    request_frame[6] = (uint8_t)(crc & 0xFF);      // CRC ???
    request_frame[7] = (uint8_t)((crc >> 8) & 0xFF); // CRC ???

    encoder_rx_idx = 0; // ?????????

    // ???????
    RS485_TX_MODE();
    // ?????8?????
    HAL_UART_Transmit(&huart1, request_frame, 8, 100);
    // ???????
    RS485_RX_MODE();

    DEBUG_PRINTF("Encoder Modbus request sent\n");
}

/* --- END: MODIFIED/NEW CODE FOR MODBUS ENCODER --- */

/* ??????? */
typedef enum {
    CMD_STATE_IDLE,            // ????
    CMD_STATE_RECEIVE_EQUALS,  // ??'='
    CMD_STATE_RECEIVE_DATA_A,  // ??A????
    CMD_STATE_RECEIVE_DATA_B,  // ??B????
    CMD_STATE_RECEIVE_DATA_E,  // ??E????
    CMD_STATE_RECEIVE_DATA_Q,  // ??Q????
} cmd_state_t;
volatile cmd_state_t cmd_state = CMD_STATE_IDLE;


void motorEnable(void)
{
    motor_enabled = 1;
    DEBUG_PRINTF("Motor enabled\n");
}

void motorDisable(void)
{
    motor_enabled = 0;
    setPhaseVoltage(0, 0, 0);
    DEBUG_PRINTF("Motor disabled\n");
}

extern float Bias_buf;
void run()
{    
    if (pattern == 0)
    {
        DEBUG_PRINTF("Initializing FOC controller\n");
        
        voltage.q = 0;
        voltage.d = 0;

        PID_current_q.P =  0.08;
        PID_current_q.I =  0.05;
        PID_current_q.D =  0;
        PID_current_q.limit = 2;
        
        pole_pairs=11;  
        
        zero_electric_angle =2.3;
        pattern=4;
        
        // ????????
        angle_data_prev = 0; // ????????????????
        full_rotation_offset = 0.0f;
        rotation_count = 0;
        
        RS485_RX_MODE(); // ?????????
        HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
        HAL_UART_Receive_IT(&huart2, &rx_byte_uart2, 1);
        
        DEBUG_PRINTF("FOC controller initialized\n");
    }

    shaft_angle = getAngle();
    electrical_angle = electricalAngle();

    // DEBUG_PRINTF("Shaft angle: %.2f rad, Electrical angle: %.2f rad\n", shaft_angle, electrical_angle);

    if (!motor_enabled) {
        // DEBUG_PRINTF("Motor disabled, setting zero voltage\n");
        voltage.q = 0;
        voltage.d = 0;
        setPhaseVoltage(0, 0, 0);
        return;
    }

    switch(pattern)
    {
        case 2:
            // DEBUG_PRINTF("Starting sensor alignment\n");
            // ... ??????? ...
            break;
        case 3:
            voltage.q =0.5;
            voltage.d =0;
            // DEBUG_PRINTF("Voltage control mode: Uq=%.2f, Ud=%.2f\n", voltage.q, voltage.d);
            break;
        case 4:
            P_angle.P = 5;
            P_angle.I = 0;
            P_angle.D = 0;      
            voltage.q = PIDoperator(&P_angle, (target_angle - shaft_angle));
            // DEBUG_PRINTF("Position control: target=%.2f, current=%.2f, output=%.2f\n", 
            //              target_angle, shaft_angle, voltage.q);
            break;
    }
    
    // ??????
    setPhaseVoltage(voltage.q, voltage.d, electrical_angle);

    if(feedback_send_flag==1)
    {
        feedback_send_flag=0;
        // DEBUG_PRINTF("Feedback sent\n");
    }
}

void SysTick_1ms()
{
    static float speedLpfBuf =0; 
    static int count=0;
    if(count++>=100)
    {
        count=0;
        float angle = shaft_angle;
        shaft_velocity = (angle-speedLpfBuf)/100*1000;
        speedLpfBuf = angle;
        shaft_velocity = Slid_Filter(&Filter_shaft_velocity,shaft_velocity,5);
        // DEBUG_PRINTF("Shaft velocity: %.2f rad/s\n", shaft_velocity);
    }

    // UART1 ??????
    if (uart1_rx_timeout_counter > 0) {
        uart1_rx_timeout_counter--;
        if (uart1_rx_timeout_counter == 0) {
            encoder_rx_idx = 0; // ??,??????
            DEBUG_PRINTF("Encoder RX timeout, index reset.\n");
        }
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim->Instance == TIM2){
        DEBUG_PRINTF("TIM2 elapsed, sending encoder request\n");
        Send_Request();
    }
    else if(htim->Instance == TIM4)
    {
        static int count_tim4=0;
        if(count_tim4++>=20)
        {
            count_tim4=0;
            feedback_send_flag=1;
            HAL_GPIO_TogglePin(LED1_GPIO_Port,LED1_Pin);
            // DEBUG_PRINTF("Feedback flag set\n");
        }
        
        if(uart2_doutntime_rx)
        {
            uart2_doutntime_rx--;
        }
        else
        {
            cmd_state = CMD_STATE_IDLE;
            cmd_idx = 0;
            // ?????
            for(int i=0;i<CMD_BUFFER_SIZE;i++) cmd_buffer[i]=0;
            // ?? UART ?????????????
            if(__HAL_UART_GET_FLAG(&huart2, UART_FLAG_ORE) != RESET) {
               __HAL_UART_CLEAR_OREFLAG(&huart2);
            }
            HAL_UART_Receive_IT(&huart2, &rx_byte_uart2, 1);
            // DEBUG_PRINTF("UART2 timeout, resetting command state\n");
        }
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    // --- START: MODIFIED USART1 CALLBACK FOR MODBUS ---
    if (huart->Instance == USART1)
    {
        // ????????,??????
        if (encoder_rx_idx < ENCODER_RESPONSE_SIZE)
        {
            encoder_rx_buffer[encoder_rx_idx] = rx_byte;
            
            // ?????????,???????
            if (encoder_rx_idx == 0) {
                uart1_rx_timeout_counter = 10; // 10ms ??
            }
            
            encoder_rx_idx++;
        }

        // ?????7???
        if (encoder_rx_idx >= ENCODER_RESPONSE_SIZE)
        {
            uart1_rx_timeout_counter = 0; // ?????,????
            
            // 1. ??CRC
            // ????CRC?????,?????
            uint16_t received_crc = (encoder_rx_buffer[6] << 8) | encoder_rx_buffer[5];
            // ???5????CRC
            uint16_t calculated_crc = crc16((uint8_t*)encoder_rx_buffer, 5);

            if (received_crc == calculated_crc)
            {
                // 2. ?????????? (?????)
                if (encoder_rx_buffer[0] == 0x01 && encoder_rx_buffer[1] == 0x03)
                {
                    // 3. ???? (????, ?????)
                    MGT_angle = (encoder_rx_buffer[3] << 8) | encoder_rx_buffer[4];
                    MGT_angle_rx_flag = 1;
                    DEBUG_PRINTF("Valid encoder data (Modbus): %d\n", MGT_angle);
                }
                else
                {
                    DEBUG_PRINTF("Encoder response error: Invalid Slave ID or Func Code\n");
                }
            }
            else
            {
                DEBUG_PRINTF("Encoder Modbus CRC error! Received: 0x%04X, Calculated: 0x%04X\n",
                             received_crc, calculated_crc);
            }

            // ??????,???????
            encoder_rx_idx = 0;
        }

        // ??????,???????
        HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
    }
    // --- END: MODIFIED USART1 CALLBACK FOR MODBUS ---
    else if (huart->Instance == USART2) {
        char received_char = rx_byte_uart2;
        // DEBUG_PRINTF("USART2 received: %c (0x%02X)\n", received_char, received_char);
        
        uart2_doutntime_rx=300;

        switch(cmd_state) {
            case CMD_STATE_IDLE:
                if (received_char == 'A' || received_char == 'a' ||
                    received_char == 'B' || received_char == 'b'||
                    received_char == 'E' || received_char == 'e'||
                    received_char == 'Q' || received_char == 'q') {
                    // DEBUG_PRINTF("Command start: %c\n", received_char);
                    cmd_buffer[0] = received_char;
                    cmd_idx = 1;
                    cmd_state = CMD_STATE_RECEIVE_EQUALS;
                }
                else if (received_char == 'C' || received_char == 'c') {
                    // DEBUG_PRINTF("C command received\n");
                    printf("Angle: %.6f rad, Raw: %d, Rotation: %ld\r\n", 
                           getAngle(), MGT_angle, rotation_count);
                    cmd_state = CMD_STATE_IDLE;
                    cmd_idx = 0;
                }
                break;

            case CMD_STATE_RECEIVE_EQUALS:
                if (received_char == '=') 
                {
                    // DEBUG_PRINTF("Equals received\n");
                    cmd_buffer[cmd_idx++] = received_char;
                    if (cmd_buffer[0]  == 'A' || cmd_buffer[0] == 'a') {
                        cmd_state = CMD_STATE_RECEIVE_DATA_A;
                    }
                    else if (cmd_buffer[0]  == 'B' || cmd_buffer[0] == 'b') {
                        cmd_state = CMD_STATE_RECEIVE_DATA_B;
                    }
                    else if (cmd_buffer[0] == 'E' || cmd_buffer[0] == 'e') {
                        cmd_state = CMD_STATE_RECEIVE_DATA_E;
                    }
                    else if (cmd_buffer[0] == 'Q' || cmd_buffer[0] == 'q') {
                        cmd_state = CMD_STATE_RECEIVE_DATA_Q;
                    }
                }
                else {
                    cmd_state = CMD_STATE_IDLE;
                    cmd_idx = 0;
                }
                break;

            // --- START: RESTORED COMMAND PARSING LOGIC ---
            case CMD_STATE_RECEIVE_DATA_A:
                if (received_char == ';') 
                {
                    cmd_buffer[cmd_idx] = '\0'; // Null-terminate the string
                    // Use atof to parse the floating point number after '='
                    target_angle = atof((const char*)(cmd_buffer + 2));
                    target_angle *= reduction_ratio;
                    
                    printf("Set Target Angle: %.3f rad\r\n", target_angle);
                    command_ready = 1;
                    
                    cmd_state = CMD_STATE_IDLE;
                    cmd_idx = 0;
                }
                else 
                {
                    if(cmd_idx < CMD_BUFFER_SIZE - 1) {
                        cmd_buffer[cmd_idx++] = received_char;
                    }
                    else { // Buffer overflow
                        cmd_state = CMD_STATE_IDLE;
                        cmd_idx = 0;
                    }
                }
                break;

            case CMD_STATE_RECEIVE_DATA_B:
                if (received_char == ';') 
                {
                    cmd_buffer[cmd_idx] = '\0';
                    // Use atoi to parse the integer after '='
                    int ratio = atoi((const char*)(cmd_buffer + 2));

                    if (ratio < 1) ratio = 1;
                    if (ratio > 1000) ratio = 1000;

                    reduction_ratio = ratio;
                    printf("Set Reduction Ratio: %d\r\n", reduction_ratio);
                    command_ready = 1;

                    cmd_state = CMD_STATE_IDLE;
                    cmd_idx = 0;
                }
                else 
                {
                    if (cmd_idx < (CMD_BUFFER_SIZE_B - 1)) 
                    {
                        cmd_buffer[cmd_idx++] = received_char;
                    }
                    else // Buffer overflow
                    {
                        cmd_state = CMD_STATE_IDLE;
                        cmd_idx = 0;
                    }
                }
                break;

            case CMD_STATE_RECEIVE_DATA_E:
                if (received_char == ';') 
                {
                    if(cmd_idx == 3) { // Expects E=1; or E=0;
                        if(cmd_buffer[2] == '1') {
                            motor_enabled = 1;
                            printf("Motor Enabled\r\n");
                        } else if(cmd_buffer[2] == '0') {
                            motor_enabled = 0;
                            voltage.q = 0;
                            voltage.d = 0;
                            printf("Motor Disabled\r\n");
                        }
                    }
                    command_ready = 1;
                    cmd_state = CMD_STATE_IDLE;
                    cmd_idx = 0;
                }
                else 
                {
                    if (cmd_idx < 3) {
                        cmd_buffer[cmd_idx++] = received_char;
                    }
                    else { // Command too long
                        cmd_state = CMD_STATE_IDLE;
                        cmd_idx = 0;
                    }
                }
                break;
            case CMD_STATE_RECEIVE_DATA_Q:
                if (received_char == ';') {
                    cmd_buffer[cmd_idx] = '\0';
                    if (cmd_idx > 2) { // Q=...;
                        char *data_start = (char *)&cmd_buffer[2];
                        int32_t new_count = atoi(data_start);
                        rotation_count = new_count;
                        full_rotation_offset = rotation_count * _2PI;
                        printf("Q:%ld\r\n", rotation_count); 
                    } else { // Q;
                        printf("Q:%ld\r\n", rotation_count);
                    }
                    cmd_state = CMD_STATE_IDLE;
                    cmd_idx = 0;
                } else {
                    if (cmd_idx < CMD_BUFFER_SIZE - 1) {
                        cmd_buffer[cmd_idx++] = received_char;
                    } else { // Buffer overflow
                        cmd_state = CMD_STATE_IDLE;
                        cmd_idx = 0;
                    }
                }
                break;
            // --- END: RESTORED COMMAND PARSING LOGIC ---

            default:
                cmd_state = CMD_STATE_IDLE;
                cmd_idx = 0;
                break;
        }

        HAL_UART_Receive_IT(&huart2, &rx_byte_uart2, 1);
    }
}
