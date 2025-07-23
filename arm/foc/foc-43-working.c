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


extern float setSpeed;          
extern float setCurrent;         
extern float setPosition ;       

extern ADC_HandleTypeDef hadc1;

float shaft_angle,sensor_offset,zero_electric_angle,electrical_angle;
int pole_pairs=1,pattern=0;
char sensor_direction=1;
long angle_data, angle_data_prev;
float full_rotation_offset;      
extern uint8_t isRunning ;             

DQVoltage_s voltage;                  
Kalman kalman_current_Iq_Filter;
DQCurrent_s current;                  

#define ENCODER_RESOLUTION 128

volatile uint8_t motor_enabled = 0;  


#define ENCODER_MAX_COUNT 65535

int32_t rotation_count = 0; 


int16_t last_count = 0;      
float shaft_angle = 0.0f;    
float Target_speed,shaft_velocity,full_rotation_offset;
int MGT_angle;
char MGT_angle_rx_flag=0;
char feedback_send_flag;
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
	





	
  return adc_;
}


PhaseCurrent_s getPhaseCurrents(void)
{
	PhaseCurrent_s current;
	ADC_1011=_readADCVoltageInline();
	current.a = (ADC_1011.adc_10 - 1.65)*10;
	current.b = (ADC_1011.adc_11 - 1.67)*10;



	current.c = 0; 
	
	return current;
}

DQCurrent_s getFOCCurrents(float angle_el)
{
	PhaseCurrent_s current;
	float i_alpha, i_beta;
	float ct,st;
	DQCurrent_s ret;
	
	
	current = getPhaseCurrents();



	
	if(!current.c)
	{
		
		i_alpha = -current.b;  
		i_beta = _1_SQRT3 * current.b + _2_SQRT3 * current.a;
	}
	else
	{
		
		float mid = (1.f/3) * (current.a + current.b + current.c);
		float a = current.a - mid;
		float b = current.b - mid;
		i_alpha = a;
		i_beta = _1_SQRT3 * a + _2_SQRT3 * b;
	}
	
	
	
	ct = _cos(angle_el);
	st = _sin(angle_el);
	ret.d = i_alpha * ct + i_beta * st;
	ret.q = i_beta * ct - i_alpha * st;
	

	ret.q=Kalman_Filter(&kalman_current_Iq_Filter,ret.q);
	ret.q= Slid_Filter(&Filter_ret_q,ret.q,40);





	return ret;
}
/******************************************************************************/


float Position_KP		=0.15,Position_KI=0.001,Position_KD=0.32;          /* Î»ï¿½ï¿½Ê½PIDÏµï¿½ï¿½ */
float Incremental_KP=1.1,Incremental_KI=0.09,Incremental_KD=0.25;   /* ï¿½ï¿½ï¿½ï¿½Ê½PIDÏµï¿½ï¿½ */


float Position_PID(float reality,float target)
{ 	
    static float Bias,Pwm,Last_Bias,Integral_bias=0;
    
    Bias=target-reality;                            /* ï¿½ï¿½ï¿½ï¿½Æ«ï¿½ï¿½ */
    Integral_bias+=Bias;	                        /* Æ«ï¿½ï¿½ï¿½Û»ï¿½ */
    
    if(Integral_bias> 5) Integral_bias = 5;   /* ï¿½ï¿½ï¿½ï¿½ï¿½Þ·ï¿½ */
    if(Integral_bias<-5) Integral_bias =-5;
    
    Pwm = (Position_KP*Bias)                        /* ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ */
         +(Position_KI*Integral_bias)               /* ï¿½ï¿½ï¿½Ö»ï¿½ï¿½ï¿½ */
         +(Position_KD*(Bias-Last_Bias));           /* Î¢ï¿½Ö»ï¿½ï¿½ï¿½ */
    
    Last_Bias=Bias;                                 /* ï¿½ï¿½ï¿½ï¿½ï¿½Ï´ï¿½Æ«ï¿½ï¿½ */
    return Pwm;                                     /* ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ */
}

float Bias_buf;

float Incremental_PID(float reality,float target)
{ 	
	 static float Bias,Pwm,Last_bias=0,Prev_bias=0;
    
	
	 Bias=target-reality;                                   /* ï¿½ï¿½ï¿½ï¿½Æ«ï¿½ï¿½ */
    Bias_buf=Bias;
	 Pwm += (Incremental_KP*(Bias-Last_bias))               /* ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ */
           +(Incremental_KI*Bias)                           /* ï¿½ï¿½ï¿½Ö»ï¿½ï¿½ï¿½ */
           +(Incremental_KD*(Bias-2*Last_bias+Prev_bias));  /* Î¢ï¿½Ö»ï¿½ï¿½ï¿½ */ 	
   Prev_bias=Last_bias;                                   /* ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ï´ï¿½Æ«ï¿½ï¿½ */
	 Last_bias=Bias;	                                    /* ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ò»ï¿½ï¿½Æ«ï¿½ï¿½ */
		
		Pwm=_constrain(Pwm,-3,3);
	 return Pwm;                                            /* ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ */
}
float electricalAngle(void)
{
    return _normalizeAngle((shaft_angle + sensor_offset) * pole_pairs - zero_electric_angle);
}

void setPhaseVoltage(float Uq, float Ud, float angle_el)
{
    float Uout;               
    uint32_t sector;          
    float T0, T1, T2;         
    float Ta, Tb, Tc;         
    
    
    if(Uq > voltage_limit) Uq = voltage_limit;
    if(Uq < -voltage_limit) Uq = -voltage_limit;
    
    if(Ud > voltage_limit) Ud = voltage_limit;
    if(Ud < -voltage_limit) Ud = -voltage_limit;
    
    
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
    
    
    if(Uout > 0.577) Uout = 0.577;
    if(Uout < -0.577) Uout = -0.577;
    
    
    sector = (angle_el / _PI_3) + 1;
    
    T1 = _SQRT3 * _sin(sector * _PI_3 - angle_el) * Uout;
    T2 = _SQRT3 * _sin(angle_el - (sector - 1.0) * _PI_3) * Uout;
    
    T0 = 1 - T1 - T2;
    
    
    switch(sector)
    {
        case 1:
            Ta = T1 + T2 + T0 / 2;
            Tb = T2 + T0 / 2;
            Tc = T0 / 2;
            break;
        case 2:
            Ta = T1 + T0 / 2;
            Tb = T1 + T2 + T0 / 2;
            Tc = T0 / 2;
            break;
        case 3:
            Ta = T0 / 2;
            Tb = T1 + T2 + T0 / 2;
            Tc = T2 + T0 / 2;
            break;
        case 4:
            Ta = T0 / 2;
            Tb = T1 + T0 / 2;
            Tc = T1 + T2 + T0 / 2;
            break;
        case 5:
            Ta = T2 + T0 / 2;
            Tb = T0 / 2;
            Tc = T1 + T2 + T0 / 2;
            break;
        case 6:
            Ta = T1 + T2 + T0 / 2;
            Tb = T0 / 2;
            Tc = T1 + T0 / 2;
            break;
        default:
            Ta = 0;
            Tb = 0;
            Tc = 0;
    }
    
    
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, Ta * PWM_Period); 
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, Tb * PWM_Period); 
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, Tc * PWM_Period); 

}




float getAngle(void)
{
    float d_angle; 

    angle_data =  MGT_angle ; 

    
    d_angle = angle_data - angle_data_prev;

    
    
        

		if(fabs(d_angle) > (0.8 * 16383)) {
        if (d_angle > 0) {
            full_rotation_offset -= _2PI;
            rotation_count--; 
        } else {
            full_rotation_offset += _2PI;
            rotation_count++; 
        }
    }

    
    angle_data_prev = angle_data;

    
    return (full_rotation_offset + (angle_data * 1.0 / 16383) * _2PI);
}

 float current_target_angle = 0.0f;
 float initial_angle = 0.0f;
 char motion_complete = 0;
char test_flag;
 float total_target_angle = 8.0f * (float)_PI; 
 float desired_speed = 2.0f * (float)_PI;      
#define CONTROL_PERIOD 0.1f


/* ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½×´Ì¬ */
typedef enum {
    CMD_STATE_WAIT_START,      
    CMD_STATE_RECEIVE_COMMAND,  
} CMD_State;

/* ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ */
typedef enum {
    CMD_NONE,                
    CMD_SET_REDUCTION_RATIO, 
    CMD_SET_TARGET_ANGLE     
} CommandType;

/* ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½î»ºï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ð¡ */
#define CMD_BUFFER_SIZE 64

/* ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ø±ï¿½ï¿½ï¿½ */

volatile char cmd_buffer[CMD_BUFFER_SIZE];
volatile uint8_t cmd_idx = 0;
volatile CommandType received_command = CMD_NONE;
volatile float target_angle = 0.0f;
volatile int reduction_ratio = 1;
volatile uint8_t command_ready = 0; 

uint8_t tx_buffer[10] = {0x02};     
uint8_t rx_buffer[10];     

typedef enum {
    WAIT_FOR_START,
    WAIT_FOR_IDENT,
    WAIT_FOR_DATA1,
    WAIT_FOR_DATA2,
    WAIT_FOR_CHECKSUM
} ReceiveState;
volatile ReceiveState current_state = WAIT_FOR_START;
volatile uint8_t received_bytes[4]; 
volatile uint8_t byte_index = 0;
uint8_t rx_byte,rx_byte_uart2; 
int uart2_doutntime_rx;


#define RS485_TX_MODE() HAL_GPIO_WritePin(RS485_EN_GPIO_Port, RS485_EN_Pin, GPIO_PIN_SET)   
#define RS485_RX_MODE() HAL_GPIO_WritePin(RS485_EN_GPIO_Port, RS485_EN_Pin, GPIO_PIN_RESET) 


void Send_Request(void)
{
    uint8_t tx_buffer = 0x02;
    RS485_TX_MODE(); 
    HAL_UART_Transmit(&huart1, &tx_buffer, 1, HAL_MAX_DELAY);
    RS485_RX_MODE(); 
}

/* Definitions ---------------------------------------------------------------*/
#define CMD_BUFFER_SIZE_A 12  
#define CMD_BUFFER_SIZE_B 8   

/* Enumerations --------------------------------------------------------------*/
typedef enum {
    CMD_STATE_IDLE,            
    CMD_STATE_RECEIVE_EQUALS,  
    CMD_STATE_RECEIVE_DATA_A,  
    CMD_STATE_RECEIVE_DATA_B,  
		CMD_STATE_RECEIVE_DATA_C,  
    CMD_STATE_RECEIVE_DATA_E,  
		CMD_STATE_RECEIVE_DATA_Q,  
    CMD_STATE_RECEIVE_TERMINATOR 
} cmd_state_t;
volatile cmd_state_t cmd_state = CMD_STATE_IDLE;


char current_cmd_type = '\0';

void motorEnable(void)
{
    motor_enabled = 1;
    
    printf("Motor enabled\r\n");
}

void motorDisable(void)
{
    motor_enabled = 0;
    
    setPhaseVoltage(0, 0, 0);
    
    printf("Motor disabled\r\n");
}


extern float Bias_buf;
void run()
{    
    if (pattern == 0)
	{
		voltage.q = 0;
		voltage.d = 0;

		PID_current_q.P =  0.08;
		PID_current_q.I =  0.05;
		PID_current_q.D =  0;
		PID_current_q.limit = 2;
		
		pole_pairs=11;  
		
		zero_electric_angle =2.3;
		pattern=4;
		HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
		HAL_UART_Receive_IT(&huart2, &rx_byte_uart2, 1);  /* 2Ð¶,Ä¿Ç¶ÈºÍ¼Ù±È£Ò»Ö½ */
	}

	shaft_angle = getAngle();
    electrical_angle = electricalAngle();

    if (!motor_enabled) {
        
        voltage.q = 0;
        voltage.d = 0;
        setPhaseVoltage(0, 0, 0);
        return;
    }

	switch(pattern)
	{
		case 2:
			printf("MOT: Align sensor.\r\n");  

			float mid_angle,end_angle;
			float moved;

			float angle;
			for(int i=0; i<=500; i++)
			{
				angle = _3PI_2 + _2PI * i / 500.0;  
				setPhaseVoltage(1, 0,  angle);  
				HAL_Delay(2);
			}
			mid_angle= getAngle();  
			
			for(int i=500; i>=0; i--) 
			{
				angle = _3PI_2 + _2PI * i / 500.0;
				setPhaseVoltage(1, 0,  angle);
				HAL_Delay(2);
			}
			end_angle= getAngle();  
			setPhaseVoltage(0, 0, 0);  
			HAL_Delay(200);

			printf("mid_angle=%.4f\r\n",mid_angle);
			printf("end_angle=%.4f\r\n",end_angle);
			
			moved =  fabs(mid_angle - end_angle);  
			if((mid_angle == end_angle)||(moved < 0.02))  
			{
				printf("MOT: Failed to notice movement------------------------------------- \r\n");				
			}
			else if(mid_angle < end_angle)
			{
				printf("MOT: sensor_direction==CCW\r\n");  
				sensor_direction=CCW;
			}
			else
			{
				printf("MOT: sensor_direction==CW\r\n");  
				sensor_direction=CW;
			}
			setPhaseVoltage(1, 0,  _3PI_2);  
			HAL_Delay(700);
			printf("end_angle--------_3PI_2 =%.4f\r\n", getAngle());

			zero_electric_angle = _normalizeAngle(sensor_direction* getAngle() * 7 );  
			HAL_Delay(20);
			printf("MOT: Zero elec. angle:");
			printf("%.4f\r\n",zero_electric_angle);
			
			setPhaseVoltage(0, 0, 0);  
			HAL_Delay(200);
			
			pattern=3;
			break;
		case 3:
			
			voltage.q =0.5;
			voltage.d =0;
		  break;
    case 4:
			P_angle.P = 5;
			P_angle.I = 0;
			P_angle.D = 0;		
			voltage.q = PIDoperator(&P_angle, (shaft_angle - target_angle)); 		

			break;
	}
	
    
	setPhaseVoltage(voltage.q, voltage.d, electrical_angle);

	if(feedback_send_flag==1)
	{
		feedback_send_flag=0;
		
		/*
		*/
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
	}
}


/* USER CODE BEGIN 1 */    
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if(htim->Instance == TIM2){                              

		
			{

		
					Send_Request();





			}
	}
	else
	if(htim->Instance == TIM4)
	{                              
		static int count_tim4=0;
		if(count_tim4++>=20)
		{
			count_tim4=0;
			feedback_send_flag=1;
			HAL_GPIO_TogglePin(LED1_GPIO_Port,LED1_Pin);
		}
		
		if(uart2_doutntime_rx)
		{
			uart2_doutntime_rx--;
		}
		else
		{
	
			uart2_doutntime_rx=300;
			cmd_state = CMD_STATE_IDLE;
			cmd_idx = 0;
			for(int i=0;i<10;i++)cmd_buffer[i]=0;
			HAL_UART_Receive_IT(&huart2, &rx_byte_uart2, 1);
			
			uint8_t data = (uint8_t)(huart2.Instance->DR);
			
			__HAL_UART_CLEAR_OREFLAG(&huart2);

		}
	}

}
/* USER CODE END 1 */


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        switch(current_state)
        {
            case WAIT_FOR_START:
                if (rx_byte == 0x02)
                {
                    current_state = WAIT_FOR_IDENT;
                }
                break;

            case WAIT_FOR_IDENT:
                if (rx_byte == 0x08)
                {
                    current_state = WAIT_FOR_DATA1;
                }
                else
                {
                    
                    current_state = WAIT_FOR_START;
                }
                break;

            case WAIT_FOR_DATA1:
                received_bytes[0] = rx_byte;
                current_state = WAIT_FOR_DATA2;
                break;

            case WAIT_FOR_DATA2:
                received_bytes[1] = rx_byte;
                current_state = WAIT_FOR_CHECKSUM;
                break;

            case WAIT_FOR_CHECKSUM:
                received_bytes[2] = rx_byte;
                
                {
                    uint8_t calculated_checksum = 0x02 ^ 0x08 ^ received_bytes[0] ^ received_bytes[1];
                    if (rx_byte == calculated_checksum)
                    {
                        
                        MGT_angle = -( received_bytes[0] + (received_bytes[1] << 8));
										
                        MGT_angle_rx_flag = 1;
                    }
                    
                    current_state = WAIT_FOR_START;
                }
                break;

            default:
                current_state = WAIT_FOR_START;
                break;
        }

        
        HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
    }
		else
		if (huart->Instance == USART2) {
				
        char received_char = rx_byte_uart2; 
				uart2_doutntime_rx=300;
        
        

        switch(cmd_state) {
            case CMD_STATE_IDLE:
                if (received_char == 'A' || received_char == 'a' ||
                    received_char == 'B' || received_char == 'b'||
                    received_char == 'E' || received_char == 'e'||
										received_char == 'Q' || received_char == 'q') {
                        cmd_buffer[0] = received_char;
                        cmd_idx = 1;
                        cmd_state = CMD_STATE_RECEIVE_EQUALS;
                }
										
                if (received_char == 'C' || received_char == 'c') {
                    
                    printf("%.6f\r\n", getAngle());
                    cmd_state = CMD_STATE_IDLE;
                    cmd_idx = 0;
                }

                
                break;

            case CMD_STATE_RECEIVE_EQUALS:
                if (received_char == '=') 
								{
                    cmd_buffer[cmd_idx++] = received_char;
                    if (cmd_buffer[0]  == 'A') {
                        cmd_state = CMD_STATE_RECEIVE_DATA_A;
                    }
                    else if (cmd_buffer[0]  == 'B') {
                        cmd_state = CMD_STATE_RECEIVE_DATA_B;
                    }
                    else if (cmd_buffer[0] == 'E') {
                        cmd_state = CMD_STATE_RECEIVE_DATA_E;
                    }else if (cmd_buffer[0] == 'Q') {
                        cmd_state = CMD_STATE_RECEIVE_DATA_Q;
                    }

                }
                else {
                    
                    cmd_state = CMD_STATE_IDLE;
                    cmd_idx = 0;
										for(int i=0;i<10;i++)cmd_buffer[i]=0;
									
                }
								
                break;

            case CMD_STATE_RECEIVE_DATA_A:
                if (received_char == ';'&&cmd_idx==10) 
									{

                    cmd_buffer[cmd_idx++] = received_char;
                    cmd_buffer[cmd_idx] = '\0'; 



                    

                    target_angle = (cmd_buffer[3]-'0')*100 + (cmd_buffer[4]-'0')*10 +(cmd_buffer[5]-'0')*1 + (cmd_buffer[7]-'0')*0.1 + (cmd_buffer[8]-'0')*0.01 +( cmd_buffer[9]-'0')*0.001 ;
										target_angle *=reduction_ratio;
										if(cmd_buffer[2]=='-')target_angle*=-1;
										 printf("Set Target Angle: %.3f rad\r\n", target_angle);

                    command_ready = 1;

                    
                    cmd_state = CMD_STATE_IDLE;
                    cmd_idx = 0;
										for(int i=0;i<10;i++)cmd_buffer[i]=0;
                }
                else 
								{

									cmd_buffer[cmd_idx++] = received_char;

									
								}
								break;

            case CMD_STATE_RECEIVE_DATA_B:
							if (received_char == ';') 
							{
									cmd_buffer[cmd_idx++] = received_char;
									cmd_buffer[cmd_idx] = '\0'; 

									
									if (cmd_idx >= 4 && cmd_buffer[0] == 'B' && cmd_buffer[1] == '=') 
									{
											uint32_t ratio = 0;
											uint8_t digit_count = 0;
											
											
											for (uint8_t i = 2; i < cmd_idx - 1; i++) 
											{
													char c = cmd_buffer[i];
													if (c >= '0' && c <= '9') 
													{
															ratio = ratio * 10 + (c - '0');
															digit_count++;
													}
													else 
													{
															printf("Invalid digit in ratio\r\n");
															goto reset_state; 
													}
											}

											if (digit_count > 0) 
											{
													
													if (ratio < 1) ratio = 1;
													if (ratio > 1000) ratio = 1000;

													reduction_ratio = ratio;
													printf("Set Reduction Ratio: %d\r\n", reduction_ratio);
													command_ready = 1;
											}
											else 
											{
													printf("No valid digits after B=\r\n");
											}
									}
									else 
									{
											printf("Invalid B command format (expected 'B=...;')\r\n");
									}

					reset_state:
									
									cmd_state = CMD_STATE_IDLE;
									cmd_idx = 0;
							}
							else 
							{
									
									if (cmd_idx < (CMD_BUFFER_SIZE_B - 1)) 
									{
											cmd_buffer[cmd_idx++] = received_char;
									}
									else 
									{
											printf("Error: Command too long\r\n");
											cmd_state = CMD_STATE_IDLE;
											cmd_idx = 0;
									}
							}
							break;

            case CMD_STATE_RECEIVE_DATA_E:
                if (received_char == ';' && cmd_idx == 3) 
                {
                    cmd_buffer[cmd_idx++] = received_char;
                    cmd_buffer[cmd_idx] = '\0';

                    if(cmd_buffer[2] == '1') {
                        motor_enabled = 1;
                        printf("Motor Enabled\r\n");
                    } else if(cmd_buffer[2] == '0') {
                        motor_enabled = 0;
                        voltage.q = 0;
                        voltage.d = 0;
                        printf("Motor Disabled\r\n");
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
                    else {
                        cmd_state = CMD_STATE_IDLE;
                        cmd_idx = 0;
                    }
                }
                break;
						case CMD_STATE_RECEIVE_DATA_Q:
							if (received_char == ';') {
									cmd_buffer[cmd_idx] = '\0'; 
									if (cmd_idx > 2) { 
											char *data_start = (char *)&cmd_buffer[2]; 
											if (*data_start == '=') { 
													data_start++; 
													int32_t new_count = atoi(data_start);
													rotation_count = new_count;
													full_rotation_offset = rotation_count * _2PI;
													printf("Q:%ld\r\n", rotation_count); 
											} else {
													printf("ERROR: Invalid Q format\r\n");
											}
									} else { 
											printf("Q:%ld\r\n", rotation_count);
									}
									cmd_state = CMD_STATE_IDLE;
									cmd_idx = 0;
							} else {
									if (cmd_idx < CMD_BUFFER_SIZE) {
											cmd_buffer[cmd_idx++] = received_char;
									} else {
											cmd_state = CMD_STATE_IDLE;
											cmd_idx = 0;
									}
							}
							break;

            default:
                
                cmd_state = CMD_STATE_IDLE;
                cmd_idx = 0;
                break;
        }

        
        HAL_UART_Receive_IT(&huart2, &rx_byte_uart2, 1);
		}
}
