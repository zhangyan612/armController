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


extern float setSpeed;          // �ٶ��趨ֵ
extern float setCurrent;         // �����趨ֵ
extern float setPosition ;       // λ���趨ֵ

extern ADC_HandleTypeDef hadc1;

float shaft_angle,sensor_offset,zero_electric_angle,electrical_angle;
int pole_pairs=1,pattern=0;
char sensor_direction=-1;
long angle_data, angle_data_prev;// ��ǰ��ǰһ�εĽǶ�����
float full_rotation_offset;      // ȫת�Ƕ�ƫ������������չ�Ƕȷ�Χ
extern uint8_t isRunning ;             // ����״̬��־

DQVoltage_s voltage;                  //!< DQ����ϵ�µĵ�ѹ
Kalman kalman_current_Iq_Filter;
DQCurrent_s current;                  //!< DQ����ϵ�µĵ���
// �������ź���
#define ENCODER_RESOLUTION 128

volatile uint8_t motor_enabled = 0;  // ���ʹ�ܱ�־

// ��������ʼ������ֵ
#define ENCODER_MAX_COUNT 65535
// rotatation count
int32_t rotation_count = 0; // rotatation

// ��������
int16_t last_count = 0;      // ��һ�εļ���ֵ�����ڼ���������
float shaft_angle = 0.0f;    // ��ǰ����Ƕȣ����ȣ�
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
    
//	HAL_ADC_Start(&hadc1);
//	HAL_ADC_PollForConversion(&hadc1,50);
//	Temperature=HAL_ADC_GetValue(&hadc1);
//	HAL_ADC_Stop (&hadc1);

    
  return adc_;
}


PhaseCurrent_s getPhaseCurrents(void)
{
    PhaseCurrent_s current;
    ADC_1011=_readADCVoltageInline();
    current.a = (ADC_1011.adc_10 - 1.65)*10;// amps
    current.b = (ADC_1011.adc_11 - 1.67)*10;// amps
//	current.a = (ADC_1011.adc_10 - 0)*1;// amps
//	current.b = (ADC_1011.adc_11 - 0)*1;// amps

    current.c = 0; // amps
    
    return current;
}

DQCurrent_s getFOCCurrents(float angle_el)
{
    PhaseCurrent_s current;
    float i_alpha, i_beta;
    float ct,st;
    DQCurrent_s ret;
    
    // read current phase currents
    current = getPhaseCurrents();
//	current.a = Slid_Filter(&Current_a,current.a,10);
//	current.b = Slid_Filter(&Current_b,current.b,10);
//	printf("%.2f,%.2f\n",current.a,current.b);
    // calculate clarke transform
    if(!current.c)
    {
        // if only two measured currents
        i_alpha = -current.b;  
        i_beta = _1_SQRT3 * current.b + _2_SQRT3 * current.a;
    }
    else
    {
        // signal filtering using identity a + b + c = 0. Assumes measurement error is normally distributed.
        float mid = (1.f/3) * (current.a + current.b + current.c);
        float a = current.a - mid;
        float b = current.b - mid;
        i_alpha = a;
        i_beta = _1_SQRT3 * a + _2_SQRT3 * b;
    }
    
    // calculate park transform
    
    ct = _cos(angle_el);
    st = _sin(angle_el);
    ret.d = i_alpha * ct + i_beta * st;
    ret.q = i_beta * ct - i_alpha * st;//-0.21*-0.19-0.16*0.98
    

    ret.q=Kalman_Filter(&kalman_current_Iq_Filter,ret.q);
    ret.q= Slid_Filter(&Filter_ret_q,ret.q,40);
//	ret.q=Kalman_Filter(&kalman_current_Iq_Filter,ret.q);
//	printf("%.2f\n",ret.q);
//		printf("%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",current.a,current.b,ret.q,0.0,shaft_velocity,zero_electric_angle);
//printf("%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",current.a,current.b,ret.q,0.0,i_alpha,i_beta,ct,st,(angle_el-3.14)/3);

    return ret;
}
/******************************************************************************/


float Position_KP		=0.15,Position_KI=0.001,Position_KD=0.32;          /* λ��ʽPIDϵ�� */
float Incremental_KP=1.1,Incremental_KI=0.09,Incremental_KD=0.25;   /* ����ʽPIDϵ�� */


float Position_PID(float reality,float target)
{ 	
    static float Bias,Pwm,Last_Bias,Integral_bias=0;
    
    Bias=target-reality;                            /* ����ƫ�� */
    Integral_bias+=Bias;	                        /* ƫ���ۻ� */
    
    if(Integral_bias> 5) Integral_bias = 5;   /* �����޷� */
    if(Integral_bias<-5) Integral_bias =-5;
    
    Pwm = (Position_KP*Bias)                        /* �������� */
         +(Position_KI*Integral_bias)               /* ���ֻ��� */
         +(Position_KD*(Bias-Last_Bias));           /* ΢�ֻ��� */
    
    Last_Bias=Bias;                                 /* �����ϴ�ƫ�� */
    return Pwm;                                     /* ������ */
}

float Bias_buf;

float Incremental_PID(float reality,float target)
{ 	
     static float Bias,Pwm,Last_bias=0,Prev_bias=0;
    
    
     Bias=target-reality;                                   /* ����ƫ�� */
    Bias_buf=Bias;
     Pwm += (Incremental_KP*(Bias-Last_bias))               /* �������� */
           +(Incremental_KI*Bias)                           /* ���ֻ��� */
           +(Incremental_KD*(Bias-2*Last_bias+Prev_bias));  /* ΢�ֻ��� */ 	
   Prev_bias=Last_bias;                                   /* �������ϴ�ƫ�� */
     Last_bias=Bias;	                                    /* ������һ��ƫ�� */
        
        Pwm=_constrain(Pwm,-3,3);
     return Pwm;                                            /* ������ */
}
float electricalAngle(void)
{
    return _normalizeAngle((shaft_angle + sensor_offset) * pole_pairs - zero_electric_angle);
}

void setPhaseVoltage(float Uq, float Ud, float angle_el)
{
    float Uout;               // �����ѹ�Ĺ�һ����ֵ
    uint32_t sector;          // ��ǰ���ڵĵ���������1��6��
    float T0, T1, T2;         // �ռ�ʸ�������еĻ���ռ�ձ�
    float Ta, Tb, Tc;         // ABC�������ռ�ձ�
    
    // ����Q���ѹ��Ԥ���ĵ�ѹ���Ʒ�Χ��
    if(Uq > voltage_limit) Uq = voltage_limit;
    if(Uq < -voltage_limit) Uq = -voltage_limit;
    // ����D���ѹ��Ԥ���ĵ�ѹ���Ʒ�Χ��
    if(Ud > voltage_limit) Ud = voltage_limit;
    if(Ud < -voltage_limit) Ud = -voltage_limit;
    
    // ���D���ѹ��Ϊ0��˵��ͬʱ������Ud��Uq
    if(Ud) // ֻ����������Ud��Uqʱ
    {
        // �����һ����������ѹ��ֵ��ʹ�ý��Ƶ�ƽ���������Լ4%��
        Uout = _sqrt(Ud * Ud + Uq * Uq) / voltage_power_supply;
        // ���㲢��һ�������Ƕȣ�ȷ����0��2��֮��
        // ��һ������ʹ�ý��Ƶ�sin��cos����ʱ��Ҫ
        angle_el = _normalizeAngle(angle_el + atan2(Uq, Ud));
    }
    else
    {
        // ֻ��Uq����ʱ������ʹ��atan2��sqrt
        Uout = Uq / voltage_power_supply;
        // ���㲢��һ�������Ƕȣ�ƫ�Ʀ�/2��ȷ���Ƕ���0��2��֮��
        angle_el = _normalizeAngle(angle_el + _PI_2);
    }
    
    // ���������ѹ��ֵ��0.577���ڣ����ѹ���ֵ�Ľ���ֵ��
    if(Uout > 0.577) Uout = 0.577;
    if(Uout < -0.577) Uout = -0.577;
    
    // ���㵱ǰ���ڵ�������1��6��
    sector = (angle_el / _PI_3) + 1;
    // ����T1��T2�����ڵ�ǰ�����͵����Ƕ�
    T1 = _SQRT3 * _sin(sector * _PI_3 - angle_el) * Uout;
    T2 = _SQRT3 * _sin(angle_el - (sector - 1.0) * _PI_3) * Uout;
    // ����T0����Ϊʣ��ʱ��
    T0 = 1 - T1 - T2;
    
    // ���ݵ�ǰ��������ABC�������ռ�ձ�
    switch(sector)
    {
        case 1:
            /**
             * ����1��
             * A�ࣺT1 + T2 + T0/2
             * B�ࣺT2 + T0/2
             * C�ࣺT0/2
             */
            Ta = T1 + T2 + T0 / 2;
            Tb = T2 + T0 / 2;
            Tc = T0 / 2;
            break;
        case 2:
            /**
             * ����2��
             * A�ࣺT1 + T0/2
             * B�ࣺT1 + T2 + T0/2
             * C�ࣺT0/2
             */
            Ta = T1 + T0 / 2;
            Tb = T1 + T2 + T0 / 2;
            Tc = T0 / 2;
            break;
        case 3:
            /**
             * ����3��
             * A�ࣺT0/2
             * B�ࣺT1 + T2 + T0/2
             * C�ࣺT2 + T0/2
             */
            Ta = T0 / 2;
            Tb = T1 + T2 + T0 / 2;
            Tc = T2 + T0 / 2;
            break;
        case 4:
            /**
             * ����4��
             * A�ࣺT0/2
             * B�ࣺT1 + T0/2
             * C�ࣺT1 + T2 + T0/2
             */
            Ta = T0 / 2;
            Tb = T1 + T0 / 2;
            Tc = T1 + T2 + T0 / 2;
            break;
        case 5:
            /**
             * ����5��
             * A�ࣺT2 + T0/2
             * B�ࣺT0/2
             * C�ࣺT1 + T2 + T0/2
             */
            Ta = T2 + T0 / 2;
            Tb = T0 / 2;
            Tc = T1 + T2 + T0 / 2;
            break;
        case 6:
            /**
             * ����6��
             * A�ࣺT1 + T2 + T0/2
             * B�ࣺT0/2
             * C�ࣺT1 + T0/2
             */
            Ta = T1 + T2 + T0 / 2;
            Tb = T0 / 2;
            Tc = T1 + T0 / 2;
            break;
        default:
            /**
             * Ĭ����������ܵĴ���״̬����
             * �������ռ�ձ���Ϊ0��ϵͳ������κε�ѹ
             */
            Ta = 0;
            Tb = 0;
            Tc = 0;
    }
    
    // ������õ���ռ�ձȳ���PWM���ڣ����õ���ʱ���ıȽϼĴ���
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, Ta * PWM_Period); // ����A���ռ�ձ�
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, Tb * PWM_Period); // ����B���ռ�ձ�
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, Tc * PWM_Period); // ����C���ռ�ձ�

//		printf("%.3f,%.3f,%.3f\n",Ta * PWM_Period,Tb * PWM_Period,Tc * PWM_Period); // ��ѡ�ĵ����������ӡ��Ƕ�

}




float getAngle(void)
{
    float d_angle; // �Ƕȱ仯ֵ

    angle_data =  MGT_angle ; // ��ȡ��ǰ�ĽǶȼ���ֵ

    // ����Ƕȱ仯�������ڼ��ȫת
    d_angle = angle_data - angle_data_prev;

    // ����Ƕȱ仯��������ֵ��80%��CPR�����ж�Ϊ�����������ȫת��
    //if(fabs(d_angle) > (0.8 * 16383)) 
        //full_rotation_offset += d_angle > 0 ? -_2PI : _2PI; // ���ݷ������ȫתƫ����

        if(fabs(d_angle) > (0.8 * 16383)) {
        if (d_angle > 0) {
            full_rotation_offset -= _2PI;
            rotation_count--; 
        } else {
            full_rotation_offset += _2PI;
            rotation_count++; 
        }
    }

    // ���浱ǰ�Ƕȼ���ֵ��Ϊ�´μ�����׼��
    angle_data_prev = angle_data;

    // ���㲢������������Ƕȣ�����ȫתƫ�����͵�ǰ����ֵ��Ӧ�ĽǶ�
    return (full_rotation_offset + (angle_data * 1.0 / 16383) * _2PI);
}

 float current_target_angle = 0.0f;
 float initial_angle = 0.0f;
 char motion_complete = 0;
char test_flag;
 float total_target_angle = 8.0f * (float)_PI; // 8�� rad
 float desired_speed = 2.0f * (float)_PI;      // 2�� rad/s
#define CONTROL_PERIOD 0.1f


/* �����������״̬ */
typedef enum {
    CMD_STATE_WAIT_START,      // �ȴ�������ʼ�ַ�
    CMD_STATE_RECEIVE_COMMAND,  // ������������
} CMD_State;

/* ������������ */
typedef enum {
    CMD_NONE,                // ������
    CMD_SET_REDUCTION_RATIO, // �趨���ٱ�
    CMD_SET_TARGET_ANGLE     // �趨Ŀ��Ƕ�
} CommandType;

/* �������������С */
#define CMD_BUFFER_SIZE 64

/* �������������ر��� */

volatile char cmd_buffer[CMD_BUFFER_SIZE];
volatile uint8_t cmd_idx = 0;
volatile CommandType received_command = CMD_NONE;
volatile float target_angle = 0.0f;
volatile int reduction_ratio = 1;
volatile uint8_t command_ready = 0; // ������ɱ�־

uint8_t tx_buffer[10] = {0x02};     // ���ͻ�����
uint8_t rx_buffer[10];     // ���ջ�����

typedef enum {
    WAIT_FOR_START,
    WAIT_FOR_IDENT,
    WAIT_FOR_DATA1,
    WAIT_FOR_DATA2,
    WAIT_FOR_CHECKSUM
} ReceiveState;
volatile ReceiveState current_state = WAIT_FOR_START;
volatile uint8_t received_bytes[4]; // �洢��ʶ�ֽڡ�2�������ֽڼ�У���ֽ�
volatile uint8_t byte_index = 0;
uint8_t rx_byte,rx_byte_uart2; // ���������ֽ�
int uart2_doutntime_rx;

// RS485 �л�ģʽ
#define RS485_TX_MODE() HAL_GPIO_WritePin(RS485_EN_GPIO_Port, RS485_EN_Pin, GPIO_PIN_SET)   // ����ģʽ
#define RS485_RX_MODE() HAL_GPIO_WritePin(RS485_EN_GPIO_Port, RS485_EN_Pin, GPIO_PIN_RESET) // ����ģʽ


void Send_Request(void)
{
    uint8_t tx_buffer = 0x02;
    RS485_TX_MODE(); // �л�������ģʽ
    HAL_UART_Transmit(&huart1, &tx_buffer, 1, HAL_MAX_DELAY);
    RS485_RX_MODE(); // �л��ؽ���ģʽ
}

/* Definitions ---------------------------------------------------------------*/
#define CMD_BUFFER_SIZE_A 12  // A������󳤶ȣ�A=+123.456;
#define CMD_BUFFER_SIZE_B 8   // B������󳤶ȣ�B=0001;

/* Enumerations --------------------------------------------------------------*/
typedef enum {
    CMD_STATE_IDLE,            // ����״̬���ȴ�������ʼ�ַ�
    CMD_STATE_RECEIVE_EQUALS,  // �ѽ��յ�A/B���ȴ�=�ַ�
    CMD_STATE_RECEIVE_DATA_A,  // ����A��������ݲ���
    CMD_STATE_RECEIVE_DATA_B,  // ����B��������ݲ���
        CMD_STATE_RECEIVE_DATA_C,  // encoder value
    CMD_STATE_RECEIVE_DATA_E,  // ����E��������ݲ���
    CMD_STATE_RECEIVE_DATA_Q,  // Rotation
    CMD_STATE_RECEIVE_DATA_J,  // 校准（J命令）
    CMD_STATE_RECEIVE_TERMINATOR // ����������ֹ��;
} cmd_state_t;
volatile cmd_state_t cmd_state = CMD_STATE_IDLE;


char current_cmd_type = '\0';

void motorEnable(void)
{
    motor_enabled = 1;
    // Optional: Print a message to indicate the motor is enabled
    printf("Motor enabled\r\n");
}

void motorDisable(void)
{
    motor_enabled = 0;
    // Force the motor to stop by setting zero voltage
    setPhaseVoltage(0, 0, 0);
    // Optional: Print a message to indicate the motor is disabled
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
        HAL_UART_Receive_IT(&huart2, &rx_byte_uart2, 1);  /* 2ж,ĿǶȺͼٱȣһֽ */
    }

    shaft_angle = getAngle();
    electrical_angle = electricalAngle();

    if (!motor_enabled) {
        // Zero out the voltages to prevent any movement
        voltage.q = 0;
        voltage.d = 0;
        setPhaseVoltage(0, 0, 0);
        return;
    }

    switch(pattern)
    {
        case 2:
            printf("MOT: Align sensor.\r\n");  // ӡУ׼ʼϢ

            float mid_angle,end_angle;
            float moved;

            float angle;
            for(int i=0; i<=500; i++)
            {
                angle = _3PI_2 + _2PI * i / 500.0;  // Ƕ
                setPhaseVoltage(1, 0,  angle);  // Ӧѹ
                HAL_Delay(2);
            }
            mid_angle= getAngle();  // ȡмǶ
            
            for(int i=500; i>=0; i--) 
            {
                angle = _3PI_2 + _2PI * i / 500.0;
                setPhaseVoltage(1, 0,  angle);
                HAL_Delay(2);
            }
            end_angle= getAngle();  // ȡǶ
            setPhaseVoltage(0, 0, 0);  // رѹ
            HAL_Delay(200);

            printf("mid_angle=%.4f\r\n",mid_angle);
            printf("end_angle=%.4f\r\n",end_angle);
            
            moved =  fabs(mid_angle - end_angle);  // ƶĽǶȲ
            if((mid_angle == end_angle)||(moved < 0.02))  // мǶȺͽǶͬ򼸺ûƶ
            {
                printf("MOT: Failed to notice movement------------------------------------- \r\n");				
            }
            else if(mid_angle < end_angle)
            {
                printf("MOT: sensor_direction==CCW\r\n");  // Ϊʱ
                sensor_direction=CCW;
            }
            else
            {
                printf("MOT: sensor_direction==CW\r\n");  // Ϊ˳ʱ
                sensor_direction=CW;
            }
            setPhaseVoltage(1, 0,  _3PI_2);  // ƫƽǶȵѹ
            HAL_Delay(700);
            printf("end_angle--------_3PI_2 =%.4f\r\n", getAngle());

            zero_electric_angle = _normalizeAngle(sensor_direction* getAngle() * 7 );  // 㲢һǶ
            HAL_Delay(20);
            printf("MOT: Zero elec. angle:");
            printf("%.4f\r\n",zero_electric_angle);
            
            setPhaseVoltage(0, 0, 0);  // رѹ
            HAL_Delay(200);
            
            pattern=3;
            break;
        case 3:
            //printf("case 3: %.3f,%.3f\n",shaft_angle,electrical_angle); // ѡĵӡǶ
            voltage.q =0.5;
            voltage.d =0;
          break;
    case 4:
            P_angle.P = 5;
            P_angle.I = 0;
            P_angle.D = 0;		
            voltage.q = PIDoperator(&P_angle, (target_angle - shaft_angle)); 		
//		  printf("%.3f,%.3f,%.3f\n",shaft_angle,shaft_velocity,voltage.q); // ѡĵӡǶ
            break;
    }
    
    // ŤؿƵִӦĿƲ
    setPhaseVoltage(voltage.q, voltage.d, electrical_angle);

    if(feedback_send_flag==1)
    {
        feedback_send_flag=0;
        //printf("C=%d\r\n",MGT_angle);
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


/* USER CODE BEGIN 1 */    //`���ڲ��800hz  *2 ���ٶ�ˢ��
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if(htim->Instance == TIM2){                              //�漰�������ʱ���жϵ�ʱ����ͺ��б�Ҫ��
//			static int count_tim2=0;
        //	if(count_tim2++>=2)
            {
//					count_tim2=0;
        //			HAL_GPIO_TogglePin(LED1_GPIO_Port,LED1_Pin);
                    Send_Request();
//					if(MGT_angle_rx_flag)
//					{
//						HAL_GPIO_TogglePin(LED2_GPIO_Port,LED2_Pin);
//						MGT_angle_rx_flag=0;
//					}
            }
    }
    else
    if(htim->Instance == TIM4)
    {                              //�漰�������ʱ���жϵ�ʱ����ͺ��б�Ҫ��
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
    //		MX_USART2_UART_Init();
            uart2_doutntime_rx=300;
            cmd_state = CMD_STATE_IDLE;
            cmd_idx = 0;
            for(int i=0;i<10;i++)cmd_buffer[i]=0;
            HAL_UART_Receive_IT(&huart2, &rx_byte_uart2, 1);
            //ȡ�����ڻ����ж�����ֽ���
            uint8_t data = (uint8_t)(huart2.Instance->DR);
            //���ORE����
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
                    // ��ƥ�䣬���µȴ���ʼ�ֽ�
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
                // ����У��ֵ
                {
                    uint8_t calculated_checksum = 0x02 ^ 0x08 ^ received_bytes[0] ^ received_bytes[1];
                    if (rx_byte == calculated_checksum)
                    {
                        // У��ͨ���������Ƕ�
                        MGT_angle = -( received_bytes[0] + (received_bytes[1] << 8));
                                        //		shaft_angle = MGT_angle * _2PI / 16383;
                        MGT_angle_rx_flag = 1;
                    }
                    // ����У���Ƿ�ͨ��������Ҫ����״̬��
                    current_state = WAIT_FOR_START;
                }
                break;

            default:
                current_state = WAIT_FOR_START;
                break;
        }

        // ���¿����жϽ�����һ���ֽ�
        HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
    }
        else
        if (huart->Instance == USART2) {
                //A=+562.384;
        char received_char = rx_byte_uart2; // ��ȡ���յ��ַ�
                uart2_doutntime_rx=300;
        // ������������Ը�����Ҫ����
        // printf("Received: %c (0x%02X)\r\n", received_char, received_char);

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
                // 新增J命令，直接进入校准流程
                if (received_char == 'J' || received_char == 'j') {
                    cmd_state = CMD_STATE_RECEIVE_DATA_J;
                    cmd_idx = 0;
                }
                if (received_char == 'C' || received_char == 'c') {
                    printf("%.6f\r\n", getAngle());
                    cmd_state = CMD_STATE_IDLE;
                    cmd_idx = 0;
                }
            case CMD_STATE_RECEIVE_DATA_J:
                // 只要收到J命令就立即执行pattern=2的校准流程
                printf("MOT: Align sensor.\r\n");
                float mid_angle, end_angle, moved, angle;
                motor_enabled = 1;
                for(int i=0; i<=500; i++) {
                    angle = _3PI_2 + _2PI * i / 500.0;
                    setPhaseVoltage(1, 0, angle);
                    Send_Request();
                    HAL_Delay(2);
                }
                Send_Request();
                HAL_Delay(2);
                mid_angle = getAngle();
                for(int i=500; i>=0; i--) {
                    angle = _3PI_2 + _2PI * i / 500.0;
                    setPhaseVoltage(1, 0, angle);
                    Send_Request();
                    HAL_Delay(2);
                }
                Send_Request();
                HAL_Delay(2);
                end_angle = getAngle();
                setPhaseVoltage(0, 0, 0);
                HAL_Delay(200);
                printf("mid_angle=%.4f\r\n", mid_angle);
                printf("end_angle=%.4f\r\n", end_angle);
                moved = fabs(mid_angle - end_angle);
                if((mid_angle == end_angle)||(moved < 0.02)) {
                    printf("MOT: Failed to notice movement------------------------------------- \r\n");
                } else if(mid_angle < end_angle) {
                    printf("MOT: sensor_direction==CCW\r\n");
                    sensor_direction = CCW;
                } else {
                    printf("MOT: sensor_direction==CW\r\n");
                    sensor_direction = CW;
                }
                setPhaseVoltage(1, 0, _3PI_2);
                HAL_Delay(700);
                printf("end_angle--------_3PI_2 =%.4f\r\n", getAngle());
                zero_electric_angle = _normalizeAngle(sensor_direction * getAngle() * 7);
                HAL_Delay(20);
                printf("MOT: Zero elec. angle:");
                printf("%.4f\r\n", zero_electric_angle);
                setPhaseVoltage(0, 0, 0);
                HAL_Delay(200);
                cmd_state = CMD_STATE_IDLE;
                cmd_idx = 0;
                break;

                // �Ƿ���ʼ�ַ���������IDLE״̬
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
                    // �Ƿ��ַ�������״̬��
                    cmd_state = CMD_STATE_IDLE;
                    cmd_idx = 0;
                                        for(int i=0;i<10;i++)cmd_buffer[i]=0;
                                    
                }
                                
                break;

            case CMD_STATE_RECEIVE_DATA_A:
                if (received_char == ';'&&cmd_idx==10) 
                                    {

                    cmd_buffer[cmd_idx++] = received_char;
                    cmd_buffer[cmd_idx] = '\0'; // �ַ�����ֹ��

//                    received_command = CMD_SET_REDUCTION_RATIO;
//                    reduction_ratio = atoi(&cmd_buffer[2]); // ��������
                    // ����A����

                    target_angle = (cmd_buffer[3]-'0')*100 + (cmd_buffer[4]-'0')*10 +(cmd_buffer[5]-'0')*1 + (cmd_buffer[7]-'0')*0.1 + (cmd_buffer[8]-'0')*0.01 +( cmd_buffer[9]-'0')*0.001 ;
                                        target_angle *=reduction_ratio;
                                        if(cmd_buffer[2]=='-')target_angle*=-1;
                                         printf("Set Target Angle: %.3f rad\r\n", target_angle);

                    command_ready = 1;

                    // ����״̬��
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
                                    cmd_buffer[cmd_idx] = '\0'; // Null terminate

                                    // Manually parse "B=1234;" format
                                    if (cmd_idx >= 4 && cmd_buffer[0] == 'B' && cmd_buffer[1] == '=') 
                                    {
                                            uint32_t ratio = 0;
                                            uint8_t digit_count = 0;
                                            
                                            // Start reading digits after 'B='
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
                                                            goto reset_state; // Invalid character, reset
                                                    }
                                            }

                                            if (digit_count > 0) 
                                            {
                                                    // Apply limits
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
                                    // Reset state machine
                                    cmd_state = CMD_STATE_IDLE;
                                    cmd_idx = 0;
                            }
                            else 
                            {
                                    // Check buffer space
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
                                    cmd_buffer[cmd_idx] = '\0'; // ?????
                                    if (cmd_idx > 2) { // ??? Q=123;
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
                                    } else { // ??? Q;
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
                // ��Ԥ��״̬������״̬��
                cmd_state = CMD_STATE_IDLE;
                cmd_idx = 0;
                break;
        }

        // �����������ڽ����ж�
        HAL_UART_Receive_IT(&huart2, &rx_byte_uart2, 1);
        }
}
