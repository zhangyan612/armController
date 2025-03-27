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


extern float setSpeed;          // 速度设定值
extern float setCurrent;         // 电流设定值
extern float setPosition ;       // 位置设定值

extern ADC_HandleTypeDef hadc1;

float shaft_angle,sensor_offset,zero_electric_angle,electrical_angle;
int pole_pairs=1,pattern=0;
char sensor_direction=1;
long angle_data, angle_data_prev;// 当前和前一次的角度数据
float full_rotation_offset;      // 全转角度偏移量，用于扩展角度范围
extern uint8_t isRunning ;             // 运行状态标志

DQVoltage_s voltage;                  //!< DQ坐标系下的电压
Kalman kalman_current_Iq_Filter;
DQCurrent_s current;                  //!< DQ坐标系下的电流
// 编码器信号数
#define ENCODER_RESOLUTION 128

volatile uint8_t motor_enabled = 0;  // 电机使能标志

// 编码器初始化计数值
#define ENCODER_MAX_COUNT 65535
// rotatation count
int32_t rotation_count = 0; // rotatation

// 变量定义
int16_t last_count = 0;      // 上一次的计数值（用于计算增量）
float shaft_angle = 0.0f;    // 当前的轴角度（弧度）
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


float Position_KP		=0.15,Position_KI=0.001,Position_KD=0.32;          /* 位置式PID系数 */
float Incremental_KP=1.1,Incremental_KI=0.09,Incremental_KD=0.25;   /* 增量式PID系数 */


float Position_PID(float reality,float target)
{ 	
    static float Bias,Pwm,Last_Bias,Integral_bias=0;
    
    Bias=target-reality;                            /* 计算偏差 */
    Integral_bias+=Bias;	                        /* 偏差累积 */
    
    if(Integral_bias> 5) Integral_bias = 5;   /* 积分限幅 */
    if(Integral_bias<-5) Integral_bias =-5;
    
    Pwm = (Position_KP*Bias)                        /* 比例环节 */
         +(Position_KI*Integral_bias)               /* 积分环节 */
         +(Position_KD*(Bias-Last_Bias));           /* 微分环节 */
    
    Last_Bias=Bias;                                 /* 保存上次偏差 */
    return Pwm;                                     /* 输出结果 */
}

float Bias_buf;

float Incremental_PID(float reality,float target)
{ 	
	 static float Bias,Pwm,Last_bias=0,Prev_bias=0;
    
	
	 Bias=target-reality;                                   /* 计算偏差 */
    Bias_buf=Bias;
	 Pwm += (Incremental_KP*(Bias-Last_bias))               /* 比例环节 */
           +(Incremental_KI*Bias)                           /* 积分环节 */
           +(Incremental_KD*(Bias-2*Last_bias+Prev_bias));  /* 微分环节 */ 	
   Prev_bias=Last_bias;                                   /* 保存上上次偏差 */
	 Last_bias=Bias;	                                    /* 保存上一次偏差 */
		
		Pwm=_constrain(Pwm,-3,3);
	 return Pwm;                                            /* 输出结果 */
}
float electricalAngle(void)
{
    return _normalizeAngle((shaft_angle + sensor_offset) * pole_pairs - zero_electric_angle);
}

void setPhaseVoltage(float Uq, float Ud, float angle_el)
{
    float Uout;               // 输出电压的归一化幅值
    uint32_t sector;          // 当前所在的电气扇区（1到6）
    float T0, T1, T2;         // 空间矢量调制中的基波占空比
    float Ta, Tb, Tc;         // ABC三个相的占空比
    
    // 限制Q轴电压在预定的电压限制范围内
    if(Uq > voltage_limit) Uq = voltage_limit;
    if(Uq < -voltage_limit) Uq = -voltage_limit;
    // 限制D轴电压在预定的电压限制范围内
    if(Ud > voltage_limit) Ud = voltage_limit;
    if(Ud < -voltage_limit) Ud = -voltage_limit;
    
    // 如果D轴电压不为0，说明同时设置了Ud和Uq
    if(Ud) // 只有在设置了Ud和Uq时
    {
        // 计算归一化后的输出电压幅值，使用近似的平方根函数（误差约4%）
        Uout = _sqrt(Ud * Ud + Uq * Uq) / voltage_power_supply;
        // 计算并归一化电气角度，确保在0到2π之间
        // 这一步仅在使用近似的sin和cos函数时必要
        angle_el = _normalizeAngle(angle_el + atan2(Uq, Ud));
    }
    else
    {
        // 只有Uq可用时，无需使用atan2和sqrt
        Uout = Uq / voltage_power_supply;
        // 计算并归一化电气角度，偏移π/2以确保角度在0到2π之间
        angle_el = _normalizeAngle(angle_el + _PI_2);
    }
    
    // 限制输出电压幅值在0.577以内（相电压最大值的近似值）
    if(Uout > 0.577) Uout = 0.577;
    if(Uout < -0.577) Uout = -0.577;
    
    // 计算当前所在的扇区（1到6）
    sector = (angle_el / _PI_3) + 1;
    // 计算T1和T2，基于当前扇区和电气角度
    T1 = _SQRT3 * _sin(sector * _PI_3 - angle_el) * Uout;
    T2 = _SQRT3 * _sin(angle_el - (sector - 1.0) * _PI_3) * Uout;
    // 计算T0，作为剩余时间
    T0 = 1 - T1 - T2;
    
    // 根据当前扇区设置ABC三个相的占空比
    switch(sector)
    {
        case 1:
            /**
             * 扇区1：
             * A相：T1 + T2 + T0/2
             * B相：T2 + T0/2
             * C相：T0/2
             */
            Ta = T1 + T2 + T0 / 2;
            Tb = T2 + T0 / 2;
            Tc = T0 / 2;
            break;
        case 2:
            /**
             * 扇区2：
             * A相：T1 + T0/2
             * B相：T1 + T2 + T0/2
             * C相：T0/2
             */
            Ta = T1 + T0 / 2;
            Tb = T1 + T2 + T0 / 2;
            Tc = T0 / 2;
            break;
        case 3:
            /**
             * 扇区3：
             * A相：T0/2
             * B相：T1 + T2 + T0/2
             * C相：T2 + T0/2
             */
            Ta = T0 / 2;
            Tb = T1 + T2 + T0 / 2;
            Tc = T2 + T0 / 2;
            break;
        case 4:
            /**
             * 扇区4：
             * A相：T0/2
             * B相：T1 + T0/2
             * C相：T1 + T2 + T0/2
             */
            Ta = T0 / 2;
            Tb = T1 + T0 / 2;
            Tc = T1 + T2 + T0 / 2;
            break;
        case 5:
            /**
             * 扇区5：
             * A相：T2 + T0/2
             * B相：T0/2
             * C相：T1 + T2 + T0/2
             */
            Ta = T2 + T0 / 2;
            Tb = T0 / 2;
            Tc = T1 + T2 + T0 / 2;
            break;
        case 6:
            /**
             * 扇区6：
             * A相：T1 + T2 + T0/2
             * B相：T0/2
             * C相：T1 + T0/2
             */
            Ta = T1 + T2 + T0 / 2;
            Tb = T0 / 2;
            Tc = T1 + T0 / 2;
            break;
        default:
            /**
             * 默认情况（可能的错误状态）：
             * 所有相的占空比设为0，系统不输出任何电压
             */
            Ta = 0;
            Tb = 0;
            Tc = 0;
    }
    
    // 将计算得到的占空比乘以PWM周期，设置到定时器的比较寄存器
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, Ta * PWM_Period); // 设置A相的占空比
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, Tb * PWM_Period); // 设置B相的占空比
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, Tc * PWM_Period); // 设置C相的占空比

//		printf("%.3f,%.3f,%.3f\n",Ta * PWM_Period,Tb * PWM_Period,Tc * PWM_Period); // 可选的调试输出，打印轴角度

}




float getAngle(void)
{
    float d_angle; // 角度变化值

    angle_data =  MGT_angle ; // 读取当前的角度计数值

    // 计算角度变化量，用于检测全转
    d_angle = angle_data - angle_data_prev;

    // 如果角度变化量超过阈值（80%的CPR），判断为发生了溢出（全转）
    //if(fabs(d_angle) > (0.8 * 16383)) 
        //full_rotation_offset += d_angle > 0 ? -_2PI : _2PI; // 根据方向调整全转偏移量

		if(fabs(d_angle) > (0.8 * 16383)) {
        if (d_angle > 0) {
            full_rotation_offset -= _2PI;
            rotation_count--; 
        } else {
            full_rotation_offset += _2PI;
            rotation_count++; 
        }
    }

    // 保存当前角度计数值，为下次计算做准备
    angle_data_prev = angle_data;

    // 计算并返回完整的轴角度，包括全转偏移量和当前计数值对应的角度
    return (full_rotation_offset + (angle_data * 1.0 / 16383) * _2PI);
}

 float current_target_angle = 0.0f;
 float initial_angle = 0.0f;
 char motion_complete = 0;
char test_flag;
 float total_target_angle = 8.0f * (float)_PI; // 8π rad
 float desired_speed = 2.0f * (float)_PI;      // 2π rad/s
#define CONTROL_PERIOD 0.1f


/* 定义命令接收状态 */
typedef enum {
    CMD_STATE_WAIT_START,      // 等待命令起始字符
    CMD_STATE_RECEIVE_COMMAND,  // 接收命令内容
} CMD_State;

/* 定义命令类型 */
typedef enum {
    CMD_NONE,                // 无命令
    CMD_SET_REDUCTION_RATIO, // 设定减速比
    CMD_SET_TARGET_ANGLE     // 设定目标角度
} CommandType;

/* 定义命令缓冲区大小 */
#define CMD_BUFFER_SIZE 64

/* 串口命令解析相关变量 */

volatile char cmd_buffer[CMD_BUFFER_SIZE];
volatile uint8_t cmd_idx = 0;
volatile CommandType received_command = CMD_NONE;
volatile float target_angle = 0.0f;
volatile int reduction_ratio = 1;
volatile uint8_t command_ready = 0; // 命令完成标志

uint8_t tx_buffer[10] = {0x02};     // 发送缓冲区
uint8_t rx_buffer[10];     // 接收缓冲区

typedef enum {
    WAIT_FOR_START,
    WAIT_FOR_IDENT,
    WAIT_FOR_DATA1,
    WAIT_FOR_DATA2,
    WAIT_FOR_CHECKSUM
} ReceiveState;
volatile ReceiveState current_state = WAIT_FOR_START;
volatile uint8_t received_bytes[4]; // 存储标识字节、2个数据字节及校验字节
volatile uint8_t byte_index = 0;
uint8_t rx_byte,rx_byte_uart2; // 单个接收字节
int uart2_doutntime_rx;

// RS485 切换模式
#define RS485_TX_MODE() HAL_GPIO_WritePin(RS485_EN_GPIO_Port, RS485_EN_Pin, GPIO_PIN_SET)   // 发送模式
#define RS485_RX_MODE() HAL_GPIO_WritePin(RS485_EN_GPIO_Port, RS485_EN_Pin, GPIO_PIN_RESET) // 接收模式


void Send_Request(void)
{
    uint8_t tx_buffer = 0x02;
    RS485_TX_MODE(); // 切换到发送模式
    HAL_UART_Transmit(&huart1, &tx_buffer, 1, HAL_MAX_DELAY);
    RS485_RX_MODE(); // 切换回接收模式
}

/* Definitions ---------------------------------------------------------------*/
#define CMD_BUFFER_SIZE_A 12  // A命令最大长度：A=+123.456;
#define CMD_BUFFER_SIZE_B 8   // B命令最大长度：B=0001;

/* Enumerations --------------------------------------------------------------*/
typedef enum {
    CMD_STATE_IDLE,            // 空闲状态，等待命令起始字符
    CMD_STATE_RECEIVE_EQUALS,  // 已接收到A/B，等待=字符
    CMD_STATE_RECEIVE_DATA_A,  // 接收A命令的数据部分
    CMD_STATE_RECEIVE_DATA_B,  // 接收B命令的数据部分
		CMD_STATE_RECEIVE_DATA_C,  // encoder value
    CMD_STATE_RECEIVE_DATA_E,  // 接收E命令的数据部分
		CMD_STATE_RECEIVE_DATA_Q,  // Rotation
    CMD_STATE_RECEIVE_TERMINATOR // 接收命令终止符;
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
    if (!motor_enabled) {
        // Zero out the voltages to prevent any movement
        voltage.q = 0;
        voltage.d = 0;
        setPhaseVoltage(0, 0, 0);
        return;
    }

	switch(pattern)
	{
		case 0:
			voltage.q = 0;
			voltage.d = 0;

			PID_current_q.P =  0.08;
			PID_current_q.I =  0.05;
			PID_current_q.D =  0;
			PID_current_q.limit = 2;
			
			pole_pairs=7;  
			
			zero_electric_angle =2.3;
			pattern=4;
			HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
		    HAL_UART_Receive_IT(&huart2, &rx_byte_uart2, 1);  /* 启动串口2接收中断,接收目标角度和减速比，接收一个字节 */

			break;
		case 2:
			printf("MOT: Align sensor.\r\n");  // 打印传感器校准开始的信息

			float mid_angle,end_angle;
			float moved;

			float angle;
			for(int i=0; i<=500; i++)
			{
				angle = _3PI_2 + _2PI * i / 500.0;  // 计算角度
				setPhaseVoltage(1, 0,  angle);  // 设置相应的相电压
				HAL_Delay(2);
			}
			mid_angle= getAngle();  // 获取中间角度
			
			for(int i=500; i>=0; i--) 
			{
				angle = _3PI_2 + _2PI * i / 500.0;
				setPhaseVoltage(1, 0,  angle);
				HAL_Delay(2);
			}
			end_angle= getAngle();  // 获取结束角度
			setPhaseVoltage(0, 0, 0);  // 关闭相电压
			HAL_Delay(200);

			printf("mid_angle=%.4f\r\n",mid_angle);
			printf("end_angle=%.4f\r\n",end_angle);
			
			moved =  fabs(mid_angle - end_angle);  // 计算移动的角度差
			if((mid_angle == end_angle)||(moved < 0.02))  // 如果中间角度和结束角度相同或几乎没有移动
			{
				printf("MOT: Failed to notice movement------------------------------------- \r\n");				
			}
			else if(mid_angle < end_angle)
			{
				printf("MOT: sensor_direction==CCW\r\n");  // 传感器方向为逆时针
				sensor_direction=CW;
			}
			else
			{
				printf("MOT: sensor_direction==CW\r\n");  // 传感器方向为顺时针
				sensor_direction=CCW;
			}
			setPhaseVoltage(1, 0,  _3PI_2);  // 设置零点偏移角度的相电压
			HAL_Delay(700);
			printf("end_angle--------_3PI_2 =%.4f\r\n", getAngle());

			zero_electric_angle = _normalizeAngle(sensor_direction* getAngle() * 7 );  // 计算并归一化电气角度
			HAL_Delay(20);
			printf("MOT: Zero elec. angle:");
			printf("%.4f\r\n",zero_electric_angle);
			
			setPhaseVoltage(0, 0, 0);  // 关闭相电压
			HAL_Delay(200);
			
			pattern=3;
			break;
		case 3:
			printf("case 3: %.3f,%.3f\n",shaft_angle,electrical_angle); // 可选的调试输出，打印轴角度
			voltage.q =0.5;
			voltage.d =0;
		  break;
    case 4:
			P_angle.P = 5;
			P_angle.I = 0;
			P_angle.D = 0;		
			voltage.q = PIDoperator(&P_angle, (shaft_angle - target_angle)); 		
//		  printf("%.3f,%.3f,%.3f\n",shaft_angle,shaft_velocity,voltage.q); // 可选的调试输出，打印轴角度
			break;
	}
	
	shaft_angle = getAngle();
    electrical_angle = electricalAngle();
    
    // 根据扭矩控制的类型执行相应的控制策略
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


/* USER CODE BEGIN 1 */    //`现在差不多800hz  *2 的速度刷新
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if(htim->Instance == TIM2){                              //涉及到多个定时器中断的时候检查就很有必要了
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
	{                              //涉及到多个定时器中断的时候检查就很有必要了
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
			//取出串口缓存中多余的字节数
			uint8_t data = (uint8_t)(huart2.Instance->DR);
			//清除ORE错误
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
                    // 不匹配，重新等待起始字节
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
                // 计算校验值
                {
                    uint8_t calculated_checksum = 0x02 ^ 0x08 ^ received_bytes[0] ^ received_bytes[1];
                    if (rx_byte == calculated_checksum)
                    {
                        // 校验通过，解析角度
                        MGT_angle = -( received_bytes[0] + (received_bytes[1] << 8));
										//		shaft_angle = MGT_angle * _2PI / 16383;
                        MGT_angle_rx_flag = 1;
                    }
                    // 无论校验是否通过，都需要重置状态机
                    current_state = WAIT_FOR_START;
                }
                break;

            default:
                current_state = WAIT_FOR_START;
                break;
        }

        // 重新开启中断接收下一个字节
        HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
    }
		else
		if (huart->Instance == USART2) {
				//A=+562.384;
        char received_char = rx_byte_uart2; // 获取接收的字符
				uart2_doutntime_rx=300;
        // 调试输出，可以根据需要启用
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
										
                if (received_char == 'C' || received_char == 'c') {
                    printf("%d\r\n", MGT_angle);
                    cmd_state = CMD_STATE_IDLE;
                    cmd_idx = 0;
                }

                // 非法起始字符，保持在IDLE状态
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
                    // 非法字符，重置状态机
                    cmd_state = CMD_STATE_IDLE;
                    cmd_idx = 0;
										for(int i=0;i<10;i++)cmd_buffer[i]=0;
									
                }
								
                break;

            case CMD_STATE_RECEIVE_DATA_A:
                if (received_char == ';'&&cmd_idx==10) 
									{

                    cmd_buffer[cmd_idx++] = received_char;
                    cmd_buffer[cmd_idx] = '\0'; // 字符串终止符

//                    received_command = CMD_SET_REDUCTION_RATIO;
//                    reduction_ratio = atoi(&cmd_buffer[2]); // 解析整数
                    // 解析A命令

                    target_angle = (cmd_buffer[3]-'0')*100 + (cmd_buffer[4]-'0')*10 +(cmd_buffer[5]-'0')*1 + (cmd_buffer[7]-'0')*0.1 + (cmd_buffer[8]-'0')*0.01 +( cmd_buffer[9]-'0')*0.001 ;
										target_angle *=reduction_ratio;
										if(cmd_buffer[2]=='-')target_angle*=-1;
										 printf("Set Target Angle: %.3f rad\r\n", target_angle);

                    command_ready = 1;

                    // 重置状态机
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
                // 非预期状态，重置状态机
                cmd_state = CMD_STATE_IDLE;
                cmd_idx = 0;
                break;
        }

        // 重新启动串口接收中断
        HAL_UART_Receive_IT(&huart2, &rx_byte_uart2, 1);
		}
}
