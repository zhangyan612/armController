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


extern float setSpeed;          // ï¿½Ù¶ï¿½ï¿½è¶¨Öµ
extern float setCurrent;         // ï¿½ï¿½ï¿½ï¿½ï¿½è¶¨Öµ
extern float setPosition ;       // Î»ï¿½ï¿½ï¿½è¶¨Öµ

extern ADC_HandleTypeDef hadc1;

float shaft_angle,sensor_offset,zero_electric_angle,electrical_angle;
int pole_pairs=1,pattern=0;
char sensor_direction=1;
long angle_data, angle_data_prev;// ï¿½ï¿½Ç°ï¿½ï¿½Ç°Ò»ï¿½ÎµÄ½Ç¶ï¿½ï¿½ï¿½ï¿½ï¿½
float full_rotation_offset;      // È«×ªï¿½Ç¶ï¿½Æ«ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Õ¹ï¿½Ç¶È·ï¿½Î§
extern uint8_t isRunning ;             // ï¿½ï¿½ï¿½ï¿½×´Ì¬ï¿½ï¿½Ö¾

DQVoltage_s voltage;                  //!< DQï¿½ï¿½ï¿½ï¿½Ïµï¿½ÂµÄµï¿½Ñ¹
Kalman kalman_current_Iq_Filter;
DQCurrent_s current;                  //!< DQï¿½ï¿½ï¿½ï¿½Ïµï¿½ÂµÄµï¿½ï¿½ï¿½
// ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Åºï¿½ï¿½ï¿½
#define ENCODER_RESOLUTION 128

volatile uint8_t motor_enabled = 0;  // ï¿½ï¿½ï¿½Ê¹ï¿½Ü±ï¿½Ö¾

// ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ê¼ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Öµ
#define ENCODER_MAX_COUNT 65535
// rotatation count
int32_t rotation_count = 0; // rotatation

// ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
int16_t last_count = 0;      // ï¿½ï¿½Ò»ï¿½ÎµÄ¼ï¿½ï¿½ï¿½Öµï¿½ï¿½ï¿½ï¿½ï¿½Ú¼ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
float shaft_angle = 0.0f;    // ï¿½ï¿½Ç°ï¿½ï¿½ï¿½ï¿½Ç¶È£ï¿½ï¿½ï¿½ï¿½È£ï¿½
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
    float Uout;               // ï¿½ï¿½ï¿½ï¿½ï¿½Ñ¹ï¿½Ä¹ï¿½Ò»ï¿½ï¿½ï¿½ï¿½Öµ
    uint32_t sector;          // ï¿½ï¿½Ç°ï¿½ï¿½ï¿½ÚµÄµï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½1ï¿½ï¿½6ï¿½ï¿½
    float T0, T1, T2;         // ï¿½Õ¼ï¿½Ê¸ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ÐµÄ»ï¿½ï¿½ï¿½Õ¼ï¿½Õ±ï¿½
    float Ta, Tb, Tc;         // ABCï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Õ¼ï¿½Õ±ï¿½
    
    // ï¿½ï¿½ï¿½ï¿½Qï¿½ï¿½ï¿½Ñ¹ï¿½ï¿½Ô¤ï¿½ï¿½ï¿½Äµï¿½Ñ¹ï¿½ï¿½ï¿½Æ·ï¿½Î§ï¿½ï¿½
    if(Uq > voltage_limit) Uq = voltage_limit;
    if(Uq < -voltage_limit) Uq = -voltage_limit;
    // ï¿½ï¿½ï¿½ï¿½Dï¿½ï¿½ï¿½Ñ¹ï¿½ï¿½Ô¤ï¿½ï¿½ï¿½Äµï¿½Ñ¹ï¿½ï¿½ï¿½Æ·ï¿½Î§ï¿½ï¿½
    if(Ud > voltage_limit) Ud = voltage_limit;
    if(Ud < -voltage_limit) Ud = -voltage_limit;
    
    // ï¿½ï¿½ï¿½Dï¿½ï¿½ï¿½Ñ¹ï¿½ï¿½Îª0ï¿½ï¿½Ëµï¿½ï¿½Í¬Ê±ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Udï¿½ï¿½Uq
    if(Ud) // Ö»ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Udï¿½ï¿½UqÊ±
    {
        // ï¿½ï¿½ï¿½ï¿½ï¿½Ò»ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ñ¹ï¿½ï¿½Öµï¿½ï¿½Ê¹ï¿½Ã½ï¿½ï¿½Æµï¿½Æ½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ô¼4%ï¿½ï¿½
        Uout = _sqrt(Ud * Ud + Uq * Uq) / voltage_power_supply;
        // ï¿½ï¿½ï¿½ã²¢ï¿½ï¿½Ò»ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ç¶È£ï¿½È·ï¿½ï¿½ï¿½ï¿½0ï¿½ï¿½2ï¿½ï¿½Ö®ï¿½ï¿½
        // ï¿½ï¿½Ò»ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ê¹ï¿½Ã½ï¿½ï¿½Æµï¿½sinï¿½ï¿½cosï¿½ï¿½ï¿½ï¿½Ê±ï¿½ï¿½Òª
        angle_el = _normalizeAngle(angle_el + atan2(Uq, Ud));
    }
    else
    {
        // Ö»ï¿½ï¿½Uqï¿½ï¿½ï¿½ï¿½Ê±ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ê¹ï¿½ï¿½atan2ï¿½ï¿½sqrt
        Uout = Uq / voltage_power_supply;
        // ï¿½ï¿½ï¿½ã²¢ï¿½ï¿½Ò»ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ç¶È£ï¿½Æ«ï¿½Æ¦ï¿½/2ï¿½ï¿½È·ï¿½ï¿½ï¿½Ç¶ï¿½ï¿½ï¿½0ï¿½ï¿½2ï¿½ï¿½Ö®ï¿½ï¿½
        angle_el = _normalizeAngle(angle_el + _PI_2);
    }
    
    // ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ñ¹ï¿½ï¿½Öµï¿½ï¿½0.577ï¿½ï¿½ï¿½Ú£ï¿½ï¿½ï¿½ï¿½Ñ¹ï¿½ï¿½ï¿½Öµï¿½Ä½ï¿½ï¿½ï¿½Öµï¿½ï¿½
    if(Uout > 0.577) Uout = 0.577;
    if(Uout < -0.577) Uout = -0.577;
    
    // ï¿½ï¿½ï¿½ãµ±Ç°ï¿½ï¿½ï¿½Úµï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½1ï¿½ï¿½6ï¿½ï¿½
    sector = (angle_el / _PI_3) + 1;
    // ï¿½ï¿½ï¿½ï¿½T1ï¿½ï¿½T2ï¿½ï¿½ï¿½ï¿½ï¿½Úµï¿½Ç°ï¿½ï¿½ï¿½ï¿½ï¿½Íµï¿½ï¿½ï¿½ï¿½Ç¶ï¿½
    T1 = _SQRT3 * _sin(sector * _PI_3 - angle_el) * Uout;
    T2 = _SQRT3 * _sin(angle_el - (sector - 1.0) * _PI_3) * Uout;
    // ï¿½ï¿½ï¿½ï¿½T0ï¿½ï¿½ï¿½ï¿½ÎªÊ£ï¿½ï¿½Ê±ï¿½ï¿½
    T0 = 1 - T1 - T2;
    
    // ï¿½ï¿½ï¿½Ýµï¿½Ç°ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ABCï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Õ¼ï¿½Õ±ï¿½
    switch(sector)
    {
        case 1:
            /**
             * ï¿½ï¿½ï¿½ï¿½1ï¿½ï¿½
             * Aï¿½à£ºT1 + T2 + T0/2
             * Bï¿½à£ºT2 + T0/2
             * Cï¿½à£ºT0/2
             */
            Ta = T1 + T2 + T0 / 2;
            Tb = T2 + T0 / 2;
            Tc = T0 / 2;
            break;
        case 2:
            /**
             * ï¿½ï¿½ï¿½ï¿½2ï¿½ï¿½
             * Aï¿½à£ºT1 + T0/2
             * Bï¿½à£ºT1 + T2 + T0/2
             * Cï¿½à£ºT0/2
             */
            Ta = T1 + T0 / 2;
            Tb = T1 + T2 + T0 / 2;
            Tc = T0 / 2;
            break;
        case 3:
            /**
             * ï¿½ï¿½ï¿½ï¿½3ï¿½ï¿½
             * Aï¿½à£ºT0/2
             * Bï¿½à£ºT1 + T2 + T0/2
             * Cï¿½à£ºT2 + T0/2
             */
            Ta = T0 / 2;
            Tb = T1 + T2 + T0 / 2;
            Tc = T2 + T0 / 2;
            break;
        case 4:
            /**
             * ï¿½ï¿½ï¿½ï¿½4ï¿½ï¿½
             * Aï¿½à£ºT0/2
             * Bï¿½à£ºT1 + T0/2
             * Cï¿½à£ºT1 + T2 + T0/2
             */
            Ta = T0 / 2;
            Tb = T1 + T0 / 2;
            Tc = T1 + T2 + T0 / 2;
            break;
        case 5:
            /**
             * ï¿½ï¿½ï¿½ï¿½5ï¿½ï¿½
             * Aï¿½à£ºT2 + T0/2
             * Bï¿½à£ºT0/2
             * Cï¿½à£ºT1 + T2 + T0/2
             */
            Ta = T2 + T0 / 2;
            Tb = T0 / 2;
            Tc = T1 + T2 + T0 / 2;
            break;
        case 6:
            /**
             * ï¿½ï¿½ï¿½ï¿½6ï¿½ï¿½
             * Aï¿½à£ºT1 + T2 + T0/2
             * Bï¿½à£ºT0/2
             * Cï¿½à£ºT1 + T0/2
             */
            Ta = T1 + T2 + T0 / 2;
            Tb = T0 / 2;
            Tc = T1 + T0 / 2;
            break;
        default:
            /**
             * Ä¬ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ÜµÄ´ï¿½ï¿½ï¿½×´Ì¬ï¿½ï¿½ï¿½ï¿½
             * ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Õ¼ï¿½Õ±ï¿½ï¿½ï¿½Îª0ï¿½ï¿½ÏµÍ³ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ÎºÎµï¿½Ñ¹
             */
            Ta = 0;
            Tb = 0;
            Tc = 0;
    }
    
    // ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ãµï¿½ï¿½ï¿½Õ¼ï¿½Õ±È³ï¿½ï¿½ï¿½PWMï¿½ï¿½ï¿½Ú£ï¿½ï¿½ï¿½ï¿½Ãµï¿½ï¿½ï¿½Ê±ï¿½ï¿½ï¿½Ä±È½Ï¼Ä´ï¿½ï¿½ï¿½
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, Ta * PWM_Period); // ï¿½ï¿½ï¿½ï¿½Aï¿½ï¿½ï¿½Õ¼ï¿½Õ±ï¿½
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, Tb * PWM_Period); // ï¿½ï¿½ï¿½ï¿½Bï¿½ï¿½ï¿½Õ¼ï¿½Õ±ï¿½
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, Tc * PWM_Period); // ï¿½ï¿½ï¿½ï¿½Cï¿½ï¿½ï¿½Õ¼ï¿½Õ±ï¿½

//		printf("%.3f,%.3f,%.3f\n",Ta * PWM_Period,Tb * PWM_Period,Tc * PWM_Period); // ï¿½ï¿½Ñ¡ï¿½Äµï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ó¡ï¿½ï¿½Ç¶ï¿½

}




float getAngle(void)
{
    float d_angle; // ï¿½Ç¶È±ä»¯Öµ

    angle_data =  MGT_angle ; // ï¿½ï¿½È¡ï¿½ï¿½Ç°ï¿½Ä½Ç¶È¼ï¿½ï¿½ï¿½Öµ

    // ï¿½ï¿½ï¿½ï¿½Ç¶È±ä»¯ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ú¼ï¿½ï¿½È«×ª
    d_angle = angle_data - angle_data_prev;

    // ï¿½ï¿½ï¿½ï¿½Ç¶È±ä»¯ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Öµï¿½ï¿½80%ï¿½ï¿½CPRï¿½ï¿½ï¿½ï¿½ï¿½Ð¶ï¿½Îªï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½È«×ªï¿½ï¿½
    //if(fabs(d_angle) > (0.8 * 16383)) 
        //full_rotation_offset += d_angle > 0 ? -_2PI : _2PI; // ï¿½ï¿½ï¿½Ý·ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½È«×ªÆ«ï¿½ï¿½ï¿½ï¿½

        if(fabs(d_angle) > (0.8 * 16383)) {
        if (d_angle > 0) {
            full_rotation_offset -= _2PI;
            rotation_count--; 
        } else {
            full_rotation_offset += _2PI;
            rotation_count++; 
        }
    }

    // ï¿½ï¿½ï¿½æµ±Ç°ï¿½Ç¶È¼ï¿½ï¿½ï¿½Öµï¿½ï¿½Îªï¿½Â´Î¼ï¿½ï¿½ï¿½ï¿½ï¿½×¼ï¿½ï¿½
    angle_data_prev = angle_data;

    // ï¿½ï¿½ï¿½ã²¢ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ç¶È£ï¿½ï¿½ï¿½ï¿½ï¿½È«×ªÆ«ï¿½ï¿½ï¿½ï¿½ï¿½Íµï¿½Ç°ï¿½ï¿½ï¿½ï¿½Öµï¿½ï¿½Ó¦ï¿½Ä½Ç¶ï¿½
    return (full_rotation_offset + (angle_data * 1.0 / 16383) * _2PI);
}

 float current_target_angle = 0.0f;
 float initial_angle = 0.0f;
 char motion_complete = 0;
char test_flag;
 float total_target_angle = 8.0f * (float)_PI; // 8ï¿½ï¿½ rad
 float desired_speed = 2.0f * (float)_PI;      // 2ï¿½ï¿½ rad/s
#define CONTROL_PERIOD 0.1f


/* ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½×´Ì¬ */
typedef enum {
    CMD_STATE_WAIT_START,      // ï¿½È´ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ê¼ï¿½Ö·ï¿½
    CMD_STATE_RECEIVE_COMMAND,  // ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
} CMD_State;

/* ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ */
typedef enum {
    CMD_NONE,                // ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
    CMD_SET_REDUCTION_RATIO, // ï¿½è¶¨ï¿½ï¿½ï¿½Ù±ï¿½
    CMD_SET_TARGET_ANGLE     // ï¿½è¶¨Ä¿ï¿½ï¿½Ç¶ï¿½
} CommandType;

/* ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½î»ºï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ð¡ */
#define CMD_BUFFER_SIZE 64

/* ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ø±ï¿½ï¿½ï¿½ */

volatile char cmd_buffer[CMD_BUFFER_SIZE];
volatile uint8_t cmd_idx = 0;
volatile CommandType received_command = CMD_NONE;
volatile float target_angle = 0.0f;
volatile int reduction_ratio = 1;
volatile uint8_t command_ready = 0; // ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½É±ï¿½Ö¾

uint8_t tx_buffer[10] = {0x02};     // ï¿½ï¿½ï¿½Í»ï¿½ï¿½ï¿½ï¿½ï¿½
uint8_t rx_buffer[10];     // ï¿½ï¿½ï¿½Õ»ï¿½ï¿½ï¿½ï¿½ï¿½

typedef enum {
    WAIT_FOR_START,
    WAIT_FOR_IDENT,
    WAIT_FOR_DATA1,
    WAIT_FOR_DATA2,
    WAIT_FOR_CHECKSUM
} ReceiveState;
volatile ReceiveState current_state = WAIT_FOR_START;
volatile uint8_t received_bytes[4]; // ï¿½æ´¢ï¿½ï¿½Ê¶ï¿½Ö½Ú¡ï¿½2ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ö½Ú¼ï¿½Ð£ï¿½ï¿½ï¿½Ö½ï¿½
volatile uint8_t byte_index = 0;
uint8_t rx_byte,rx_byte_uart2; // ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ö½ï¿½
int uart2_doutntime_rx;

// RS485 ï¿½Ð»ï¿½Ä£Ê½
#define RS485_TX_MODE() HAL_GPIO_WritePin(RS485_EN_GPIO_Port, RS485_EN_Pin, GPIO_PIN_SET)   // ï¿½ï¿½ï¿½ï¿½Ä£Ê½
#define RS485_RX_MODE() HAL_GPIO_WritePin(RS485_EN_GPIO_Port, RS485_EN_Pin, GPIO_PIN_RESET) // ï¿½ï¿½ï¿½ï¿½Ä£Ê½


void Send_Request(void)
{
    uint8_t tx_buffer = 0x02;
    RS485_TX_MODE(); // ï¿½Ð»ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ä£Ê½
    HAL_UART_Transmit(&huart1, &tx_buffer, 1, HAL_MAX_DELAY);
    RS485_RX_MODE(); // ï¿½Ð»ï¿½ï¿½Ø½ï¿½ï¿½ï¿½Ä£Ê½
}

/* Definitions ---------------------------------------------------------------*/
#define CMD_BUFFER_SIZE_A 12  // Aï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ó³¤¶È£ï¿½A=+123.456;
#define CMD_BUFFER_SIZE_B 8   // Bï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ó³¤¶È£ï¿½B=0001;

/* Enumerations --------------------------------------------------------------*/
typedef enum {
    CMD_STATE_IDLE,            // ï¿½ï¿½ï¿½ï¿½×´Ì¬ï¿½ï¿½ï¿½È´ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ê¼ï¿½Ö·ï¿½
    CMD_STATE_RECEIVE_EQUALS,  // ï¿½Ñ½ï¿½ï¿½Õµï¿½A/Bï¿½ï¿½ï¿½È´ï¿½=ï¿½Ö·ï¿½
    CMD_STATE_RECEIVE_DATA_A,  // ï¿½ï¿½ï¿½ï¿½Aï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ý²ï¿½ï¿½ï¿½
    CMD_STATE_RECEIVE_DATA_B,  // ï¿½ï¿½ï¿½ï¿½Bï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ý²ï¿½ï¿½ï¿½
        CMD_STATE_RECEIVE_DATA_C,  // encoder value
    CMD_STATE_RECEIVE_DATA_E,  // ï¿½ï¿½ï¿½ï¿½Eï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ý²ï¿½ï¿½ï¿½
        CMD_STATE_RECEIVE_DATA_Q,  // Rotation
    CMD_STATE_RECEIVE_TERMINATOR // ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ö¹ï¿½ï¿½;
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
        
        pole_pairs=7;  
        
        zero_electric_angle =2.5;
        pattern=4;
        HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
        HAL_UART_Receive_IT(&huart2, &rx_byte_uart2, 1);  /* 2Ð¶,Ä¿Ç¶ÈºÍ¼Ù±È£Ò»Ö½ */
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
            printf("MOT: Align sensor.\r\n");  // Ó¡Ð£×¼Ê¼Ï¢

            float mid_angle,end_angle;
            float moved;

            float angle;
            motor_enabled = 1; // ????
            for(int i=0; i<=500; i++)
            {
                angle = _3PI_2 + _2PI * i / 500.0;
                setPhaseVoltage(1, 0,  angle);
                Send_Request();
                HAL_Delay(2);
                printf("i=%d, angle=%.3f, MGT_angle=%d, getAngle=%.3f\n", i, angle, MGT_angle, getAngle());
            }
            // ?????????????,??mid_angle???MGT_angle???
            Send_Request();
            HAL_Delay(2);
            mid_angle= getAngle();

            for(int i=500; i>=0; i--)
            {
                angle = _3PI_2 + _2PI * i / 500.0;
                setPhaseVoltage(1, 0,  angle);
                Send_Request();
                HAL_Delay(2);
            }
            // ?????????????,??end_angle???MGT_angle???
            Send_Request();
            HAL_Delay(2);
            end_angle= getAngle();
            setPhaseVoltage(0, 0, 0);  // Ø±Ñ¹
            HAL_Delay(200);

            printf("mid_angle=%.4f\r\n",mid_angle);
            printf("end_angle=%.4f\r\n",end_angle);
            
            moved =  fabs(mid_angle - end_angle);  // Æ¶Ä½Ç¶È²
            if((mid_angle == end_angle)||(moved < 0.02))  // Ð¼Ç¶ÈºÍ½Ç¶Í¬ò¼¸ºÃ»Æ¶
            {
                printf("MOT: Failed to notice movement------------------------------------- \r\n");				
            }
            else if(mid_angle < end_angle)
            {
                printf("MOT: sensor_direction==CCW\r\n");  // ÎªÊ±
                sensor_direction=CCW;
            }
            else
            {
                printf("MOT: sensor_direction==CW\r\n");  // ÎªË³Ê±
                sensor_direction=CW;
            }
            setPhaseVoltage(1, 0,  _3PI_2);  // Æ«Æ½Ç¶ÈµÑ¹
            HAL_Delay(700);
            printf("end_angle--------_3PI_2 =%.4f\r\n", getAngle());

            zero_electric_angle = _normalizeAngle(sensor_direction* getAngle() * 7 );  // ã²¢Ò»Ç¶
            HAL_Delay(20);
            printf("MOT: Zero elec. angle:");
            printf("%.4f\r\n",zero_electric_angle);
            
            setPhaseVoltage(0, 0, 0);  // Ø±Ñ¹
            HAL_Delay(200);
            
            pattern=3;
            break;
        case 3:
            //printf("case 3: %.3f,%.3f\n",shaft_angle,electrical_angle); // Ñ¡ÄµÓ¡Ç¶
            voltage.q =0.5;
            voltage.d =0;
          break;
    case 4:
            P_angle.P = 5;
            P_angle.I = 0;
            P_angle.D = 0;		
            voltage.q = PIDoperator(&P_angle, (shaft_angle - target_angle)); 		
//		  printf("%.3f,%.3f,%.3f\n",shaft_angle,shaft_velocity,voltage.q); // Ñ¡ÄµÓ¡Ç¶
            break;
    }
    
    // Å¤Ø¿ÆµÖ´Ó¦Ä¿Æ²
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


/* USER CODE BEGIN 1 */    //`ï¿½ï¿½ï¿½Ú²î²»ï¿½ï¿½800hz  *2 ï¿½ï¿½ï¿½Ù¶ï¿½Ë¢ï¿½ï¿½
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if(htim->Instance == TIM2){                              //ï¿½æ¼°ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ê±ï¿½ï¿½ï¿½Ð¶Ïµï¿½Ê±ï¿½ï¿½ï¿½ï¿½Íºï¿½ï¿½Ð±ï¿½Òªï¿½ï¿½
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
    {                              //ï¿½æ¼°ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ê±ï¿½ï¿½ï¿½Ð¶Ïµï¿½Ê±ï¿½ï¿½ï¿½ï¿½Íºï¿½ï¿½Ð±ï¿½Òªï¿½ï¿½
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
            //È¡ï¿½ï¿½ï¿½ï¿½ï¿½Ú»ï¿½ï¿½ï¿½ï¿½Ð¶ï¿½ï¿½ï¿½ï¿½ï¿½Ö½ï¿½ï¿½ï¿½
            uint8_t data = (uint8_t)(huart2.Instance->DR);
            //ï¿½ï¿½ï¿½OREï¿½ï¿½ï¿½ï¿½
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
                    // ï¿½ï¿½Æ¥ï¿½ä£¬ï¿½ï¿½ï¿½ÂµÈ´ï¿½ï¿½ï¿½Ê¼ï¿½Ö½ï¿½
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
                // ï¿½ï¿½ï¿½ï¿½Ð£ï¿½ï¿½Öµ
                {
                    uint8_t calculated_checksum = 0x02 ^ 0x08 ^ received_bytes[0] ^ received_bytes[1];
                    if (rx_byte == calculated_checksum)
                    {
                        // Ð£ï¿½ï¿½Í¨ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ç¶ï¿½
                        MGT_angle = -( received_bytes[0] + (received_bytes[1] << 8));
                                        //		shaft_angle = MGT_angle * _2PI / 16383;
                        MGT_angle_rx_flag = 1;
                    }
                    // ï¿½ï¿½ï¿½ï¿½Ð£ï¿½ï¿½ï¿½Ç·ï¿½Í¨ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Òªï¿½ï¿½ï¿½ï¿½×´Ì¬ï¿½ï¿½
                    current_state = WAIT_FOR_START;
                }
                break;

            default:
                current_state = WAIT_FOR_START;
                break;
        }

        // ï¿½ï¿½ï¿½Â¿ï¿½ï¿½ï¿½ï¿½Ð¶Ï½ï¿½ï¿½ï¿½ï¿½ï¿½Ò»ï¿½ï¿½ï¿½Ö½ï¿½
        HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
    }
        else
        if (huart->Instance == USART2) {
                //A=+562.384;
        char received_char = rx_byte_uart2; // ï¿½ï¿½È¡ï¿½ï¿½ï¿½Õµï¿½ï¿½Ö·ï¿½
                uart2_doutntime_rx=300;
        // ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ô¸ï¿½ï¿½ï¿½ï¿½ï¿½Òªï¿½ï¿½ï¿½ï¿½
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
                    // ??foc-16bit,????????????????
                    printf("Angle: %.6f rad, Raw: %d, Rotation: %ld\r\n", getAngle(), MGT_angle, rotation_count);
                    cmd_state = CMD_STATE_IDLE;
                    cmd_idx = 0;
                }

                if (received_char == 'J' || received_char == 'j') {
                    // ????pattern=2????
                    pattern = 2;
                    printf("Remote calibration triggered: pattern=2\r\n");
                    cmd_state = CMD_STATE_IDLE;
                    cmd_idx = 0;
                }

                // ??IDLE??
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
                    // ï¿½Ç·ï¿½ï¿½Ö·ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½×´Ì¬ï¿½ï¿½
                    cmd_state = CMD_STATE_IDLE;
                    cmd_idx = 0;
                                        for(int i=0;i<10;i++)cmd_buffer[i]=0;
                                    
                }
                                
                break;

            case CMD_STATE_RECEIVE_DATA_A:
                if (received_char == ';'&&cmd_idx==10) 
                                    {

                    cmd_buffer[cmd_idx++] = received_char;
                    cmd_buffer[cmd_idx] = '\0'; // ï¿½Ö·ï¿½ï¿½ï¿½ï¿½ï¿½Ö¹ï¿½ï¿½

//                    received_command = CMD_SET_REDUCTION_RATIO;
//                    reduction_ratio = atoi(&cmd_buffer[2]); // ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
                    // ï¿½ï¿½ï¿½ï¿½Aï¿½ï¿½ï¿½ï¿½

                    target_angle = (cmd_buffer[3]-'0')*100 + (cmd_buffer[4]-'0')*10 +(cmd_buffer[5]-'0')*1 + (cmd_buffer[7]-'0')*0.1 + (cmd_buffer[8]-'0')*0.01 +( cmd_buffer[9]-'0')*0.001 ;
                                        target_angle *=reduction_ratio;
                                        if(cmd_buffer[2]=='-')target_angle*=-1;
                                         printf("Set Target Angle: %.3f rad\r\n", target_angle);

                    command_ready = 1;

                    // ï¿½ï¿½ï¿½ï¿½×´Ì¬ï¿½ï¿½
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
                // ï¿½ï¿½Ô¤ï¿½ï¿½×´Ì¬ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½×´Ì¬ï¿½ï¿½
                cmd_state = CMD_STATE_IDLE;
                cmd_idx = 0;
                break;
        }

        // ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ú½ï¿½ï¿½ï¿½ï¿½Ð¶ï¿½
        HAL_UART_Receive_IT(&huart2, &rx_byte_uart2, 1);
        }
}
