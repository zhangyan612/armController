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
signed char sensor_direction=CCW; // ?? signed char ???????
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



// Add these definitions after existing mode definitions
#define OPEN_LOOP_SPEED_MODE 5
#define TRAJECTORY_MODE 6

// Add these variables after existing global variables
float setSpeedOpenLoop = 0.0f;        // Open-loop target speed
float openLoopAngle = 0.0f;           // Open-loop electrical angle
uint8_t openLoopInitialized = 0;      // Open-loop initialization flag

// Trajectory mode structure
typedef struct {
    float target_speed;       // Target speed (rad/s)
    float acceleration;       // Acceleration (rad/s²)
    float target_revolutions;// Target revolutions
    float start_angle;        // Start position (rad)
    float target_angle;       // Absolute target position (rad)
    float current_speed;      // Current speed (rad/s)
    float distance_covered;   // Distance moved (rad)
    float total_distance;     // Total distance to move (rad)
    uint8_t active;           // Whether trajectory is active
    int8_t direction;         // Movement direction
    uint8_t phase;            // Current phase
    uint8_t stage;            // Control stage
    float open_loop_angle;    // Open-loop angle for velocity stage
} TrajectoryProfile;

TrajectoryProfile trajectory = {0}; // Trajectory control instance

// Add these function prototypes
void initOpenLoopMode(float speed);
void initTrajectoryMode(float speed, float accel, float revs);
void updateTrajectoryMode();




typedef struct {
    float resistance;
    float inductance;
    float kv_rating;
    float volt_limit;
    float current_limit;
} MotorParams;

MotorParams motor_params = {0};


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
    // Apply sensor direction and pole pairs correctly
    float angle = (sensor_direction * shaft_angle * pole_pairs) - zero_electric_angle;
    return _normalizeAngle(angle);
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
    float d_angle = (float)(MGT_angle - angle_data_prev);

    // Handle wrap-around with correct direction detection
    if(fabs(d_angle) > (0.8 * 65535.0f)) 
    {
        if (d_angle > 0) {
            // CCW rotation (0 -> 65535)
            full_rotation_offset -= _2PI;
            rotation_count--;
        } else {
            // CW rotation (65535 -> 0)
            full_rotation_offset += _2PI;
            rotation_count++;
        }
    }

    angle_data_prev = MGT_angle;
    float current_angle = (MGT_angle / 65535.0f) * _2PI;
    return full_rotation_offset + current_angle;
}


// Measure phase resistance
void measure_phase_resistance(MotorParams *params) {
    printf("Measuring phase resistance...\n");
    
    // Disable motor temporarily
    uint8_t prev_enabled = motor_enabled;
    motor_enabled = 0;
    
    float test_voltage = 1.0f; // Low voltage for safety
    float sum_resistance = 0;
    int measurements = 0;
    
    // Test multiple phases and angles
    for(int phase_test = 0; phase_test < 3; phase_test++) {
        for(int angle_test = 0; angle_test < 4; angle_test++) {
            float angle = angle_test * _PI_2;
            
            // Apply test voltage
            setPhaseVoltage(test_voltage, 0, angle);
            HAL_Delay(50); // Wait for current to stabilize
            
            // Measure current
            PhaseCurrent_s currents = getPhaseCurrents();
            
            // Calculate resistance (R = V/I)
            if(fabs(currents.a) > 0.1f) { // Avoid division by zero
                float resistance = test_voltage / fabs(currents.a);
                sum_resistance += resistance;
                measurements++;
                printf("V=%.2fV, I=%.3fA, R=%.3fO\n", 
                       test_voltage, currents.a, resistance);
            }
            
            HAL_Delay(50);
        }
    }
    
    // Calculate average resistance
    if(measurements > 0) {
        params->resistance = sum_resistance / measurements;
        printf("Average phase resistance: %.4f ohms\n", params->resistance);
    } else {
        params->resistance = 1.0f; // Default value
        printf("Could not measure resistance, using default: %.2f ohms\n", params->resistance);
    }
    
    // Restore motor state
    motor_enabled = prev_enabled;
    setPhaseVoltage(0, 0, 0); // Turn off voltage
}

// Measure phase inductance (simplified method)
void measure_phase_inductance(MotorParams *params) {
    printf("Measuring phase inductance (simplified method)...\n");
    
    // This is a simplified method - for accurate measurement, 
    // you need specialized equipment or more complex techniques
    
    // Apply a voltage step and measure current rise time
    float test_voltage = 2.0f;
    float current_threshold = 0.5f;
    uint32_t start_time, end_time;
    
    // Disable motor temporarily
    uint8_t prev_enabled = motor_enabled;
    motor_enabled = 0;
    
    // Reset current measurement
    getPhaseCurrents(); // Dummy read to clear any stale values
    
    // Apply voltage
    setPhaseVoltage(test_voltage, 0, 0);
    start_time = HAL_GetTick();
    
    // Wait for current to reach threshold
    while(1) {
        PhaseCurrent_s currents = getPhaseCurrents();
        if(fabs(currents.a) >= current_threshold) {
            end_time = HAL_GetTick();
            break;
        }
        if(HAL_GetTick() - start_time > 1000) { // Timeout after 1 second
            printf("Inductance measurement timeout\n");
            end_time = HAL_GetTick();
            break;
        }
        HAL_Delay(1);
    }
    
    // Calculate inductance (L = V * dt/di)
    float dt = (end_time - start_time) / 1000.0f; // Convert to seconds
    float di = current_threshold;
    params->inductance = (test_voltage * dt) / di;
    
    printf("Current rise time: %.3fs, Inductance: %.6fH\n", dt, params->inductance);
    
    // Restore motor state
    motor_enabled = prev_enabled;
    setPhaseVoltage(0, 0, 0); // Turn off voltage
}

// Characterize motor parameters
void characterize_motor(MotorParams *params) {
    printf("Characterizing motor parameters...\n");
    
    // Measure KV rating by spinning motor at known voltage and measuring speed
    float test_voltage = 3.0f;
    int measurements = 0;
    float sum_rpm = 0;
    
    // Enable motor
    uint8_t prev_enabled = motor_enabled;
    motor_enabled = 1;
    
    // Apply voltage
    setPhaseVoltage(test_voltage, 0, 0);
    
    // Wait for speed to stabilize
    HAL_Delay(1000);
    
    // Take multiple speed measurements
    for(int i = 0; i < 10; i++) {
        // Get velocity in RPM (rad/s to RPM: RPM = rad/s * 60 / (2*PI))
        float rpm = shaft_velocity * 60.0f / _2PI;
        sum_rpm += rpm;
        measurements++;
        
        HAL_Delay(100);
    }
    
    // Calculate KV rating (RPM per volt)
    if(measurements > 0 && test_voltage > 0) {
        float avg_rpm = sum_rpm / measurements;
        params->kv_rating = avg_rpm / test_voltage;
        printf("At %.1fV, RPM=%.1f, KV=%.1f RPM/V\n", 
               test_voltage, avg_rpm, params->kv_rating);
    } else {
        params->kv_rating = 100.0f; // Default value
        printf("Could not measure KV, using default: %.1f RPM/V\n", params->kv_rating);
    }
    
    // Restore motor state
    motor_enabled = prev_enabled;
    setPhaseVoltage(0, 0, 0); // Turn off voltage
}

// Find encoder offset with correct electrical angles
void find_encoder_offset() {
    printf("Finding encoder offset...\n");
    
    // Define electrical angles
    float electrical_angles[] = {0, _PI_2, _PI, _3PI_2};
    float encoder_readings[4];
    
    // Enable motor
    uint8_t prev_enabled = motor_enabled;
    motor_enabled = 1;
    
    for(int i = 0; i < 4; i++) {
        // Apply voltage to hold position at specific electrical angle
        setPhaseVoltage(2.0f, 0, electrical_angles[i]);
        HAL_Delay(500); // Wait for settling
        
        // Take multiple encoder readings
        float sum = 0;
        int readings = 0;
        for(int j = 0; j < 20; j++) {
            float angle = getAngle();
            sum += angle;
            readings++;
            HAL_Delay(10);
        }
        encoder_readings[i] = sum / readings;
        
        printf("Electrical: %.2f, Mechanical: %.2f\n", 
               electrical_angles[i], encoder_readings[i]);
    }
    
    // Calculate offset - this depends on your specific setup
    // For a simple approach, we can find the average difference
    float sum_offset = 0;
    for(int i = 0; i < 4; i++) {
        float expected_mechanical = electrical_angles[i] / pole_pairs;
        float offset = encoder_readings[i] - expected_mechanical;
        sum_offset += offset;
    }
    
    sensor_offset = sum_offset / 4;
    printf("Calculated sensor offset: %.4f rad\n", sensor_offset);
    
    // Restore motor state
    motor_enabled = prev_enabled;
    setPhaseVoltage(0, 0, 0); // Turn off voltage
}


// Update PID parameters based on motor characteristics
void update_pid_from_motor_params(MotorParams *params) {
    // Adjust PID parameters based on motor characteristics
    // These are empirical formulas - you may need to adjust them

    // Position PID - based on system responsiveness
    Position_KP = 0.5f / params->resistance;
    Position_KI = Position_KP * 0.1f;
    Position_KD = Position_KP * 0.05f;
    
    // Current PID - based on electrical characteristics
    PID_current_q.P = 0.1f * params->resistance;
    PID_current_q.I = 0.5f * params->resistance / params->inductance;
    PID_current_q.D = 0.01f * params->inductance;
    
    printf("Updated PID parameters:\n");
    printf("Position: KP=%.3f, KI=%.3f, KD=%.3f\n", 
           Position_KP, Position_KI, Position_KD);
    printf("Current: KP=%.3f, KI=%.3f, KD=%.3f\n", 
           PID_current_q.P, PID_current_q.I, PID_current_q.D);
}


// Improved alignment function
int align_motor(MotorParams *params) {
    printf("Starting motor alignment procedure...\n");
    
    // Step 1: Measure phase resistance
    printf("Measuring phase resistance...\n");
    measure_phase_resistance(params);
    
    // Step 2: Measure phase inductance
    printf("Measuring phase inductance...\n");
    measure_phase_inductance(params);
    
    // Step 3: Find encoder offset
    printf("Finding encoder offset...\n");
    find_encoder_offset();
    
    // Step 4: Determine motor parameters
    printf("Determining motor parameters...\n");
    characterize_motor(params);
    
    printf("Alignment complete!\n");
    return 1;
}


// Add this to your initialization
void init_motor_control() {
    // Initialize with safe parameters
    motor_params.resistance = 1.0f;     // Will be measured
    motor_params.inductance = 0.001f;   // Will be measured
    motor_params.kv_rating = 100.0f;    // Will be characterized
    motor_params.volt_limit = 24.0f;
    motor_params.current_limit = 2.0f;
    
    // Set initial PID values based on motor type
    Position_KP = 0.5f;
    Position_KI = 0.05f;
    Position_KD = 0.1f;
    
    // Run alignment procedure
    align_motor(&motor_params);
    
    // Update PID based on characterized motor
    update_pid_from_motor_params(&motor_params);
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



// Initialize open-loop speed mode
void initOpenLoopMode(float speed) {
    setSpeedOpenLoop = speed;
    openLoopAngle = electrical_angle;
    openLoopInitialized = 1;
    printf("Open loop speed mode enabled. Speed: %.2f rad/s\r\n", speed);
}


void initTrajectoryMode(float speed, float accel, float target_angle_val) {
    // Calculate the signed difference to determine direction
    float angle_diff = target_angle_val - shaft_angle;
    
    // Set signed speed based on the angle difference
    if (angle_diff > 0) {
        // Clockwise movement: negative speed
        trajectory.target_speed = -fabs(speed);
    } else if (angle_diff < 0) {
        // Counterclockwise movement: positive speed
        trajectory.target_speed = fabs(speed);
    } else {
        // No movement if already at target
        trajectory.target_speed = 0;
    }
    
    // Calculate absolute distance to target
    trajectory.total_distance = fabs(angle_diff);
    trajectory.acceleration = fabs(accel);
    trajectory.target_angle = target_angle_val;
    trajectory.start_angle = shaft_angle;
    
    // Determine direction based on the sign of target_speed
    trajectory.direction = (trajectory.target_speed >= 0) ? 1 : -1;
    
    trajectory.distance_covered = 0.0f;
    trajectory.current_speed = 0.0f;
    trajectory.phase = 0;
    trajectory.active = 1;
    trajectory.stage = 0;
    trajectory.open_loop_angle = electrical_angle;

    printf("Trajectory mode started:\r\n");
    printf("  Speed: %.2f rad/s\r\n", trajectory.target_speed);
    printf("  Accel: %.2f rad/s²\r\n", trajectory.acceleration);
    printf("  Start angle: %.2f rad\r\n", trajectory.start_angle);
    printf("  Target angle: %.2f rad\r\n", trajectory.target_angle);
    printf("  Distance to move: %.2f rad\r\n", trajectory.total_distance);
    printf("  Direction: %s\r\n", trajectory.direction > 0 ? "CCW" : "CW");
}

// Update trajectory mode

void updateTrajectoryMode() {
    if (!trajectory.active) return;
    
    const float dt = 0.001f;
    float remaining = trajectory.total_distance - trajectory.distance_covered;
    
    // Use absolute values for acceleration calculations
    float abs_target_speed = fabs(trajectory.target_speed);
    float abs_current_speed = fabs(trajectory.current_speed);
    float decel_distance = (abs_current_speed * abs_current_speed) / (2.0f * trajectory.acceleration);
    
		printf("  Remaining: %.2f rad/s\r\n", remaining);
    // Check if we've reached the target position
    if (remaining <= 0.001f) { // Small threshold to account for floating point errors
        // Hold position at the target
        trajectory.current_speed = 0.0f;
        voltage.q = 0;
        voltage.d = 0;
        
        // Optional: Add a small holding torque if needed
        voltage.q = Position_PID(shaft_angle, trajectory.target_angle);
        
        //printf("Target position reached. Holding position.\r\n");
        return;
    }
    
    if (trajectory.phase != 2 && decel_distance >= remaining) {
        trajectory.phase = 2;
        printf("Entering deceleration phase\r\n");
    }
    
    switch (trajectory.phase) {
        case 0: // Acceleration
            abs_current_speed += trajectory.acceleration * dt;
            if (abs_current_speed >= abs_target_speed) {
                abs_current_speed = abs_target_speed;
                trajectory.phase = 1;
                printf("Entering constant speed phase\r\n");
            }
            break;
        case 1: // Constant speed
            abs_current_speed = abs_target_speed;
            break;
        case 2: // Deceleration
            abs_current_speed -= trajectory.acceleration * dt;
            if (abs_current_speed <= 0) {
                abs_current_speed = 0.0f;
                //printf("Reached target position\r\n");
            }
            break;
    }
    
    // Apply the direction to get signed speed
    trajectory.current_speed = (trajectory.target_speed >= 0) ? abs_current_speed : -abs_current_speed;
    trajectory.distance_covered += abs_current_speed * dt;
    
    // Update the open-loop angle with the correct signed speed
    trajectory.open_loop_angle += trajectory.current_speed * pole_pairs * dt;
    //trajectory.open_loop_angle = _normalizeAngle(trajectory.open_loop_angle);
    electrical_angle = trajectory.open_loop_angle;
    
    // Set voltage with correct sign for direction
    voltage.q = (trajectory.current_speed >= 0) ? 3.0f : -3.0f;
    voltage.d = 0;
}




/* ??????? */
typedef enum {
    CMD_STATE_IDLE,            // ????
    CMD_STATE_RECEIVE_EQUALS,  // ??'='
    CMD_STATE_RECEIVE_DATA_A,  // ??A????
    CMD_STATE_RECEIVE_DATA_B,  // ??B????
		CMD_STATE_RECEIVE_DATA_C,  // Receiving encoder value
    CMD_STATE_RECEIVE_DATA_E,  // ??E????
    CMD_STATE_RECEIVE_DATA_Q,  // ??Q????
		CMD_STATE_RECEIVE_DATA_T, 
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
static float test_voltage = 0.5f;
static int debug_count = 0;


MotorParams motor_params;
uint8_t alignment_state = 0;
uint32_t alignment_timer = 0;
uint8_t alignment_in_progress = 0;


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
        PID_current_q.limit = 3;
        
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
		//printf("Electrical angle: %.2f rad (Zero: %.2f)\n", 
             //electrical_angle, zero_electric_angle);

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
					//printf("MOT: Starting improved alignment\n");
					
//					// Step 1: Move to 270° electrical position
//					for(int i=0; i<=500; i++) {
//							float angle = _3PI_2;  // Fixed 270° electrical
//							setPhaseVoltage(3.0, 0, angle);  // Higher voltage (3V)
//							HAL_Delay(2);
//					}
//					HAL_Delay(500);  // Settling time
//					
//					// Step 2: Read mechanical position
//					float aligned_mech = getAngle();
//					printf("Aligned mechanical: %.4f rad\n", aligned_mech);
//					
//					// Step 3: Calculate zero electrical angle (corrected)
//					zero_electric_angle = _normalizeAngle(sensor_direction * aligned_mech * pole_pairs - _3PI_2);
//					printf("Calculated zero electrical: %.4f rad\n", zero_electric_angle);
//					
//					// Step 4: Verify alignment
//					setPhaseVoltage(1.5, 0, _3PI_2);  // Reduced holding voltage
//					HAL_Delay(500);
//					printf("Verification angle: %.4f rad\n", getAngle());
//					pattern = 3;  // Go to voltage control for testing
	
					
					    if (!alignment_in_progress) {
								printf("Starting comprehensive motor alignment procedure...\n");
								alignment_in_progress = 1;
								alignment_state = 0;
								alignment_timer = HAL_GetTick();
								
								// Initialize motor parameters with safe defaults
								motor_params.resistance = 1.0f;
								motor_params.inductance = 0.001f;
								motor_params.kv_rating = 100.0f;
								motor_params.volt_limit = 24.0f;
								motor_params.current_limit = 2.0f;
								
								printf("Alignment step 0: Initializing...\n");
						}
						
			// Alignment state machine
			switch(alignment_state) {
					case 0: // Initialization complete, move to resistance measurement
							if (HAL_GetTick() - alignment_timer > 1000) {
									alignment_state = 1;
									alignment_timer = HAL_GetTick();
									printf("Alignment step 1: Measuring phase resistance...\n");
							}
							break;
							
					case 1: // Measure phase resistance
							measure_phase_resistance(&motor_params);
							alignment_state = 2;
							alignment_timer = HAL_GetTick();
							printf("Alignment step 2: Measuring phase inductance...\n");
							break;
							
					case 2: // Measure phase inductance
							measure_phase_inductance(&motor_params);
							alignment_state = 3;
							alignment_timer = HAL_GetTick();
							printf("Alignment step 3: Finding encoder offset...\n");
							break;
							
					case 3: // Find encoder offset
							find_encoder_offset();
							alignment_state = 4;
							alignment_timer = HAL_GetTick();
							printf("Alignment step 4: Characterizing motor...\n");
							break;
							
					case 4: // Characterize motor
							characterize_motor(&motor_params);
							alignment_state = 5;
							alignment_timer = HAL_GetTick();
							printf("Alignment step 5: Updating PID parameters...\n");
							break;
							
					case 5: // Update PID parameters
							update_pid_from_motor_params(&motor_params);
							alignment_state = 6;
							alignment_timer = HAL_GetTick();
							printf("Alignment step 6: Finalizing...\n");
							break;
							
					case 6: // Finalization
						
						    printf("MOT: Starting improved alignment\n");
    
								// ?????? - ?????????????????
								float alignment_voltage = 2.0f; // ???????????????
								
								// ????????????
								for(int i = 0; i < 4; i++) {
										float elec_angle = i * _PI_2; // 0, 90, 180, 270????
										
										// ??????????????
										setPhaseVoltage(alignment_voltage, 0, elec_angle);
										HAL_Delay(500); // ????
										
										// ??????
										float mech_angle = getAngle();
										printf("Electrical: %.2f, Mechanical: %.2f\n", elec_angle, mech_angle);
										
										// ??????? (???? * ???)
										float expected_elec_angle = _normalizeAngle(mech_angle * pole_pairs);
										
										// ????????
										float zero_offset = elec_angle - expected_elec_angle;
										printf("Zero offset candidate: %.2f\n", zero_offset);
								}
								
								// ??????????????
								zero_electric_angle = _normalizeAngle(3 * _PI_2 - getAngle() * pole_pairs);
								printf("Calculated zero electrical angle: %.4f rad\n", zero_electric_angle);

								// ????
								setPhaseVoltage(0, 0, 0);

							if (HAL_GetTick() - alignment_timer > 500) {
									printf("Motor alignment complete!\n");
									printf("Resistance: %.4f ohms\n", motor_params.resistance);
									printf("Inductance: %.6f H\n", motor_params.inductance);
									printf("KV Rating: %.1f RPM/V\n", motor_params.kv_rating);
									
									alignment_in_progress = 0;
									pattern = 4; // Move to next pattern
							}
							break;
			}
			
			// During alignment, apply a small voltage to keep motor positioned
			setPhaseVoltage(0.5f, 0, electrical_angle);

					break;

        case 3:
				{
						static float test_angle = _3PI_2;
						static uint32_t last_change = 0;
						
						// Change angle every 2 seconds
						if(HAL_GetTick() - last_change > 2000) {
								test_angle += _PI_2;  // 90° steps
								last_change = HAL_GetTick();
								printf("New test angle: %.2f rad\n", test_angle);
						}
						
						voltage.q = 2.0;  // Fixed voltage
						voltage.d = 0;
						
						// Use test_angle instead of electrical_angle
						setPhaseVoltage(voltage.q, voltage.d, test_angle);
						break;
				}
				case 4:
				{
						// Temporary debugging output
						//if(debug_count++ > 100) {
								//debug_count = 0;
								//printf("ANG: Target=%.2f, Shaft=%.2f, Elec=%.2f, Zero=%.2f, Uq=%.2f\n",
											 //target_angle, shaft_angle, electrical_angle, zero_electric_angle, voltage.q);
						//}
				
						//							if(debug_count++ > 50) {
						//								debug_count = 0;
						//								printf("POS_CTRL: Target=%.2f, Actual=%.2f, Error=%.2f, Uq=%.2f\n",
						//											 target_angle, shaft_angle, target_angle - shaft_angle, voltage.q);
						//								printf("Electrical: %.2f, Zero: %.2f\n", electrical_angle, zero_electric_angle);
						//						}


						P_angle.P = 6;  // Reduced from 5.0 to prevent saturation
						P_angle.I = 0.1;  // Add small integral term to overcome stiction
						P_angle.D = 0.1;  // Add derivative term for damping
						
						
						float error = target_angle - shaft_angle;
    
						// ?????[-p, p]???(????)
						if (error > _PI) {
								error -= _2PI;
						} else if (error < -_PI) {
								error += _2PI;
						}

						// Calculate PID output with anti-windup
						voltage.q = PIDoperator(&P_angle, error);
						
						// Add feed-forward term (adjust based on your motor)
//						float feed_forward = 0.2 * _sign(target_angle - shaft_angle);
//						voltage.q += feed_forward;
						
						// Apply voltage limiting
						voltage.q = _constrain(voltage.q, -voltage_limit, voltage_limit);
						
						
						    // ???? - ?????????????
						static int debug_count = 0;
						if(debug_count++ > 50) {
								debug_count = 0;
								float raw_error = target_angle - shaft_angle;
								printf("RAW_ERR: %.2f, ADJ_ERR: %.2f, Target: %.2f, Actual: %.2f\n",
											 raw_error, error, target_angle, shaft_angle);
						}

		
						DEBUG_PRINTF("Position: target=%.2f, current=%.2f, err=%.2f, Uq=%.2f\n", 
												 target_angle, shaft_angle, target_angle - shaft_angle, voltage.q);
						break;
				}
				case OPEN_LOOP_SPEED_MODE:
					if (!openLoopInitialized) {
							initOpenLoopMode(setSpeedOpenLoop);
					}
					openLoopAngle += setSpeedOpenLoop * pole_pairs * 0.001f;
					openLoopAngle = _normalizeAngle(openLoopAngle);
					voltage.q = (setSpeedOpenLoop >= 0) ? 3.0f : -3.0f;
					voltage.d = 0;
					electrical_angle = openLoopAngle;
					break;

				case TRAJECTORY_MODE:
						updateTrajectoryMode();
						if (trajectory.active) {
								if (trajectory.stage == 0) {
										// Use the signed current speed directly
										trajectory.open_loop_angle += trajectory.current_speed * pole_pairs * 0.001f;
										trajectory.open_loop_angle = _normalizeAngle(trajectory.open_loop_angle);
										electrical_angle = trajectory.open_loop_angle;
										voltage.q = (trajectory.current_speed >= 0) ? 3.0f : -3.0f;
										voltage.d = 0;
									} 
//								else {
//										// Position control stage
//										P_angle.P = 6;
//										P_angle.I = 0.1;
//										P_angle.D = 0.1;
//										
//										float error = target_angle - shaft_angle;
//										
//										if (error > _PI) {
//												error -= _2PI;
//										} else if (error < -_PI) {
//												error += _2PI;
//										}

//										voltage.q = PIDoperator(&P_angle, error);
//										voltage.q = _constrain(voltage.q, -voltage_limit, voltage_limit);
//								}
						}
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
                    received_char == 'Q' || received_char == 'q'||
										received_char == 'S' || received_char == 's'||
                    received_char == 'T' || received_char == 't') {
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
										else if (cmd_buffer[0]  == 'S' || cmd_buffer[0] == 's') {
                        cmd_state = CMD_STATE_RECEIVE_DATA_A;
                    }
                    else if (cmd_buffer[0]  == 'T' || cmd_buffer[0] == 't') {
                        cmd_state = CMD_STATE_RECEIVE_DATA_T;
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
                if (received_char == ')' && cmd_idx > 5 && (cmd_buffer[0] == 'T' || cmd_buffer[0] == 't')) 
                {
                    cmd_buffer[cmd_idx] = '\0';
                    
                    // Parse trajectory parameters: T=(speed,accel,revolutions)
                    float speed = 0.0f, accel = 0.0f, revolutions = 0.0f;
                    int parsed = sscanf((const char*)cmd_buffer, "T=(%f,%f,%f", 
                                       &speed, &accel, &revolutions);
                    
                    if (parsed == 3) {
                        // Parameter validation
                        if (speed <= 0 || accel <= 0) {
                            printf("Error: Speed and acceleration must be positive\r\n");
                        } else {
                            // Initialize trajectory mode
                            initTrajectoryMode(speed, accel, revolutions);
                            pattern = TRAJECTORY_MODE;
                        }
                    } else {
                        printf("Error parsing trajectory parameters\r\n");
                    }
                    
                    cmd_state = CMD_STATE_IDLE;
                    cmd_idx = 0;
                }
                else if (received_char == ';' && cmd_idx >= 3) 
                {
                    cmd_buffer[cmd_idx] = '\0';
                    
                    // Parse float value
                    int sign = 1;
                    float value = 0.0f;
                    int decimal = -1;
                    
                    for (int i = 2; i < cmd_idx; i++)
                    {
                        if (cmd_buffer[i] == '-') sign = -1;
                        else if (cmd_buffer[i] == '.') decimal = 0;
                        else if (cmd_buffer[i] >= '0' && cmd_buffer[i] <= '9')
                        {
                            if (decimal >= 0) value += (cmd_buffer[i] - '0') * powf(0.1f, ++decimal);
                            else value = value * 10.0f + (cmd_buffer[i] - '0');
                        }
                    }
                    
                    value *= sign;
                    
                    // Handle different commands
                    if(cmd_buffer[0] == 'A' || cmd_buffer[0] == 'a')
                    {
                        // Angle command
                        target_angle = value * reduction_ratio;
                        pattern = 4; // Switch to position mode
                        printf("Set Target Angle: %.3f rad\r\n", target_angle);
                        command_ready = 1;
                    }
                    else if(cmd_buffer[0] == 'S' || cmd_buffer[0] == 's')
                    {
                        // Open-loop speed command
                        setSpeedOpenLoop = value;
                        pattern = OPEN_LOOP_SPEED_MODE; // Switch to open-loop mode
                        printf("Set OpenLoop Speed: %.2f rad/s\r\n", setSpeedOpenLoop);
                    }
                    
                    cmd_state = CMD_STATE_IDLE;
                    cmd_idx = 0;
                }
                else
                {
                    if (cmd_idx < (CMD_BUFFER_SIZE - 1))
                    {
                        cmd_buffer[cmd_idx++] = received_char;
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
						case CMD_STATE_RECEIVE_DATA_T:
								if (received_char == ')') {
										cmd_buffer[cmd_idx] = '\0';
										float speed, accel, target_angle_val;
										// Parse T=(speed,accel,target_angle)
										if (sscanf((const char*)cmd_buffer, "T=(%f,%f,%f", &speed, &accel, &target_angle_val) == 3) {
												initTrajectoryMode(speed, accel, target_angle_val);
												pattern = TRAJECTORY_MODE;
										} else {
												printf("Error parsing trajectory parameters\r\n");
										}
										cmd_state = CMD_STATE_IDLE;
										cmd_idx = 0;
								} else if (cmd_idx < CMD_BUFFER_SIZE - 1) {
										cmd_buffer[cmd_idx++] = received_char;
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
