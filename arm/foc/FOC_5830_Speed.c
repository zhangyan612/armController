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

// Global variables
extern float setSpeed;          // Target speed value
extern float setCurrent;         // Target current value
extern float setPosition ;       // Target position value

extern ADC_HandleTypeDef hadc1;

float shaft_angle,sensor_offset,zero_electric_angle,electrical_angle;
int pole_pairs=1,pattern=0;
char sensor_direction=1;
long angle_data, angle_data_prev; // Current and previous angle values
float full_rotation_offset;      // Full rotation offset for angle unwrapping
extern uint8_t isRunning ;       // Motor running status flag

DQVoltage_s voltage;             // DQ voltage values
Kalman kalman_current_Iq_Filter;
DQCurrent_s current;             // DQ current values

// Encoder configuration
#define ENCODER_RESOLUTION 128

volatile uint8_t motor_enabled = 0;  // Motor enable flag

// Encoder initialization values
#define ENCODER_MAX_COUNT 65535
int32_t rotation_count = 0;     // Rotation counter
int16_t last_count = 0;         // Previous encoder count
float shaft_angle = 0.0f;       // Current shaft angle
float Target_speed,shaft_velocity,full_rotation_offset;
int MGT_angle;
char MGT_angle_rx_flag=0;
char feedback_send_flag;

// Structure for ADC readings
typedef struct{
    float adc_10;
    float adc_11;
} adc_1011;

adc_1011 ADC_1011;

// Read ADC voltage values
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

// Get phase currents
PhaseCurrent_s getPhaseCurrents(void)
{
    PhaseCurrent_s current;
    ADC_1011=_readADCVoltageInline();
    current.a = (ADC_1011.adc_10 - 1.65)*10;  // amps
    current.b = (ADC_1011.adc_11 - 1.67)*10;  // amps
    current.c = 0;  // amps
    
    return current;
}

// Get FOC currents
DQCurrent_s getFOCCurrents(float angle_el)
{
    PhaseCurrent_s current;
    float i_alpha, i_beta;
    float ct,st;
    DQCurrent_s ret;
    
    // Read current phase currents
    current = getPhaseCurrents();
    
    // Calculate Clarke transform
    if(!current.c)
    {
        // If only two measured currents
        i_alpha = -current.b;  
        i_beta = _1_SQRT3 * current.b + _2_SQRT3 * current.a;
    }
    else
    {
        // Signal filtering using identity a + b + c = 0
        float mid = (1.f/3) * (current.a + current.b + current.c);
        float a = current.a - mid;
        float b = current.b - mid;
        i_alpha = a;
        i_beta = _1_SQRT3 * a + _2_SQRT3 * b;
    }
    
    // Calculate Park transform
    ct = _cos(angle_el);
    st = _sin(angle_el);
    ret.d = i_alpha * ct + i_beta * st;
    ret.q = i_beta * ct - i_alpha * st;
    
    // Filter Q current
    ret.q=Kalman_Filter(&kalman_current_Iq_Filter,ret.q);
    ret.q= Slid_Filter(&Filter_ret_q,ret.q,40);

    return ret;
}

// PID parameters for position control
float Position_KP = 0.15, Position_KI = 0.001, Position_KD = 0.32;
float Incremental_KP = 1.1, Incremental_KI = 0.09, Incremental_KD = 0.25;

// Position PID controller
float Position_PID(float reality, float target)
{ 	
    static float Bias, Pwm, Last_Bias, Integral_bias = 0;
    
    Bias = target - reality;              // Calculate error
    Integral_bias += Bias;	            // Integrate error
    
    // Anti-windup clamping
    if(Integral_bias > 5) Integral_bias = 5;
    if(Integral_bias < -5) Integral_bias = -5;
    
    // Calculate PID output
    Pwm = (Position_KP * Bias) + (Position_KI * Integral_bias) + (Position_KD * (Bias - Last_Bias));
    
    Last_Bias = Bias;  // Store last error
    return Pwm;
}

// Incremental PID controller
float Bias_buf;
float Incremental_PID(float reality, float target)
{ 	
    static float Bias, Pwm, Last_bias = 0, Prev_bias = 0;
    
    Bias = target - reality;  // Calculate error
    
    // Calculate PID output
    Pwm += (Incremental_KP * (Bias - Last_bias)) +
           (Incremental_KI * Bias) +
           (Incremental_KD * (Bias - 2*Last_bias + Prev_bias));
    
    Prev_bias = Last_bias;  // Store previous last error
    Last_bias = Bias;       // Store current error
        
    // Constrain output
    Pwm = _constrain(Pwm, -3, 3);
    return Pwm;
}

// Calculate electrical angle
float electricalAngle(void)
{
    return _normalizeAngle((shaft_angle + sensor_offset) * pole_pairs - zero_electric_angle);
}

// Set phase voltages
void setPhaseVoltage(float Uq, float Ud, float angle_el)
{
    float Uout;        // Normalized voltage magnitude
    uint32_t sector;   // Current PWM sector (1-6)
    float T0, T1, T2;  // PWM duty cycles
    float Ta, Tb, Tc;  // Phase duty cycles
    
    // Constrain Q voltage
    if(Uq > voltage_limit) Uq = voltage_limit;
    if(Uq < -voltage_limit) Uq = -voltage_limit;
    
    // Constrain D voltage
    if(Ud > voltage_limit) Ud = voltage_limit;
    if(Ud < -voltage_limit) Ud = -voltage_limit;
    
    // Handle D voltage
    if(Ud)
    {
        // When both Ud and Uq are used
        Uout = _sqrt(Ud * Ud + Uq * Uq) / voltage_power_supply;
        angle_el = _normalizeAngle(angle_el + atan2(Uq, Ud));
    }
    else
    {
        // When only Uq is used
        Uout = Uq / voltage_power_supply;
        angle_el = _normalizeAngle(angle_el + _PI_2);
    }
    
    // Constrain normalized voltage
    if(Uout > 0.577) Uout = 0.577;
    if(Uout < -0.577) Uout = -0.577;
    
    // Determine current sector
    sector = (angle_el / _PI_3) + 1;
    
    // Calculate PWM duty cycles
    T1 = _SQRT3 * _sin(sector * _PI_3 - angle_el) * Uout;
    T2 = _SQRT3 * _sin(angle_el - (sector - 1.0) * _PI_3) * Uout;
    T0 = 1 - T1 - T2;
    
    // Calculate phase duty cycles based on sector
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
            // Default to zero voltage
            Ta = 0;
            Tb = 0;
            Tc = 0;
    }
    
    // Set PWM duty cycles
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, Ta * PWM_Period);  // Phase A
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, Tb * PWM_Period);  // Phase B
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, Tc * PWM_Period);  // Phase C
}

// Get current angle from encoder
float getAngle(void)
{
    float d_angle; // Angle change value

    angle_data =  MGT_angle ; // Get current angle value

    // Calculate angle change and handle full rotations
    d_angle = angle_data - angle_data_prev;

    // Handle full rotation offset
    if(fabs(d_angle) > (0.8 * 16383)) {
        if (d_angle > 0) {
            full_rotation_offset -= _2PI;
            rotation_count--; 
        } else {
            full_rotation_offset += _2PI;
            rotation_count++; 
        }
    }

    // Update previous angle
    angle_data_prev = angle_data;

    // Return unwrapped angle
    return (full_rotation_offset + (angle_data * 1.0 / 16383) * _2PI);
}

// Open-loop speed mode variables
#define OPEN_LOOP_SPEED_MODE 5  // New mode for open-loop speed control
float setSpeedOpenLoop = 0.0f;   // Set speed for open-loop mode
float openLoopAngle = 0.0f;      // Accumulated electrical angle for open-loop
uint8_t openLoopInitialized = 0; // Initialization flag for open-loop mode

// Motor control states
float current_target_angle = 0.0f;
float initial_angle = 0.0f;
char motion_complete = 0;
char test_flag;
float total_target_angle = 8.0f * (float)_PI; // 8 rotations in rad
float desired_speed = 2.0f * (float)_PI;      // 2 rad/s
#define CONTROL_PERIOD 0.1f

// Command states
typedef enum {
    CMD_STATE_WAIT_START,       // Waiting for start byte
    CMD_STATE_RECEIVE_COMMAND,  // Receiving command
} CMD_State;

// Command types
typedef enum {
    CMD_NONE,                // No command
    CMD_SET_REDUCTION_RATIO, // Set reduction ratio
    CMD_SET_TARGET_ANGLE     // Set target angle
} CommandType;

// Command buffer size
#define CMD_BUFFER_SIZE 64

// Command processing variables
volatile char cmd_buffer[CMD_BUFFER_SIZE];
volatile uint8_t cmd_idx = 0;
volatile CommandType received_command = CMD_NONE;
volatile float target_angle = 0.0f;
volatile int reduction_ratio = 1;
volatile uint8_t command_ready = 0; // Command ready flag

uint8_t tx_buffer[10] = {0x02};     // Transmit buffer
uint8_t rx_buffer[10];              // Receive buffer

// UART receive states
typedef enum {
    WAIT_FOR_START,
    WAIT_FOR_IDENT,
    WAIT_FOR_DATA1,
    WAIT_FOR_DATA2,
    WAIT_FOR_CHECKSUM
} ReceiveState;
volatile ReceiveState current_state = WAIT_FOR_START;
volatile uint8_t received_bytes[4]; // Received bytes storage
volatile uint8_t byte_index = 0;
uint8_t rx_byte, rx_byte_uart2;     // Received bytes
int uart2_doutntime_rx;

// RS485 mode control
#define RS485_TX_MODE() HAL_GPIO_WritePin(RS485_EN_GPIO_Port, RS485_EN_Pin, GPIO_PIN_SET)   // Transmit mode
#define RS485_RX_MODE() HAL_GPIO_WritePin(RS485_EN_GPIO_Port, RS485_EN_Pin, GPIO_PIN_RESET) // Receive mode

// Send request to encoder
void Send_Request(void)
{
    uint8_t tx_buffer = 0x02;
    RS485_TX_MODE(); // Set to transmit mode
    HAL_UART_Transmit(&huart1, &tx_buffer, 1, HAL_MAX_DELAY);
    RS485_RX_MODE(); // Set to receive mode
}

// Command buffer sizes
#define CMD_BUFFER_SIZE_A 12  // Command A buffer size (e.g., A=+123.456;)
#define CMD_BUFFER_SIZE_B 8   // Command B buffer size (e.g., B=0001;)

// Command states
typedef enum {
    CMD_STATE_IDLE,            // Idle state
    CMD_STATE_RECEIVE_EQUALS,  // Waiting for '=' after command
    CMD_STATE_RECEIVE_DATA_A,  // Receiving data for A command
    CMD_STATE_RECEIVE_DATA_B,  // Receiving data for B command
    CMD_STATE_RECEIVE_DATA_C,  // Receiving encoder value
    CMD_STATE_RECEIVE_DATA_E,  // Receiving data for E command
    CMD_STATE_RECEIVE_DATA_Q,  // Receiving rotation data
    CMD_STATE_RECEIVE_TERMINATOR // Waiting for terminator
} cmd_state_t;
volatile cmd_state_t cmd_state = CMD_STATE_IDLE;

char current_cmd_type = '\0';

// Enable motor driver
void motorEnable(void)
{
    motor_enabled = 1;
    printf("Motor enabled\r\n");
}

// Disable motor driver
void motorDisable(void)
{
    motor_enabled = 0;
    // Force motor stop by setting zero voltage
    setPhaseVoltage(0, 0, 0);
    printf("Motor disabled\r\n");
}

// Main motor control function
extern float Bias_buf;
void run()
{    
    // Initialize on first run
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
        HAL_UART_Receive_IT(&huart2, &rx_byte_uart2, 1);
    }

    // Reset open-loop initialization when switching modes
    if (pattern != OPEN_LOOP_SPEED_MODE) {
        openLoopInitialized = 0;
    }

    shaft_angle = getAngle();
    electrical_angle = electricalAngle();

    // Stop motor if disabled
    if (!motor_enabled) {
        voltage.q = 0;
        voltage.d = 0;
        setPhaseVoltage(0, 0, 0);
        return;
    }

    // Handle different control modes
    switch(pattern)
    {
        case 2:
            // Sensor alignment mode
            printf("MOT: Align sensor.\r\n");
            float mid_angle, end_angle;
            float moved;
            float angle;
            motor_enabled = 1;
            
            // Forward rotation
            for(int i=0; i<=500; i++)
            {
                angle = _3PI_2 + _2PI * i / 500.0;
                setPhaseVoltage(1, 0,  angle);
                Send_Request();
                HAL_Delay(2);
                printf("i=%d, angle=%.3f, MGT_angle=%d, getAngle=%.3f\n", i, angle, MGT_angle, getAngle());
            }
            Send_Request();
            HAL_Delay(2);
            mid_angle = getAngle();

            // Reverse rotation
            for(int i=500; i>=0; i--)
            {
                angle = _3PI_2 + _2PI * i / 500.0;
                setPhaseVoltage(1, 0,  angle);
                Send_Request();
                HAL_Delay(2);
            }
            Send_Request();
            HAL_Delay(2);
            end_angle = getAngle();
            setPhaseVoltage(0, 0, 0);
            HAL_Delay(200);

            printf("mid_angle=%.4f\r\n",mid_angle);
            printf("end_angle=%.4f\r\n",end_angle);
            
            // Determine sensor direction
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
            
            // Set zero position
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
            // Basic voltage control mode
            voltage.q =0.5;
            voltage.d =0;
            break;
            
        case 4:
            // Position control mode
            P_angle.P = 5;
            P_angle.I = 0;
            P_angle.D = 0;		
            voltage.q = PIDoperator(&P_angle, (shaft_angle - target_angle)); 		
            break;
            
        case OPEN_LOOP_SPEED_MODE:  // Open-loop speed mode (new)
            // Initialize on first entry
            if (!openLoopInitialized) {
                openLoopAngle = electrical_angle; // Start from current electrical angle
                openLoopInitialized = 1;
                printf("Open loop speed mode enabled. Speed: %.2f rad/s\r\n", setSpeedOpenLoop);
            }
            
            // Integrate angle: d_angle = speed (mech rad/s) * pole_pairs * dt
            openLoopAngle += setSpeedOpenLoop * pole_pairs * 0.001f; // dt ≈ 1ms
            openLoopAngle = _normalizeAngle(openLoopAngle);
            
            // Set fixed voltage (direction depends on speed sign)
            voltage.q = (setSpeedOpenLoop >= 0) ? 3.0f : -3.0f;
            voltage.d = 0;
            
            // Use integrated angle for control
            electrical_angle = openLoopAngle;
            break;
    }
    
    // Apply phase voltages
    setPhaseVoltage(voltage.q, voltage.d, electrical_angle);

    // Send feedback if requested
    if(feedback_send_flag == 1)
    {
        feedback_send_flag = 0;
    }
}

// System tick handler (1ms interval)
void SysTick_1ms()
{
    static float speedLpfBuf = 0; 
    static int count = 0;
    
    // Calculate velocity every 100ms
    if(count++ >= 100)
    {
        count = 0;
        float angle = shaft_angle;
        shaft_velocity = (angle - speedLpfBuf) / 100 * 1000; // rad/s
        speedLpfBuf = angle;
        shaft_velocity = Slid_Filter(&Filter_shaft_velocity, shaft_velocity, 5);
    }
}

// Timer period elapsed callback
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim->Instance == TIM2)
    {
        // Send encoder request
        Send_Request();
    }
    else if(htim->Instance == TIM4)
    {
        static int count_tim4 = 0;
        if(count_tim4++ >= 20)
        {
            count_tim4 = 0;
            feedback_send_flag = 1;
            HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
        }
        
        // Handle UART timeout
        if(uart2_doutntime_rx)
        {
            uart2_doutntime_rx--;
        }
        else
        {
            uart2_doutntime_rx = 300;
            cmd_state = CMD_STATE_IDLE;
            cmd_idx = 0;
            for(int i=0; i<10; i++) cmd_buffer[i] = 0;
            HAL_UART_Receive_IT(&huart2, &rx_byte_uart2, 1);
            
            // Clear overrun flag
            uint8_t data = (uint8_t)(huart2.Instance->DR);
            __HAL_UART_CLEAR_OREFLAG(&huart2);
        }
    }
}

// UART receive complete callback
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        // Encoder data protocol
        switch(current_state)
        {
            case WAIT_FOR_START:
                if (rx_byte == 0x02) current_state = WAIT_FOR_IDENT;
                break;

            case WAIT_FOR_IDENT:
                if (rx_byte == 0x08) current_state = WAIT_FOR_DATA1;
                else current_state = WAIT_FOR_START;
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
                // Verify checksum
                uint8_t calculated_checksum = 0x02 ^ 0x08 ^ received_bytes[0] ^ received_bytes[1];
                if (rx_byte == calculated_checksum)
                {
                    // Valid frame - update angle
                    MGT_angle = -(received_bytes[0] + (received_bytes[1] << 8));
                    MGT_angle_rx_flag = 1;
                }
                current_state = WAIT_FOR_START;
                break;

            default:
                current_state = WAIT_FOR_START;
                break;
        }
        HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
    }
    else if (huart->Instance == USART2)
    {
        // Command processing
        char received_char = rx_byte_uart2;
        uart2_doutntime_rx = 300;

        switch(cmd_state)
        {
            case CMD_STATE_IDLE:
                // Command detection
                if (received_char == 'A' || received_char == 'a' ||
                    received_char == 'B' || received_char == 'b' ||
                    received_char == 'E' || received_char == 'e' ||
                    received_char == 'Q' || received_char == 'q' ||
                    received_char == 'S' || received_char == 's') // New open-loop speed command
                {
                    cmd_buffer[0] = received_char;
                    cmd_idx = 1;
                    cmd_state = CMD_STATE_RECEIVE_EQUALS;
                }

                // Angle request
                if (received_char == 'C' || received_char == 'c') {
                    printf("Angle: %.6f rad, Raw: %d, Rotation: %ld\r\n", 
                           getAngle(), MGT_angle, rotation_count);
                }

                // Calibration request
                if (received_char == 'J' || received_char == 'j') {
                    pattern = 2;
                    printf("Remote calibration triggered: pattern=2\r\n");
                }
                break;

            case CMD_STATE_RECEIVE_EQUALS:
                if (received_char == '=') 
                {
                    cmd_buffer[cmd_idx++] = received_char;
                    switch(cmd_buffer[0])
                    {
                        case 'A': case 'a': cmd_state = CMD_STATE_RECEIVE_DATA_A; break;
                        case 'B': case 'b': cmd_state = CMD_STATE_RECEIVE_DATA_B; break;
                        case 'E': case 'e': cmd_state = CMD_STATE_RECEIVE_DATA_E; break;
                        case 'Q': case 'q': cmd_state = CMD_STATE_RECEIVE_DATA_Q; break;
                        case 'S': case 's': cmd_state = CMD_STATE_RECEIVE_DATA_A; break; // Use same parser as angle
                    }
                }
                else
                {
                    cmd_state = CMD_STATE_IDLE;
                    cmd_idx = 0;
                    for(int i=0; i<10; i++) cmd_buffer[i] = 0;
                }
                break;

            case CMD_STATE_RECEIVE_DATA_A:
                if (received_char == ';' && cmd_idx >= 3) 
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
                        printf("Set Target Angle: %.3f rad\r\n", target_angle);
                        command_ready = 1;
                    }
                    else if(cmd_buffer[0] == 'S' || cmd_buffer[0] == 's')
                    {
                        // Open-loop speed command (new)
                        setSpeedOpenLoop = value;
                        pattern = OPEN_LOOP_SPEED_MODE; // Switch to open-loop mode
                        printf("Set OpenLoop Speed: %.2f rad/s\r\n", setSpeedOpenLoop);
                    }
                    
                    cmd_state = CMD_STATE_IDLE;
                    cmd_idx = 0;
                }
                else
                {
                    if (cmd_idx < (CMD_BUFFER_SIZE_A - 1))
                    {
                        cmd_buffer[cmd_idx++] = received_char;
                    }
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
                cmd_state = CMD_STATE_IDLE;
                cmd_idx = 0;
                break;
        }
        HAL_UART_Receive_IT(&huart2, &rx_byte_uart2, 1);
    }
}