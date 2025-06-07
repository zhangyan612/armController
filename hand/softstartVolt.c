// Soft-start PWM motor control to protect low voltage motors
// Add these variables to your global declarations

typedef struct {
    uint8_t motor_id;
    uint16_t target_pwm;        // Final PWM value we want to reach
    uint16_t current_pwm;       // Current PWM value being output
    uint16_t ramp_step;         // How much to increase PWM each step
    uint8_t is_ramping;         // Flag to indicate if motor is ramping up
    uint8_t ramp_delay_counter; // Counter for ramp timing
    uint8_t ramp_delay_max;     // How many timer ticks between ramp steps
} soft_start_motor_t;

// Array to track soft-start for each motor
soft_start_motor_t soft_start_motors[12] = {0};

// Configuration constants
#define RAMP_STEP_DEFAULT    10     // Default PWM increment per step (adjust as needed)
#define RAMP_DELAY_DEFAULT   5      // Timer ticks between steps (adjust for speed)
#define MAX_SAFE_PWM_6V      500    // Maximum PWM for 6V motors (adjust based on your PWM resolution)

// Initialize soft start for a motor
void init_soft_start_motor(uint8_t motor_id, uint16_t target_pwm, uint8_t is_6v_motor)
{
    if (motor_id >= 1 && motor_id <= 12) {
        uint8_t idx = motor_id - 1;
        
        // Limit PWM for 6V motors
        if (is_6v_motor && target_pwm > MAX_SAFE_PWM_6V) {
            target_pwm = MAX_SAFE_PWM_6V;
            printf("限制6V电机PWM: %d\r\n", target_pwm);
        }
        
        soft_start_motors[idx].motor_id = motor_id;
        soft_start_motors[idx].target_pwm = target_pwm;
        soft_start_motors[idx].current_pwm = 0;  // Start from 0
        soft_start_motors[idx].ramp_step = RAMP_STEP_DEFAULT;
        soft_start_motors[idx].is_ramping = 1;
        soft_start_motors[idx].ramp_delay_counter = 0;
        soft_start_motors[idx].ramp_delay_max = RAMP_DELAY_DEFAULT;
        
        printf("电机 %d 软启动初始化: 目标PWM=%d\r\n", motor_id, target_pwm);
    }
}

// Process soft start ramping - call this in your timer interrupt or main loop
void process_soft_start_ramping(void)
{
    for (uint8_t i = 0; i < 12; i++) {
        if (soft_start_motors[i].is_ramping && soft_start_motors[i].motor_id > 0) {
            // Check if it's time to increment PWM
            soft_start_motors[i].ramp_delay_counter++;
            
            if (soft_start_motors[i].ramp_delay_counter >= soft_start_motors[i].ramp_delay_max) {
                soft_start_motors[i].ramp_delay_counter = 0;
                
                // Increment PWM value
                if (soft_start_motors[i].current_pwm < soft_start_motors[i].target_pwm) {
                    soft_start_motors[i].current_pwm += soft_start_motors[i].ramp_step;
                    
                    // Don't overshoot target
                    if (soft_start_motors[i].current_pwm > soft_start_motors[i].target_pwm) {
                        soft_start_motors[i].current_pwm = soft_start_motors[i].target_pwm;
                    }
                    
                    // Update the motor data array with current PWM value
                    uint8_t motor_id = soft_start_motors[i].motor_id;
                    uint8_t data_idx = (motor_id - 1) * 4;
                    
                    // Check which direction (forward or reverse) and update accordingly
                    if (Moto_dat[data_idx + 1] > 0) {
                        // Forward direction
                        Moto_dat[data_idx + 1] = soft_start_motors[i].current_pwm;
                    } else if (Moto_dat[data_idx + 2] > 0) {
                        // Reverse direction  
                        Moto_dat[data_idx + 2] = soft_start_motors[i].current_pwm;
                    }
                    
                    printf("电机 %d 软启动: PWM=%d/%d\r\n", 
                           motor_id, soft_start_motors[i].current_pwm, soft_start_motors[i].target_pwm);
                    
                    // Check if we've reached target
                    if (soft_start_motors[i].current_pwm >= soft_start_motors[i].target_pwm) {
                        soft_start_motors[i].is_ramping = 0;
                        printf("电机 %d 软启动完成\r\n", motor_id);
                    }
                }
            }
        }
    }
}

// Stop soft start for a motor
void stop_soft_start_motor(uint8_t motor_id)
{
    if (motor_id >= 1 && motor_id <= 12) {
        uint8_t idx = motor_id - 1;
        soft_start_motors[idx].is_ramping = 0;
        soft_start_motors[idx].current_pwm = 0;
        soft_start_motors[idx].target_pwm = 0;
        soft_start_motors[idx].motor_id = 0;
        printf("电机 %d 软启动已停止\r\n", motor_id);
    }
}

// Modified Moto_dat_get function with soft start support
void Moto_dat_get_with_soft_start(uint8_t is_6v_motor)
{
    if (Moto <= 12 && Moto > 0) {
        Moto_dat[(Moto-1)*4] = Moto;  // 存储电机ID
        flag_tim = 1;
    } else {
        printf("电机超范围!!!\r\n");
        flag_tim = 0;
        return;
    }
    
    uint16_t calculated_pwm = 0;
    uint8_t direction = 0; // 0=stop, 1=forward, 2=reverse
    
    if (Speed < 2000) {
        calculated_pwm = 2000 - Speed;
        direction = 2; // Reverse
    } else if (Speed > 2000) {
        calculated_pwm = Speed - 2000;
        direction = 1; // Forward
    } else if (Speed == 2000) {
        calculated_pwm = 0;
        direction = 0; // Stop
    } else {
        printf("速度错误!!!\r\n");
        flag_tim = 0;
        return;
    }
    
    if (flag_tim == 1) {
        // Don't set PWM directly - let soft start handle it
        if (calculated_pwm > 0) {
            // Initialize soft start
            init_soft_start_motor(Moto, calculated_pwm, is_6v_motor);
            
            // Set direction but start with 0 PWM
            if (direction == 1) {
                Moto_dat[Moto*4-3] = 0;  // Will be ramped up by soft start
                Moto_dat[Moto*4-2] = 0;  // Reverse PWM = 0
            } else if (direction == 2) {
                Moto_dat[Moto*4-3] = 0;  // Forward PWM = 0
                Moto_dat[Moto*4-2] = 0;  // Will be ramped up by soft start
            }
        } else {
            // Stop motor immediately
            Moto_dat[Moto*4-3] = 0;
            Moto_dat[Moto*4-2] = 0;
            stop_soft_start_motor(Moto);
        }
        
        Moto_dat[Moto*4-1] = time;       // 存储时间
        tim_user[Moto-1] = TimerCnt;     // 记录启动时间
    }
}

// Modified stop_motor function to handle soft start
void stop_motor_with_soft_start(uint8_t motor_id)
{
    if (motor_id >= 1 && motor_id <= 12) {
        uint8_t idx = (motor_id-1)*4;
        
        // Stop soft start ramping
        stop_soft_start_motor(motor_id);
        
        // Clear PWM values immediately
        Moto_dat[idx+1] = 0;         // 正转PWM置0
        Moto_dat[idx+2] = 0;         // 反转PWM置0
        Moto_dat[idx+3] = 0;         // 运行时间置0
        
        // Clear motor ID
        Moto_dat[idx] = 0;           // 电机ID置0
        
        tim_user[motor_id-1] = 0;     // 清除时间记录
        
        printf("电机 %d 已停止\r\n", motor_id);
    } else {
        printf("无效电机ID: %d\r\n", motor_id);
    }
}

// Modified main loop to include soft start processing
void loop_with_soft_start(void)
{
    // Process soft start ramping
    process_soft_start_ramping();
    
    // Check motor timeouts (existing logic)
    for (uint8_t my_moto_tim = 0; my_moto_tim < 12; my_moto_tim++)
    {
        // Skip inactive motors
        if (Moto_dat[my_moto_tim * 4] == 0) continue;
        
        // Check timeout
        if(TimerCnt - tim_user[my_moto_tim] >= Moto_dat[my_moto_tim * 4 + 3])
        {
            // Time expired, stop motor
            stop_motor_with_soft_start(my_moto_tim + 1);
        }
    }
    
    // Update PWM output
    PWM_get();
}

// Usage example in your command processing:
void process_motor_command_with_soft_start(uint8_t* cmd_data)
{
    // ... existing command parsing ...
    
    if (cmd_data[0] == '#' && cmd_data[3] == '#' && 
        cmd_data[8] == '#' && cmd_data[14] == '#') 
    {
        num_data_get(cmd_data);
        value_get();
        
        // Determine if this is a 6V motor (you can modify this logic)
        uint8_t is_6v_motor = 0;
        if (Moto == 1 || Moto == 2 || Moto == 3) {  // Example: motors 1,2,3 are 6V
            is_6v_motor = 1;
        }
        
        Moto_dat_get_with_soft_start(is_6v_motor);
        
        printf("电机=%d (6V=%d)\r\n", Moto, is_6v_motor);
        printf("速度=%d\r\n", Speed);
        printf("时间=%d\r\n", time);
        printf("\r\n");
        
        HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    }
}