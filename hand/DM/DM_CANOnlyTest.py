import can
import time
import os
import sys
import select
import math

# 达妙4310电机参数限制
class Limit_Motor:
    def __init__(self, P_MAX: float, V_MAX: float, T_MAX: float):
        self.Q_MAX = P_MAX    # 位置最大值 (rad)
        self.DQ_MAX = V_MAX   # 速度最大值 (rad/s)
        self.TAU_MAX = T_MAX  # 扭矩最大值 (Nm)

# 达妙4310电机参数
DM_4310_Limit = Limit_Motor(12.5, 30, 10)

# 数值范围限制函数
def limit_value(x, min_val, max_val):
    if x < min_val:
        return min_val
    elif x > max_val:
        return max_val
    return x

# 浮点数转整型
def float_to_uint(x: float, x_min: float, x_max: float, bits):
    x = limit_value(x, x_min, x_max)
    span = x_max - x_min
    data_norm = (x - x_min) / span
    return int(data_norm * ((1 << bits) - 1))

# 整型转浮点数
def uint_to_float(x: int, min_val: float, max_val: float, bits):
    span = max_val - min_val
    data_norm = float(x) / ((1 << bits) - 1)
    return data_norm * span + min_val

# 使能电机
def enable_motor(bus, can_id):
    data = [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC]
    msg = can.Message(
        is_extended_id=False,
        arbitration_id=can_id,
        data=data
    )
    try:
        bus.send(msg)
        print(f"Motor {can_id} enable command sent")
        return True
    except can.CanError as e:
        print(f"Error sending enable command: {e}")
        return False

# 失能电机
def disable_motor(bus, can_id):
    data = [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD]
    msg = can.Message(
        is_extended_id=False,
        arbitration_id=can_id,
        data=data
    )
    try:
        bus.send(msg)
        print(f"Motor {can_id} disable command sent")
        return True
    except can.CanError as e:
        print(f"Error sending disable command: {e}")
        return False

# MIT控制模式
def control_mit(bus, can_id, kp: float, kd: float, q: float, dq: float, tau: float):
    """
    MIT控制模式
    :param bus: CAN总线对象
    :param can_id: 电机ID
    :param kp: 位置增益
    :param kd: 速度增益
    :param q: 目标位置 (rad)
    :param dq: 目标速度 (rad/s)
    :param tau: 目标扭矩 (Nm)
    """
    try:
        # 转换参数为整型值
        kp_uint = float_to_uint(kp, 0, 500, 12)
        kd_uint = float_to_uint(kd, 0, 5, 12)
        q_uint = float_to_uint(q, -DM_4310_Limit.Q_MAX, DM_4310_Limit.Q_MAX, 16)
        dq_uint = float_to_uint(dq, -DM_4310_Limit.DQ_MAX, DM_4310_Limit.DQ_MAX, 12)
        tau_uint = float_to_uint(tau, -DM_4310_Limit.TAU_MAX, DM_4310_Limit.TAU_MAX, 12)
        
        # 构建数据帧
        data = [
            (q_uint >> 8) & 0xFF,    # 位置高8位
            q_uint & 0xFF,            # 位置低8位
            dq_uint >> 4,             # 速度高8位
            ((dq_uint & 0xF) << 4) | ((kp_uint >> 8) & 0xF),  # 速度低4位 + KP高4位
            kp_uint & 0xFF,           # KP低8位
            kd_uint >> 4,             # KD高8位
            ((kd_uint & 0xF) << 4) | ((tau_uint >> 8) & 0xF), # KD低4位 + 扭矩高4位
            tau_uint & 0xFF           # 扭矩低8位
        ]
        
        # 发送CAN消息
        msg = can.Message(
            is_extended_id=False,
            arbitration_id=can_id,
            data=data
        )
        bus.send(msg)
        return True
    except Exception as e:
        print(f"Error in control_mit: {e}")
        return False

# 解析电机反馈
def parse_feedback(data):
    """
    解析电机反馈数据
    :param data: 8字节数据列表
    :return: 位置(rad), 速度(rad/s), 扭矩(Nm)
    """
    try:
        # 解析原始值
        q_uint = (data[1] << 8) | data[2]
        dq_uint = (data[3] << 4) | (data[4] >> 4)
        tau_uint = ((data[4] & 0x0F) << 8) | data[5]
        
        # 转换为实际物理值
        q = uint_to_float(q_uint, -DM_4310_Limit.Q_MAX, DM_4310_Limit.Q_MAX, 16)
        dq = uint_to_float(dq_uint, -DM_4310_Limit.DQ_MAX, DM_4310_Limit.DQ_MAX, 12)
        tau = uint_to_float(tau_uint, -DM_4310_Limit.TAU_MAX, DM_4310_Limit.TAU_MAX, 12)
        
        return q, dq, tau
    except Exception as e:
        print(f"Error parsing feedback: {e}")
        return 0.0, 0.0, 0.0

# 清空CAN缓冲区
def flush_can_buffer(bus):
    """清空CAN接收缓冲区"""
    flushed_count = 0
    start_time = time.time()
    while time.time() - start_time < 0.1:  # 清空100ms
        msg = bus.recv(timeout=0.01)
        if msg is None:
            break
        flushed_count += 1
    if flushed_count > 0:
        print(f"Flushed {flushed_count} messages from buffer")

# 检查CAN通道可用性
def check_can_channel(channel='can0'):
    """检查CAN通道是否可用"""
    return os.path.exists(f"/sys/class/net/{channel}")

# 增加系统接收缓冲区大小
def increase_socket_buffer():
    """增加系统socket缓冲区大小"""
    try:
        # 增加接收缓冲区大小
        os.system("sudo sysctl -w net.core.rmem_max=26214400")
        # 增加发送缓冲区大小
        os.system("sudo sysctl -w net.core.wmem_max=26214400")
        print("Increased socket buffer sizes")
    except Exception as e:
        print(f"Error increasing socket buffers: {e}")

def main():
    # 电机参数
    motor_id = 0x01
    target_frequency = 50  # Hz (目标控制频率)
    cycle_time_target = 1.0 / target_frequency
    
    # 检查CAN通道是否可用
    if not check_can_channel('can0'):
        print("Error: can0 interface not found. Check hardware connection.")
        sys.exit(1)
    
    # 尝试增加socket缓冲区大小
    increase_socket_buffer()
    
    try:
        # 创建CAN总线连接 (921600波特率)
        bus = can.interface.Bus(
            interface='socketcan',
            channel='can0',
            bitrate=921600,
            receive_own_messages=False  # 不接收自己发送的消息
        )
        print(f"Connected to CAN bus at 921600 bps. Target frequency: {target_frequency}Hz")
        
        # 清空缓冲区
        flush_can_buffer(bus)
        
        # 使能电机
        if not enable_motor(bus, motor_id):
            print("Failed to enable motor. Exiting.")
            return
        
        # 等待电机响应
        print("Waiting for motor feedback...")
        feedback_received = False
        start_time = time.time()
        
        # 等待反馈最多3秒
        while time.time() - start_time < 3.0:
            msg = bus.recv(timeout=0.1)
            if msg and msg.arbitration_id == motor_id:
                q, dq, tau = parse_feedback(msg.data)
                print(f"Initial feedback: Position={q:.3f} rad, Speed={dq:.3f} rad/s, Torque={tau:.3f} Nm")
                feedback_received = True
                break
        
        if not feedback_received:
            print("Warning: No feedback received after enabling motor.")
            # 尝试继续但降低控制频率
            target_frequency = 30
            cycle_time_target = 1.0 / target_frequency
            print(f"Reducing control frequency to {target_frequency} Hz")
        
        # 主控制循环
        print("Starting control loop. Press Ctrl+C to stop.")
        cycle_count = 0
        total_cycle_time = 0
        min_cycle_time = float('inf')
        max_cycle_time = 0
        
        # 初始位置设置
        target_position = 0.0
        position_step = 0.1  # 位置步进值
        max_position = 3.0   # 最大位置
        
        try:
            while True:
                cycle_start = time.time()
                
                # 更新目标位置 (正弦波运动)
                # target_position = max_position * math.sin(cycle_count * 0.05)
                
                # 或者使用步进运动
                target_position = (target_position + position_step) % max_position
                
                # 发送控制命令
                if not control_mit(bus, motor_id, 
                                  kp=50.0,          # 位置增益
                                  kd=1.0,           # 速度增益
                                  q=target_position, # 目标位置 (rad)
                                  dq=0.0,           # 目标速度 (rad/s)
                                  tau=0.5):         # 目标扭矩 (Nm)
                    print("Failed to send control command")
                
                # 接收反馈 - 增加等待时间
                feedback = None
                receive_timeout = min(cycle_time_target * 1.5, 0.05)  # 最多等待50ms
                
                # 等待反馈消息
                feedback = bus.recv(timeout=receive_timeout)
                
                if feedback and feedback.arbitration_id == motor_id:
                    q, dq, tau = parse_feedback(feedback.data)
                    if cycle_count % 10 == 0:  # 每10个周期打印一次
                        print(f"Target: {target_position:.2f} rad | Actual: {q:.3f} rad | Speed: {dq:.3f} rad/s | Torque: {tau:.3f} Nm")
                elif feedback:
                    # 接收到其他ID的消息
                    pass
                else:
                    if cycle_count % 20 == 0:  # 每20个周期打印一次
                        print("No feedback received this cycle")
                
                # 计算实际循环时间
                cycle_duration = time.time() - cycle_start
                total_cycle_time += cycle_duration
                min_cycle_time = min(min_cycle_time, cycle_duration)
                max_cycle_time = max(max_cycle_time, cycle_duration)
                
                # 每100个周期打印性能统计
                if cycle_count % 100 == 0 and cycle_count > 0:
                    avg_cycle_time = total_cycle_time / 100
                    actual_frequency = 1.0 / avg_cycle_time
                    print(f"Performance: Min={min_cycle_time*1000:.1f}ms, Max={max_cycle_time*1000:.1f}ms, Avg={avg_cycle_time*1000:.1f}ms, Freq={actual_frequency:.1f}Hz")
                    # 重置统计
                    total_cycle_time = 0
                    min_cycle_time = float('inf')
                    max_cycle_time = 0
                
                # 控制频率 - 自适应调整
                cycle_count += 1
                
                # 计算剩余时间并休眠
                remaining_time = cycle_time_target - cycle_duration
                if remaining_time > 0.001:  # 只有剩余时间>1ms才休眠
                    time.sleep(remaining_time)
                else:
                    if cycle_count % 50 == 0:
                        print(f"Cycle {cycle_count} exceeded: {cycle_duration*1000:.1f}ms > {cycle_time_target*1000:.1f}ms")
                    
                    # 如果连续超时，考虑降低频率
                    if cycle_duration > cycle_time_target * 1.5 and target_frequency > 20:
                        new_frequency = max(20, target_frequency - 5)
                        print(f"Reducing frequency from {target_frequency}Hz to {new_frequency}Hz due to consistent overruns")
                        target_frequency = new_frequency
                        cycle_time_target = 1.0 / target_frequency
                
        except KeyboardInterrupt:
            print("\nControl loop stopped by user")
            
    except can.CanError as e:
        print(f"CAN bus error: {e}")
    except Exception as e:
        print(f"Unexpected error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # 失能电机
        if 'bus' in locals():
            disable_motor(bus, motor_id)
            bus.shutdown()
            print("CAN bus disconnected")

if __name__ == "__main__":
    main()