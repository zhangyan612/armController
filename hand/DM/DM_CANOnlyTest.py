import can
import time

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
    bus.send(msg)
    print(f"Motor {can_id} enabled")

# 失能电机
def disable_motor(bus, can_id):
    data = [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD]
    msg = can.Message(
        is_extended_id=False,
        arbitration_id=can_id,
        data=data
    )
    bus.send(msg)
    print(f"Motor {can_id} disabled")

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
    # print(f"Sent MIT control to motor {can_id}")

# 解析电机反馈
def parse_feedback(data):
    """
    解析电机反馈数据
    :param data: 8字节数据列表
    :return: 位置(rad), 速度(rad/s), 扭矩(Nm)
    """
    # 解析原始值
    q_uint = (data[1] << 8) | data[2]
    dq_uint = (data[3] << 4) | (data[4] >> 4)
    tau_uint = ((data[4] & 0x0F) << 8) | data[5]
    
    # 转换为实际物理值
    q = uint_to_float(q_uint, -DM_4310_Limit.Q_MAX, DM_4310_Limit.Q_MAX, 16)
    dq = uint_to_float(dq_uint, -DM_4310_Limit.DQ_MAX, DM_4310_Limit.DQ_MAX, 12)
    tau = uint_to_float(tau_uint, -DM_4310_Limit.TAU_MAX, DM_4310_Limit.TAU_MAX, 12)
    
    return q, dq, tau

def main():
    # 电机参数
    motor_id = 0x01
    control_frequency = 100  # Hz (控制频率)
    
    try:
        # 创建CAN总线连接 (921600波特率)
        bus = can.interface.Bus(interface='socketcan', channel='can0', bitrate=921600)
        print("Connected to CAN bus at 921600 bps")
        
        # 使能电机
        enable_motor(bus, motor_id)
        time.sleep(0.1)  # 等待电机响应
        
        # 主控制循环
        print("Starting control loop. Press Ctrl+C to stop.")
        try:
            while True:
                # 发送控制命令 (示例值)
                control_mit(bus, motor_id, 
                           kp=50.0,   # 位置增益
                           kd=1.0,    # 速度增益
                           q=1.0,     # 目标位置 (rad)
                           dq=0.0,    # 目标速度 (rad/s)
                           tau=0.5)   # 目标扭矩 (Nm)
                
                # 接收反馈
                feedback = bus.recv(timeout=1.0/control_frequency)
                
                if feedback and feedback.arbitration_id == motor_id:
                    q, dq, tau = parse_feedback(feedback.data)
                    print(f"Position: {q:.3f} rad | Speed: {dq:.3f} rad/s | Torque: {tau:.3f} Nm")
                
                # 控制频率
                time.sleep(1.0/control_frequency)
                
        except KeyboardInterrupt:
            print("\nControl loop stopped by user")
            
    except Exception as e:
        print(f"Error: {e}")
    finally:
        # 失能电机
        disable_motor(bus, motor_id)
        bus.shutdown()
        print("CAN bus disconnected")

if __name__ == "__main__":
    main()