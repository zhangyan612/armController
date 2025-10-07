#!/usr/bin/python3
# -*- coding: UTF-8 -*-  

import can  
import time
import struct

hostID = 0xfd             #主机ID fd
pai = 3.1415926

# 不可改动
P_MIN = -12.5     #扭矩下限范围
P_MAX = 12.5      #扭矩上限范围
T_MIN = -12.0     #角度(弧度)下限范围
T_MAX = 12.0      #角度(弧度)上限范围
V_MIN = -30.0     #速度下限范围
V_MAX = 30.0      #速度上限范围
KP_MIN = 0.0      #KP下限范围
KP_MAX = 500.0    #KP上限范围
KD_MIN = 0.0      #KD下限范围
KD_MAX = 5.0      #KD上限范围

# Parameter indexes from documentation
PARAM_INDEXES = {
    0x7005: "run_mode",
    0x7006: "iq_ref", 
    0x700A: "spd_ref",
    0x700B: "limit_torque",
    0x7010: "cur_kp",
    0x7011: "cur_ki",
    0x7014: "cur_filt_gain",
    0x7016: "loc_ref",
    0x7017: "limit_spd",
    0x7018: "limit_cur",
    0x7019: "mechPos",
    0x701A: "iqf",
    0x701B: "mechVel",
    0x701C: "VBUS",
    0x701D: "rotation",
    0x701E: "loc_kp",
    0x701F: "spd_kp",
    0x7020: "spd_ki"
}

#=========================发送扩展CAN帧(主路线)=====================
def send_extended_frame_main(bus, arbitration_id, data, block_receive):  
    state = 0
    rx_data = [0 for i in range(8)]  
    message = can.Message(arbitration_id=arbitration_id, data=data, is_extended_id=True)  

    print(f"发送 -> 仲裁ID: 0x{arbitration_id:08X}, 数据: {[hex(x) for x in data]}")
    
    try:
        bus.send(message)  
        print("发送成功")
    except can.CanError as e:
        print(f"发送失败: {e}")
        return (1, rx_data)

    if block_receive == 1:
        time_s = time.time()
        while True:  
            try:
                message_ou = bus.recv(1.0)  # 1秒超时
                if message_ou is not None:  
                    rx_data = message_ou.data
                    print(f"接收 <- 仲裁ID: 0x{message_ou.arbitration_id:08X}, 数据: {[hex(x) for x in rx_data]}")
                    break
                elif time.time() - time_s > 1:
                    print("超时: 未收到回复")
                    state = 1
                    break
            except Exception as e:
                print(f"接收数据时出错: {e}")
                state = 1
                break

    return (state, rx_data)

#**************************************************************************
# 解析电机反馈数据 (通信类型2)
#**************************************************************************
def parse_motor_feedback(data):
    """
    解析电机反馈数据
    数据格式:
    Byte 0-1: 当前角度 [0-65535] 对应 (-4π ~ 4π)
    Byte 2-3: 当前角速度 [0-65535] 对应 (-30rad/s ~ 30rad/s)  
    Byte 4-5: 当前扭矩 [0-65535] 对应 (-12Nm ~ 12Nm)
    Byte 6-7: 当前温度: Temp (摄氏度)*10
    """
    if len(data) < 8:
        return None
    
    # 解析角度
    angle_raw = (data[0] << 8) | data[1]
    angle = (angle_raw / 65535.0) * (4 * pai - (-4 * pai)) + (-4 * pai)
    
    # 解析角速度
    velocity_raw = (data[2] << 8) | data[3]
    velocity = (velocity_raw / 65535.0) * (V_MAX - V_MIN) + V_MIN
    
    # 解析扭矩
    torque_raw = (data[4] << 8) | data[5]
    torque = (torque_raw / 65535.0) * (P_MAX - P_MIN) + P_MIN
    
    # 解析温度
    temp_raw = (data[6] << 8) | data[7]
    temperature = temp_raw / 10.0
    
    return {
        'angle_raw': angle_raw,
        'angle_rad': angle,
        'angle_deg': angle * 180 / pai,
        'velocity_raw': velocity_raw,
        'velocity_rad_s': velocity,
        'torque_raw': torque_raw,
        'torque_nm': torque,
        'temp_raw': temp_raw,
        'temperature_c': temperature
    }

#**************************************************************************
# 解析仲裁ID获取通信信息
#**************************************************************************
def parse_arbitration_id(arb_id):
    """
    解析29位仲裁ID:
    Bit 28-24: 通信类型
    Bit 23-8: 数据区域2
    Bit 7-0: 目标地址
    """
    comm_type = (arb_id >> 24) & 0x1F
    data_area2 = (arb_id >> 8) & 0xFFFF
    target_addr = arb_id & 0xFF
    
    return {
        'comm_type': comm_type,
        'data_area2': data_area2,
        'target_addr': target_addr
    }

#**************************************************************************
# 读取电机参数 (通信类型17) - 改进版本
#**************************************************************************
def read_parameter(bus, motorID, param_index, retries=2):
    """
    读取单个参数，带重试机制
    """
    for attempt in range(retries):
        arbitration_id = 0x1100fd00  # 通信类型17 + 主机ID
        arbitration_id = arbitration_id + motorID
        
        # 准备数据: index(2字节) + 00(2字节) + 00(4字节)
        data_s = [0 for i in range(8)]
        data_s[0] = (param_index >> 8) & 0xFF
        data_s[1] = param_index & 0xFF
        
        param_name = PARAM_INDEXES.get(param_index, f"0x{param_index:04X}")
        print(f"\n=== 读取电机 {motorID} 参数 {param_name} (尝试 {attempt + 1}/{retries}) ===")
        
        (state, rx_data) = send_extended_frame_main(bus, arbitration_id, data_s, 1)
        
        if state == 0 and len(rx_data) >= 8:
            # 检查是否是有效的响应
            arb_info = parse_arbitration_id(arbitration_id)
            if arb_info['comm_type'] == 17:  # 参数读取响应
                # 解析响应数据
                rx_index = (rx_data[0] << 8) | rx_data[1]
                if rx_index == param_index:  # 确认是我们请求的参数
                    try:
                        param_value = struct.unpack('<f', bytes(rx_data[4:8]))[0]  # 小端序float
                        print(f"参数 {param_name}: {param_value}")
                        return param_value
                    except:
                        print(f"解析参数 {param_name} 值失败")
        
        print(f"读取参数 {param_name} 失败")
        time.sleep(0.1)  # 短暂延迟后重试
    
    return None

#**************************************************************************
# 写入电机参数 (通信类型18) - 改进版本
#**************************************************************************
def write_parameter(bus, motorID, param_index, param_value, retries=2):
    """
    写入单个参数，带重试机制
    """
    for attempt in range(retries):
        arbitration_id = 0x1200fd00  # 通信类型18 + 主机ID
        arbitration_id = arbitration_id + motorID
        
        # 准备数据: index(2字节) + 00(2字节) + 参数值(4字节)
        data_s = [0 for i in range(8)]
        data_s[0] = (param_index >> 8) & 0xFF
        data_s[1] = param_index & 0xFF
        
        # 将float转换为4字节
        param_bytes = struct.pack('<f', param_value)
        data_s[4:8] = param_bytes
        
        param_name = PARAM_INDEXES.get(param_index, f"0x{param_index:04X}")
        print(f"\n=== 写入电机 {motorID} 参数 {param_name} = {param_value} (尝试 {attempt + 1}/{retries}) ===")
        
        (state, rx_data) = send_extended_frame_main(bus, arbitration_id, data_s, 1)
        
        if state == 0:
            print(f"写入参数 {param_name} 成功")
            return True
        else:
            print(f"写入参数 {param_name} 失败，重试...")
            time.sleep(0.1)
    
    return False

#**************************************************************************
# 设置电机到位置模式并移动到指定位置
#**************************************************************************
def set_position_mode_and_move(bus, motorID, target_position_rad, speed_limit=5.0, current_limit=5.0, kp=30.0):
    """
    设置电机到位置模式并移动到指定位置
    """
    print(f"\n{'='*60}")
    print(f"设置电机 {motorID} 到位置模式")
    print(f"{'='*60}")
    
    # 1. 设置运行模式为位置模式 (1 = 位置模式)
    if not write_parameter(bus, motorID, 0x7005, 1.0):  # run_mode = 1 (位置模式)
        print("设置位置模式失败!")
        return False
    
    time.sleep(0.5)
    
    # 2. 设置位置环KP
    if not write_parameter(bus, motorID, 0x701E, kp):  # loc_kp
        print("设置位置环KP失败!")
    
    time.sleep(0.5)
    
    # 3. 设置速度限制
    if not write_parameter(bus, motorID, 0x7017, speed_limit):  # limit_spd
        print("设置速度限制失败!")
    
    time.sleep(0.5)
    
    # 4. 设置电流限制
    if not write_parameter(bus, motorID, 0x7018, current_limit):  # limit_cur
        print("设置电流限制失败!")
    
    time.sleep(0.5)
    
    # 5. 读取当前旋转圈数
    rotation = read_parameter(bus, motorID, 0x701D)  # rotation
    if rotation is not None:
        print(f"当前旋转圈数: {rotation}")
    
    time.sleep(0.5)
    
    # 6. 设置目标位置
    print(f"\n设置目标位置: {target_position_rad} rad")
    if not write_parameter(bus, motorID, 0x7016, target_position_rad):  # loc_ref
        print("设置目标位置失败!")
        return False
    
    print(f"位置命令发送完成，等待电机移动...")
    return True

#**************************************************************************
# 连续发送运动命令
#**************************************************************************
def continuous_motion_control(bus, motorID, duration=10, interval=0.1):
    """
    连续发送运动命令，用于测试电机响应
    """
    print(f"\n开始连续运动控制，持续时间: {duration}秒")
    start_time = time.time()
    count = 0
    
    while time.time() - start_time < duration:
        # 发送运动命令 - 小幅摆动
        import math
        angle = 0.5 * math.sin(count * 0.5)  # 正弦波运动
        leg_set_motion_parameter_L(bus, motorID, 0, angle, 2.0, 50.0, 1.0)
        
        count += 1
        time.sleep(interval)
    
    print(f"连续运动控制完成，共发送 {count} 个命令")

#**************************************************************************
# 监听并解析电机反馈
#**************************************************************************
def listen_and_parse_feedback(bus, duration=5):
    """
    监听电机反馈消息并解析
    """
    print(f"\n=== 开始监听电机反馈 ({duration}秒) ===")
    start_time = time.time()
    message_count = 0
    
    while time.time() - start_time < duration:
        try:
            message = bus.recv(0.1)  # 非阻塞接收
            if message is not None:
                message_count += 1
                arb_info = parse_arbitration_id(message.arbitration_id)
                comm_type = arb_info['comm_type']
                
                print(f"\n--- 消息 {message_count} ---")
                print(f"仲裁ID: 0x{message.arbitration_id:08X}")
                print(f"通信类型: {comm_type}")
                print(f"数据: {[hex(x) for x in message.data]}")
                
                # 解析不同类型的消息
                if comm_type == 2:  # 电机反馈数据
                    feedback = parse_motor_feedback(message.data)
                    if feedback:
                        print("电机反馈数据:")
                        print(f"  角度: {feedback['angle_rad']:.3f} rad ({feedback['angle_deg']:.1f}°)")
                        print(f"  速度: {feedback['velocity_rad_s']:.3f} rad/s")
                        print(f"  扭矩: {feedback['torque_nm']:.3f} Nm")
                        print(f"  温度: {feedback['temperature_c']:.1f}°C")
                
                elif comm_type == 17:  # 参数读取响应
                    if len(message.data) >= 8:
                        param_index = (message.data[0] << 8) | message.data[1]
                        param_name = PARAM_INDEXES.get(param_index, f"0x{param_index:04X}")
                        try:
                            param_value = struct.unpack('<f', bytes(message.data[4:8]))[0]
                            print(f"参数响应 - {param_name}: {param_value}")
                        except:
                            print(f"参数响应 - {param_name}: 解析失败")
                
                elif comm_type == 18:  # 参数写入响应
                    print("参数写入确认")
                    
        except Exception as e:
            print(f"接收时出错: {e}")
    
    print(f"\n监听完成，共收到 {message_count} 条消息")

#**************************************************************************
# 设置电机零角度功能函数
#**************************************************************************
def set_motor_angle_zero(bus, motorID):
    arbitration_id = 0x0600fd00 + motorID
    data_s = [0x01] + [0 for i in range(7)] 

    print(f"\n=== 设置电机 {motorID} 零角度 ===")
    (state, rx_data) = send_extended_frame_main(bus, arbitration_id, data_s, 1)
    
    if state == 0:
        print(f"电机 {motorID} 设置零角度 OK")
        # 解析反馈数据
        if rx_data != [0 for i in range(8)]:  # 如果不是全零数据
            feedback = parse_motor_feedback(rx_data)
            if feedback:
                print(f"当前角度: {feedback['angle_rad']:.3f} rad")
    else:
        print(f"电机 {motorID} 设置零角度 ERROR")

    return state

#**************************************************************************
# 设置电机模式: 运控模式    
#**************************************************************************
def set_motion_mode(bus, motorID):  
    arbitration_id = 0x1200fd00 + motorID
    data_s = [0 for i in range(8)] 

    print(f"\n=== 设置电机 {motorID} 运控模式 ===")
    (state, rx_data) = send_extended_frame_main(bus, arbitration_id, data_s, 1)
    
    if state == 0:
        print(f"电机 {motorID} 运控模式 OK")
    else:
        print(f"电机 {motorID} 运控模式 ERROR")

    return state

#**************************************************************************
# 设置电机模式使能
#**************************************************************************
def set_motion_enable(bus, motorID):
    arbitration_id = 0x0300fd00 + motorID
    data_s = [0 for i in range(8)] 

    print(f"\n=== 设置电机 {motorID} 使能 ===")
    (state, rx_data) = send_extended_frame_main(bus, arbitration_id, data_s, 1)
    
    if state == 0:
        print(f"电机 {motorID} 使能 OK")
    else:
        print(f"电机 {motorID} 使能 ERROR")

    return state

#**************************************************************************
# 有符号浮点型转十六进制(0~65536)无符号型
#**************************************************************************
def float_to_uint16(float_data, float_data_min, float_data_max):
    if float_data > float_data_max:
        float_data_s = float_data_max
    elif float_data < float_data_min:
        float_data_s = float_data_min
    else:
        float_data_s = float_data
    
    return int((float_data_s - float_data_min) / (float_data_max - float_data_min) * 65535)

#**************************************************************************
# 设置电机运控参数
#**************************************************************************
def leg_set_motion_parameter_L(bus, motorID, torque, radian, speed, KP, KD):
    data_s = [0 for i in range(8)] 

    # 转化扭矩参数
    data_int16 = (float_to_uint16(torque, P_MIN, P_MAX)) << 8 
    arbitration_id = 0x01000000 | data_int16 | motorID

    # 转化角度(弧度)参数载入
    data_int16 = (float_to_uint16(radian, T_MIN, T_MAX))
    data_s[0] = data_int16 >> 8
    data_s[1] = data_int16 & 0x00ff

    # 转化载入速度参数
    data_int16 = (float_to_uint16(speed, V_MIN, V_MAX))
    data_s[2] = data_int16 >> 8
    data_s[3] = data_int16 & 0x00ff

    # 转化载入KP参数
    data_int16 = (float_to_uint16(KP, KP_MIN, KP_MAX))
    data_s[4] = data_int16 >> 8
    data_s[5] = data_int16 & 0x00ff  

    # 转化载入KD参数
    data_int16 = (float_to_uint16(KD, KD_MIN, KD_MAX))
    data_s[6] = data_int16 >> 8
    data_s[7] = data_int16 & 0x00ff 

    print(f"设置运动参数 - 扭矩: {torque} Nm, 角度: {radian:.2f} rad, 速度: {speed} rad/s")
    
    (state, rx_data) = send_extended_frame_main(bus, arbitration_id, data_s, 0)  # 不等待回复
    
    return state

#**************************************************************************
# 测试读取多个参数
#**************************************************************************
def test_read_parameters(bus, motorID):
    """
    测试读取多个有用的参数
    """
    print(f"\n{'='*50}")
    print("测试读取电机参数")
    print(f"{'='*50}")
    
    # 读取机械位置
    mechPos = read_parameter(bus, motorID, 0x7019)  # mechPos
    
    # 读取总线电压
    VBUS = read_parameter(bus, motorID, 0x701C)  # VBUS
    
    # 读取负载端速度
    mechVel = read_parameter(bus, motorID, 0x701B)  # mechVel
    
    # 读取运行模式
    run_mode = read_parameter(bus, motorID, 0x7005)  # run_mode
    
    # 读取旋转圈数
    rotation = read_parameter(bus, motorID, 0x701D)  # rotation
    
    return {
        'mechPos': mechPos,
        'VBUS': VBUS,
        'mechVel': mechVel,
        'run_mode': run_mode,
        'rotation': rotation
    }

############主函数############
def main():  
    print("初始化 CAN 总线...")
    bus = can.interface.Bus(bustype='socketcan', channel='can0', bitrate=1000000)  

    try:
        # 只控制一个电机，发送一个命令
        motorID = 0x01
        
        print("\n" + "="*50)
        print("开始初始化电机...")
        print("="*50)
        
        # 测试读取初始参数
        initial_params = test_read_parameters(bus, motorID)
        time.sleep(1)
        
        #------1：将电机机械角度归零
        set_motor_angle_zero(bus, motorID)
        time.sleep(2)
        
        # #------2：将电机设置为运控模式
        # set_motion_mode(bus, motorID)
        # time.sleep(2)
        
        # #------3：将电机设置为运控模式使能
        # set_motion_enable(bus, motorID)
        # time.sleep(2)

        # 再次读取参数查看状态变化
        test_read_parameters(bus, motorID)
        time.sleep(1)

        print("\n" + "="*50)
        print("测试运控模式")
        print("="*50)
        
        # # 测试连续运动控制
        # continuous_motion_control(bus, motorID, duration=3, interval=0.2)
        
        # # 监听反馈
        # listen_and_parse_feedback(bus, duration=3)
        
        print("\n" + "="*50)
        print("测试位置模式")
        print("="*50)
        
        # # 测试位置模式
        # # 移动到1弧度位置
        set_position_mode_and_move(bus, motorID, target_position_rad=0.2, speed_limit=2.0)
        time.sleep(3)
        
        # # 监听移动过程中的反馈
        listen_and_parse_feedback(bus, duration=3)
        
        # 读取移动后的参数
        test_read_parameters(bus, motorID)
        time.sleep(1)
        
        # 移回0弧度位置
        # set_position_mode_and_move(bus, motorID, target_position_rad=0.0, speed_limit=2.0)
        # time.sleep(3)
        
        # 最终参数读取
        final_params = test_read_parameters(bus, motorID)

    except KeyboardInterrupt:  
        print("\n按下 Ctrl+C, 结束----")
    except Exception as e:
        print(f"发生错误: {e}")
        import traceback
        traceback.print_exc()
    finally:
        print("关闭 CAN 总线...")
        bus.shutdown() 

if __name__ == "__main__":  
    main()