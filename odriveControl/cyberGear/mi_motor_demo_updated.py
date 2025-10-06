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
#设置电机零角度功能函数
#**************************************************************************
def set_motor_angle_zero(bus, motorID):
    arbitration_id = 0x0600fd00 + motorID
    data_s = [0x01] + [0 for i in range(7)] 

    print(f"\n=== 设置电机 {motorID} 零角度 ===")
    (state, rx_data) = send_extended_frame_main(bus, arbitration_id, data_s, 1)

    if state == 0:
        print(f"电机 {motorID} 设置零角度 OK")
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

    print(f"\n=== 设置电机 {motorID} 运动参数 ===")
    print(f"角度: {radian} rad, 速度: {speed} rad/s, KP: {KP}, KD: {KD}")
    
    (state, rx_data) = send_extended_frame_main(bus, arbitration_id, data_s, 0)  # 不等待回复
    
    return state

############主函数############
def main():  
    print("初始化 CAN 总线...")
    bus = can.interface.Bus(bustype='socketcan', channel='can0', bitrate=1000000)  

    try:
        # 只控制一个电机，发送一个命令
        motorID = 6
        
        print("\n" + "="*50)
        print("开始初始化电机...")
        print("="*50)
        
        #------1：将电机机械角度归零
        set_motor_angle_zero(bus, motorID)
        time.sleep(1)  # 等待1秒
        
        #------2：将电机设置为运控模式
        set_motion_mode(bus, motorID)
        time.sleep(1)  # 等待1秒
        
        #------3：将电机设置为运控模式使能
        set_motion_enable(bus, motorID)
        time.sleep(1)  # 等待1秒

        print("\n" + "="*50)
        print("发送单个运动命令...")
        print("="*50)
        
        #------4：发送一个运动命令
        leg_set_motion_parameter_L(bus, motorID, 0, 0.5, 0, 10, 0.5)
        
        print("\n" + "="*50)
        print("命令发送完成!")
        print("="*50)
        
        # 等待一段时间看是否有任何回复
        print("\n等待接收任何回复消息(5秒)...")
        start_time = time.time()
        while time.time() - start_time < 5:
            try:
                message = bus.recv(0.1)  # 非阻塞接收
                if message is not None:
                    print(f"额外接收 <- 仲裁ID: 0x{message.arbitration_id:08X}, 数据: {[hex(x) for x in message.data]}")
            except Exception as e:
                print(f"接收时出错: {e}")

    except KeyboardInterrupt:  
        print("\n按下 Ctrl+C, 结束----")
    except Exception as e:
        print(f"发生错误: {e}")
    finally:
        print("关闭 CAN 总线...")
        bus.shutdown() 

if __name__ == "__main__":  
    main()