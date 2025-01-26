
#!/usr/bin/python3
# -*- coding: UTF-8 -*-  



#****************************************************说明：*********************************************************
# 1：如果你的上位机未安装CAN库，按照连接帖子安装：# CAN库的安装 https://blog.csdn.net/molangmolang/article/details/140389153

# 2：如果你需要使用TTL串口或485总线串口  需要设置串口读写权限 权限设置命令 sudo chmod 777 /dev/ttyUSB0

# 3：如果你需要使用CAN总线接口请按照第1步安装 CAN 库 安装好后 执行下面命令 打开 CAN 通道 会看到对应板子上 两个红色LED灯发光说明 CAN通道已经打开

'''
#  以下是打开 每路 CAN 总线接口的命令，用哪路 就 执行对于命令，执行后 板子上对应CAN通道的两个LED会长亮说明 总线打开成功

#  注意 第二行的 波特率 要设置跟电机设置的一致

sudo ip link set up can0
sudo ip link set can0 type can bitrate 500000 loopback off
sudo ip link set up can0


sudo ip link set down can1
sudo ip link set can1 type can bitrate 500000 loopback off
sudo ip link set up can1


sudo ip link set down can2
sudo ip link set can2 type can bitrate 500000 loopback off
sudo ip link set up can2


sudo ip link set down can3
sudo ip link set can3 type can bitrate 500000 loopback off
sudo ip link set up can3


'''




import can  

import time
import struct



#=========================发送标准CAN帧(主路线)=====================
# bus:                   CAN总线对象。
# arbitration_id:        11位仲裁ID。
# data:                  CAN数据字段，长度为0-8字节。
# block_receive:         是否阻塞接收 0否(不管接收) 1是(等待接收)
# rx_data:               接收数据数组
#================================================================
def send_extended_frame_main(bus, arbitration_id , data , block_receive):  

    state = 0
    rx_data = [0 for i in range(8)]  
    message = can.Message(arbitration_id=arbitration_id,data=data,is_extended_id=False,is_remote_frame=False)  

    # 发送CAN消息  
    bus.send(message)  

    if block_receive == 1:
        
        time_s=time.time()
        while (1):  
            message_ou = bus.recv(0.1)  
            if message_ou is not None:  
                rx_data = message_ou.data
                break

            elif time.time() - time_s >1:
                print("仲裁号: %d 未受到回馈-error" % arbitration_id)
                state = 1
                break


    return(state,rx_data) 



#**************************************************************************
# 电机校准  
# bus:          总线端口号# bus:                   CAN总线对象。
# motorID :     电机ID
# state 0:      数据正常接收 1:通讯故障
#**************************************************************************

def calibration(bus,motorID):  
    CMDID = 0x07
    arbitration_id = motorID<<5
    arbitration_id = arbitration_id + CMDID
    print('---CAN-ID----', hex(arbitration_id))
    data_s = [0 for i in range(8)] 
    data_s[0] = 0x04
    print('---data----', data_s)

    message = can.Message(arbitration_id=arbitration_id,data=data_s,is_extended_id=False,is_remote_frame=False)
    bus.send(message)

    time.sleep(0.1)

    CMDID = 0x07
    arbitration_id = motorID<<5
    arbitration_id = arbitration_id + CMDID
    print('---CAN-ID----', hex(arbitration_id))
    data_s = [0 for i in range(8)] 
    data_s[0] = 0x07
    print('---data----', data_s)

    message = can.Message(arbitration_id=arbitration_id,data=data_s,is_extended_id=False,is_remote_frame=False)
    bus.send(message)


    '''
    time_s=time.time()
    while True:  
        message_ou = bus.recv(0.1)  
        if message_ou is not None:  
            rx_data = message_ou.data
            break

        elif time.time() - time_s >1:
            print("CAN-ID: %d 未受到回馈-error" % arbitration_id)
            state = 1
            break
    '''




#**************************************************************************
# 设置电机模式:   运控模式(位置控制模式)    
# bus:          总线端口号# bus:                   CAN总线对象。
# motorID :     电机ID
# state 0:      数据正常接收 1:通讯故障
#**************************************************************************
def set_controller_mode(bus,motorID):  
    CMDID = 0x0B
    arbitration_id = motorID<<5
    arbitration_id = arbitration_id + CMDID
    print('---CAN-ID----', hex(arbitration_id))

    data_s = [0 for i in range(8)] 
    data_s[0] = 0x03
    data_s[4] = 0x03

    print('---data----', data_s)

    message = can.Message(arbitration_id=arbitration_id,data=data_s,is_extended_id=False,is_remote_frame=False) 
    bus.send(message)

    time_s=time.time()
    while True:  
        message_ou = bus.recv(0.1)  
        if message_ou is not None:  
            rx_data = message_ou.data
            break

        elif time.time() - time_s >1:
            print("CAN-ID: %d 未受到回馈-error" % arbitration_id)
            state = 1
            break




#**************************************************************************
# 设置进入闭环状态
# bus:         总线端口号
# motorID :    电机ID
# state 0:     数据正常接收 1:通讯故障
#**************************************************************************
def set_axis_state(bus,motorID):  

    CMDID = 0x07
    arbitration_id = motorID<<5
    arbitration_id = arbitration_id + CMDID
    print('---CAN-ID----', hex(arbitration_id))

    data_s = [0 for i in range(8)] 
    data_s[0] = 0x08

    print('---data----', data_s)
    message = can.Message(arbitration_id=arbitration_id,data=data_s,is_extended_id=False,is_remote_frame=False) 
    bus.send(message)

    time_s=time.time()
    while True:  
        message_ou = bus.recv(0.1)  
        if message_ou is not None:  
            rx_data = message_ou.data
            break

        elif time.time() - time_s >1:
            print("CAN-ID: %d 未受到回馈-error" % arbitration_id)
            state = 1
            break



#**************************************************************************
# 左腿设置电机运控参数(速度模式)
# bus:      总线端口号
# motorID:  电机ID号 
# speed:    设定速度  
# torque:   设定扭矩        
#**************************************************************************
def set_input_vel(bus,motorID,speed,torque):

    CMDID = 0x0d
    arbitration_id = motorID<<5
    arbitration_id = arbitration_id + CMDID
    print('---CAN-ID----', hex(arbitration_id))

    data_s = [0 for i in range(8)] 
    data_s[0] = 0x00
    data_s[1] = 0x00
    data_s[2] = 0x20
    data_s[3] = 0x41
    data_s[4] = 0x00
    data_s[5] = 0x00
    data_s[6] = 0x00
    data_s[7] = 0x00


    print('---data----', data_s)
    message = can.Message(arbitration_id=arbitration_id,data=data_s,is_extended_id=False,is_remote_frame=False) 
    bus.send(message)

    time_s=time.time()
    while True:  
        message_ou = bus.recv(0.1)  
        if message_ou is not None:  
            rx_data = message_ou.data
            break

        elif time.time() - time_s >1:
            print("CAN-ID: %d 未受到回馈-error" % arbitration_id)
            state = 1
            break




'''
#**************************************************************************
#有符号浮点型转四位十六进制(输出小端模式：低在左高在右) 适合单个参数写函数
#**************************************************************************
def float_to_P4hex(float_data):
    # 将十进制浮点数转换为十六进制浮点数
    byte_representation = struct.pack('f', float_data)
    return byte_representation


#**************************************************************************
# 四位十六进制转有符号浮点型转(输入小端模式：低在左高在右) 适合单个参数写函数
#**************************************************************************
def P4hex_to_float(P4hex_data):
    bytes_obj = P4hex_data.to_bytes(4, byteorder='big') 
    float_value = struct.unpack('f', bytes_obj)[0]

    return float_value


#**************************************************************************
# 有符号浮点型转十六进制(0~65536)无符号型      适合运控模式参赛转换
# float_data:        要转换的有符号浮点数据
# float_data_min：   下限范围
# float_data_max：   上限范围
# return :           输出0~65535的int型数据
#**************************************************************************
def float_to_uint16(float_data,float_data_min,float_data_max):

    if float_data > float_data_max:
        float_data_s = float_data_max
    elif float_data < float_data_min:
        float_data_s = float_data_min
    else:
        float_data_s = float_data
    
    return int((float_data_s - float_data_min)/(float_data_max - float_data_min) * 65535)


#**************************************************************************
# 十六进制(0~65535)无符号型转有符号浮点型  适合运控模式参赛转换
# uint16_data:       要转换的无符号型数据
# float_data_min：   下限范围
# float_data_max：   上限范围
# return :           输出上下限范围内的比例 有符合浮点型数据
#**************************************************************************
def uint16_to_float(uint16_data,float_data_min,float_data_max):
   return float((uint16_data - 32767)/65535) * (float_data_max - float_data_min)

'''







############主函数############
############主函数############

def main():    
    bus = can.interface.Bus(interface='socketcan', channel='can0', bitrate=500000) 
    #bus = can.interface.Bus(interface='socketcan', channel='can0', bitrate=1000000) 
    
    motorID1 = 0x01                      
    calibration(bus,motorID1)             # 执行校准

    motorID1 = 0x02                       
    calibration(bus,motorID1)             # 执行校准



    # 在退出时确保资源被释放  
    bus.shutdown() 


  
if __name__ == "__main__":  
    main()

