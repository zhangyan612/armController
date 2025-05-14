#!/usr/bin/python3
# encoding: utf-8
import serial
import time
import ctypes
import sys
import serial.tools.list_ports

LOBOT_SERVO_FRAME_HEADER         = 0x55
LOBOT_SERVO_MOVE_TIME_WRITE      = 1
LOBOT_SERVO_MOVE_TIME_READ       = 2
LOBOT_SERVO_MOVE_TIME_WAIT_WRITE = 7
LOBOT_SERVO_MOVE_TIME_WAIT_READ  = 8
LOBOT_SERVO_MOVE_START           = 11
LOBOT_SERVO_MOVE_STOP            = 12
LOBOT_SERVO_ID_WRITE             = 13
LOBOT_SERVO_ID_READ              = 14
LOBOT_SERVO_ANGLE_OFFSET_ADJUST  = 17
LOBOT_SERVO_ANGLE_OFFSET_WRITE   = 18
LOBOT_SERVO_ANGLE_OFFSET_READ    = 19
LOBOT_SERVO_ANGLE_LIMIT_WRITE    = 20
LOBOT_SERVO_ANGLE_LIMIT_READ     = 21
LOBOT_SERVO_VIN_LIMIT_WRITE      = 22
LOBOT_SERVO_VIN_LIMIT_READ       = 23
LOBOT_SERVO_TEMP_MAX_LIMIT_WRITE = 24
LOBOT_SERVO_TEMP_MAX_LIMIT_READ  = 25
LOBOT_SERVO_TEMP_READ            = 26
LOBOT_SERVO_VIN_READ             = 27
LOBOT_SERVO_POS_READ             = 28
LOBOT_SERVO_OR_MOTOR_MODE_WRITE  = 29
LOBOT_SERVO_OR_MOTOR_MODE_READ   = 30
LOBOT_SERVO_LOAD_OR_UNLOAD_WRITE = 31
LOBOT_SERVO_LOAD_OR_UNLOAD_READ  = 32
LOBOT_SERVO_LED_CTRL_WRITE       = 33
LOBOT_SERVO_LED_CTRL_READ        = 34
LOBOT_SERVO_LED_ERROR_WRITE      = 35
LOBOT_SERVO_LED_ERROR_READ       = 36

def getServoPort():
        ports = serial.tools.list_ports.comports()
        for port, desc, hwid in sorted(ports):
                if 'USB-SERIAL CH340' in desc:
                        return port
        print('Error: Servo is not connected to any port')
        return None

port = getServoPort()

serialHandle = serial.Serial(port, 115200)  # 初始化串口， 波特率为115200

def checksum(buf):
    # 计算校验和
    sum = 0x00
    for b in buf:  # 求和
        sum += b
    sum = sum - 0x55 - 0x55  # 去掉命令开头的两个 0x55
    sum = ~sum  # 取反
    return sum & 0xff

def serial_serro_wirte_cmd(id=None, w_cmd=None, dat1=None, dat2=None):
    '''
    写指令
    :param id: 舵机id=1
    :param w_cmd: 命令 = 1
    :param dat1:
    :param dat2:
    :return:
    '''
    #portWrite()
    buf = bytearray(b'\x55\x55')  # 帧头
    buf.append(id)
    
    # 指令长度
    if dat1 is None and dat2 is None:
        buf.append(3)
    elif dat1 is not None and dat2 is None:
        buf.append(4)
    elif dat1 is not None and dat2 is not None:
        buf.append(7)
        
    # 指令
    buf.append(w_cmd) 
    
    # 写数据
    if dat1 is None and dat2 is None:
        pass
    elif dat1 is not None and dat2 is None:
        buf.append(dat1 & 0xff)  # 偏差
    elif dat1 is not None and dat2 is not None:
    ## 限制5号舵机的转动范围
        # if id == 5:
        #     if dat1 >= 875:
        #         dat1 = 875
        #     elif dat1 <= 125:
        #         dat1 = 125
                
        buf.extend([(0xff & dat1), (0xff & (dat1 >> 8))])  # 分低8位 高8位 放入缓存
        buf.extend([(0xff & dat2), (0xff & (dat2 >> 8))])  # 分低8位 高8位 放入缓存
        
    # 校验和
    buf.append(checksum(buf))
    
    # 发送
    serialHandle.write(buf)  

def serial_servo_read_cmd(id=None, r_cmd=None):
    '''
    发送读取命令
    :param id:
    :param r_cmd:
    :param dat:
    :return:
    '''
    #portWrite()
    buf = bytearray(b'\x55\x55')  # 帧头
    buf.append(id)
    buf.append(3)  # 指令长度
    buf.append(r_cmd)  # 指令
    buf.append(checksum(buf))  # 校验和
    serialHandle.write(buf)  # 发送
    # time.sleep(0.00034)

def serial_servo_get_rmsg(cmd):
    '''
    # 获取指定读取命令的数据
    :param cmd: 读取命令
    :return: 数据
    '''    
    serialHandle.flushInput()  # 清空接收缓存
    # portRead()  # 将单线串口配置为输入
    time.sleep(0.005)  # 稍作延时，等待接收完毕 重要
    count = serialHandle.inWaiting()    # 获取接收缓存中的字节数
    if count != 0:  # 如果接收到的数据不空
        recv_data = serialHandle.read(count)  # 读取接收到的数据
        try:
            if recv_data[0] == 0x55 and recv_data[1] == 0x55 and recv_data[4] == cmd:
                dat_len = recv_data[3]
                serialHandle.flushInput()  # 清空接收缓存
                if dat_len == 4:
                    return recv_data[5]
                elif dat_len == 5:
                    pos = 0xffff & (recv_data[5] | (0xff00 & (recv_data[6] << 8)))
                    return ctypes.c_int16(pos).value
                elif dat_len == 7:
                    pos1 = 0xffff & (recv_data[5] | (0xff00 & (recv_data[6] << 8)))
                    pos2 = 0xffff & (recv_data[7] | (0xff00 & (recv_data[8] << 8)))
                    return ctypes.c_int16(pos1).value, ctypes.c_int16(pos2).value
            else:
                return None
        except BaseException as e:
            return None
            print(e)
    else:
        serialHandle.flushInput()  # 清空接收缓存
        return None
    
def portRest():
    time.sleep(0.1)
    serialHandle.close()
    serialHandle.open()
    time.sleep(0.1)
		
def move_servo(position, id=1, time=500):
    # position: 412 - 620 physical limit
    # time it takes to finish the movement
    if position < 412:
        position = 412
    if position > 990:
        position = 990
    if serialHandle.is_open == False:
        serialHandle.open()
    serial_serro_wirte_cmd(id,1,position,time)
    serialHandle.close()

def move_command(command, id=1):
    if command == 'open':
        move_servo(412, id)
    if command == 'close':
        move_servo(1000, id)

if __name__ == "__main__":
    move_command('open')
    time.sleep(1)
    move_command('close')

    # while True:
    #     try:
    #         # serial_serro_wirte_cmd(1,1,0,2000) #发送命令 参数1 舵机id=1, 参数2 命令 = 1, 参数3 位置 = 0, 参数4 时间 = 1000ms
    #         # time.sleep(1)
    #         # serial_serro_wirte_cmd(1,1,1000,2000)
    #         # time.sleep(1)
    #         move_servo(1, 500, 2000)
    #         move_servo(1, 1000, 2000)
    #     except Exception as e:
    #         print(e)
    #         break
