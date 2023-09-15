#!/usr/bin/python3
# encoding: utf-8
# 配置串口舵机的参数
# 每次只能配置一个舵机，且扩展板只能连接一个舵机，既是一个舵机一个舵机配置参数
from servoControl import *

timeout_count = 200

def serial_servo_set_id(oldid, newid):
    """
    配置舵机id号, 出厂默认为1
    :param oldid: 原来的id 出厂默认为1
    :param newid: 新的id
    """
    serial_serro_wirte_cmd(oldid, LOBOT_SERVO_ID_WRITE, newid)

def serial_servo_read_id(id=None):
    """
    读取串口舵机id
    :param id: 默认为空
    :return: 返回舵机id
    """
    global timeout_count
    
    count = 0
    while True:
        if id is None:  # 总线上只能有一个舵机
            serial_servo_read_cmd(0xfe, LOBOT_SERVO_ID_READ)
        else:
            serial_servo_read_cmd(id, LOBOT_SERVO_ID_READ)
        # 获取内容
        msg = serial_servo_get_rmsg(LOBOT_SERVO_ID_READ)
        if msg is not None:
            count = 0
            return msg
        else:
            count += 1
        if count >= timeout_count:
            count = 0
            return None

def serial_servo_stop(id=None):
    '''
    停止舵机运行
    :param id:
    :return:
    '''
    serial_serro_wirte_cmd(id, LOBOT_SERVO_MOVE_STOP)

def serial_servo_set_deviation(id, d=0):
    """
    调整偏差
    :param id: 舵机id
    :param d:  偏差
    """
    serial_serro_wirte_cmd(id, LOBOT_SERVO_ANGLE_OFFSET_ADJUST, d)

def serial_servo_save_deviation(id):
    """
    配置偏差，掉电保护
    :param id: 舵机id
    """
    serial_serro_wirte_cmd(id, LOBOT_SERVO_ANGLE_OFFSET_WRITE)

def serial_servo_read_deviation(id):
    '''
    读取偏差值
    :param id: 舵机号
    :return:
    '''
    # 发送读取偏差指令
    global timeout_count
    count = 0
    while True:
        serial_servo_read_cmd(id, LOBOT_SERVO_ANGLE_OFFSET_READ)
        # 获取
        msg = serial_servo_get_rmsg(LOBOT_SERVO_ANGLE_OFFSET_READ)
        if msg is not None:
            count = 0
            return msg
        else:
            count += 1
        if count >= timeout_count:
            count = 0
            return None

def serial_servo_set_angle_limit(id, low, high):
    '''
    设置舵机转动范围
    :param id:
    :param low:
    :param high:
    :return:
    '''
    serial_serro_wirte_cmd(id, LOBOT_SERVO_ANGLE_LIMIT_WRITE, low, high)

def serial_servo_read_angle_limit(id):
    '''
    读取舵机转动范围
    :param id:
    :return: 返回元祖 0 低位  1 高位
    '''
    global timeout_count
    
    count = 0
    while True:
        serial_servo_read_cmd(id, LOBOT_SERVO_ANGLE_LIMIT_READ)
        msg = serial_servo_get_rmsg(LOBOT_SERVO_ANGLE_LIMIT_READ)
        if msg is not None:
            count = 0
            return msg
        else:
            count += 1
        if count >= timeout_count:
            count = 0
            return None

def serial_servo_set_vin_limit(id, low, high):
    '''
    设置舵机电压范围
    :param id:
    :param low:
    :param high:
    :return:
    '''
    serial_serro_wirte_cmd(id, LOBOT_SERVO_VIN_LIMIT_WRITE, low, high)

def serial_servo_read_vin_limit(id):
    '''
    读取舵机转动范围
    :param id:
    :return: 返回元祖 0 低位  1 高位
    '''
    global timeout_count
    count = 0
    while True:
        serial_servo_read_cmd(id, LOBOT_SERVO_VIN_LIMIT_READ)
        msg = serial_servo_get_rmsg(LOBOT_SERVO_VIN_LIMIT_READ)
        if msg is not None:
            count = 0
            return msg
        else:
            count += 1
        if count >= timeout_count:
            count = 0
            return None

def serial_servo_set_max_temp(id, m_temp):
    '''
    设置舵机最高温度报警
    :param id:
    :param m_temp:
    :return:
    '''
    serial_serro_wirte_cmd(id, LOBOT_SERVO_TEMP_MAX_LIMIT_WRITE, m_temp)

def serial_servo_read_temp_limit(id):
    '''
    读取舵机温度报警范围
    :param id:
    :return:
    '''
    global timeout_count
    
    count = 0
    while True:
        serial_servo_read_cmd(id, LOBOT_SERVO_TEMP_MAX_LIMIT_READ)
        msg = serial_servo_get_rmsg(LOBOT_SERVO_TEMP_MAX_LIMIT_READ)
        if msg is not None:
            count = 0
            return msg
        else:
            count += 1
        if count >= timeout_count:
            count = 0
            return None

def serial_servo_read_pos(id):
    '''
    读取舵机当前位置
    :param id:
    :return:
    '''
    global timeout_count
    
    count = 0
    while True:
        serial_servo_read_cmd(id, LOBOT_SERVO_POS_READ)
        msg = serial_servo_get_rmsg(LOBOT_SERVO_POS_READ)
        if msg is not None:
            count = 0
            return msg
        else:
            count += 1
        if count >= timeout_count:
            count = 0
            return None

def serial_servo_read_temp(id):
    '''
    读取舵机温度
    :param id:
    :return:
    '''
    global timeout_count
    count = 0
    while True:
        serial_servo_read_cmd(id, LOBOT_SERVO_TEMP_READ)
        msg = serial_servo_get_rmsg(LOBOT_SERVO_TEMP_READ)
        if msg is not None:
            count = 0
            return msg
        else:
            count += 1
        if count >= timeout_count:
            count = 0
            return None

def serial_servo_read_vin(id):
    '''
    读取舵机电压
    :param id:
    :return:
    '''
    global timeout_count
    
    count = 0
    while True:
        serial_servo_read_cmd(id, LOBOT_SERVO_VIN_READ)
        msg = serial_servo_get_rmsg(LOBOT_SERVO_VIN_READ)
        if msg is not None:
            count = 0
            return msg
        else:
            count += 1
        if count >= timeout_count:
            count = 0
            return None

def serial_servo_rest_pos(oldid):
    # 舵机清零偏差和P值中位（500）
    serial_servo_set_deviation(oldid, 0)    # 清零偏差
    time.sleep(0.1)
    serial_serro_wirte_cmd(oldid, LOBOT_SERVO_MOVE_TIME_WRITE, 500, 100)    # 中位

##掉电
def serial_servo_load_or_unload_write(id):
    serial_serro_wirte_cmd(id, LOBOT_SERVO_LOAD_OR_UNLOAD_WRITE, 0)

##读取是否掉电
def serial_servo_load_or_unload_read(id):
    global timeout_count
    
    count = 0
    while True:
        serial_servo_read_cmd(id, LOBOT_SERVO_LOAD_OR_UNLOAD_READ)
        msg = serial_servo_get_rmsg(LOBOT_SERVO_LOAD_OR_UNLOAD_READ)
        if msg is not None:
            count = 0
            return msg
        else:
            count += 1
        if count >= 600:
            count = 0
            return None
        
def show_servo_state():
    '''
    显示信息
    :return:
    '''
    oldid = serial_servo_read_id()
    # portRest()
    if oldid is not None:
        print('Current Servo ID:%d' % oldid)
        pos = serial_servo_read_pos(oldid)
        print('Current Servo Position:%d' % pos)
        # portRest()

        now_temp = serial_servo_read_temp(oldid)
        print('Current Servo Temp:%d°' % now_temp)
        # portRest()

        now_vin = serial_servo_read_vin(oldid)
        print('Current Servo Voltage:%dmv' % now_vin)
        # portRest()

        d = serial_servo_read_deviation(oldid)
        print('Current Servo deviation:%d' % ctypes.c_int8(d).value)
        # portRest()

        limit = serial_servo_read_angle_limit(oldid)
        print('Current Servo angle_limit:%d-%d' % (limit[0], limit[1]))
        # portRest()

        vin = serial_servo_read_vin_limit(oldid)
        print('Current Servo vin_limit:%dmv-%dmv' % (vin[0], vin[1]))
        # portRest()

        temp = serial_servo_read_temp_limit(oldid)
        print('Current Servo temp_limit:50°-%d°' % temp)
        # portRest()
    else:
        print('Read id fail')

def read_servo_limits(servoId):
    if serialHandle.is_open == False:
        serialHandle.open()

    d = serial_servo_read_deviation(servoId)
    limit = serial_servo_read_angle_limit(servoId)
    vin = serial_servo_read_vin_limit(servoId)
    temp = serial_servo_read_temp_limit(servoId)
    servoLimits = {
        'deviation': ctypes.c_int8(d).value, 'angle_low': limit[0], 'angle_high': limit[1], 'vin_low': vin[0], 'vin_high': vin[1], 'temp': temp
    }
    serialHandle.close()
    return servoLimits

def read_servo_state(servoId):
    if serialHandle.is_open == False:
        serialHandle.open()
    pos = serial_servo_read_pos(servoId)
    now_temp = serial_servo_read_temp(servoId)
    now_vin = serial_servo_read_vin(servoId)
    servoData = {
        'id': servoId,
        'position': pos,
        'temp': now_temp,
        'voltage': now_vin,
    }
    serialHandle.close()
    return servoData

def servo_state_continuous():
    servoId = serial_servo_read_id()
    previousState = {
        'id': 1,
        'position': 0,
        'temp': 0,
        'voltage': 0
    }
    if servoId is not None:
        while True:
            pos = serial_servo_read_pos(servoId)
            now_temp = serial_servo_read_temp(servoId)
            now_vin = serial_servo_read_vin(servoId)
            servoData = {
                'id': servoId,
                'position': pos,
                'temp': now_temp,
                'voltage': now_vin,
            }
            if servoData['position'] != previousState['position']:
                print(servoData)
                previousState = servoData

            time.sleep(1) # only check every seconds, otherwise will receive too much data

            
if __name__ == '__main__':
    # show_servo_state()
    state = read_servo_state(1)
    print(state)
    # servo_state_continuous()
    print(read_servo_limits(1))