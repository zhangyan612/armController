import time
import serial

class MotorController:
    def __init__(self, port='COM9', baudrate=115200):
        self.ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=1
        )
        if self.ser.isOpen():
            print("Serial port is open")
        else:
            raise IOError("Failed to open serial port")

    def construct_message(self, id_value, pwm_value, time_value):
        if not (isinstance(pwm_value, int) and isinstance(time_value, int)):
            raise ValueError("PWM and Time values must be integers.")
        pwm_str = f"{pwm_value:04}"
        time_str = f"{time_value:05}"
        return f"#{id_value}#{pwm_str}#{time_str}#".encode('utf-8')

    def construct_multi_message(self, motor_commands):
        """
        构造同时控制多个电机的消息
        :param motor_commands: 列表，每个元素是(id_value, pwm_value, time_value)的元组
        :return: 编码后的消息字节串
        """
        message_parts = []
        for id_value, pwm_value, time_value in motor_commands:
            if not (isinstance(pwm_value, int) and isinstance(time_value, int)):
                raise ValueError("PWM and Time values must be integers.")
            pwm_str = f"{pwm_value:04}"
            time_str = f"{time_value:05}"
            message_parts.append(f"#{id_value}#{pwm_str}#{time_str}#")
        
        # 将所有电机命令合并为一个消息，用分号分隔
        full_message = ";".join(message_parts)
        return full_message.encode('utf-8')

    def construct_stop_command(self, motor_ids):
        """
        构造停止电机的命令（关键修改）
        :param motor_ids: 要停止的电机ID列表，如 [1, 2, 3]
        :return: 编码后的消息字节串
        """
        # 移除前导零，发送纯数字ID
        id_list = ",".join(str(id) for id in motor_ids)
        return f"#STOP#{id_list}#".encode('utf-8')
    
    def move_motor(self, id, pwm, duration):
        limitPWMList = {'03', '06'}
        if id not in limitPWMList:
            pwm = max(1580, min(pwm, 2420))

        msg = self.construct_message(id, pwm, duration)
        print(f"Sending: {msg}")
        self.ser.write(msg)
        time.sleep(0.01)

    def move_multi_motor(self, idList, pwm, duration):
        # 限制特定电机的PWM值 因为电压不同
        # limitPWMList = {'03', '06'}
        # for id in idList:
        #     if id not in limitPWMList:
        #         pwm = max(1580, min(pwm, 2420))

        message_parts = []
        for id in idList:
            if not (isinstance(pwm, int) and isinstance(pwm, int)):
                raise ValueError("PWM and Time values must be integers.")
            pwm_str = f"{pwm:04}"
            time_str = f"{duration:05}"
            message_parts.append(f"#{id}#{pwm_str}#{time_str}#")
        
        # 将所有电机命令合并为一个消息，用分号分隔
        full_message = ";".join(message_parts)
        msg = full_message.encode('utf-8')
        print(f"Sending: {msg}")
        self.ser.write(msg)
        # time.sleep(0.01)

    def stop_motor(self, idList):
        """
        停止指定电机的运动
        :param idList: 要停止的电机ID列表
        """
        stop_command = self.construct_stop_command(idList)
        print(f"Sending stop command: {stop_command}")
        self.ser.write(stop_command)
        time.sleep(0.01)

    def sendMsg(self, command):
        self.ser.write(command)

    def close(self):
        self.ser.close()

