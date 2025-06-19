#!/usr/bin/env python
import sys
import time
from scservo_def import *
from protocol_packet_handler import *
from group_sync_write import *
from scservo_sdk import scscl, PortHandler

class FeetechServoController:
    def __init__(self, port='COM10', baudrate=1000000):
        """
        初始化舵机控制器
        :param port: 串口设备路径
        :param baudrate: 波特率
        """
        self.port_handler = PortHandler(port)
        self.servo = scscl(self.port_handler)
        self.min_position = 0  # 最小位置
        self.max_position = 1000  # 最大位置，根据舵机型号调整
        
        if not self.port_handler.openPort():
            print("无法打开串口")
            sys.exit(1)
        
        print(f"成功连接到串口 {port}")
    
    def move_to_position(self, servo_id, position, move_time=0, speed=0):
        """移动指定舵机到指定位置"""
        if position < self.min_position:
            position = self.min_position
        elif position > self.max_position:
            position = self.max_position
        
        # 发送位置指令，返回两个值
        scs_comm_result, scs_error = self.servo.WritePos(servo_id, position, move_time, speed)
        
        if scs_comm_result != COMM_SUCCESS:
            print(f"通信错误: {self.get_txrx_result(scs_comm_result)}")
        if scs_error != 0:
            print(f"舵机 {servo_id} 错误: {self.get_rx_packet_error(scs_error)}")
        
        if scs_comm_result == COMM_SUCCESS and scs_error == 0:
            print(f"舵机 {servo_id} 移动到位置: {position}")
        
        return scs_comm_result, scs_error
    
    def release_torque(self, servo_id):
        """释放扭矩（舵机进入自由状态）"""
        scs_comm_result, scs_error = self.servo.write1ByteTxRx(servo_id, SCSCL_TORQUE_ENABLE, 0)
        
        if scs_comm_result != COMM_SUCCESS:
            print(f"通信错误: {self.get_txrx_result(scs_comm_result)}")
        if scs_error != 0:
            print(f"舵机 {servo_id} 错误: {self.get_rx_packet_error(scs_error)}")
        
        if scs_comm_result == COMM_SUCCESS and scs_error == 0:
            print(f"舵机 {servo_id} 扭矩已释放")
        
        return scs_comm_result, scs_error
    
    def enable_torque(self, servo_id):
        """使能扭矩"""
        scs_comm_result, scs_error = self.servo.write1ByteTxRx(servo_id, SCSCL_TORQUE_ENABLE, 1)
        
        if scs_comm_result != COMM_SUCCESS:
            print(f"通信错误: {self.get_txrx_result(scs_comm_result)}")
        if scs_error != 0:
            print(f"舵机 {servo_id} 错误: {self.get_rx_packet_error(scs_error)}")
        
        if scs_comm_result == COMM_SUCCESS and scs_error == 0:
            print(f"舵机 {servo_id} 扭矩已使能")
        
        return scs_comm_result, scs_error
    
    def read_position(self, servo_id):
        """读取指定舵机的当前位置"""
        position, scs_comm_result, scs_error = self.servo.ReadPos(servo_id)
        
        if scs_comm_result != COMM_SUCCESS:
            print(f"通信错误: {self.get_txrx_result(scs_comm_result)}")
        if scs_error != 0:
            print(f"舵机 {servo_id} 错误: {self.get_rx_packet_error(scs_error)}")
        
        if scs_comm_result == COMM_SUCCESS and scs_error == 0:
            print(f"舵机 {servo_id} 当前位置: {position}")
            return position, scs_comm_result, scs_error
        else:
            return None, scs_comm_result, scs_error
    
    def get_txrx_result(self, result):
        """获取通信结果描述"""
        results = {
            COMM_SUCCESS: "通信成功",
            COMM_PORT_BUSY: "端口忙",
            COMM_TX_FAIL: "发送失败",
            COMM_RX_FAIL: "接收失败",
            COMM_TX_ERROR: "发送错误",
            COMM_RX_WAITING: "等待接收",
            COMM_RX_TIMEOUT: "接收超时",
            COMM_RX_CORRUPT: "接收数据损坏",
            COMM_NOT_AVAILABLE: "不可用"
        }
        return results.get(result, "未知通信错误")
    
    def get_rx_packet_error(self, error):
        """获取舵机错误描述"""
        errors = {
            0: "无错误",
            ERRBIT_VOLTAGE: "电压超出范围",
            ERRBIT_ANGLE: "角度超出限制",
            ERRBIT_OVERHEAT: "过热",
            # ERRBIT_RANGE: "超出范围",
            # ERRBIT_CHECKSUM: "校验和错误",
            ERRBIT_OVERLOAD: "过载",
            # ERRBIT_INSTRUCTION: "指令错误"
        }
        return errors.get(error, "未知舵机错误")
    
    def close(self):
        """关闭连接"""
        self.port_handler.closePort()
        print("串口连接已关闭")

def press(controller, servo_id, position=500):
    try:
        comm_result, error = controller.move_to_position(servo_id, position)
        if comm_result == COMM_SUCCESS and error == 0:
            print(f"舵机 {servo_id} 成功移动到位置: {position}")
    except ValueError:
        print(f"舵机 {servo_id} 下压出错")

def lift(controller, servo_id, position=600):
    try:
        comm_result, error = controller.move_to_position(servo_id, position)
        if comm_result == COMM_SUCCESS and error == 0:
            print(f"舵机 {servo_id} 成功移动到位置: {position}")
    except ValueError:
        print(f"舵机 {servo_id} 上拉出错")

def click(controller, servo_id):
    press(controller, servo_id)
    time.sleep(1)
    lift(controller, servo_id)
    time.sleep(1)
    controller.release_torque(servo_id)

def main():
    # 创建舵机控制器实例
    try:
        controller = FeetechServoController(port='COM10', baudrate=1000000)
    except:
        return
    
    # 1 500 - 移动ID1舵机到位置500
    # 2 600 - 移动ID2舵机到位置600
    # c 1 - 执行ID1舵机的点击动作
    # c 2 - 执行ID2舵机的点击动作


    print("飞特舵机控制程序")
    print("输入目标位置(0-1000)移动舵机")
    print("输入'r ID'释放扭矩，输入'e ID'使能扭矩")
    print("输入'p ID'读取当前位置，输入'q'退出")
    print("输入'd ID'下压，'s ID'上拉，'c ID'点击")
    
    try:
        while True:
            user_input = input("> ").strip().lower().split()
            
            if not user_input:
                continue
                
            if user_input[0] == 'q':
                print("退出程序...")
                break
            elif user_input[0] == 'r' and len(user_input) == 2:
                try:
                    servo_id = int(user_input[1])
                    controller.release_torque(servo_id)
                except ValueError:
                    print("无效舵机ID")
            elif user_input[0] == 'e' and len(user_input) == 2:
                try:
                    servo_id = int(user_input[1])
                    controller.enable_torque(servo_id)
                except ValueError:
                    print("无效舵机ID")
            elif user_input[0] == 'p' and len(user_input) == 2:
                try:
                    servo_id = int(user_input[1])
                    pos, comm_result, error = controller.read_position(servo_id)
                    if comm_result == COMM_SUCCESS and error == 0:
                        print(f"舵机 {servo_id} 当前位置: {pos}")
                except ValueError:
                    print("无效舵机ID")
            elif user_input[0] == 'd' and len(user_input) == 2:
                try:
                    servo_id = int(user_input[1])
                    press(controller, servo_id)
                except ValueError:
                    print("无效舵机ID")
            elif user_input[0] == 's' and len(user_input) == 2:
                try:
                    servo_id = int(user_input[1])
                    lift(controller, servo_id)
                except ValueError:
                    print("无效舵机ID")
            elif user_input[0] == 'c' and len(user_input) == 2:
                try:
                    servo_id = int(user_input[1])
                    click(controller, servo_id)
                except ValueError:
                    print("无效舵机ID")
            else:
                try:
                    if len(user_input) == 2:
                        servo_id = int(user_input[0])
                        position = int(user_input[1])
                        comm_result, error = controller.move_to_position(servo_id, position)
                        if comm_result == COMM_SUCCESS and error == 0:
                            print(f"舵机 {servo_id} 成功移动到位置: {position}")
                            time.sleep(1)
                            controller.release_torque(servo_id) 
                    else:
                        print("无效输入")
                except ValueError:
                    print("无效输入，请输入'命令 ID'或'ID 位置'")

    finally:
        controller.close()



def clickServo(controller, servo_id):
    position = 490
    if servo_id == 1:
        position = 490 #1 - 490  2-500
    if servo_id == 2:
        position = 450 #1 - 490  2-500
    if servo_id == 3:
        position = 430

    press(controller, servo_id, position)
    time.sleep(1)
    lift(controller, servo_id)
    time.sleep(1)
    controller.release_torque(servo_id)


if __name__ == "__main__":
    controller = FeetechServoController(port='COM10', baudrate=1000000)
    # clickServo(controller, 1)
    clickServo(controller, 2) # laptop 
    # clickServo(controller, 3)

    # while True:
    #     clickServo(controller, 3)
    #     time.sleep(5)