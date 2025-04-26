from MotorControlShell import *
import math
import time
import numpy as np


class Robot:
    def __init__(self, port, baudrate):
        self.motor_control = MotorControl(port, baudrate)
        self.motor_list = list()
        self.encoder = pow(2, 15)

    def register_motor(self, motor_id, ratio):
        self.motor_control.check_id(motor_id)
        self.motor_list.append(dict(id=motor_id, ratio=ratio))

    def servo_enable(self):
        for motor in self.motor_list:
            self.motor_control.write_servo_state(motor["id"], True)

    def servo_disable(self):
        for motor in self.motor_list:
            self.motor_control.write_servo_state(motor["id"], False)

    def return_home(self):
        for motor in self.motor_list:
            self.motor_control.return_home(motor["id"])
        print("Running return home command ...")
        
        self.running_blocked()
        print("Return home complete!")


    def return_home_opt(self):
        # 创建三个空列表，用于存储电机位置、输出位置和比例
        encoder_motor = list()
        encoder_output = list()
        ratio = list()
        # 遍历电机列表，读取每个电机的位置、输出位置和比例
        for motor in self.motor_list:
            encoder_motor.append(self.motor_control.read_position_motor(motor["id"]))
            eo = self.motor_control.read_position_output(motor["id"])
            encoder_output.append(eo if eo < 16384 else eo - 32767)
            ratio.append(motor["ratio"])
        # 将列表转换为数组
        encoder_motor = np.array(encoder_motor)
        encoder_output = np.array(encoder_output)
        ratio = np.array(ratio)
        # 计算偏移量
        offset = (-encoder_output * ratio + encoder_motor) * 2 * math.pi / (ratio * self.encoder)

        self.write_position(offset, np.ones(len(self.motor_list)) * 0.2)
        self.running_blocked()

        self.return_home()
        self.running_blocked()


    def read_position(self):
        pos = list()
        for motor in self.motor_list:
            pos.append(self.encoder_to_rad(self.motor_control.read_position_output(motor["id"])))
        return np.array(pos)
    
    
    def write_position(self, pos, vel):
        # pos: rad, vel: rad/s
        motor_count = len(self.motor_list)
        if motor_count != len(pos) or motor_count != len(vel):
            raise Exception("The number of pos/vel is inconsistent with the number of motors!")
        for i in range(motor_count):
            motor = self.motor_list[i]
            self.motor_control.write_position(motor["id"], int(motor["ratio"] * self.rad_to_encoder(pos[i])), int(vel[i] * motor["ratio"] * 60 / (2 * math.pi)))
            print(f"Motor {motor['id']} set to position {int(motor['ratio'] * self.rad_to_encoder(pos[i]))} rad with velocity {int(vel[i] * motor['ratio'] * 60 / (2 * math.pi))} rad/s")
        print("Running write position command ...")

        self.running_blocked()
        print("Running complete!")

    def running_blocked(self):
        while (self.is_running()):
            time.sleep(0.02)


    def is_running (self):
        for motor in self.motor_list:
            if self.motor_control.read_running_state(motor["id"]):
                return True
            # 将pos和vel转换为电机编码器的值
        return False


    def rad_to_encoder(self, data):
        return self.encoder * data / (2 * math.pi)
    
    
    def encoder_to_rad(self, data):
        return data / self.encoder * 2 * math.pi



if __name__ == "__main__":

    # 设置串口号与波特率
    robot = Robot("COM8", 2250000)
    # 设置电机ID与减速比
    # robot.register_motor(2, 101)
    robot.register_motor(6, 101)
    
    # motor 1 180

    # motor 2 # -2 to 2
    # motor 3  # -2.2 to 2.25 
    # motor 4 180
    # motor 5 range [-2,2]
    # motor 6 180

    # 1654220
    # 3307940

    # 期望位置与运动速度
    # pos1 = np.array([np.pi/4])
    # pos1 = np.array([5, 10])
    # # pos2 = np.array([-np.pi/4])
    # vel = np.array([0.5, 0.5])
    vel = np.array([2.5])
    # 2.08 0 
    # 电机使能
    # robot.servo_disable()

    robot.servo_enable()
    # print(robot.read_position())

    pos1 = np.array([0])
    pos2 = np.array([3.14])
    # pos3 = np.array([6.28])

    # print(pos1)
    # print(pos2)
    # print(vel)
    # time.sleep(10)
    # print(robot.read_position())
    # time.sleep(10)
    # print(robot.read_position())
    # time.sleep(10)
    # print(robot.read_position())

    # # 电机回到零位
    # robot.return_home_opt()
    # print(robot.read_position())
    # # print(pos1)
    robot.write_position(pos1, vel)
    # # # print(robot.read_position())
    # # # robot.return_home_opt()
    # # # print(robot.read_position())
    time.sleep(5)
    robot.write_position(pos2, vel)

    # time.sleep(5)

    # robot.write_position(pos3, vel)

    # print(robot.read_position())
    # robot.return_home_opt()
    # print(robot.read_position())
    
    # 电机失能
    robot.servo_disable()



# from fastapi import FastAPI, HTTPException
# from pydantic import BaseModel
# import numpy as np


# robot = Robot("COM8", 2250000)
# for i in range(1, 7):  # Register six motors
#     robot.register_motor(i, 101)

# app = FastAPI()

# class PositionData(BaseModel):
#     motor_id: int
#     position: float
#     velocity: float

# @app.get("/read/{motor_id}")
# def read_motor_position(motor_id: int):
#     position = robot.read_position(motor_id)
#     if position is None:
#         raise HTTPException(status_code=404, detail="Motor not found")
#     return {"motor_id": motor_id, "position": position}

# @app.post("/write/")
# def write_motor_position(data: PositionData):
#     response = robot.write_position(data.motor_id, data.position, data.velocity)
#     if response["status"] == "error":
#         raise HTTPException(status_code=404, detail=response["message"])
#     return {"motor_id": data.motor_id, "new_position": data.position}