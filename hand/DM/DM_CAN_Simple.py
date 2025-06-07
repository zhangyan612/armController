import can
import numpy as np
import time
from struct import unpack
from struct import pack

# -------------------- Motor Parameter Limits --------------------
class LimitMotor:
    def __init__(self, P_MAX: float, V_MAX: float, T_MAX: float):
        self.Q_MAX = P_MAX
        self.DQ_MAX = V_MAX
        self.TAU_MAX = T_MAX

Motor_Param_limits = [
    LimitMotor(12.5, 30, 10),  # DM_4310_Limit
    LimitMotor(12.5, 50, 10),  # DM_4310_48V_Limit
    LimitMotor(12.5, 10, 28),  # DM_4340_Limit
    LimitMotor(12.5, 45, 20),  # DM_6006_Limit
    LimitMotor(12.5, 45, 40),  # DM_8006_Limit
    LimitMotor(12.5, 45, 54),  # DM_8009_Limit
]

# Choose your motor type
MotorType = 0  # 0 = DM_4310


# -------------------- Conversion Functions --------------------
def float_to_uint(x: float, x_min: float, x_max: float, bits: int) -> int:
    x = min(max(x, x_min), x_max)
    span = x_max - x_min
    data_norm = (x - x_min) / span
    return int(data_norm * ((1 << bits) - 1))


def uint_to_float(x: int, x_min: float, x_max: float, bits: int) -> float:
    span = x_max - x_min
    data_norm = float(x) / ((1 << bits) - 1)
    return data_norm * span + x_min


def float_to_uint8s(value):
    # Pack the float into 4 bytes
    packed = pack('f', value)
    # Unpack the bytes into four uint8 values
    return unpack('4B', packed)

# -------------------- Main Motor Controller Class --------------------
class MotorController:
    def __init__(self, port='COM12', bitrate=1000000, can_id=0x01, motor_type=0):
        self.port = port
        self.bitrate = bitrate
        self.can_id = can_id
        self.motor_type = motor_type
        self.limits = Motor_Param_limits[motor_type]
        self.bus = can.interface.Bus(interface='slcan', channel=self.port, bitrate=self.bitrate)

    def shutdown(self):
        self.bus.shutdown()
        print("CAN bus shutdown completed.")

    def send_command(self, data):
        msg = can.Message(arbitration_id=self.can_id, data=data, is_extended_id=False)
        try:
            self.bus.send(msg)
            print(f"Sent: {data}")
        except can.CanError:
            print("Send failed.")

    def enable_motor(self):        self.send_command([0xFF]*7 + [0xFC])
    def disable_motor(self):       self.send_command([0xFF]*7 + [0xFD])
    def save_zero_position(self):  self.send_command([0xFF]*7 + [0xFE])
    def clear_error(self):         self.send_command([0xFF]*7 + [0xFB])

    def control_mit(self, q: float, dq: float, kp: float, kd: float, tau: float):
        q_u   = float_to_uint(q,   -self.limits.Q_MAX,   self.limits.Q_MAX,   16)
        dq_u  = float_to_uint(dq,  -self.limits.DQ_MAX,  self.limits.DQ_MAX,  12)
        kp_u  = float_to_uint(kp,  0, 500, 12)
        kd_u  = float_to_uint(kd,  0, 5,   12)
        tau_u = float_to_uint(tau, -self.limits.TAU_MAX, self.limits.TAU_MAX, 12)

        data = [
            (q_u >> 8) & 0xFF,
            q_u & 0xFF,
            dq_u >> 4,
            ((dq_u & 0xF) << 4) | ((kp_u >> 8) & 0xF),
            kp_u & 0xFF,
            kd_u >> 4,
            ((kd_u & 0xF) << 4) | ((tau_u >> 8) & 0xF),
            tau_u & 0xFF,
        ]
        self.send_command(bytearray(data))

    def control_pos_force(self, Pos_des: float, Vel_des, i_des):
        """
        control the motor in EMIT control mode 电机力位混合模式
        :param Pos_des: desired position rad  期望位置 单位为rad
        :param Vel_des: desired velocity rad/s  期望速度 为放大100倍
        :param i_des: desired current rang 0-10000 期望电流标幺值放大10000倍
        电流标幺值：实际电流值除以最大电流值，最大电流见上电打印
        """
        data_buf = np.array([0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00], np.uint8)
        Pos_desired_uint8s = float_to_uint8s(Pos_des)
        data_buf[0:4] = Pos_desired_uint8s
        Vel_uint = np.uint16(Vel_des)
        ides_uint = np.uint16(i_des)
        data_buf[4] = Vel_uint & 0xff
        data_buf[5] = Vel_uint >> 8
        data_buf[6] = ides_uint & 0xff
        data_buf[7] = ides_uint >> 8
        self.send_command(self.can_id, data_buf)
        msg = self.bus.recv(timeout=1.0)
        if msg:
            self.parse_feedback(msg)

    def listen(self, duration=5.0):
        print("Listening for motor feedback...")
        start = time.time()
        while time.time() - start < duration:
            msg = self.bus.recv(timeout=1.0)
            if msg:
                self.parse_feedback(msg)

    def parse_feedback(self, msg):
        data = msg.data
        q_u = (data[1] << 8) | data[2]
        dq_u = (data[3] << 4) | (data[4] >> 4)
        tau_u = ((data[4] & 0xF) << 8) | data[5]

        q = uint_to_float(q_u,   -self.limits.Q_MAX,   self.limits.Q_MAX,   16)
        dq = uint_to_float(dq_u, -self.limits.DQ_MAX,  self.limits.DQ_MAX,  12)
        tau = uint_to_float(tau_u, -self.limits.TAU_MAX, self.limits.TAU_MAX, 12)

        print(f"[Feedback] q: {q:.3f} rad\t dq: {dq:.3f} rad/s\t tau: {tau:.3f} Nm")



if __name__ == "__main__":
    motor = MotorController(port='COM12', bitrate=1000000, can_id=0x01, motor_type=0)

    motor.clear_error()
    time.sleep(0.1)

    motor.enable_motor()
    time.sleep(0.2)

    # setting pmax 50 range -8 - 12 
    # setting pmax 30 range -12 - 20 

    param_sets = [
        # (1.0, 0.2, 0, 1.0, 0.0),  # 太慢
        # (5, 0.5, 0.5, 1, 0.0),
        # (0, 1, 1, 0.5, 1),
        # (5, 1, 0.5, 0.6, 1), //good
        # (0, 5, 0.3, 0.1, 1),
        (25, 6, 0.3, 0.1, 2),
        # (-2000, 6, 0.3, 0.1, 2),
    ]

    # 速度 dq （刚度）kp （阻尼）kd	力矩 tau		
    for q, dq, kp, kd, tau in param_sets:
        print(f"\nTrying q={q}, dq={dq}, kp={kp}, kd={kd}, tau={tau}")
        motor.control_mit(q=q, dq=dq, kp=kp, kd=kd, tau=tau)
        motor.listen(duration=2)
        time.sleep(0.5)


    # # Send MIT control command
    # motor.control_mit(q=2.0, dq=0.5, kp=10, kd=5, tau=1.0)
    # time.sleep(5)

    # motor.control_mit(q=1, dq=0.5, kp=5, kd=5, tau=1.0)
    # time.sleep(5)
    # motor.save_zero_position()
    # time.sleep(0.5)

    motor.listen(duration=10)

    motor.disable_motor()
    motor.shutdown()
