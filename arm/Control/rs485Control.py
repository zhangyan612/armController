import minimalmodbus
import serial
import time

class MotorController:
    def __init__(self, port, slave_address, baudrate=19200):
        """初始化MODBUS RTU连接"""
        self.instrument = minimalmodbus.Instrument(port, slave_address)
        self.instrument.serial.baudrate = baudrate
        self.instrument.serial.bytesize = 8
        self.instrument.serial.parity = serial.PARITY_NONE
        self.instrument.serial.stopbits = 1
        self.instrument.mode = minimalmodbus.MODE_RTU
        self.instrument.clear_buffers_before_each_transaction = True
        self.instrument.close_port_after_each_call = False

    def set_servo_status(self, status):
        """设置伺服状态（地址16，0:关闭/1:打开）"""
        self.instrument.write_register(16, status, functioncode=6)

    def set_position_mode(self, target):
        """位置模式无加减速（地址129，0081H）"""
        self.instrument.write_long(
            registeraddress=129,
            value=target,
            signed=True,
            byteorder=minimalmodbus.BYTEORDER_LITTLE_SWAP
        )

    def read_encoder_position(self):
        """读取编码器绝对位置（地址21，0015H）"""
        return self.instrument.read_long(
            registeraddress=21,
            signed=False,  # 根据文档0015H为0~32767
            byteorder=minimalmodbus.BYTEORDER_LITTLE_SWAP
        )

    def safe_move(self, target, max_retry=3):
        """安全移动流程（含超时和重试）"""
        for _ in range(max_retry):
            self.set_position_mode(target)
            timeout = 30  # 最大等待时间（秒）
            start_time = time.time()
            
            while time.time() - start_time < timeout:
                current_pos = self.read_encoder_position()
                if current_pos == target:
                    print(f"成功到达位置 {target}")
                    return
                time.sleep(0.5)
            
            print(f"移动超时，当前位置 {current_pos}，剩余重试次数 {max_retry-_}")
        
        raise Exception("移动失败：超过最大重试次数")

# 执行控制流程
if __name__ == "__main__":
    mc = MotorController(port='COM8', slave_address=1)
    
    try:
        # 初始化伺服系统
        mc.set_servo_status(1)
        print("伺服已启动")
        
        # 移动到0位置
        print("正在归零...")
        mc.safe_move(0)
        
        # 移动到目标位置
        print("正在移动到327680...")
        mc.safe_move(327680)
        
    except Exception as e:
        print(f"控制异常：{str(e)}")
    finally:
        mc.set_servo_status(0)
        print("伺服已关闭")