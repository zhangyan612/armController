class MotorController:
    def __init__(self, 
                 interface='pcan',
                 can_channel='PCAN_USBBUS1',
                 bitrate=1000000,
                 position_max=1,       # 位置满量程圈数（默认1圈）
                 velocity_max=200,     # 速度满量程（rad/s）
                 current_max=4):       # 电流满量程（A）
        # 原有总线初始化...
        
        # 电机参数配置
        self.position_max = position_max
        self.velocity_max = velocity_max
        self.current_max = current_max

    def _parse_response(self, data):
        """增强型协议解析"""
        try:
            if len(data) == 8:
                return self._parse_standard_feedback(data)
            elif len(data) == 6:
                return self._parse_compact_feedback(data)
            else:
                return {"error": f"无效数据长度: {len(data)}字节"}
        except Exception as e:
            return {"error": f"解析异常: {str(e)}"}

    def _parse_standard_feedback(self, data):
        """解析标准8字节反馈数据（位置16bit + 速度12bit + 电流12bit）"""
        # 转换为64位整数
        value = int.from_bytes(data, byteorder='big', signed=False)
        
        # 字段提取（使用位掩码和位移）
        position_code = (value >> 48) & 0xFFFF  # 高16位为位置
        velocity_code = (value >> 36) & 0xFFF    # 中间12位速度
        current_code = (value >> 24) & 0xFFF     # 低12位电流
        
        return {
            'position': self._parse_position(position_code),
            'velocity': self._parse_velocity(velocity_code),
            'current': self._parse_current(current_code)
        }

    def _parse_compact_feedback(self, data):
        """解析紧凑型6字节反馈数据"""
        value = int.from_bytes(data, byteorder='big', signed=False)
        
        # 字段提取（根据具体协议调整）
        position_code = (value >> 36) & 0xFFFF   # 假设前16位位置
        velocity_code = (value >> 24) & 0xFFF    # 中间12位速度
        current_code = (value >> 12) & 0xFFF     # 最后12位电流
        
        return {
            'position': self._parse_position(position_code),
            'velocity': self._parse_velocity(velocity_code),
            'current': self._parse_current(current_code)
        }

    def _parse_position(self, code):
        """解析位置编码"""
        if code == 0xFFFF:
            return "正向超限"
        if code == 0x0000:
            return "反向超限"
            
        # 正常值计算
        position_deg = ((code - 0x8000) / 0x8000) * 360 * self.position_max
        return f"{position_deg:.2f}°"

    def _parse_velocity(self, code):
        """解析速度编码"""
        velocity = (code - 0x800) / 0x800 * self.velocity_max
        return f"{velocity:.2f} rad/s"

    def _parse_current(self, code):
        """解析电流编码"""
        current = (code - 0x800) / 0x800 * self.current_max
        return f"{current:.3f} A"

# 测试验证
if __name__ == "__main__":
    # 初始化带参数（根据实际电机型号设置）
    mc = MotorController(
        position_max=1,    # 位置量程1圈
        velocity_max=200,  # 速度量程200rad/s 
        current_max=4      # 电流量程4A
    )
    
    # 测试样本数据
    test_data = [
        (bytes.fromhex('FFA495D3E83E8866'), "位置超限+正常速度电流"),
        (bytes.fromhex('000095D3E83E8866'), "位置超限+正常速度电流"),
        (bytes.fromhex('8000ABA000000A00'), "零位位置+速度电流")
    ]
    
    for data, desc in test_data:
        print(f"\n测试案例: {desc}")
        print(f"原始数据: {data.hex().upper()}")
        parsed = mc._parse_response(data)
        print("解析结果:", parsed)