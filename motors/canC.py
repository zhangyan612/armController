def parse_can_data(rx_data):
    """Python版本解析逻辑（严格对应C代码）"""
    # 检查数据长度
    if len(rx_data) < 6:
        raise ValueError("CAN数据至少需要6个字节")

    # 位置解析（无符号16位）
    motor_position = (rx_data[1] << 8) | rx_data[2]

    # 速度解析（有符号12位）
    velocity_high = (rx_data[3] & 0xFF) << 4
    velocity_low = (rx_data[4] & 0xF0) >> 4
    motor_velocity = (velocity_high | velocity_low)
    motor_velocity = (motor_velocity - 2048) if motor_velocity < 2048 else (motor_velocity - 4096)

    # 电流解析（有符号12位）
    current_high = (rx_data[4] & 0x0F) << 8
    current_low = rx_data[5] & 0xFF
    motor_current = (current_high | current_low)
    motor_current = (motor_current - 2048) if motor_current < 2048 else (motor_current - 4096)

    return {
        'position': motor_position,
        'velocity': motor_velocity,
        'current': motor_current
    }

# 测试数据验证
test_cases = [
    # 原始C测试案例
    {
        "input": [0x00, 0x80, 0x00, 0x08, 0x00, 0x08, 0x00, 0x00],
        "expected": {'position': 0x8000, 'velocity': 0, 'current': 0}
    },
    # 边界值测试
    {
        "input": [0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF],
        "expected": {'position': 0xFFFF, 'velocity': -2048, 'current': -2048}
    },
    # 典型值测试
    {
        "input": [0x00, 0x7F, 0xFF, 0x0F, 0xF0, 0x0F, 0xF0, 0x00],
        "expected": {'position': 0x7FFF, 'velocity': 2047, 'current': 2047}
    }
]

# 执行测试验证
print(f"| {'测试数据':^20} | {'Position(C→Py)':^18} | {'Velocity(C→Py)':^18} | {'Current(C→Py)':^18} |")
print("|----------------------|--------------------|--------------------|--------------------|")
for case in test_cases:
    result = parse_can_data(case["input"])
    input_hex = ' '.join(f'{x:02X}' for x in case["input"])
    
    row = f"| {input_hex:20} | " \
          f"{case['expected']['position']:4} → {result['position']:4} | " \
          f"{case['expected']['velocity']:4} → {result['velocity']:4} | " \
          f"{case['expected']['current']:4} → {result['current']:4} |"
    print(row)