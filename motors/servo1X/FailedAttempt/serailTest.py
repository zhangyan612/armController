import serial

def send_modbus_raw():
    # 配置串口
    ser = serial.Serial(
        port='COM22',     # 替换为你自己的串口号，Windows上可能是 'COM3'
        baudrate=38400,          # 波特率（根据设备设置）
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=2                # 设置超时时间
    )

    if not ser.is_open:
        print("❌ 串口未打开")
        return

    print("✅ 串口已打开")

    # 要发送的 Modbus RTU 命令（十六进制字节）
    command = bytes.fromhex('FF 03 00 1E 00 02 B1 D3')

    try:
        # 发送命令
        ser.write(command)
        print(f"📤 已发送命令: {command.hex()}")

        # 读取返回数据（假设返回长度为9字节：1字节站号 + 1功能码 + 1字节长度 + N字节数据 + 2 CRC）
        response = ser.read(9)  # 可以先尝试读固定长度，或根据实际响应长度调整

        if len(response) == 0:
            print("⚠️ 没有收到响应，请检查从机地址、CRC、波特率等")
        else:
            print(f"📥 收到响应: {response.hex()}")
            # 解析响应
            if len(response) >= 5:
                byte_count = response[2]
                data_bytes = response[3:-2]  # 去掉最后两个CRC校验字节
                print(f"🔢 数据字节数: {byte_count}")
                print(f"📦 数据部分 (Hex): {data_bytes.hex()}")
                
                # 假设是两个16位寄存器（高位在前），合并为整数
                values = []
                for i in range(0, len(data_bytes), 2):
                    high = data_bytes[i]
                    low = data_bytes[i+1]
                    value = (high << 8) | low
                    values.append(value)
                print(f"📊 解析后的寄存器值: {values}")

    except Exception as e:
        print("❌ 异常发生:", str(e))

    finally:
        ser.close()
        print("🔌 串口已关闭")

if __name__ == "__main__":
    send_modbus_raw()