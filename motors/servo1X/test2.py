from pymodbus.client import ModbusSerialClient

def test_connection():
    client = ModbusSerialClient(
        port='COM4',      # 替换为你实际使用的串口号
        baudrate=38400,           # 默认值，根据设备修改
        bytesize=8,
        parity='N',
        stopbits=1,
        timeout=2,
    )

    if not client.connect():
        print("❌ 无法连接到设备，请检查串口配置")
        return

    print("✅ 已连接到设备")

    try:
        # 发送一个简单的读取命令（例如读取地址30）
        response = client.read_input_registers(address=29, count=1, slave=1)

        if not response.isError():
            print("🟢 收到有效响应:", response.registers)
        else:
            print("🔴 收到错误响应:", response)

    except Exception as e:
        print("⚠️ 异常发生:", str(e))

    finally:
        client.close()

if __name__ == "__main__":
    test_connection()