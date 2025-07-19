from pymodbus.client import ModbusSerialClient

def main():
    # 创建 Modbus RTU 客户端配置
    client = ModbusSerialClient(
        port='COM4',      # 替换为你的串口号，Windows上可能是 'COM3'
        baudrate=38400,           # 波特率，根据设备设置调整
        bytesize=8,
        parity='N',
        stopbits=1,
        timeout=1
    )

    # 连接设备
    if not client.connect():
        print("❌ 无法连接到设备，请检查串口配置和连接")
        return
    print("✅ 已连接到设备")

    try:
        # 读取输入寄存器（功能码 0x04），读取地址 30（十进制）
        # 注意：有些文档中的地址是从 1 开始的，Modbus 是从 0 开始，所以这里写 address=29
        response = client.read_input_registers(address=29, count=1, slave=1)

        if not response.isError():
            temperature = response.registers[0] / 10.0  # 设备温度单位为 0.1℃
            print(f"🌡️ 当前设备温度: {temperature} ℃")
        else:
            print("⚠️ 读取数据时出错:", response)

    except Exception as e:
        print("⚠️ 发生异常:", str(e))

    finally:
        # 关闭连接
        client.close()
        print("🔌 已关闭连接")

if __name__ == "__main__":
    main()