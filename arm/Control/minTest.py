import minimalmodbus

# 直接使用minimalmodbus原生API测试
instrument = minimalmodbus.Instrument('COM8', 1)
instrument.serial.baudrate = 19200

try:
    # 尝试读取地址0（版本信息）
    version = instrument.read_register(0, functioncode=3)
    print(f"通信成功！版本号: {version}")
except minimalmodbus.NoResponseError:
    print("无响应，请检查硬件连接和配置！")
