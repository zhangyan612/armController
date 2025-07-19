import can

# 检测可用的 CAN 配置
available_configs = can.detect_available_configs()
print("Available CAN configurations:", available_configs)

# 使用第一个可用的 pcan 接口配置
for config in available_configs:
    if config['interface'] == 'pcan':
        try:
            # 初始化 CAN 总线
            bus = can.interface.Bus(**config, bitrate=500000)
            print(f"Successfully initialized CAN bus on {config['channel']} with interface {config['interface']}")
            
            # 发送一个测试消息
            msg = can.Message(arbitration_id=0x123, data=[0x11, 0x22, 0x33], is_extended_id=False)
            bus.send(msg)
            print("Message sent successfully")
            
            # 关闭总线
            bus.shutdown()
        except Exception as e:
            print(f"Failed to initialize CAN bus: {e}")
        break
else:
    print("No suitable PCAN interface found.")



