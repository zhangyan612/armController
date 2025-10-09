import can
import time
import struct

def pcan_self_test():
    print("\n=== PCAN 自检 ===")
    try:
        bus = can.interface.Bus(bustype='pcan', channel='PCAN_USBBUS1', bitrate=500000)
        
        # 自环测试
        print("发送自环测试消息...")
        msg = can.Message(
            arbitration_id=0x123,
            data=[0xAA, 0x55, 0x01, 0x02],
            is_extended_id=False
        )
        bus.send(msg)
        
        # 接收自身消息
        echo = bus.recv(timeout=0.5)
        if echo and echo.data == msg.data:
            print("✅ PCAN自环测试成功")
        else:
            print("❌ PCAN自环失败 - 检查硬件连接")
            
        bus.shutdown()
    except Exception as e:
        print(f"PCAN初始化失败: {e}")
        print("请检查:")
        print("1. PCAN驱动是否安装")
        print("2. PCANView是否占用设备")
        print("3. 设备管理器中的PCAN状态")

# 运行自检
pcan_self_test()