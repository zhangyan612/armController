import can
import time
import robstride

# 初始化CAN总线
bus = can.interface.Bus(channel='can0', bustype='socketcan', bitrate=1000000)

# 创建客户端
rs_client = robstride.Client(bus, host_can_id=0xAA)

try:
    # 设置运行模式为位置模式
    rs_client.write_param(1, 'run_mode', robstride.RunMode.Position)
    
    # 使能电机
    feedback = rs_client.enable(1)
    print(f"Motor enabled. Current position: {feedback.angle:.2f} rad")
    
    # 移动到零位
    rs_client.write_param(1, 'loc_ref', 0.0)
    print("Moving to zero position...")
    
    # 等待移动完成
    time.sleep(2)
    
    # 获取当前位置
    position = rs_client.read_param(1, 'mechpos')
    print(f"Current position: {position:.2f} rad")
    
    # 移动到新位置
    rs_client.write_param(1, 'loc_ref', 1.57)  # π/2
    print("Moving to 90 degrees...")
    
    # 等待移动完成
    time.sleep(2)
    
    # 获取反馈信息
    feedback = rs_client.get_feedback(1)
    print(f"Position: {feedback.angle:.2f} rad, Temp: {feedback.temp:.1f}°C")
    
    # 设置零点
    rs_client.set_zero_position(1)
    print("Zero position set")
    
finally:
    # 禁用电机并关闭总线
    rs_client.disable(1)
    bus.shutdown()
