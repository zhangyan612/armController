import pyautogui
import socket
import time

# 配置参数
SERVER_IP = '192.168.1.100'  # 接收端IP地址
SERVER_PORT = 5555
INTERVAL = 0.02  # 发送间隔（秒）

def get_mouse_position():
    """获取当前鼠标坐标"""
    return pyautogui.position()

def main():
    # 创建UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    
    try:
        while True:
            # 获取并发送鼠标坐标
            x, y = get_mouse_position()
            message = f"{x},{y}"
            sock.sendto(message.encode(), (SERVER_IP, SERVER_PORT))
            time.sleep(INTERVAL)
    except KeyboardInterrupt:
        print("\n发送端已停止")

if __name__ == "__main__":
    main()