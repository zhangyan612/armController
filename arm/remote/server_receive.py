import pyautogui
import socket

# 配置参数
LISTEN_PORT = 5555

def move_mouse(x, y):
    """移动鼠标到指定位置"""
    pyautogui.moveTo(x, y)

def main():
    # 创建UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('0.0.0.0', LISTEN_PORT))
    
    print(f"监听端口 {LISTEN_PORT}...")
    
    try:
        while True:
            # 接收并处理数据
            data, addr = sock.recvfrom(1024)
            try:
                x_str, y_str = data.decode().split(',')
                x, y = int(x_str), int(y_str)
                move_mouse(x, y)
            except (ValueError, pyautogui.FailSafeException):
                pass
    except KeyboardInterrupt:
        print("\n接收端已停止")

if __name__ == "__main__":
    main()