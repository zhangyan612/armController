# ====== 导入驱动模块 ======
from qudongku import initialize_serial, enable_action, grab_action, recover_action, release_action

# ====== 主执行函数 ======
def main():
    # 初始化串口
    initialize_serial()
    while(1):
        # 按顺序执行动作
        recover_action()  # 解除堵转
        grab_action()  # 夹紧到最小



if __name__ == "__main__":
    main()