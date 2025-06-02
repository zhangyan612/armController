from qudongku import initialize_serial, enable_action, grab_action, recover_action, release_action

def main():
   
    initialize_serial()
    while(1):
       
        enable_action()  # 电机使能
        grab_action()  # 夹紧到最小
        recover_action()  # 解除堵转
        release_action()  # 松开到最大


if __name__ == "__main__":
    main()