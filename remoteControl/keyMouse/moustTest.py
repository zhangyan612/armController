import serial
import ch9329Comm

serial.ser = serial.Serial('COM10', 9600)  # 开启串口

# 键盘输出helloworld
keyboard = ch9329Comm.keyboard.DataComm()
# keyboard.send_data('HH')  # 按下HELLO
# keyboard.release()  # 松开
# keyboard.send_data('WWOORRLLDD')  # 按下WORLD
# keyboard.release()  # 松开
keyboard.send_multiple_keys(("L_CTRL", "CC", "", "", "", ""))

# keyboard.send_data('WWOORRLLDD')
keyboard.release()  # 松开
# （相对）鼠标右移100px 下移100px并单击左键
# mouse = ch9329Comm.mouse.DataComm(1920,1080) # 屏幕分辨率为1920*1080

# mouse.send_data_absolute(1, 1)
# mouse.click()  # 单击左键


serial.ser.close()  # 关闭串口
