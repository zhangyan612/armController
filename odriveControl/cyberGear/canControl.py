import serial
import struct
import time
# Connect to the CAN bus with 1 Mbit/s bitrate
ser = serial.Serial('COM14', 921600, timeout=1)
# Enter "AT command mode"
ser.write(bytes.fromhex('41 54 2b 41 54 0d 0a'))
# 双电机给定速度运行，速度模式的通信类型：18-->3-->18-->18

# 通信类型18：发送电机模式参数写入命令，设置runmode=2
# 90 07 e8 0c-->12 00 FD 01(电机1的canid=1) 05 70 00 00 02 00 00 00-->0x7005的runmode=2
ser.write(bytes.fromhex('41 54 90 07 e8 0c 08 05 70 00 00 02 00 00 00 0d 0a'))

# 90 07 e8 14-->12 00 FD 02(电机2的canid=2) 05 70 00 00 02 00 00 00-->0x7005的runmode=2
ser.write(bytes.fromhex('41 54 90 07 e8 14 08 05 70 00 00 02 00 00 00 0d 0a'))
# 通信类型3：发送电机使能运行帧
# 18 07 e8 0c-->03 00 FD 01(canid1使能运行)
ser.write(bytes.fromhex('41 54 18 07 e8 0c 08 00 00 00 00 00 00 00 00 0d 0a'))
# 18 07 e8 14-->03 00 FD 02(canid2使能运行)
ser.write(bytes.fromhex('41 54 18 07 e8 14 08 00 00 00 00 00 00 00 00 0d 0a'))
# 通信类型18：发送电机模式参数写入命令，设置 limit_cur 参数为预设最大电流指令
# 90 07 e8 0c-->12 00 FD 01(canid1) 18 70 00 00 00 00 b8 41 0d 0a-->0x7018的limit_cur=23
ser.write(bytes.fromhex('41 54 90 07 e8 0c 08 18 70 00 00 00 00 b8 41 0d 0a'))
# 90 07 e8 14-->12 00 FD 02(canid2) 18 70 00 00 00 00 b8 41 0d 0a-->0x7018的limit_cur=23
ser.write(bytes.fromhex('41 54 90 07 e8 14 08 18 70 00 00 00 00 b8 41 0d 0a'))
#通信类型18：发送电机模式参数写入命令，设置 spd_ref 参数为预设速度指令
# 90 07 e8 0c-->12 00 FD 01(canid1) 0a 70 00 00 00 00 b8 41 0d 0a-->0x700a的spd_ref=5
ser.write(bytes.fromhex('41 54 90 07 e8 0c 08 0a 70 00 00 00 00 a0 40 0d 0a'))
# 90 07 e8 14-->12 00 FD 02(canid2) 0a 70 00 00 00 00 b8 41 0d 0a-->0x700a的spd_ref=5
ser.write(bytes.fromhex('41 54 90 07 e8 14 08 0a 70 00 00 00 00 a0 40 0d 0a'))

# 通信类型4：电机停止运行
# 20 07 e8 0c-->04 00 FD 01(canid1停止运行)
ser.write(bytes.fromhex('41 54 20 07 e8 0c 08 00 00 00 00 00 00 00 00 0d 0a'))
# 20 07 e8 14-->04 00 FD 02(canid2停止运行)
ser.write(bytes.fromhex('41 54 20 07 e8 14 08 00 00 00 00 00 00 00 00 0d 0a'))
# Close the serial port
ser.close()