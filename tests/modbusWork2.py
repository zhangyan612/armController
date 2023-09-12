import struct
import platform
import time
import serial #导入模块


sysstr = platform.system()

#RS485文件的通信地址,在开发板上是/dev/ttysWK0
if(sysstr=="Windows"):
    RS485FileName = "COM5"
elif(sysstr =="Linux"):
    RS485FileName = "/dev/ttysWK0"
# RS485：/dev/ttysWK0
# RS232：/dev/ttysWK1
# UART2：/dev/ttysWK3
# UART1：/dev/ttysWK2
else:
    RS485FileName = "/dev/ttysWK0"


# ===============================================================
def CalCRC16(data, length):
    #print(data, length) #打印数据，长度
    crc=0xFFFF
    if length == 0:
        length = 1
        j = 0
        while length != 0:
            crc ^= list.__getitem__(data, j)
            #print('j=0x%02x, length=0x%02x, crc=0x%04x' %(j,length,crc))
            for i in range(0,8):
                if crc & 1:
                    crc >>= 1
                    crc ^= 0xA001
                else:
                    crc >>= 1
            length -= 1
            j += 1
        return crc
    
# ===============================================================
def CRCBuffer(buffer):
    crc_transformation = CalCRC16(buffer,len(buffer))
    #crc_calculation = hex(crc_transformation)
    #print('crc_calculation:',crc_calculation)
    tasd = [0x00,0x00]
    tasd[0] = crc_transformation & 0xFF
    tasd[1] = (crc_transformation >> 8) & 0xFF
    H =hex(tasd[0])
    L =hex(tasd[1])
    H_value = int(H,16)
    L_value = int(L,16)
    buffer.append(H_value)
    buffer.append(L_value)
    return buffer
# ===============================================================


# if __name__ == '__main__':

# print(CRCBuffer(buffer))
def Send485CMD_OLD(cmd):
    global RS485FileName

    with open(RS485FileName, "wb") as rs485:
        buffercrc = CRCBuffer(cmd)
        ret = rs485.write(bytes(buffercrc))
        rs485.close()
        return ret

def SendAndRead485CMD(cmd):
    global RS485FileName

    ser = serial.Serial(RS485FileName,2250000,timeout=10)
    buffercrc = CRCBuffer(cmd)
    print("RS485 TX =>", bytes(bytearray(buffercrc)))
    ser.write(buffercrc) # 这里直接发送int类型的数据, 内部会自动转换成byte字节.
    # ret = ser.write(bytes(buffercrc))
    time.sleep(0.2)
    res = ser.read_all()
    print("RS485 RX =>", res)
    ser.close()
    return res

def ReadGateSate(gatenum):
    cmd = [
    gatenum, #设备地址='\x00'# 0x00 is guang bo
    0x03, #功能码='\x06' #\06 is write, \03 is read, \10 is write mutil
    0x00, #寄存器地址高='\x00'
    0x15, #寄存器地址低='\x0E' #0E is open door cmd
    0x00, #数据高='\x00'
    0x01, #数据低='\x01'
    # 0x00, #CRC高='\x00'
    # 0x00, #CRC低='\x00'
    ]
    return SendAndRead485CMD(cmd)


def OpenTheDoor(gatenum):
    cmd = [
    gatenum, #设备地址='\x00'# 0x00 is guang bo
    0x06, #功能码='\x06' #\06 is write, \03 is read, \10 is write mutil
    0x00, #寄存器地址高='\x00'
    0x0E, #寄存器地址低='\x0E' #0E is open door cmd
    0x00, #数据高='\x00'
    0x01, #数据低='\x01'
    # 0x00, #CRC高='\x00'
    # 0x00, #CRC低='\x00'
    ]
    SendAndRead485CMD(cmd);
    pass

def CloseTheDoor(gatenum):
    cmd = [
    gatenum, #设备地址='\x00'# 0x00 is guang bo
    0x06, #功能码='\x06' #\06 is write, \03 is read, \10 is write mutil
    0x00, #寄存器地址高='\x00'
    0x0E, #寄存器地址低='\x0E' #0E is open door cmd
    0x00, #数据高='\x00'
    0xFF, #数据低='\x01'
    # 0x00, #CRC高='\x00'
    # 0x00, #CRC低='\x00'
    ]
    SendAndRead485CMD(cmd);
    pass

def OpenTheLid(gatenum):
    cmd = [
    gatenum, #设备地址='\x00'# 0x00 is guang bo
    0x06, #功能码='\x06' #\06 is write, \03 is read, \10 is write mutil
    0x00, #寄存器地址高='\x00'
    0x0E, #寄存器地址低='\x0E' #0E is open door cmd
    0x00, #数据高='\x00'
    0x01, #数据低='\x01'
    # 0x00, #CRC高='\x00'
    # 0x00, #CRC低='\x00'
    ]
    SendAndRead485CMD(cmd)
    pass

def CloseTheLid(gatenum):
    cmd = [
    gatenum, #设备地址='\x00'# 0x00 is guang bo
    0x06, #功能码='\x06' #\06 is write, \03 is read, \10 is write mutil
    0x00, #寄存器地址高='\x00'
    0x0E, #寄存器地址低='\x0E' #0E is open door cmd
    0x00, #数据高='\x00'
    0xFF, #数据低='\x01'
    # 0x00, #CRC高='\x00'
    # 0x00, #CRC低='\x00'
    ]
    SendAndRead485CMD(cmd);
    pass


res = OpenTheDoor(0x01)
res = ReadGateSate(0x01)


# 串口HEX

# GET 
# 01 03 00 15 00 00 00 02 C5 3E
# Receive 01 03 00 15 00 00 7F FF 74 DF

# 01 03 00 13 00 00 00 02 C5 B6
# Receive 01 03 00 00 00 00 0D C1 F6

# 01 03 00 14 00 00 00 02 05 03
# Receive 01 03 00 14 00 00 00 00
# Receive C4 82

#Post