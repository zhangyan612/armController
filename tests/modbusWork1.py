import modbus_tk 
from modbus_tk import modbus_rtu 
import serial 
def main(): 
    try: 
        # 通信设置 
        master = modbus_rtu.RtuMaster(serial.Serial(port='COM5', # 连接端口 
                                                    baudrate=2250000, # 连接波特率 
                                                    bytesize=8, # 数据位 
                                                    parity='N', # 奇偶校验位 
                                                    stopbits=1)) # 停止位 
        master.set_timeout(5.0) 
        master.set_verbose(True) 
        # 发送数据并接受数据 
        read = master.execute(slave=1, # 从机地址 
                               function_code=0x03, # 功能码 
                               starting_address=0x01, # 寄存器开始地址 
                               quantity_of_x=9)
        print(read) # 打印获取的数据 
    except Exception as exc: 
        print(str(exc)) 

if __name__ == "__main__": 
    main()