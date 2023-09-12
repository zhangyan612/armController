import minimalmodbus

PORT='COM5'
TEMP_REGISTER = 100
HUM_REGISTER = 102

#Set up instrument
instrument = minimalmodbus.Instrument(PORT,1,mode=minimalmodbus.MODE_RTU)

#Make the settings explicit
instrument.serial.baudrate = 2250000        # Baud
instrument.serial.bytesize = 8
instrument.serial.parity   = minimalmodbus.serial.PARITY_EVEN
instrument.serial.stopbits = 1
instrument.serial.timeout  = 1          # seconds

# Good practice
instrument.close_port_after_each_call = True

instrument.clear_buffers_before_each_transaction = True

# Read temperatureas a float
# if you need to read a 16 bit register use instrument.read_register()
temperature = instrument.read_float(TEMP_REGISTER)

# Read the humidity
humidity = instrument.read_float(HUM_REGISTER)

#Pront the values
print('The temperature is: %.1f deg C\r' % temperature)
print('The humidity is: %.1f percent\r' % humidity)




# import pymodbus
# # from pymodbus.pdu import ModbusRequest
# # from pymodbus.client.sync import ModbusSerialClient as ModbusClient
# # # from pymodbus.transaction import ModbusRtuFramer

# # client = ModbusClient(method='rtu', port="COM5", baudrate=2250000, parity='E', timeout=0.1)
# # connection = client.connect()

# # read_vals  = client.read_holding_registers(248, 4, unit=1) # start_address, count, slave_id
# # print(read_vals.registers)

# # write registers
# # write  = client.write_register(1,425,unit=1)# address = 1, value to set = 425, slave ID = 1



# from pylibmodbus import ModbusRtu


# # For Python 3.x you have to explicitly indicate ASCII enconding
# client=ModbusRtu(
#     device="COM5".encode("ascii"), 
#     baud=2250000, 
#     parity="N".encode("ascii"), 
#     data_bit=8, 
#     stop_bit=1
# )

# #Read and set timeout
# timeout_sec = client.get_response_timeout()
# print(timeout_sec)
# client.set_response_timeout(timeout_sec+1)

# #Connect
# client.connect()

# SERVER_ID=5
# BCM_PIN_DE=18
# BCM_PIN_RE=17

# #Set Slave ID number
# client.set_slave(SERVER_ID)

# #Enable RPi GPIO Functions
# client.enable_rpi(1)

# #Define pin numbers to be used as Read Enable (RE) and Drive Enable (DE)
# client.configure_rpi_bcm_pins(BCM_PIN_DE,BCM_PIN_RE)

# #Export pin direction (set as outputs)
# client.rpi_pin_export_direction()

# #Write Modbus registers, 10 starting from 0
# #client.write_registers(0, [0]*10)

# #Read 10 input registers starting from number 0
# result=(client.read_registers(0, 1))

# #Show register values
# print(result[0])

# #Release pins and close connection
# client.rpi_pin_unexport_direction()
# client.close()
