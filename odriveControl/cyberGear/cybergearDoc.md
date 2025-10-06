# Motor Driver Communication Protocol

## 4. Driver communication protocol and usage instructions

Motor communication is CAN 2.0 communication interface, with a baud rate of 1Mbps and an extended frame format, as shown:

| Data field | 29-bit ID |  |  | 8 Byte data area |
|------------|-----------|--|--|------------------|
| **Size** | Bit 28 ~ 24 | Bit 23 ~ 8 | Bit 7 ~ 0 | Byte 0 ~ 7 |
| **Description** | Communication type | Data area 2 | Target address | Data area 1 |

The control modes supported by the motor include:

- Operation control mode: given 5 parameters for motor operation control
- Current mode: given the specified Iq current of the motor
- Speed mode: given the specified operating speed of the motor
- Position mode: Given a specified position of the motor, the motor will run to the specified position

## 4.1 Communication protocol type description

### 4.1.1 Get device ID (communication type 0)

Get the device's ID and 64-bit MCU unique identifier

**Request frame:**

| Data field | 29-bit ID |  |  | 8 byte data area |
|------------|-----------|--|--|------------------|
| **Position** | Bit 28 ~ 24 | Bit 23 ~ 8 | Bit 7 ~ 0 | Byte 0 ~ 7 |
| **Description** | 0 | Bit 15 ~ 8: Host CAN_ID | Motor CAN_ID | 0 |

**Response frame:**

| Data field | 29-bit ID |  |  | 8 byte data area |
|------------|-----------|--|--|------------------|
| **Position** | Bit 28 ~ 24 | Bit 23 ~ 8 | Bit 7 ~ 0 | Byte 0 ~ 7 |
| **Description** | 0 | Motor CAN_ID | 0xFE | 64-bit MCU unique identifier |

### 4.1.2 Motor control instructions (communication type 1)

Used to send control instructions to the motor in operation control mode.

**Request frame:**

| Data field | 29-bit ID |  |  | 8 byte data area |
|------------|-----------|--|--|------------------|
| **Position** | Bit 28 ~ 24 | Bit 23 ~ 8 | Bit 7 ~ 0 | Byte 0 ~ 7 |
| **Description** | 1 | Byte 2: Torque (0 ~ 65535) corresponding to (-12Nm ~ 12Nm) | Motor CAN_ID | Byte 0 ~ 1: Target angle [0 ~ 65535] corresponding to (-4π ~ 4π)<br>Byte 2 ~ 3: Target angular velocity [0 ~ 65535] corresponds to (-30rad/s ~ 30rad/s)<br>Byte 4 ~ 5: Kp [0 ~ 65535] corresponds to (0.0 ~ 500.0)<br>Byte 6 ~ 7: Kd [0 ~ 65535] corresponds to (0.0 ~ 5.0) |

**Response frame:** Reply motor feedback frame (see communication type 2)

### 4.1.3 Motor feedback data (communication type 2)

Used to feedback the motor operating status to the host

| Data field | 29-bit ID |  |  | 8 byte data area |
|------------|-----------|--|--|------------------|
| **Position** | Bit 28 ~ 24 | Bit 23 ~ 8 | Bit 7 ~ 0 | Byte 0 ~ 7 |
| **Description** | 2 | Bit 8 ~ 15: Motor CAN ID<br>Bit 21 ~ 16: Fault information (0 - No, 1 - Yes)<br>- Bit 21: not calibrated<br>- Bit 20: HALL encoding failure<br>- Bit 19: Magnetic encoding failure<br>- Bit 18: over temperature<br>- Bit 17: overcurrent<br>- Bit 16: Undervoltage fault<br>Bit 22 ~ 23: mode status:<br>0: Reset mode [reset]<br>1: Cali mode [Calibration]<br>2: Motor mode [Run] | Host CAN_ID | Byte 0 ~ 1: Current angle [0 ~ 65535] corresponds to (-4π ~ 4π)<br>Byte 2 ~ 3: Current angular velocity [0 ~ 65535] corresponds to (-30rad/s ~ 30rad/s)<br>Byte 4 ~ 5: Current torque [0 ~ 65535] corresponds to (-12Nm ~ 12Nm)<br>Byte 6 ~ 7: Current temperature: Temp (degrees Celsius)*10 |

### 4.1.4 Motor enable operation (communication type 3)

| Data field | 29-bit ID |  |  | 8 byte data area |
|------------|-----------|--|--|------------------|
| **Position** | Bit 28 ~ 24 | Bit 23 ~ 8 | Bit 7 ~ 0 | Byte 0 ~ 7 |
| **Description** | 3 | Bit 15 ~ 8: Host CAN_ID | Motor CAN_ID |  |

**Reply frame:** Reply motor feedback frame (see communication type 2)

### 4.1.5 Motor stopped (communication type 4)

| Data field | 29-bit ID |  |  | 8 byte data area |
|------------|-----------|--|--|------------------|
| **Position** | Bit 28 ~ 24 | Bit 23 ~ 8 | Bit 7 ~ 0 | Byte 0 ~ 7 |
| **Description** | 4 | Bit 15 ~ 8: Host CAN_ID | Motor CAN_ID | During normal operation, the data area needs to be cleared to 0;<br>When Byte[0]=1: Clear fault; |

**Reply frame:** Reply motor feedback frame (see communication type 2)

### 4.1.6 Setting the mechanical zero position of the motor (communication type 6)

Will set the current motor position to the mechanical zero position (lost after power failure)

| Data field | 29-bit ID |  |  | 8 byte data area |
|------------|-----------|--|--|------------------|
| **Position** | Bit 28 ~ 24 | Bit 23 ~ 8 | Bit 7 ~ 0 | Byte 0 ~ 7 |
| **Description** | 6 | Bit 15 ~ 8: Host CAN_ID | Motor CAN_ID | Byte[0]=1 |

**Reply frame:** Reply motor feedback frame (see communication type 2)

### 4.1.7 Set motor CAN_ID (communication type 7)

Change the current motor CAN_ID, which will take effect immediately.

| Data field | 29-bit ID |  |  | 8 byte data area |
|------------|-----------|--|--|------------------|
| **Position** | Bit 28 ~ 24 | Bit 23 ~ 8 | Bit 7 ~ 0 | Byte 0 ~ 7 |
| **Description** | 7 | Bit 15 ~ 8: Host CAN_ID<br>Bit 16 ~ 23: New motor CAN_ID | Motor CAN_ID | Byte[0]=1 |

**Reply frame:** Reply motor broadcast frame (see communication type 0)

### 4.1.8 Single parameter reading (communication type 17)

**Request frame:**

| Data field | 29-bit ID |  |  | 8 byte data area |
|------------|-----------|--|--|------------------|
| **Position** | Bit 28 ~ 24 | Bit 23 ~ 8 | Bit 7 ~ 0 | Byte 0 ~ 7 |
| **Description** | 17 | Bit 15 ~ 8: Host CAN_ID | Motor CAN_ID | Byte 0 ~ 1: index, see 4.1.11 for parameter list<br>Byte 2 ~ 3: 00<br>Byte 4 ~ 7: 00 |

**Response frame:**

| Data field | 29-bit ID |  |  | 8 byte data area |
|------------|-----------|--|--|------------------|
| **Position** | Bit 28 ~ 24 | Bit 23 ~ 8 | Bit 7 ~ 0 | Byte 0 ~ 7 |
| **Description** | 17 | Bit 15 ~ 8: Motor CAN_ID | Host CAN_ID | Byte 0 ~ 1: index, for parameter list, see 4.1.11<br>Byte 2 ~ 3: 00<br>Byte 4 ~ 7: parameter data, 1 byte data is in Byte 4 |

### 4.1.9 Single parameter writing (communication type 18)

(lost after power failure)

| Data field | 29-bit ID |  |  | 8 byte data area |
|------------|-----------|--|--|------------------|
| **Position** | Bit 28 ~ 24 | Bit 23 ~ 8 | Bit 7 ~ 0 | Byte 0 ~ 7 |
| **Description** | 18 | Bit 15 ~ 8: Host CAN_ID | Motor CAN_ID | Byte 0 ~ 1: index, see 4.1.11 for parameter list details<br>Byte 2 ~ 3: 00<br>Byte 4 ~ 7: parameter data |

**Reply frame:** Reply motor feedback frame (see communication type 2)

### 4.1.10 Fault feedback frame (communication type 21)

| Data field | 29-bit ID |  |  | 8 byte data area |
|------------|-----------|--|--|------------------|
| **Position** | Bit 28 ~ 24 | Bit 23 ~ 8 | Bit 7 ~ 0 | Byte 0 ~ 7 |
| **Description** | 21 | Bit 15 ~ 8: Host CAN_ID | Motor CAN_ID | Byte 0 ~ 3: fault value (not 0: fault, 0: normal)<br>- Bit 16: A phase current sampling overcurrent<br>- Bit 15 ~ 8: overload fault<br>- Bit 7: encoder not calibrated<br>- Bit 5: C phase current sampling overcurrent<br>- Bit 4: B phase current sampling overcurrent<br>- Bit 3: overvoltage fault<br>- Bit 2: under-voltage fault<br>- Bit 1: driver chip fault<br>- Bit 0: motor over-temperature fault, default 80 degrees<br>Byte 4 ~ 7: warning value<br>- Bit 0: motor over-temperature warning, default 75 degrees |

### 4.1.11 Baud rate modification (communication type 22)

(version 1.2.1.5 can be modified, please refer to the document process to modify it carefully. Operation errors may cause problems such as being unable to connect to the motor and being unable to upgrade)

| Data field | 29-bit ID |  |  | 8 byte data area |
|------------|-----------|--|--|------------------|
| **Position** | Bit 28 ~ 24 | Bit 23 ~ 8 | Bit 7 ~ 0 | Byte 0 ~ 7 |
| **Description** | 22 | Bit 15 ~ 8: Host CAN_ID | Motor CAN_ID | Byte0: Motor baud rate<br>1: 1Mbps<br>2: 500kbps<br>3: 250kbps<br>4: 125kbps |

**Reply frame:** Reply motor broadcast frame (see communication type 0)

### 4.1.12 Readable and writable single parameter list

(7019-7020 is readable by firmware version 1.2.1.5)

| Parameter index | parameter name | describe | type | Number of bytes | Possible values | Permission |
|-----------------|----------------|----------|------|-----------------|-----------------|------------|
| 0x7005 | run_mode | 0: Operation control mode<br>1: Position mode<br>2: Speed mode<br>3: Current mode | uint8 | 1 |  | W/R |
| 0x7006 | iq_ref | Current Mode Iq Command | float | 4 | -23 ~ 23A | W/R |
| 0x700A | spd_ref | Speed mode speed command | float | 4 | -30 ~ 30rad/s | W/R |
| 0x700B | limit_torque | Torque limit | float | 4 | 0~12Nm | W/R |
| 0x7010 | cur_kp | Kp of current | float | 4 | Default value 0.125 | W/R |
| 0x7011 | cur_ki | Current Ki | float | 4 | Default value 0.0158 | W/R |
| 0x7014 | cur_filt_gain | Current filter coefficient filt_gain | float | 4 | 0~1.0, default value 0.1 | W/R |
| 0x7016 | loc_ref | Position mode angle command | float | 4 | rad | W/R |
| 0x7017 | limit_spd | Position mode speed limit | float | 4 | 0 ~ 30rad/s | W/R |
| 0x7018 | limit_cur | Speed Position Mode Current Limit | float | 4 | 0 ~ 23A | W/R |
| 0x7019 | mechPos | Load end lap counting mechanical angle | float | 4 | rad | R |
| 0x701A | iqf | iq filter value | float | 4 | -23 ~ 23A | R |
| 0x701B | mechVel | Load end speed | float | 4 | -30 ~ 30rad/s | R |
| 0x701C | VBUS | bus voltage | float | 4 | V | R |
| 0x701D | rotation | Number of turns | int16 | 2 | Number of turns | W/R |
| 0x701E | loc_kp | kp of position | float | 4 | Default value 30 | W/R |
| 0x701F | spd_kp | Speed in kp | float | 4 | Default value 1 | W/R |
| 0x7020 | spd_ki | Speed of ki | float | 4 | Default value 0.002 | W/R |

## 4.2 Instructions for use of control mode

### 4.2.1 Program sample

The following provides examples of controlling motors in various modes (taking gd32f303 as an example). The following calls libraries, functions and macro definitions for various examples.

```c
#define P_MIN -12.5f
#define P_MAX 12.5f
#define V_MIN -30.0f
#define V_MAX 30.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define T_MIN -12.0f
#define T_MAX 12.0f

struct exCanIdInfo{
    uint32_t id:8;
    uint32_t data:16;
    uint32_t mode:5;
    uint32_t res:3;
};

can_receive_message_struct rxMsg;

can_trasnmit_message_struct txMsg = {
    .tx_sfid = 0,
    .tx_efid = 0xff,
    .tx_ft = CAN_FT_DATA,
    .tx_ff = CAN_FF_EXTENDED,
    .tx_dlen = 8,
};

#define txCanIdEx (((struct exCanIdInfo)&(txMsg.tx_efid)))

// Parse extended frame id into custom data structure
#define rxCanIdEx (((struct exCanIdInfo)&(rxMsg.rx_efid)))

int float_to_uint(float x, float x_min, float x_max, int bits) {
    float span = x_max - x_min;
    float offset = x_min;
    if(x > x_max) x=x_max;
    else if(x < x_min) x= x_min;
    return (int) ((x-offset)*((float)((1<<bits)-1))/span);
}

#define can_txd() can_message_transmit(CAN0, &txMsg)
#define can_rxd() can_message_receive(CAN0, CAN_FIFO1, &rxMsg)
```

Common communication types are listed below:

**Motor enable running frame (communication type 3)**

```c
void motor_enable(uint8_t id, uint16_t master_id) {
    txCanIdEx.mode = 3;
    txCanIdEx.id = id;
    txCanIdEx.res = 0;
    txCanIdEx.data = master_id;
    txMsg.tx_dlen = 8;
    txCanIdEx.data = 0;
    can_txd();
}
```

**Operation control mode motor control instructions (communication type 1)**

```c
void motor_controlmode(uint8_t id, float torque, float MechPosition, float speed, float kp, float kd) {
    txCanIdEx.mode = 1;
    txCanIdEx.id = id;
    txCanIdEx.res = 0;
    txCanIdEx.data = float_to_uint(torque,T_MIN,T_MAX,16);
    txMsg.tx_dlen = 8;
    txMsg.tx_data[0]=float_to_uint(MechPosition,P_MIN,P_MAX,16)>>8;
    txMsg.tx_data[1]=float_to_uint(MechPosition,P_MIN,P_MAX,16);
    txMsg.tx_data[2]=float_to_uint(speed,V_MIN,V_MAX,16)>>8;
    txMsg.tx_data[3]=float_to_uint(speed,V_MIN,V_MAX,16);
    txMsg.tx_data[4]=float_to_uint(kp,KP_MIN,KP_MAX,16)>>8;
    txMsg.tx_data[5]=float_to_uint(kp,KP_MIN,KP_MAX,16);
    txMsg.tx_data[6]=float_to_uint(kd,KD_MIN,KD_MAX,16)>>8;
    txMsg.tx_data[7]=float_to_uint(kd,KD_MIN,KD_MAX,16);
    can_txd();
}
```

**Motor stop running frame (communication type 4)**

```c
void motor_reset(uint8_t id, uint16_t master_id) {
    txCanIdEx.mode = 4;
    txCanIdEx.id = id;
    txCanIdEx.res = 0;
    txCanIdEx.data = master_id;
    txMsg.tx_dlen = 8;
    for(uint8_t i=0;i<8;i++) {
        txMsg.tx_data[i]=0;
    }
    can_txd();
}
```

**Motor mode parameter write command (communication type 18, operating mode switching)**

```c
uint8_t runmode;
uint16_t index;
void motor_modechange(uint8_t id, uint16_t master_id) {
    txCanIdEx.mode = 0x12;
    txCanIdEx.id = id;
    txCanIdEx.res = 0;
    txCanIdEx.data = master_id;
    txMsg.tx_dlen = 8;
    for(uint8_t i=0;i<8;i++) {
        txMsg.tx_data[i]=0;
    }
    memcpy(&txMsg.tx_data[0],&index,2);
    memcpy(&txMsg.tx_data[4],&runmode, 1);
    can_txd();
}
```

**Motor mode parameter write command (communication type 18, control parameter write)**

```c
uint16_t index;
float ref;
void motor_write(uint8_t id, uint16_t master_id) {
    txCanIdEx.mode = 0x12;
    txCanIdEx.id = id;
    txCanIdEx.res = 0;
    txCanIdEx.data = master_id;
    txMsg.tx_dlen = 8;
    for(uint8_t i=0;i<8;i++) {
        txMsg.tx_data[i]=0;
    }
    memcpy(&txMsg.tx_data[0],&index,2);
    memcpy(&txMsg.tx_data[4],&ref,4);
    can_txd();
}
```