# RS04 驱动器说明书

## 运行模式设置（位置模式）
- **控制模式**：位置模式（CSP）
- **速度设置**：5.0 rad/s（默认值）
- **角度指令**：0.0 rad

### 操作步骤
1. 控制模式切换为位置模式
2. 电机运行，进入 `motor_mode` 模式
3. 设置速度（未设置则按默认值运行），最后设置位置指令（目标位置）
4. 点击停止按钮，电机停止运行

---

## 正弦信号测试
| 参数       | 值   | 操作 |
|------------|------|------|
| 幅值       | 1.0  | 确定 |
| 频率（Hz） | 1.0  | 启动 |

### 电机位置正弦测试步骤
1. 控制模式切换为位置模式
2. 电机运行，进入 `motor_mode` 模式
3. 设置幅值和频率 → 点击“确定” → 点击“启动”（目标指令按正弦规律规划）
4. 点击停止按钮，电机停止运行

---

## 驱动器协议说明
### 通信基础
- **接口**：CAN 2.0
- **波特率**：1 Mbps
- **帧格式**：扩展帧（29位ID）

#### 帧结构
| 数据域     | 29位ID              | 8Byte数据区 |
|------------|---------------------|-------------|
| **描述**   | Bit28~24: 通信类型  | Byte0~Byte7 |
|            | Bit23~8: 数据区2    |             |
|            | Bit7~0: 目标地址    |             |

### 支持的控制模式
- 运控模式（给定5个参数）
- 电流模式（给定Iq电流）
- 速度模式（给定运行速度）
- 位置模式（给定目标位置）

---

## 通信协议详解
### 通信类型0：获取设备ID
**请求帧**：
| 29位ID          | 数据区           |
|-----------------|------------------|
| `0x0`           | 主机CAN_ID (Bit15~8) |
| 目标电机CAN_ID   | `0` (填充)       |

**应答帧**：
| 29位ID          | 数据区              |
|-----------------|---------------------|
| `0x0`           | 目标电机CAN_ID      |
| `0xFE`          | 64位MCU唯一标识符   |

---

### 通信类型1：运控模式控制指令
**请求帧**：
| 29位ID | 数据区                          | 内容说明                     |
|--------|---------------------------------|------------------------------|
| `0x1`  | Byte2: 转矩值 (0~65535 → -120~120Nm) | 目标电机CAN_ID + 8Byte控制参数 |

**应答**：电机反馈帧（通信类型2）

---

### 通信类型2：电机反馈数据
**帧结构**：
| 29位ID | 数据域（Bit23~8）                                                                 | 主机CAN_ID | 数据区（Byte0~7）                     |
|--------|-----------------------------------------------------------------------------------|------------|----------------------------------------|
| `0x2`  | - Bit8~15: 当前电机CAN_ID<br>- Bit16~21: 故障标志<br>- Bit22~23: 模式状态（0:Reset, 1:Call, 2:Motor） | 主机ID     | Byte2: 转矩<br>Byte6~7: 温度（×10）    |

> **故障标志**：  
> - Bit16: 欠压故障  
> - Bit17: 驱动故障  
> - Bit18: 过速  
> - Bit19: 超温故障  
> - Bit20: 过载故障  
> - Bit21: 未标定  

---

### 通信类型3：电机使能运行
**请求帧**：
| 29位ID | 数据区          | 内容   |
|--------|-----------------|--------|
| `0x3`  | 主机CAN_ID (Bit15~8) | 目标电机CAN_ID |

**应答**：电机反馈帧（通信类型2）

⽌运⾏
RS04-CHN 25
数据域 29位ID 8Byte数据区
⼤⼩ Bit28~bit24 bit23~8 bit7~0 Byte0~Byte7
描述 0x4 bit15~8:⽤来标识主
CAN_ID ⽬标电机CAN_ID
正常运⾏时，data 区需清0；
Byte[0]=1时：清故障；

---

### 其他关键通信类型
| 类型 | 功能               | 关键参数               |
|------|--------------------|------------------------|
| 4    | 电机停止运行       | Byte[0]=1: 清故障      |
| 6    | 设置电机机械零位   | Byte[0]=1             |
| 7    | 设置电机CAN_ID     | Byte[0]=1             |
| 17   | 单个参数读取       | Byte0~1: 参数Index     |
| 18   | 单个参数写入       | Byte0~1: Index + 写入值 |
| 22   | 保存所有参数       | 数据区固定为 `01 02 03...` |
| 24   | 开启主动上报       | F_CMD=01 (开启)        |

---

## 可读写参数列表
| Index   | 参数名称       | 描述                          | 类型   | 单位/范围        | R/W  |
|---------|----------------|-------------------------------|--------|------------------|------|
| 0X7005  | `run_mode`     | 控制模式 (0:运控, 1:PP位置...) | uint8  | -                | W/R  |
| 0X7006  | `id_ref`       | 电流模式指令                  | float  | -90~90A          | W/R  |
| 0X7016  | `loc_ref`      | 位置模式角度指令              | float  | rad              | W/R  |
| 0X7019  | `mechPos`      | 负载端机械角度                | float  | rad              | R    |
| **完整列表见文档第5-6页** |  |  |  |  |  |

---

## 功能说明
### 主动上报
- 默认关闭，通过**类型24**开启（`F_CMD=01`）
- 上报间隔默认10ms，可通过修改 `EPScan_time` 参数调整

### 零点标定
- **CSP/运控模式**：标零后电机期望值立即归零
- **PP模式**：不支持标零
- 通过 `zero_sta` 标志位设置零点范围（0: 0~2π；1: -π~π）

### 协议切换
- 通过 **类型25** 切换协议（0:私有协议, 1:Canopen, 2:MT）
- **注意**：切换后需重新上电生效

---




通信类型0：获取设备ID
获取设备的ID和64位MCU唯⼀标识符
数据域 29位ID 8Byte数据区
⼤⼩ Bit28~bit24 bit23~8 bit7~0 Byte0~Byte7
描述 0x0 bit15~8:⽤来标识主
机CAN_ID ⽬标电机CAN_ID 0
应答帧：
数据域 29位ID 8Byte数据区
⼤⼩ Bit28~bit24 bit23~8 bit7~0 Byte0~Byte7
描述 0x0 ⽬标电机CAN_ID 0XFE 64位MCU唯⼀标识
符


通信类型1：运控模式电机控制指令
数据域 29位ID 8Byte数据区
⼤⼩ Bit28~bit24 bit23~8 bit7~0 Byte0~Byte7
描述 0x1
Byte2:⼒矩
(0~65535) 对应
（-120Nm~120Nm）
⽬标电机CAN _ID
Byte0~1: ⽬标⻆度
[0~65535]对应
(-12.57f~12.57f)
Byte2~3: ⽬标⻆速
度[0~65535]对应
(-15rad/s~15rad/s)
Byte4~5：Kp
[0~65535]对应
(0.0~5000.0)
Byte6~7：Kd
[0~65535]对应
(0.0~100.0) 以上数
据转换后⾼字节在
前，低字节在后

通信类型2：电机反馈数据
数据域 29位ID 8Byte数据区
⼤⼩ Bit28~bit24 bit23~8 bit7~0 Byte0~Byte7
描述 0x2
Bit8~Bit15:当前电
机CAN ID
bit21~16:故障信息
（0⽆ 1有）
bit21: 未标定
bit20: 堵转过载故
障
bit19: 磁编码故障
bit18: 过温
bit17: 驱动故障
bit16: ⽋压故障
bit22~23: 模式状态 0 : Reset 模式[复位]
1 : Cali 模式[标定]
2 : Motor模式[运
⾏]
主机CAN _ID
Byte0~1: 当前⻆度
[0~65535]对
应-12.57f~12.57f)
Byte2~3: 当前⻆速度
[0~65535]对应
(-15rad/s~15rad/s)
Byte4~5:当前⼒矩
[0~65535]对应
（-120Nm~120Nm）
Byte6~7:当前温度：
Temp(摄⽒度）*10
以上数据⾼字节在
前，低字节在后

通信类型3：电机使能运⾏
数据域 29位ID 8Byte数据区
⼤⼩ Bit28~bit24 bit23~8 bit7~0 Byte0~Byte7
描述 0x3 bit15~8:⽤来标识主
CAN_ID ⽬标电机CAN_ID
应答帧：应答电机反馈帧(⻅通信类型2)

通信类型4：电机停





## 控制模式示例代码（C语言）
### 电机使能运行（类型3）
```c
void motor_enable(uint8_t id, uint16_t master_id) {
    txCanIdEx.mode = 3;
    txCanIdEx.id = id;
    txCanIdEx.data = master_id;
    can_txd();
}






Configuring your CAN bus
Setting up your CAN bus depends on how you're commuinicating with CAN. If you're using a Canable compatible USB-CAN device, you can set up your can bus as follows:

# Configure the can0 device to use 1Mbps
sudo ip link set can0 type can bitrate 1000000 loopback off

# Enable can0
sudo ifconfig can0 up
The simplest way to use this library is to configure your CAN setup in the ~/.can file, which if you're using the setup above looks like this:

[default]
interface = socketcan
channel = can0
You can also configure these manually when creating the can.Bus object in code. For the robstride CLI client you can configure these values with the optional --interface and --channel CLI arguments.




gpt 翻译

RS04 驱动器使用说明书
1. 控制模式操作指南
1.1 位置模式控制

切换控制模式为 位置模式

使能电机，进入 motor_mode

设置 速度（可选，默认值运行）

设置 位置指令（目标位置）

点击 停止 → 电机停止运行

1.2 电机位置正弦测试

切换控制模式为 位置模式

电机进入 motor_mode

设置 幅值 和 频率 → 点击启动
电机将按正弦规律运行

点击 停止 → 电机停止运行

2. 驱动器通信协议
2.1 通信总览

协议类型：私有协议

通信方式：CAN 2.0

波特率：1 Mbps

帧类型：扩展帧

数据格式：

29-bit ID | 8 Byte Data
┌─────────┬─────────────┬─────────────┐
│ Bit28~24│ Bit23~8     │ Bit7~0      │
│ 通信类型 │ 目标地址    │ 数据内容    │
└─────────┴─────────────┴─────────────┘

2.2 支持的控制模式
模式类型	功能说明
运控模式	提供电机的 5 个运控参数
电流模式	给定指定 Iq 电流
速度模式	给定电机指定速度
位置模式	给定目标位置，电机自动运动到达
3. 通信类型定义
3.1 获取设备 ID（通信类型 0）

用途：读取设备 ID 和 64 位 MCU 唯一标识符。

发送帧
29bit ID: [0x0]
Byte0-7: 设备请求

应答帧
29bit ID: [0x0]
Byte0-7: MCU 唯一 ID

3.2 运控模式电机控制指令（通信类型 1）

功能：向电机发送目标角度、速度、力矩、Kp、Kd。

字节	内容	范围	说明
0-1	目标角度	0 ~ 65535	对应 -12.57 ~ 12.57 rad
2-3	目标速度	0 ~ 65535	对应 -15 ~ 15 rad/s
4-5	Kp	0 ~ 65535	对应 0.0 ~ 5000.0
6-7	Kd	0 ~ 65535	对应 0.0 ~ 100.0

应答帧：参考通信类型 2。

3.3 电机反馈数据（通信类型 2）
位段	内容	说明
Bit8-15	当前电机 CAN ID	
Bit16	欠压故障	
Bit17	驱动故障	
Bit18	过温故障	
Bit19	编码器故障	
Bit20	堵转过载	
Bit21	未标定	
Bit22-23	模式状态：0=Reset，1=Cali，2=Motor	
字节	内容	范围	单位
0-1	当前角度	0~65535	-12.57 ~ 12.57 rad
2-3	当前速度	0~65535	-15 ~ 15 rad/s
4-5	当前力矩	0~65535	-120 ~ 120 Nm
6-7	温度	温度×10	℃
3.4 电机使能（通信类型 3）

作用：启动电机进入运行状态。

发送帧
txCanIdEx.mode = 3;
txCanIdEx.id = 电机ID;
txCanIdEx.data = master_id;
can_txd();


应答帧：通信类型 2。

3.5 电机停止（通信类型 4）

作用：停止电机运行。
如需清除故障，Byte[0] = 1。

3.6 设置机械零位（通信类型 6）

设置电机的当前角度为机械零点。

3.7 设置电机 CAN_ID（通信类型 7）

支持修改电机 ID，需重新上电生效。

4. 参数索引表（可读写）
Index	参数名称	描述	类型	范围	读写
0x7005	run_mode	模式选择	uint8	0-5	W/R
0x7006	iq_ref	电流指令	float	-90~90A	W/R
0x700A	spd_ref	速度指令	float	-20~20rad/s	W/R
0x7016	loc_ref	位置指令	float	rad	W/R
0x7017	limit_spd	CSP模式速度限制	float	0~20rad/s	W/R
0x7018	limit_cur	电流限制	float	0~90A	W/R
0x701E	loc_kp	位置 Kp	float	默认 80	W/R
0x701F	spd_kp	速度 Kp	float	默认 6	W/R
0x7020	spd_ki	速度 Ki	float	默认 0.02	W/R
5. 电机控制示例
5.1 使能电机
void motor_enable(uint8_t id, uint16_t master_id)
{
    txCanIdEx.mode = 3;
    txCanIdEx.id = id;
    txCanIdEx.data = master_id;
    txMsg.tx_dlen = 8;
    can_txd();
}

5.2 运控模式控制
void motor_control(uint8_t id, float torque, float pos, float speed, float kp, float kd)
{
    txCanIdEx.mode = 1;
    txCanIdEx.id = id;
    txCanIdEx.data = float_to_uint(torque, T_MIN, T_MAX, 16);
    txMsg.tx_dlen = 8;

    txMsg.tx_data[0] = float_to_uint(pos, P_MIN, P_MAX, 16) >> 8;
    txMsg.tx_data[1] = float_to_uint(pos, P_MIN, P_MAX, 16);
    txMsg.tx_data[2] = float_to_uint(speed, V_MIN, V_MAX, 16) >> 8;
    txMsg.tx_data[3] = float_to_uint(speed, V_MIN, V_MAX, 16);
    txMsg.tx_data[4] = float_to_uint(kp, KP_MIN, KP_MAX, 16) >> 8;
    txMsg.tx_data[5] = float_to_uint(kp, KP_MIN, KP_MAX, 16);
    txMsg.tx_data[6] = float_to_uint(kd, KD_MIN, KD_MAX, 16) >> 8;
    txMsg.tx_data[7] = float_to_uint(kd, KD_MIN, KD_MAX, 16);

    can_txd();
}

6. 运行模式切换流程
模式	run_mode 值	指令流程
运控模式	0	使能 → 控制指令
位置模式 CSP	5	使能 → 设置 limit_spd → loc_ref
位置模式 PP	1	使能 → 设置 vel_max → acc_set → loc_ref
速度模式	2	使能 → 设置 acc_rad → spd_ref
电流模式	3	使能 → 设置 iq_ref