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

