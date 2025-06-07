import serial


class DataComm:
    """
    此类初始化两个字典，control_button_hex_dict和normal_button_hex_dict，它们包含键盘上控制和普通按钮的十六进制值。
    如果你需要更多的按钮，请自行根据协议文档补充
    """

    def __init__(self):
        self.control_button_hex_dict = {"NULL": b'\x00',
                                "R_WIN": b"\x80",
                                "R_ALT": b"\x40",
                                "R_SHIFT": b"\x20",
                                "R_CTRL": b"\x10",
                                "L_WIN": b"\x08",
                                "L_ALT": b"\x04",
                                "L_SHIFT": b"\x02",
                                "L_CTRL": b"\x01"
                                }
        self.normal_button_hex_dict = {
            # Alphabet keys (unchanged)
            "AA": b'\x04', "BB": b'\x05', "CC": b'\x06', "DD": b'\x07', "EE": b'\x08', 
            "FF": b'\x09', "GG": b'\x0A', "HH": b'\x0B', "II": b'\x0C', "JJ": b'\x0D',
            "KK": b'\x0E', "LL": b'\x0F', "MM": b'\x10', "NN": b'\x11', "OO": b'\x12',
            "PP": b'\x13', "QQ": b'\x14', "RR": b'\x15', "SS": b'\x16', "TT": b'\x17',
            "UU": b'\x18', "VV": b'\x19', "WW": b'\x1A', "XX": b'\x1B', "YY": b'\x1C',
            "ZZ": b'\x1D', "NU": b'\x00',
            
            # Digits (unchanged)
            "11": b'\x1E', "22": b'\x1F', "33": b'\x20', "44": b'\x21', "55": b'\x22',
            "66": b'\x23', "77": b'\x24', "88": b'\x25', "99": b'\x26', "00": b'\x27',
            
            # Special keys (corrected based on your feedback)
            "EN": b'\x28',  # Enter
            "ES": b'\x29',  # Escape
            "BA": b'\x2A',  # Backspace (was BK)
            "TB": b'\x2B',  # Tab
            "SP": b'\x2C',  # Space
            "MI": b'\x2D',  # Minus
            "EQ": b'\x2E',  # Equal
            "LB": b'\x2F',  # Left Bracket (was LBR)
            "RB": b'\x30',  # Right Bracket (was RBR)
            "BS": b'\x31',  # Backslash (was BK)
            "SC": b'\x33',  # Semicolon
            "SQ": b'\x34',  # Single Quote
            "GR": b'\x35',  # Grave
            "CM": b'\x36',  # Comma
            "DT": b'\x37',  # Dot
            "SL": b'\x38',  # Slash (was BK)
            "CL": b'\x39',  # Caps Lock
            
            # Function keys (unchanged)
            "F1": b'\x3A', "F2": b'\x3B', "F3": b'\x3C', "F4": b'\x3D', "F5": b'\x3E',
            "F6": b'\x3F', "F7": b'\x40', "F8": b'\x41', "F9": b'\x42', "F10": b'\x43',
            "F11": b'\x44', "F12": b'\x45',
            
            # System keys
            "PS": b'\x46',  # Print Screen
            "SL": b'\x47',  # Scroll Lock (was SLCK)
            "PB": b'\x48',  # Pause/Break (was PA)
            
            # Navigation keys (corrected based on your feedback)
            "IN": b'\x49',  # Insert (was INS)
            "HM": b'\x4A',  # Home
            "PU": b'\x4B',  # Page Up (was PGUP)
            "DE": b'\x4C',  # Delete (was DEL)
            "END": b'\x4D',  # End
            "PD": b'\x4E',  # Page Down (was PGDN)
            "RA": b'\x4F',  # Right Arrow (was RGHT)
            "LA": b'\x50',  # Left Arrow (was LEFT)
            "DA": b'\x51',  # Down Arrow (was DOWN)
            "UA": b'\x52',  # Up Arrow (was UP)
            
            # Numpad keys
            "NL": b'\x53',   # Num Lock
            "KD": b'\x54',   # Numpad / (was K/)
            "KM": b'\x55',   # Numpad * (was K*)
            "K-": b'\x56',   # Numpad - (was K-)
            "K+": b'\x57',   # Numpad + (was K+)
            "KE": b'\x58',   # Numpad Enter (was KEN)
            "K1": b'\x59',   # Numpad End
            "K2": b'\x5A',   # Numpad Down
            "K3": b'\x5B',   # Numpad PageDn
            "K4": b'\x5C',   # Numpad Left
            "K5": b'\x5D',   # Numpad Clear
            "K6": b'\x5E',   # Numpad Right
            "K7": b'\x5F',   # Numpad Home
            "K8": b'\x60',   # Numpad Up
            "K9": b'\x61',   # Numpad PageUp
            "K0": b'\x62',   # Numpad Insert
            "K.": b'\x63',   # Numpad Delete
            "KA": b'\x65'    # Numpad Apps (was KAPP)
        }
    


    """
    发送数据到串口。
    
    参数:
        data (str): 要发送的按键信息。
        ctrl (str): 要发送的控制键。
        port (serial): 要发送数据的串口。
    
    返回:
        bool: 如果数据成功发送，则为True，否则为False。
    """

    def send_data(self, data, ctrl='', port=serial):
        # 将字符转写为数据包
        HEAD = b'\x57\xAB'  # 帧头
        ADDR = b'\x00'  # 地址
        CMD = b'\x02'  # 命令
        LEN = b'\x08'  # 数据长度
        DATA = b''  # 数据

        # 控制键
        if ctrl == '':
            DATA += b'\x00'
        elif isinstance(ctrl, int):
            DATA += bytes([ctrl])
        else:
            DATA += self.control_button_hex_dict[ctrl]

        # DATA固定码
        DATA += b'\x00'

        # 读入data
        for i in range(0, len(data), 2):
            DATA += self.normal_button_hex_dict[data[i:i + 2]]
        if len(DATA) < 8:
            DATA += b'\x00' * (8 - len(DATA))
        else:
            DATA = DATA[:8]

        # 分离HEAD中的值，并计算和
        HEAD_hex_list = []
        for byte in HEAD:
            HEAD_hex_list.append(byte)
        HEAD_add_hex_list = sum(HEAD_hex_list)

        # 分离DATA中的值，并计算和
        DATA_hex_list = []
        for byte in DATA:
            DATA_hex_list.append(byte)
        DATA_add_hex_list = sum(DATA_hex_list)

        #
        try:
            SUM = sum([HEAD_add_hex_list, int.from_bytes(ADDR, byteorder='big'),
                       int.from_bytes(CMD, byteorder='big'), int.from_bytes(LEN, byteorder='big'),
                       DATA_add_hex_list]) % 256  # 校验和
        except OverflowError:
            print("int too big to convert")
            return False
        packet = HEAD + ADDR + CMD + LEN + DATA + bytes([SUM])  # 数据包
        port.ser.write(packet)  # 将命令代码写入串口
        return True  # 如果成功，则返回True，否则引发异常

    """
    释放按钮。
    
    参数:
        serial (object): 用于发送数据的串行对象。
    
    返回:
        None
    """

    def release(serial):
        serial.send_data('')
