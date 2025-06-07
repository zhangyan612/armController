from __future__ import annotations
import time
import random
from typing import List
from typing import Literal
from typing import Tuple
from hid import HID_MAPPING
from serial import Serial


class CH9329Error(Exception):
    pass


class InvalidModifier(CH9329Error):
    pass


class InvalidKey(CH9329Error):
    pass


class TooManyKeysError(CH9329Error):
    pass


class ProtocolError(CH9329Error):
    pass



Modifier = Literal[
    "ctrl",
    "ctrl_left",
    "ctrl_right",
    "shift",
    "shift_left",
    "shift_right",
    "alt",
    "alt_left",
    "alt_right",
    "win",
    "win_left",
    "win_right",
]

MODIFIER_MAP = {
    "ctrl": 0b00000001,
    "ctrl_left": 0b00000001,
    "shift": 0b00000010,
    "shift_left": 0b00000010,
    "alt": 0b00000100,
    "alt_left": 0b00000100,
    "win": 0b00001000,
    "win_left": 0b00001000,
    "ctrl_right": 0b00010000,
    "shift_right": 0b00100000,
    "alt_right": 0b01000000,
    "win_right": 0b10000000,
}

# 将字符转换为数据包
HEAD = b"\x57\xab"  # 帧头
ADDR = b"\x00"  # 地址
CMD = b"\x02"  # 命令
LEN = b"\x08"  # 数据长度



def get_packet(
    head: bytes, addr: bytes, cmd: bytes, length: bytes, data: bytes
):
    """
    累加和计算工具
    :param head:
    :param addr:
    :param cmd:
    :param length:
    :param data:
    :return: 返回累加和
    """
    head_hex_list = list(head)
    head_hex_sum = sum(head_hex_list)
    data_hex_list = list(data)
    data_hex_sum = sum(data_hex_list)
    checksum = (
        sum(
            [
                head_hex_sum,
                int.from_bytes(addr, byteorder="big"),
                int.from_bytes(cmd, byteorder="big"),
                int.from_bytes(length, byteorder="big"),
                data_hex_sum,
            ]
        )
        % 256
    )
    packet = head + addr + cmd + length + data + bytes([checksum])
    return packet


class Keyboard:

    def __init__(self, serial):
        """

        :param serial: 串口控制器
        """
        self.Serial = serial

    def send(
            self, keys: Tuple[str, str, str, str, str, str] = ("", "", "", "", "", ""),
            modifiers: List[Modifier] = [],
    ) -> None:
        """
        发送键盘按键
        :param keys: 按键
        :param modifiers: 控制键
        :return: None
        """
        # CMD_SEND_KB_GENERAL_DATA 数据正好有 8 个字节
        data = b""

        # 第一个字节修饰键，每一位代表1个键
        #
        # BIT0 - ctrl_left
        # BIT1 - shift_left
        # BIT2 - alt_left
        # BIT3 - win_left
        # BIT4 - ctrl_right
        # BIT5 - shift_right
        # BIT6 - alt_right
        # BIT7 - win_right
        modifier = 0x00
        for m in modifiers:
            if m not in MODIFIER_MAP:
                raise InvalidModifier(m)
            modifier |= MODIFIER_MAP[m]
        data += modifier.to_bytes(1, byteorder="little")

        # 第二个字节必须是 0x00
        data += b"\x00"

        # 第三到第八个字节是键
        # 我们最多可以按 6 个按钮
        for key in keys:
            if key not in HID_MAPPING:
                raise InvalidKey(key)
            hid, _ = HID_MAPPING[key]
            data += hid

        # 创建数据包并发送
        packet = get_packet(HEAD, ADDR, CMD, LEN, data)
        self.Serial.write(packet)

    def press(self, key: str, modifiers: List[Modifier] = []) -> None:
        if key not in HID_MAPPING:
            raise InvalidKey(key)
        _, shift = HID_MAPPING[key]
        if shift:
            modifiers = modifiers.copy()
            modifiers.append("shift")
        self.send((key, "", "", "", "", ""), modifiers)

    def release(self) -> None:
        """释放按键"""
        self.send(("", "", "", "", "", ""))

    def press_and_release(
            self,
            key: str,
            modifiers: List[Modifier] = [],
            min_interval: float = 0.02,
            max_interval: float = 0.06,
    ) -> None:
        """
        按下单个按键 或者组合按键
        :param key: 需要按下的按键
        :param modifiers: 控制按键
        :param min_interval: 默认即可
        :param max_interval: 默认即可
        :return:
        """
        self.press(key, modifiers)
        time.sleep(random.uniform(min_interval, max_interval))
        self.release()

    def trigger_keys(
            self, keys: list[str], modifiers: List[Modifier] = []
    ) -> None:
        press_keys = keys.copy()
        press_modifiers = modifiers.copy()
        press_keys = list(set(press_keys))
        press_modifiers = list(set(press_modifiers))
        # Supports press to 6 normal buttons at the same time
        if len(press_keys) > 6:
            raise TooManyKeysError(
                "CH9329最多支持同时按下6个按键。"
            )
        if len(modifiers) > 8:
            raise TooManyKeysError(
                "CH9329最多支持同时按下8个控制键。"
            )
        # if len(keys) <= 6, add empty keys
        while len(press_keys) != 6:
            press_keys.append("")

        self.send(
            (
                press_keys[0],
                press_keys[1],
                press_keys[2],
                press_keys[3],
                press_keys[4],
                press_keys[5],
            ),
            press_modifiers,
        )

    def write(
            self,
            text: str,
            min_interval: float = 0.02,
            max_interval: float = 0.06,
    ) -> None:
        """
        按下多个按键,一般用于输入
        :param text: 按键文本
        :param min_interval: 默认即可
        :param max_interval: 默认即可
        :return:
        """
        for char in text:
            self.press_and_release(char, [], min_interval, max_interval)
            time.sleep(random.uniform(min_interval, max_interval))

    def longpress(self,key:str):
        """
        长按按键
        :param key: 按键
        :return:
        """
        self.press(key)






MouseCtrl = Literal["null", "left", "right", "center"]

ctrl_to_hex_mapping: dict[MouseCtrl, bytes] = {
    "null": b"\x00",
    "left": b"\x01",
    "right": b"\x02",
    "center": b"\x04",
}

HEAD = b"\x57\xab"  # 帧头
ADDR = b"\x00"  # 地址
CMD_ABS = b"\x04"  # 绝对命令
CMD_REL = b"\x05"  # 相对命令
LEN_ABS = b"\x07"  # 绝对数据长度
LEN_REL = b"\x05"  # 相对数据长度


class Mouse:
    ser: Serial = None
    screenx: int = None
    screeny: int = None

    def __init__(self, ser, screenx: int, screeny: int):
        """
        鼠标控制
        :param ser: 串口控制器
        :param screenx: 屏幕X 比如1920
        :param screeny: 屏幕Y 比如1080
        """
        self.ser = ser
        self.screenx = screenx
        self.screeny = screeny

    def wheel_int_to_bytes(self, wheel_delta: int):
        """
        将wheel_delta人类可读整数转换为ch9329可读字节
        """
        # 如果为0x00，则表示没有滚动动作
        # 0x01-0x7F，表示向上滚动
        # 0x81-0xFF，表示向下滚动
        if abs(wheel_delta) > 127:
            raise RuntimeError("允许的最大车轮增量为 127。")
        if wheel_delta >= 0:
            return (0x00 + wheel_delta).to_bytes(1)
        else:
            return (0x100 + wheel_delta).to_bytes(1)

    def send_data_absolute(
            self,
            x: int,
            y: int,
            ctrl: MouseCtrl = "null",
            wheel_delta: int = 0,
    ) -> None:
        """绝对移动"""
        # CMD_SEND_MS_ABS_DATA 正好有 7 个字节

        # 第一个字节始终是 0x02
        data = b"\x02"

        # 第二个字节是鼠标按钮值
        data += ctrl_to_hex_mapping[ctrl]

        # 第三和第四个字节是x坐标
        x_cur = (4096 * x) // self.screenx
        data += x_cur.to_bytes(2, byteorder="little")

        # 第五和第六个字节是 y 坐标
        y_cur = (4096 * y) // self.screeny
        data += y_cur.to_bytes(2, byteorder="little")

        # 第七个字节包含车轮数据
        data += self.wheel_int_to_bytes(wheel_delta)

        packet = get_packet(HEAD, ADDR, CMD_ABS, LEN_ABS, data)
        self.ser.write(packet)

    def send_data_relative(self,
                           x: int, y: int, ctrl: MouseCtrl = "null", wheel_delta: int = 0
                           ) -> None:
        """相对移动"""
        # 第一个字节始终为 0x01
        data = b"\x01"

        # 第二个字节是鼠标按钮值
        data += ctrl_to_hex_mapping[ctrl]

        # 第三个字节是x距离
        if x < 0:
            data += (0 - abs(x)).to_bytes(1, byteorder="big", signed=True)
        else:
            data += x.to_bytes(1, byteorder="big", signed=True)

        # 第四个字节是 y 距离
        if y < 0:
            data += (0 - abs(y)).to_bytes(1, byteorder="big", signed=True)
        else:
            data += y.to_bytes(1, byteorder="big", signed=True)

        # 第五个字节包含车轮数据
        data += self.wheel_int_to_bytes(wheel_delta)

        packet = get_packet(HEAD, ADDR, CMD_REL, LEN_REL, data)
        self.ser.write(packet)

    # def move(
    #         self,
    #         x: int,
    #         y: int,
    #         relative: bool = False,
    # ) -> None:
    #     if relative:
    #         self.send_data_relative(x, y, "null")
    #     else:
    #         self.send_data_absolute(x, y, "null", )

    def press(self, button: MouseCtrl = "left") -> None:
        """
        长按鼠标,默认左键
        :param button: "left", "right", "center"
        :return:
        """
        self.send_data_relative(0, 0, button)

    def release(self) -> None:
        """释放鼠标"""
        self.send_data_relative(0, 0, "null")

    def click(self, button: MouseCtrl = "left") -> None:
        """
        单击鼠标,默认左键
        :param button: "left", "right", "center"
        :return:
        """
        self.press(button)
        # 模拟自然行为的 20ms 到 60 毫秒延迟
        time.sleep(random.uniform(0.02, 0.06))
        self.release()

    def wheel(self, wheel: int) -> None:
        """
        滚动鼠标轮子
        :param wheel: 滚动距离
        :return:
        """
        self.send_data_relative(0, 0, wheel_delta=wheel)

    def relative_move(self, x, y):
        """
        相对移动
        :param x: 移动X
        :param y: 移动Y
        :return:
        """
        self.send_data_relative(x, y, "null")

    def absolute_move(self, x, y):
        """
        绝对移动
        :param x: 移动X
        :param y: 移动Y
        :return:
        """
        self.send_data_absolute(x, y, "null")
