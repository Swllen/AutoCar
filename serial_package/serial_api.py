import math
import struct

# === 通用工具函数 ===
def int16_to_bytes(val):
    """将有符号整数转为两个字节"""
    return val.to_bytes(2, byteorder='big', signed=True)

def calc_checksum(data: list):
    """计算校验和：所有字节相加后取低8位"""
    return sum(data) % 256

# === 通用速度帧构建函数 ===
def build_velocity_frame(x_speed=0, y_speed=0, w_speed=0):
    frame = [0xAA, 0x55, 0x0B, 0x50]
    frame += list(int16_to_bytes(x_speed))
    frame += list(int16_to_bytes(y_speed))
    frame += list(int16_to_bytes(w_speed))
    frame.append(calc_checksum(frame))
    return bytearray(frame)

# === 麦轮控制指令构建 ===
def forward(speed_mps):
    return build_velocity_frame(x_speed=int(speed_mps * 1000))

def backward(speed_mps):
    return build_velocity_frame(x_speed=int(-speed_mps * 1000))
def motion(angle, speed_mps):
    """
    根据角度和速度构建运动指令
    angle: 角度，单位为弧度
    speed_mps: 速度，单位为米每秒
    """
    x_speed = int(speed_mps * 1000 * math.cos(angle))
    y_speed = int(speed_mps * 1000 * math.sin(angle))
    return build_velocity_frame(x_speed=x_speed, y_speed=y_speed)

def rotate_cw(w_rads):
    return build_velocity_frame(w_speed=int(-w_rads * 1000))

def rotate_ccw(w_rads):
    return build_velocity_frame(w_speed=int(w_rads * 1000))

def turn_left(speed_mps, turn_ratio):
    return build_velocity_frame(x_speed=int(speed_mps * 1000),
                                y_speed=0,
                                w_speed=int(turn_ratio * 1000))

def turn_right(speed_mps, turn_ratio):
    return build_velocity_frame(x_speed=int(speed_mps * 1000),
                                y_speed=0,
                                w_speed=int(-turn_ratio * 1000))

def stop():
    return build_velocity_frame(0, 0, 0)

# === 灯光控制构建 ===
def build_light_frame(r, g, b, duration):
    """
    返回构造好的灯光控制数据包
    """
    frame = [0xAA, 0x55, 0x0B, 0x52, 0x01, 0x01, duration, r, g, b]
    frame.append(calc_checksum(frame))
    return bytearray(frame)

# === 蜂鸣器控制构建 ===
def build_beep_on_frame():
    frame = [0xAA, 0x55, 0x06, 0x54, 0x01]
    frame.append(calc_checksum(frame))
    return bytearray(frame)

def build_beep_off_frame():
    frame = [0xAA, 0x55, 0x06, 0x54, 0x00]
    frame.append(calc_checksum(frame))
    return bytearray(frame)


def build_yaw_pitch_package(yaw_delta, pitch_delta,flag):
    header = b'\xAA\x55'
    cmd = b'\x01'
    flag_byte = struct.pack('B', flag)  # 标记位：0或1
    payload = flag_byte + struct.pack('<ff', yaw_delta, pitch_delta)  # 标记 + 两个float
    length = len(payload).to_bytes(1, 'little')

    checksum = 0
    for b in cmd + length + payload:
        checksum ^= b

    packet = header + cmd + length + payload + bytes([checksum])

    return packet

