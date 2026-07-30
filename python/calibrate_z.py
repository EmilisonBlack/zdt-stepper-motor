"""
Z 轴两点标定 — 记录放下/抬升两个状态的位置
===========================================
流程: 读放下位置 → 释放力矩 → 手动推至抬升 → 保持力矩 → 读抬升位置
"""

import serial
import time
import sys

PORT = 'COM8'
BAUDRATE = 115200
TIMEOUT = 0.5

ADDR = 3  # Z 轴
NAME = 'Z轴'


def make_cmd(addr, func, payload=b''):
    return bytes([addr, func]) + payload + b'\x6B'

def recv_frame(ser):
    buf = bytearray()
    t0 = time.time()
    while time.time() - t0 < TIMEOUT:
        if ser.in_waiting > 0:
            b = ser.read(1)
            buf.extend(b)
            if b == b'\x6B' and len(buf) >= 3:
                return bytes(buf)
        else:
            time.sleep(0.01)
    return bytes(buf)

def read_position(ser, addr):
    ser.write(make_cmd(addr, 0x36))
    time.sleep(0.1)
    data = recv_frame(ser)
    if len(data) < 8:
        return None
    sign = -1 if data[2] != 0 else 1
    val = (data[3] << 24) | (data[4] << 16) | (data[5] << 8) | data[6]
    return val / 10.0 * sign

def enable_motor(ser, addr, on):
    payload = bytes([0xAB, 1 if on else 0, 0])
    ser.write(make_cmd(addr, 0xF3, payload))
    time.sleep(0.05)
    data = recv_frame(ser)
    return len(data) >= 4 and data[2] == 0x02


def main():
    print('=' * 50)
    print('  Z 轴两点标定')
    print('  记录放下 / 抬升 两个位置')
    print('=' * 50)

    try:
        ser = serial.Serial(PORT, BAUDRATE, timeout=TIMEOUT,
                            bytesize=8, parity='N', stopbits=1)
        print(f'\n[✓] 串口 {PORT} 已打开')
    except Exception as e:
        print(f'[✗] 打开串口失败: {e}')
        return

    time.sleep(0.5)
    ser.reset_input_buffer()
    time.sleep(0.2)

    # ── 第1步: 读取放下位置 ──
    input('\n  确保 Z 轴处于【放下】状态，然后按 Enter 读取位置... ⏎')
    pos_down = read_position(ser, ADDR)
    if pos_down is None:
        print('  [✗] 读取失败')
        ser.close()
        return
    print(f'  [✓] Z 放下位置 = {pos_down:.2f}°')

    # ── 第2步: 释放力矩 ──
    print('\n─── 释放力矩 ───')
    if not enable_motor(ser, ADDR, False):
        print('  [✗] 释放失败')
        ser.close()
        return
    print('  [✓] 力矩已释放')

    # ── 第3步: 用户推到抬升 ──
    input('\n  手动将 Z 轴推到【抬升】状态，然后按 Enter... ⏎')

    # ── 第4步: 保持力矩 ──
    print('\n─── 恢复力矩 ───')
    if not enable_motor(ser, ADDR, True):
        print('  [✗] 力矩恢复失败')
        ser.close()
        return
    print('  [✓] 力矩已恢复')
    time.sleep(0.2)

    # ── 第5步: 读取抬升位置 ──
    pos_up = read_position(ser, ADDR)
    if pos_up is None:
        print('  [✗] 读取失败')
        ser.close()
        return
    print(f'  [✓] Z 抬升位置 = {pos_up:.2f}°')

    # ── 输出结果 ──
    print('\n' + '=' * 50)
    print('  标定结果')
    print('=' * 50)
    print(f'\n  Z_DOWN = {pos_down:.2f}°   (放下)')
    print(f'  Z_UP   = {pos_up:.2f}°   (抬升)')
    print(f'  行程   = {abs(pos_up - pos_down):.2f}°')
    print()
    print('  控制时使用:')
    print(f'    zdt_PositionModeAbs(addr=3, ..., pos={pos_down * 10:.0f})  → 放下')
    print(f'    zdt_PositionModeAbs(addr=3, ..., pos={pos_up * 10:.0f})    → 抬升')
    print()
    print('  💡 放下后记得释放力矩 (zdt_Enable(false))')

    ser.close()
    print('\n串口已关闭')


if __name__ == '__main__':
    main()
