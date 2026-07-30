"""
R 轴角度验证 — 手动旋转，验证读数是否匹配
=========================================
流程: 读初始角度 → 释放力矩 → 手动逆时针转90° → 保持力矩 → 读最终角度
"""

import serial
import time

PORT = 'COM8'
BAUDRATE = 115200
TIMEOUT = 0.5
ADDR = 4
NAME = 'R轴'


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
    print('  R 轴角度验证')
    print('  手动逆时针旋转 90°，验证读数')
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

    # ── 第1步: 读初始角度 ──
    input('\n  确保 R 轴在 0° 初始位置，按 Enter 读取... ⏎')
    pos_before = read_position(ser, ADDR)
    if pos_before is None:
        print('  [✗] 读取失败')
        ser.close()
        return
    print(f'  [✓] 初始角度 = {pos_before:.1f}°')

    # ── 第2步: 释放力矩 ──
    print('\n─── 释放力矩 ───')
    if not enable_motor(ser, ADDR, False):
        print('  [✗] 释放失败')
        ser.close()
        return
    print('  [✓] 力矩已释放，可以手动旋转')

    # ── 第3步: 用户旋转 ──
    input('\n  手动【逆时针】旋转 R 轴约 90°，然后按 Enter... ⏎')

    # ── 第4步: 保持力矩 ──
    print('\n─── 恢复力矩 ───')
    if not enable_motor(ser, ADDR, True):
        print('  [✗] 力矩恢复失败')
        ser.close()
        return
    print('  [✓] 力矩已恢复')
    time.sleep(0.2)

    # ── 第5步: 读最终角度 ──
    pos_after = read_position(ser, ADDR)
    if pos_after is None:
        print('  [✗] 读取失败')
        ser.close()
        return
    print(f'  [✓] 最终角度 = {pos_after:.1f}°')

    # ── 分析 ──
    delta = pos_after - pos_before

    print('\n' + '=' * 50)
    print('  验证结果')
    print('=' * 50)
    print(f'  初始角度:  {pos_before:.1f}°')
    print(f'  最终角度:  {pos_after:.1f}°')
    print(f'  变化:      {delta:+.1f}°')
    print(f'  预期:      +90.0° (逆时针)')
    print()

    # R 轴 DIR_SIGN = -1，所以逆时针旋转时读数会减少（负变化）
    if abs(abs(delta) - 90) < 15:
        print(f'  ✅ 验证通过！读数变化 {abs(delta):.0f}°，与 90° 基本一致')
        print(f'  💡 注意: R 轴 DIR_SIGN = -1，所以逆时针旋转时角度读数为负')
        print(f'     控制时发送正角度 → 驱动器反向运动，最终实际正转')
    else:
        print(f'  ⚠️  偏差较大 ({abs(delta):.0f}° vs 90°)，建议检查')
        print(f'     可能原因: 旋转方向判断有误，或读数异常')

    ser.close()
    print('\n串口已关闭')


if __name__ == '__main__':
    main()
