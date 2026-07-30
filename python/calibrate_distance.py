"""
距离标定 — 已知移动距离，计算 K 系数
======================================
流程: 读初始值 → 释放力矩 → 手动推已知距离 → 保持力矩 → 读最终值 → 算K
"""

import serial
import time
import sys

PORT = 'COM8'
BAUDRATE = 115200
TIMEOUT = 0.5

MOTOR_NAMES = {1: 'X轴', 2: 'Y轴', 3: 'Z轴', 4: 'R轴'}

# ============================================================
#  协议辅助
# ============================================================

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
    """读取位置，返回度数 (float)"""
    ser.write(make_cmd(addr, 0x36))
    time.sleep(0.1)
    data = recv_frame(ser)
    if len(data) < 8:
        print(f'    [调试] 0x36 addr={addr} raw={data.hex()!r}', file=sys.stderr)
        return None
    sign = -1 if data[2] != 0 else 1
    val = (data[3] << 24) | (data[4] << 16) | (data[5] << 8) | data[6]
    return val / 10.0 * sign

def read_encoder(ser, addr):
    """读取编码器原始值 (0-65535)"""
    ser.write(make_cmd(addr, 0x31))
    time.sleep(0.1)
    data = recv_frame(ser)
    if len(data) < 5:
        print(f'    [调试] 0x31 addr={addr} raw={data.hex()!r}', file=sys.stderr)
        return None
    return (data[2] << 8) | data[3]

def enable_motor(ser, addr, on):
    """使能/失使能电机"""
    payload = bytes([0xAB, 1 if on else 0, 0])
    ser.write(make_cmd(addr, 0xF3, payload))
    time.sleep(0.05)
    data = recv_frame(ser)
    return len(data) >= 4 and data[2] == 0x02


# ============================================================
#  主程序
# ============================================================

def main():
    print('=' * 55)
    print('  距离标定 — 计算 K 系数 (°/mm)')
    print('=' * 55)

    # 从命令行参数读取: python calibrate_distance.py <地址> <距离mm>
    if len(sys.argv) >= 3:
        addr = int(sys.argv[1])
        known_mm = float(sys.argv[2])
    elif len(sys.argv) == 2:
        addr = int(sys.argv[1])
        known_mm = float(input('  输入移动距离 (mm): '))
    else:
        addr = int(input('  输入电机地址 (1-4): '))
        known_mm = float(input('  输入移动距离 (mm): '))
    name = MOTOR_NAMES.get(addr, f'未知({addr})')

    print(f'\n  目标: {name} (地址 {addr})')
    print(f'  已知移动距离: {known_mm:.0f} mm')

    # 打开串口
    try:
        ser = serial.Serial(PORT, BAUDRATE, timeout=TIMEOUT,
                            bytesize=8, parity='N', stopbits=1)
        print(f'  串口: {PORT} @ {BAUDRATE}\n')
    except Exception as e:
        print(f'[✗] 打开串口失败: {e}')
        return

    time.sleep(0.5)
    ser.reset_input_buffer()
    time.sleep(0.2)

    # ── 预读一次清空总线 ──
    ser.write(make_cmd(addr, 0x3A))  # 读状态
    time.sleep(0.1)
    _ = recv_frame(ser)  # 丢弃
    time.sleep(0.1)
    ser.reset_input_buffer()

    # ── 1. 读初始值 ──
    print('─── 第1步: 读取初始值 ───')
    pos_before = read_position(ser, addr)
    if pos_before is None:
        print('  [✗] 读取失败')
        ser.close()
        return
    print(f'  [✓] 位置 = {pos_before:.2f}°')

    # ── 2. 释放力矩 ──
    print('\n─── 第2步: 释放力矩 ───')
    input(f'  按 Enter 释放力矩，然后手动推 {name} 正方向 {known_mm:.0f}mm... ⏎')
    if not enable_motor(ser, addr, False):
        print('  [✗] 释放失败')
        ser.close()
        return
    print('  [✓] 力矩已释放')

    # ── 3. 等待用户推动 ──
    input('\n─── 第3步: 推完 180mm 后，按 Enter 恢复力矩... ⏎')

    # ── 4. 保持力矩 ──
    if not enable_motor(ser, addr, True):
        print('  [✗] 力矩恢复失败')
        ser.close()
        return
    print('  [✓] 力矩已恢复')
    time.sleep(0.2)

    # ── 5. 读最终值 ──
    print('\n─── 第4步: 读取最终值 ───')
    pos_after = read_position(ser, addr)
    if pos_after is None:
        print('  [✗] 读取失败')
        ser.close()
        return
    print(f'  [✓] 位置 = {pos_after:.2f}°')

    # ── 6. 计算 K ──
    print('\n' + '=' * 55)
    print('  标定结果')
    print('=' * 55)

    delta_pos = pos_after - pos_before

    # 用位置变化的绝对值算 K (K 恒为正数)
    K_abs = abs(delta_pos) / known_mm

    print(f'\n  位置变化:  {delta_pos:+.2f}°')
    print(f'  移动距离:  {known_mm:.0f} mm')
    print(f'\n  📐 K = |Δθ| / d = {abs(delta_pos):.2f}° / {known_mm:.0f}mm')
    print(f'  📐 K = {K_abs:.6f} °/mm')
    print(f'  📐 K = {K_abs * 1000:.4f} °/m')

    # 换算为每转对应的移动距离
    deg_per_rev = 360.0
    mm_per_rev = deg_per_rev / K_abs if K_abs > 0 else float('inf')
    print(f'  ⚙  电机每转 = {mm_per_rev:.2f} mm')
    if K_abs > 0:
        print(f'  ⚙  每mm对应 = {1.0/K_abs:.4f} mm/°')

    ser.close()
    print('\n串口已关闭')


if __name__ == '__main__':
    main()
