"""
examples/read_params.py — 读取所有电机参数示例

用法:
    python examples/read_params.py

确保:
    - 驱动器已通电
    - OLED 设置: P_Serial=UART_FUN, Response=Receive, Checksum=0x6B
    - USB2TTL 连接正确
"""
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from python.zdt_stepper import ZDTStepper


def main():
    port = 'COM9'

    with ZDTStepper(port) as motor:
        print(f"{'='*50}")
        print(f"ZDT Stepper — Parameters")
        print(f"{'='*50}")

        print(f"\n{'Version':12s}: {motor.read_version()}")
        print(f"{'Voltage':12s}: {motor.read_bus_voltage():.2f} V")

        res, ind = motor.read_motor_params()
        print(f"{'Resistance':12s}: {res:.1f} mOhm")
        print(f"{'Inductance':12s}: {ind:.1f} uH")

        print(f"{'Current(set)':12s}: {motor.read_phase_current()} mA")
        print(f"{'Current(act)':12s}: {motor.read_actual_current()} mA")
        print(f"{'Temperature':12s}: {motor.read_temperature()} C")
        print(f"{'Encoder':12s}: {motor.read_encoder()}")
        print(f"{'Hall Angle':12s}: {motor.read_hall_angle():.1f} deg")
        print(f"{'Position':12s}: {motor.read_position():.1f} deg")
        print(f"{'Speed':12s}: {motor.read_speed()} RPM")
        print(f"{'Position Err':12s}: {motor.read_position_error():.1f} deg")

        status = motor.read_status()
        print(f"\n{'Status':12s}: {'ENABLED' if status['enabled'] else 'disabled'}"
              f", {'IN_POS' if status['in_position'] else 'moving'}"
              f", {'STALL' if status['stall'] else 'ok'}")


if __name__ == '__main__':
    main()
