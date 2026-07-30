"""
Actuator — 4 轴拼图机器人执行器统一控制类
===========================================
封装 X/Y/Z/R 四轴控制，自动处理：
  - 方向修正 (DIR_SIGN)
  - 角度↔mm 换算 (K 系数)
  - XY 直线插补 (速度分配，同时到达)
  - Z 轴两点定位 + 力矩自动管理
  - R 轴旋转 + 回零
  - 安全巡检 (过流/过热/堵转)
  - 日志记录 + 异常抛出

使用示例:
    act = Actuator('COM8')
    act.move_to(150, 130, rpm=30)
    act.z_up()
    act.pickup()
    act.move_to(50, 80)
    act.z_down()
    act.release()
    act.close()
"""

import serial
import time
import logging
import sys
from typing import Optional, Tuple

# ============================================================
#  日志配置
# ============================================================
logger = logging.getLogger('Actuator')
if not logger.handlers:
    logger.setLevel(logging.DEBUG)
    _sh = logging.StreamHandler(sys.stdout)
    _sh.setFormatter(logging.Formatter(
        '[%(asctime)s] %(levelname)s  %(message)s', datefmt='%H:%M:%S'))
    logger.addHandler(_sh)
    # 文件日志
    _fh = logging.FileHandler('actuator.log', encoding='utf-8')
    _fh.setFormatter(logging.Formatter(
        '%(asctime)s [%(levelname)s] %(message)s'))
    logger.addHandler(_fh)


# ============================================================
#  异常定义
# ============================================================
class ActuatorError(Exception):
    """执行器通用异常"""
    pass

class SafetyError(ActuatorError):
    """安全保护触发"""
    pass

class LimitError(ActuatorError):
    """超出软限位"""
    pass

class CommsError(ActuatorError):
    """通信失败"""
    pass


# ============================================================
#  Actuator 类
# ============================================================
class Actuator:
    # ---- 标定参数 ----
    K = {1: 8.9980, 2: 8.9394}           # °/mm (X, Y)
    DIR_SIGN = {1: +1, 2: -1, 3: +1, 4: -1}
    MOTOR_NAMES = {1: 'X', 2: 'Y', 3: 'Z', 4: 'R'}

    # ---- Z 轴两点 ----
    Z_DOWN = 0.0     # °
    Z_UP   = 73.9    # °

    # ---- R 轴限位 ----
    R_MIN = -240     # °
    R_MAX = +240     # °

    # ---- 速度限制 ----
    MAX_RPM = 40

    # ---- 安全阈值 ----
    CURRENT_MAX_MA = 1000
    TEMP_MAX_C = 50

    # ---- 协议常量 ----
    DIR_CW  = 0
    DIR_CCW = 1

    # ================================================================
    #  初始化
    # ================================================================
    def __init__(self, port: str = 'COM8', baudrate: int = 115200,
                 timeout: float = 0.5):
        self._port = port
        self._baudrate = baudrate
        self._timeout = timeout

        # 内部状态
        self._x_mm = 0.0   # 当前 X 坐标 (mm)
        self._y_mm = 0.0   # 当前 Y 坐标 (mm)
        self._z_up = False # Z 是否处于抬升状态
        self._enabled = {1: True, 2: True, 3: False, 4: True}

        # 打开串口
        try:
            self._ser = serial.Serial(
                port, baudrate, timeout=timeout,
                bytesize=8, parity='N', stopbits=1)
            logger.info(f'串口 {port} 已打开 @ {baudrate}')
        except Exception as e:
            raise ActuatorError(f'打开串口 {port} 失败: {e}')

        time.sleep(0.5)
        self._flush()

        # 读取初始位置
        self._sync_position()
        logger.info(f'Actuator 初始化完成, 坐标: ({self._x_mm:.1f}, {self._y_mm:.1f})')

    def close(self):
        """关闭串口"""
        if hasattr(self, '_ser') and self._ser and self._ser.is_open:
            self._ser.close()
            logger.info('串口已关闭')

    def __del__(self):
        self.close()

    # ================================================================
    #  内部: 串口通信
    # ================================================================
    def _make_cmd(self, addr: int, func: int, payload: bytes = b'') -> bytes:
        return bytes([addr, func]) + payload + b'\x6B'

    def _recv_frame(self) -> bytes:
        buf = bytearray()
        t0 = time.time()
        while time.time() - t0 < self._timeout:
            if self._ser.in_waiting > 0:
                b = self._ser.read(1)
                buf.extend(b)
                if b == b'\x6B' and len(buf) >= 3:
                    return bytes(buf)
            else:
                time.sleep(0.005)
        return bytes(buf)

    def _flush(self):
        """清空串口缓冲区 + 预读同步"""
        self._ser.reset_input_buffer()
        time.sleep(0.2)
        # 轮询4个地址各一次，确保总线干净
        for addr in [1, 2, 3, 4]:
            for _ in range(2):
                self._ser.write(self._make_cmd(addr, 0x3A))
                time.sleep(0.1)
                self._recv_frame()
                time.sleep(0.05)
        self._ser.reset_input_buffer()
        time.sleep(0.2)

    def _send_cmd(self, addr: int, func: int, payload: bytes,
                  expect_ok: bool = True) -> bytes:
        """发送命令并等待响应 (自动处理 RS485 回显)"""
        self._ser.write(self._make_cmd(addr, func, payload))
        time.sleep(0.08)
        data = self._recv_frame()
        # 如果是回显(3字节), 丢弃再收
        if len(data) == 3 and data[1] == func:
            time.sleep(0.08)
            data = self._recv_frame()
        if len(data) < 3:
            raise CommsError(f'{self.MOTOR_NAMES[addr]} 无响应 (cmd=0x{func:02X})')
        if data[0] != addr:
            raise CommsError(f'{self.MOTOR_NAMES[addr]} 地址错: 期望{addr:02X} 收到{data[0]:02X}')
        if expect_ok and len(data) >= 4 and data[2] != 0x02:
            raise CommsError(f'{self.MOTOR_NAMES[addr]} 命令拒绝 (cmd=0x{func:02X}, resp={data.hex()})')
        return data

    # ================================================================
    #  内部: 读取数据
    # ================================================================
    def _read_position_deg(self, addr: int, retry: int = 2) -> float:
        """读取电机位置 (°)，带重试"""
        for attempt in range(retry + 1):
            # 写后等足够长让驱动器回复
            self._ser.write(self._make_cmd(addr, 0x36))
            time.sleep(0.15)
            # 收帧: 可能先收到命令回显再收到数据
            data = self._recv_frame()
            # 如果收到的是自己的命令(3字节), 再收一次
            if len(data) == 3 and data[0] == addr and data[1] == 0x36:
                time.sleep(0.15)
                data = self._recv_frame()
            if len(data) >= 8 and data[0] == addr:
                sign = -1 if data[2] != 0 else 1
                val = (data[3] << 24) | (data[4] << 16) | (data[5] << 8) | data[6]
                return val / 10.0 * sign
            if attempt < retry:
                logger.debug(f'{self.MOTOR_NAMES[addr]} 读位置重试 ({data.hex()})')
                time.sleep(0.3)
        raise CommsError(f'{self.MOTOR_NAMES[addr]} 读位置失败 ({data.hex()})')

    def _read_current_ma(self, addr: int, retry: int = 1) -> int:
        """读取实际相电流 (mA)"""
        for _ in range(retry + 1):
            self._ser.write(self._make_cmd(addr, 0x26))
            time.sleep(0.12)
            data = self._recv_frame()
            if len(data) == 3 and data[1] == 0x26:
                time.sleep(0.1)
                data = self._recv_frame()
            if len(data) >= 5 and data[0] == addr and data[1] == 0x26:
                return (data[2] << 8) | data[3]
        return -1

    def _read_temperature(self, addr: int, retry: int = 1) -> float:
        """读取温度 (°C)"""
        for _ in range(retry + 1):
            self._ser.write(self._make_cmd(addr, 0x39))
            time.sleep(0.12)
            data = self._recv_frame()
            if len(data) == 3 and data[1] == 0x39:
                time.sleep(0.1)
                data = self._recv_frame()
            if len(data) >= 5 and data[0] == addr and data[1] == 0x39:
                return float((data[2] << 8) | data[3])
        return -1

    def _read_status_flags(self, addr: int, retry: int = 1) -> int:
        """读取状态标志"""
        for _ in range(retry + 1):
            self._ser.write(self._make_cmd(addr, 0x3A))
            time.sleep(0.1)
            data = self._recv_frame()
            if len(data) == 3 and data[1] == 0x3A:
                time.sleep(0.1)
                data = self._recv_frame()
            if len(data) >= 4 and data[0] == addr and data[1] == 0x3A:
                return data[2]
        return -1

    def _read_speed(self, addr: int, retry: int = 1) -> float:
        """读取转速 (RPM)"""
        for _ in range(retry + 1):
            self._ser.write(self._make_cmd(addr, 0x35))
            time.sleep(0.12)
            data = self._recv_frame()
            if len(data) == 3 and data[1] == 0x35:
                time.sleep(0.1)
                data = self._recv_frame()
            if len(data) >= 6 and data[0] == addr and data[1] == 0x35:
                spd = ((data[3] << 8) | data[4]) / 10.0
                return -spd if data[2] != 0 else spd
        return 0.0

    # ================================================================
    #  内部: 电机控制
    # ================================================================
    def _enable(self, addr: int, on: bool):
        """使能/失能电机"""
        payload = bytes([0xAB, 1 if on else 0, 0])
        self._send_cmd(addr, 0xF3, payload)
        self._enabled[addr] = on
        time.sleep(0.05)

    def _send_position_abs(self, addr: int, rpm: int, accel: int,
                           target_deg: float):
        """发送绝对位置命令"""
        dir_byte = self.DIR_CW if target_deg >= 0 else self.DIR_CCW
        speed10 = rpm * 10
        pos_x10 = int(abs(target_deg) * 10)
        payload = bytes([
            dir_byte,
            (accel >> 8) & 0xFF, accel & 0xFF,
            (accel >> 8) & 0xFF, accel & 0xFF,
            (speed10 >> 8) & 0xFF, speed10 & 0xFF,
            (pos_x10 >> 24) & 0xFF, (pos_x10 >> 16) & 0xFF,
            (pos_x10 >> 8) & 0xFF, pos_x10 & 0xFF,
            0x01,  # abs
            0x00,  # sync
        ])
        self._send_cmd(addr, 0xFD, payload)

    def _send_position_rel(self, addr: int, rpm: int, accel: int,
                           degrees: float, dir_ccw: bool = False):
        """发送相对位置命令"""
        dir_byte = self.DIR_CCW if dir_ccw else self.DIR_CW
        speed10 = rpm * 10
        pos_x10 = int(abs(degrees) * 10)
        payload = bytes([
            dir_byte,
            (accel >> 8) & 0xFF, accel & 0xFF,
            (accel >> 8) & 0xFF, accel & 0xFF,
            (speed10 >> 8) & 0xFF, speed10 & 0xFF,
            (pos_x10 >> 24) & 0xFF, (pos_x10 >> 16) & 0xFF,
            (pos_x10 >> 8) & 0xFF, pos_x10 & 0xFF,
            0x00,  # rel
            0x00,  # sync
        ])
        self._send_cmd(addr, 0xFD, payload)

    def _wait_until_idle(self, addr: int, timeout_s: int = 60) -> bool:
        """等待电机到位"""
        t0 = time.time()
        while time.time() - t0 < timeout_s:
            flags = self._read_status_flags(addr)
            if flags >= 0 and (flags & 0x02):
                return True
            time.sleep(0.2)
        logger.warning(f'{self.MOTOR_NAMES[addr]} 等待到位超时 ({timeout_s}s)')
        return False

    def _sync_position(self):
        """从驱动器读取并同步内部坐标"""
        for addr in [1, 2]:
            deg = self._read_position_deg(addr)
            sign = self.DIR_SIGN[addr]
            mm = deg / self.K[addr] * sign
            if addr == 1:
                self._x_mm = mm
            else:
                self._y_mm = mm

    # ================================================================
    #  安全巡检
    # ================================================================
    def safety_check(self, addr: Optional[int] = None) -> bool:
        """安全巡检，返回是否触发保护

        Args:
            addr: 指定电机地址，None=全部

        Returns:
            True=安全正常 | False=触发保护
        """
        targets = [addr] if addr else [1, 2, 3, 4]
        ok = True
        for a in targets:
            name = self.MOTOR_NAMES[a]

            # 过流检测
            try:
                ma = self._read_current_ma(a)
                if ma > self.CURRENT_MAX_MA:
                    logger.error(f'⚠ 过流 {name}: {ma}mA > {self.CURRENT_MAX_MA}mA')
                    self._enable(a, False)
                    raise SafetyError(f'{name} 过流: {ma}mA')
            except SafetyError:
                ok = False
                continue
            except Exception:
                pass

            # 温度检测
            try:
                temp = self._read_temperature(a)
                if temp > self.TEMP_MAX_C:
                    logger.error(f'⚠ 过热 {name}: {temp:.1f}°C > {self.TEMP_MAX_C}°C')
                    self._enable(a, False)
                    raise SafetyError(f'{name} 过热: {temp:.1f}°C')
            except SafetyError:
                ok = False
                continue
            except Exception:
                pass

            # 堵转检测
            try:
                flags = self._read_status_flags(a)
                if flags >= 0 and (flags & 0x04):
                    logger.error(f'⚠ 堵转 {name}!')
                    self._enable(a, False)
                    raise SafetyError(f'{name} 堵转')
            except SafetyError:
                ok = False
                continue
            except Exception:
                pass

        return ok

    # ================================================================
    #  XY 直线插补移动
    # ================================================================
    def move_to(self, x_mm: float, y_mm: float,
                rpm: int = 30, accel: int = 200):
        """XY 直线插补移动到目标坐标

        按距离比例分配两轴速度，确保同时到达。
        较长距离轴运行在目标速度，较短轴按比例降速。

        Args:
            x_mm: 目标 X 坐标 (mm)
            y_mm: 目标 Y 坐标 (mm)
            rpm:  最大转速 (RPM), 上限 40
            accel: 加速度
        """
        # 软限位 (X/Y 暂用宽范围, 后续实测后收紧)
        if x_mm < -9999 or x_mm > 9999:
            raise LimitError(f'X 目标 {x_mm}mm 超出软限位')
        if y_mm < -9999 or y_mm > 9999:
            raise LimitError(f'Y 目标 {y_mm}mm 超出软限位')

        # 安全巡检
        self.safety_check()

        dx = x_mm - self._x_mm
        dy = y_mm - self._y_mm

        logger.info(f'移动至 ({x_mm:.1f}, {y_mm:.1f})  '
                    f'| 增量 (ΔX={dx:+.1f}, ΔY={dy:+.1f})')

        if abs(dx) < 0.01 and abs(dy) < 0.01:
            logger.info('已在目标位置')
            return

        # 计算速度分配（直线插补）
        abs_dx, abs_dy = abs(dx), abs(dy)
        d_max = max(abs_dx, abs_dy)
        d_min = min(abs_dx, abs_dy)

        speed_max = min(rpm, self.MAX_RPM)
        if d_min > 0.1:
            speed_min = max(1, int(speed_max * d_min / d_max))
        else:
            speed_min = 0  # 只有单轴移动

        # 分配速度
        if abs_dx >= abs_dy:
            spd_x, spd_y = speed_max, speed_min
        else:
            spd_x, spd_y = speed_min, speed_max

        # 转换为电机角度
        deg_x = dx * self.K[1] * self.DIR_SIGN[1]   # +1
        deg_y = dy * self.K[2] * self.DIR_SIGN[2]   # -1

        # 确定方向
        dir_x_ccw = deg_x < 0
        dir_y_ccw = deg_y < 0

        logger.debug(f'  X: {spd_x}RPM {"CCW" if dir_x_ccw else "CW"} {abs(deg_x):.1f}°')
        logger.debug(f'  Y: {spd_y}RPM {"CCW" if dir_y_ccw else "CW"} {abs(deg_y):.1f}°')

        # 发送命令 (都用相对定位, sync=0 各自立即执行)
        if abs_dx > 0.1:
            self._send_position_rel(1, spd_x, accel, abs(deg_x), dir_x_ccw)
            time.sleep(0.01)

        if abs_dy > 0.1:
            self._send_position_rel(2, spd_y, accel, abs(deg_y), dir_y_ccw)
            time.sleep(0.01)

        # 等待两轴到位
        if abs_dx > 0.1:
            self._wait_until_idle(1)
        if abs_dy > 0.1:
            self._wait_until_idle(2)

        # 安全巡检
        self.safety_check()

        # 更新内部坐标
        self._x_mm = x_mm
        self._y_mm = y_mm
        logger.info(f'到位 ✓  坐标 ({self._x_mm:.1f}, {self._y_mm:.1f})')

    # ================================================================
    #  Z 轴控制
    # ================================================================
    def z_up(self, rpm: int = 10, accel: int = 100):
        """Z 轴抬升 + 保持力矩"""
        self.safety_check(3)
        self._send_position_abs(3, rpm, accel, self.Z_UP)
        self._wait_until_idle(3)
        if not self._enabled[3]:
            self._enable(3, True)
        self._z_up = True
        logger.info(f'Z 抬升至 {self.Z_UP:.1f}° (力矩保持)')

    def z_down(self, rpm: int = 10, accel: int = 100):
        """Z 轴放下 + 释放力矩"""
        self.safety_check(3)
        self._send_position_abs(3, rpm, accel, self.Z_DOWN)
        self._wait_until_idle(3)
        # 放下后释放力矩
        self._enable(3, False)
        self._z_up = False
        logger.info(f'Z 放至 {self.Z_DOWN:.1f}° (力矩释放)')

    def z_is_up(self) -> bool:
        return self._z_up

    # ================================================================
    #  R 轴控制
    # ================================================================
    def rotate(self, deg: float, rpm: int = 25, accel: int = 200):
        """R 轴旋转到指定角度 (绝对位置)

        Args:
            deg: 目标角度, 逆时针为正, 范围 [-240, +240]
        """
        if deg < self.R_MIN or deg > self.R_MAX:
            raise LimitError(f'R 目标 {deg}° 超出限位 [{self.R_MIN}, {self.R_MAX}]')

        self.safety_check(4)
        # R 轴 DIR_SIGN=-1, 用绝对位置
        # 用户角度 → 驱动器角度: driver_deg = deg * DIR_SIGN
        driver_deg = deg * self.DIR_SIGN[4]  # * (-1)
        self._send_position_abs(4, rpm, accel, driver_deg)
        self._wait_until_idle(4)
        logger.info(f'R 旋转至 {deg:.1f}°')

    def rotate_home(self, rpm: int = 25, accel: int = 200):
        """R 轴回到 0°"""
        self.rotate(0.0, rpm, accel)

    # ================================================================
    #  电磁铁控制 (预留)
    # ================================================================
    def pickup(self):
        """电磁铁通电吸取 (GPIO 控制, 待实现)"""
        logger.info('电磁铁: 通电吸取 (GPIO 未接入)')
        # TODO: GPIO 置高 (树莓派5)
        # import RPi.GPIO as GPIO
        # GPIO.output(PIN_ELECTROMAGNET, GPIO.HIGH)
        time.sleep(0.05)  # 吸合延时

    def release(self):
        """电磁铁断电释放 (GPIO 控制, 待实现)"""
        logger.info('电磁铁: 断电释放 (GPIO 未接入)')
        # TODO: GPIO 置低
        # GPIO.output(PIN_ELECTROMAGNET, GPIO.LOW)
        time.sleep(0.05)  # 释放延时

    # ================================================================
    #  归零
    # ================================================================
    def home_all(self, rpm: int = 25, accel: int = 200):
        """全部轴归零"""
        logger.info('--- 全部轴归零 ---')

        # 1. R 轴回零
        self.rotate_home(rpm, accel)

        # 2. X/Y 回到 (0, 0)
        self.move_to(0.0, 0.0, rpm, accel)

        # 3. Z 放下
        if self._z_up:
            self.z_down()

        logger.info('--- 归零完成 ---')

    # ================================================================
    #  状态查询
    # ================================================================
    def get_position(self) -> Tuple[float, float, float, float]:
        """获取当前位置: (x_mm, y_mm, z_deg, r_deg)"""
        self._sync_position()
        z_deg = self._read_position_deg(3)
        r_deg = self._read_position_deg(4) * self.DIR_SIGN[4]  # 转回用户坐标
        return (self._x_mm, self._y_mm, z_deg, r_deg)

    def status_report(self) -> str:
        """生成状态报告"""
        lines = ['=' * 50, '  Actuator 状态报告', '=' * 50]
        try:
            x, y, z, r = self.get_position()
            lines.append(f'  坐标: X={x:.1f}mm  Y={y:.1f}mm')
            lines.append(f'  Z:   {z:.1f}°  ({"抬升" if self._z_up else "放下"})')
            lines.append(f'  R:   {r:.1f}°')
            lines.append('')

            for addr in [1, 2, 3, 4]:
                name = self.MOTOR_NAMES[addr]
                try:
                    ma = self._read_current_ma(addr)
                    temp = self._read_temperature(addr)
                    flags = self._read_status_flags(addr)
                    status = []
                    if flags >= 0:
                        if flags & 0x01: status.append('使能')
                        if flags & 0x02: status.append('到位')
                        if flags & 0x04: status.append('堵转!')
                    lines.append(f'  {name}: {ma}mA  {temp:.0f}°C  {"|".join(status) if status else "—"}')
                except Exception:
                    lines.append(f'  {name}: 读取失败')
        except Exception as e:
            lines.append(f'  读取异常: {e}')

        lines.append('=' * 50)
        return '\n'.join(lines)
