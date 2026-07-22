"""
ZDT_X_V2 闭环步进电机 Python 控制库
====================================
基于 ctypes 调用 zdt_stepper.dll，提供 Pythonic 的 API。

用法:
    from zdt_stepper import ZDTStepper
    
    motor = ZDTStepper('COM9', addr=1)
    motor.enable()
    motor.speed_mode(dir_cw=True, rpm=30, accel=100)
    motor.stop()
    pos = motor.read_position()  # 返回浮点数, 单位 °
    motor.close()
"""

import ctypes
import ctypes.util
import os
from typing import Optional, Tuple

# 查找 DLL 路径
_DLL_DIR = os.path.join(os.path.dirname(__file__), '..', 'sdk')
_DLL_NAME = 'zdt_stepper.dll'


class ZDTStepperError(Exception):
    """ZDT 步进电机通信异常"""
    pass


class ZDTStepper:
    """
    ZDT_X_V2 闭环步进电机控制类

    封装了 zdt_stepper.dll 的所有功能，提供 Pythonic 的接口。
    所有角度单位均为 **度 (°)**，内部自动转换为驱动器的 0.1° 单位。
    转速单位为 RPM。
    """

    # 方向常量
    DIR_CW = 0    # 顺时针
    DIR_CCW = 1   # 逆时针

    # 回零模式
    HOME_NEAREST = 0
    HOME_DIR = 1
    HOME_COLLISION = 2
    HOME_LIMIT_SW = 3

    # 控制模式
    MODE_PULSE_OFF = 0
    MODE_OPENLOOP = 1
    MODE_CLOSEDLOOP = 2
    MODE_LIMIT_SW = 3

    def __init__(self, port: str = 'COM9', addr: int = 1):
        """打开串口并初始化驱动器

        Args:
            port: 串口名称, 如 'COM9'
            addr: 驱动器地址 (1-255)
        """
        self._addr = addr
        self._load_dll()
        self._open(port)

    def _load_dll(self):
        dll_path = os.path.abspath(os.path.join(_DLL_DIR, _DLL_NAME))
        if not os.path.exists(dll_path):
            # 尝试在当前目录找
            dll_path = os.path.abspath(_DLL_NAME)
        if not os.path.exists(dll_path):
            raise ZDTStepperError(f'找不到 {_DLL_NAME}，请先编译')
        self._dll = ctypes.CDLL(dll_path)
        self._setup_argtypes()

    def _setup_argtypes(self):
        d = self._dll

        # platform_uart_open(char* port, int baudrate) -> int
        d.platform_uart_open.argtypes = [ctypes.c_char_p, ctypes.c_int]
        d.platform_uart_open.restype = ctypes.c_int

        # platform_uart_close()
        d.platform_uart_close.argtypes = []
        d.platform_uart_close.restype = None

        # zdt_Init(uint8_t addr)
        d.zdt_Init.argtypes = [ctypes.c_uint8]
        d.zdt_Init.restype = None

        # --- 读参数 ---
        d.zdt_ReadVersion.argtypes = [ctypes.c_char_p]
        d.zdt_ReadVersion.restype = ctypes.c_int

        for func_name in ['zdt_ReadBusVoltage', 'zdt_ReadPhaseCurrent',
                          'zdt_ReadActualCurrent']:
            getattr(d, func_name).argtypes = [ctypes.POINTER(ctypes.c_uint32)]
            getattr(d, func_name).restype = ctypes.c_int

        d.zdt_ReadMotorParams.argtypes = [ctypes.POINTER(ctypes.c_uint16),
                                          ctypes.POINTER(ctypes.c_uint16)]
        d.zdt_ReadMotorParams.restype = ctypes.c_int

        d.zdt_ReadHallAngle.argtypes = [ctypes.POINTER(ctypes.c_uint16)]
        d.zdt_ReadHallAngle.restype = ctypes.c_int

        d.zdt_ReadEncoder.argtypes = [ctypes.POINTER(ctypes.c_int32)]
        d.zdt_ReadEncoder.restype = ctypes.c_int

        d.zdt_ReadSpeed.argtypes = [ctypes.POINTER(ctypes.c_int16)]
        d.zdt_ReadSpeed.restype = ctypes.c_int

        for func_name in ['zdt_ReadPosition', 'zdt_ReadTargetPosition',
                          'zdt_ReadAbsolutePosition', 'zdt_ReadPositionError']:
            getattr(d, func_name).argtypes = [ctypes.POINTER(ctypes.c_int32)]
            getattr(d, func_name).restype = ctypes.c_int

        d.zdt_ReadStatusFlags.argtypes = [ctypes.POINTER(ctypes.c_uint8)]
        d.zdt_ReadStatusFlags.restype = ctypes.c_int

        d.zdt_ReadHomeStatus.argtypes = [ctypes.POINTER(ctypes.c_uint8)]
        d.zdt_ReadHomeStatus.restype = ctypes.c_int

        d.zdt_ReadTemperature.argtypes = [ctypes.POINTER(ctypes.c_int16)]
        d.zdt_ReadTemperature.restype = ctypes.c_int

        # --- 控制 ---
        d.zdt_Enable.argtypes = [ctypes.c_bool, ctypes.c_bool]
        d.zdt_Enable.restype = ctypes.c_int

        d.zdt_SpeedMode.argtypes = [ctypes.c_uint8, ctypes.c_uint16,
                                    ctypes.c_uint16, ctypes.c_bool]
        d.zdt_SpeedMode.restype = ctypes.c_int

        d.zdt_PositionModeRel.argtypes = [ctypes.c_uint8, ctypes.c_uint16,
                                          ctypes.c_uint16, ctypes.c_uint32,
                                          ctypes.c_bool]
        d.zdt_PositionModeRel.restype = ctypes.c_int

        d.zdt_PositionModeAbs.argtypes = [ctypes.c_uint8, ctypes.c_uint16,
                                          ctypes.c_uint16, ctypes.c_uint32,
                                          ctypes.c_bool]
        d.zdt_PositionModeAbs.restype = ctypes.c_int

        d.zdt_Stop.argtypes = [ctypes.c_bool]
        d.zdt_Stop.restype = ctypes.c_int

        d.zdt_SyncMotion.argtypes = []
        d.zdt_SyncMotion.restype = ctypes.c_int

        d.zdt_TorqueMode.argtypes = [ctypes.c_uint8, ctypes.c_uint16,
                                     ctypes.c_uint16, ctypes.c_bool]
        d.zdt_TorqueMode.restype = ctypes.c_int

        # --- 模式 ---
        d.zdt_SetCtrlMode.argtypes = [ctypes.c_bool, ctypes.c_uint8]
        d.zdt_SetCtrlMode.restype = ctypes.c_int

        # --- 回零 ---
        d.zdt_SetHomeZero.argtypes = [ctypes.c_bool]
        d.zdt_SetHomeZero.restype = ctypes.c_int

        d.zdt_TriggerHome.argtypes = [ctypes.c_uint8, ctypes.c_bool]
        d.zdt_TriggerHome.restype = ctypes.c_int

        d.zdt_AbortHome.argtypes = []
        d.zdt_AbortHome.restype = ctypes.c_int

        # --- 工具 ---
        d.zdt_ResetPosition.argtypes = []
        d.zdt_ResetPosition.restype = ctypes.c_int

        d.zdt_ReleaseStall.argtypes = []
        d.zdt_ReleaseStall.restype = ctypes.c_int

        d.zdt_FactoryReset.argtypes = []
        d.zdt_FactoryReset.restype = ctypes.c_int

        d.zdt_CalibrateEncoder.argtypes = []
        d.zdt_CalibrateEncoder.restype = ctypes.c_int

        # --- 高级 ---
        d.zdt_SetHomeParams.argtypes = [
            ctypes.c_uint8, ctypes.c_uint8, ctypes.c_uint16,
            ctypes.c_uint32, ctypes.c_uint16, ctypes.c_uint16,
            ctypes.c_uint16, ctypes.c_bool]
        d.zdt_SetHomeParams.restype = ctypes.c_int

        d.zdt_SetAddrID.argtypes = [ctypes.c_uint8, ctypes.c_bool]
        d.zdt_SetAddrID.restype = ctypes.c_int

        d.zdt_SetMicrostep.argtypes = [ctypes.c_uint16, ctypes.c_bool]
        d.zdt_SetMicrostep.restype = ctypes.c_int

    # ----------------------------------------------------------------
    #  内部方法
    # ----------------------------------------------------------------
    def _open(self, port: str):
        ret = self._dll.platform_uart_open(port.encode(), 115200)
        if ret != 0:
            raise ZDTStepperError(f'打开 {port} 失败 (ret={ret})')
        self._dll.zdt_Init(self._addr)

    def _check(self, ret: int, context: str = ''):
        if ret != 0:
            raise ZDTStepperError(f'{context} 失败 (ret={ret})')

    def _read_u16(self, func) -> int:
        val = ctypes.c_uint16()
        self._check(func(ctypes.byref(val)))
        return val.value

    def _read_u32(self, func) -> int:
        val = ctypes.c_uint32()
        self._check(func(ctypes.byref(val)))
        return val.value

    def _read_i16(self, func) -> int:
        val = ctypes.c_int16()
        self._check(func(ctypes.byref(val)))
        return val.value

    def _read_i32(self, func) -> int:
        val = ctypes.c_int32()
        self._check(func(ctypes.byref(val)))
        return val.value

    # ----------------------------------------------------------------
    #  状态读取
    # ----------------------------------------------------------------
    def read_version(self) -> str:
        """读取固件版本号"""
        buf = ctypes.create_string_buffer(32)
        self._check(self._dll.zdt_ReadVersion(buf), 'ReadVersion')
        # 版本号可能是二进制格式, 尝试解析
        raw = buf.raw[:buf.value.find(b'\x00')] if b'\x00' in buf.value else buf.value
        try:
            return raw.decode('ascii')
        except:
            return f'V{raw[0]}.{raw[1]}.{raw[2]}' if len(raw) >= 3 else raw.hex()

    def read_bus_voltage(self) -> float:
        """读取总线电压 (V)"""
        return self._read_u32(self._dll.zdt_ReadBusVoltage) / 1000.0

    def read_phase_current(self) -> int:
        """读取设置电流 (mA)"""
        return self._read_u32(self._dll.zdt_ReadPhaseCurrent)

    def read_actual_current(self) -> int:
        """读取实际相电流 (mA)"""
        return self._read_u32(self._dll.zdt_ReadActualCurrent)

    def read_motor_params(self) -> Tuple[float, float]:
        """读取电机参数: (电阻 mΩ, 电感 uH)"""
        res = ctypes.c_uint16()
        ind = ctypes.c_uint16()
        self._check(self._dll.zdt_ReadMotorParams(ctypes.byref(res), ctypes.byref(ind)))
        return res.value / 10.0, ind.value / 10.0

    def read_hall_angle(self) -> float:
        """读取霍尔角度 (度)"""
        raw = self._read_u16(self._dll.zdt_ReadHallAngle)
        return raw * 360.0 / 16384.0

    def read_encoder(self) -> int:
        """读取编码器原始值 (0-65535)"""
        return self._read_i32(self._dll.zdt_ReadEncoder)

    def read_speed(self) -> float:
        """读取实时转速 (RPM), 负值=反转"""
        return self._read_i16(self._dll.zdt_ReadSpeed)

    def read_position(self) -> float:
        """读取实时位置 (度), 负值=反转方向"""
        return self._read_i32(self._dll.zdt_ReadPosition) / 10.0

    def read_target_position(self) -> float:
        """读取目标位置 (度)"""
        return self._read_i32(self._dll.zdt_ReadTargetPosition) / 10.0

    def read_absolute_position(self) -> float:
        """读取绝对位置 (度)"""
        return self._read_i32(self._dll.zdt_ReadAbsolutePosition) / 10.0

    def read_position_error(self) -> float:
        """读取位置误差 (度)"""
        return self._read_i32(self._dll.zdt_ReadPositionError) / 10.0

    def read_temperature(self) -> int:
        """读取驱动器温度 (℃)"""
        return self._read_i16(self._dll.zdt_ReadTemperature)

    def read_status_flags(self) -> int:
        """读取状态标志位"""
        return self._read_u16  # 临时, 实际用 uint8

    def read_status(self) -> dict:
        """读取全部状态, 返回字典"""
        flags = ctypes.c_uint8()
        self._dll.zdt_ReadStatusFlags(ctypes.byref(flags))
        return {
            'enabled': bool(flags.value & 0x01),
            'in_position': bool(flags.value & 0x02),
            'stall': bool(flags.value & 0x04),
            'position': self.read_position(),
            'speed': self.read_speed(),
            'voltage': self.read_bus_voltage(),
            'current_set': self.read_phase_current(),
            'current_actual': self.read_actual_current(),
            'temperature': self.read_temperature(),
        }

    # ----------------------------------------------------------------
    #  电机控制
    # ----------------------------------------------------------------
    def enable(self):
        """使能电机 (锁定)"""
        self._check(self._dll.zdt_Enable(True, False), 'Enable')

    def disable(self):
        """不使能电机 (释放力矩)"""
        self._check(self._dll.zdt_Enable(False, False), 'Disable')

    def speed_mode(self, dir_cw: bool = True, rpm: int = 30,
                   accel: int = 100, wait: bool = False):
        """速度模式

        Args:
            dir_cw: True=顺时针, False=逆时针
            rpm: 目标转速
            accel: 加速度 (0.1RPM/s 单位, 如 100=10.0RPM/s)
            wait: 是否等待到位 (对速度模式无意义, 保持 False)
        """
        d = self.DIR_CW if dir_cw else self.DIR_CCW
        self._check(self._dll.zdt_SpeedMode(d, rpm, accel, wait), 'SpeedMode')

    def stop(self):
        """减速停止"""
        self._check(self._dll.zdt_Stop(False), 'Stop')

    def position_rel(self, dir_cw: bool = True, rpm: int = 30,
                     accel: int = 200, degrees: float = 90.0, wait: bool = False):
        """相对位置模式

        Args:
            dir_cw: 方向
            rpm: 转速
            accel: 加速度 (0.1RPM/s)
            degrees: 目标角度 (度)
            wait: 是否等待到位
        """
        d = self.DIR_CW if dir_cw else self.DIR_CCW
        pulses = int(degrees * 10)  # 转为 0.1° 单位
        self._check(self._dll.zdt_PositionModeRel(d, rpm, accel, pulses, wait),
                    'PositionModeRel')

    def position_abs(self, dir_cw: bool = True, rpm: int = 30,
                     accel: int = 200, degrees: float = 0.0, wait: bool = False):
        """绝对位置模式"""
        d = self.DIR_CW if dir_cw else self.DIR_CCW
        pulses = int(degrees * 10)
        self._check(self._dll.zdt_PositionModeAbs(d, rpm, accel, pulses, wait),
                    'PositionModeAbs')

    def torque_mode(self, dir_cw: bool = True, accel: int = 200,
                    current_ma: int = 50, wait: bool = False):
        """力矩模式 (电流控制)

        Args:
            dir_cw: 方向
            accel: 电流爬升速率 (Ma/s)
            current_ma: 目标电流 (mA)
            wait: 是否同步
        """
        d = self.DIR_CW if dir_cw else self.DIR_CCW
        self._check(self._dll.zdt_TorqueMode(d, accel, current_ma, wait),
                    'TorqueMode')

    def sync(self):
        """同步执行所有已发送的异步命令"""
        self._check(self._dll.zdt_SyncMotion(), 'SyncMotion')

    # ----------------------------------------------------------------
    #  回零
    # ----------------------------------------------------------------
    def set_home_zero(self, save: bool = False):
        """将当前位置设为机械零点"""
        self._check(self._dll.zdt_SetHomeZero(save), 'SetHomeZero')

    def trigger_home(self, mode: int = 0):
        """触发回零"""
        self._check(self._dll.zdt_TriggerHome(mode, False), 'TriggerHome')

    def abort_home(self):
        """中止回零"""
        self._check(self._dll.zdt_AbortHome(), 'AbortHome')

    # ----------------------------------------------------------------
    #  工具
    # ----------------------------------------------------------------
    def reset_position(self):
        """当前位置清零"""
        self._check(self._dll.zdt_ResetPosition(), 'ResetPosition')

    def release_stall(self):
        """解除堵转保护"""
        self._check(self._dll.zdt_ReleaseStall(), 'ReleaseStall')

    def calibrate_encoder(self):
        """编码器校准"""
        self._check(self._dll.zdt_CalibrateEncoder(), 'CalibrateEncoder')

    # ----------------------------------------------------------------
    #  资源管理
    # ----------------------------------------------------------------
    def close(self):
        """关闭串口"""
        if hasattr(self, '_dll'):
            try:
                self._dll.platform_uart_close()
            except:
                pass

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()

    def __del__(self):
        self.close()
