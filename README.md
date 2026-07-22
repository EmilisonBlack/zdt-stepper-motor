# ZDT_X_V2 Closed-Loop Stepper Motor Control Library

适用于张大头 ZDT_X_V2 固件的闭环步进电机驱动器 (X28/X35/X42/X57)

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

---

## 概述

完整、可部署的步进电机驱动控制方案：

| 层级 | 内容 | 说明 |
|------|------|------|
| **C SDK** | `sdk/` | 完整自定义 0x6B 协议封装，30+ API |
| **Python 绑定** | `python/zdt_stepper.py` | ctypes DLL 封装，Pythonic API |
| **追踪控制器** | `python/pixel_tracker.py` | 视觉追踪比例控制 ⭐ |
| **文档** | `docs/` | 中文技术手册 + 最佳实践 |

## 快速开始

### 硬件连接

```
驱动器         USB2TTL
R/A/H  ──────  TXD
T/B/L  ──────  RXD
Gnd    ──────  GND
```

**OLED 关键设置**: `P_Serial=UART_FUN` · `Response=Receive` · `Checksum=0x6B` · `UartBaud=115200`

### 安装使用

```python
# 读参数
from python.zdt_stepper import ZDTStepper
m = ZDTStepper('COM9')
print(f"位置: {m.read_position():.1f}°")
print(f"电压: {m.read_bus_voltage():.1f}V")

# 视觉追踪 (只需像素偏差)
from python.pixel_tracker import PixelTracker
t = PixelTracker('COM9')
t.motor.enable()
t.step(pixel_error=320)   # 目标偏右 320px → 顺时针转
```

## 目录结构

```
zdt-stepper-motor/
├── sdk/                        # C SDK
│   ├── include/zdt_stepper.h   #   完整 API 声明
│   ├── src/zdt_stepper.c       #   协议实现
│   └── zdt_stepper.dll         #   Windows 预编译 DLL
│
├── python/                     # Python 绑定
│   ├── __init__.py
│   ├── zdt_stepper.py          #   底层 DLL 封装
│   └── pixel_tracker.py        #   视觉追踪控制器 ⭐
│
├── examples/                   # 平台移植参考
│   ├── platform_win32.c        #   Windows 串口实现
│   ├── example_simple.c        #   POSIX 参考
│   └── example_stm32_hal.c     #   STM32 参考
│
├── docs/                       # 文档
│   └── *.md                    #   技术手册 + 最佳实践
│
├── Makefile                    # 编译脚本
└── README.md
```

## 核心 API

### 控制命令

| 功能 | C API | Python |
|------|-------|--------|
| 使能 | `zdt_Enable()` | `motor.enable()` |
| 速度模式 | `zdt_SpeedMode()` | `motor.speed_mode(rpm=30)` |
| 绝对位置 | `zdt_PositionModeAbs()` | `motor.position_abs(degrees=90)` |
| 相对位置 | `zdt_PositionModeRel()` | `motor.position_rel(degrees=45)` |
| 停止 | `zdt_Stop()` | `motor.stop()` |
| 力矩模式 | `zdt_TorqueMode()` | `motor.torque_mode(current_ma=50)` |

### 读参数

| 参数 | C API | Python |
|------|-------|--------|
| 位置 | `zdt_ReadPosition()` | `motor.read_position()` → float |
| 转速 | `zdt_ReadSpeed()` | `motor.read_speed()` → float |
| 电压 | `zdt_ReadBusVoltage()` | `motor.read_bus_voltage()` → float |
| 温度 | `zdt_ReadTemperature()` | `motor.read_temperature()` → int |
| 状态 | `zdt_ReadStatusFlags()` | `motor.read_status()` → dict |
| 编码器 | `zdt_ReadEncoder()` | `motor.read_encoder()` → int |

## 视觉追踪

适用于摄像头反馈场景：只知道目标偏离画面中心的像素数，不知道精确角度。

```python
from python.pixel_tracker import PixelTracker

t = PixelTracker('COM9')
t.motor.enable()

# 每帧调用一次 (20Hz)
for each_frame:
    px = detect_target()       # 你的视觉检测
    info = t.step(px)          # 一步追踪
    if info['mode'] == 'hold':
        print('目标已居中')
```

详见 `docs/视觉追踪控制最佳实践.md`。

## 通信协议

默认自定义 0x6B 协议 (OLED `Checksum=0x6B`):

```
[地址] [功能码] [参数...] [0x6B]
```

也可切换 Modbus-RTU (CRC16)。详见 `docs/ZDT_X_V2_技术手册.md`。

## 构建 SDK

```bash
# Windows (MinGW)
make dll

# Linux
make linux

# 清理
make clean
```

## License

MIT

## 致谢

- [张大头闭环伺服](https://zhangdatou.taobao.com) — 硬件与技术支持
- QQ 交流群: 262438510
