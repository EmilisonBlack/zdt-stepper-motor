"""
PixelTracker — 基于像素偏差的视觉追踪控制器
=============================================

适用于: 摄像头视觉反馈 + 步进电机连续追踪

原理:
    speed = clamp(KP × |pixel_error|, MIN_SPEED, MAX_SPEED)
    direction = sign(pixel_error)

无需精确角度, 只需知道目标偏离画面中心的像素数。
低增益比例控制天然防震荡, 最低速蠕动到位。

用法:
    tracker = PixelTracker(port='COM9')
    tracker.motor.enable()

    for each_camera_frame:
        px = detect_target()       # 你的视觉检测
        info = tracker.step(px)    # 一步追踪
        if info['mode'] == 'hold':
            print('到位')
"""

import time
import sys
import os
sys.path.insert(0, os.path.dirname(__file__))
from zdt_stepper import ZDTStepper


class PixelTracker:
    """基于像素偏差的比例速度追踪控制器"""

    def __init__(self, port='COM9', addr=1):
        """初始化

        Args:
            port: 串口名称
            addr: 驱动器地址
        """
        self.motor = ZDTStepper(port, addr)

        # ═══════════════════════════════════════════
        #  用户可调参数 (这是唯一需要动的地方)
        # ═══════════════════════════════════════════
        self.KP = 0.05           # RPM/像素 — 比例增益
        self.MAX_SPEED = 28      # RPM — 最高转速
        self.MIN_SPEED = 1       # RPM — 最低蠕动速度
        self.DEADBAND_PX = 3     # 像素 — 死区宽度
        self.ACCEL = 1500        # 0.1RPM/s — 加减速
        self.HOLD_MA = 25        # mA — 到位保持电流

        # 内部状态
        self._last_pos = 0.0
        self._last_time = time.perf_counter()

    def step(self, pixel_error: float) -> dict:
        """执行一步追踪

        Args:
            pixel_error: 目标在画面中的像素偏差
                         正 = 偏右 → 电机顺时针转
                         负 = 偏左 → 电机逆时针转

        Returns:
            dict: {pos, vel, mode, speed, desc}
        """
        pos = self.motor.read_position()
        vel = self.motor.read_speed()

        abs_px = abs(pixel_error)

        if abs_px < self.DEADBAND_PX:
            # ─── 死区内: 停止 + 保持力矩 ───
            self.motor.stop()
            time.sleep(0.01)
            self.motor.torque_mode(dir_cw=True,
                                   current_ma=self.HOLD_MA,
                                   accel=200)
            return {
                'pos': pos,
                'vel': vel,
                'mode': 'hold',
                'speed': 0,
                'desc': f'保持 {self.HOLD_MA}mA (px={abs_px:.0f})',
            }

        # ─── 比例速度 ───
        speed = self.KP * abs_px
        speed = max(self.MIN_SPEED, min(self.MAX_SPEED, speed))

        # 执行速度模式
        self.motor.speed_mode(dir_cw=(pixel_error > 0),
                              rpm=int(speed),
                              accel=self.ACCEL)

        self._last_pos = pos

        return {
            'pos': pos,
            'vel': vel,
            'mode': 'track',
            'speed': speed,
            'desc': f'{int(speed)}RPM (px={abs_px:.0f})',
        }

    def close(self):
        """关闭串口"""
        self.motor.stop()
        self.motor.close()


# ====================================================================
#  自测: 随机目标追踪演示
# ====================================================================
if __name__ == '__main__':
    import random

    print("=" * 60)
    print("PixelTracker 自测 — 随机目标追踪")
    print("=" * 60)

    tracker = PixelTracker(port='COM9')
    tracker.motor.reset_position()
    tracker.motor.enable()

    print(f"\n参数: KP={tracker.KP}  MAX={tracker.MAX_SPEED}RPM  "
          f"MIN={tracker.MIN_SPEED}RPM  DEADBAND={tracker.DEADBAND_PX}px\n")

    # 生成随机目标
    targets = []
    t = 0
    for _ in range(8):
        angle = random.uniform(10, 170)
        targets.append((t + 2, angle))
        t += random.uniform(3, 5)

    current_target = 0.0
    target_idx = 0
    log = []
    t_start = time.perf_counter()
    PIX_PER_DEG = 640 / 60

    print(f"{'时间':>5} {'目标':>6} {'位置':>7} {'误差':>6} {'命令':>5} {'状态'}")
    print("-" * 50)

    while True:
        elapsed = time.perf_counter() - t_start
        if elapsed > 28:
            break

        # 切换目标
        if target_idx < len(targets) and elapsed >= targets[target_idx][0]:
            current_target = targets[target_idx][1]
            target_idx += 1

        pos = tracker.motor.read_position()
        err = current_target - pos

        # 模拟像素测量
        px = err * PIX_PER_DEG + random.gauss(0, 3)

        # 执行追踪
        info = tracker.step(px)
        log.append({'t': elapsed, 'pos': pos, 'target': current_target, 'err': err})

        if len(log) % 10 == 0 or info['mode'] == 'hold':
            print(f"  {elapsed:4.1f}s  {current_target:5.0f}°  "
                  f"{pos:6.1f}°  {err:+5.1f}°  "
                  f"{int(info['speed']):3d}  [{info['mode']}]")

        time.sleep(0.05)

    # 统计
    errors = [abs(l['err']) for l in log]
    recent = [abs(l['err']) for l in log if l['t'] > log[-1]['t'] - 3]

    print("-" * 50)
    print(f"\n统计 ({len(log)} 帧, {elapsed:.0f}秒):")
    print(f"  全程平均误差: {sum(errors)/len(errors):.1f}°")
    print(f"  最大误差: {max(errors):.1f}°")
    print(f"  最后3秒平均误差: {sum(recent)/len(recent):.1f}°")
    print(f"  误差<1°占比: {sum(1 for e in errors if e<1)/len(errors)*100:.0f}%")

    tracker.close()
    print("\n✅ 自测完成")
