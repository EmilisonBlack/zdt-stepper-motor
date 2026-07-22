"""
真实视觉追踪模拟
================
模拟摄像头视觉追踪的真实场景:

物理世界:
  真实目标在 X° 位置 (未知)
  电机当前位置已知 (编码器)
  误差 = 真实目标 - 电机位置 (只有上帝知道)

摄像头:
  将角度误差投影到像素平面上
  加入检测噪声和量化
  返回: "目标偏右 320 像素"

控制器视角:
  只知道 "像素偏差 = +320"
  不知道这对应多少度
  只能: "像素偏右多就转快点, 偏左多就转慢点或反转"

映射关系 (控制器不知道!):
  640px 画面 / 60° FOV ≈ 10.67 像素/度
"""

import time
import sys
import os
import math
import random
sys.path.insert(0, os.path.dirname(__file__))
from zdt_stepper import ZDTStepper


class CameraSim:
    """模拟摄像头

    物理世界→像素映射:
       角度误差 → 像素偏移 (带噪声和量化)

    控制器视角:
       只知道像素值, 不知道对应多少度
    """

    def __init__(self, fov_deg=60, frame_width=640, pixel_noise=3):
        self.fov = fov_deg
        self.width = frame_width
        self.px_per_deg = frame_width / fov_deg  # 物理参数, 控制器不知道!
        self.pixel_noise = pixel_noise

    def measure(self, angle_error_deg):
        """将真实角度误差转为像素测量值"""
        # 物理: 角度→像素
        pixel_offset = angle_error_deg * self.px_per_deg

        # 检测噪声
        noisy_px = pixel_offset + random.gauss(0, self.pixel_noise)

        # 像素量化
        measured_px = round(noisy_px / 1.0) * 1.0

        return measured_px


class PixelController:
    """基于像素误差的控制器

    不知道角度! 只知道像素偏差。
    策略: speed = KP * pixel_error (带限幅)
    """

    def __init__(self, port='COM9'):
        self.motor = ZDTStepper(port)

        # 控制器参数 (直接基于像素)
        self.KP = 0.12           # RPM/像素
        self.MAX_SPEED = 30      # 最大 RPM
        self.MIN_SPEED = 4       # 最低有效 RPM
        self.ACCEL = 2000

        self.DEADBAND_PX = 5     # 像素死区
        self.HOLD_MA = 25

        # 滤波
        self.FILTER = 0.3        # 低通滤波系数
        self._filtered_px = 0.0

    def step(self, pixel_error: float) -> dict:
        """基于像素误差执行一步控制

        Args:
            pixel_error: 摄像头检测的像素偏差 (正=偏右)
        """
        pos = self.motor.read_position()
        vel = self.motor.read_speed()

        # 低通滤波
        self._filtered_px = self._filtered_px * (1 - self.FILTER) + \
                            pixel_error * self.FILTER
        px = self._filtered_px
        abs_px = abs(px)

        if abs_px < self.DEADBAND_PX:
            # 死区内: 停止 + 保持
            self.motor.stop()
            time.sleep(0.01)
            self.motor.torque_mode(dir_cw=True, current_ma=self.HOLD_MA,
                                   accel=200)
            mode = 'hold'
            desc = f'死区({abs_px:.0f}px)'
            speed_cmd = 0
        else:
            # 比例速度: speed = KP * |pixel|
            speed = self.KP * abs_px
            speed = max(self.MIN_SPEED, min(self.MAX_SPEED, speed))

            dir_cw = px > 0  # 像素偏右→CW
            self.motor.speed_mode(dir_cw=dir_cw, rpm=int(speed),
                                  accel=self.ACCEL)
            mode = 'track'
            desc = f'{int(speed)}RPM ({abs_px:.0f}px)'
            speed_cmd = speed

        return {
            'pos': pos,
            'vel': vel,
            'pixel_raw': pixel_error,
            'pixel_filtered': px,
            'mode': mode,
            'desc': desc,
            'speed_cmd': speed_cmd,
        }

    def close(self):
        self.motor.stop()
        self.motor.close()


# ====================================================================
#  测试: 模拟真实视觉追踪
# ====================================================================
def run_test(controller, camera, true_target_deg, max_frames=80):
    """运行一次视觉追踪测试

    控制器不知道 true_target_deg, 只知道 camera.measure() 返回的像素值
    """
    results = []
    start_time = time.perf_counter()
    converged = False

    for frame in range(max_frames):
        pos = controller.motor.read_position()
        t = time.perf_counter() - start_time

        # 真实误差 (只有上帝知道)
        true_error = true_target_deg - pos

        # 摄像头测量 (控制器只知道这个)
        pixel_measurement = camera.measure(true_error)

        # 控制器执行 (基于像素)
        info = controller.step(pixel_measurement)
        info['frame'] = frame
        info['time'] = t
        info['true_error'] = true_error
        results.append(info)

        if abs(true_error) < 0.5:
            if not converged:
                converged = True
                converge_time = t

        time.sleep(0.05)

    return results, converged


def print_trace(results, title=""):
    """打印追踪轨迹"""
    print(f"\n--- {title} ---")
    for r in results[::5] + ([results[-1]] if len(results) > 5 else []):
        print(f"  f{r['frame']:2d}  t={r['time']:.2f}s  "
              f"pos={r['pos']:6.1f}°  "
              f"err={r['true_error']:+5.1f}°  "
              f"px={r['pixel_raw']:+4.0f}px  "
              f"滤波={r['pixel_filtered']:+4.0f}px  "
              f"[{r['mode']}] {r['desc']}")


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--demo', action='store_true', help='演示多种条件')
    args = parser.parse_args()

    # ─── 单次测试 ───
    if not args.demo:
        print("=" * 60)
        print("真实视觉追踪模拟")
        print("  控制器只知道像素偏差, 不知道角度!")
        print("=" * 60)

        ctrl = PixelController(port='COM9')
        time.sleep(0.3)
        ctrl.motor.reset_position()
        time.sleep(0.2)
        ctrl.motor.enable()

        # 场景: 目标在 90° 方向, 噪声 5px
        cam = CameraSim(fov_deg=60, frame_width=640, pixel_noise=5)
        true_target = 90

        print(f"\n摄像头: FOV={cam.fov}°  {cam.width}px  噪声=±{cam.pixel_noise}px")
        print(f"控制器: KP={ctrl.KP} RPM/px  MAX={ctrl.MAX_SPEED}RPM  死区={ctrl.DEADBAND_PX}px")
        print(f"真实目标: {true_target}° (控制器不知道!)")
        print(f"物理映射: {cam.px_per_deg:.1f}px/° (控制器也不知道!)\n")

        results, converged = run_test(ctrl, cam, true_target)
        print_trace(results, "追踪轨迹")

        if converged:
            final_err = abs(results[-1]['true_error'])
            print(f"\n✅ 到位! 最终误差={final_err:.2f}°")
        else:
            print(f"\n❌ 未到位, 最终误差={abs(results[-1]['true_error']):.2f}°")

        ctrl.close()

    # ─── 多条件测试 ───
    if args.demo:
        print("=" * 60)
        print("多条件追踪测试")
        print("=" * 60)

        scenarios = [
            ("近距小噪声", 45, 2),
            ("近距大噪声", 45, 8),
            ("中距小噪声", 90, 2),
            ("中距大噪声", 90, 8),
            ("远距小噪声", 180, 2),
            ("远距大噪声", 180, 8),
            ("反向中距", -90, 3),
        ]

        for name, target, noise in scenarios:
            ctrl = PixelController(port='COM9')
            time.sleep(0.2)
            ctrl.motor.reset_position()
            time.sleep(0.1)
            ctrl.motor.enable()
            time.sleep(0.1)

            cam = CameraSim(fov_deg=60, frame_width=640, pixel_noise=noise)
            results, converged = run_test(ctrl, cam, target, max_frames=60)

            final_err = abs(results[-1]['true_error'])
            print(f"\n  [{name}] 目标={target}° 噪声={noise}px  "
                  f"{'✅' if converged else '❌'}  "
                  f"最终误差={final_err:.1f}°  "
                  f"到位时间={results[-1]['time']:.1f}s")

            ctrl.close()
