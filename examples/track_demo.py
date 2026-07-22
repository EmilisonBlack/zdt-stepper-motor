"""
examples/track_demo.py — 视觉追踪演示

模拟摄像头追踪场景: 目标随机跳变, 控制器只知道像素偏差。

用法:
    python examples/track_demo.py
"""
import sys
import os
import time
import random
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from python.pixel_tracker import PixelTracker


def main():
    port = 'COM9'
    PIX_PER_DEG = 640 / 60  # 像素/度 (模拟用)

    tracker = PixelTracker(port)
    tracker.motor.reset_position()
    tracker.motor.enable()

    print("=" * 60)
    print("PixelTracker Demo — Random Target Tracking")
    print(f"Params: KP={tracker.KP}, MAX={tracker.MAX_SPEED}RPM, "
          f"MIN={tracker.MIN_SPEED}RPM")
    print("=" * 60)

    # Generate random targets
    targets = []
    t = 0
    for _ in range(6):
        angle = random.uniform(15, 165)
        targets.append((t + 2, angle))
        t += random.uniform(4, 6)

    current = 0.0
    idx = 0
    stats = []
    t0 = time.perf_counter()

    while True:
        elapsed = time.perf_counter() - t0
        if elapsed > 25:
            break

        if idx < len(targets) and elapsed >= targets[idx][0]:
            current = targets[idx][1]
            idx += 1
            print(f"\n>>> New target: {current:.0f} deg")

        pos = tracker.motor.read_position()
        err = current - pos
        px = err * PIX_PER_DEG + random.gauss(0, 3)
        info = tracker.step(px)

        if len(stats) % 10 == 0:
            print(f"  t={elapsed:.1f}s  pos={pos:6.1f}  "
                  f"err={err:+.1f}  [{info['mode']}]")
        stats.append(abs(err))
        time.sleep(0.05)

    avg = sum(stats) / len(stats)
    recent = [e for e in stats if len(stats) - stats.index(e) < 60]
    stable = sum(recent) / len(recent) if recent else 0
    print(f"\n--- Stats ---")
    print(f"Average error: {avg:.1f} deg")
    print(f"Last 3s error: {stable:.1f} deg")
    print(f"Samples: {len(stats)}")

    tracker.close()


if __name__ == '__main__':
    main()
