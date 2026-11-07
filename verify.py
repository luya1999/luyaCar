#!/usr/bin/env python3
"""verify.py —— 一键回归：无头跑 7 个场景，检查 v0.4 的安全与跟随规则。

用法：
    python verify.py
    C:/Tools/ananconda3/python.exe verify.py

不弹窗口、不依赖显示器。任何场景出现「安全距违规 / 位置瞬移 / 人狗穿模」
都会让脚本以退出码 1 结束，适合改完 config.py 后立刻自检。
"""

import math
import sys

from cube_chase import config
from cube_chase.simulation import World

LAPS = 5


def run(label: str, **kw) -> tuple[bool, str]:
    w = World()
    w.autonomous = kw.get("auto_ped", True)
    if kw.get("manual"):
        w.person.mode = "manual"

    frames = int(LAPS * w.track.total_length / config.PERSON_SPEED * 60)
    dt = 1 / 60
    min_owner = float("inf")

    for i in range(frames):
        d = None
        if kw.get("manual") == "circle":
            d = (math.cos(i / frames * 6.0), math.sin(i / frames * 6.0))
        elif kw.get("manual") == "stop":
            d = (1.0, 0.0) if i < frames * 0.55 else None

        o = w.obstacles[0] if kw.get("drag") else None
        t = None
        if o is not None:
            if kw["drag"] == "sweep":
                t = (w.dog.x + 180, w.dog.y + 120 * math.sin(i / 90.0))
            else:
                t = (w.dog.x + 60 * math.sin(i / 120.0),
                     w.dog.y + 60 * math.cos(i / 120.0))

        w.update(dt, manual_dir=d, drag_obstacle=o, drag_target=t,
                 patrol=kw.get("patrol", False))
        min_owner = min(min_owner, w.dog.surface_gap(w.person))

    g = sorted(w.gaps) or [0.0]
    n = len(g)
    ok = (w.violation_frames == 0
          and w.teleport_frames == 0
          and min_owner > 0
          and w.max_speed_seen <= config.DOG_MAX_SPEED + 1.0)
    line = (f"{'OK ' if ok else 'FAIL'} {label:<16s} "
            f"跟距中位={g[n//2]:4.0f} P90={g[n*9//10]:4.0f} | "
            f"障碍={w.min_clearance_seen:5.1f} 行人={w.min_pedestrian_gap:5.1f} "
            f"违规={w.violation_frames:3d} 瞬移={w.teleport_frames:3d} "
            f"人狗最小={min_owner:4.0f} 最高速={w.max_speed_seen/100:.2f}")
    return ok, line


def main() -> int:
    cases = [
        ("默认5圈", {}),
        ("行人自主关闭", {"auto_ped": False}),
        ("全体巡逻", {"patrol": True}),
        ("手操人绕圈", {"manual": "circle"}),
        ("障碍侧向扫过", {"drag": "sweep"}),
        ("障碍追着压", {"drag": "chase"}),
        ("人中途急停", {"manual": "stop"}),
    ]
    print(f"v0.4 回归 · {LAPS} 圈/场景 · 安全距下限 {config.SAFE_CLEARANCE:.0f}cm "
          f"· 跟距目标 {config.FOLLOW_GAP_MIN:.0f}~{config.FOLLOW_GAP_MAX:.0f}cm")
    print("-" * 118)
    results = []
    for label, kw in cases:
        ok, line = run(label, **kw)
        results.append(ok)
        print(line)
    print("-" * 118)
    passed = sum(results)
    print(f"通过 {passed}/{len(results)} 个场景")
    return 0 if all(results) else 1


if __name__ == "__main__":
    sys.exit(main())
