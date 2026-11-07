"""track.py —— 操场形状的跑道（stadium / 胶囊形）。

几何：两条水平直线段 + 左右两个半圆，中心线是一个"胶囊"的边界。
    - 脊柱：从 (-L/2, 0) 到 (+L/2, 0) 的水平线段
    - 中心线 = 距脊柱恰好为 R 的点集

弧长参数化（s 从"下直道左端"出发，逆时针一圈）：
    0 ~ L            下直道（左 → 右）
    L ~ L+πR         右弯（下 → 上）
    L+πR ~ 2L+πR    上直道（右 → 左）
    2L+πR ~ 2L+2πR  左弯（上 → 下）

另外提供 O(1) 的 frame(x, y) → (s, lateral)：
    s        = 这个点投影到中心线的弧长位置
    lateral  = 带符号的横向偏移，**正=朝跑道外侧，负=朝场芯（内圈）**
这个接口是"预测未来路线 / 人自动避障 / 栅格引导代价"三处共用的基础。
"""

import math

from . import config


class Track:
    def __init__(self, cx: float, cy: float) -> None:
        self.cx, self.cy = cx, cy
        self.L = config.STRAIGHT_LEN
        self.R = config.CURVE_RADIUS
        self.half = config.TRACK_HALF
        self.arc = math.pi * self.R
        self.total_length = 2 * self.L + 2 * self.arc

    # ---------- 正向：弧长 → 坐标 ----------

    def _frame_at(self, s: float):
        """返回 (x, y, tx, ty, nx, ny)：中心线点、单位切向、单位外法向。"""
        s = s % self.total_length
        L, R, cx, cy = self.L, self.R, self.cx, self.cy

        if s < L:                                   # 下直道
            x, y = cx - L / 2 + s, cy + R
            tx, ty = 1.0, 0.0
        elif s < L + self.arc:                      # 右弯
            th = math.pi / 2 - (s - L) / R
            x = cx + L / 2 + R * math.cos(th)
            y = cy + R * math.sin(th)
            tx, ty = math.sin(th), -math.cos(th)
        elif s < 2 * L + self.arc:                  # 上直道
            s2 = s - L - self.arc
            x, y = cx + L / 2 - s2, cy - R
            tx, ty = -1.0, 0.0
        else:                                       # 左弯
            s3 = s - 2 * L - self.arc
            th = -math.pi / 2 - s3 / R
            x = cx - L / 2 + R * math.cos(th)
            y = cy + R * math.sin(th)
            tx, ty = math.sin(th), -math.cos(th)

        # 外法向 = 切向在屏幕坐标(y向下)里顺时针转 90°，即 (-ty, tx)
        nx, ny = -ty, tx
        return x, y, tx, ty, nx, ny

    def point_at(self, s: float, lateral: float = 0.0) -> tuple[float, float]:
        """中心线上弧长 s 处，再沿外法向偏移 lateral 的点。"""
        x, y, _, _, nx, ny = self._frame_at(s)
        return x + nx * lateral, y + ny * lateral

    def tangent_at(self, s: float) -> tuple[float, float]:
        _, _, tx, ty, _, _ = self._frame_at(s)
        return tx, ty

    # ---------- 反向：坐标 → (弧长, 横向偏移) ----------

    def frame(self, x: float, y: float) -> tuple[float, float]:
        """O(1) 反解：给定世界坐标，返回 (s, lateral)。lateral 正=外侧。"""
        L, R, cx, cy = self.L, self.R, self.cx, self.cy
        left, right = cx - L / 2, cx + L / 2

        if left <= x <= right:                      # 两条直道
            if y >= cy:
                return x - left, (y - cy) - R
            return L + self.arc + (right - x), (cy - y) - R

        if x > right:                               # 右弯
            dx, dy = x - right, y - cy
            phi = math.atan2(dy, dx)
            phi = max(-math.pi / 2, min(math.pi / 2, phi))
            return L + R * (math.pi / 2 - phi), math.hypot(dx, dy) - R

        dx, dy = x - left, y - cy                   # 左弯
        phi = math.atan2(dy, dx)
        if phi > math.pi / 2:
            phi -= 2 * math.pi                       # 归一到 (-3π/2, π/2]
        return 2 * L + self.arc - R * (phi + math.pi / 2), math.hypot(dx, dy) - R

    def dist_to_centerline(self, x: float, y: float) -> float:
        """到中心线的绝对距离（栅格引导代价用）。"""
        _, lateral = self.frame(x, y)
        return abs(lateral)

    # ---------- 绘制用 ----------

    def points(self, n: int = 240, lateral: float = 0.0) -> list[tuple[float, float]]:
        """按弧长均匀采一圈，转成像素坐标。"""
        out = []
        for i in range(n):
            x, y = self.point_at(self.total_length * i / n, lateral)
            out.append((x / config.SCALE, y / config.SCALE))
        return out
