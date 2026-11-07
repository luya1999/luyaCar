"""road.py —— 蜿蜒环形道路。

用参数方程生成一条闭合的"波浪形环路"：
    x(t) = cx + a*cos(t)
    y(t) = cy + b*sin(t) + wiggle*sin(3t)
第三个叠加项让路弯弯曲曲，而不是一个光滑椭圆。

道路以"弧长索引"的方式提供：按弧长 s 取点和切线方向，
人沿着它跑、狗算"人身后 50cm 的点"都靠这个接口。
"""

import math

from . import config

CORRIDOR_HALF_WIDTH = 55.0   # 路的走廊半宽（1.1m 宽的小路，够狗绕障又不用贴边）


def _curve(t: float) -> tuple[float, float]:
    cx, cy = config.WINDOW_W / 2, config.WINDOW_H / 2
    x = cx + 430 * math.cos(t)
    y = cy + 255 * math.sin(t) + 65 * math.sin(3 * t)
    return x, y


class Road:
    """把参数曲线离散成密集点列，支持按弧长查询。"""

    def __init__(self, samples: int = 2000) -> None:
        # 1) 均匀采样参数 t，得到折线点列
        self.points: list[tuple[float, float]] = [
            _curve(2 * math.pi * i / samples) for i in range(samples)
        ]
        # 2) 预计算累计弧长，之后可以 O(1) 级别按弧长定位
        self.cum_len: list[float] = [0.0]
        for i in range(1, len(self.points) + 1):
            x0, y0 = self.points[i - 1]
            x1, y1 = self.points[i % len(self.points)]
            self.cum_len.append(self.cum_len[-1] + math.hypot(x1 - x0, y1 - y0))
        self.total_length = self.cum_len[-1]

    def point_at(self, s: float) -> tuple[float, float]:
        """弧长 s 处的坐标（s 自动对总长取模，可以无限往前走）。"""
        s = s % self.total_length
        lo, hi = 0, len(self.cum_len) - 1
        while hi - lo > 1:  # 二分查找 s 落在哪一段
            mid = (lo + hi) // 2
            if self.cum_len[mid] <= s:
                lo = mid
            else:
                hi = mid
        seg = self.cum_len[hi] - self.cum_len[lo]
        ratio = 0.0 if seg == 0 else (s - self.cum_len[lo]) / seg
        x0, y0 = self.points[lo % len(self.points)]
        x1, y1 = self.points[hi % len(self.points)]
        return x0 + (x1 - x0) * ratio, y0 + (y1 - y0) * ratio

    def direction_at(self, s: float) -> tuple[float, float]:
        """弧长 s 处的单位切向量（人/狗的朝向参考）。"""
        eps = 2.0
        x0, y0 = self.point_at(s - eps)
        x1, y1 = self.point_at(s + eps)
        d = math.hypot(x1 - x0, y1 - y0)
        if d == 0:
            return 1.0, 0.0
        return (x1 - x0) / d, (y1 - y0) / d

    def nearest_s(self, x: float, y: float) -> float:
        """找道路上离 (x, y) 最近的点的弧长（障碍物贴路摆放时用）。"""
        best_i, best_d = 0, float("inf")
        step = 5  # 粗采样即可，只用于初始化摆障碍
        for i in range(0, len(self.points), step):
            px, py = self.points[i]
            d = (px - x) ** 2 + (py - y) ** 2
            if d < best_d:
                best_d, best_i = d, i
        return self.cum_len[best_i]
