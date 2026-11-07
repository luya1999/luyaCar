"""obstacles.py —— 静态圆形障碍物。

障碍物贴着路边摆放（横向偏移路中心线 25~40px），这样：
- 人沿中心线跑，刚好从旁边挤过去；
- 狗如果只会"直线跟在人屁股后面"，就一定会撞上；
- 所以狗必须用 A* 规划一条绕开障碍的路 —— 这正是本仿真要演示的核心。
"""

import math

from . import config
from .road import Road


class Obstacle:
    def __init__(self, x: float, y: float, radius: float) -> None:
        self.x = x
        self.y = y
        self.radius = radius

    def clearance(self, px: float, py: float, body_radius: float = 0.0) -> float:
        """点 (px, py) 处的物体（半径 body_radius）到本障碍的间隙。

        > 0：安全；= 0：刚好贴上；< 0：已经撞进去了。
        """
        return math.hypot(px - self.x, py - self.y) - self.radius - body_radius


def build_obstacles(road: Road) -> list[Obstacle]:
    """沿道路均匀摆 6 个障碍，左右交错，留出可通过的缝隙。"""
    obstacles = []
    fractions = [0.10, 0.26, 0.42, 0.58, 0.74, 0.90]   # 在全程的百分比位置
    # 障碍"半探"进路里：人沿中心线刚好不蹭到（间隙 >= 10px），
    # 但狗走直线必撞，必须绕行；同时给狗留出可规划通过的通道
    offsets = [42.0, -45.0, 40.0, -42.0, 48.0, -40.0]  # 横向偏移（正=路左侧）
    radii = [22.0, 24.0, 20.0, 22.0, 24.0, 20.0]

    for frac, offset, radius in zip(fractions, offsets, radii):
        s = frac * road.total_length
        cx, cy = road.point_at(s)
        dx, dy = road.direction_at(s)
        # 法向量 = 切向量旋转 90°
        nx, ny = -dy, dx
        obstacles.append(Obstacle(cx + nx * offset, cy + ny * offset, radius))
    return obstacles


def min_clearance(x: float, y: float, obstacles: list[Obstacle],
                  body_radius: float = config.DOG_RADIUS) -> float:
    """到所有障碍的最小间隙（监控安全性用）。"""
    if not obstacles:  # 没有障碍时视为无穷大间隙
        return float("inf")
    return min(obs.clearance(x, y, body_radius) for obs in obstacles)
