"""obstacles.py —— 障碍物（规则7：与人一样大、移动速度与人一致）。

- 半径 25cm（宽 50cm，高 1.7m），和人的碰撞模型完全一致
- 默认"半探入"跑道、左右交错摆放：人沿中心线跑会被逼着左右绕，走不成直线（规则4）
- 移动方式：
    1. 鼠标按住拖动（move_toward，限速 5 m/s，体现"速度与人一致"）
    2. 数字键 1~6 选中后用 IJKL 移动
    3. P 键让所有障碍沿跑道反向自动巡逻（同样是 5 m/s），用来演示动态避障
"""

import math

from . import config
from .track import Track


class Obstacle:
    def __init__(self, x: float, y: float, s: float = 0.0,
                 lateral: float = 0.0) -> None:
        self.x, self.y = x, y
        self.s = s
        self.lateral = lateral
        self.radius = config.OBSTACLE_RADIUS
        self.vx = self.vy = 0.0
        self.selected = False

    def clearance(self, x: float, y: float, body_radius: float) -> float:
        """物体（半径 body_radius）与本障碍的**表面间距**。"""
        return math.hypot(x - self.x, y - self.y) - self.radius - body_radius

    def move_toward(self, tx: float, ty: float, dt: float) -> None:
        """朝目标点移动，但速度上限 = 人速（5m/s），所以拖鼠标也不会瞬移。"""
        dx, dy = tx - self.x, ty - self.y
        d = math.hypot(dx, dy)
        if d < 1e-6 or dt <= 0:
            self.vx = self.vy = 0.0
            return
        v = min(config.OBSTACLE_MAX_SPEED, d / dt)
        self.vx, self.vy = dx / d * v, dy / d * v
        self.x += self.vx * dt
        self.y += self.vy * dt
        self.x = max(self.radius, min(config.WORLD_W - self.radius, self.x))
        self.y = max(self.radius, min(config.WORLD_H - self.radius, self.y))

    def move_by_input(self, dx: float, dy: float, dt: float) -> None:
        """IJKL 控制：方向归一化，速度固定 5 m/s。"""
        n = math.hypot(dx, dy)
        if n < 1e-6:
            self.vx = self.vy = 0.0
            return
        self.vx, self.vy = dx / n * config.OBSTACLE_MAX_SPEED, dy / n * config.OBSTACLE_MAX_SPEED
        self.x = max(self.radius, min(config.WORLD_W - self.radius,
                                      self.x + self.vx * dt))
        self.y = max(self.radius, min(config.WORLD_H - self.radius,
                                      self.y + self.vy * dt))

    def update_patrol(self, track: Track, dt: float) -> None:
        """沿跑道反向巡逻（速度与人一致），用于动态避障演示。"""
        self.s = (self.s - config.OBSTACLE_MAX_SPEED * dt) % track.total_length
        self.x, self.y = track.point_at(self.s, self.lateral)
        tx, ty = track.tangent_at(self.s)
        self.vx, self.vy = -tx * config.OBSTACLE_MAX_SPEED, -ty * config.OBSTACLE_MAX_SPEED

    def sync_track_frame(self, track: Track) -> None:
        """被拖动之后重新对齐跑道坐标，巡逻时不会突然跳位。"""
        self.s, self.lateral = track.frame(self.x, self.y)


def build_obstacles(track: Track) -> list[Obstacle]:
    """沿跑道摆 6 个障碍，全部从外侧半探入跑道。"""
    # 弧长百分比（不均匀，人侧移的节奏不会是机械的等间隔）
    fractions = [0.07, 0.20, 0.34, 0.51, 0.69, 0.86]
    # 横向偏移（cm）：正=跑道外侧。障碍集中在同一侧时，存在一条内侧快车道
    # （约 lateral -55cm），木块可以一路平滑走过去：既满足离障碍 >=50cm，
    # 又不用每圈横穿跑道 6 次把速度全耗在打方向上 —— 这正是规则3 的"最近路线"。
    # 人则每个障碍都要往内侧让一次再回中，跑成波浪线（规则4）。
    laterals = [44.0, 50.0, 42.0, 48.0, 45.0, 52.0]
    out = []
    for frac, lat in zip(fractions, laterals):
        s = frac * track.total_length
        x, y = track.point_at(s, lat)
        out.append(Obstacle(x, y, s, lat))
    return out


def min_clearance(x: float, y: float, obstacles: list[Obstacle],
                  body_radius: float = config.CUBE_RADIUS) -> float:
    """木块到最近障碍的表面间距（规则5 的监控量）。"""
    if not obstacles:
        return float("inf")
    return min(o.clearance(x, y, body_radius) for o in obstacles)
