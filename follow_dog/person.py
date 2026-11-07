"""person.py —— 沿蜿蜒道路匀速跑步的人。

人用"弧长 s"定位：每帧 s += speed * dt，位置由 road.point_at(s) 给出。
这样人永远严格在路上跑，狗的目标点（人身后 50cm）也好算。
"""

import math

from . import config
from .road import Road


class Person:
    def __init__(self, road: Road, start_s: float = 0.0) -> None:
        self.road = road
        self.s = start_s
        self.speed = config.PERSON_SPEED
        self.x, self.y = road.point_at(self.s)
        self.dir_x, self.dir_y = road.direction_at(self.s)
        self.trail: list[tuple[float, float]] = [(self.x, self.y)]

    def update(self, dt: float) -> None:
        self.s = (self.s + self.speed * dt) % self.road.total_length
        self.x, self.y = self.road.point_at(self.s)
        self.dir_x, self.dir_y = self.road.direction_at(self.s)
        self.trail.append((self.x, self.y))
        if len(self.trail) > 600:
            self.trail = self.trail[-600:]

    def ahead_point(self, distance: float) -> tuple[float, float]:
        """人前方 distance 处的路点。

        规划器的终点要用"人前方"而不是"人身后 50cm"：
        如果路径一到跟随点就结束，狗稍微一冲就跑到路径外面，
        前视点落到身后 → 判定要掉头 → 原地打转（实测 |omega| 全程饱和）。
        让路径一直延伸到人前方，狗就永远有"前面的路"可看。
        """
        return self.road.point_at(self.s + distance)

    def follow_target(self) -> tuple[float, float]:
        """人身后 FOLLOW_DISTANCE(50cm) 的点 —— 狗要占的位置。

        注意：这里沿"人走过的轨迹"往回退 50cm，而不是沿当前朝向退 50cm。
        区别在弯道上非常关键：
        - 沿切线退：目标点会跑到弯道外侧/内侧，狗为了抄近路走直线弦，
          在环路里会越抄越靠内，stable 落后到 ~1 米（实测过）；
        - 沿轨迹退：目标点永远落在人实际走过的路上，狗跟着同一条曲线走，
          距离才能真正稳在 50cm。这也是真狗跟着人走的自然方式。
        """
        trail = self.trail
        acc = 0.0
        for i in range(len(trail) - 1, 0, -1):
            x1, y1 = trail[i]
            x0, y0 = trail[i - 1]
            seg = math.hypot(x1 - x0, y1 - y0)
            if seg <= 1e-9:
                continue
            if acc + seg >= config.FOLLOW_DISTANCE:
                t = (config.FOLLOW_DISTANCE - acc) / seg
                return x0 + (x1 - x0) * t, y0 + (y1 - y0) * t
            acc += seg
        # 轨迹还没积累够 50cm（刚启动）：退回沿切线估算
        return (self.x - self.dir_x * config.FOLLOW_DISTANCE,
                self.y - self.dir_y * config.FOLLOW_DISTANCE)
