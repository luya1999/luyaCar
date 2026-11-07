"""person.py —— 被跟随的跑者。

两种模式：
- auto   ：沿跑道自动跑，遇到障碍/行人自动侧移绕开，合速度恒 5 m/s
- manual ：WASD / 方向键控制方向，速度仍然恒 5 m/s（只改方向，不改快慢）

对外的两个关键接口（供机器狗使用）：
- follow_target()  ：沿"跑者真实走过的轨迹"往回退 FOLLOW_DIST(260cm)，
                     得到狗该占的位置（对应约 2m 身后间距）
- future_route()   ：按跑者当前走向预测未来一串路点（提前定义未来路线）
"""

import math

from . import config
from .track import Track

# 侧移余量：按对方实际半径算（路锥 18cm / 行人 25cm 不一样）
PERSON_AVOID_MARGIN = 15.0
PERSON_LOOKAHEAD = 500.0     # 提前多远开始侧移（cm）。v0.3 的 340cm 在
                             # 5m/s 下只有 0.68s，横穿行人来不及反应
LATERAL_LIMIT = 100.0        # 最多偏出中心线多少（跑道半宽 120cm）


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


class Person:
    def __init__(self, track: Track) -> None:
        self.track = track
        self.mode = "auto"
        self.reset()

    def reset(self) -> None:
        self.s = 0.0
        self.lateral = 0.0
        self.x, self.y = self.track.point_at(self.s)
        self.dx, self.dy = self.track.tangent_at(self.s)
        self.speed = 0.0                  # 这一帧的实际速率
        self.trail: list[tuple[float, float]] = [(self.x, self.y)]
        self.lap = 0
        self._prev_s = 0.0
        self.bump_count = 0               # 人蹭到障碍的次数（观察用）

    def toggle_mode(self) -> None:
        if self.mode == "auto":
            self.mode = "manual"
        else:
            # 手动 → 自动：先回到跑道上，避免瞬移到很远的地方
            self.s, self.lateral = self.track.frame(self.x, self.y)
            self.lateral = _clamp(self.lateral, -LATERAL_LIMIT, LATERAL_LIMIT)
            self.mode = "auto"

    # ---------- 自动避障 ----------

    def _desired_lateral(self, obstacles) -> float:
        """看前方一段路上有没有障碍，有就往它的反方向让。

        障碍是"半探入"跑道的（中心 lateral 约 ±38cm），人沿中心线会碰上，
        必须往另一侧偏；障碍左右交错摆放，人就自然跑成 S 形，不能走直线。
        """
        best_ds, best = PERSON_LOOKAHEAD, None
        for o in obstacles:
            so, lo = self.track.frame(o.x, o.y)
            ds = (so - self.s) % self.track.total_length
            if ds < best_ds and abs(lo) < 150.0:
                best_ds, best = ds, (lo, o.radius)
        if best is None:
            return 0.0
        lo, radius = best
        sign = 1.0 if lo >= 0 else -1.0
        # 需要的中心距 = 人半径 + 对方半径 + 余量；按实际半径算，
        # 不再对所有障碍统一写死 58cm
        need = max(50.0, config.PERSON_RADIUS + radius
                   + PERSON_AVOID_MARGIN - abs(lo))
        return _clamp(-sign * need, -LATERAL_LIMIT, LATERAL_LIMIT)

    # ---------- 步进 ----------

    def update(self, dt: float, obstacles, manual_dir=None) -> None:
        if self.mode == "auto":
            # "速度固定 5m/s" 指的是**合速率**恒定。绕障碍左右移动要花掉一部分
            # 速度，沿跑道前进的分量就变成 sqrt(500² - v_lat²)，人会明显变慢。
            # 这是"巡航不超人速仍能跟上"的物理前提：机器狗走平滑最短路，
            # 不横移，同样 5m/s 全部用于前进，才能补回绕障损失的距离。
            target_lat = self._desired_lateral(obstacles)
            v_lat = _clamp((target_lat - self.lateral) / dt,
                           -config.PERSON_LAT_RATE, config.PERSON_LAT_RATE)
            v_along = math.sqrt(max(0.0, config.PERSON_SPEED ** 2 - v_lat ** 2))
            self.speed = math.hypot(v_along, v_lat)
            self.s += v_along * dt
            if self.s - self._prev_s >= self.track.total_length:
                self.lap += 1
                self._prev_s = self.s
            self.lateral += v_lat * dt
            self.x, self.y = self.track.point_at(self.s, self.lateral)
            self.dx, self.dy = self.track.tangent_at(self.s)
        else:
            if manual_dir is None or (manual_dir[0] == 0 and manual_dir[1] == 0):
                self.speed = 0.0
            else:
                n = math.hypot(*manual_dir)
                self.dx, self.dy = manual_dir[0] / n, manual_dir[1] / n
                self.speed = config.PERSON_SPEED
                self.x += self.dx * self.speed * dt
                self.y += self.dy * self.speed * dt
                self.x = _clamp(self.x, 40.0, config.WORLD_W - 40.0)
                self.y = _clamp(self.y, 40.0, config.WORLD_H - 40.0)
            # 同步 (s, lateral)，供预测路线与 HUD 使用
            self.s, self.lateral = self.track.frame(self.x, self.y)

        self.trail.append((self.x, self.y))
        if len(self.trail) > 120:
            self.trail = self.trail[-120:]

    # ---------- 给机器狗用的接口 ----------

    def follow_target(self) -> tuple[float, float]:
        """沿跑者走过的轨迹往回退 FOLLOW_DIST，得到"身后约 2m"的目标点。

        用轨迹而不是"沿当前朝向退一段"，是为了弯道上也严格落在跑者
        实际走过的路上，不会切进内场。
        """
        trail = self.trail
        back = config.FOLLOW_DIST + config.FOLLOW_BIAS
        acc = 0.0
        for i in range(len(trail) - 1, 0, -1):
            x1, y1 = trail[i]
            x0, y0 = trail[i - 1]
            seg = math.hypot(x1 - x0, y1 - y0)
            if seg <= 1e-9:
                continue
            if acc + seg >= back:
                t = (back - acc) / seg
                return x0 + (x1 - x0) * t, y0 + (y1 - y0) * t
            acc += seg
        return (self.x - self.dx * back, self.y - self.dy * back)

    def future_route(self, steps: int = 8, spacing: float = 60.0
                     ) -> list[tuple[float, float]]:
        """预测人未来 steps×spacing 厘米的走向（规则3）。

        auto   ：沿跑道弧长往前取点（可以精确预知弯道）
        manual ：按当前速度方向线性外推（人此刻的方向就是最好的预测）
        """
        pts = []
        for i in range(1, steps + 1):
            d = i * spacing
            if self.mode == "auto":
                pts.append(self.track.point_at(self.s + d, self.lateral))
            else:
                pts.append((_clamp(self.x + self.dx * d, 20.0, config.WORLD_W - 20.0),
                            _clamp(self.y + self.dy * d, 20.0, config.WORLD_H - 20.0)))
        return pts

    def center_gap_to(self, x: float, y: float) -> float:
        """到某点中心的表面间距（扣除双方半径）。"""
        return math.hypot(x - self.x, y - self.y) - config.PERSON_RADIUS
