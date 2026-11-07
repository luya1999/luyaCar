"""cube.py —— 小木块：运动学 + 纯追踪 + 弧长跟随控制。

规则落实
--------
- 规则2：沿"人走过的轨迹"往回退 81cm 得到跟随点（≈ 表面间距 35cm），
  用**弧长误差**控速：恰好在位就与人同速，落后就加速（但见下条），超前就减速。
- 规则3：速度上限严格 = 人速（500cm/s）。追不上时靠 A* 抄近道（走弯道的弦）
  和预测未来路线来补距离，而不是超速。
- 规则5：路径层用膨胀保证中心距 ≥ 50cm + 跟踪余量；控制层再加近障排斥转向
  与近障限速，双保险。间隙真的掉到危险值时蠕行脱困（绝不冻结，冻结会死锁）。
"""

import math

from . import config
from .obstacles import Obstacle, min_clearance
from .person import Person


def wrap_angle(a: float) -> float:
    while a > math.pi:
        a -= 2 * math.pi
    while a <= -math.pi:
        a += 2 * math.pi
    return a


def polyline_cum(path: list[tuple[float, float]]) -> list[float]:
    cum = [0.0]
    for i in range(1, len(path)):
        cum.append(cum[-1] + math.hypot(path[i][0] - path[i - 1][0],
                                        path[i][1] - path[i - 1][1]))
    return cum


def project_on_path(path, cum, px: float, py: float) -> float:
    best_s, best_d = 0.0, float("inf")
    for i in range(len(path) - 1):
        ax, ay = path[i]
        bx, by = path[i + 1]
        dx, dy = bx - ax, by - ay
        L2 = dx * dx + dy * dy
        if L2 <= 1e-9:
            continue
        t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / L2))
        d = math.hypot(px - (ax + dx * t), py - (ay + dy * t))
        if d < best_d:
            best_d = d
            best_s = cum[i] + math.sqrt(L2) * t
    return best_s


def point_at_s(path, cum, s: float) -> tuple[float, float]:
    s = max(0.0, min(cum[-1], s))
    for i in range(len(path) - 1):
        if cum[i + 1] >= s:
            seg = cum[i + 1] - cum[i]
            t = 0.0 if seg <= 1e-9 else (s - cum[i]) / seg
            ax, ay = path[i]
            bx, by = path[i + 1]
            return ax + (bx - ax) * t, ay + (by - ay) * t
    return path[-1]


class Cube:
    def __init__(self, x: float, y: float, theta: float = 0.0) -> None:
        self.x, self.y, self.theta = x, y, theta
        self.v = 0.0
        self.omega = 0.0
        self.path: list[tuple[float, float]] = []
        self.trail: list[tuple[float, float]] = [(x, y)]
        self.danger = False              # 间隙跌破硬下限（HUD 报警）
        self.repelling = False           # 正在被障碍排斥

    # ---------- 前视 ----------

    def _lookahead(self) -> float:
        return max(config.LOOKAHEAD_MIN,
                   min(config.LOOKAHEAD_MAX, abs(self.v) * config.LOOKAHEAD_RATIO))

    # ---------- 主步进 ----------

    def step(self, person: Person, obstacles: list[Obstacle], dt: float,
             path_age: float = 0.0) -> None:
        target = person.follow_target()
        clearance = min_clearance(self.x, self.y, obstacles)
        self.danger = clearance < config.SAFE_CLEARANCE

        # 1) 在规划路径上定位自己与跟随点（弧长坐标）
        if len(self.path) >= 2:
            cum = polyline_cum(self.path)
            s_dog = project_on_path(self.path, cum, self.x, self.y)
            s_goal = project_on_path(self.path, cum, target[0], target[1])
            # 跟随点本身以人速在移动，而路径每 0.18s 才刷新一次。
            # 不做补偿的话，木块会在两次重规划之间"追上一个静止的旧目标"然后
            # 周期性减速，平均速度掉到 3.5m/s → 被越甩越远。
            s_goal += config.TARGET_LEAD * person.speed * path_age
            look = point_at_s(self.path, cum, s_dog + self._lookahead())
            s_err = s_goal - s_dog
        else:
            # 规划失败的退化路径：直接朝跟随点
            s_err = math.hypot(target[0] - self.x, target[1] - self.y)
            look = target

        # 2) 障碍排斥：把前视点往"远离障碍"的方向推，近了推得更狠。
        #    这是动态障碍（会被拖着走）下的最后一道主动避让。
        self.repelling = False
        rx = ry = 0.0
        for o in obstacles:
            dx, dy = self.x - o.x, self.y - o.y
            d = math.hypot(dx, dy)
            gap = d - o.radius - config.CUBE_RADIUS
            if gap < config.REPEL_RANGE and d > 1e-6:
                w = (config.REPEL_RANGE - gap) / config.REPEL_RANGE
                rx += dx / d * w * w
                ry += dy / d * w * w
                self.repelling = True
        if self.repelling:
            push = config.DODGE_WEIGHT * self._lookahead()
            look = (look[0] + rx * push, look[1] + ry * push)

        # 3) 纯追踪转向
        desired = math.atan2(look[1] - self.y, look[0] - self.x)
        err = wrap_angle(desired - self.theta)
        L = self._lookahead()
        if abs(self.v) > 10.0:
            omega = config.TURN_GAIN * abs(self.v) * math.sin(err) / L
        else:
            omega = 2.5 * err
        self.omega = max(-config.CUBE_MAX_OMEGA,
                         min(config.CUBE_MAX_OMEGA, omega))

        # 4) 速度：人速前馈 + 弧长误差反馈，**硬上限 = 人速**（规则3）
        v_cap = min(config.CUBE_MAX_SPEED, person.speed)
        if person.speed <= 1e-6:
            target_v = max(0.0, min(v_cap, config.KP_SPEED * s_err))
        else:
            target_v = person.speed + config.KP_SPEED * s_err
            target_v = max(0.0, min(v_cap, target_v))
            if abs(err) > 1.0:                 # 大角度转向先减速
                target_v *= max(0.0, math.cos(err))

        # 5) 近障限速：间隙越小越慢，给转向与刹车留出余量（规则5）
        #    ⚠️ 不能降太狠：木块速度上限就等于人速，一旦被拖到 150cm/s，
        #       落后 100cm 就再也追不回来（同速追不上同速）。最低保 60% 人速。
        if clearance < 85.0:
            ratio = max(0.60, min(1.0, (clearance - 45.0) / 40.0 + 0.6))
            target_v = min(target_v, v_cap * ratio)
        if clearance < config.SAFE_CLEARANCE * 0.45:
            # 极近时蠕行脱困：不能 v=0 冻结，否则位置和间隙都不再变化 → 死锁
            target_v = config.CUBE_REVERSE_SPEED * 0.4

        # 5.5) 不许贴人太近：路径投影的 s_err 和真实间距偶尔会脱节，
        #      表面间距掉到 30cm 以下就主动收油，守住规则2 的下界。
        gap_now = self.surface_gap(person)
        if gap_now < config.FOLLOW_GAP_MIN - 5.0:
            target_v *= max(0.0, min(1.0, (gap_now - 5.0) / 20.0))

        # 6) 加速度限制 + 运动学积分
        dv = target_v - self.v
        max_dv = config.CUBE_ACCEL * dt
        self.v += max(-max_dv, min(max_dv, dv))
        self.theta = wrap_angle(self.theta + self.omega * dt)
        self.x += self.v * math.cos(self.theta) * dt
        self.y += self.v * math.sin(self.theta) * dt
        self.x = max(15.0, min(config.WORLD_W - 15.0, self.x))
        self.y = max(15.0, min(config.WORLD_H - 15.0, self.y))

        # 7) 安全过滤层（Control Barrier Function 思路）：
        #    规划层留了余量，纯追踪贴弯仍可能啃掉几 cm → 用软侧推补。
        #    如果用户把障碍硬怼到木块身上（障碍与人同速，木块逃不掉），
        #    软推来不及，就做一次"投影回安全集"的硬修正，保证永不穿模。
        #    迭代两轮是为了处理同时被两个障碍夹住的情况。
        if obstacles:
            limit = config.SAFE_CLEARANCE + config.SAFETY_FILTER_BAND
            floor = config.SAFE_CLEARANCE + 2.0     # 留 2cm 数值余量
            for _ in range(3):
                moved = False
                for o in obstacles:
                    dx, dy = self.x - o.x, self.y - o.y
                    d = math.hypot(dx, dy)
                    if d < 1e-6:
                        dx, dy, d = 1.0, 0.0, 1.0
                    gap = d - o.radius - config.CUBE_RADIUS
                    if gap >= limit:
                        continue
                    push = ((limit - gap) * config.SAFETY_FILTER_GAIN
                            if gap >= floor else floor - gap)
                    self.x += dx / d * push
                    self.y += dy / d * push
                    moved = True
                if not moved:
                    break
            self.x = max(15.0, min(config.WORLD_W - 15.0, self.x))
            self.y = max(15.0, min(config.WORLD_H - 15.0, self.y))

        self.trail.append((self.x, self.y))
        if len(self.trail) > 240:
            self.trail = self.trail[-240:]

    # ---------- 观测 ----------

    def surface_gap(self, person: Person) -> float:
        """到人的表面间距（规则2 的 30~40cm 就是这个量）。"""
        return (math.hypot(person.x - self.x, person.y - self.y)
                - config.PERSON_RADIUS - config.CUBE_RADIUS)
