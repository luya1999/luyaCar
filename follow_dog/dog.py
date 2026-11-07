"""dog.py —— 机器狗：差速运动学 + 纯追踪（Pure Pursuit）控制器。

运动学模型和轮式小车一样（四足狗在全向移动前，也可以先当成
"只能前进 + 转弯"的底盘来控制）：
    x' = v*cos(theta),  y' = v*sin(theta),  theta' = omega

控制策略：
- 沿 A* 规划好的路径走"纯追踪"：在路径前方 LOOKAHEAD 距离处
  取一个前视点，朝它打方向；
- 速度由"离跟随目标还有多远"决定，转向没对准时自动减速；
- 进入人身后 (50 - HOLD_TOLERANCE)cm 内 → 停车等待；
- 任何时候与障碍间隙 < EMERGENCY_CLEARANCE → 紧急停车（保底安全层）。
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


# ---------- 折线几何工具（弧长参数化） ----------

def polyline_cum(path: list[tuple[float, float]]) -> list[float]:
    """折线各顶点的累计弧长，cum[i] 是走到第 i 个点的总长度。"""
    cum = [0.0]
    for i in range(1, len(path)):
        cum.append(cum[-1] + math.hypot(path[i][0] - path[i - 1][0],
                                        path[i][1] - path[i - 1][1]))
    return cum


def project_on_path(path: list[tuple[float, float]], cum: list[float],
                    px: float, py: float) -> float:
    """把一个点投影到折线上，返回它对应的弧长 s（取最近的一段）。"""
    best_s, best_d = 0.0, float("inf")
    for i in range(len(path) - 1):
        ax, ay = path[i]
        bx, by = path[i + 1]
        dx, dy = bx - ax, by - ay
        L2 = dx * dx + dy * dy
        if L2 <= 1e-9:
            continue
        t = ((px - ax) * dx + (py - ay) * dy) / L2
        t = max(0.0, min(1.0, t))
        qx, qy = ax + dx * t, ay + dy * t
        d = math.hypot(px - qx, py - qy)
        if d < best_d:
            best_d = d
            best_s = cum[i] + math.sqrt(L2) * t
    return best_s


def point_at_s(path: list[tuple[float, float]], cum: list[float],
               s: float) -> tuple[float, float]:
    """取折线上弧长 s 处的坐标（超出两端就取端点）。"""
    s = max(0.0, min(cum[-1], s))
    for i in range(len(path) - 1):
        if cum[i + 1] >= s:
            seg = cum[i + 1] - cum[i]
            t = 0.0 if seg <= 1e-9 else (s - cum[i]) / seg
            ax, ay = path[i]
            bx, by = path[i + 1]
            return ax + (bx - ax) * t, ay + (by - ay) * t
    return path[-1]


class Dog:
    def __init__(self, x: float, y: float, theta: float = 0.0) -> None:
        self.x, self.y, self.theta = x, y, theta
        self.v = 0.0
        self.omega = 0.0
        self.path: list[tuple[float, float]] = []   # 当前规划路径
        self.trail: list[tuple[float, float]] = [(x, y)]
        self.emergency_stopped = False              # 供界面显示"触发过急停"

    # ---------- 纯追踪 ----------

    def _lookahead(self) -> float:
        """随速度自适应的前视距离（详见 config 注释）。"""
        return max(config.LOOKAHEAD_MIN,
                   min(config.LOOKAHEAD_MAX, abs(self.v) * config.LOOKAHEAD_RATIO))

    def _lookahead_point(self, target: tuple[float, float]) -> tuple[float, float]:
        """在路径上找距狗一个前视距离的点；路径尽头就是跟随目标。"""
        if not self.path:
            return target
        lookahead = self._lookahead()
        # 从离狗最近的路径点开始，沿路径往前累计 lookahead 距离
        dists = [math.hypot(px - self.x, py - self.y) for px, py in self.path]
        start = min(range(len(dists)), key=dists.__getitem__)
        acc = 0.0
        for i in range(start, len(self.path) - 1):
            x0, y0 = self.path[i]
            x1, y1 = self.path[i + 1]
            seg = math.hypot(x1 - x0, y1 - y0)
            if acc + seg >= lookahead:
                ratio = (lookahead - acc) / seg if seg else 0.0
                return x0 + (x1 - x0) * ratio, y0 + (y1 - y0) * ratio
            acc += seg
        return self.path[-1]

    # ---------- 主步进 ----------

    def step(self, person: Person, obstacles: list[Obstacle], dt: float) -> None:
        target = person.follow_target()

        # 1) 安全层：间隙过小 → 进入蠕行脱困模式。
        #    ⚠️ 这里千万不能"直接 v=0 冻结"：狗一旦停住，位置不再变化、
        #    间隙也不会变，会永久卡死（实测会冻结一整圈），必须保留蠕行。
        clearance = min_clearance(self.x, self.y, obstacles)
        self.emergency_stopped = clearance < config.EMERGENCY_CLEARANCE

        # 2) 在规划路径上定位自己：s_dog（当前进度）、s_goal（跟随点进度）
        #    —— 关键：路径终点在"人前方"，所以 s_dog 永远有前方的路可看，
        #       狗不会冲过终点后再掉头（那是之前原地打转的根因）。
        if len(self.path) >= 2:
            cum = polyline_cum(self.path)
            s_dog = project_on_path(self.path, cum, self.x, self.y)
            s_goal = project_on_path(self.path, cum, target[0], target[1])
            look = point_at_s(self.path, cum, s_dog + self._lookahead())
        else:
            # 没有路径（规划失败）→ 退化：直接朝跟随点开
            s_dog = 0.0
            s_goal = math.hypot(target[0] - self.x, target[1] - self.y)
            look = target

        # 3) 纯追踪转向：omega = GAIN * v * sin(err) / LOOKAHEAD
        #    几何上天然稳定（误差越大转弯越急，接近目标自动回正），
        #    不会像 P 控制那样高速来回画龙。低速时用 P 控制原地对准。
        desired = math.atan2(look[1] - self.y, look[0] - self.x)
        err = wrap_angle(desired - self.theta)
        if abs(self.v) > 10.0:
            omega = config.TURN_GAIN * abs(self.v) * math.sin(err) / self._lookahead()
        else:
            omega = 2.0 * err
        self.omega = max(-config.DOG_MAX_OMEGA,
                         min(config.DOG_MAX_OMEGA, omega))

        # 4) 速度 = 人速前馈 + 弧长误差反馈
        #    s_err = 跟随点在自己前方还有多远（沿路径量的"掉队距离"）。
        #    用弧长而不是直线距离，弯道上才不会把"横向偏移"误当成"掉队"。
        s_err = s_goal - s_dog
        if self.emergency_stopped:
            target_v = config.SLOWDOWN_MIN_SPEED * 0.5  # 极慢蠕行，等规划带它绕出去
        else:
            target_v = person.speed + config.KP_SPEED * s_err
            target_v = max(0.0, min(config.DOG_MAX_SPEED, target_v))
            # 只有大角度转向（>~57°）才减速；小角度修正是跟随弯路的常态，
            # 全程乘 cos(err) 会让狗永远比人慢一截（实测会稳定落后 ~60cm）
            if abs(err) > 1.0:
                target_v *= max(0.0, math.cos(err))

            # 障碍近旁限速：纯追踪高速过弯会往弯心"切"，规划留的余量不够它切
            # → 间隙越小速度越低，宁可暂时落后、不可蹭到障碍。
            if clearance < config.SLOWDOWN_CLEARANCE:
                v_cap = max(config.SLOWDOWN_MIN_SPEED,
                            config.SLOWDOWN_GAIN * (clearance - 5.0))
                target_v = min(target_v, v_cap)

        # 4) 加速度限制
        dv = target_v - self.v
        max_dv = config.DOG_ACCEL * dt
        self.v += max(-max_dv, min(max_dv, dv))

        # 5) 运动学积分
        self.theta = wrap_angle(self.theta + self.omega * dt)
        self.x += self.v * math.cos(self.theta) * dt
        self.y += self.v * math.sin(self.theta) * dt

        self.trail.append((self.x, self.y))
        if len(self.trail) > 600:
            self.trail = self.trail[-600:]

    def distance_to(self, person: Person) -> float:
        return math.hypot(person.x - self.x, person.y - self.y)
