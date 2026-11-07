"""dog.py —— 四足机器狗：运动学 + 纯追踪 + 弧长跟随 + 安全速度层。

v0.4 相对 v0.3 的关键修正（详见《优化说明_v0.4.md》）
----------------------------------------------------
1. 【重大 bug】v0.3 用 `self.x += push` 把狗"投影回安全集"，等于每帧
   凭空侧移几十厘米、无需发力也不消耗速度 —— 四足机器人做不到，会摔。
   本版改为**限制本帧位移**：把会侵入安全圈的速度分量投影到障碍切向
   （沿边缘滑过），并同步降低实际速度。物理上等价于"电机执行不了就减速
   贴边绕行"，不瞬移、不产生额外位移。
2. 【重大缺失】把被跟随的跑者本身纳入碰撞保护。v0.3 只把"障碍"当碰撞体，
   人突然停下/转身时狗会追尾。本版加人速前馈 + 紧急制动。
3. 【物理修正】最大偏航角速度随速度衰减：5m/s 跑步时不可能 6rad/s 急转。
4. 【物理修正】加速度与制动分离，制动按 DOG_DECEL（12m/s²）执行。
5. 【安全性】规划失败/路径不可行时减速停车，而不是抱着陈旧路径继续冲。
6. 【可恢复性】稳态巡航严格不超人速；只有掉队时才短暂使用 15% 速度余量
   恢复队形（可设 DOG_RECOVERY_MARGIN=0 回到原规则）。

保留的成熟做法：沿人走过轨迹回退取跟随点、弧长误差控速、A* 抄近道、
预测未来路线、纯追踪转向、近障排斥转向。
"""

import math

from . import config
from .obstacles import Obstacle, capsule_min_clearance, min_clearance
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


class RobotDog:
    """四足机器狗。位置 (x,y)、朝向 theta、线速度 v、偏航角速度 omega。"""

    def __init__(self, x: float, y: float, theta: float = 0.0) -> None:
        self.x, self.y, self.theta = x, y, theta
        self.v = 0.0
        self.omega = 0.0
        self.path: list[tuple[float, float]] = []
        self.trail: list[tuple[float, float]] = [(x, y)]
        self.danger = False        # 精确胶囊间隙跌破硬下限
        self.repelling = False     # 正在被障碍排斥转向
        self.braking = False       # 安全层正在限速 / 紧急制动
        self.plan_ok = True        # 本次重规划是否成功
        self.brake_reason = ""     # HUD 显示当前限速原因
        self.safety_clipped = 0.0  # 本帧位移被安全层削减的比例（诊断用）

    # ---------- 前视 ----------

    def _lookahead(self) -> float:
        return max(config.LOOKAHEAD_MIN,
                   min(config.LOOKAHEAD_MAX, abs(self.v) * config.LOOKAHEAD_RATIO))

    def _omega_limit(self) -> float:
        """高速时转向能力下降：5m/s 跑步不可能原地急打方向。"""
        k = min(1.0, abs(self.v) / config.DOG_CRUISE_SPEED)
        return (config.DOG_MAX_OMEGA_LOW
                - (config.DOG_MAX_OMEGA_LOW - config.DOG_MAX_OMEGA_HIGH) * k)

    # ---------- 主步进 ----------

    def step(self, person: Person, obstacles: list[Obstacle], dt: float,
             path_age: float = 0.0) -> None:
        target = person.follow_target()
        clearance = min_clearance(self.x, self.y, obstacles)
        exact_clearance = capsule_min_clearance(self.x, self.y, self.theta,
                                                obstacles)
        self.danger = exact_clearance < config.SAFE_CLEARANCE
        self.braking = False
        self.brake_reason = ""

        # 1) 在规划路径上定位自己与跟随点（弧长坐标）
        if len(self.path) >= 2:
            cum = polyline_cum(self.path)
            s_dog = project_on_path(self.path, cum, self.x, self.y)
            s_goal = project_on_path(self.path, cum, target[0], target[1])
            # 跟随点以人速移动，而路径每 0.15s 才刷新：不做补偿会周期性
            # "追上静止旧目标 → 减速"，平均速度掉到 3.5m/s 被越甩越远。
            s_goal += config.TARGET_LEAD * person.speed * path_age
            look = point_at_s(self.path, cum, s_dog + self._lookahead())
            s_err = s_goal - s_dog
        else:
            s_err = math.hypot(target[0] - self.x, target[1] - self.y)
            look = target

        # 1.5) 【bug 修复】弧长投影在路径折返/自交时会给出错误的符号：
        #      狗明明掉队十几米，投影却可能算成"已经超前"，于是
        #      target_v = 人速 + KP×负数 被钳到 0，狗原地冻结（实测停 2 秒）。
        #      用「到跟随点的真实直线距离」兜底：只要几何上明显掉队，
        #      误差就不允许为负。FOLLOW_GAP_MAX 是允许的跟随区间上界。
        d_target = math.hypot(target[0] - self.x, target[1] - self.y)
        s_err = max(s_err, d_target - config.FOLLOW_GAP_MAX)

        # 2) 障碍排斥：把前视点往远离障碍方向推（被动态行人逼近时主动让）
        self.repelling = False
        rx = ry = 0.0
        for o in obstacles:
            dx, dy = self.x - o.x, self.y - o.y
            d = math.hypot(dx, dy)
            gap = d - o.radius - config.DOG_PLAN_RADIUS
            if gap < config.REPEL_RANGE and d > 1e-6:
                w = (config.REPEL_RANGE - gap) / config.REPEL_RANGE
                rx += dx / d * w * w
                ry += dy / d * w * w
                self.repelling = True
        if self.repelling:
            push = config.DODGE_WEIGHT * self._lookahead()
            lx, ly = look[0] + rx * push, look[1] + ry * push
            # 【bug 修复】多个障碍的排斥力叠加时，前视点可能被推到狗身后，
            # 导致朝向误差接近 ±180°、大角度减速把速度乘成 0 → 永久死锁。
            # 把排斥造成的偏角限制在 ±50°，机器人不该突然"掉头找目标"。
            base_ang = math.atan2(look[1] - self.y, look[0] - self.x)
            new_ang = math.atan2(ly - self.y, lx - self.x)
            delta = wrap_angle(new_ang - base_ang)
            cap = math.radians(50.0)
            delta = max(-cap, min(cap, delta))
            r = math.hypot(lx - self.x, ly - self.y)
            a = base_ang + delta
            look = (self.x + math.cos(a) * r, self.y + math.sin(a) * r)

        # 2.5) 【bug 修复】严重掉队时，A* 的绕行路径可能把前视点指向侧后方，
        #      狗照着走会越跑越偏。掉队越远越直接朝跟随点回收方向，
        #      避障仍由规划膨胀 + 位移投影层兜底，不会硬闯障碍。
        if d_target > config.FOLLOW_GAP_MAX * 1.6:
            w = min(0.80, (d_target - config.FOLLOW_GAP_MAX * 1.6) / 400.0)
            gd = math.atan2(target[1] - self.y, target[0] - self.x)
            Lg = self._lookahead()
            gx = self.x + math.cos(gd) * Lg
            gy = self.y + math.sin(gd) * Lg
            look = (look[0] * (1 - w) + gx * w,
                    look[1] * (1 - w) + gy * w)

        # 3) 纯追踪转向。⚠️ 误差接近 ±180° 时 sin(err)→0，纯追踪几乎不转，
        #    是经典奇异点：狗会背对目标慢速打转。这里强制给一个掉头角速度。
        desired = math.atan2(look[1] - self.y, look[0] - self.x)
        err = wrap_angle(desired - self.theta)
        L = self._lookahead()
        lim = self._omega_limit()
        if abs(err) > 2.0:
            omega = math.copysign(lim * 0.85, err)
        elif abs(self.v) > 10.0:
            omega = config.TURN_GAIN * abs(self.v) * math.sin(err) / L
        else:
            omega = 2.5 * err
        self.omega = max(-lim, min(lim, omega))

        # 4) 速度指令：人速前馈 + 弧长误差反馈
        #    稳态严格不超人速；只有明显掉队才允许 +15% 余量恢复队形。
        #    ⚠️ v0.3 把上限钉死在人速，导致任何一次掉队永久不可恢复 —— 因为
        #       同速追同速距离不会缩短。真实机器人必须有速度储备。
        recovering = s_err > 40.0
        if recovering:
            v_cap = min(config.DOG_MAX_SPEED,
                        person.speed + config.DOG_RECOVERY_MARGIN * config.PERSON_SPEED)
        else:
            v_cap = min(config.DOG_CRUISE_SPEED, person.speed)

        if person.speed <= 1e-6:
            target_v = 0.0                      # 人停 → 狗停（见第 6 步）
        else:
            target_v = person.speed + config.KP_SPEED * s_err
            target_v = max(0.0, min(v_cap, target_v))
            if abs(err) > 1.0:                 # 大角度转向先减速
                # ⚠️ 不能用 max(0, cos(err))：误差 >90° 时速度直接归零，
                #    狗既不前进也无法把朝向转回来，会永久冻结（实测停 2 秒、
                #    掉队 13m）。保留 30% 最低速度让它能边走边转脱困。
                target_v *= max(0.30, math.cos(err))

        # 5) 安全层 A：短时预测减速（Control Barrier Function 思路）
        #    预测 0.35s 后"狗的位置"与"障碍/行人的位置"之间的间隙，
        #    若会跌破安全距就提前收油。
        #    ⚠️ 只取**最紧的一条**约束：v0.4 初版对每个障碍连乘 scale，
        #       绕行经过 3~4 个对象时速度被压到 1.8m/s、掉队近 6m。
        h = 0.35
        fx = self.x + target_v * math.cos(self.theta) * h
        fy = self.y + target_v * math.sin(self.theta) * h
        tightest = 1.0
        for o in obstacles:
            ox, oy = o.x + o.vx * h, o.y + o.vy * h
            d = math.hypot(fx - ox, fy - oy)
            gap_future = d - o.radius - config.DOG_PLAN_RADIUS
            if gap_future < config.SAFE_CLEARANCE:
                tightest = min(tightest,
                               max(0.35, gap_future / config.SAFE_CLEARANCE))
        if tightest < 1.0:
            target_v *= tightest
            self.braking = True
            self.brake_reason = "预测到接近，提前减速"

        # 6) 【新增】被跟随者的碰撞保护：绝不撞主人
        #    v0.3 把人排除在碰撞集合外，只靠"间距 < 25cm 收油"，
        #    在 5m/s 下 25cm 只够 0.05 秒，完全刹不住。
        gap_person = self.surface_gap(person)
        if person.speed < config.PERSON_SPEED * 0.5:
            # 跟车制动：随剩余间距线性收油，目标是在 FOLLOW_GAP_MIN 处停住，
            # 而不是等贴到 FOLLOW_HARD_MIN 才急刹（5m/s 制动距离约 1m）。
            target_v = min(target_v,
                           max(0.0, config.KP_SPEED
                               * (gap_person - config.FOLLOW_GAP_MIN)))
            self.braking = True
            self.brake_reason = "跑者减速/停下，跟车制动"
        if gap_person < config.FOLLOW_HARD_MIN:
            target_v = 0.0
            self.brake_reason = "距主人过近，紧急制动"
        if gap_person < config.FOLLOW_GAP_MIN:
            self.braking = True

        # 7) 规划不可行 → 停车等待，不抱着陈旧路径继续冲
        if not self.plan_ok:
            target_v = 0.0
            self.braking = True
            self.brake_reason = "路径不可行，停车等待"

        # 诊断字段（无头回归时读取，确认速度指令到底被谁压住）
        self.dbg_s_err = s_err
        self.dbg_pspeed = person.speed
        self.dbg_dtarget = d_target
        self.dbg_target_v = target_v
        self.dbg_plan_ok = self.plan_ok

        # 8) 近障最低蠕行 + 加速度/制动分离
        if clearance < config.SAFE_CLEARANCE * 0.5:
            target_v = min(target_v, config.DOG_REVERSE_SPEED * 0.5)
        dv = target_v - self.v
        a = config.DOG_ACCEL if dv > 0 else config.DOG_DECEL
        self.v += max(-a * dt, min(a * dt, dv))
        self.v = max(0.0, self.v)

        # 9) 本帧期望位移
        self.theta = wrap_angle(self.theta + self.omega * dt)
        dx_want = self.v * math.cos(self.theta) * dt
        dy_want = self.v * math.sin(self.theta) * dt
        want_len = math.hypot(dx_want, dy_want)

        # 10) 【核心修复】位移投影到安全集，替代 v0.3 的坐标瞬移
        #     只削减"会侵入安全圈的法向分量"，保留沿障碍边缘的切向分量；
        #     同时按削减比例下调实际速度。位移永远不会超过本帧期望值，
        #     因此完全由电机速度实现，不产生任何凭空位移。
        guards = [(person.x, person.y, config.PERSON_RADIUS,
                   config.OWNER_HARD_GAP)]
        dx_real, dy_real = self._safe_displacement(dx_want, dy_want,
                                                   obstacles, dt, guards)
        real_len = math.hypot(dx_real, dy_real)
        # 位移硬上限 = 主动位移 + 一帧的低速脱困横移。
        # ⚠️ 脱困位移不能反过来放大速度：v0.4 初版直接用
        #    real_len/want_len 缩放 self.v，狗静止被行人挤时比例高达 47 倍，
        #    速度当场暴涨。这里钳到 <=1，保证 v 只减不增、位移不超物理上限。
        allowed = want_len + config.DOG_REVERSE_SPEED * dt
        if real_len > allowed:
            dx_real *= allowed / real_len
            dy_real *= allowed / real_len
            real_len = allowed
        ratio = min(1.0, real_len / want_len) if want_len > 1e-6 else 1.0
        self.safety_clipped = 1.0 - ratio
        if self.safety_clipped > 0.02:
            self.v *= ratio
            self.braking = True
            if not self.brake_reason:
                self.brake_reason = "贴障碍绕行，速度受限"

        self.x = max(20.0, min(config.WORLD_W - 20.0, self.x + dx_real))
        self.y = max(20.0, min(config.WORLD_H - 20.0, self.y + dy_real))

        self.trail.append((self.x, self.y))
        if len(self.trail) > 240:
            self.trail = self.trail[-240:]

    # ---------- 安全位移层 ----------

    def _safe_displacement(self, dx: float, dy: float,
                           obstacles: list[Obstacle], dt: float,
                           guards: list[tuple[float, float, float, float]]
                           ) -> tuple[float, float]:
        """把期望位移投影到"所有安全圈之外"，返回物理可执行的位移。

        circles 里既有障碍/行人，也有**被跟随的主人**（v0.4 新增）：
        主人转身折返或手动操控迎面跑向狗时，过去会直接穿模。
        用保守外接圆 DOG_PLAN_RADIUS 判定，所以精确胶囊验收必然更宽松。
        """
        hard_r_add = config.DOG_PLAN_RADIUS + config.SAFE_CLEARANCE
        circles = [(o.x, o.y, o.radius + hard_r_add) for o in obstacles]
        # guards: (x, y, 半径, 专属安全距)
        circles += [(gx, gy, gr + config.DOG_PLAN_RADIUS + gclear)
                    for gx, gy, gr, gclear in guards]
        if not circles:
            return dx, dy
        want0 = math.hypot(dx, dy)
        want_dx, want_dy = dx, dy

        # (a) 侵入或逼近安全圈：以受限低速沿远离方向脱困。
        #     阈值放宽 r+15cm 是必要的：等到完全贴住才推，多个安全圈
        #     会把狗夹在中间、位移被削成 0 而永久冻结（实测踩过）。
        esc_x = esc_y = 0.0
        for cx0, cy0, r in circles:
            ox, oy = self.x - cx0, self.y - cy0
            d = math.hypot(ox, oy)
            rr = r + 15.0
            if d < rr and d > 1e-6:
                step = min(rr - d, config.DOG_REVERSE_SPEED * dt)
                esc_x += ox / d * step
                esc_y += oy / d * step
        esc_len = math.hypot(esc_x, esc_y)
        cap = config.DOG_REVERSE_SPEED * dt
        if esc_len > cap:
            esc_x *= cap / esc_len
            esc_y *= cap / esc_len

        # (b) 在"脱困后的位置"上，把主动位移的靠近分量削掉
        bx, by = self.x + esc_x, self.y + esc_y
        for _ in range(3):
            changed = False
            for cx0, cy0, r in circles:
                nx, ny = bx + dx - cx0, by + dy - cy0
                nd = math.hypot(nx, ny)
                if nd >= r or nd < 1e-6:
                    continue
                ux, uy = bx - cx0, by - cy0
                ud = math.hypot(ux, uy)
                if ud < 1e-6:
                    ux, uy, ud = 1.0, 0.0, 1.0
                ux, uy = ux / ud, uy / ud
                toward = dx * ux + dy * uy      # 负 = 朝安全圈靠近
                if toward < 0.0:
                    dx -= toward * ux          # 投影到切平面：只留切向
                    dy -= toward * uy
                    changed = True
            if not changed:
                break

        # (c) 卡死兜底：所有方向都被削光时，沿最近安全圈的切向低速侧移。
        #     四足本来就能横移，位移仍受 (10) 的 allowed 约束，不是瞬移。
        if math.hypot(dx, dy) < 0.12 * want0 and want0 > 1e-6:
            cx0, cy0, _ = min(circles,
                              key=lambda c: math.hypot(c[0] - self.x,
                                                       c[1] - self.y))
            ux, uy = self.x - cx0, self.y - cy0
            un = math.hypot(ux, uy)
            if un > 1e-6:
                ux, uy = ux / un, uy / un
                tx, ty = -uy, ux
                if want_dx * tx + want_dy * ty < 0:
                    tx, ty = -tx, -ty
                step = min(config.DOG_REVERSE_SPEED * dt, want0 * 0.3)
                dx += tx * step
                dy += ty * step
        return dx + esc_x, dy + esc_y

    # ---------- 观测 ----------

    def surface_gap(self, person: Person) -> float:
        """狗前缘到跑者后背的表面间距（跟随指标）。保守用规划半径。"""
        return (math.hypot(person.x - self.x, person.y - self.y)
                - config.PERSON_RADIUS - config.DOG_PLAN_RADIUS)


# 兼容旧引用（v0.3 测试脚本 / 外部 import）
Cube = RobotDog
