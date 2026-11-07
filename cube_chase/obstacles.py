"""obstacles.py —— 静态障碍 + 其他行人（v0.4 新增「避免与旁人碰撞」）。

落地场景里的避碰对象分三类，行为完全不同：

============  ==================  =====================  ==================
kind           实体                运动                    狗的应对
============  ==================  =====================  ==================
cone           路锥 / 路桩 r=18    静止                    绕行
person-same    其他跑者 r=25       同向慢走 1.5m/s         提前减速、侧向超越
person-oppo    对向行人 r=25       迎面走来 1.5m/s         提前让出通道
person-cross   横穿行人 r=25       横向来回穿跑道          预测轨迹、停车礼让
============  ==================  =====================  ==================

保留的交互（对应原规则7：障碍可移动、速度上限与人一致 5m/s）：
    1. 鼠标左键拖动  move_toward()   限速 5m/s
    2. 数字键选中 + IJKL            限速 5m/s
    3. P 键让所有对象沿跑道反向巡逻  限速 5m/s
    4. C 键开/关行人的自主运动（关掉后它们完全是静态/可拖动的）

⚠️ 设计要点：自主行人**不服从"实体约束"**（不会被狗推开）。真实世界里
   旁人不会给机器人让路，所以狗必须靠预测规划 + 速度投影自己让开；
   只有被用户手动拖动的对象才做实体约束，避免用户故意把它们怼进狗身体
   造成无解的穿模。
"""

import math

from . import config
from .track import Track


class Obstacle:
    """静态障碍基类（路锥）。vx/vy 供规划器做运动预测。"""

    kind = "cone"

    def __init__(self, x: float, y: float, s: float = 0.0,
                 lateral: float = 0.0, radius: float | None = None,
                 label: str = "") -> None:
        self.x, self.y = x, y
        self.s = s
        self.lateral = lateral
        self.radius = config.CONE_RADIUS if radius is None else radius
        self.vx = self.vy = 0.0
        self.selected = False
        self.label = label
        self.autonomous = False      # 是否自主运动（行人按 C 键可开关）

    # ---------- 几何 ----------

    def clearance(self, x: float, y: float, body_radius: float) -> float:
        """物体（半径 body_radius）与本障碍的**表面间距**（圆-圆）。"""
        return math.hypot(x - self.x, y - self.y) - self.radius - body_radius

    def capsule_clearance(self, x: float, y: float,
                          theta: float) -> float:
        """机器狗胶囊体到本障碍的表面间距（精确几何，不用外接圆）。

        胶囊 = 沿狗朝向的中轴线段 [P0,P1] + 端点半径 DOG_RADIUS。
        先算障碍中心到线段的最短距离，再扣掉双方半径。
        这样狗"纵向细长"的形状被正确利用：正对着障碍时比侧身更危险。
        """
        ax = math.cos(theta) * config.DOG_HALF_AXIS
        ay = math.sin(theta) * config.DOG_HALF_AXIS
        p0x, p0y = x - ax, y - ay
        p1x, p1y = x + ax, y + ay
        dx, dy = p1x - p0x, p1y - p0y
        L2 = dx * dx + dy * dy
        if L2 <= 1e-9:
            t = 0.0
        else:
            t = max(0.0, min(1.0, ((self.x - p0x) * dx + (self.y - p0y) * dy) / L2))
        d = math.hypot(self.x - (p0x + dx * t), self.y - (p0y + dy * t))
        return d - self.radius - config.DOG_RADIUS

    # ---------- 用户控制 ----------

    def move_toward(self, tx: float, ty: float, dt: float) -> None:
        """朝目标点移动，速度上限 = 人速（5m/s），拖鼠标也不会瞬移。"""
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
        """IJKL 控制：方向归一化，速度固定 5m/s。"""
        n = math.hypot(dx, dy)
        if n < 1e-6:
            self.vx = self.vy = 0.0
            return
        self.vx = dx / n * config.OBSTACLE_MAX_SPEED
        self.vy = dy / n * config.OBSTACLE_MAX_SPEED
        self.x = max(self.radius, min(config.WORLD_W - self.radius,
                                      self.x + self.vx * dt))
        self.y = max(self.radius, min(config.WORLD_H - self.radius,
                                      self.y + self.vy * dt))

    def update_patrol(self, track: Track, dt: float) -> None:
        """沿跑道反向巡逻（速度与人一致），用于动态避障演示。"""
        self.s = (self.s - config.OBSTACLE_MAX_SPEED * dt) % track.total_length
        self.x, self.y = track.point_at(self.s, self.lateral)
        tx, ty = track.tangent_at(self.s)
        self.vx = -tx * config.OBSTACLE_MAX_SPEED
        self.vy = -ty * config.OBSTACLE_MAX_SPEED

    def sync_track_frame(self, track: Track) -> None:
        """被拖动后重新对齐跑道坐标，巡逻时不会突然跳位。"""
        self.s, self.lateral = track.frame(self.x, self.y)


class Pedestrian(Obstacle):
    """其他行人：与跑者同尺寸（50cm 宽），有自己的运动意图。

    behavior:
        same     同向慢走（沿跑道正向）
        opposite 对向走来（沿跑道反向）
        cross    横向来回横穿跑道（最考验预测与礼让）
    """

    kind = "person"

    def __init__(self, track: Track, s: float, lateral: float,
                 behavior: str, phase: float = 0.0, label: str = "") -> None:
        x, y = track.point_at(s, lateral)
        super().__init__(x, y, s, lateral, config.PEDESTRIAN_RADIUS, label)
        self.behavior = behavior
        self.phase = phase
        self.base_s = s
        self.base_lateral = lateral
        self.autonomous = True
        self.speed = config.PEDESTRIAN_SPEED
        # 横穿只发生在跑道**外侧半幅**（lateral 40~130cm），内侧始终留一条
        # 快车道。若让行人从 -130 穿到 +130，其 120cm 安全圈会把 2.4m
        # 跑道完全封死，机器狗无路可走（实测会冻结后掉队 10m）。
        # 扫参结论：85±45 时跟随中位 207cm（目标 200），优于 65±65 的 251cm。
        self.cross_center = 85.0
        self.cross_amp = 45.0
        self.cross_omega = 1.05

    def update_autonomous(self, track: Track, dt: float, t: float) -> None:
        """自主运动。用前后位置差反推 vx/vy，供规划器做多点轨迹预测。"""
        ox, oy = self.x, self.y
        if self.behavior == "same":
            self.s = (self.s + self.speed * dt) % track.total_length
            self.lateral = self.base_lateral
        elif self.behavior == "opposite":
            self.s = (self.s - self.speed * dt) % track.total_length
            self.lateral = self.base_lateral
        elif self.behavior == "cross":
            a = self.cross_omega * (t + self.phase)
            self.lateral = self.cross_center + self.cross_amp * math.sin(a)
            # s 保持 base_s：横穿行人只做横向运动，轨迹更容易被预测，
            # 也避免"边走边横穿"把整个跑道都封死
        self.x, self.y = track.point_at(self.s, self.lateral)
        self.vx = (self.x - ox) / dt if dt > 1e-6 else 0.0
        self.vy = (self.y - oy) / dt if dt > 1e-6 else 0.0

    def predicted_positions(self) -> list[tuple[float, float]]:
        """未来 PREDICT_HORIZON 内的采样位置（规划器把它标成扫掠禁区）。"""
        pts = []
        for i in range(1, config.PREDICT_SAMPLES + 1):
            h = config.PREDICT_HORIZON * i / config.PREDICT_SAMPLES
            dx, dy = self.vx * h, self.vy * h
            d = math.hypot(dx, dy)
            if d > config.PREDICT_MAX_DIST:
                k = config.PREDICT_MAX_DIST / d
                dx, dy = dx * k, dy * k
            pts.append((self.x + dx, self.y + dy))
        return pts


def build_obstacles(track: Track) -> list[Obstacle]:
    """6 个路锥 + 4 名行人。

    路锥全部从跑道**外侧**探入（lateral +50~+70cm），留出一条内侧快车道：
    机器狗沿内侧平滑通过，既满足离障碍 ≥80cm，又不用每圈横穿跑道 6 次
    把速度全耗在打方向上。跑者则每个路锥都要往内侧让一次，跑成波浪线。
    """
    out: list[Obstacle] = []

    fractions = [0.07, 0.20, 0.34, 0.51, 0.69, 0.86]
    laterals = [55.0, 68.0, 52.0, 65.0, 58.0, 70.0]
    for i, (frac, lat) in enumerate(zip(fractions, laterals), start=1):
        s = frac * track.total_length
        x, y = track.point_at(s, lat)
        out.append(Obstacle(x, y, s, lat, label=f"锥{i}"))

    # 行人错开跑者起点（跑者从 s=0 出发），避免开局就贴身。
    # ⚠️ 所有行人都在**外侧**半幅（lateral > 0）：v0.4 初版把对向行人
    #    放在内侧 -95cm，正好堵死机器狗的"内侧快车道"，跟随中位从 207cm
    #    恶化到 495cm。真实跑道上也一样 —— 逆行的人靠外，机器狗才能贴内线过。
    specs = [
        (0.30, 100.0, "same", 0.0, "同向慢跑者"),
        (0.62, 90.0, "opposite", 0.0, "对向行人"),
        (0.45, 85.0, "cross", 0.0, "横穿行人A"),
        (0.80, 85.0, "cross", 2.1, "横穿行人B"),
    ]
    for s_frac, lat, behavior, phase, label in specs:
        out.append(Pedestrian(track, s_frac * track.total_length,
                              lat, behavior, phase, label))
    return out


def min_clearance(x: float, y: float, obstacles: list[Obstacle],
                  body_radius: float = config.DOG_PLAN_RADIUS) -> float:
    """到最近障碍的表面间距（保守外接圆版，用于规划/限速判断）。"""
    if not obstacles:
        return float("inf")
    return min(o.clearance(x, y, body_radius) for o in obstacles)


def capsule_min_clearance(x: float, y: float, theta: float,
                          obstacles: list[Obstacle]) -> float:
    """到最近障碍的表面间距（精确胶囊版，用于安全验收与 HUD）。"""
    if not obstacles:
        return float("inf")
    return min(o.capsule_clearance(x, y, theta) for o in obstacles)
