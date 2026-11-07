"""simulation.py —— 场景、输入与 pygame 渲染（v0.4 机器狗跑步跟随）。

按键一览
--------
    M          切换跑者：自动沿跑道跑 / 我用手操（速度都固定 5m/s）
    WASD/方向键 手动模式下控制跑者的方向
    鼠标左键拖动 推动任意障碍/行人（限速 5m/s）
    1~9 / 0    选中某个对象； IJKL 移动选中的对象（限速 5m/s）
    C          开/关其他行人的自主运动（同向 / 对向 / 横穿）
    P          开/关全体对象沿跑道反向巡逻（5m/s，演示动态避让）
    空格 暂停   R 重置   ESC 退出
    [ / ]      仿真倍率 0.25× / 0.5× / 1×（物理速度不变，方便看清 5m/s 动作）
"""

import math
import sys

import pygame

from . import config
from .dog import RobotDog
from .obstacles import (Obstacle, Pedestrian, build_obstacles,
                        capsule_min_clearance, min_clearance)
from .person import Person
from .planner import Planner
from .track import Track

# ---------- 配色（浅色主题） ----------
BG = (238, 244, 238)
TRACK_COLOR = (196, 132, 112)
TRACK_EDGE = (250, 250, 248)
TRACK_CENTER = (246, 232, 224)
PERSON_COLOR = (200, 62, 58)          # 被跟随的跑者：红
PERSON_HEAD = (150, 38, 36)
PEDESTRIAN_COLOR = (70, 112, 168)     # 其他行人：蓝
PEDESTRIAN_HEAD = (44, 76, 124)
DOG_COLOR = (214, 126, 48)            # 机器狗：橙
DOG_TOP = (244, 174, 96)
DOG_EDGE = (128, 68, 20)
OBST_COLOR = (128, 136, 142)          # 路锥：灰
OBST_EDGE = (86, 94, 100)
OBST_SEL = (230, 160, 40)
SAFE_RING = (222, 186, 180)
PATH_COLOR = (46, 150, 96)
FUTURE_COLOR = (120, 170, 210)
TARGET_COLOR = (138, 68, 173)
TRAIL_P = (236, 176, 172)
TRAIL_D = (232, 196, 152)
TEXT = (46, 54, 50)
WARN = (196, 58, 48)


class World:
    """把场景、重规划和统计收在一起，方便脱离 pygame 做无头回归测试。"""

    def __init__(self) -> None:
        self.track = Track(config.WORLD_W / 2, config.WORLD_H / 2)
        self.obstacles = build_obstacles(self.track)
        self.pedestrians = [o for o in self.obstacles if isinstance(o, Pedestrian)]
        self.planner = Planner(self.track, self.obstacles)
        self.planner.set_obstacles(self.obstacles)
        self.person = Person(self.track)

        tx, ty = self.person.follow_target()
        dx, dy = self.person.dx, self.person.dy
        self.dog = RobotDog(tx, ty, math.atan2(dy, dx))
        # 初速直接给到 5m/s：从 0 起步会被拉开约 1m，跟随队形很难收敛
        self.dog.v = config.PERSON_SPEED

        self.autonomous = True       # 其他行人是否自主运动（C 键切换）
        self.future: list[tuple[float, float]] = []
        self.replan_timer = 0.0
        self.path_age = 0.0
        self.replan_count = 0
        self.time = 0.0

        # ---------- 验收统计 ----------
        self.min_clearance_seen = float("inf")       # 与所有对象（精确胶囊）
        self.min_pedestrian_gap = float("inf")       # 只统计"其他行人"
        self.violation_frames = 0                   # 间隙 < 80cm 的帧
        self.person_bumps = 0                       # 跑者蹭到障碍的帧
        self.gaps: list[float] = []                 # 身后间距采样
        self.max_speed_seen = 0.0                   # 机器狗最高速
        self.brake_frames = 0                       # 安全层介入帧数
        self.teleport_frames = 0                    # 位移超过物理上限的帧（应恒为 0）
        self._prev_dog_xy = (self.dog.x, self.dog.y)

        self._replan()

    def _replan(self) -> None:
        """规划到跟随点，再沿跑者未来路线追加延伸段。

        终点必须是"人身后 2m"的跟随点本身，否则 A* 会横穿操场去追人前方的点；
        但路径不能在跟随点结束（狗会冲过终点后掉头打转），所以补上未来路线，
        让狗始终有前方的路可看 —— 同时实现"提前定义未来行走路线"。
        """
        target = self.person.follow_target()
        self.future = self.person.future_route(steps=10, spacing=70.0)
        path = self.planner.find_path((self.dog.x, self.dog.y), target)
        self.dog.plan_ok = path is not None
        if path:
            # 只把未来路线的**前 3 个点**拼进路径，够纯追踪前视（≤140cm）用。
            # 拼满 10 点会让绿色路径和浅蓝色预测路线完全重叠、画面分不清。
            self.dog.path = path[:-1] + [target] + self.future[:3]
            self.replan_count += 1
        # 规划失败时保留上一帧路径，控制器会因 plan_ok=False 减速停车
        self.replan_timer = 0.0
        self.path_age = 0.0

    def update(self, dt: float, manual_dir=None, drag_obstacle=None,
               drag_target=None, obstacle_input=None, patrol=False) -> None:
        self.time += dt

        # 1) 对象先动：速度清零 → 自主运动 / 巡逻 / 用户控制
        for o in self.obstacles:
            o.vx = o.vy = 0.0
        pedestrians_free = self.autonomous and not patrol

        if patrol:
            for o in self.obstacles:
                o.update_patrol(self.track, dt)
        else:
            for o in self.obstacles:
                if pedestrians_free and isinstance(o, Pedestrian):
                    o.update_autonomous(self.track, dt, self.time)
            if drag_obstacle is not None and drag_target is not None:
                drag_obstacle.move_toward(*drag_target, dt)
                drag_obstacle.sync_track_frame(self.track)
            if obstacle_input is not None:
                for o in self.obstacles:
                    if o.selected:
                        o.move_by_input(*obstacle_input, dt)
                        o.sync_track_frame(self.track)

        # 2) 实体约束：只对"非自主行人"生效。
        #    真实世界里旁人不会给机器人让路，自主行人不能被狗推开 —— 狗必须
        #    靠预测规划与速度投影自己让开。用户手动拖动的对象才做约束，
        #    避免故意把物体怼进狗身体造成无解的互相穿透。
        need = (config.DOG_PLAN_RADIUS + config.SAFE_CLEARANCE + 3.0)
        for o in self.obstacles:
            if pedestrians_free and isinstance(o, Pedestrian):
                continue
            d = math.hypot(o.x - self.dog.x, o.y - self.dog.y)
            limit = need + o.radius
            if d < limit:
                dx, dy = o.x - self.dog.x, o.y - self.dog.y
                if d < 1e-6:
                    dx, dy, d = 1.0, 0.0, 1.0
                o.x = self.dog.x + dx / d * limit
                o.y = self.dog.y + dy / d * limit
                o.sync_track_frame(self.track)

        # 3) 跑者
        self.person.update(dt, self.obstacles, manual_dir)
        for o in self.obstacles:
            if math.hypot(self.person.x - o.x, self.person.y - o.y) < \
                    config.PERSON_RADIUS + o.radius:
                self.person_bumps += 1

        # 4) 重规划 + 控制
        self.replan_timer += dt
        self.path_age += dt
        if self.replan_timer >= config.REPLAN_INTERVAL:
            self._replan()
        self.dog.step(self.person, self.obstacles, dt, self.path_age)

        # 5) 验收统计（用精确胶囊几何，不是保守外接圆）
        exact = capsule_min_clearance(self.dog.x, self.dog.y,
                                      self.dog.theta, self.obstacles)
        self.min_clearance_seen = min(self.min_clearance_seen, exact)
        ped_gap = capsule_min_clearance(self.dog.x, self.dog.y,
                                        self.dog.theta, self.pedestrians)
        self.min_pedestrian_gap = min(self.min_pedestrian_gap, ped_gap)
        if exact < config.SAFE_CLEARANCE:
            self.violation_frames += 1
        if self.dog.braking:
            self.brake_frames += 1
        self.max_speed_seen = max(self.max_speed_seen, self.dog.v)
        # 反作弊检查：单帧位移不得超过 v*dt（+ 数值余量），否则说明还在瞬移
        moved = math.hypot(self.dog.x - self._prev_dog_xy[0],
                           self.dog.y - self._prev_dog_xy[1])
        if moved > self.dog.v * dt + config.DOG_REVERSE_SPEED * dt + 0.5:
            self.teleport_frames += 1
        self._prev_dog_xy = (self.dog.x, self.dog.y)

        if self.time > 2.0:                    # 跳过起步 2 秒再统计
            self.gaps.append(self.dog.surface_gap(self.person))


# ---------- 绘制 ----------

def _font(size: int):
    for path in ("C:/Windows/Fonts/msyh.ttc", "C:/Windows/Fonts/simhei.ttf"):
        try:
            return pygame.font.Font(path, size)
        except OSError:
            continue
    return pygame.font.SysFont("arial", size)


def draw_track(screen, track: Track) -> None:
    # 外边界多边形 + 内边界挖空：粗折线在直线/半圆接缝会留方形端帽
    outer = track.points(300, config.TRACK_HALF)
    inner = track.points(300, -config.TRACK_HALF)
    pygame.draw.polygon(screen, TRACK_COLOR, outer)
    pygame.draw.polygon(screen, BG, inner)
    pygame.draw.lines(screen, TRACK_EDGE, True, outer, 3)
    pygame.draw.lines(screen, TRACK_EDGE, True, inner, 3)
    cpts = track.points(300)
    for i in range(0, len(cpts), 12):
        pygame.draw.lines(screen, TRACK_CENTER, False, cpts[i:i + 6], 2)


def draw_person(screen, p: Person, color=PERSON_COLOR,
                head=PERSON_HEAD) -> None:
    x, y = p.x / config.SCALE, p.y / config.SCALE
    r = config.PERSON_RADIUS / config.SCALE
    ang = math.atan2(p.dy, p.dx)
    w = int(config.PERSON_RADIUS * 2 / config.SCALE)
    h = int(20 / config.SCALE * 1.6)
    rect = pygame.Rect(0, 0, w, h)
    rect.center = (x, y)
    pygame.draw.ellipse(screen, color, rect)
    pygame.draw.circle(screen, head,
                       (x + math.cos(ang) * r * 0.55,
                        y + math.sin(ang) * r * 0.55), max(3, r * 0.38))
    pygame.draw.line(screen, head, (x, y),
                     (x + math.cos(ang) * r * 1.7,
                      y + math.sin(ang) * r * 1.7), 2)


def draw_pedestrian(screen, o: Pedestrian) -> None:
    x, y = o.x / config.SCALE, o.y / config.SCALE
    r = o.radius / config.SCALE
    # 预测扫掠虚影：让"提前避让"在画面上可见
    if math.hypot(o.vx, o.vy) > 1.0 and hasattr(o, "predicted_positions"):
        for px, py in o.predicted_positions():
            pygame.draw.circle(screen, (178, 202, 226),
                               (px / config.SCALE, py / config.SCALE),
                               max(2, int(r * 0.55)), 1)
    rect = pygame.Rect(0, 0, int(r * 2), int(r * 1.3))
    rect.center = (x, y)
    pygame.draw.ellipse(screen, PEDESTRIAN_COLOR, rect)
    pygame.draw.ellipse(screen, OBST_SEL if o.selected else PEDESTRIAN_HEAD,
                        rect, 2)
    # 运动方向箭头
    if math.hypot(o.vx, o.vy) > 1.0:
        n = math.hypot(o.vx, o.vy)
        pygame.draw.line(screen, PEDESTRIAN_HEAD, (x, y),
                         (x + o.vx / n * r * 1.8, y + o.vy / n * r * 1.8), 2)


def draw_dog(screen, dog: RobotDog) -> None:
    """按真实 70×35cm 长宽比画四足本体 + 四个足端 + 头部传感器。"""
    L = int(config.DOG_LENGTH / config.SCALE)
    W = int(config.DOG_WIDTH / config.SCALE)
    body = pygame.Surface((L, W), pygame.SRCALPHA)
    body.fill(DOG_COLOR)
    pygame.draw.rect(body, DOG_EDGE, (0, 0, L, W), 2)
    pygame.draw.rect(body, DOG_TOP, (4, 4, L - 8, W - 8), 2)
    # 四个足端（深棕小方块），长边前后各两个
    for fx in (L * 0.18, L * 0.82):
        for fy in (2, W - 6):
            pygame.draw.rect(body, DOG_EDGE, (int(fx), int(fy), 5, 4))
    # 头部：前端半圆 + 传感器点（车头方向 = +x）
    pygame.draw.circle(body, DOG_EDGE, (L - 4, W // 2), max(3, W // 5))
    pygame.draw.circle(body, (250, 236, 180), (L - 4, W // 2), 2)
    rot = pygame.transform.rotate(body, -math.degrees(dog.theta))
    screen.blit(rot, rot.get_rect(center=(dog.x / config.SCALE,
                                          dog.y / config.SCALE)))


def draw_object(screen, o: Obstacle) -> None:
    x, y = o.x / config.SCALE, o.y / config.SCALE
    safe_r = (o.radius + config.DOG_PLAN_RADIUS
              + config.SAFE_CLEARANCE) / config.SCALE
    pygame.draw.circle(screen, SAFE_RING, (x, y), safe_r, 1)
    if isinstance(o, Pedestrian):
        draw_pedestrian(screen, o)
        return
    r = o.radius / config.SCALE
    pygame.draw.circle(screen, OBST_COLOR, (x, y), r)
    pygame.draw.circle(screen, OBST_SEL if o.selected else OBST_EDGE,
                       (x, y), r, 3 if o.selected else 2)
    pygame.draw.line(screen, OBST_EDGE, (x - r * 0.5, y), (x + r * 0.5, y), 2)
    pygame.draw.line(screen, OBST_EDGE, (x, y - r * 0.5), (x, y + r * 0.5), 2)


def draw(screen, font, small, world: World, paused: bool, rate: float,
         patrol: bool) -> None:
    screen.fill(BG)
    draw_track(screen, world.track)

    for o in world.obstacles:
        draw_object(screen, o)

    if len(world.dog.path) > 1:
        pts = [(x / config.SCALE, y / config.SCALE) for x, y in world.dog.path]
        pygame.draw.lines(screen, PATH_COLOR, False, pts, 3)

    # 预测路线画在规划路径之上（用细虚线+点），两者只在前视段重叠，
    # 仍能看出"绿色=本次要走 / 浅蓝=跑者未来会去哪"
    if len(world.future) > 1:
        pts = [(x / config.SCALE, y / config.SCALE) for x, y in world.future]
        for a, b in zip(pts[:-1], pts[1:]):
            pygame.draw.line(screen, FUTURE_COLOR, a, b, 2)
        for p in pts[::2]:
            pygame.draw.circle(screen, FUTURE_COLOR, p, 3)

    for trail, color in ((world.person.trail, TRAIL_P),
                         (world.dog.trail, TRAIL_D)):
        if len(trail) > 1:
            pygame.draw.lines(screen, color, False,
                              [(x / config.SCALE, y / config.SCALE)
                               for x, y in trail], 2)

    tx, ty = world.person.follow_target()
    tx, ty = tx / config.SCALE, ty / config.SCALE
    pygame.draw.circle(screen, TARGET_COLOR, (tx, ty), 5)
    pygame.draw.circle(screen, TARGET_COLOR, (tx, ty), 4, 1)

    draw_person(screen, world.person)
    draw_dog(screen, world.dog)

    # ---------- HUD ----------
    gap = world.dog.surface_gap(world.person)
    clr = capsule_min_clearance(world.dog.x, world.dog.y,
                                world.dog.theta, world.obstacles)
    brake_cm = config.braking_distance()
    p_mode = "自动沿跑道" if world.person.mode == "auto" else "我用手操"
    left = [
        "机器狗跟随跑者 · 操场跑道 v0.4",
        f"跑者 {p_mode}  {config.PERSON_SPEED/100:.1f} m/s  第 {world.person.lap+1} 圈",
        f"身后间距 {gap:6.0f} cm   (目标 {config.FOLLOW_GAP_MIN:.0f}~{config.FOLLOW_GAP_MAX:.0f})",
        f"狗速 {world.dog.v/100:4.2f} m/s  巡航≤5.00  恢复≤{config.DOG_MAX_SPEED/100:.2f}",
        f"障碍间隙 {clr:6.0f} cm  (硬下限 {config.SAFE_CLEARANCE:.0f})",
        f"全程最小 {world.min_clearance_seen:6.0f} cm  行人最小 {world.min_pedestrian_gap:6.0f} cm  违规 {world.violation_frames} 帧",
    ]
    right = [
        f"制动安全距 {brake_cm:.0f} cm  |  重规划 {world.replan_count} 次  |  倍率 {rate}x",
        f"行人自主 {'开' if world.autonomous and not patrol else '关'}   其他行人 {len(world.pedestrians)} 名   安全介入 {world.brake_frames} 帧",
        "M 切换跑者 自动/手动    WASD 控制跑者(5m/s)",
        "鼠标拖物体    1~9/0 选中 + IJKL 移动",
        "C 行人自主运动    P 全体巡逻    [ ] 倍率",
        "空格 暂停    R 重置    ESC 退出",
    ]
    panel = pygame.Surface((1100, 140), pygame.SRCALPHA)
    panel.fill((252, 253, 250, 210))
    screen.blit(panel, (10, 8))
    for i, text in enumerate(left):
        screen.blit((font if i == 0 else small).render(text, True, TEXT),
                    (22, 14 + i * 21))
    for i, text in enumerate(right):
        screen.blit(small.render(text, True, TEXT), (545, 14 + i * 21))

    if world.dog.braking and world.dog.brake_reason:
        screen.blit(font.render("减速中：" + world.dog.brake_reason,
                                True, (168, 104, 30)), (22, 152))
    if world.dog.danger:
        screen.blit(font.render("! 安全间隙不足", True, WARN), (22, 178))
    if not world.dog.plan_ok:
        screen.blit(font.render("! 路径不可行，停车等待", True, WARN), (22, 204))
    if paused:
        screen.blit(font.render("已暂停", True, WARN),
                    (config.WINDOW_W - 130, 158))
    if patrol:
        screen.blit(font.render("全体巡逻中", True, (200, 120, 40)),
                    (config.WINDOW_W - 160, 185))


def main() -> None:
    pygame.init()
    screen = pygame.display.set_mode((config.WINDOW_W, config.WINDOW_H))
    pygame.display.set_caption("luyaCar v0.4 · 机器狗跑步跟随（避障 + 行人防碰撞）")
    clock = pygame.time.Clock()
    font, small = _font(20), _font(16)

    world = World()
    paused = False
    patrol = False
    rates = (0.25, 0.5, 1.0)
    rate_idx = 2
    drag_obstacle: Obstacle | None = None
    drag_target = None

    while True:
        real_dt = clock.tick(config.FPS) / 1000.0
        for e in pygame.event.get():
            if e.type == pygame.QUIT:
                pygame.quit(); sys.exit(0)
            if e.type == pygame.KEYDOWN:
                if e.key == pygame.K_ESCAPE:
                    pygame.quit(); sys.exit(0)
                elif e.key == pygame.K_SPACE:
                    paused = not paused
                elif e.key == pygame.K_r:
                    world = World()
                elif e.key == pygame.K_m:
                    world.person.toggle_mode()
                elif e.key == pygame.K_p:
                    patrol = not patrol
                elif e.key == pygame.K_c:
                    world.autonomous = not world.autonomous
                elif e.key == pygame.K_LEFTBRACKET:
                    rate_idx = max(0, rate_idx - 1)
                elif e.key == pygame.K_RIGHTBRACKET:
                    rate_idx = min(len(rates) - 1, rate_idx + 1)
                elif pygame.K_1 <= e.key <= pygame.K_9:
                    idx = e.key - pygame.K_1
                    for i, o in enumerate(world.obstacles):
                        o.selected = (i == idx)
                elif e.key == pygame.K_0:
                    for i, o in enumerate(world.obstacles):
                        o.selected = (i == 9)
            if e.type == pygame.MOUSEBUTTONDOWN and e.button == 1:
                mx, my = e.pos[0] * config.SCALE, e.pos[1] * config.SCALE
                hits = [(math.hypot(o.x - mx, o.y - my), o)
                        for o in world.obstacles]
                if hits and min(h[0] for h in hits) < 45.0:
                    drag_obstacle = min(hits, key=lambda h: h[0])[1]
                    for o in world.obstacles:
                        o.selected = (o is drag_obstacle)
                    drag_target = (mx, my)
            if e.type == pygame.MOUSEMOTION and drag_obstacle is not None:
                drag_target = (e.pos[0] * config.SCALE, e.pos[1] * config.SCALE)
            if e.type == pygame.MOUSEBUTTONUP and e.button == 1:
                drag_obstacle, drag_target = None, None

        keys = pygame.key.get_pressed()
        manual_dir = None
        if world.person.mode == "manual":
            dx = (keys[pygame.K_d] or keys[pygame.K_RIGHT]) - \
                 (keys[pygame.K_a] or keys[pygame.K_LEFT])
            dy = (keys[pygame.K_s] or keys[pygame.K_DOWN]) - \
                 (keys[pygame.K_w] or keys[pygame.K_UP])
            if dx or dy:
                manual_dir = (dx, dy)
        oi = (keys[pygame.K_l] - keys[pygame.K_j],
              keys[pygame.K_k] - keys[pygame.K_i])
        obstacle_input = oi if (oi[0] or oi[1]) else None

        if not paused:
            world.update(real_dt * rates[rate_idx], manual_dir,
                         drag_obstacle, drag_target, obstacle_input, patrol)

        draw(screen, font, small, world, paused, rates[rate_idx], patrol)
        pygame.display.flip()
