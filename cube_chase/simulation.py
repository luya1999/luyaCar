"""simulation.py —— 场景、输入与 pygame 渲染。

按键一览
--------
    M        切换人：自动沿跑道跑 / 我用手操（速度都固定 5m/s）
    WASD/方向键  手动模式下控制人的方向
    鼠标左键拖动   推动障碍（限速 5m/s）
    1~6      选中某个障碍； IJKL 移动选中的障碍（限速 5m/s）
    P        开/关障碍自动巡逻（沿跑道反向 5m/s，演示动态避让）
    空格 暂停    R 重置    ESC 退出
    [ / ]    仿真倍率 0.25× / 0.5× / 1×（物理速度不变，方便看清 5m/s 的动作）
"""

import math
import sys

import pygame

from . import config
from .cube import Cube
from .obstacles import Obstacle, build_obstacles, min_clearance
from .person import Person
from .planner import Planner
from .track import Track

# ---------- 配色（浅色主题） ----------
BG = (238, 244, 238)             # 草地
GRASS_LINE = (222, 233, 220)
TRACK_COLOR = (196, 132, 112)    # 塑胶跑道
TRACK_EDGE = (250, 250, 248)     # 白色边线
TRACK_CENTER = (246, 232, 224)   # 中心虚线
PERSON_COLOR = (200, 62, 58)
PERSON_HEAD = (150, 38, 36)
CUBE_COLOR = (168, 112, 62)
CUBE_TOP = (204, 156, 98)
CUBE_EDGE = (110, 70, 38)
OBST_COLOR = (128, 136, 142)
OBST_EDGE = (86, 94, 100)
OBST_SEL = (230, 160, 40)
SAFE_RING = (214, 120, 112)
PATH_COLOR = (46, 150, 96)
FUTURE_COLOR = (120, 170, 210)
TARGET_COLOR = (138, 68, 173)
TRAIL_P = (236, 176, 172)
TRAIL_C = (196, 166, 132)
TEXT = (46, 54, 50)
PANEL = (252, 253, 250)


class World:
    """把场景、重规划和统计收在一起，方便脱离 pygame 做无头回归测试。"""

    def __init__(self) -> None:
        self.track = Track(config.WORLD_W / 2, config.WORLD_H / 2)
        self.obstacles = build_obstacles(self.track)
        self.planner = Planner(self.track, self.obstacles)
        self.planner.set_obstacles(self.obstacles)
        self.person = Person(self.track)
        tx, ty = self.person.follow_target()
        dx, dy = self.person.dx, self.person.dy
        self.cube = Cube(tx, ty, math.atan2(dy, dx))
        # 关键：初速直接给到 5m/s。若从 0 起步，加速的 0.2s 会被人拉开 ~80cm，
        # 而规则3 又限制木块不能超速，这个差距一辈子追不回来。
        self.cube.v = config.PERSON_SPEED

        self.future: list[tuple[float, float]] = []
        self.replan_timer = 0.0
        self.path_age = 0.0      # 距上次重规划过了多久（用于目标点前滑补偿）
        self.replan_count = 0
        self.time = 0.0
        self.min_clearance_seen = float("inf")
        self.violation_frames = 0        # 间隙 < 50cm 的帧数（规则5 违规）
        self.person_bumps = 0            # 人撞到障碍的帧数
        self.gaps: list[float] = []      # 表面间距采样（规则2）
        self._replan()

    def _replan(self) -> None:
        # 规划终点必须是"人身后 30~40cm 的跟随点"本身：
        # 若把终点设到人前方，A* 会直接横穿操场去那个点，木块就不再是"跟在身后"。
        # 但路径不能到跟随点就结束（会掉头打转），所以再沿"人未来路线"追加延伸段，
        # 让木块始终有前方的路可看 —— 这同时实现了规则3 的提前规划。
        target = self.person.follow_target()
        self.future = self.person.future_route(steps=10, spacing=70.0)
        path = self.planner.find_path((self.cube.x, self.cube.y), target)
        if path:
            self.cube.path = path[:-1] + [target] + self.future
            self.replan_count += 1
        self.replan_timer = 0.0
        self.path_age = 0.0

    def update(self, dt: float, manual_dir=None, drag_obstacle=None,
               drag_target=None, obstacle_input=None, patrol=False) -> None:
        self.time += dt

        # 障碍先动（它们也是动态障碍，规划会用它们的速度做预测）
        for o in self.obstacles:
            o.vx = o.vy = 0.0
        if patrol:
            for o in self.obstacles:
                o.update_patrol(self.track, dt)
        else:
            if drag_obstacle is not None and drag_target is not None:
                drag_obstacle.move_toward(*drag_target, dt)
                drag_obstacle.sync_track_frame(self.track)
            if obstacle_input is not None:
                for o in self.obstacles:
                    if o.selected:
                        o.move_by_input(*obstacle_input, dt)
                        o.sync_track_frame(self.track)

        # 实体约束：障碍和木块都是有体积的，障碍不能"压进"木块的安全圈。
        # 用户把障碍拖到木块身上、或巡逻的障碍迎面撞过来时，把障碍挡在圈外，
        # 而不是让两个实体互相穿透（否则同速约束下木块再怎么逃都无解）。
        need = (config.OBSTACLE_RADIUS + config.CUBE_RADIUS
                + config.SAFE_CLEARANCE + 3.0)
        for o in self.obstacles:
            dx, dy = o.x - self.cube.x, o.y - self.cube.y
            d = math.hypot(dx, dy)
            if d < need:
                if d < 1e-6:
                    dx, dy, d = 1.0, 0.0, 1.0
                o.x = self.cube.x + dx / d * need
                o.y = self.cube.y + dy / d * need
                o.sync_track_frame(self.track)

        self.person.update(dt, self.obstacles, manual_dir)

        for o in self.obstacles:
            if math.hypot(self.person.x - o.x, self.person.y - o.y) < \
                    config.PERSON_RADIUS + o.radius:
                self.person_bumps += 1

        self.replan_timer += dt
        self.path_age += dt
        if self.replan_timer >= config.REPLAN_INTERVAL:
            self._replan()

        self.cube.step(self.person, self.obstacles, dt, self.path_age)

        clr = min_clearance(self.cube.x, self.cube.y, self.obstacles)
        self.min_clearance_seen = min(self.min_clearance_seen, clr)
        if clr < config.SAFE_CLEARANCE:
            self.violation_frames += 1
        if self.time > 2.0:                    # 跳过起步 2 秒再统计
            self.gaps.append(self.cube.surface_gap(self.person))


# ---------- 绘制 ----------

def _font(size: int):
    for path in ("C:/Windows/Fonts/msyh.ttc", "C:/Windows/Fonts/simhei.ttf"):
        try:
            return pygame.font.Font(path, size)
        except OSError:
            continue
    return pygame.font.SysFont("arial", size)


def draw_track(screen, track: Track) -> None:
    # 用"外边界多边形 + 内边界挖空"画环带，而不是 draw.lines 加粗线：
    # 粗折线在直线段与半圆的接缝处会留下方形端帽，画面上像破了一块。
    outer = track.points(300, config.TRACK_HALF)
    inner = track.points(300, -config.TRACK_HALF)
    pygame.draw.polygon(screen, TRACK_COLOR, outer)
    pygame.draw.polygon(screen, BG, inner)
    pygame.draw.lines(screen, TRACK_EDGE, True, outer, 3)
    pygame.draw.lines(screen, TRACK_EDGE, True, inner, 3)
    # 中心虚线
    cpts = track.points(300)
    for i in range(0, len(cpts), 12):
        pygame.draw.lines(screen, TRACK_CENTER, False, cpts[i:i + 6], 2)


def draw_person(screen, p: Person) -> None:
    x, y = p.x / config.SCALE, p.y / config.SCALE
    r = config.PERSON_RADIUS / config.SCALE
    ang = math.atan2(p.dy, p.dx)
    # 俯视人形：肩（宽 50cm 的椭圆）+ 头 + 朝向小箭头
    w = int(config.PERSON_RADIUS * 2 / config.SCALE)
    h = int(20 / config.SCALE * 1.6)
    rect = pygame.Rect(0, 0, w, h)
    rect.center = (x, y)
    pygame.draw.ellipse(screen, PERSON_COLOR, rect)
    hx = x + math.cos(ang) * r * 0.55
    hy = y + math.sin(ang) * r * 0.55
    pygame.draw.circle(screen, PERSON_HEAD, (hx, hy), max(3, r * 0.38))
    pygame.draw.line(screen, PERSON_HEAD, (x, y),
                     (x + math.cos(ang) * r * 1.7, y + math.sin(ang) * r * 1.7), 2)


def draw_cube(screen, cube: Cube) -> None:
    size = int(config.CUBE_SIZE / config.SCALE)
    body = pygame.Surface((size, size), pygame.SRCALPHA)
    body.fill(CUBE_COLOR)
    pygame.draw.rect(body, CUBE_EDGE, (0, 0, size, size), 2)
    pygame.draw.rect(body, CUBE_TOP, (3, 3, size - 6, size - 6), 2)
    pygame.draw.line(body, CUBE_EDGE, (size - 6, size / 2 - 2),
                     (size - 2, size / 2), 2)   # 车头标记
    rot = pygame.transform.rotate(body, -math.degrees(cube.theta))
    screen.blit(rot, rot.get_rect(center=(cube.x / config.SCALE,
                                          cube.y / config.SCALE)))


def draw_obstacle(screen, o: Obstacle) -> None:
    x, y = o.x / config.SCALE, o.y / config.SCALE
    r = o.radius / config.SCALE
    # 50cm 安全圈（规则5 的可视化）
    safe_r = (o.radius + config.CUBE_RADIUS + config.SAFE_CLEARANCE) / config.SCALE
    pygame.draw.circle(screen, SAFE_RING, (x, y), safe_r, 1)
    pygame.draw.circle(screen, OBST_COLOR, (x, y), r)
    pygame.draw.circle(screen, OBST_SEL if o.selected else OBST_EDGE,
                       (x, y), r, 3 if o.selected else 2)
    pygame.draw.line(screen, OBST_EDGE, (x - r * 0.5, y), (x + r * 0.5, y), 2)
    pygame.draw.line(screen, OBST_EDGE, (x, y - r * 0.5), (x, y + r * 0.5), 2)


def draw(screen, font, small, world: World, paused: bool, rate: float,
         patrol: bool) -> None:
    screen.fill(BG)
    draw_track(screen, world.track)

    # 预测的未来路线（规则3 可视化）
    if len(world.future) > 1:
        pts = [(x / config.SCALE, y / config.SCALE) for x, y in world.future]
        pygame.draw.lines(screen, FUTURE_COLOR, False, pts, 2)
        for p in pts[::2]:
            pygame.draw.circle(screen, FUTURE_COLOR, p, 3)

    for o in world.obstacles:
        draw_obstacle(screen, o)

    # 规划路径
    if len(world.cube.path) > 1:
        pts = [(x / config.SCALE, y / config.SCALE) for x, y in world.cube.path]
        pygame.draw.lines(screen, PATH_COLOR, False, pts, 3)

    # 轨迹
    for trail, color in ((world.person.trail, TRAIL_P),
                         (world.cube.trail, TRAIL_C)):
        if len(trail) > 1:
            pygame.draw.lines(screen, color, False,
                              [(x / config.SCALE, y / config.SCALE)
                               for x, y in trail], 2)

    # 跟随目标点 + 30~40cm 区间
    tx, ty = world.person.follow_target()
    tx, ty = tx / config.SCALE, ty / config.SCALE
    pygame.draw.circle(screen, TARGET_COLOR, (tx, ty), 5)
    pygame.draw.circle(screen, TARGET_COLOR, (tx, ty), 4, 1)

    draw_person(screen, world.person)
    draw_cube(screen, world.cube)

    # ---------- HUD ----------
    gap = world.cube.surface_gap(world.person)
    clr = min_clearance(world.cube.x, world.cube.y, world.obstacles)
    p_mode = "自动沿跑道跑" if world.person.mode == "auto" else "我用手操"
    left = [
        "小木块追人 · 操场跑道仿真",
        f"人: {p_mode}  5.0 m/s  第 {world.person.lap + 1} 圈",
        f"身后间距 {gap:6.1f} cm  (目标 30~40)",
        f"木块速度 {world.cube.v / 100:4.2f} m/s  (上限 5.00)",
        f"障碍间隙 {clr:6.1f} cm  (要求 >= 50)",
        f"全程最小 {world.min_clearance_seen:6.1f} cm  违规 {world.violation_frames} 帧",
    ]
    right = [
        f"重规划 {world.replan_count} 次  人撞障碍 {world.person_bumps} 帧  倍率 {rate}x",
        "M 切换人 自动/手动   WASD 手动控制人(速度固定)",
        "鼠标左键拖障碍   1~6 选中 + IJKL 移动障碍",
        "P 障碍自动巡逻   [ ] 调播放倍率",
        "空格 暂停   R 重置   ESC 退出",
        "人/障碍 50cm 宽 · 跑道 100cm 宽 · 木块 30cm",
    ]
    # 顶部横向双栏：跑道最高处约 y=150，面板到 y=148 为止，不遮挡跑道
    panel = pygame.Surface((1040, 140), pygame.SRCALPHA)
    panel.fill((252, 253, 250, 205))
    screen.blit(panel, (10, 8))
    for i, text in enumerate(left):
        screen.blit((font if i == 0 else small).render(text, True, TEXT),
                    (22, 14 + i * 21))
    for i, text in enumerate(right):
        screen.blit(small.render(text, True, TEXT), (510, 14 + i * 21))

    if world.cube.danger:
        screen.blit(font.render("! 间隙不足 50cm", True, (200, 60, 50)), (22, 152))
    if paused:
        screen.blit(font.render("已暂停", True, (200, 60, 50)),
                    (config.WINDOW_W - 130, 158))
    if patrol:
        screen.blit(font.render("障碍巡逻中", True, (200, 120, 40)),
                    (config.WINDOW_W - 160, 185))


def main() -> None:
    pygame.init()
    screen = pygame.display.set_mode((config.WINDOW_W, config.WINDOW_H))
    pygame.display.set_caption("luyaCar v0.3 · 小木块操场跑道追人（5m/s）")
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
                elif e.key == pygame.K_LEFTBRACKET:
                    rate_idx = max(0, rate_idx - 1)
                elif e.key == pygame.K_RIGHTBRACKET:
                    rate_idx = min(len(rates) - 1, rate_idx + 1)
                elif pygame.K_1 <= e.key <= pygame.K_6:
                    idx = e.key - pygame.K_1
                    for i, o in enumerate(world.obstacles):
                        o.selected = (i == idx)
            if e.type == pygame.MOUSEBUTTONDOWN and e.button == 1:
                mx, my = e.pos[0] * config.SCALE, e.pos[1] * config.SCALE
                hits = [(math.hypot(o.x - mx, o.y - my), o)
                        for o in world.obstacles]
                if hits and min(h[0] for h in hits) < config.OBSTACLE_RADIUS * 2.2:
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
