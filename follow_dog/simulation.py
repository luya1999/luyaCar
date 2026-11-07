"""simulation.py —— pygame 主循环。

演示内容：
- 米色宽线 = 蜿蜒的路；红点 = 沿路跑步的人
- 灰圆（红描边）= 障碍物；绿线 = A* 实时规划的路径
- 蓝色机身 = 机器狗；小黄旗点 = 人身后 50cm 的跟随目标点
- HUD：跟随距离 / 与障碍最小间隙 / 重规划次数 / 碰撞次数

运行：python run.py
按键：R 重置 | 空格 暂停 | ESC 退出
"""

import math
import sys

import pygame

from . import config
from .dog import Dog
from .obstacles import Obstacle, build_obstacles, min_clearance
from .person import Person
from .planner import Planner
from .road import CORRIDOR_HALF_WIDTH, Road

COLOR_BG = (250, 250, 247)
COLOR_ROAD = (238, 230, 214)
COLOR_ROAD_EDGE = (214, 202, 180)
COLOR_PERSON = (214, 69, 65)
COLOR_PERSON_TRAIL = (242, 196, 194)
COLOR_DOG = (41, 128, 185)
COLOR_DOG_TRAIL = (178, 208, 230)
COLOR_NOSE = (241, 196, 15)
COLOR_OBSTACLE = (149, 155, 160)
COLOR_OBSTACLE_EDGE = (192, 57, 43)
COLOR_PATH = (39, 174, 96)
COLOR_TARGET = (142, 68, 173)
COLOR_TEXT = (60, 60, 60)


class World:
    """把"场景 + 重规划逻辑"收在一起，方便测试脚本脱离 pygame 复用。"""

    def __init__(self) -> None:
        self.road = Road()
        self.obstacles = build_obstacles(self.road)
        # 用道路中心线做规划引导线：狗会贴着路走，而不是在弯道上抄直线弦
        self.planner = Planner(self.obstacles, guide=self.road.points)
        self.person = Person(self.road, start_s=0.0)
        self.dog = Dog(*self.road.point_at(-120))   # 狗从人身后 120px 出发
        self.replan_timer = 0.0
        self.replan_count = 0
        self.collision_count = 0
        self.min_clearance_seen = float("inf")
        self._person_since_plan = 0.0
        self._last_person_pos = (self.person.x, self.person.y)
        self._replan()

    def _replan(self) -> None:
        # 终点取"人前方 160px"：让路径一直延伸到人的前面，
        # 狗才始终有前视点可追，不会冲过终点后掉头打转。
        path = self.planner.find_path((self.dog.x, self.dog.y),
                                      self.person.ahead_point(160.0))
        if path:
            self.dog.path = path
            self.replan_count += 1
        self._person_since_plan = 0.0

    def update(self, dt: float) -> None:
        self.person.update(dt)
        self._person_since_plan += math.hypot(
            self.person.x - self._last_person_pos[0],
            self.person.y - self._last_person_pos[1])
        self._last_person_pos = (self.person.x, self.person.y)

        # 定时 / 人跑远了 → 重新规划
        self.replan_timer += dt
        if (self.replan_timer >= config.REPLAN_INTERVAL
                or self._person_since_plan >= config.REPLAN_MOVE_THRESH):
            self.replan_timer = 0.0
            self._replan()

        self.dog.step(self.person, self.obstacles, dt)

        clearance = min_clearance(self.dog.x, self.dog.y, self.obstacles)
        self.min_clearance_seen = min(self.min_clearance_seen, clearance)
        if clearance < 0:
            self.collision_count += 1


# ---------- 渲染 ----------

def draw_road(screen: pygame.Surface, road: Road) -> None:
    pts = road.points + [road.points[0]]
    pygame.draw.lines(screen, COLOR_ROAD, True, pts, int(CORRIDOR_HALF_WIDTH * 2))
    pygame.draw.lines(screen, COLOR_ROAD_EDGE, True, pts, 2)


def draw(screen, font, world: World, paused: bool) -> None:
    screen.fill(COLOR_BG)
    draw_road(screen, world.road)

    for obs in world.obstacles:
        pygame.draw.circle(screen, COLOR_OBSTACLE, (obs.x, obs.y), obs.radius)
        pygame.draw.circle(screen, COLOR_OBSTACLE_EDGE, (obs.x, obs.y), obs.radius, 2)

    # 规划路径（绿）
    if len(world.dog.path) > 1:
        pygame.draw.lines(screen, COLOR_PATH, False, world.dog.path, 2)

    # 轨迹
    for trail, color in ((world.person.trail, COLOR_PERSON_TRAIL),
                         (world.dog.trail, COLOR_DOG_TRAIL)):
        if len(trail) > 1:
            pygame.draw.lines(screen, color, False, trail, 2)

    # 跟随目标点（紫旗）
    tx, ty = world.person.follow_target()
    pygame.draw.circle(screen, COLOR_TARGET, (tx, ty), 5)
    pygame.draw.circle(screen, COLOR_TARGET, (tx, ty),
                       config.FOLLOW_DISTANCE, 1)  # 以目标为心的参考圈

    # 人
    p = world.person
    pygame.draw.circle(screen, COLOR_PERSON, (p.x, p.y), config.PERSON_RADIUS)
    pygame.draw.line(screen, COLOR_PERSON, (p.x, p.y),
                     (p.x + p.dir_x * 18, p.y + p.dir_y * 18), 2)

    # 狗（旋转矩形 + 黄色鼻头）
    dog = world.dog
    body = pygame.Surface((config.DOG_LENGTH, config.DOG_WIDTH), pygame.SRCALPHA)
    body.fill(COLOR_DOG)
    pygame.draw.rect(body, COLOR_NOSE,
                     (config.DOG_LENGTH - 7, config.DOG_WIDTH / 2 - 3, 7, 6))
    rotated = pygame.transform.rotate(body, -math.degrees(dog.theta))
    screen.blit(rotated, rotated.get_rect(center=(dog.x, dog.y)))

    # HUD
    clearance = min_clearance(dog.x, dog.y, world.obstacles)
    lines = [
        f"跟随距离: {dog.distance_to(world.person):5.1f} px (目标 {config.FOLLOW_DISTANCE:.0f})",
        f"障碍间隙: {clearance:5.1f} px   车速: {dog.v:5.1f} px/s",
        f"重规划: {world.replan_count} 次   碰撞: {world.collision_count} 次",
        "R 重置 | 空格 暂停 | ESC 退出",
    ]
    if dog.emergency_stopped:
        lines.append("!! 紧急停车：离障碍太近 !!")
    if paused:
        lines.append("== 已暂停 ==")
    for i, text in enumerate(lines):
        screen.blit(font.render(text, True, COLOR_TEXT), (12, 10 + i * 24))


def main() -> None:
    pygame.init()
    screen = pygame.display.set_mode((config.WINDOW_W, config.WINDOW_H))
    pygame.display.set_caption("luyaCar v0.2 · 机器狗蜿蜒路跟随（A* 避障）")
    clock = pygame.time.Clock()
    font = pygame.font.SysFont("microsoftyahei,simhei,arial", 18)

    world = World()
    paused = False

    while True:
        dt = clock.tick(config.FPS) / 1000.0
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit(0)
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    pygame.quit()
                    sys.exit(0)
                if event.key == pygame.K_r:
                    world = World()
                if event.key == pygame.K_SPACE:
                    paused = not paused

        if not paused:
            world.update(dt)

        draw(screen, font, world, paused)
        pygame.display.flip()
