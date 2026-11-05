"""simulation.py —— pygame 主循环：渲染 + 键盘输入。

运行方式：python run.py
按键：
    方向键 / WASD —— 移动"人"
    R             —— 重置人和车的位置
    空格          —— 暂停 / 继续
    ESC           —— 退出
"""

import math
import sys

import pygame

from .car import Car
from .person import Person

# ---------- 界面常量 ----------
WINDOW_W, WINDOW_H = 1200, 800
FPS = 60

COLOR_BG = (250, 250, 248)
COLOR_GRID = (230, 230, 226)
COLOR_PERSON = (214, 69, 65)       # 红 = 人
COLOR_PERSON_TRAIL = (240, 190, 188)
COLOR_CAR = (41, 128, 185)         # 蓝 = 小车
COLOR_CAR_TRAIL = (180, 210, 232)
COLOR_CAR_NOSE = (241, 196, 15)    # 黄点标车头方向
COLOR_TEXT = (60, 60, 60)

CAR_LENGTH, CAR_WIDTH = 40, 26     # 小车外形（px）
PERSON_RADIUS = 10


def draw_grid(screen: pygame.Surface) -> None:
    for x in range(0, WINDOW_W, 50):
        pygame.draw.line(screen, COLOR_GRID, (x, 0), (x, WINDOW_H))
    for y in range(0, WINDOW_H, 50):
        pygame.draw.line(screen, COLOR_GRID, (0, y), (WINDOW_W, y))


def draw_trail(screen: pygame.Surface, trail, color) -> None:
    if len(trail) > 1:
        pygame.draw.lines(screen, color, False, trail, 2)


def draw_person(screen: pygame.Surface, person: Person) -> None:
    pos = (int(person.x), int(person.y))
    pygame.draw.circle(screen, COLOR_PERSON, pos, PERSON_RADIUS)
    pygame.draw.circle(screen, (150, 30, 28), pos, PERSON_RADIUS, 2)


def draw_car(screen: pygame.Surface, car: Car) -> None:
    # 车体 = 一个旋转的矩形
    rect = pygame.Surface((CAR_LENGTH, CAR_WIDTH), pygame.SRCALPHA)
    rect.fill(COLOR_CAR)
    # 车头方向画一道杠 + 鼻子上的黄点，转起来才能看出朝向
    pygame.draw.rect(rect, COLOR_CAR_NOSE, (CAR_LENGTH - 8, CAR_WIDTH // 2 - 3, 8, 6))
    rotated = pygame.transform.rotate(rect, -math.degrees(car.theta))
    screen.blit(rotated, rotated.get_rect(center=(car.x, car.y)))


def draw_hud(screen, font, car: Car, person: Person, paused: bool) -> None:
    dist = car.distance_to(person)
    lines = [
        f"距离: {dist:6.1f} px   (目标跟随距离 {car.follow_distance:.0f})",
        f"车速: {car.v:6.1f} px/s   角速度: {car.omega:+.2f} rad/s",
        "方向键/WASD 移动人 | R 重置 | 空格 暂停 | ESC 退出",
    ]
    if paused:
        lines.append("== 已暂停 ==")
    for i, text in enumerate(lines):
        screen.blit(font.render(text, True, COLOR_TEXT), (12, 10 + i * 24))


def reset() -> tuple[Car, Person]:
    person = Person(WINDOW_W * 0.6, WINDOW_H * 0.4)
    car = Car(WINDOW_W * 0.3, WINDOW_H * 0.7, theta=math.pi / 4)
    return car, person


def main() -> None:
    pygame.init()
    screen = pygame.display.set_mode((WINDOW_W, WINDOW_H))
    pygame.display.set_caption("luyaCar · 跟随小车仿真 v0.1")
    clock = pygame.time.Clock()
    font = pygame.font.SysFont("microsoftyahei,simhei,arial", 18)

    car, person = reset()
    paused = False

    while True:
        dt = clock.tick(FPS) / 1000.0  # 本帧真实耗时（秒）

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit(0)
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    pygame.quit()
                    sys.exit(0)
                if event.key == pygame.K_r:
                    car, person = reset()
                if event.key == pygame.K_SPACE:
                    paused = not paused

        if not paused:
            person.update_from_keys(pygame.key.get_pressed(), dt)
            # 不让人跑出窗口
            person.x = max(0, min(WINDOW_W, person.x))
            person.y = max(0, min(WINDOW_H, person.y))
            car.step(person, dt)

        # ---------- 渲染 ----------
        screen.fill(COLOR_BG)
        draw_grid(screen)
        draw_trail(screen, person.trail, COLOR_PERSON_TRAIL)
        draw_trail(screen, car.trail, COLOR_CAR_TRAIL)
        draw_person(screen, person)
        draw_car(screen, car)
        draw_hud(screen, font, car, person, paused)
        pygame.display.flip()
