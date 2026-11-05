"""person.py —— 被跟随的"人"。

第一阶段先用键盘模拟人的移动（真实项目里，人的位置会来自相机/激光雷达的目标检测）。
单位约定：像素 = 厘米，即 1 px 约等于 1 cm，方便把屏幕坐标和真实尺寸对上。
"""

import math


class Person:
    """一个可以在平面上移动的点，代表人。"""

    # 人走路/慢跑的速度（px/s）。慢跑大约 200 cm/s = 2 m/s。
    RUN_SPEED = 200.0

    def __init__(self, x: float, y: float) -> None:
        self.x = x
        self.y = y
        self.speed = self.RUN_SPEED
        # 记录走过的轨迹，用来在界面上画拖尾
        self.trail: list[tuple[float, float]] = [(x, y)]

    def update_from_keys(self, keys, dt: float) -> None:
        """根据键盘输入更新位置。keys 是 pygame.key.get_pressed() 的结果。"""
        import pygame  # 延迟导入，方便无头测试时不用装 SDL 也能 import 本模块

        dx = dy = 0.0
        if keys[pygame.K_LEFT] or keys[pygame.K_a]:
            dx -= 1.0
        if keys[pygame.K_RIGHT] or keys[pygame.K_d]:
            dx += 1.0
        if keys[pygame.K_UP] or keys[pygame.K_w]:
            dy -= 1.0
        if keys[pygame.K_DOWN] or keys[pygame.K_s]:
            dy += 1.0

        # 斜向移动时归一化，避免斜着走比直走快 41%（√2 倍）
        norm = math.hypot(dx, dy)
        if norm > 0:
            self.x += dx / norm * self.speed * dt
            self.y += dy / norm * self.speed * dt
            self._record_trail()

    def move_toward(self, target_x: float, target_y: float, dt: float) -> None:
        """自动朝目标点移动（无头测试 / 自动演示用）。"""
        dx, dy = target_x - self.x, target_y - self.y
        dist = math.hypot(dx, dy)
        if dist > 1.0:
            step = min(self.speed * dt, dist)
            self.x += dx / dist * step
            self.y += dy / dist * step
            self._record_trail()

    def _record_trail(self) -> None:
        self.trail.append((self.x, self.y))
        # 只保留最近 600 个点，防止内存无限增长
        if len(self.trail) > 600:
            self.trail = self.trail[-600:]
