"""car.py —— 差速驱动小车 + 跟随控制器。

差速车（两个轮子各自有速度）的运动学是机器人入门的经典模型：
    x' = v * cos(theta)      线速度在 x 方向的分量
    y' = v * sin(theta)
    theta' = omega           角速度
其中 v 由两个轮子的平均速度决定，omega 由两轮速度差决定。

跟随控制器（纯追踪 / 比例控制的极简版）：
1. 算出小车到人的连线方向 → 期望航向角
2. 航向误差越大 → 转得越快（P 控制）；转向大时自动减速，防止"甩尾"
3. 距离越远 → 追得越快；距离 < 目标跟随距离 → 停下
"""

import math

from .person import Person


def wrap_angle(angle: float) -> float:
    """把角度规整到 (-pi, pi]，防止 359° 和 -1° 之间"绕远路"转。"""
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle <= -math.pi:
        angle += 2 * math.pi
    return angle


class Car:
    def __init__(
        self,
        x: float,
        y: float,
        theta: float = 0.0,
        follow_distance: float = 80.0,   # 目标跟随距离（px，约 0.8 m）
        max_speed: float = 260.0,        # 最大线速度（px/s），要比人快才追得上
        max_omega: float = 3.5,          # 最大角速度（rad/s）
        accel: float = 500.0,            # 线加速度限制（px/s^2），让起步/停车平滑
    ) -> None:
        self.x = x
        self.y = y
        self.theta = theta              # 朝向角（弧度），0 = 朝右，逆时针为正
        self.follow_distance = follow_distance
        self.max_speed = max_speed
        self.max_omega = max_omega
        self.accel = accel

        self.v = 0.0        # 当前线速度
        self.omega = 0.0    # 当前角速度
        self.trail: list[tuple[float, float]] = [(x, y)]

    # ---------- 控制器 ----------

    def compute_command(self, target: Person) -> tuple[float, float]:
        """输入目标（人），输出本帧的期望 (v, omega)。真实机器人里这就是 /cmd_vel。"""
        dx = target.x - self.x
        dy = target.y - self.y
        distance = math.hypot(dx, dy)

        # 1) 期望航向 = 指向人的方向
        desired_theta = math.atan2(dy, dx)
        heading_error = wrap_angle(desired_theta - self.theta)

        # 2) 角速度：P 控制，误差大转得快；限幅防止原地陀螺
        kp_omega = 4.0
        omega = max(-self.max_omega, min(self.max_omega, kp_omega * heading_error))

        # 3) 线速度：超出跟随距离的部分按距离比例给速度（也是 P 控制）
        kp_speed = 2.0
        gap = distance - self.follow_distance
        if gap <= 0:
            target_v = 0.0            # 已经够近，停车等待
        else:
            target_v = min(self.max_speed, kp_speed * gap)
            # 转向没对准时减速：cos(误差)，误差 90° 时速度为 0，先转后走
            target_v *= max(0.0, math.cos(heading_error))

        # 4) 加速度限制：模拟电机不能瞬间从 0 到满速
        dv = target_v - self.v
        max_dv = self.accel * (1 / 60)  # 按 60 FPS 估算单帧最大速度变化
        v = self.v + max(-max_dv, min(max_dv, dv))

        return v, omega

    # ---------- 运动学 ----------

    def step(self, target: Person, dt: float) -> None:
        """推进一个仿真步：算指令 → 更新状态 → 记轨迹。"""
        self.v, self.omega = self.compute_command(target)

        # 一阶积分（欧拉法），dt 很小时足够准
        self.theta = wrap_angle(self.theta + self.omega * dt)
        self.x += self.v * math.cos(self.theta) * dt
        self.y += self.v * math.sin(self.theta) * dt

        self.trail.append((self.x, self.y))
        if len(self.trail) > 600:
            self.trail = self.trail[-600:]

    def distance_to(self, target: Person) -> float:
        return math.hypot(target.x - self.x, target.y - self.y)
