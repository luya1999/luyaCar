"""planner.py —— 动态栅格 A* 路径规划（规则3 + 规则5）。

设计要点
--------
1. **障碍膨胀 = 硬约束**：把障碍半径 + 木块半径 + 50cm 安全距 + 跟踪余量
   作为禁区。这样规划出的路径中心天然满足"离障碍至少 50cm"，不靠运气。
2. **预测障碍未来位置**（规则7 障碍会动）：把"障碍 0.35 秒后的位置"也标成
   软禁区，木块会提前往安全侧让，而不是等障碍贴脸才反应。
3. **弱引导代价**：离跑道中心线越远略微越贵，但权重很低 —— 木块速度不能
   超过人，必须允许它在弯道**抄近道走弦**才追得上（规则3 的核心）。
4. 每 0.18 秒重规划一次；障碍/目标一直在动，路径必须跟着刷新。
"""

import heapq
import math

from . import config
from .obstacles import Obstacle
from .track import Track

Cell = tuple[int, int]

MAX_SHORTCUT = 140.0        # 平滑时单段最大直线长度（cm）


def _to_cell(x: float, y: float) -> Cell:
    return int(x // config.GRID_CELL), int(y // config.GRID_CELL)


def _to_world(c: Cell) -> tuple[float, float]:
    return (c[0] + 0.5) * config.GRID_CELL, (c[1] + 0.5) * config.GRID_CELL


class GridMap:
    def __init__(self, track: Track) -> None:
        self.w = int(config.WORLD_W // config.GRID_CELL)
        self.h = int(config.WORLD_H // config.GRID_CELL)
        self.blocked: set[Cell] = set()
        self.extra: dict[Cell, float] = {}
        # 静态引导代价：跑道内便宜，越往场芯/场外越贵。
        # 权重必须够大，否则 A* 会直接横穿操场中心去"抄近道"，
        # 木块就不再是"跟在人身后"，而是跑到场地另一头（实测踩过）。
        # 跑道边缘 45cm 内保持 1.0，允许木块贴内线小幅切弯；
        # 深入场芯 150cm 以上代价 5 倍，基本杜绝横穿。
        self.guide: list[list[float]] = [[1.0] * self.w for _ in range(self.h)]
        for cy in range(self.h):
            wy = (cy + 0.5) * config.GRID_CELL
            for cx in range(self.w):
                wx = (cx + 0.5) * config.GRID_CELL
                _, lat = track.frame(wx, wy)
                d = abs(lat)
                # 跑道及两侧 70cm 内不罚 → 木块可以贴内线走弦抄近道；
                # 再往场芯深处走才迅速变贵 → 又不会横穿整个操场
                base = 1.0 + config.GUIDE_WEIGHT * min(
                    1.0, (max(0.0, d - config.GUIDE_FREE) / 140.0) ** 2)
                # 障碍都从外侧探入，所以外侧略微加价，让 A* 自然往内侧多让几 cm。
                # 这几 cm 正好把"离障碍 47cm"抬到 50cm 以上（规则5）。
                if lat > 0:
                    base += config.GUIDE_OUTER_BIAS * min(1.0, lat / 80.0)
                self.guide[cy][cx] = base

    def free(self, c: Cell) -> bool:
        return 0 <= c[0] < self.w and 0 <= c[1] < self.h and c not in self.blocked

    def cost_at(self, c: Cell) -> float:
        return self.guide[c[1]][c[0]] + self.extra.get(c, 0.0)

    def nearest_free(self, c: Cell, max_radius: int = 24) -> Cell | None:
        """起点/终点落在膨胀区里 → 螺旋向外找最近自由格。"""
        if self.free(c):
            return c
        for r in range(1, max_radius + 1):
            for dy in range(-r, r + 1):
                for dx in range(-r, r + 1):
                    if max(abs(dx), abs(dy)) != r:
                        continue
                    cand = (c[0] + dx, c[1] + dy)
                    if self.free(cand):
                        return cand
        return None

    def line_free(self, a: Cell, b: Cell) -> bool:
        ax, ay = _to_world(a)
        bx, by = _to_world(b)
        steps = max(1, int(math.hypot(bx - ax, by - ay) / (config.GRID_CELL / 2)))
        for i in range(steps + 1):
            t = i / steps
            if _to_cell(ax + (bx - ax) * t, ay + (by - ay) * t) in self.blocked:
                return False
        return True

    def refresh(self, obstacles: list[Obstacle]) -> None:
        """按障碍当前位置 + 预测位置重建禁区与排斥代价（只扫局部，够快）。"""
        self.blocked.clear()
        self.extra.clear()
        r_hard = (config.OBSTACLE_RADIUS + config.CUBE_RADIUS
                  + config.SAFE_CLEARANCE + config.TRACKING_MARGIN)
        r_pred = r_hard - 12.0
        r_repel = r_hard + 45.0

        for o in obstacles:
            # 当前位置：硬禁区
            self._stamp(o.x, o.y, r_hard, hard=True)
            # 预测位置：障碍有速度时，提前往它要去的地方让
            if math.hypot(o.vx, o.vy) > 1.0:
                px = o.x + o.vx * config.PREDICT_HORIZON
                py = o.y + o.vy * config.PREDICT_HORIZON
                self._stamp(px, py, r_pred, hard=True)
            # 安全线外一小圈的软排斥：让路径走"缝隙中间"而不是贴着安全线蹭
            self._stamp(o.x, o.y, r_repel, hard=False, inner=r_hard)

    def _stamp(self, ox: float, oy: float, radius: float, hard: bool,
               inner: float = 0.0) -> None:
        cell = config.GRID_CELL
        x0 = max(0, int((ox - radius) // cell))
        x1 = min(self.w - 1, int((ox + radius) // cell))
        y0 = max(0, int((oy - radius) // cell))
        y1 = min(self.h - 1, int((oy + radius) // cell))
        r2 = radius * radius
        inner2 = inner * inner
        for cy in range(y0, y1 + 1):
            wy = (cy + 0.5) * cell
            for cx in range(x0, x1 + 1):
                wx = (cx + 0.5) * cell
                d2 = (wx - ox) ** 2 + (wy - oy) ** 2
                if d2 >= r2:
                    continue
                c = (cx, cy)
                if hard:
                    self.blocked.add(c)
                elif d2 > inner2:
                    # 越贴近安全线越贵（线性衰减），把路径推向缝隙中间
                    d = math.sqrt(d2)
                    w = 1.0 - (d - inner) / max(1.0, radius - inner)
                    self.extra[c] = self.extra.get(c, 0.0) + 1.5 * w


def _neighbors(grid: GridMap, c: Cell):
    for dx, dy, cost in ((1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
                         (1, 1, math.sqrt(2)), (1, -1, math.sqrt(2)),
                         (-1, 1, math.sqrt(2)), (-1, -1, math.sqrt(2))):
        n = (c[0] + dx, c[1] + dy)
        if not grid.free(n):
            continue
        if dx and dy:  # 禁止切角穿过障碍角
            if not grid.free((c[0] + dx, c[1])) or not grid.free((c[0], c[1] + dy)):
                continue
        yield n, cost


def _h(a: Cell, b: Cell) -> float:
    dx, dy = abs(a[0] - b[0]), abs(a[1] - b[1])
    return max(dx, dy) + (math.sqrt(2) - 1) * min(dx, dy)


def astar(grid: GridMap, start: Cell, goal: Cell) -> list[Cell] | None:
    heap = [(_h(start, goal), 0.0, start)]
    came: dict[Cell, Cell] = {start: start}
    g: dict[Cell, float] = {start: 0.0}
    while heap:
        _, gcur, cur = heapq.heappop(heap)
        if cur == goal:
            path = [cur]
            while path[-1] != start:
                path.append(came[path[-1]])
            return path[::-1]
        if gcur > g.get(cur, float("inf")):
            continue
        for nxt, base in _neighbors(grid, cur):
            ng = gcur + base * grid.cost_at(nxt)
            if ng < g.get(nxt, float("inf")):
                g[nxt] = ng
                came[nxt] = cur
                heapq.heappush(heap, (ng + _h(nxt, goal), ng, nxt))
    return None


def smooth(grid: GridMap, cells: list[Cell]) -> list[Cell]:
    """视线平滑，但限制单段长度，避免整条弯道被拉成一根弦。"""
    if len(cells) <= 2:
        return cells
    out = [cells[0]]
    i = 0
    while i < len(cells) - 1:
        j = len(cells) - 1
        while j > i + 1:
            a, b = cells[i], cells[j]
            dist = math.hypot(b[0] - a[0], b[1] - a[1]) * config.GRID_CELL
            if dist <= MAX_SHORTCUT and grid.line_free(a, b):
                break
            j -= 1
        out.append(cells[j])
        i = j
    return out


class Planner:
    def __init__(self, track: Track, obstacles: list[Obstacle]) -> None:
        self.grid = GridMap(track)
        self.obstacles_holder = obstacles
        self.grid.refresh(obstacles)

    def find_path(self, start_xy: tuple[float, float],
                  goal_xy: tuple[float, float]) -> list[tuple[float, float]] | None:
        self.grid.refresh(self.obstacles_holder)
        s = self.grid.nearest_free(_to_cell(*start_xy))
        g = self.grid.nearest_free(_to_cell(*goal_xy))
        if s is None or g is None:
            return None
        cells = astar(self.grid, s, g)
        if cells is None:
            return None
        return [_to_world(c) for c in smooth(self.grid, cells)]

    def set_obstacles(self, obstacles: list[Obstacle]) -> None:
        self.obstacles_holder = obstacles
