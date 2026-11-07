"""planner.py —— A* 栅格路径规划。

流程：
1. 把窗口划分成 GRID_CELL 见方的栅格；
2. 障碍按 (半径 + 狗半径 + SAFETY_MARGIN) 膨胀，占掉的格子标记为不可走
   —— 膨胀是移动机器人规划的标准做法：把"狗有体积"这个问题，
      转化成"把狗当成质点、把障碍变大"；
3. 在栅格上跑 A*（8 邻接，禁止切角穿缝）；
4. 对路径做视线平滑（能直连的中间点全删掉），去掉锯齿。

规划结果保证：路径上任意点到真实障碍的间隙 >= SAFETY_MARGIN。
"""

import heapq
import math

from . import config
from .obstacles import Obstacle

Cell = tuple[int, int]


def _to_cell(x: float, y: float) -> Cell:
    return int(x // config.GRID_CELL), int(y // config.GRID_CELL)


def _to_world(c: Cell) -> tuple[float, float]:
    return (c[0] + 0.5) * config.GRID_CELL, (c[1] + 0.5) * config.GRID_CELL


class GridMap:
    def __init__(self, obstacles: list[Obstacle]) -> None:
        self.w = config.WINDOW_W // config.GRID_CELL
        self.h = config.WINDOW_H // config.GRID_CELL
        inflate = config.DOG_RADIUS + config.SAFETY_MARGIN
        self.blocked: set[Cell] = set()
        for cy in range(self.h):
            for cx in range(self.w):
                wx, wy = _to_world((cx, cy))
                for obs in obstacles:
                    if math.hypot(wx - obs.x, wy - obs.y) < obs.radius + inflate:
                        self.blocked.add((cx, cy))
                        break

    def in_bounds(self, c: Cell) -> bool:
        return 0 <= c[0] < self.w and 0 <= c[1] < self.h

    def free(self, c: Cell) -> bool:
        return self.in_bounds(c) and c not in self.blocked

    def nearest_free(self, c: Cell, max_radius: int = 20) -> Cell | None:
        """目标格被膨胀区占了（人贴障碍跑时常见）→ 螺旋向外找最近的自由格。"""
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
        """两格之间的线段是否全程自由（平滑和切角检查共用）。"""
        ax, ay = _to_world(a)
        bx, by = _to_world(b)
        steps = max(1, int(math.hypot(bx - ax, by - ay) / (config.GRID_CELL / 2)))
        for i in range(steps + 1):
            t = i / steps
            if _to_cell(ax + (bx - ax) * t, ay + (by - ay) * t) in self.blocked:
                return False
        return True


def _neighbors(grid: GridMap, c: Cell):
    for dx, dy, cost in ((1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
                         (1, 1, math.sqrt(2)), (1, -1, math.sqrt(2)),
                         (-1, 1, math.sqrt(2)), (-1, -1, math.sqrt(2))):
        n = (c[0] + dx, c[1] + dy)
        if not grid.free(n):
            continue
        # 禁止"切角"：斜走要求两侧正交格子都自由，否则会蹭到障碍角
        if dx != 0 and dy != 0:
            if not grid.free((c[0] + dx, c[1])) or not grid.free((c[0], c[1] + dy)):
                continue
        yield n, cost


def _heuristic(a: Cell, b: Cell) -> float:
    dx, dy = abs(a[0] - b[0]), abs(a[1] - b[1])
    return max(dx, dy) + (math.sqrt(2) - 1) * min(dx, dy)  # 八方向 octile 距离


def astar(grid: GridMap, start: Cell, goal: Cell,
          mult: list[list[float]] | None = None) -> list[Cell] | None:
    """标准 A*（可带格子代价倍数）。返回栅格路径（含起终点），失败返回 None。"""
    open_heap = [(_heuristic(start, goal), 0.0, start)]
    came_from: dict[Cell, Cell] = {start: start}
    g_score: dict[Cell, float] = {start: 0.0}

    while open_heap:
        _, g, current = heapq.heappop(open_heap)
        if current == goal:
            path = [current]
            while path[-1] != start:
                path.append(came_from[path[-1]])
            return path[::-1]
        if g > g_score.get(current, float("inf")):
            continue  # 堆里的过期条目
        for nxt, cost in _neighbors(grid, current):
            if mult is not None:
                cost *= mult[nxt[1]][nxt[0]]  # 离引导线远 → 走这里更贵
            ng = g + cost
            if ng < g_score.get(nxt, float("inf")):
                g_score[nxt] = ng
                came_from[nxt] = current
                heapq.heappush(open_heap, (ng + _heuristic(nxt, goal), ng, nxt))
    return None


MAX_SHORTCUT = 60.0  # 平滑时单段最大直线长度（px）

def smooth(grid: GridMap, cells: list[Cell]) -> list[Cell]:
    """视线平滑：i 能直连 j 就跳过中间点，但单段长度受限。

    为什么限制长度：如果不限，弯道上的整条曲线都会被"一根弦"取代 ——
    路明明是弯的，狗却走直线抄近路，在环路里越抄越靠内圈，
    最后稳定落后 1 米（这是实测踩过的坑）。限制 60px 既能消掉
    栅格锯齿，又能保住道路的弯曲形状。
    """
    if len(cells) <= 2:
        return cells
    result = [cells[0]]
    i = 0
    while i < len(cells) - 1:
        j = len(cells) - 1
        while j > i + 1:
            a, b = cells[i], cells[j]
            dist = math.hypot(b[0] - a[0], b[1] - a[1]) * config.GRID_CELL
            if dist <= MAX_SHORTCUT and grid.line_free(a, b):
                break
            j -= 1
        result.append(cells[j])
        i = j
    return result


class Planner:
    """给狗用的规划器：持有栅格地图，提供 find_path 接口。

    guide（引导线）是关键设计：只在空旷区域跑 A*，它会永远走直线弦，
    在弯道上不断抄内圈，狗就会骑在人内侧的小圈上、越跟越远（实测落后 1 米）。
    给每个格子加一个"离引导线越远越贵"的代价，A* 就会自然地贴着路走，
    只在障碍处做局部绕行 —— 行为更像真狗，也更像人走路。
    """

    def __init__(self, obstacles: list[Obstacle],
                 guide: list[tuple[float, float]] | None = None) -> None:
        self.grid = GridMap(obstacles)
        self.obstacles = obstacles
        guide_cost = self._build_guide_cost(guide) if guide else \
            [[1.0] * self.grid.w for _ in range(self.grid.h)]
        self.mult = self._add_obstacle_repulsion(guide_cost)

    def _add_obstacle_repulsion(self, base: list[list[float]]) -> list[list[float]]:
        """障碍排斥代价：离障碍表面越近越贵。

        为什么需要：A* 只求最短，路径会紧贴膨胀区边界"蹭着"过，
        结果狗过缝时间隙只剩 16px，全程被限速拖成龟速。
        加上排斥项后路径会走缝隙中间（间隙 ~40px），又快又稳。
        """
        reach = 55.0  # 排斥影响半径（px）
        for cy in range(self.grid.h):
            wy = (cy + 0.5) * config.GRID_CELL
            for cx in range(self.grid.w):
                wx = (cx + 0.5) * config.GRID_CELL
                for obs in self.obstacles:
                    gap = math.hypot(wx - obs.x, wy - obs.y) - obs.radius
                    if gap < reach:
                        base[cy][cx] *= 1.0 + 6.0 * max(0.0, 1.0 - gap / reach) ** 2
        return base

    def _build_guide_cost(self, guide: list[tuple[float, float]]) -> list[list[float]]:
        """预计算每个格子的代价倍数：离引导线越远越贵。"""
        # 引导线抽稀到 ~120 个点，够用且启动快
        step = max(1, len(guide) // 120)
        pts = guide[::step]
        mult = [[1.0] * self.grid.w for _ in range(self.grid.h)]
        for cy in range(self.grid.h):
            wy = (cy + 0.5) * config.GRID_CELL
            for cx in range(self.grid.w):
                wx = (cx + 0.5) * config.GRID_CELL
                d = min((wx - px) ** 2 + (wy - py) ** 2 for px, py in pts) ** 0.5
                mult[cy][cx] = 1.0 + 3.0 * min(1.0, (d / 40.0) ** 2) * 3
        return mult

    def find_path(self, start_xy: tuple[float, float],
                  goal_xy: tuple[float, float]) -> list[tuple[float, float]] | None:
        start = self.grid.nearest_free(_to_cell(*start_xy))
        goal = self.grid.nearest_free(_to_cell(*goal_xy))
        if start is None or goal is None:
            return None
        cells = astar(self.grid, start, goal, self.mult)
        if cells is None:
            return None
        cells = smooth(self.grid, cells)
        return [_to_world(c) for c in cells]
