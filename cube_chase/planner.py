"""planner.py —— 动态栅格 A* 路径规划（v0.4：面向真实行人的预测避碰）。

设计要点
--------
1. **按对象实际半径膨胀**：路锥 18cm / 行人 25cm 分别算，加上机器狗的
   规划半径 35cm、安全距 80cm、跟踪余量 25cm，构成硬禁区。
2. **多点轨迹预测**：v0.3 只预测 0.35s 后的一个点，5m/s 下视野不足 2m，
   横穿行人根本来不及避。本版沿速度方向采样 3 个时刻（共 1.2s）形成
   "扫掠体"。⚠️ 预测区标成**软代价而不是硬禁区**：否则一条 4~5m 长的
   膨胀带会把 2.4m 宽的跑道整个封死，A* 会直接判定无解、狗原地停车。
3. **弱引导代价**：偏离中心线越远略贵，但允许弯道抄近道走弦 —— 机器人
   巡航不能超人速，靠平滑最短路才能保持队形。
4. 每 0.15 秒重规划一次；行人一直在动，路径必须持续刷新。
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
        # 机器狗就不再是"跟在人身后"，而是跑到场地另一头（实测踩过）。
        # 跑道中心线两侧 100cm 内保持 1.0，允许狗贴内线小幅切弯；
        # 再往外代价最高 4.5 倍，基本杜绝横穿内场。
        self.guide: list[list[float]] = [[1.0] * self.w for _ in range(self.h)]
        for cy in range(self.h):
            wy = (cy + 0.5) * config.GRID_CELL
            for cx in range(self.w):
                wx = (cx + 0.5) * config.GRID_CELL
                _, lat = track.frame(wx, wy)
                d = abs(lat)
                # 跑道及两侧 GUIDE_FREE 内不罚 → 狗可以贴内线走弦抄近道；
                # 再往场芯深处走才迅速变贵 → 又不会横穿整个操场
                base = 1.0 + config.GUIDE_WEIGHT * min(
                    1.0, (max(0.0, d - config.GUIDE_FREE) / 140.0) ** 2)
                # 障碍/行人都从外侧探入，外侧可略微加价把路径推向内侧
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
        """按对象当前位置 + 未来扫掠轨迹重建禁区与代价（只扫局部，够快）。"""
        self.blocked.clear()
        self.extra.clear()

        for o in obstacles:
            # ① 当前位置：硬禁区（对象半径按实际值，不再统一写死 25cm）
            r_hard = (o.radius + config.DOG_PLAN_RADIUS
                      + config.SAFE_CLEARANCE + config.TRACKING_MARGIN)
            self._stamp(o.x, o.y, r_hard, hard=True)

            # ② 未来扫掠：只加软代价，越远的时刻权重越低
            speed = math.hypot(o.vx, o.vy)
            if speed > 1.0:
                if hasattr(o, "predicted_positions"):
                    preds = o.predicted_positions()
                else:
                    preds = []
                    for i in range(1, config.PREDICT_SAMPLES + 1):
                        h = config.PREDICT_HORIZON * i / config.PREDICT_SAMPLES
                        preds.append((o.x + o.vx * h, o.y + o.vy * h))
                n = len(preds)
                for k, (px, py) in enumerate(preds):
                    # 预测半径随距离收缩，避免通道被完全封死
                    shrink = 1.0 - 0.25 * (k + 1) / max(1, n)
                    weight = 2.2 * (1.0 - 0.6 * k / max(1, n))
                    self._stamp(px, py, r_hard * shrink,
                                hard=False, inner=0.0, weight=weight)

            # ③ 安全线外一圈的软排斥：让路径走"缝隙中间"而不是贴边蹭
            self._stamp(o.x, o.y, r_hard + 40.0,
                        hard=False, inner=r_hard)

    def _stamp(self, ox: float, oy: float, radius: float, hard: bool,
               inner: float = 0.0, weight: float = 1.5) -> None:
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
                    self.extra[c] = self.extra.get(c, 0.0) + weight * w


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
