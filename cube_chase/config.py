"""config.py —— 全部参数集中在这里（v0.4：真实机器狗跑步跟随场景）。

单位约定
--------
长度一律用 **厘米(cm)**，速度用 **cm/s**（500 cm/s = 5 m/s）。
只在渲染时除以 SCALE 换算成像素：1 像素 = 1.5 cm。

v0.4 场景对齐说明
-----------------
之前是"30cm 小木块贴着人 35cm 跑"的抽象玩具。真实产品是**四足机器狗
跟随跑者**，必须按机器人本体尺寸、制动距离、行人安全距离重设参数：

* 机器狗 70×35×40cm（Unitree Go2 / B2 量级），不是 30cm 方块；
* 跟随间距由"制动安全距"反推，不再是随手写的 30~40cm：
      d_min = v·t_reaction + v²/(2·a_brake)
           = 500×0.15 + 500²/(2×1200) ≈ 75 + 104 = 179 cm
  取整设目标 200cm、允许 150~250cm，给转向和感知噪声留余量；
* 与行人/障碍的横向安全距 80cm（摆臂 30cm + 狗身半宽 18cm + 动态余量）；
* 跑道宽 240cm：100cm 宽时狗(35)+人(50)+安全距(80) 根本无法错身，
  物理上不成立，改为标准健身跑道的两条分道宽。
"""

import math

# ---------- 屏幕与比例 ----------
SCALE = 1.5                 # 1 像素 = 1.5 cm
WINDOW_W, WINDOW_H = 1280, 860
WORLD_W = WINDOW_W * SCALE  # 1920 cm = 19.2 m
WORLD_H = WINDOW_H * SCALE  # 1290 cm = 12.9 m
FPS = 60

# ---------- 人（被跟随的跑者） ----------
PERSON_SPEED = 500.0        # 5 m/s 快跑（沿用需求）
PERSON_RADIUS = 25.0        # 肩宽 50cm → 半径 25cm
PERSON_HEIGHT = 170.0       # 身高（俯视图仅用于说明/绘制标注）
PERSON_LAT_RATE = 260.0     # 自动避障时横向速度上限（cm/s）
                            # 合速率恒定 5m/s：横移越快，沿跑道前进越慢

# ---------- 机器狗（四足，本体尺寸） ----------
DOG_LENGTH = 70.0           # 体长（含头尾）cm
DOG_WIDTH = 35.0            # 体宽 cm
DOG_HEIGHT = 40.0           # 站立高 cm
DOG_RADIUS = DOG_WIDTH / 2.0                    # 胶囊体端点半径 17.5cm
DOG_HALF_AXIS = (DOG_LENGTH - DOG_WIDTH) / 2.0  # 胶囊中轴半长 17.5cm
# 规划用保守外接圆：狗原地转向时扫掠半径 = 体长一半
DOG_PLAN_RADIUS = DOG_LENGTH / 2.0              # 35cm

DOG_CRUISE_SPEED = PERSON_SPEED                 # 稳态巡航严格不超人速
DOG_RECOVERY_MARGIN = 0.15  # 掉队恢复队形时允许 +15% 速度余量。
                            # ⚠️ 设 0 就退回 v0.3「绝对不超人速」规则，
                            #    但那样任何一次掉队都永久不可恢复（同速追同速），
                            #    真实机器人必须有余量，详见优化说明文档。
DOG_MAX_SPEED = DOG_CRUISE_SPEED * (1.0 + DOG_RECOVERY_MARGIN)  # 575 cm/s
DOG_MAX_OMEGA_LOW = 3.0     # 低速时最大偏航角速度（rad/s）
DOG_MAX_OMEGA_HIGH = 1.2    # 5m/s 高速时最大偏航角速度：跑起来不能急打方向
DOG_ACCEL = 1200.0          # 加速度上限 12 m/s²
DOG_DECEL = 1200.0          # 制动减速度 12 m/s²（用于安全距计算）
DOG_REVERSE_SPEED = 120.0   # 被逼到死角时的倒车速度上限

# ---------- 跟随间距（由制动距离反推） ----------
PERCEPTION_LATENCY = 0.15   # 感知+控制延迟（s）
FOLLOW_GAP = 200.0          # 目标：狗前缘到人后背 200cm（2m）
FOLLOW_GAP_MIN = 150.0
FOLLOW_GAP_MAX = 250.0
FOLLOW_HARD_MIN = 110.0     # 低于此值触发紧急制动（绝不撞主人）
OWNER_HARD_GAP = 45.0       # 与主人的**实体表面间距**硬下限（cm）。
                            # 主人必须作为碰撞体进入位移投影层：否则人转身
                            # 折返、或手动操控时迎面跑向狗，仍可能穿模
# 中心距 = 人半径 + 狗规划半径 + 表面间距
FOLLOW_DIST = PERSON_RADIUS + DOG_PLAN_RADIUS + FOLLOW_GAP   # 260 cm
FOLLOW_BIAS = 0.0           # 跟随点标定偏置（cm），按回归结果调整

# ---------- 障碍 / 其他行人 ----------
CONE_RADIUS = 18.0          # 路锥、路桩半径（比人小）
PEDESTRIAN_RADIUS = PERSON_RADIUS   # 其他行人与跑者同尺寸 50cm 宽
PEDESTRIAN_SPEED = 150.0    # 其他行人自主速度 1.5 m/s（走路/慢跑）
OBSTACLE_MAX_SPEED = PERSON_SPEED    # 拖动/键盘控制上限仍与人一致（5m/s）
OBSTACLE_COUNT = 6          # 静态路锥数量
PEDESTRIAN_COUNT = 4        # 动态行人数量

# ---------- 安全间距 ----------
# 80cm 在 2.4m 跑道上过于保守：行人半径 25 + 狗规划半径 35 + 80 = 140cm
# 中心距，两名横穿行人就能把整条跑道封死，机器狗会无处可走而冻结。
# 取原需求区间 50~70cm 的中值 60cm，实际中心距 25+35+60=120cm，
# 加上狗身真实半宽后约 1m，符合真实人机错身距离，也留得出内侧通道。
SAFE_CLEARANCE = 60.0       # 狗与任何障碍/行人的表面间距硬下限（cm）
PREFER_CLEARANCE = 90.0     # 期望保持区间 60~90cm
REPEL_RANGE = 90.0          # 排斥转向作用范围（表面间距，cm）

# ---------- 跑道（标准健身跑道，两条分道宽） ----------
TRACK_WIDTH = 240.0
TRACK_HALF = TRACK_WIDTH / 2.0
STRAIGHT_LEN = 760.0        # 直线段 7.6m
CURVE_RADIUS = 320.0        # 弯道中心线半径 3.2m
# 一圈 = 2×760 + 2π×320 ≈ 35.3 m，5m/s 跑完约 7 秒

# ---------- 路径规划 ----------
GRID_CELL = 12.0            # 栅格边长（cm）
REPLAN_INTERVAL = 0.15      # 重规划周期（s），动态行人需要更快的刷新
PREDICT_HORIZON = 1.2       # 预测时长（s）：v0.3 的 0.35s 在 5m/s 下
                            # 只有 1.75m，横穿行人根本来不及避
PREDICT_SAMPLES = 3         # 沿运动方向采样 3 个时刻，形成"扫掠体"禁区
PREDICT_MAX_DIST = 300.0    # 单次预测最大前伸距离（cm），防止拖行时过度膨胀
GUIDE_WEIGHT = 3.5          # 偏离跑道的代价强度
GUIDE_FREE = 100.0          # 距中心线多少 cm 内不惩罚（跑道半宽 120）
GUIDE_OUTER_BIAS = 0.0
TRACKING_MARGIN = 25.0      # 规划膨胀里额外留的跟踪余量（cm）
SAFETY_FILTER_BAND = 8.0    # 位移投影层：间隙低于 (80+BAND) 开始起作用

# ---------- 控制器 ----------
KP_SPEED = 3.0              # 弧长误差 → 速度增益
LOOKAHEAD_RATIO = 0.30      # 前视距离 = 速度 × 比例
LOOKAHEAD_MIN = 40.0
LOOKAHEAD_MAX = 140.0       # 狗体变大、跟随距离变远，前视也要相应放大
TARGET_LEAD = 1.0           # 跟随点在两次重规划之间按人速前滑的比例
TURN_GAIN = 1.2             # 纯追踪转向增益
DODGE_WEIGHT = 1.0          # 障碍排斥对转向的影响权重


def braking_distance(v: float | None = None) -> float:
    """当前速度下的最小安全跟车距离（cm），HUD 与文档共用同一个公式。"""
    speed = DOG_CRUISE_SPEED if v is None else v
    return speed * PERCEPTION_LATENCY + speed ** 2 / (2.0 * DOG_DECEL)
