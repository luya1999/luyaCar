"""config.py —— 全部参数集中在这里。

单位约定：
    长度一律用 **厘米(cm)**，速度用 **cm/s**（500 cm/s = 5 m/s）。
    只在渲染时除以 SCALE 换算成像素：1 像素 = 1.5 cm。

这样代码里写的 35、50、100 都直接是"35cm、50cm、1m"，
看代码就等于看需求，不会出现"到底这个数是像素还是厘米"的困惑。
"""

import math

# ---------- 屏幕与比例 ----------
SCALE = 1.5                 # 1 像素 = 1.5 cm
WINDOW_W, WINDOW_H = 1280, 860
WORLD_W = WINDOW_W * SCALE  # 1920 cm = 19.2 m
WORLD_H = WINDOW_H * SCALE  # 1290 cm = 12.9 m
FPS = 60

# ---------- 人（规则1） ----------
PERSON_SPEED = 500.0        # 5 m/s，固定速度
PERSON_RADIUS = 25.0        # 宽 50cm → 半径 25cm
PERSON_HEIGHT = 170.0       # 高 1.7m（仅用于绘制/显示）
PERSON_LAT_RATE = 260.0     # 自动避障时横向移动速度上限（cm/s）
                            # 合速率恒定 5m/s：横移越快，沿跑道前进越慢

# ---------- 小木块（规则1） ----------
CUBE_SIZE = 30.0
CUBE_RADIUS = CUBE_SIZE * math.sqrt(2.0) / 2.0   # 外接圆 21.2cm，碰撞按这个算偏保守
CUBE_MAX_SPEED = PERSON_SPEED                    # 规则3：绝不超过人速
CUBE_MAX_OMEGA = 6.0        # 最大角速度（rad/s）
CUBE_ACCEL = 2500.0         # 线加速度上限
CUBE_REVERSE_SPEED = 180.0  # 被逼到死角时可以倒车的最大速度

# ---------- 跟随间距（规则2） ----------
FOLLOW_GAP = 35.0           # 目标：木块左右表面到人后背 35cm
FOLLOW_GAP_MIN = 30.0
FOLLOW_GAP_MAX = 40.0
# 木块中心到人中心的距离 = 人半径 + 木块半径 + 表面间距
FOLLOW_DIST = PERSON_RADIUS + CUBE_RADIUS + FOLLOW_GAP   # ≈ 81.2 cm
FOLLOW_BIAS = 30.0          # 跟随点额外后退量（cm）：标定控制器系统性超前用
                            # 扫参结论：30 时中位 38cm、P10 22、P90 50

# ---------- 障碍物（规则7：与人同大小、同速度） ----------
OBSTACLE_RADIUS = PERSON_RADIUS
OBSTACLE_MAX_SPEED = PERSON_SPEED
OBSTACLE_COUNT = 6

# ---------- 安全间距（规则5） ----------
SAFE_CLEARANCE = 50.0       # 木块与障碍表面间距硬下限（cm）
PREFER_CLEARANCE = 70.0     # 期望保持在 50~70cm 区间
REPEL_RANGE = 68.0          # 排斥力作用范围（表面间距，cm）。
                            # 只在逼近 50cm 硬下限时才启动。设太大（100+）会
                            # 让排斥力全程开启，前视点被推歪、转向饱和打转

# ---------- 跑道（规则1：宽 100cm，操场形状） ----------
TRACK_WIDTH = 100.0
TRACK_HALF = TRACK_WIDTH / 2.0
STRAIGHT_LEN = 760.0        # 直线段长 7.6m
CURVE_RADIUS = 320.0        # 弯道半径 3.2m（中心线）
# 一圈 = 2×760 + 2π×320 ≈ 35.3 m，5m/s 跑完约 7 秒

# ---------- 路径规划 ----------
GRID_CELL = 12.0            # 栅格边长（cm）
REPLAN_INTERVAL = 0.18      # 重规划周期（秒）
PREDICT_HORIZON = 0.35      # 预测障碍未来位置的时间（秒）
GUIDE_WEIGHT = 3.5          # 偏离跑道的代价强度：0=纯最短路（允许切内场抄近道）
GUIDE_FREE = 70.0           # 距中心线多少 cm 内不惩罚
GUIDE_OUTER_BIAS = 0.0      # 外侧额外加价（内侧偏好），0=关闭
SAFETY_FILTER_BAND = 6.0    # 安全过滤层：间隙低于 (50+BAND) 就开始侧推
SAFETY_FILTER_GAIN = 0.9    # 侧推强度（每帧修正多少比例的缺口）
TRACKING_MARGIN = 30.0      # 规划膨胀里额外留的跟踪余量（cm）

# ---------- 木块控制器 ----------
KP_SPEED = 3.0              # 弧长误差 → 速度增益
LOOKAHEAD_RATIO = 0.25      # 前视距离 = 速度 × 比例，限幅如下
LOOKAHEAD_MIN = 25.0
LOOKAHEAD_MAX = 80.0
TARGET_LEAD = 1.0           # 跟随点在两次重规划之间按人速前滑的比例
TURN_GAIN = 1.2             # 纯追踪转向增益
DODGE_WEIGHT = 1.0          # 障碍排斥对转向的影响权重
