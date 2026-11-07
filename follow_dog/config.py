"""config.py —— 所有可调参数集中在这里。

单位约定：1 像素(px) = 1 厘米(cm)。所以"跟人 50cm"就是 50 px。
"""

# ---------- 窗口与仿真 ----------
WINDOW_W = 1200
WINDOW_H = 800
FPS = 60

# ---------- 跟随目标 ----------
FOLLOW_DISTANCE = 50.0      # 目标跟随距离：人身后 50cm（这个点就是狗该占的位置）
STOP_GAP = 6.0              # 距该目标点小于 6cm 就停车等待，防止在人脚边来回抖

# ---------- 人 ----------
PERSON_SPEED = 180.0        # 人跑步速度（px/s ≈ 1.8 m/s，慢跑）
PERSON_RADIUS = 10.0

# ---------- 机器狗 ----------
DOG_LENGTH = 34.0           # 机身长（四足机器狗比轮式小车长一点）
DOG_WIDTH = 20.0
DOG_RADIUS = 14.0           # 碰撞模型：用包住机身的等效圆做碰撞检测
DOG_MAX_SPEED = 320.0       # 最大速度，必须 > 人速，否则绕障后追不上
DOG_MAX_OMEGA = 2.8         # 最大角速度（rad/s）
DOG_ACCEL = 900.0           # 线加速度上限（也是刹车减速度，刹不住会撞）

# ---------- 路径规划（A*） ----------
GRID_CELL = 10              # 栅格边长（px）。越小路径越精细、规划越慢
SAFETY_MARGIN = 20.0        # 障碍膨胀余量：必须覆盖栅格量化误差（格点半对角 ~7px）+ 跟踪抖动
REPLAN_INTERVAL = 0.4       # 每 0.4 秒重新规划一次（人是动的，目标点一直在变）
REPLAN_MOVE_THRESH = 25.0   # 或者人累计移动超过 25px 也立即重规划

# ---------- 纯追踪控制器 ----------
# 前视距离随速度自适应：LOOKAHEAD = clamp(v * RATIO, MIN, MAX)
# 高速看远（行驶平顺不画龙），低速看近（贴障碍蠕行时死死咬住路径不切弯）
LOOKAHEAD_RATIO = 0.25
LOOKAHEAD_MIN = 12.0
LOOKAHEAD_MAX = 30.0
TURN_GAIN = 2.0             # 纯追踪转向增益：omega = GAIN * v * sin(err) / LOOKAHEAD
KP_SPEED = 3.5              # 速度比例增益

# ---------- 障碍近旁限速 ----------
SLOWDOWN_CLEARANCE = 45.0   # 间隙小于该值开始限速（px）。要给刹车留距离：
                            # 190px/s 以 900px/s² 减速需要 ~20px，再留跟踪余量
SLOWDOWN_MIN_SPEED = 50.0   # 贴障碍通过时的最低速度（px/s）
SLOWDOWN_GAIN = 10.0        # 限速斜率：v_cap = GAIN * (clearance - 5)

# ---------- 紧急保护 ----------
EMERGENCY_CLEARANCE = 1.0   # 与障碍间隙小于 1px 时进入蠕行脱困（不能彻底冻结，否则死锁）
