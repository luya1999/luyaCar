"""luyaCar v0.2 —— 机器狗蜿蜒路跟随仿真包。

模块：
- config.py     所有可调参数（窗口、速度、跟随距离、安全余量……）
- road.py       蜿蜒环形道路（参数曲线 + 弧长索引）
- obstacles.py  静态障碍物（圆形）+ 碰撞/间隙计算
- planner.py    A* 栅格路径规划（含障碍膨胀与路径平滑）
- person.py     被跟随的人（沿道路匀速跑步）
- dog.py        机器狗（差速运动学 + 纯追踪控制器）
- simulation.py pygame 主循环（渲染 + 重规划逻辑）
"""

__version__ = "0.2.0"
