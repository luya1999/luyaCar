# luyaCar

计划打造一款具有跟随人进行跑步的功能小车，先实现小车跟人行动的功能

## 快速开始

```bash
pip install -r requirements.txt   # 只需 pygame
python run.py
```

弹出仿真窗口后：方向键 / WASD 控制红色的"人"，蓝色小车会自动跟随并保持约 80cm 距离。
`R` 重置，`空格` 暂停，`ESC` 退出。

详细的分步说明（含常见问题）见工作区的《如何运行luyaCar（临时）.md》。

## 项目结构

```
luyaCar/
├── run.py                  # 启动入口
├── requirements.txt
└── follow_car/
    ├── person.py           # 被跟随的"人"（键盘控制，后续替换为相机检测）
    ├── car.py              # 差速小车运动学 + 跟随控制器（核心）
    └── simulation.py       # pygame 渲染与主循环
```

`car.py` 中 `compute_command()` 输出的 `(v, omega)` 与 ROS 的 `/cmd_vel` 语义一致，
为后续迁移到 ROS 2 / 真车预留了接口。

## 路线图

1. ✅ v0.1 键盘控制人 + 差速小车仿真跟随（当前）
2. 自动跑轨迹的"人"，用于回归测试
3. 控制器升级（PD / Pure Pursuit），消除稳态误差
4. OpenCV 视觉检测真人
5. 迁移 ROS 2（`/cmd_vel` 节点 + Gazebo/Isaac 仿真）
6. 真车落地
