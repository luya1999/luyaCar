#!/usr/bin/env python3
"""luyaCar 仿真启动入口。

默认启动 v0.3：小木块在操场跑道上追人（5m/s、动态障碍、可手动操控）。
    python run.py

想回看上一版机器狗仿真：
    python run.py --v02
"""

import sys


def main() -> None:
    if "--v02" in sys.argv:
        from follow_dog.simulation import main as old_main
        old_main()
    else:
        from cube_chase.simulation import main as new_main
        new_main()


if __name__ == "__main__":
    main()
