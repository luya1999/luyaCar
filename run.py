#!/usr/bin/env python3
"""luyaCar 仿真启动入口。

默认启动 v0.4：四足机器狗在操场跑道上跟随跑者，避开静态障碍与其他行人。
    python run.py

回看上一版机器狗蜿蜒路仿真（v0.2）：
    python run.py --v02

v0.3（抽象小木块版）已原地升级为 v0.4，需要对照可读 git 历史 3052e64。
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
