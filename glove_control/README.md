# glove_control 兼容子集

本目录存放 `hand_command_arbiter.py`（纯标准库实现）的副本。采集器与测试在
`/home/svt/glove_control` 不存在时，会回退到本目录加载该模块，使仓库在无手套
遥操作环境的机器上也能运行与测试。

完整的按钮流程采集还需要 `/home/svt/glove_control` 中的手套遥操作运行时
（`mixed_glove_teleop`、Wuji / LinkerHand 硬件 SDK 等），这部分依赖真实手套
硬件，不随本仓库分发。在本机（SVT 机器人）上，`/home/svt/glove_control`
始终优先于本目录。
