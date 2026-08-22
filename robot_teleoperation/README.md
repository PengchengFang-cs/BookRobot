# Wanda 遥操作运行时覆盖层

本目录是 Wanda 当前正式遥操作所需的完整 ARM64 / CPython 3.10 运行时覆盖层。
将它纳入 Git，是为了让 `scripts/deploy_to_robot.sh` 通过 `git archive HEAD` 部署时能够
完整复现 `/home/unix_ai/fpc/robot_teleoperation`，不再依赖 5090 工作树中的未跟踪文件。

## 运行时入口

Wanda 的 `manipulation.service` 执行：

```text
/home/unix_ai/work/manipulation/start.sh
```

该脚本先设置：

```bash
export PYTHONPATH="/home/unix_ai/fpc/robot_teleoperation:${PYTHONPATH:-}"
```

然后运行 `ros2 run teleoperation teleop`。ROS 入口仍来自 Luca 已安装的软件包，Python
模块则由本目录优先覆盖。

## 正式修改

只读基线是：

```text
/home/unix_ai/luca/ros2_ws/src/teleoperation/teleoperation
```

与该基线比较，当前 47 个运行时文件中唯一有意修改的是：

```text
teleoperation/robot_state_machine.py
```

进入 `teleop`、`teleop_l` 或 `teleop_r` 状态时，它会先读取机器人当前关节状态，把
当前头部和升降柱位置同步到 whole-body 命令缓存，再进入遥操作。这是当前正式遥操作
要求，用于避免遥操作接管瞬间把头部或升降柱拉回旧缓存姿态。

文件摘要：

```text
Luca 基线 robot_state_machine.py:
b8ab6dba9b9795283a36069874b411d6c2a13592daee17f5b465535bc9ba560f

当前覆盖层 robot_state_machine.py:
5f7d278a1c27a85485b7225b5b45f3ee3d411cd734dc184b13f4024299843172
```

## 平台和完整性

- 运行时 payload：47 个文件，约 18 MB；其中 37 个是 ARM64 CPython 3.10 `.so`。
- RTX 5090 是 x86_64，只能保存、审查和部署本目录，不能直接导入这些扩展。
- 2026-08-22 核对时，5090 与 Wanda 的完整 payload 一致。

从本目录复核 payload 摘要：

```bash
find ./teleoperation -type f -print0 \
  | LC_ALL=C sort -z \
  | xargs -0 sha256sum \
  | sha256sum
```

预期结果：

```text
12a41e90262afc86ef4e67af85f4134e6cbfa493928304091c936cac3fd06fb1  -
```

## 部署边界

提交本目录不会自动部署、重启服务或触发机器人动作。正式部署仍必须使用仓库的
`scripts/deploy_to_robot.sh`，并遵守 `AGENTS.md` 的批准要求。部署后，覆盖层要到下一次
机器人开机或经批准重启 `manipulation.service` 时才会被新进程加载；不需要为此单独
重启 `controller.service`。
