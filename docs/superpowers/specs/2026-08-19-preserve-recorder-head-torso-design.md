# DataReplay 录制器保持头部与升降柱姿态设计

## 目标

将机器人当前 V3 单段录制器复制到 FPC 权威仓库中，仅移除启动阶段自动把头部俯仰设为 `0.25 rad`、把升降柱设为 `0.200 m` 的行为。录制器启动后保持用户预先调整好的头部和升降柱位置，并继续使用现有 `s/e/w/d` 交互流程。

## 范围

- 从 `/home/unix_ai/DataCollector/DataReplay_v3/recorder` 只读复制当前录制器所需文件到 `/home/cvailab/fpc/datareplay_recorder`。
- 保持相机健康检查、controller 启动、DDS 环境、warmup、传感器选择、保存参数、HDF5 格式和交互按键不变。
- 移除 `pi_collector.py` 中用于发布默认头部与升降柱位置的控制节点、启动调用、五秒动作等待和相应的到位验证。
- 不增加自动姿态检查、人工确认或新命令行参数。
- 不修改机器人参考目录 `/home/unix_ai/DataCollector`。

## 部署与启动

改动提交到本地 `main` 后，使用现有 `scripts/deploy_to_robot.sh` 部署到 `/home/unix_ai/fpc`。部署后的录制入口为：

```bash
cd /home/unix_ai/fpc
bash datareplay_recorder/run_single_action_recorder.sh \
  --save_dir /home/unix_ai/DataCollector/DataReplay_v3/v3_assets/takes/20260819_libraryrobot_datareplay
```

启动脚本仍可能启动 controller 和头部 RGB-D 相机服务，但不会主动发布头部或升降柱位置命令。

## 验证

按用户要求不新增单元测试。部署前执行以下无动作验证：

- 与机器人参考副本逐文件比较，确认唯一有意的行为差异是移除默认头部/升降柱定位流程及其专用控制类。
- 对 Python 文件运行语法编译检查。
- 对 shell 启动器运行 `bash -n`。
- 运行启动器 `--help` 路径，确认它在服务启动或机器人动作之前退出。

本变更不在验证阶段启动正常录制入口，也不发送头部、升降柱、机械臂、底盘或吸盘命令。
