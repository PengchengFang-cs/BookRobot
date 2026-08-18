# 可选向量式 XY 对位设计

日期：2026-08-18

## 目标

保留现有 `legacy` 对位作为默认选择，同时增加 `vector` 模式。新模式把视觉 `dx/dy` 合成为一次二维位移，减少原方法为横向修正执行的两次约 90°旋转和额外 X 动作。Z 仍只输出给 Pipeline，不进入导航。

用户接受 X 残差不超过 `10 mm`；本次 Y 目标也设为不超过 `10 mm`。一次移动后只验收，不自动进行第二次修正。

## 已确认的故障边界

旧真机测试只保存了动作完成后的 odom，缺少动作前基线，因此“执行了 4 条命令”和视觉坐标变化不能单独证明物理位移。随后机器人静止时连续执行 5 次感知，4 次有效结果中目标书本 Y 跨度约 `1.0 mm`、X 跨度约 `1.5 mm`，第 5 次明确返回无书。因此旧测试约 `48 mm` 的 Y 坐标变化不是正常静止视觉抖动，但仍需下一次测试用 odom 起终值直接证明底盘位移。

## 模式

命令行增加：

```text
--book-align-mode legacy|vector
```

默认值为 `legacy`，保证现有行为可随时回退。只有显式选择 `vector` 才启用新逻辑。

### legacy

继续调用 `navnav_final.build_base_alignment_commands()`，保持现有 Y 后 X 的离散命令序列。

### vector

计算：

```text
dx = observed_x - reference_x
dy = observed_y - reference_y
distance = hypot(dx, dy)
bearing = atan2(dy, dx)
```

比较前进朝向与倒车朝向所需的旋转绝对值，选择旋转更小的分支。生成至多三条命令：

```text
SPIN(shortest_heading)
DRIVE_FORWARD_OR_BACKWARD(distance)
SPIN(-shortest_heading)
```

每条 `SPIN` 继续使用 `WandaRos2Adapter` 的实时 IMU 闭环；直线段继续使用 precision mode 下的 Nav2/odom action。该模式是最小向量式改进，不宣称是 20–50 Hz 连续 SE(2) 控制器。

## 运动证据

两种模式都必须在动作前调用 `capture_task_origin()`，动作后调用 `current_task_pose()`，返回并打印：

- odom 相对 `dx/dy`；
- IMU 相对 yaw；
- 命令数和使用的模式。

这样即使视觉坐标变化，也能独立判断底盘是否真实运动。动作后仍重新检测一次，报告视觉 X/Y 残差。成功标准为：

```text
abs(final_dx) <= 0.010 m
abs(final_dy) <= 0.010 m
```

不满足时明确报告未达标，但不自动再移动。

## 边界

- 不改变固定参考 `X=0.911600 m, Y=-0.315100 m`。
- 不改变 XY-only 选书和固定 Z offset。
- 不运行 torso、机械臂、DataReplay 或 D01。
- 不修改只读 `/home/unix_ai/navnav_final`。
- 真机测试前仍需用户确认现场可运动。

## 测试

自动化测试覆盖默认 legacy、显式 vector、前进/倒车最短转向、零位移、Z 不进入导航、任务日志包含 odom/IMU 证据，以及 CLI 模式正确传入导航器。
