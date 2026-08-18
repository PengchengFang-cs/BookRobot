# Stage 1 XY 导航与 Z 偏移交接设计

日期：2026-08-18

## 背景

Stage 1 DataReplay 使用绝对升降柱轨迹。若 FPC 在导航阶段根据视觉 Z 调整升降柱，回放开始后原始 `0.20 m` torso 帧会覆盖这次调整。此前 FPC 的 `0.20 m + Z delta` 修复只解决了独立导航测试的起始高度错误，不能解决整段 Pick/Place 回放的高度一致性。

## 范围

本次只修改 FPC 的感知、选书和导航交接边界：

- 选书只按吸取点的 X/Y 距离排序。
- 底盘对位只接收 X/Y，传给 `navnav_final` 的 observed Z 固定为 reference Z。
- 视觉 Z 只计算一次 `z_offset_m = observed_z - reference_z`，不调整升降柱、不累加、不乘增益。
- FPC 将 `z_offset_m` 连同选中书本对象保留在任务结果中，供旧 Pipeline 创建 run/book/asset scoped token。

机械臂轨迹变形、DataReplay episode 内存副本、token 生命周期和 D01 状态转换由旧 Pipeline 补丁负责，不在 FPC 中重复实现。

## 选书与偏移

每个候选仍保留完整三维吸取点和 `dx/dy/dz`，但排序距离固定为：

```text
distance_xy_squared = dx * dx + dy * dy
```

Z 不参与排序。选中后：

```text
z_offset_m = dz = observed_grasp_base_z_m - reference_grasp_base_z_m
```

FPC 要求 `z_offset_m` 为有限实数且 `abs(z_offset_m) <= 0.03 m`。合法的 `0.0 m` 作为普通数值保留。超出业务范围时在发送任何导航命令前失败，不选择其他书本替代。

## 导航

`BookAlignmentNavigator.align()` 继续复用只读的 `navnav_final` Y/X 底盘命令，但传入：

```text
reference = (reference_x, reference_y, reference_z)
observed  = (observed_x,  observed_y,  reference_z)
```

因此构建器不会生成 `TORSO_POSITION` 命令。适配层只读取一次当前 torso 作为旧构建器的必需参数，不设置 torso 目标，也不执行高度到位验收。返回值只包含执行命令数。

这一步仍保留现有离散底盘对位方式；连续 `MICRO_DOCK_SE2` 属于后续独立工作，不在本补丁中同时实现。

## 任务输出

任务入口完成移动前检测、XY-only 导航和移动后检测。结果明确输出：

- 初始 `dx/dy/dz`；
- 本次固定的 `z_offset_m`；
- 执行的 X/Y 导航命令数；
- 移动后重新关联目标的 `dx/dy/dz`。

移动后的视觉 Z 只作为观测报告，不覆盖本次已经固定的 `z_offset_m`，避免多轮累加或因重分割噪声改变同一本书的回放偏移。

## 测试

自动化测试覆盖：

- 两本书 X/Y 距离不同但 Z 差异相反时，按 X/Y 选择。
- `z_offset_m` 精确等于 `observed_z - reference_z`。
- 正、负和 `0.0 m` 偏移均保留。
- 超过 `±0.03 m` 的偏移在导航前失败。
- 导航构建器收到的 observed Z 等于 reference Z。
- 执行命令中不包含 Z/torso 调整。
- 任务结果和日志保留初始固定 Z offset，不用移动后的视觉 Z 覆盖。
- 原有感知、底盘启动顺序和“不调用机械臂/DataReplay/D01”行为保持不变。

部署验证只运行编译、导入和纯单元测试，不启动真机运动。
