# Stage 1 单本吸书回放设计

日期：2026-08-18

## 目标

在 FPC 中新增一次性真机入口，顺序执行当前书本检测、一次底盘对位、固定 Z 偏移生成、Stage 1 Pick DataReplay 和右 D01 吸附确认。流程只吸起一本书并保持，不执行 Place、返航、整场比赛 Pipeline 或 MoveIt。

## 已批准的数据流

```text
FPC 书本 RGB-D 检测
→ 选择最接近录制抓取点的书
→ 当前 legacy/vector 底盘 X/Y 对位
→ 保存 initial observed_z - reference_z
→ 内存复制 S1_TABLE_PICK_BOOK episode
→ 607 帧 target_qpos_torso 全部加同一 Z offset
→ 升降柱预置到修正后的 frame 0 高度
→ DataReplay 逐帧发布，frame 300 调用 right_suction_start
→ 回放完成后读取 D01 holding 状态
→ 保持举书并退出
```

## 边界

- 新入口为 `./run.sh --book-pick --book-align-mode vector`；现有 `--book-align` 行为不变。
- 书本检测、选书和 X/Y/Z 计算只使用 FPC 当前实现，不重新运行旧 Pipeline 的视觉或对位。
- Z offset 沿用当前 `BookAlignmentRun.z_offset_m`，定义为首次选中书本的 `observed_z-reference_z`，不乘 gain、不逐轮累计。
- 原始 HDF5 只读。代码复制 episode、`actions` mapping 与 `target_qpos_torso` 数组后再平移，不修改磁盘资产或原 episode。
- 修正后的所有 torso 帧必须有限且位于 `[0.0, 0.3] m`；超界时不得发送 frame 0。
- 回放前使用 `navnav_final` 的现有 `TORSO_POSITION` 路径把升降柱移动到修正后的首帧高度，并记录目标与反馈。
- 复用 `/home/unix_ai/WHRC/v3_pipeline` 的 DataReplay 发布、服务切换和 D01 sidecar 调度；`S1_TABLE_PICK_BOOK` 禁止底盘回放。
- 不执行 SHA 校验，不增加重复检测、自动重试或额外比赛安全门。现有 DataReplay 所需的控制器切换和 D01 事件确认属于功能链路，继续保留。
- 当前用户明确接受本轮由地面摩擦产生的 Y 残差，因此 Pick 入口报告最终 XY，但不因现有 `10 mm` 视觉验收结果阻断回放。

## 组件

- `book_pick_replay.py`：复制并平移 episode、预置 torso、调用旧 DataReplay 的 Pick 发布和最终 D01 检查。
- `mission.py`：新增 `run_book_pick_once`，复用 `run_book_alignment_once` 并把固定 Z offset 交给 replayer。
- `main.py`：新增 `--book-pick`，延迟加载真机 replay 依赖。
- `run.sh`：让 `--book-pick` 与 `--book-align` 共用书本视觉环境，并避免启动 FruitTest MoveIt。

## 结果与错误

成功结果至少包含：607 帧已发送、Z offset、首帧 torso 目标、实际 torso 反馈和 D01 holding。任一 episode 加载、torso 超界、预置、DataReplay 或 D01 错误直接结束，不执行 Place，也不自动重试。

## 测试

- 纯单元测试证明正/负/零 offset、原 episode 不变、607 帧恒定偏移以及越界时拒绝。
- 编排测试证明顺序是 alignment 后唯一一次 pick，并原样传递固定 Z offset，即使 XY 报告未达 10 mm 也按用户批准继续。
- CLI/launcher 测试证明 `--book-pick` 使用书本环境、不启动 MoveIt。
- 部署后先运行机器人端纯测试和 import 检查，再在当前已确认无人且用户观察的现场执行一次真机 Pick。
