# BookRobot

Wanda 图书机器人比赛项目。机器人使用头部 RGB-D 相机采集现场图像，将彩色图发送到
RTX 5090 上的 Grounding DINO + SAM 服务完成二维检测与分割，再在 Wanda 本机结合
深度、相机内参和机器人姿态计算三维目标，最后通过底盘对位与 DataReplay 完成抓放。

## 当前进度

项目目前正式进入 **Stage 2**。

- **Stage 1：桌面书搬运到推车。** 主流程已经打通，历史上完成过三本书连续真机
  循环；当前两本速度实验也已经跑通。遗留待办只有两项：继续提升整体速度，以及
  小幅微调推车 Place 的前后距离估计。此前整体增加 `30 mm` 的补偿已确认过度并撤销。
- **Stage 2：推车三本书归架。** 当前开发主线。从推车依次取出三本书，通过 OCR
  确定目标层和编号，再分别放到书架第三、第四或第五层的正确位置。
- **Stage 3：纠正唯一错放书。** Stage 2 完成后扫描书架，通过 OCR 和书位关系找到
  唯一错放书，将其取出并放回正确空位。

旧文档曾把“感知、底盘、抓取、demo、比赛集成”等开发里程碑称为 Stage 1～5；
该编号方式已经废弃，不再用于描述当前比赛流程。

## Stage 1 当前结果

Stage 1 已经串联以下完整流程：

```text
桌面书检测与精定位
→ Pick DataReplay
→ 粗导航到推车
→ 平台与槽位精定位
→ Place DataReplay
→ 返回书桌继续下一本
```

当前两本实验在不计算第一次到书桌粗导航时，核心耗时约 `335 s`，尚未达到五分钟
目标。最近一次 Place 使用的 `30 mm` 前后补偿过度，当前代码已经恢复到补偿前参考，
后续需要从该参考继续做小幅调整。

## Stage 2 当前入口

Stage 2/3 的任务编排入口为：

```text
run.sh --stage23
mission_main2.py
mission2.py
```

Stage 2 会复用 Stage 1 已有的视觉传输、底盘精定位和 DataReplay 桥。当前资产、
执行顺序和剩余接口以交接文档及实际源码为准。

## 系统分工

- **RTX 5090**：运行 Grounding DINO、SAM 和 OCR，返回二维视觉结果。
- **Wanda**：采集 RGB-D，在本机完成深度投影与机器人坐标计算，并控制底盘、升降柱、
  机械臂和吸盘。
- **DataReplay**：恢复对应录像的第 0 帧全身姿态，并按录制时序执行完整动作。

## 开发与部署

- 多人最终集成仓库：`/home/cvailab/Ruan/Library_306`
- 已验证基线仓库：`/home/cvailab/fpc`
- Wanda 运行副本：`/home/unix_ai/fpc`

GitHub 只用于代码备份，不是机器人部署通道。机器人不会从 GitHub pull；部署由 5090
当前仓库的 `scripts/deploy_to_robot.sh` 将已提交且干净的当前分支直接复制到 Wanda。

## 文档入口

- [当前状态](docs/CURRENT_STATUS.md)
- [比赛阶段路线图](docs/ROADMAP.md)
- [Stage 2/3 交接文档](docs/HANDOFF_STAGE23_20260823.md)
- [Stage 2/3 待补接口](docs/STAGE23_PENDING_INTERFACES.md)
- [Stage 1 当前 Pipeline](docs/STAGE1_PIPELINE.md)
- [完整历史交接](docs/HANDOFF_20260822.md)
- [工作日志](docs/WORKLOG.md)
- [机器人问题记录](docs/ROBOT_ISSUES.md)

修改、部署和真机操作必须遵守仓库根目录 [AGENTS.md](AGENTS.md)。
