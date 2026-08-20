# FPC 项目文档

这里是 FPC 项目记录的统一入口。FPC 当前使用从 FruitTest 复制出的可废弃工作副本，验证书本视觉和机器人部件；安全边界始终以仓库根目录的 `AGENTS.md` 为准。

## Agent 必读顺序

1. 先读工作区根目录的 `AGENTS.md`，确认允许写入的位置、远端操作边界和真机安全要求。
2. 再读 [`PROJECT_MAINLINE.md`](PROJECT_MAINLINE.md)，了解项目目标、5090 与 Wanda 的职责以及部署方式。
3. 然后读 [`CURRENT_STATUS.md`](CURRENT_STATUS.md)，了解当前进度、未解决问题和下一步。
4. 涉及真机任务顺序时读 [`STAGE1_PIPELINE.md`](STAGE1_PIPELINE.md)，它是当前唯一 Pipeline。
5. 根据任务查阅路线图、故障档案、工作日志或具体设计与实施计划。

不得用其他文档中的旧描述覆盖 `AGENTS.md` 的规则。若内容冲突，先停止相关写入、部署或真机操作，并按 `AGENTS.md` 处理。

## 当前工作入口

- [`CURRENT_STATUS.md`](CURRENT_STATUS.md)：当前阶段、有效能力、正在进行的工作和下一步。
- [`STAGE1_PIPELINE.md`](STAGE1_PIPELINE.md)：当前唯一 Stage 1 任务顺序和实现边界。
- [`ROADMAP.md`](ROADMAP.md)：阶段顺序、状态和完成依据。
- [`ROBOT_ISSUES.md`](ROBOT_ISSUES.md)：带稳定编号的永久机器人故障档案。
- [`WORKLOG.md`](WORKLOG.md)：按日期记录的重要操作、结果和判断。
- [`INFRASTRUCTURE.md`](INFRASTRUCTURE.md)：5090、Wanda、项目路径和直接部署方式。

## 历史与详细设计

- [2026-08-17 FruitTest 书本感知替换设计](superpowers/specs/2026-08-17-fruittest-book-perception-design.md)
- [2026-08-17 FruitTest 书本感知实施计划](superpowers/plans/2026-08-17-fruittest-book-perception.md)
- [2026-08-18 FPC 项目记录体系设计](superpowers/specs/2026-08-18-project-records-design.md)
- [2026-08-18 FPC 项目记录体系实施计划](superpowers/plans/2026-08-18-project-records.md)
- [2026-08-18 单仓库直接部署设计](superpowers/specs/2026-08-18-single-repo-deployment-design.md)
- [2026-08-18 单仓库直接部署实施计划](superpowers/plans/2026-08-18-single-repo-deployment.md)

设计和实施计划保存具体方案与任务拆分，不代替当前状态、故障档案或安全规则。

## 维护规则

- 长期稳定的目标、系统边界和部署关系更新在 `PROJECT_MAINLINE.md`。
- 当前进度只更新在 `CURRENT_STATUS.md`；阶段安排更新在 `ROADMAP.md`。
- 机器人异常保留在 `ROBOT_ISSUES.md`，解决后更新状态但不删除；重要过程写入 `WORKLOG.md`。
- 同一事实选择一个权威位置，其他文档使用链接引用，避免复制后产生冲突。
- 数值、状态和根因判断必须有命令输出或现场观察支持；未确认的原因明确标为假设。
- 完成重要实现、部署、真机实验或问题调查后，同步更新相应当前视图和历史记录。
- 文档只记录可公开提交的工程信息，不记录密码、密钥、令牌、证书内容、数据集、模型、日志或缓存。
