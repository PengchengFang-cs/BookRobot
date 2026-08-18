# 项目工作日志

## 维护方式

- 日期条目按倒序排列，新记录插入旧日期之前。
- 已记录的事实不为掩盖错误而改写；需要更正时，按更正日期追加更正记录，并链接原记录。
- 不记录凭证、密钥或日志原文件；只保留理解项目演进所需的非敏感事实和结论。
- 本日志记录重要过程，但不替代 [`CURRENT_STATUS.md`](CURRENT_STATUS.md) 的当前状态或 [`ROBOT_ISSUES.md`](ROBOT_ISSUES.md) 的永久故障档案。

## 2026-08-18（Asia/Shanghai）

- 建立“当前状态 + 完整历史”的项目文档制度：当前有效信息由 `CURRENT_STATUS.md` 维护，长期过程和机器人问题分别由本日志与 `ROBOT_ISSUES.md` 保存。
- 调查重启后的底盘释放状态；完成现场释放处理后，前进和后退恢复可用。调查经过和后续待办见 [`BOT-20260818-01`](ROBOT_ISSUES.md#bot-20260818-01)。
- 监控手柄旋转尝试时的 `/cmd_vel`，确认左右旋转期间 `angular.z` 始终为 `0`；问题仍待沿输入链路隔离，见 [`BOT-20260818-02`](ROBOT_ISSUES.md#bot-20260818-02)。
- 更正手柄旋转结论：第二次监测显示操作产生最高约 `0.399 m/s` 的 `linear.x`、`angular.z` 仍为零；用户随后确认此前操作方式错误。改用正确操作后旋转可用，并已把机器人移动到感知测试位置。`BOT-20260818-02` 更新为已解决，未修改软件。
- 在连续执行 release 与 `30°` spin 的组合序列前，odom 航向约为 `0°`；`/spin` 末反馈约为 `32.9°`，序列结束后的总航向约为 `57.8°`。由于没有 release 后、spin 前的独立 odom 基线，两者相差的约 `24.9°` 只能记为未解释残差，不能归因于 release、mode transition、spin 或其他单一阶段。
- 随后使用低速 `/odom` 闭环把航向修正到 `30.307°`；后续静态读数约为 `30.2°`，twist 为零，`diffbot_base_controller` 保持 active。组合序列证据、修正和剩余隔离工作见 [`BOT-20260818-03`](ROBOT_ISSUES.md#bot-20260818-03)。
- 首次从 `/home/unix_ai/fpc` 调用 perception-only 入口时，默认 Python 环境缺少 `grpc`。确认 gRPC 已存在于机器人 vision-rpc sysroot，只是新入口没有加载；临时复用既有 runner 环境后继续测试。永久入口问题见 [`BOT-20260818-04`](ROBOT_ISSUES.md#bot-20260818-04)。
- 五条相机、CameraInfo 和关节 topic 均在线，但 64 条关节历史无法覆盖图像约 `830 ms` 的到达延迟，导致默认同步选择失败。临时把 `joint_buffer_size` 扩大到 512 后，真实链路首次成功返回 `base_link=(1.082, 0.292, 0.740) m`；叠加图中目标位于画面下缘，仍需在整本书完整入镜后正式复测。详见 [`BOT-20260818-05`](ROBOT_ISSUES.md#bot-20260818-05)。
- 用户确认首次识别的是桌沿，因为机器人头部没有低头。经明确授权，保持头部 yaw 不变，将 `joint_head1` 从约 `0 rad` 平滑移动到现有标定低头姿态 `0.24997 rad`。新图可见桌上 5 本书；当前单书选择逻辑覆盖最左侧绿色书，并输出 `base_link=(0.630, 0.262, 0.777) m`。
- 将 FruitTest 感知输出改为：过滤置信度低于 `0.25` 的候选，对几何有效结果按置信度排序，最多返回 5 本，不足时返回实际数量。30 项本地测试通过；部署到 Wanda 后只读真机拍摄返回 5 本，五个 mask 均覆盖对应书本。
- 将部署架构收敛到主仓库：`/home/cvailab/fpc` 保存代码、项目记录、实验环境拓扑和直接部署脚本；`/home/cvailab/fpc-ops` 暂时只保留为历史参考。Wanda 的 `/home/unix_ai/fpc` 明确为普通运行副本，不要求 Git，GitHub 只作独立备份。
- 使用新脚本把 commit `104e9278761988cffe04be3200e1d438564789ea` 从 5090 直接部署到 Wanda。机器人端完成 `run.sh` 语法检查、新 Python 文件编译、10 项导航对位相关纯测试、`main.py --help` 入口检查和 `.deployed-commit` 对比，全部通过且没有发送运动命令。

## 2026-08-17（Asia/Shanghai）

- 建立免密 SSH 与持久反向隧道，使 RTX 5090 能通过 `tsinghuaBot` 访问 Wanda；本记录不包含任何凭证细节。
- 将 GitHub 仓库 `PengchengFang-cs/BookRobot` 定位为公开备份，本地目录继续保持为 `fpc`；Wanda 不从 GitHub pull。
- 将 FruitTest 复制到 5090 与 Wanda 的两个受控 `fpc` 工作区；机器人上的原始项目保持只读，仅作为参考。
- 以书本视觉替换 HSV 水果识别，并将审查修复合入 `main`。详细背景、边界和实施步骤见[书本感知替换设计](superpowers/specs/2026-08-17-fruittest-book-perception-design.md)与[实施计划](superpowers/plans/2026-08-17-fruittest-book-perception.md)。
