# FPC 项目主线

## 当前目标

FPC 当前基于 FruitTest 的可废弃副本实现 Stage 1 完整任务：到书桌抓书、转移到小推车、按五个槽位精定位并执行 Place DataReplay。当前唯一运行顺序见 [`STAGE1_PIPELINE.md`](STAGE1_PIPELINE.md)。

这个副本用于隔离试验，不直接修改或依赖机器人上的其他旧工程作为当前开发工作区。

## 系统分工

- RTX 5090 运行 Grounding DINO 和 SAM，只负责从 Wanda 的彩色图像产生书本 bbox 与 mask 等二维视觉结果。
- Wanda 保留对机器人本体数据和坐标的控制，在本机使用同一采集时刻的 aligned depth、相机内参和机器人姿态完成三维计算，输出 `base_link` 坐标系下的吸取点。
- 深度、内参、机器人姿态和动作控制不交给 5090。二维推理与本体几何的边界应保持清晰，便于分别测试和替换。

## 工作副本与参考代码

允许修改的项目路径只有：

- RTX 5090 已验证基线：`/home/cvailab/fpc`
- RTX 5090 多人最终集成：`/home/cvailab/Ruan/Library_306`
- Wanda：`/home/unix_ai/fpc`

机器人上的其他目录都是只读参考材料。可以用只读命令检查并把需要的文件复制到上述两个 `fpc` 工作路径中，但不得在原参考路径修改、删除，或递归更改权限、所有权和内容。

开发与测试围绕可废弃的 FruitTest 副本进行。旧工程提供行为和接口参考，不是可以就地改造的工作副本。

## 部署与备份

正常部署方向是从 RTX 5090 当前选定仓库自身的 `scripts/deploy_to_robot.sh` 直接更新 Wanda 的 `/home/unix_ai/fpc`。脚本按自身位置确定仓库根，因此`/home/cvailab/fpc`和`/home/cvailab/Ruan/Library_306`互不串仓。部署源可以是已提交且没有任何工作树修改或未跟踪文件的`main`或功能分支，脚本只导出当前`HEAD`。

Wanda 的 `/home/unix_ai/fpc` 是普通运行副本，不是 Git 工作树，不需要 clone、pull 或 fast-forward。脚本默认不删除机器人额外的日志、模型、缓存或其他运行文件。服务器地址、用户、目录和命令见 [`INFRASTRUCTURE.md`](INFRASTRUCTURE.md)。

GitHub 仓库 `PengchengFang-cs/BookRobot` 是公开备份，不是部署通道。部署脚本不自动 push GitHub，Wanda 也不从 GitHub pull。

备份中只包含可公开提交的源码和文档。密码、密钥、令牌、证书、数据集、模型、日志和缓存均不得进入 Git。

## 工作约束

- 根目录 `AGENTS.md` 是写入范围、远端操作和真机安全的权威规则；其他说明与它冲突时以其为准。
- 检查机器人参考代码时使用只读挂载或只读命令，只把所需内容复制进允许写入的工作副本。
- 不使用 `git reset --hard`、强制推送或未经用户明确批准的 `rsync --delete`。
- 任何可能让底盘、机械臂、升降机构或吸取装置运动的命令，都必须先获得用户明确授权，并确认现场物理环境安全、人员和障碍物已清离。
- 事实、测量与假设必须分开记录；没有验证的根因不得写成结论。

## 当前实现原则

- Pick 和 Place 都以对应 DataReplay 第 0 帧的机器人—目标相对几何作为精定位参考。
- 整车中心只用于书桌到小推车之间的粗导航，不作为 Place 精定位目标。
- 不使用旧地图固定点、`0.48 m` 水果距离或 `0.8 m` 整车中心距离。
- 原参考工程继续保持只读；所有实现只进入两个允许写入的 `fpc` 工作路径。
