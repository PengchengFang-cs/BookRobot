# FPC 单仓库直接部署设计

日期：2026-08-18

## 决策

`/home/cvailab/fpc` 是代码、项目文档、服务器拓扑和部署工具的唯一权威开发仓库。RTX 5090 直接把该仓库已提交的运行文件同步到 Wanda 的 `/home/unix_ai/fpc`；机器人运行目录是普通文件副本，不要求包含 Git，也不从 GitHub 拉取。

`/home/cvailab/fpc-ops` 不再承载业务代码或当前部署流程，暂时只保留为历史参考，不在本次工作中删除。

## 允许写入的目录

RTX 5090 上允许修改用户自己的两个项目目录：

- `/home/cvailab/fpc`
- `/home/cvailab/fpc-ops`

Wanda 上仍只允许修改 `/home/unix_ai/fpc`。机器人上的其他工程和系统目录继续作为只读参考。

## 服务器记录边界

项目允许记录连接该实验环境所需的 Host 别名、IP 地址、用户名、端口、项目路径和部署方向。这些信息集中记录在 `docs/INFRASTRUCTURE.md`。

密码、私钥、Token、证书内容仍不得写入 Git。数据集、模型、日志、缓存和运行生成物也不进入 Git。

## 部署流程

部署脚本放在 `scripts/deploy_to_robot.sh`，执行以下步骤：

1. 确认来源是 `/home/cvailab/fpc` 的 `main` 分支且工作树干净。
2. 用 `git archive HEAD` 导出当前已提交文件到临时目录。
3. 通过 `ssh tsinghuaBot` 确认目标目录 `/home/unix_ai/fpc` 存在。
4. 用 `rsync --archive` 从 5090 直接更新机器人运行副本。
5. 在机器人运行目录写入 `.deployed-commit`，记录本次本地 commit。

部署不自动执行 `git push`，GitHub 备份由独立操作完成。部署默认不使用 `rsync --delete`，因此不会删除机器人运行目录中的日志、模型、缓存或其他未包含在 Git 归档中的文件。

## 验证

部署脚本通过 shell 测试验证：只允许从干净的 `main` 部署、不调用 GitHub、不使用 `rsync --delete`、来源是 `git archive HEAD`、目标固定为 `/home/unix_ai/fpc`。

部署后先在 Wanda 运行 Python 编译、纯逻辑测试和 `main.py --help`；这些命令不发送运动。真机动作仍只在用户明确授权且现场环境已确认后执行。
