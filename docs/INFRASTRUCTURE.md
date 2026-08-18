# FPC 实验环境与部署

## 主机

| SSH Host | 地址 | 用户 | 端口 | 用途 |
|---|---|---|---:|---|
| `tsinghua5090` | `10.10.0.213` | `cvailab` | 22 | 主开发机与视觉推理服务器 |
| `tsinghuaBot` | `192.168.137.212` | `unix_ai` | 22 | Wanda 机器人 |

本地开发仓库是 RTX 5090 上的 `/home/cvailab/fpc`。机器人运行副本是 `/home/unix_ai/fpc`。

`/home/cvailab/fpc-ops` 是此前用于连接与部署记录的独立目录。当前部署工具已并入主仓库，该目录暂时只保留为历史参考。

## 部署方向

```text
/home/cvailab/fpc (5090 committed main)
    └── SSH/rsync ──> /home/unix_ai/fpc (Wanda plain runtime copy)
```

在 5090 执行：

```bash
/home/cvailab/fpc/scripts/deploy_to_robot.sh
```

脚本使用 `git archive HEAD` 导出已提交文件，直接同步到机器人，并在 `/home/unix_ai/fpc/.deployed-commit` 记录版本。它不要求机器人目录有 Git，不从机器人访问 GitHub，也不删除机器人额外的运行文件。

GitHub 仓库 `PengchengFang-cs/BookRobot` 仅用于公开备份，不是部署通道；备份操作与机器人部署分开执行。

## 可以和不可以记录的内容

可以在项目中记录本实验环境的 Host 别名、IP、用户名、端口、目录和部署关系。

密码、私钥、Token 和证书内容不得提交。数据集、模型、日志、缓存及运行生成物也不得进入 Git。
