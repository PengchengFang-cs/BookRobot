# 当前阶段：书本视觉与吸取点

这个工作副本正在把 FruitTest 的水果颜色识别替换为书本视觉。当前已经接入的链路是：

```text
Wanda 头部彩色图
  -> 本机 127.0.0.1:7443 转发
  -> RTX 5090 Grounding DINO + SAM 场景分割
  -> 书本 bbox + mask
  -> Wanda 本地对齐深度、内参和头部姿态
  -> base_link 下的封面吸取点
```

桌面平放书本的吸取点固定为：从机器人视角下缘沿长轴向内 `0.13 m`，再从右边缘沿短轴向左 `0.10 m`。如果书本尺寸放不下这两个偏移和 `0.015 m` 的边缘余量，就不输出坐标。

Wanda 当前 Orbbec 驱动虽然发布在 `image_raw` 名称下，但已经启用硬件对齐、彩色目标对齐、深度注册和帧同步；实际彩色与深度消息同为 `1920×1080`、同属彩色光学 frame。代码会在最近几帧中配对同一组 RGB-D/CameraInfo，并使用最接近彩色图拍摄时刻的头部和升降柱关节值。不同 capture 不会混用，同一 capture 也不会重复请求 5090。

只测试感知时使用：

```bash
cd /home/unix_ai/fpc
source /opt/ros/humble/setup.bash
source /home/unix_ai/work/controller/install/setup.bash
export ROS_DOMAIN_ID=69
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/unix_ai/config/cyclonedds.xml
python3 test_book_perception.py
```

这个入口只创建相机、关节状态和 TF 订阅，不导入导航或机械臂模块。成功时输出：

```json
{"book_count": 1, "books": [{"bbox_xywh": [10, 20, 30, 40], "confidence": 0.9, "suction_point_m": [0.41, -0.07, 0.82]}], "frame_id": "base_link", "ok": true}
```

结果按置信度从高到低排列，只保留置信度不低于 `0.25` 且三维几何有效的书本，最多返回 5 本；不足 5 本时返回实际数量。

## DataReplay 两级对位

Stage 1 抓书不再把 `0.48 m` 当作最终抓取坐标。当前流程分成两级：

```text
当前 RGB-D 找到并锁定目标书
  -> 0.48 m 只用于进入粗略工作范围
  -> 停稳后重新拍摄并用 odom/IMU 重关联同一本书
  -> 读取 config/stage1_pick_reference.json
  -> 当前书的 13 cm / 10 cm 点减去录制吸盘固定参考
  -> X/Y 底盘对位，Z 平移整条 Pick torso 轨迹
  -> 才执行 DataReplay
```

参考 JSON 来自只读 Pick HDF5：第 300 帧蓝色吸盘中心对应回第
`0,10,20,40,80` 帧无遮挡深度，再变换到录制时的 `base_link`。标定命令
只读取录制数据、调用书本视觉并生成 JSON/叠加图，不发送机器人运动命令：

```bash
python3 scripts/calibrate_replay_pick_reference.py \
  --h5 /home/unix_ai/DataCollector/DataReplay_v3/v3_assets/takes/pi05_wanda_new1.2_20260816_034411.h5 \
  --output config/stage1_pick_reference.json \
  --overlay logs/stage1_pick_reference_overlay.jpg
```

`./run.sh --book-align --book-align-mode vector` 执行两级底盘对位并在最终测量后退出；`./run.sh --book-pick --book-align-mode vector` 在同一结果上继续执行一次 Pick。两者都要求参考 JSON 已经生成。

当前资产的固定参考是 `base_link=(0.7709968,-0.3608004,0.7538117) m`。这只是 DataReplay 吸盘落点；五本书分别使用各自实时视觉算出的 0.13/0.10 抓取点与它做差，因此一条单书录制轨迹可以逐本复用。

运行前还需要满足：

- 机器人本机 `127.0.0.1:7443` 已转发到 5090 正在运行的视觉 listener。
- `/var/lib/bookbot/vision-mtls/` 中已有 Wanda 客户端 mTLS 文件。
- `/var/lib/bookbot/task3/release_inputs/vision_generated/vision_v2_pb2.py` 已安装。
- 环境变量 `BOOK_VISION_MODEL_VERSION`、`BOOK_VISION_CONFIG_HASH`、`BOOK_VISION_CALIBRATION_VERSION` 已设置为本次已启动服务的准确身份。

可用其他环境变量覆盖 endpoint、证书和 protobuf 路径；具体变量集中在 `config.py`。调试叠加图保存在 `logs/last_detection.jpg`。

下面保留原始 FruitTest 的说明作为导航和机械臂参考；其中水果视觉描述不再适用于当前 `vision.py`。

---

# 原始 FruitTest：听名字，找水果，拿回来

这是一个故意写得很简单的 ROS 2 冒烟测试。主程序只做六件事：

1. 等你说一种水果。
2. 用头部 RGB-D 相机按颜色找水果，并算出三维坐标。
3. 没看到就让 Nav2 原地转 90°，最多看四个方向。
4. Nav2 先转向水果，再直线走到水果前方。
5. MoveIt 规划右臂动作，夹爪拿起水果。
6. Nav2 直线回到启动位置并恢复原朝向，松开水果，然后继续等下一句话。

这是 5 米 × 5 米空场冒烟测试：导航只用 Nav2 的转向和直行行为，
故意不做通用障碍物绕行。

## 水果怎么摆

- 机器人放在约 5 米 × 5 米空地中央。
- 香蕉、苹果、橙子、梨分别放在机器人四周。
- 模型颜色要鲜艳，彼此不要使用相同颜色。
- 水果中心建议离地 `0.80～1.00 m`，放在小支架上；不要直接放地面。
- 水果到机器人中心约 `1.0 m`。程序会停在水果前约 `0.48 m`，所以底盘通常实际走约 `0.52 m`。这样水果不会进入底盘内部，右臂也容易够到。
- 在机器人启动位置右侧放一个小篮子。机器人回原点后会在收纳姿态直接松开夹爪。

## 第一次可以做这些小测试

```bash
cd /home/unix_ai/FruitTest

# 1. 完全不连接机器人，只看积木调用顺序
./run.sh --fake --fruit 香蕉 --once

# 2. 连接真机，但只检查相机、Nav2、关节状态和 MoveIt 规划，不运动
./run.sh --check

# 3. 可选：给 Nav2 和右臂发送“保持当前位置”的真实命令
./run.sh --command-check

# 4. 可选：底盘往返 40 cm、右臂小幅往返、夹爪开合
./run.sh --motion-check

# 如果只想单独测试一块积木
./run.sh --base-motion-check
./run.sh --arm-motion-check

# 不放水果，在空中执行一次完整抓取动作
./run.sh --pick-motion-check

# 5. 真机只执行一次香蕉任务
./run.sh --fruit 香蕉 --once
```

第二条命令看到下面这行才算通过：

```text
[检查通过] RGB-D、Nav2、关节状态、MoveIt 位姿规划都正常；没有产生运动。
```

## 最终运行命令

```bash
cd /home/unix_ai/FruitTest && ./run.sh
```

程序会先语音提示你打开遥控器并按一下机身“释放”键。它收到真实按钮
事件并完成底盘释放后，才会说“已经准备好”。这时说“香蕉”“苹果”
“橙子”或“梨”。完成一次后，它会继续等下一种水果。

## 已做过的真机验证

- 头部 RGB-D、深度三维坐标、Nav2 action、关节状态都在线。
- 当前真实空场依次搜索四种水果，结果全为 `None`，没有把柜门误认成水果。
- Whisper 用真实麦克风听到“香蕉”并解析成 `banana`。
- MoveIt 三个抓取位姿都能规划，完整空抓轨迹已经真实执行成功。
- 右臂小幅往返实测 `0.042 rad`，回收误差 `0.014 rad`，夹爪开合成功。
- Nav2 零转角 action 和右臂零位移轨迹都真实执行成功。
- `DriveOnHeading` 真实接单并持续发布 `0.05 m/s`，超时后自动回到零速度。
- 释放键 ROS 消息桥已经用真实消息类型验证。

无人值守测试时遥控器/物理电机门控没有打开，所以软件无法代替现场的
“打开遥控器并按释放键”动作。最终程序已经把这一步做成启动时的语音等待，
不会在底盘没释放时提前听取水果指令。

这台机器人的 AIUI 只发布“正在识别”状态，没有发布识别文字，所以程序会自动使用机器人已经缓存好的本地 Whisper `base` 模型。第一次启动先加载几秒；等机器人真的说出“已经准备好”后，再在四秒内说水果名字。语音不会上传到网络。

如果现场太吵，也可以用下面这条命令代替说话，视觉、导航和机械臂流程完全相同：

```bash
ros2 topic pub --once /fruit_test/command std_msgs/msg/String "{data: 香蕉}"
```

## 每块积木在哪

- `main.py`：启动程序。
- `mission.py`：任务顺序，最像“积木说明书”。
- `voice.py`：把一句话变成水果名字。
- `vision.py`：5090 书本 mask 接入和本地深度三维坐标。
- `book_rpc.py`：5090 gRPC 请求与二维 mask 解析。
- `book_geometry.py`：书本 mask、点云长短轴和固定吸取点。
- `navigation.py`：转身、接近水果、返回原点。
- `arm.py`：MoveIt 规划、轨迹执行和夹爪开合。
- `config.py`：现场最常修改的数字和颜色范围。

想换水果颜色，改 `config.py` 里的 `FRUITS`。想改变机器人离水果多远，改 `APPROACH_DISTANCE_M`。想改变夹爪中心位置，改 `TOOL_FROM_WRIST_XYZ`。

视觉调试图保存在：

```text
/home/unix_ai/FruitTest/logs/last_detection.jpg
```

MoveIt 日志保存在：

```text
/home/unix_ai/FruitTest/logs/moveit.log
```

程序正常退出时会自动恢复原来的双臂遥控控制器。
