# Stage 2/3 Pipeline 集成交接文档

更新时间：2026-08-23（UTC）

本文面向下一位负责把 Stage 2/3 Pipeline 与 OCR、书架感知、导航和 DataReplay 接起来的程序员。当前仓库已经写出 Stage 2 → Stage 3 的完整任务编排，但尚不能运行：所有未确认参数和仓库中不存在的接口都被保留为明确缺口，入口会在 `rclpy.init()` 和任何机器人运动之前停止。

开始工作前先阅读：

- 仓库根目录 `AGENTS.md`；
- Stage 1 入口与编排：`main.py`、`mission.py`；
- Stage 2/3 入口与编排：`mission_main2.py`、`mission2.py`；
- 缺口清单：`docs/STAGE23_PENDING_INTERFACES.md`；
- Stage 1 DataReplay：`book_pick_replay.py`、`book_place_replay.py`；
- Stage 1 导航：`book_navigation.py`；
- OCR交接原文：`/home/cvailab/.codex-users/ruan/attachments/cb84166a-963d-43c8-acf2-3a1c1e7bfb94/ocr_handoff(1).md`；
- D01机制调查：`/home/cvailab/.codex-users/ruan/attachments/a830fa93-316e-4ad7-936c-b65a43654be1/DataReplay_D01事件机制与DR3.2处理建议_20260823.md`。

## 1. 当前交付物和状态

本轮新增但尚未提交的文件：

```text
mission_main2.py
mission2.py
docs/STAGE23_PENDING_INTERFACES.md
docs/HANDOFF_STAGE23_20260823.md
```

用户原有、不得覆盖的任务说明：

```text
Stage23_mission_jh.md
```

当前代码的职责分工与 Stage 1 保持一致：

```text
mission_main2.py
  ├─ 解析参数
  ├─ 在ROS初始化前检查所有缺口
  ├─ 初始化ROS、TF和Vision
  ├─ 创建并预加载七个DataReplay Replayer
  └─ 调用run_stage2_stage3()

mission2.py
  ├─ 记录资产、层级映射、观察姿态和验收阈值
  ├─ 复用Stage 1的DataReplay与精定位模块
  ├─ 定义OCR/感知/粗导航的待接接口
  ├─ 顺序执行Stage 2三本书
  └─ 不退出，直接顺序执行Stage 3
```

没有新增状态机、安装体系、地图系统或通用任务框架。`mission2.py`中的 dataclass 只是函数间传递数据的直接容器。

## 2. Stage 2任务内容

### 2.1 任务目标

机器人从小推车中依次取出三本书。每轮固定选择机器人视角中最右侧的可用书，通过书脊标签 OCR 得到目标书架层级和水平编号，再将书放回第三、第四或第五层对应位置。

三本书保证分别属于第三、第四、第五层各一本，但在推车中的左右顺序不固定。

标签实物是两行，例如`A03`和`0015`。程序也接受OCR已经完整输出的规范格式：

```text
^A[0-9]{2}-[0-9]{4}$
```

示例 `A03-0015`：

- `03`表示第三层；
- `0015`表示该层从人正对书架时的左边开始编号的第15号位置。

OCR无完整结果时停止并报告`OCR无法识别`；层级不属于3/4/5时停止并报告`识别结果不是第三、第四或第五层`。不得把两个独立的低置信度OCR结果猜测拼接成书号。

### 2.2 Stage 2资产

| 动作 | 资产 | HDF5文件 | 帧数 |
|---|---|---|---:|
| 推车抓书 | DR11.2 | `pi05_wanda_dr11.2_20260822_002132.h5` | 373 |
| 第五层放书 | DR5.1 | `pi05_wanda_dr5.1_20260821_223711.h5` | 557 |
| 第四层放书 | DR6.4 | `pi05_wanda_dr6.4_20260821_235247.h5` | 527 |
| 第三层放书 | DR7.1 | `pi05_wanda_dr7.1_20260821_234944.h5` | 505 |

共同根目录：

```text
/home/unix_ai/DataCollector/DataReplay_v3/v3_assets/takes/
20260819_libraryrobot_datareplay/
```

### 2.3 Stage 2执行顺序

```text
初始化视觉、OCR、导航和七个Replayer
→ 先预加载Stage 2四个录像，再预加载Stage 3三个Pick录像
→ 如果未指定--skip-stage2-initial-coarse，检测cart_body并粗导航到推车
→ 恢复DR11.2第0帧全身姿态，底盘保持静止
→ 检测推车中的竖直书本并选择最右侧书
→ OCR读取完整标签并解析目标层和编号
→ 计算书脊下端向上13 cm、书脊宽度中点的抓取点
→ 与DR11.2第0帧参考点比较，底盘最多精调三轮
→ 执行完整DR11.2，在资产专属事件帧异步启动右吸盘
→ 不调用holding检查
→ 后退20 cm并粗导航到书架，最终面向书架
→ 根据目标层选择DR7.1、DR6.4或DR5.1
→ 只为视觉观察设置该层升降柱和头部姿态
→ OCR扫描目标层并推算目标编号的空位框
→ 与对应Place第0帧参考比较，底盘最多精调三轮
→ Replayer恢复HDF5第0帧真实全身姿态
→ 完整执行对应Place，在资产专属事件帧关闭吸盘且不吹气
→ 回放结束后通过Stage 1 D01状态接口确认released
→ 第一、第二本后退45 cm，重新检测cart_body并返回推车
→ 第三本放完后退20 cm
→ 直接进入Stage 3
```

抓书验收：X `±20 mm`、Y `±10 mm`。Place验收：前后 `±30 mm`、左右 `±20 mm`、yaw `±5°`。所有视觉Z只记录，不修改DataReplay升降柱轨迹。

## 3. Stage 3任务内容

### 3.1 任务目标

Stage 3直接衔接Stage 2。机器人在底盘外沿距离书架正面60 cm、面向书架的位置扫描第三、第四和第五层全部可见书本的OCR标签，找到唯一一本放错的书，将它从当前错误位置取出，再放回标签指定的正确空位，最后回到相同的60 cm位置结束。

错误分为：

- 层级错误：标签目标层和实际所在层不同；
- 编号错误：层级正确，但书的水平位置不符合标签编号。

场景保证恰好有一本错放书，且其正确目标位置为空。如果没有找到、找到多本，或OCR信息不足以唯一判断，应停止Stage 3并报告，而不是猜测。

### 3.2 Stage 3资产

| 当前实际层 | Pick资产 | HDF5文件 | 帧数 |
|---|---|---|---:|
| 第三层 | DR9.2 | `pi05_wanda_dr9.2_20260822_000416.h5` | 366 |
| 第四层 | DR8.1 | `pi05_wanda_dr8.1_20260821_235504.h5` | 427 |
| 第五层 | DR10.1 | `pi05_wanda_dr10.1_20260822_000941.h5` | 387 |

Place继续复用Stage 2的DR7.1、DR6.4、DR5.1。Pick资产由书当前实际所在层决定；Place资产由书标签中的正确目标层决定。

### 3.3 Stage 3执行顺序

```text
Stage 2第三本Place完成并后退20 cm
→ 移动到面向书架、底盘外沿距书架60 cm的位置
→ 三/四/五层分别使用对应观察姿态
→ 每层头部扫描-45/-30/-15/0/+15/+30/+45°
→ 每个角度采集新的同步RGB-D，检测书本并关联完整OCR标签
→ 合并多角度重复观测
→ 唯一判断层级放错或编号放错的书
→ 按实际层选择DR9.2、DR8.1或DR10.1
→ 粗导航到该书对应Pick视觉工作范围
→ 恢复所选Pick第0帧全身姿态
→ 锁定同一本书，最多三轮X/Y精定位
→ 完整执行Pick并在资产专属帧异步启动吸盘
→ 不调用holding检查
→ 根据书标签确定正确层和编号
→ 保持吸盘开启，粗导航到正确书位
→ 按Stage 2相同规则推算空位并执行Place精定位
→ 恢复Place第0帧真实全身姿态并完整回放
→ 关闭吸盘、不吹气、确认released
→ 返回面向书架且底盘外沿距书架60 cm的位置
→ 关闭所有Replayer并结束
```

## 4. 已确定的公共参数

### 4.1 书架视觉观察姿态

这些数值只用于执行对应DataReplay前的视觉观察。精定位成功后，Replayer仍恢复HDF5第0帧的真实全身姿态。

| 层级 | 升降柱 | 头部pitch |
|---|---:|---:|
| 第五层 | `0.28 m` | `-0.174 rad` |
| 第四层 | `0.00 m` | `-0.174 rad` |
| 第三层 | `0.00 m` | `+0.108 rad` |

### 4.2 书位推算规则

- 编号增长方向固定为人正对书架时从左向右；
- OCR检测框中心使用检测框最左/最右和最上/最下边界的中心，现有函数为`ocr_box_center()`；
- 检测到至少两本不同编号书时，使用它们的三维中心和编号差计算现场实际编号间距；
- 只有一本编号书时，使用书架整体水平轴，并按每个编号`0.05 m`推算；
- 默认目标框宽度为`0.05 m`；
- 已提供`infer_shelf_target()`，返回`ShelfTarget`。

## 5. 当前代码已经复用的现有模块

### 5.1 DataReplay

`mission2.py`直接复用：

- `LegacyV3PickRuntime`、`Stage1BookPickReplayer`；
- `LegacyV3PlaceRuntime`、`Stage1BookPlaceReplayer`；
- 旧v3 Pipeline中的Stage 1块：Pick使用`S1-B1-02/03`，Place使用`S1-B1-05/06`。

每个Stage 2/3资产仍使用现有Stage 1资产合同作为入口，但在运行时覆盖HDF5路径、帧数、版本标签和D01 sidecar事件。不要重写另一套Replayer。

Pick行为：

- `prepare()`先平滑恢复第0帧姿态；
- 精定位只移动底盘；
- `pick(0.0, check_holding=False)`不重复恢复姿态，不做Z补偿，不检查holding。

Place行为：

- 视觉观察只移动头部/升降柱；
- `place()`在回放前恢复真实第0帧全身姿态；
- 回放后继续调用`confirm_released()`。

### 5.2 精定位和机器人反馈

- Pick X/Y：`BookAlignmentNavigator`；
- Place X/Y/yaw：`CartPlaceDockingNavigator`；
- 头部控制、RGB-D同步、关节反馈：`Vision`；
- Replayer反馈：`joint_positions=lambda: dict(vision.joints)`和`spin_until_fresh_body()`。

`CartPlaceDockingNavigator`虽然名字含Cart，但其`align(target)`执行的是通用SE(2)误差修正；Stage 2/3传入的`ShelfAlignmentCommand`具有它所需要的`reference_anchor_m`、`observed_anchor_m`和`yaw_error_rad`字段。

## 6. 必须补齐的接口合同

### 6.1 配置总开关

当前值均为`False`：

```python
OCR_TRANSPORT_READY = False
STAGE23_PERCEPTION_READY = False
STAGE23_COARSE_NAVIGATION_READY = False
```

只有相应接口真实接通后才改为`True`。同时必须填完全部D01事件帧和视觉参考，否则`require_stage23_configuration()`仍会在ROS初始化前退出。

### 6.2 OCR

```python
build_ocr_client() -> ocr_client
```

要求：

- 进程启动时创建一次，Stage 2和Stage 3全程复用；
- OCR结果至少提供`text`、`confidence`、`polygon_px`；
- 只接受完整`A03-0015`格式，不拼接、不补字、不修正低置信度结果；
- OCR框必须与选中的书本mask关联；
- Wanda transport必须使用正式安全接口，不允许insecure或跳过mTLS。

5090本机已验证的OCR入口：

```python
bookbot_vision.book_instance.factory.build_unified_ppocrv6_backend
backend.infer_text_lines(image)
```

运行时和配置位于：

```text
/home/cvailab/Ruan/WHRCompetition/runtime/book-vision-unified-20260814-r4/env/bin/python
/home/cvailab/Ruan/WHRCompetition/runtime/bookbot-rgbd-geometry-competition-20260815-r1/project
.../configs/shelf_ocr_ppocrv6_5090.competition.runtime.json
```

配置原始文件SHA-256：

```text
1d6523351d32e2a7db0684f4c84fadce2065e1662db18e7a44969fc365c695e0
```

截至2026-08-23，8.16的Wanda→5090 OCR transport尚未闭环验收，旧r2证书也已过期。不要把7444的`book_rgbd_geometry`误当OCR，不要安装环境、复制模型或绕过证书。

### 6.3 感知函数

```python
observe_rightmost_cart_book(vision, ocr_client) -> TrackedBook
```

- 检测推车中所有可见竖直书本；
- 固定选择机器人视角最右侧书；
- 抓取点为书脊下端向上`0.13 m`、书脊宽度中点；
- OCR与该书mask关联；
- `label`必须是`BookLabel`；
- `tracking_key`必须足以支持底盘移动后的同书重关联。

```python
observe_tracked_book(vision, tracking_key, *, actual_level=None) -> TrackedBook
```

- 每次底盘修正后重新采集新RGB-D；
- 重关联同一本书，不能重新选择最右书；
- 返回同语义的`suction_point_m`。

```python
observe_shelf_target(vision, ocr_client, label) -> ShelfTarget
```

- 检测目标层已有书本和OCR；
- 建立人正对书架时从左到右的编号轴；
- 调用或遵守`infer_shelf_target()`规则；
- 返回目标框中心、左右边界和书架yaw。

```python
scan_stage3_shelf(vision, ocr_client) -> object
select_unique_misplaced_book(shelf_scan) -> MisplacedBook
```

扫描结果至少需要为每本书保留：`tracking_key`、实际层、完整标签、三维中心、沿编号增长方向的两侧书脊边缘、书脊方向和置信度。`MisplacedBook.book`必须是`ShelfBookObservation`，`reason`明确写层级错误或编号错误。

### 6.4 粗导航函数

以下函数当前全部`raise NotImplementedError`：

```python
navigate_initial_to_cart(vision)
navigate_cart_to_shelf(vision, ocr_client, label)
navigate_shelf_to_cart(vision)
retreat_after_stage2_third_place(vision)
navigate_to_stage3_scan_pose(vision)
navigate_to_misplaced_book(vision, misplaced)
navigate_to_correct_shelf_target(vision, ocr_client, label)
return_to_stage3_scan_pose(vision)
```

实现约束：

- 复用`book_navigation.py`中的navnav运行时和命令类型；
- 参考Stage 1“书桌到推车/推车到书桌”的转向、直行和重新检测方式；
- 粗导航只进入DataReplay视觉工作范围，最终位置仍由精定位决定；
- Stage 2首次目标在机器人右侧，路线为先转向、再直行、最后面向推车；
- 推车Pick完成后先后退`0.20 m`；
- 书架Place后返回推车时先后退`0.45 m`并重新检测`cart_body`；
- Stage 2第三本Place后单独后退`0.20 m`；
- Stage 3扫描和结束位置均为底盘外沿距书架正面`0.60 m`且面向书架；
- 书架搜索先用头部七个角度；找不到时底盘右转每步15°最多180°，回到搜索初始朝向，再左转每步15°最多180°；
- 不新增地图、状态机、备用路线或另一套导航框架。

## 7. D01事件帧和视觉参考

### 7.1 D01 sidecar

HDF5不含D01动作或状态通道。吸盘时序必须写入`D01_EVENT_FRAME_BY_ASSET`：

| 资产 | 事件 |
|---|---|
| DR11.2 | `right_suction_start` |
| DR8.1 | `right_suction_start` |
| DR9.2 | `right_suction_start` |
| DR10.1 | `right_suction_start` |
| DR5.1 | `right_suction_stop`，`release_vacuum=False` |
| DR6.4 | `right_suction_stop`，`release_vacuum=False` |
| DR7.1 | `right_suction_stop`，`release_vacuum=False` |

语义是发布指定frame的机器人动作后立即异步调度D01事件，然后继续frame+1。不能沿用DR1.2的frame 20或DR2.4的frame 190，也不能从dexhand、gripper或breakpoint通道猜测。

### 7.2 D01状态读取

继续复用Stage 1调用链：

```text
Stage1BookPick/PlaceReplayer
→ delegate.check(...)
→ SiteRobotAdapter._read_grip()
→ SuctionClient.read("right")
→ TCP 127.0.0.1:8765
→ get_status JSON
```

Pick当前明确不检查holding；Place当前检查released。不要把D01状态读取误认为事件帧来源。

### 7.3 第0帧参考

必须分别填入：

```python
PICK_REFERENCE_POINT_M_BY_ASSET
PLACE_REFERENCE_POINT_M_BY_ASSET
PLACE_REFERENCE_YAW_RAD_BY_ASSET
```

Pick语义点和Place目标框必须与实时感知完全一致。参考点应由各自资产第0帧RGB-D标定，不能共享Stage 1 JSON，也不能共享不同楼层的参考。

## 8. 入口和运行前检查

当前命令行入口：

```bash
./run.sh --stage23
python3 mission_main2.py
python3 mission_main2.py --skip-stage2-initial-coarse
python3 mission_main2.py --book-align-mode vector
```

参数只有：

- `--skip-stage2-initial-coarse`；
- `--book-align-mode legacy|vector`，默认与Stage 1一致为`legacy`。

`run.sh --stage23`已经接入`mission_main2.py`，使用与Stage 1书本任务相同的ROS、视觉环境和时间戳实验日志目录。`--stage23`只用于脚本选择入口，不会传给Python参数解析器。

缺口存在时，运行入口只会列出缺失项并退出，不会初始化ROS。全部接通后，仍需在Wanda上准备与Stage 1相同的ROS环境：

```bash
source /opt/ros/humble/setup.bash
source /home/unix_ai/work/controller/install/setup.bash
export ROS_DOMAIN_ID=69
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/unix_ai/config/cyclonedds.xml
```

真机运动必须再次获得用户明确授权并确认现场安全。

## 9. Git和部署现状

融合目标仓库：

```text
/home/cvailab/Ruan/Library_306
```

中心融合分支：

```text
feature/stage23-integration
```

基础设施和Stage 2/3代码已经分别提交：

```text
5141b82 deploy: use the current repository as source
d8ae9f2 stage23: integrate fail-closed mission skeleton
fda6c80 docs: record stage23 integration boundaries
a49d6c1 compat: support Python 3.8 stage23 checks
```

以下交付物已经进入Git并同步到5090的`/home/cvailab/Ruan/Library_306`：

```text
Stage23_mission_jh.md
docs/STAGE23_PENDING_INTERFACES.md
docs/HANDOFF_STAGE23_20260823.md
mission2.py
mission_main2.py
```

本轮不部署Wanda，也不执行真机实验。

5090同步前原有的五个未跟踪交付文件已保存在`stash@{0}`（说明为`pre-stage23-integration-untracked-20260823`），没有直接删除。同步后的`feature/stage23-integration`工作树干净且不设置到`/home/cvailab/fpc`的upstream。

### 9.1 重要部署差异

仓库内`scripts/deploy_to_robot.sh`按脚本所在仓库确定部署源：

```text
repo_root="$(cd "$(dirname "$0")/.." && pwd)"
robot_root=/home/unix_ai/fpc
```

因此，从`/home/cvailab/Ruan/Library_306/scripts/deploy_to_robot.sh`运行时只部署该独立仓库自己的已提交HEAD；从`/home/cvailab/fpc/scripts/deploy_to_robot.sh`运行时仍只部署`fpc`自己的HEAD。脚本同时拒绝已跟踪修改和未跟踪文件，避免`git archive`静默漏掉运行依赖。

不要直接复制未提交文件到机器人，也不要手工覆盖`/home/unix_ai/fpc`。正式部署只能来自已提交、没有已跟踪修改的本地分支。Wanda的`/home/unix_ai/fpc`是普通运行副本，不是Git仓库。

## 10. 建议的接线顺序

这是依赖顺序，不是授权执行清单：

1. 与数采人员逐个确认七个D01事件帧。
2. 标定四个Pick参考点、三个Place参考点和三个Place yaw。
3. 接通正式Wanda→5090 OCR transport，实现`build_ocr_client()`。
4. 实现推车竖直书脊感知和同书重关联。
5. 实现书架OCR扫描、空位推算输入和唯一错放书判断。
6. 用现有navnav命令实现八个粗导航函数。
7. 将三个READY常量改为`True`，确认`pending_stage23_configuration()`返回空。
8. 补齐七个新资产各自的SHA-256、起始/结束状态和起始锚点合同，不能沿用Stage 1旧资产合同。
9. 从干净的`/home/cvailab/Ruan/Library_306`已提交分支部署。
10. 用户授权并确认现场安全后，按模块分段进行真机实验，最后再运行完整Stage 2→3。

不要为了让入口通过而填写猜测值、临时返回固定目标、跳过OCR/mTLS、复制Stage 1事件帧，或把`NotImplementedError`改成无动作成功返回。

## 11. 验证状态

中心工作区和5090均已运行纯逻辑单元测试、Python入口检查和shell语法检查。5090默认Python 3.8下，11项Stage 2/3测试和1项部署合同测试通过；`mission_main2.py`按预期列出20类具体缺口并以状态1退出。没有运行模型推理、相机采集、Wanda部署或机器人实验，Stage 2/3外部接口和连续任务仍未验证。

当前唯一被代码保证的安全行为是：只要任一已知缺口仍未填写，`mission_main2.py`会在ROS初始化和任何机器人运动之前失败退出。
