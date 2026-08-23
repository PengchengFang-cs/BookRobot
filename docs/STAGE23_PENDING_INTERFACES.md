# Stage 2/3待补接口

当前代码已经写出Stage 2顺序执行三本书、随后直接进入Stage 3处理唯一错放书的完整调用顺序。以下内容没有在现有仓库中找到可直接调用的正式接口或已确认参数，因此代码保留明确缺口，并在ROS初始化和任何机器人运动之前停止。

标签实物是两行`A03`/`0015`；程序也接受OCR已经完整输出的规范形式`A03-0015`，两者统一解析为同一个`BookLabel`。前缀只接受`A`，目标层只接受3/4/5。不得把两个独立的低置信度OCR结果在调用方猜测拼接。

## 1. DataReplay D01事件帧

HDF5不包含D01动作通道。必须逐个资产确认sidecar事件帧，不能沿用Stage 1的DR1.2 frame 20或DR2.4 frame 190。

| 资产 | 事件 | 待填写位置 |
|---|---|---|
| DR11.2 | `right_suction_start` | `mission2.py`中的`D01_EVENT_FRAME_BY_ASSET` |
| DR8.1 | `right_suction_start` | 同上 |
| DR9.2 | `right_suction_start` | 同上 |
| DR10.1 | `right_suction_start` | 同上 |
| DR5.1 | `right_suction_stop`，不吹气 | 同上 |
| DR6.4 | `right_suction_stop`，不吹气 | 同上 |
| DR7.1 | `right_suction_stop`，不吹气 | 同上 |

Pick继续保持Stage 1行为：默认不调用`confirm_holding()`。Place继续保持Stage 1行为：回放完成后调用`confirm_released()`。

配置门不仅检查是否为`None`，还会拒绝bool、浮点数、负数和超过对应资产总帧数的事件帧。

## 2. 每个录像的第0帧视觉参考

需要根据各录像第0帧RGB-D和对应语义点分别标定：

- DR11.2、DR8.1、DR9.2、DR10.1的Pick参考点；
- DR5.1、DR6.4、DR7.1的Place参考点和参考yaw。

填写位置：

- `PICK_REFERENCE_POINT_M_BY_ASSET`
- `PLACE_REFERENCE_POINT_M_BY_ASSET`
- `PLACE_REFERENCE_YAW_RAD_BY_ASSET`

配置门会在ROS初始化前检查三维点的维度和有限性，以及yaw是否为有限数值。

视觉Z只记录，不修改DataReplay中的升降柱轨迹。对位成功后，Replayer仍恢复并执行HDF5第0帧的真实全身姿态。

## 3. OCR传输接口

RTX 5090本机OCR后端已经存在，规范完整格式为`^A[0-9]{2}-[0-9]{4}$`。当前交接文档明确说明Wanda到5090的Stage 2/3 OCR transport尚未闭环，因此`build_ocr_client()`暂时留空。

补充时必须：

- 一个进程只初始化并复用一个OCR backend；
- OCR文字框必须与所选书本mask关联；
- 不把7444的`book_rgbd_geometry`接口误当作OCR接口；
- 不使用insecure连接或跳过mTLS校验；
- 不安装新OCR环境、复制模型或重写推理逻辑。

## 4. 竖直书脊和书架感知

当前仓库的`book_geometry.py`实现的是Stage 1平放书封面几何，不能直接作为竖直书脊语义点。当前仓库也没有书架层级、编号书位框和错放书判定接口。以下函数保留待补：

- `observe_rightmost_cart_book()`：选择机器人视角最右侧书，计算“书脊下端向上13 cm、宽度中点”的抓取点，并关联OCR；
- `observe_tracked_book()`：底盘移动后重检测并重关联同一本书；
- `observe_shelf_target()`：检测目标层已有编号书并计算空位；
- `scan_stage3_shelf()`：按三/四/五层和头部`-45°`到`+45°`七个角度扫描；
- `select_unique_misplaced_book()`：唯一判定层级放错或编号放错的书。

OCR检测框中心按检测框最左、最右、最上、最下边界的中心计算，代码已提供`ocr_box_center()`。书位计算已提供`infer_shelf_target()`：检测到至少两本编号书时使用现场实测编号间距；只有一本时使用默认`0.05 m`书宽。

`ShelfBookObservation`还需为每本书提供沿编号增长方向的两侧书脊边缘和书脊方向。相邻编号书存在时，空位必须使用两本书靠近空位的实际内侧边缘；不能退化成固定5 cm中心框。错放到当前层、但标签目标层不属于当前层的书不能参与该层编号间距标定；目标编号已经存在时必须停止。

## 5. Stage 2/3粗导航

现有`BookAlignmentNavigator`和`CartPlaceDockingNavigator`可继续负责Pick X/Y精定位和Place X/Y/yaw精定位。现有Stage 1粗导航函数的固定扫描方向和路线不等同于Stage 2/3已确认路线，因此没有直接误用，以下入口保留待补：

- `navigate_initial_to_cart()`
- `navigate_cart_to_shelf()`
- `navigate_shelf_to_cart()`
- `retreat_after_stage2_third_place()`
- `navigate_to_stage3_scan_pose()`
- `navigate_to_misplaced_book()`
- `navigate_to_correct_shelf_target()`
- `return_to_stage3_scan_pose()`

补充时沿用Stage 1的导航命令和视觉粗调/精调分工，不新增地图、状态机或另一套导航框架。

## 6. DataReplay资产合同

当前运行桥复用Stage 1资产入口，并覆盖新HDF5的路径、版本、帧数和D01事件。七个新资产各自的SHA-256、起始/结束状态和起始锚点合同尚未写入；在把它们称为正式可复现资产前必须逐个补齐，不能沿用Stage 1旧资产的摘要或状态合同。

## 7. 已写入代码的录像资产

| 资产 | 文件 | 帧数 |
|---|---|---:|
| DR5.1 | `pi05_wanda_dr5.1_20260821_223711.h5` | 557 |
| DR6.4 | `pi05_wanda_dr6.4_20260821_235247.h5` | 527 |
| DR7.1 | `pi05_wanda_dr7.1_20260821_234944.h5` | 505 |
| DR8.1 | `pi05_wanda_dr8.1_20260821_235504.h5` | 427 |
| DR9.2 | `pi05_wanda_dr9.2_20260822_000416.h5` | 366 |
| DR10.1 | `pi05_wanda_dr10.1_20260822_000941.h5` | 387 |
| DR11.2 | `pi05_wanda_dr11.2_20260822_002132.h5` | 373 |

书架视觉观察姿态已经写入：

- 第五层：升降柱`0.28 m`，头部pitch `-0.174 rad`；
- 第四层：升降柱`0.00 m`，头部pitch `-0.174 rad`；
- 第三层：升降柱`0.00 m`，头部pitch `+0.108 rad`。
