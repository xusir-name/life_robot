# applications —— 完整运行节点

> 入口：[`commen_readme.md`](../commen_readme.md) · 依赖台账：[`readme.md`](../readme.md)
> 分层：[`docs/layers/L7-applications.md`](../docs/layers/L7-applications.md)

## 定位

**一个包 = 一个应用 = 一个完整运行节点。**
能独立启动、面向一个具体场景，把下层能力组装成完整流程。
写「怎么把机器人用起来」，不写「机器人怎么动/感知/决策」。

## 什么才算

- [ ] 有明确入口（`main` / `ros2 run` / `launch`）
- [ ] 能独立启动并进入就绪
- [ ] 有明确输入/输出
- [ ] 有生命周期：启动→就绪→运行→暂停/取消→退出
- [ ] 能完成一个完整应用流程
- [ ] 依赖下层公开接口，不复制实现

不算：工具函数、数据结构、msg、驱动、协议解析、算法库、可复用 UI 组件、mock。

## 做

- 编排：组合 L5 任务、L4 查询、L6 遥操作、L1 只读状态。
- 交互：CLI/Web/语音/按钮/远程 API。
- 会话：启动、就绪检查、运行、暂停、恢复、取消、退出。
- 展示：进度、阶段、错误、场景快照、健康（只读）。
- 安全交互：危险操作二次确认、模式提示、急停引导。
- 应用级配置、操作日志、优雅退出。

## 不做

- ❌ 不重复实现下层通用能力；标定/调试类专用算法可在应用包内以**纯函数**存在，且必须可单测。
- ❌ 发 `/control/*`、直接控制硬件。
- ❌ 定义 msg/srv/action（→ `interfaces/`）。
- ❌ 放可复用库/工具/公共组件（下沉到对应层或独立库）。
- ❌ 复制其他层代码/私有消息；直接访问串口/CAN/相机。
- ❌ 维护场景真源（查 L4）；写行为树（L5）。
- ❌ 一包多应用；把调试 demo 当正式应用。

## IO

输入：人的操作、L5 状态/feedback/result、L4 场景（只读）、L1 状态（只读）、模式。
输出：L5 任务 action、取消/暂停、L6 遥操作（仅 MANUAL）、模式切换、展示/日志。

## 目录

```
applications/src/<app_name>/
├── <app_name>/main.py       # ★ 唯一入口
├── <app_name>/app.py        # 本应用流程/状态机
├── <app_name>/ui/           # 本应用私有界面
├── <app_name>/config.py
├── config/                  # 应用级参数
├── launch/                  # 本应用启动
└── test/
```

当前包：

- `hand_eye_calibration`：眼在手上标定（见其 `README.md`）。
- `graspnet_pick`：C++ 仿真抓取闭环（RGB-D 框 → 最高分抓取 → 服务触发抓取），
  两个节点 `grasp_estimator` / `grasp_executor`；算法为应用内纯函数，迁移目标见其 `README.md`。

- 语言按应用选择：Python 用上面的 `main.py/app.py` 布局；C++ 用 `src/*.cpp` + `include/<pkg>/*.hpp` + `CMakeLists.txt`（见 `graspnet_pick`）。
- 一应用一包；包名场景语义：`pick_place_app`、`teleop_app`。
- 通用逻辑下沉；应用级 launch 不承担全系统 bringup。

## 约束

- 需要新能力 → 加到对应层并暴露接口，不在应用里临时实现（专用标定/调试算法除外，见上）。
- 标定/调试类应用可在 `MANUAL`/离线模式调用运动接口（如 `/target_pose`），但**不得发 `/control/*`**；正式运行应经 L6 action。
- **仿真验证例外**：允许应用包内以纯函数实现抓取位姿估计，并通过仿真 `/target_pose` 桥打通
  “框 → 抓取 → 执行”链路；必须可单测、README 标注迁移目标，真机前迁移到 L3/L6。
- SIGINT/SIGTERM 优雅取消任务后退出；启动检查依赖就绪。
- action 异步调用，不阻塞 UI 主循环；高频展示限流。
- 危险动作二次确认；`ESTOP/FAULT` 禁用运动入口；取消展示真实 `CANCELED`。
- 每个任务记录 `task_id`/时间/意图/结果；UI 断线由 L5 超时兜底。

## 验收

- [ ] 一条命令启动并完成完整流程
- [ ] 取消/退出后无残留任务/线程/连接
- [ ] 不发 `/control/*`、不直接访问硬件
- [ ] 不含下层通用能力的重复实现；标定专用算法已隔离且可单测
- [ ] 无公共库留在应用包内
- [ ] 参数外置；模式行为正确
- [ ] 更新本文件索引与 L7 文档

## 常见错误

| 错误 | 正确 |
| ---- | ---- |
| 驱动/协议解析放这里 | `drivers/` 或 `communicate/` |
| msg 定义放这里 | `interfaces/` |
| 通用 UI/工具放这里 | 独立库或对应层 |
| 一包多应用 | 一应用一包 |
| 直接发 `/control/*` | 调 L5；遥操作调 L6 |
| 自己维护场景状态 | 查 L4 |
| UI 写行为树/规划 | L5/L6 |
| 界面关了任务还在跑 | L5 超时 + 优雅取消 |
| 调试 demo 当正式应用 | 调试放 `tools/` |

## 代码文件索引

> 路径相对仓库根目录；新增/删除/重命名文件、改变引用或话题时同步。

| 文件 | 作用 | 使用到的文件/资源 | 订阅/调用 | 发布/提供 |
| ---- | ---- | ----------------- | --------- | --------- |
| ★ `applications/src/hand_eye_calibration/hand_eye_calibration/node.py` | 标定主节点：运动、采集、时间戳对齐、写完整报告 + 纯外参、结束回原位 | `aruco.py`、`hand_eye.py`、`config/target_poses.yaml`、TF | `/camera/image_raw`、`/camera/camera_info`、`/arm_tcp_state` | `/target_pose` |
| `applications/src/hand_eye_calibration/hand_eye_calibration/aruco.py` | ArUco 检测 + PnP，输出 `T_cam_marker` | OpenCV、`hand_eye.make_transform` | — | — |
| `applications/src/hand_eye_calibration/hand_eye_calibration/hand_eye.py` | 手眼数学：`look_at`、位姿变换、`solve_eye_in_hand`、一致性指标 | OpenCV、SciPy、NumPy | — | — |
| `applications/src/hand_eye_calibration/hand_eye_calibration/__init__.py` | 包入口 | — | — | — |
| `applications/src/hand_eye_calibration/config/target_poses.yaml` | 相机目标位姿与 `look_at` | — | — | — |
| `applications/src/hand_eye_calibration/launch/hand_eye_calibration.launch.py` | 标定启动入口 | `config/target_poses.yaml`、节点参数 | — | 启动节点 |
| `applications/src/hand_eye_calibration/test/test_hand_eye.py` | 合成数据验证手眼求解 | `hand_eye.py` | — | — |
| ★ `applications/src/graspnet_pick/src/estimator_node.cpp` | 框内抓取估计：RGB-D → 点云 → 最高分候选 | `geometry.cpp`、`transforms.cpp`、TF | `/camera/image_raw`、`/camera/depth/image_raw`、`/camera/camera_info`、`/grasp/target_box` | `/grasp/best_grasp`、`/grasp/best_pose` |
| ★ `applications/src/graspnet_pick/src/executor_node.cpp` | 服务触发抓取：观察位姿/自动居中、手眼变换、候选朝向重试、开/闭爪、抬起 | `transforms.cpp`、`interfaces`、TF | `/grasp/best_grasp`、`/arm_tcp_state`、`/target_pose_status`、`/joint_states`、`/grasp/execute` | `/target_pose`、`/gripper_controller/joint_trajectory` |
| `applications/src/graspnet_pick/src/grasp_estimator_main.cpp` | 估计节点 main | `estimator_node.cpp` | — | — |
| `applications/src/graspnet_pick/src/grasp_executor_main.cpp` | 执行节点 main（多线程执行器） | `executor_node.cpp` | — | — |
| `applications/src/graspnet_pick/src/geometry.cpp` | 纯函数：反投影、去平面、聚类、OBB、抓取采样/评分/NMS | Eigen、OpenCV | — | — |
| `applications/src/graspnet_pick/src/transforms.cpp` | 刚体变换、四元数、标定 YAML 读取 | Eigen、yaml-cpp | — | — |
| `applications/src/graspnet_pick/include/graspnet_pick/geometry.hpp` | 点云/抓取算法接口 | `geometry.cpp` | — | — |
| `applications/src/graspnet_pick/include/graspnet_pick/transforms.hpp` | 刚体变换/标定接口 | `transforms.cpp` | — | — |
| `applications/src/graspnet_pick/include/graspnet_pick/estimator_node.hpp` | 估计节点声明 | `estimator_node.cpp` | — | — |
| `applications/src/graspnet_pick/include/graspnet_pick/executor_node.hpp` | 执行节点声明 | `executor_node.cpp` | — | — |
| `applications/src/graspnet_pick/CMakeLists.txt` | ament_cmake 构建：核心库、两个节点、gtest | `package.xml` | — | — |
| `applications/src/graspnet_pick/launch/graspnet_pick.launch.py` | 估计 + 执行启动入口，集中赋值参数 | 节点参数、标定 YAML | — | 启动两个节点 |
| `applications/src/graspnet_pick/test/sim_move_to_view.py` | 仿真辅助：相机观察位姿 → `/target_pose` | TF、MoveIt 桥 | — | `/target_pose`（测试） |
| `applications/src/graspnet_pick/test/sim_color_box_publisher.py` | 仿真辅助：颜色阈值代替 YOLO 发框 | OpenCV | `/camera/image_raw` | `/grasp/target_box`（测试） |
| `applications/src/graspnet_pick/test/test_geometry.cpp` | C++ gtest：几何/变换 | `geometry.cpp`、`transforms.cpp` | — | — |
