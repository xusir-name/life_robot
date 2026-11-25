# graspnet_pick

> 上级：[`applications/readme.md`](../../readme.md) · 分层：[`docs/layers/L7-applications.md`](../../../docs/layers/L7-applications.md)
> 状态：**仿真验证应用**。感知算法与应用节点暂时同包，迁移目标见文末。

## 用途

把「一个图像框」变成「一次真实抓取」：

```
RGB-D + 目标框 → 抓取候选（最高分） → GraspCandidate 话题
                                          │
                          /grasp/execute 服务（阻塞）
                                          ▼
                         手眼外参 + 图像时刻 TCP → 基座系目标
                                          ▼
                         开爪 → 预抓取 → 抓取 → 闭爪 → 抬起
```

- **一个框一个目标**：框由 YOLO/OBB/人工给出，本包不做检测。
- **GraspNet 后端**：当前是 C++ 几何后端（平面去除 + 聚类 + 桌面约束 OBB + 力闭合候选），
  输出 GraspNet 风格的 6-DoF 抓取（`z=approach`、`y=闭合轴`、`x=y×z`）。
  官方 GraspNet 网络需要 `checkpoint-rs.tar` / `pointnet2_ops` / CUDA 工具链，
  本机暂不具备；`backend` 参数保留为后续替换点。
- **纯 C++ 实现**：Eigen（线性代数）、OpenCV（图像/轮廓）、yaml-cpp（标定外参），
  核心算法集中在 `geometry.cpp` / `transforms.cpp`，可被 gtest 单测。
- **手眼外参**：执行节点启动时读取标定 YAML，用 **图像时间戳对应的 TCP** 做变换，
  避免“检测后机械臂动过”导致外参错位。

## 节点与接口

| 节点 | 入口 | 作用 |
| ---- | ---- | ---- |
| `grasp_estimator` | `grasp_estimator` | 订阅 RGB-D + 框，发布最高分抓取 |
| `grasp_executor` | `grasp_executor` | 读手眼，服务触发运动 + 夹爪 |

| 方向 | 名称 | 类型 | 说明 |
| ---- | ---- | ---- | ---- |
| 订阅 | `/camera/image_raw` | `sensor_msgs/Image` | 与框对应的 RGB |
| 订阅 | `/camera/depth/image_raw` | `sensor_msgs/Image` | 深度（32FC1 米；16UC1 用 `depth_scale_m`） |
| 订阅 | `/camera/camera_info` | `sensor_msgs/CameraInfo` | 内参 |
| 订阅 | `/grasp/target_box` | `sensor_msgs/RegionOfInterest` | 像素框 `x_offset,y_offset,width,height` |
| 发布 | `/grasp/best_grasp` | `interfaces/GraspCandidate` | 相机系最高分抓取 + score/width/源框 |
| 发布 | `/grasp/best_pose` | `geometry_msgs/PoseStamped` | 只要位姿时用（RViz/调试） |
| 订阅 | `/arm_tcp_state` | `geometry_msgs/PoseStamped` | TCP 反馈（用于时间戳对齐） |
| 订阅 | `/target_pose_status` | `std_msgs/String` | 仿真 MoveIt 桥的执行状态 |
| 发布 | `/target_pose` | `geometry_msgs/PoseStamped` | **仿真运动接口**（正式系统应换 L6 action） |
| 发布 | `/gripper_controller/joint_trajectory` | `trajectory_msgs/JointTrajectory` | 仿真夹爪 |
| 订阅 | `/joint_states` | `sensor_msgs/JointState` | 闭爪后检查是否夹空 |
| 服务 | `/grasp/execute` | `interfaces/ExecuteGrasp` | 用最近候选执行抓取，阻塞返回结果 |

## 坐标与手眼

标定文件约定 `T_tcp_cam`（`p_tcp = T · p_cam`）：

```
T_base_tcp_target = T_base_tcp(image_stamp) · T_tcp_cam · T_cam_grasp
T_base_ee_target  = T_base_tcp_target · inv(T_ee_tcp)
```

- `T_base_tcp(image_stamp)` 来自 `/arm_tcp_state` 缓冲区，按候选 `header.stamp` 找最近帧。
- `T_ee_tcp = T_ee_cam · inv(T_tcp_cam)` 在启动时由 TF 求一次（相机固定在末端）。
- 候选位姿在 `camera_optical_frame`，`source_box` 是触发它的像素框。
- 预抓取沿抓取帧 `-z` 后退 `pregrasp_distance_m`；抬起沿 `-z` 后退并叠加基座 `+Z`
  `lift_height_m`。

## 运行（仿真）

```bash
# 0) 依赖台账：Eigen/OpenCV/yaml-cpp 已在根 readme.md
source /opt/ros/humble/setup.bash
source /home/xu/arm_sim/install/setup.bash

# 1) 接口
cd /home/xu/life_robot/interfaces
colcon build --packages-select interfaces --symlink-install
source install/setup.bash

# 2) 应用
cd /home/xu/life_robot/applications
colcon build --packages-select graspnet_pick --symlink-install
source install/setup.bash

# 3) 仿真（单独终端；需要 MoveIt 提供 /target_pose）
cd /home/xu/arm_sim
ros2 launch agx_sim sim.launch.py gui:=false rviz:=false moveit:=true

# 4) 把相机移到能看见桌面的观察位姿（仿真辅助脚本）
cd /home/xu/life_robot
source /opt/ros/humble/setup.bash
source /home/xu/arm_sim/install/setup.bash
python3 applications/src/graspnet_pick/test/sim_move_to_view.py \
  --camera-position 0.20,0.0,0.40 --look-at 0.45,0.0,0.10

# 5) 启动抓取应用
source interfaces/install/setup.bash
source applications/install/setup.bash
ros2 launch graspnet_pick graspnet_pick.launch.py use_sim_time:=true

# 6) 用颜色框代替 YOLO（测试辅助）
python3 applications/src/graspnet_pick/test/sim_color_box_publisher.py --color red

# 7) 看候选，然后触发抓取
ros2 topic echo /grasp/best_grasp --once
ros2 service call /grasp/execute interfaces/srv/ExecuteGrasp \
  "{max_age_s: 5.0, timeout_s: 60.0}"
```

`/grasp/execute` 会阻塞到动作结束；返回 `success`、`error_code`、`message`、`score`。
候选超过 `max_age_s`、图像与 TCP 时间差超限、抓手闭空都会失败并给出原因。

## 关键参数

全部参数在 `launch/graspnet_pick.launch.py` 中赋值，节点内只留安全默认值。

| 参数 | 默认 | 说明 |
| ---- | ---- | ---- |
| `backend` | `geometric` | 当前唯一后端 |
| `depth_scale_m` | `0.001` | 16UC1 深度毫米→米；32FC1 自动按米 |
| `max_pair_dt_s` | `0.15` | RGB/depth 最大时间差 |
| `stride` | `2` | 反投影降采样步长 |
| `plane_distance_m` | `0.012` | 桌面平面阈值；仿真中同时去掉 ArUco 底板 |
| `plane_max_normal_angle_rad` | `0.436` | 只删除与基座 +Z 接近的平面，避免删物体侧面 |
| `cluster_eps_m` | `0.008` | 物体聚类半径 |
| `voxel_m` | `0.003` | 点云体素 |
| `min_width_m` / `max_width_m` | `0.005` / `0.09` | 夹爪可抓宽度 |
| `min_score` | `0.25` | 最高分低于此不发布 |
| `extrinsics_yaml` | `~/life_robot/config/calibration/camera_tcp_extrinsics.yaml` | 手眼外参 |
| `finger_length_m` | `0.0` | 机器人 TCP 到指腹的额外前伸；仿真 `/arm_tcp_state` 已在指根/接触中心，保持 0 |
| `pregrasp_distance_m` / `retreat_distance_m` | `0.08` / `0.08` | 预抓取/后退 |
| `lift_height_m` | `0.05` | 抬起叠加基座 +Z |
| `gripper_max_opening_m` | `0.095` | 夹爪最大开口 |
| `gripper_joint_limit_m` | `0.05` | 单指行程（仿真 prismatic） |
| `gripper_settle_s` | `1.5` | 必须大于 `gripper_move_time_s`，否则会在夹爪未闭合时就读反馈 |
| `min_grasp_opening_m` | `0.005` | 闭爪后小于此视为夹空 |
| `prefer_vertical_approach` | `true` | 候选/当前偏航/水平侧抓都失败后再尝试垂直下降 |
| `grasp_height_offset_m` | `0.0` | 抓取点沿基座 +Z 的额外偏移；接触点偏低时向上调 |

## 安全与边界

- 仿真运动走 `/target_pose`（MoveIt 桥），**不发布 `/control/*`**；真机正式运行必须经 L6 action。
- 夹爪直接发 `JointTrajectory`，仅用于当前仿真；真机由 L6/驱动适配。
- 规划失败会按“水平侧抓 → 候选朝向 → 保持当前偏航 → 当前腕姿 → 垂直”顺序重试；
  全部失败时开爪并返回 `ERROR_MOTION`。
- 闭爪后读 `/joint_states`，开口 < `min_grasp_opening_m` 判定夹空并开爪返回失败。
- 服务阻塞期间用多线程执行器保证 TCP/状态回调继续运行；同一时间只允许一个抓取。
- 没有碰撞场景注入、没有力控/滑移检测；真机前需接 L6 + 力矩/夹爪反馈。
- **仿真接触状态**：已跑通框→候选→服务→运动→夹爪→夹空检测。仿真夹爪的接触/抓取点偏置
  仍需按夹爪模型与实物继续整定（`finger_length_m`、`grasp_height_offset_m`、朝向重试顺序）；
  不要用当前仿真的夹持成功率当验收指标。

## 测试

```bash
cd /home/xu/life_robot/applications
source /opt/ros/humble/setup.bash
source /home/xu/life_robot/interfaces/install/setup.bash
colcon test --packages-select graspnet_pick --event-handlers console_direct+
colcon test-result --verbose
```

覆盖：深度反投影、平面去除（含法向约束）、聚类、PCA/桌面 OBB、抓取采样、
手眼链式矩阵、标定 YAML 读取。

## 迁移目标

| 当前 | 目标层 | 说明 |
| ---- | ---- | ---- |
| `geometry.cpp` + `grasp_estimator` | L3 `perception/` | 抓取位姿估计属于感知；届时话题不变 |
| `/target_pose` + 夹爪话题 | L6 `motion/` | 用 L6 action 替换仿真桥，执行节点只调 action |
| `sim_color_box_publisher.py` / `sim_move_to_view.py` | `tools/` 或仿真辅助 | 仅测试，不随正式 bringup 启动 |
