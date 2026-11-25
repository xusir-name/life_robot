# hand_eye_calibration

> 上级：[`applications/readme.md`](../../readme.md) · 分层：[`docs/layers/L7-applications.md`](../../../docs/layers/L7-applications.md)

## 用途

眼在手上（eye-in-hand）手眼标定：

1. 按配置设置多个**相机目标位姿**；
2. 通过运动接口逐个到位；
3. 等待 TCP 稳定、图像新鲜；
4. 检测 ArUco，估计 `T_cam_marker`；
5. **按时间戳对齐**图像、`camera_info`、TCP 位姿；
6. 收集 ≥ `min_samples` 个样本，用 `cv2.calibrateHandEye` 求 `T_tcp_cam`；
7. 写出**完整 YAML**（质量指标/样本明细）和**纯外参 YAML**；
8. 自动回到标定开始前的位姿（`restore_home`，默认开启）。

输出约定：`parent_frame = tcp`，`child_frame = camera_optical_frame`，
`transform_tcp_camera` 为 4×4 行主序矩阵。

## 输入接口

| 话题 | 类型 | 用途 |
| ---- | ---- | ---- |
| `/camera/image_raw` | `sensor_msgs/Image` | ArUco 图像 |
| `/camera/camera_info` | `sensor_msgs/CameraInfo` | 内参与畸变 |
| `/arm_tcp_state` | `geometry_msgs/PoseStamped` | TCP 在 `base_link` 的位姿 |
| `/target_pose` | `geometry_msgs/PoseStamped` | 仿真运动接口（真实系统应替换为 L6 action） |
| `/target_pose_status` | `std_msgs/String` | `accepted/planning/succeeded/failed` |

TF：需要 `motion_target_frame (link6) -> camera_optical_frame` 的静态变换。

所有话题/坐标系名都可以在 launch 里覆盖：

| launch 参数 | 默认 |
| ----------- | ---- |
| `base_frame` | `base_link` |
| `camera_frame` | `camera_optical_frame` |
| `tcp_frame` | `tcp` |
| `motion_target_frame` | `link6` |
| `image_topic` | `/camera/image_raw` |
| `camera_info_topic` | `/camera/camera_info` |
| `tcp_topic` | `/arm_tcp_state` |
| `motion_topic` | `/target_pose` |
| `motion_status_topic` | `/target_pose_status` |

> 本包在 `MANUAL/离线标定` 场景使用 `/target_pose`；正式运行时运动必须经 L6，
> 且本包**不发布 `/control/*`**。

## 运行

```bash
source /opt/ros/humble/setup.bash
source ~/arm_sim/install/setup.bash      # 仿真
cd /home/xu/life_robot/applications
colcon build --packages-select hand_eye_calibration --symlink-install
source install/setup.bash

ros2 launch hand_eye_calibration hand_eye_calibration.launch.py \
  output_yaml:=$HOME/life_robot/config/calibration/hand_eye_calibration.yaml \
  extrinsics_yaml:=$HOME/life_robot/config/calibration/camera_tcp_extrinsics.yaml
```

常用参数：

| 参数 | 默认 | 说明 |
| ---- | ---- | ---- |
| `poses_file` | 包内 `config/target_poses.yaml` | 相机位姿 + look_at |
| `output_yaml` | `~/life_robot/config/calibration/hand_eye_calibration.yaml` | 完整标定报告 |
| `extrinsics_yaml` | `~/life_robot/config/calibration/camera_tcp_extrinsics.yaml` | 纯外参文件 |
| `marker_dict` | `DICT_6X6_250` | ArUco 字典 |
| `marker_id` | `0` | 码 ID |
| `marker_size_m` | `0.16` | 码边长 |
| `hand_eye_method` | `TSAI` | TSAI/PARK/HORAUD/ANDREFF/DANIILIDIS |
| `min_samples` | `6` | 最少有效样本 |
| `max_time_diff_s` | `0.15` | 图像与 TCP 最大时间差 |
| `settle_time_s` | `1.0` | 到位后稳定等待 |
| `move_timeout_s` | `30.0` | 单次运动超时 |
| `restore_home` | `true` | 标定结束/失败后回到开始位姿 |
| `max_sample_error_m` | `0.005` | 单样本一致性阈值，超过则剔除离群 |
| `max_sample_rotation_error_rad` | `0.03` | 单样本旋转一致性阈值 |

## 位姿文件

```yaml
look_at: [0.45, 0.00, 0.127]      # 相机看向的点（base_link）
camera_positions:                # 相机光学系位置（m）
  - [0.12, 0.00, 0.34]
  - [0.14, -0.06, 0.36]
```

相机姿态由 `look_at` 自动生成（z 指向目标，x 向右，y 向下）。
需要显式姿态时可改用 `poses: [{position: [...], look_at: [...]}]`。

## 两个输出文件

**完整报告** `hand_eye_calibration.yaml`：外参 + 质量指标 + 样本明细 + 来源信息。

**纯外参** `camera_tcp_extrinsics.yaml`：只保留可直接使用的变换，无指标/样本：

```yaml
parent_frame: tcp
child_frame: camera_optical_frame
translation_m: [-0.0713, -0.0122, -0.0979]
rotation_quaternion_xyzw: [-0.1232, 0.1232, -0.6957, 0.6969]
rotation_matrix: [[...]]
transform: [[...]]        # 4x4, p_tcp = T * p_cam
```

## 回原位

启动时记录 `base_frame -> motion_target_frame` 的位姿；标定成功、样本不足或异常退出时，
在 `finally` 中自动发送该位姿回 `/target_pose`。设 `restore_home:=false` 可关闭。

## 时间戳对齐

- 只在运动 `succeeded` 且 TCP 稳定后，等待**时间戳晚于当前时刻**的新图像。
- 对每帧图像，在缓冲区中找最近的 `camera_info` 与 `/arm_tcp_state`。
- 两者与图像的时间差都必须 ≤ `max_time_diff_s`，否则丢弃并重试。
- 样本记录 `stamp_diff_s`，最终 YAML 输出 `stamp_diff_max_s`。

## 输出示例

```yaml
hand_eye_calibration:
  type: eye_in_hand
  parent_frame: tcp
  child_frame: camera_optical_frame
  method: TSAI
  samples: 8
  translation_m: [...]
  rotation_quaternion_xyzw: [...]
  rotation_matrix: [[...]]
  transform_tcp_camera: [[...]]
  metrics:
    marker_position_std_m: ...
    marker_rotation_max_error_rad: ...
    stamp_diff_max_s: ...
    reproj_error_max_px: ...
  samples_detail: [...]
```

## 质量判断

- `marker_position_std_m` 越小越好（仿真正常 < 1e-3 m）。
- `marker_rotation_max_error_rad` 越小越好（仿真正常 < 0.05 rad）。
- `reproj_error_max_px` 反映检测/PnP 质量。
- 求解后会按一致性剔除离群样本（`max_sample_error_m` / `max_sample_rotation_error_rad`），
  YAML 中记录 `removed_outliers`。
- 若一致性差：检查 ArUco 是否被夹爪遮挡、时间戳是否对齐、位姿是否足够多样。

## 测试

```bash
cd /home/xu/life_robot/applications/src/hand_eye_calibration
python3 -m pytest -q test/
```

合成数据测试会验证 `solve_eye_in_hand` 能恢复已知 `T_tcp_cam`。
