# drivers —— 第三方 / 下载驱动

> 入口：[`commen_readme.md`](../commen_readme.md) · 依赖台账：[`readme.md`](../readme.md)
> 分层：[`docs/layers/L1-drivers.md`](../docs/layers/L1-drivers.md) · 自研协议：[`communicate/readme.md`](../communicate/readme.md)

## 定位

只放**下载/克隆的第三方 ROS 2 驱动**，保持上游原状。**只翻译，不决策。**

- 适用：设备已有 ROS 2 驱动（`agx_arm_ros`、`livox_ros_driver2`、`realsense-ros`）。
- 不放：业务逻辑、算法、自研协议解析（→ `communicate/`）、自定义消息（→ `interfaces/`）、现场参数（→ `config/`）。

## 布局

```
drivers/
├── readme.md
├── agx_arm_ws/src/agx_arm_ros/        # 工作区 + 上游仓库名
├── livox_ws/src/livox_ros_driver2/    # 待收录
└── realsense_ws/src/realsense-ros/    # 待收录（当前用 apt deb）
```

- 工作区 `<硬件>_ws`，源码 `src/<上游仓库名>`；不重命名上游目录。
- 单包轻量驱动可退化为 `drivers/<repo>/`，但必须在本文件登记。
- 独立 `colcon build`；统一构建落地后纳入根 `install/`。

## 已收录

| 驱动 | 上游 / 版本 | 许可证 | 路径 | 状态 |
| ---- | ----------- | ------ | ---- | ---- |
| AgileX 机械臂 | `agilexrobotics/agx_arm_ros`，`ros2` @ `747d072` | MIT | `agx_arm_ws/src/agx_arm_ros` | ✅ |
| agx_arm_urdf（子模块） | `agilexrobotics/agx_arm_urdf`，`main` @ `680fa92` | 见仓库 | `.../agx_arm_description/agx_arm_urdf` | ✅ |
| Livox 雷达 | `Livox-SDK/livox_ros_driver2`（待定） | 见仓库 | `livox_ws/src/livox_ros_driver2` | ⬜ |
| RealSense | `IntelRealSense/realsense-ros` 4.58.4 | Apache-2.0 | apt deb | ✅ |

### AgileX 包

| 包 | 作用 |
| -- | ---- |
| `agx_arm_ctrl` | 控制节点、launch、CAN 参数 |
| `agx_arm_description` | URDF、RViz、模型 |
| `agx_arm_moveit` | MoveIt2 配置 |
| `agx_arm_msgs` | 厂商私有消息（**不要复制到 `interfaces`**） |

外部依赖：`pyAgxArm`、`python-can`、`scipy`、`numpy`、`can-utils`（详见根 `readme.md`）。
使用文档：`src/agx_arm_ros/README.md`、`docs/CAN_USER.md`、`docs/tcp_offset/`、`test/cmd.md`。
关键参数：`can_port`、`arm_type`、`effector_type`、`tcp_offset`、`speed_percent`、`control_enabled`。
控制话题 `/control/*` 的合法写者是 L6；驱动不判断调用方。

## 新增驱动

- [ ] 确认是驱动，不是算法/业务
- [ ] 建 `<name>_ws/src/<repo>`，保持上游目录名
- [ ] 记录分支 + commit/tag，不只写 `main`
- [ ] 保留 LICENSE
- [ ] 在本文件与根 `readme.md` 登记（版本/来源/命令）
- [ ] 补系统/Python/SDK/权限说明
- [ ] 验证独立编译 + 最小数据收发
- [ ] 更新 `docs/layers/L1-drivers.md`；私有消息只 `depend` 不复制
- [ ] 更新本文件「代码文件索引」

## 版本更新（AgileX）

```bash
cd /home/xu/life_robot/drivers/agx_arm_ws/src/agx_arm_ros
git fetch origin && git checkout ros2 && git pull --ff-only
git submodule update --init --recursive   # 最新：--remote
git log -1 --format='%H %ad %s' --date=short
```

更新后：重编译 → 最小控制验证 → 更新本文件与根 `readme.md` 的 commit → 若话题/参数变化，
更新 `docs/layers/L1-drivers.md` 与 `interfaces/readme.md`。

## 编译

```bash
source /opt/ros/humble/setup.bash
cd /home/xu/life_robot/drivers/agx_arm_ws
colcon build --symlink-install
source install/setup.bash
```

## 红线

- 驱动不判断任务语义、不做 IK/感知、不订阅决策 action。
- 不改厂商代码；必须改时 fork/补丁并登记，保留升级路径。
- 私有消息只 `depend`，不复制到 `interfaces/`。
- 不提交 `build/ install/ log/`、`__pycache__`、大文件。
- 代理镜像只用于加速，登记的上游 URL 必须官方。

## 代码文件索引（按包 / 入口）

> 厂商代码不逐文件；路径相对仓库根目录；新增/删除包或入口、改变话题时同步。

| 文件 / 入口 | 作用 | 使用到的文件/资源 | 订阅/调用 | 发布/提供 |
| ----------- | ---- | ----------------- | --------- | --------- |
| `drivers/agx_arm_ws/src/agx_arm_ros/src/agx_arm_ctrl/agx_arm_ctrl/agx_arm_ctrl_single_node.py` | 单臂控制主节点 | `pyAgxArm`、`agx_arm_msgs`、ROS 参数 | `/control/*`、`/control_enable` | `/feedback/joint_states`、`/feedback/tcp_pose`、`/feedback/*_status` |
| `drivers/agx_arm_ws/src/agx_arm_ros/src/agx_arm_ctrl/launch/start_single_agx_arm.launch.py` | 启动单臂 | 控制节点、description | — | 进程/参数 |
| `drivers/agx_arm_ws/src/agx_arm_ros/src/agx_arm_ctrl/launch/start_single_agx_arm_rviz.launch.py` | 驱动 + RViz | 单臂 launch、URDF | `/feedback/joint_states` 或 `/control/joint_states` | RViz 显示 |
| `drivers/agx_arm_ws/src/agx_arm_ros/src/agx_arm_ctrl/launch/start_single_agx_arm_moveit.launch.py` | 驱动 + MoveIt | 单臂 launch、`agx_arm_moveit` | `/feedback/joint_states` | MoveIt 规划/执行 |
| `drivers/agx_arm_ws/src/agx_arm_ros/src/agx_arm_description/agx_arm_urdf/` | URDF/网格/RViz | — | `/joint_states` | TF/模型 |
| `drivers/agx_arm_ws/src/agx_arm_ros/src/agx_arm_msgs/msg/` | 厂商私有消息 | — | — | `MoveMITMsg`、`HandCmd` 等 |
| `drivers/agx_arm_ws/src/agx_arm_ros/src/agx_arm_moveit/` | MoveIt2 配置 | URDF、SRDF、控制器 | — | 规划场景/轨迹 |
