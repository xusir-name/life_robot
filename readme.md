# life_robot 环境与依赖台账

> 入口：[`commen_readme.md`](commen_readme.md) · 职责：记录**装了什么、什么版本、用什么命令**。
> 任何 AI/开发者新增依赖、下载源码/模型/工具、改系统配置以支持依赖，都必须登记到这里。

---

## 1. 记录规则

- **先查后装**：先确认是否已存在、版本是否兼容；再记录，再安装。
- **每条必须包含**：名称、精确版本/分支/commit、用途、来源、**完整可复制命令**、日期、作用域、验证/回滚、状态。
- **命令可复现**：禁止写「按官方文档安装」。
- **版本锁定**：升级也要记录；失败与回滚也要记录。
- **优先项目级**：能用 `requirements.txt` / `tools/install_deps.sh` / Docker 的，不装系统 Python。
- **脚本同步**：有安装脚本时，文档命令与脚本必须一致。
- **大文件不入库**：模型/数据集/bag 只记录下载命令与存放路径。
- **未记录 = 环境不可复现 = 任务未完成**。

---

## 2. 环境基线

| 项 | 值 |
| -- | -- |
| OS | Ubuntu 22.04.5 LTS |
| ROS 2 | Humble（`/opt/ros/humble`） |
| Python | 3.10（系统） |
| Shell | bash |
| `ROS_DOMAIN_ID` | `1` |
| 构建 | `colcon` + `colcon-common-extensions` |
| 统一环境 | 目标为根目录 `install/setup.bash`；落地前按工作区 source |

---

## 3. 系统依赖（apt）

| 名称 | 版本 | 用途 | 安装命令 | 状态 |
| ---- | ---- | ---- | -------- | ---- |
| `can-utils` | — | CAN 调试（candump/cansend） | `sudo apt update && sudo apt install -y can-utils` | ⬜ 待安装 |
| `ethtool` | — | CAN 网卡查看/配置 | `sudo apt update && sudo apt install -y ethtool` | ⬜ 待安装 |
| `ros-humble-vision-msgs` | 4.1.1 | 感知 2D/3D 检测消息候选 | `sudo apt install -y ros-humble-vision-msgs` | ✅ |
| `ros-humble-realsense2-camera` | 4.58.4 | RealSense 驱动 | `sudo apt install -y ros-humble-realsense2-camera` | ✅ |
| `ros-humble-controller-manager` | 2.54.4 | 控制器管理 | `sudo apt install -y ros-humble-controller-manager` | ✅ |
| `ros-humble-xacro` | 2.1.1 | URDF/xacro | `sudo apt install -y ros-humble-xacro` | ✅ |
| `ros-humble-joint-state-publisher` | 2.4.0 | 关节状态发布 | `sudo apt install -y ros-humble-joint-state-publisher` | ✅ |
| `ros-humble-robot-state-publisher` | 3.0.3 | TF/机器人模型 | `sudo apt install -y ros-humble-robot-state-publisher` | ✅ |
| `ros-humble-cv-bridge` | 3.2.1 | ROS Image ↔ OpenCV | `sudo apt install -y ros-humble-cv-bridge` | ✅ |
| `python3-opencv` | 4.5.4 | ArUco 检测 / PnP / 手眼标定 | `sudo apt install -y python3-opencv` | ✅ |
| `python3-yaml` | 5.4.1 | 标定位姿/外参 YAML 读写 | `sudo apt install -y python3-yaml` | ✅ |
| `ros-humble-moveit` | — | MoveIt2（L6 可选） | `sudo apt install -y ros-humble-moveit` | ⬜ 用 MoveIt 时 |
| `ros-humble-ros2-control` | — | MoveIt/控制器依赖 | `sudo apt install -y ros-humble-ros2-control` | ⬜ 用 MoveIt 时 |
| `ros-humble-ros2-controllers` | — | 控制器实现 | `sudo apt install -y ros-humble-ros2-controllers` | ⬜ 用 MoveIt 时 |

---

## 4. Python 依赖（pip）

| 名称 | 版本 | 用途 | 安装命令 | 状态 |
| ---- | ---- | ---- | -------- | ---- |
| `colcon-common-extensions` | 0.3.0 | colcon 构建 | `pip3 install colcon-common-extensions` | ✅ |
| `empy` | 3.3.4 | ROS 2 构建 | `pip3 install empy==3.3.4` | ✅ |
| `pyAgxArm` | 1.0.0 | AgileX 机械臂 SDK | `git clone https://github.com/agilexrobotics/pyAgxArm.git && cd pyAgxArm && pip3 install .` | ✅ |
| `python-can` | 4.6.1 | CAN 通信 | `pip3 install python-can` | ✅ |
| `numpy` | 1.21.5 | 数值计算 | `pip3 install numpy` | ✅ |
| `scipy` | 1.8.0 | 数值/优化 | `pip3 install scipy` | ✅ |
| `bc-stark-sdk` | — | Revo2 Pro/Touch SDK | `pip3 install bc-stark-sdk` | ⬜ 按需 |

> Humble/Ubuntu 22.04 一般不需要 `--break-system-packages`；其他系统按官方说明并在本表注明。

---

## 5. 下载的源码 / 驱动

| 名称 | 本地路径 | 版本 | 用途 | 下载命令 | 状态 |
| ---- | -------- | ---- | ---- | -------- | ---- |
| `agx_arm_ros` | `drivers/agx_arm_ws/src/agx_arm_ros` | `ros2` @ `747d072` | AgileX ROS2 驱动 | `git clone -b ros2 --recurse-submodules https://github.com/agilexrobotics/agx_arm_ros.git` | ✅ |
| `agx_arm_urdf`（子模块） | `.../agx_arm_description/agx_arm_urdf` | `main` @ `680fa92` | URDF/网格 | `git submodule update --init --recursive` | ✅ |
| `pyAgxArm` | 无固定路径 | 1.0.0 | 机械臂 SDK | 见 §4 | ✅ |
| `realsense-ros` | apt 包 | 4.58.4 | RealSense 相机 | 见 §3 | ✅ deb |
| `livox_ros_driver2` | `drivers/livox_ws/src/livox_ros_driver2`（规划） | 待定 | Livox 雷达 | 收录后补全 | ⬜ |

> 驱动目录/版本/许可证规则见 [`drivers/readme.md`](drivers/readme.md)；自研协议见 [`communicate/readme.md`](communicate/readme.md)。

---

## 6. 待安装 / 待确认

- [ ] `can-utils`、`ethtool`（真机 CAN 联调前）
- [ ] `ros-humble-moveit`、`ros-humble-ros2-control`、`ros-humble-ros2-controllers`（用 MoveIt 前）
- [ ] `bc-stark-sdk`（用 Revo2 Pro/Touch 时）
- [ ] RealSense 型号/固件、Livox 型号/驱动仓库
- [ ] 统一构建脚本落地后补 `tools/install_deps.sh`

---

## 7. 新增记录模板

````markdown
### YYYY-MM-DD ｜ <名称> <版本>

- 用途：
- 来源：
- 安装/下载命令：
  ```bash
  <完整命令>
  ```
- 作用域：系统级 / venv / Docker / 用户级
- 验证命令：
  ```bash
  <命令>
  ```
- 回滚/卸载：
- 备注：踩坑、版本冲突、选择原因
````

---

## 8. 变更记录

| 日期 | 变更 | 说明 |
| ---- | ---- | ---- |
| 2026-11-25 | 建立台账 | 记录环境、依赖、源码、命令与记录规则 |
| 2026-11-25 | 补手眼标定依赖 | `cv_bridge`、`python3-opencv`、`python3-yaml` |
| 2026-11-25 | 精简 | 去重、压缩表格，规则收敛到 §1 |
