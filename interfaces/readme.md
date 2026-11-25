# interfaces —— ROS 2 接口契约层

> 入口：[`commen_readme.md`](../commen_readme.md) · 依赖台账：[`readme.md`](../readme.md)
> 分层：[`docs/layers/L2-interfaces.md`](../docs/layers/L2-interfaces.md)
> **本文档是接口唯一真源。** 文档与代码不一致时，以本文档为准并改代码。

## 1. 定位

全系统唯一的跨模块 `msg/srv/action` 定义仓库。只定义数据结构，不产生运行时行为。

| 做 | 不做 |
| -- | ---- |
| 跨 ≥2 模块的 msg/srv/action | 节点/算法/状态机/行为树 |
| 字段语义、单位、坐标系、命名 | launch/参数/rviz |
| 话题契约登记表 | 驱动私有消息（`agx_arm_msgs` 只 depend） |
| 引入版本与使用方 | 单节点内部临时结构 |

## 2. 结构

```
interfaces/
├── readme.md
├── .gitignore
└── interfaces/                 # 包名固定 interfaces
    ├── CMakeLists.txt          # INTERFACE_FILES / INTERFACE_DEPENDENCIES
    ├── package.xml             # <depend>
    ├── msg/                    # GraspCandidate
    ├── srv/                    # ExecuteGrasp
    └── action/                 # 当前为空
```

- 全系统只允许一个 `interfaces` 包。
- `msg/srv/action/` 的 `.gitkeep` 是占位，不要删。
- **接口文件索引**：新增接口必须在第 7.1 节登记（接口名/文件/类型/使用方）。

## 3. 策略：标准消息优先

1. 能用标准消息就用：`geometry_msgs`、`sensor_msgs`、`diagnostic_msgs`、`trajectory_msgs`、
   `std_srvs`、`vision_msgs`、`control_msgs` 等。
2. 能复用驱动 `/control/*`、`/feedback/*` 就不新增。
3. 只有满足任一条才自定义：
   - 标准消息无法一次原子表达（目标+力+超时+优先级+任务 ID）；
   - 需要强类型枚举/状态机；
   - 数组化批语义会丢字段；
   - 跨机/跨进程需要稳定 ABI。

驱动已提供、**不要重复定义**：`/control/joint_states`、`/control/move_j`、`/control/move_p`、
`/control/move_l`、`/control/move_c`、`/control/move_js`、`/control_enable`、
`/feedback/joint_states`、`/feedback/tcp_pose`、`/feedback/arm_status`、`/feedback/motor_status`。

## 4. 编译与验证

```bash
source /opt/ros/humble/setup.bash
cd /home/xu/life_robot/interfaces
colcon build --packages-select interfaces --symlink-install
source install/setup.bash

ros2 interface list | grep '^interfaces'
ros2 interface show interfaces/msg/<Name>
ros2 interface proto interfaces/msg/<Name>
python3 -c "from interfaces.msg import <Name>; print(<Name>())"
```

挂入已有工作区：软链接 `interfaces/interfaces` 到 `<ws>/src/interfaces`，不要复制。

## 5. 新增/修改接口（9 步）

1. **确认需要**：对照 §3，能用标准消息就不新建。
2. **设计字段**：SI + REP-103；单位后缀；Header 第一行；布尔正语义；枚举 `uint8`+常量。
3. **建文件**：`msg/PascalCase.msg`、`srv/*.srv`、`action/*.action`。
4. **登记** `CMakeLists.txt` 的 `INTERFACE_FILES`（漏登记=不生成）。
5. **补依赖**：`find_package(...)` + `INTERFACE_DEPENDENCIES`。
6. **补 `package.xml`**：`<depend>`（两侧都要有）。
7. **编译验证**：构建 + `ros2 interface show` + Python import。
8. **更新第 7 节登记表**：名称/类型/话题/发布者→订阅者/日期。
9. **打通下游**：下游 `package.xml` + CMake/setup.py 加 `interfaces`，重编；话题用 remap，不写死。

修改：只加字段且默认值合理；删/改类型/改单位/改坐标系/复用语义 = 不兼容 → 新建 V2 或升包版本。

## 6. 字段规范与示例

- 命名 `PascalCase.msg`；字段 `snake_case` + 单位后缀；常量 `UPPER_SNAKE_CASE`。
- 动态数组 `T[]`；定长在注释写预期长度；大批量写清展平顺序。
- 一个 msg 只表达一件事；字段 >15 考虑拆嵌套。
- 不塞发布者名字、日志字符串、时间戳文本。

示例 `msg/GraspCommand.msg`：

```text
std_msgs/Header header          # stamp: 生成时间; frame_id: target_pose 坐标系

uint8 MODE_POSITION_ONLY = 0
uint8 MODE_GRASP = 1
uint8 MODE_RELEASE = 2

uint8 mode
geometry_msgs/PoseStamped target_pose
float32 approach_speed_mps
float32 grasp_force_n
float32 timeout_s
uint8 priority
string task_id
```

## 7. 登记表

### 7.1 自定义接口

| 接口名 | 类型 | 话题/服务/动作 | 发布者 → 订阅者 | 引入日期 | 说明 |
| ------ | ---- | -------------- | --------------- | -------- | ---- |
| `GraspCandidate` | msg | `/grasp/best_grasp` | `grasp_estimator` → `grasp_executor`、外部展示 | 2026-11-25 | 相机系最高分抓取：pose/score/width/source_box/backend |
| `ExecuteGrasp` | srv | `/grasp/execute` | 调用方 → `grasp_executor` | 2026-11-25 | 用最近候选执行抓取；阻塞返回 success/error_code/message/score |

### 7.2 已确认使用的标准消息

| 话题/服务 | 类型 | 方向 | 提供方 |
| --------- | ---- | ---- | ------ |
| `/control/move_p`、`/control/move_l` | `geometry_msgs/PoseStamped` | 上层 → 驱动 | `agx_arm_ros` |
| `/control/move_c` | `geometry_msgs/PoseArray` | 上层 → 驱动 | `agx_arm_ros` |
| `/control/move_j`、`/control/joint_states` | `sensor_msgs/JointState` | 上层 → 驱动 | `agx_arm_ros` |
| `/feedback/joint_states` | `sensor_msgs/JointState` | 驱动 → 上层 | `agx_arm_ros` |
| `/feedback/tcp_pose` | `geometry_msgs/PoseStamped` | 驱动 → 上层 | `agx_arm_ros` |
| `/feedback/arm_status`、`/feedback/motor_status` | `diagnostic_msgs/DiagnosticArray` | 驱动 → 上层 | `agx_arm_ros` |
| `/control_enable` | `std_srvs/SetBool` | 上层 → 驱动 | `agx_arm_ros` |
| `/joint_states`、`/tf` | `sensor_msgs/JointState`、`tf2_msgs/TFMessage` | 全局 | `robot_state_publisher` |
| `/grasp/best_pose` | `geometry_msgs/PoseStamped` | 感知 → 展示/调试 | `grasp_estimator` |
| `/grasp/target_box` | `sensor_msgs/RegionOfInterest` | 检测 → 抓取估计 | YOLO/OBB/人工 |
| `/planning_scene`、`/move_action` | `moveit_msgs/*` | MoveIt 生态 | MoveIt2 |

## 8. 兼容性红线

- ✅ 只能加字段，默认值合理（`0`/`false`/空数组不报错）。
- ❌ 删字段、改类型、改语义、改单位、改坐标系。
- 不兼容：新建 `XxxV2` 或升 `package.xml` 版本，通知所有订阅方，登记备注。
- 同一字段不在不同版本复用不同含义。

## 9. 常见坑

| 现象 | 原因 | 解决 |
| ---- | ---- | ---- |
| 文件加了但 `ros2 interface show` 找不到 | 未登记 `INTERFACE_FILES` | 步骤 4 |
| 编译报类型未定义 | 缺 `package.xml`/`INTERFACE_DEPENDENCIES` | 步骤 5/6 |
| 下游找不到 Config.cmake | 没 `find_package(interfaces)` / 没 source | 步骤 9 |
| Python `ModuleNotFoundError` | 没 source / `setup.py` 未声明 | 步骤 9 |
| 改 msg 后下游异常 | 下游没重编 | `colcon build --packages-up-to <下游>` |
| 两个同名 `interfaces` 包 | 工作区重叠 | 只保留一个 |
| `float64` 传枚举 | 类型错 | 用 `uint8/uint16`；浮点写 `.0` |

## 10. 提交前自检

- [ ] 确实跨模块，不是单节点内部结构
- [ ] 能用标准消息/驱动接口的没有自造
- [ ] Header 第一行；单位后缀；枚举常量注释
- [ ] `INTERFACE_FILES` / `INTERFACE_DEPENDENCIES` / `package.xml` 已更新
- [ ] 编译通过，`ros2 interface show` 正确，Python 可导入
- [ ] 第 7 节已登记（名称/类型/话题/方向/日期）
- [ ] 下游已补依赖并重编
- [ ] 未引入节点/launch/参数/算法；未复制厂商消息
