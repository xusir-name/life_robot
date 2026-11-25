# life_robot 全局约定

> 入口：[`../commen_readme.md`](../commen_readme.md)。本文件只写跨层统一约定；
> 架构规则见 [`architecture.md`](architecture.md)，开发流程见 [`workflow.md`](workflow.md)。

---

## 1. 命名

| 对象 | 规则 | 示例 |
| ---- | ---- | ---- |
| 顶层目录 | 小写下划线 | `world_model/` |
| ROS 2 包 | `snake_case`，语义完整 | `motion_executor`、`object_detector` |
| Python 文件/模块 | `snake_case` | `grasp_planner.py` |
| C++ 文件 | `snake_case.cpp/.hpp` | `trajectory_validator.cpp` |
| 类 | `PascalCase` | `GraspPlanner` |
| 函数/变量 | `snake_case` | `compute_ik()` |
| 常量 | `UPPER_SNAKE_CASE` | `MAX_JOINT_SPEED` |
| 节点 | 包名 + 语义后缀 | `perception_object_detector` |
| msg/srv/action | `PascalCase` | `GraspCommand.msg` |
| 字段 | `snake_case` + 单位后缀 | `approach_speed_mps` |
| 布尔 | `is_` / `has_`，语义为正 | `is_grasped` |
| 枚举 | `uint8` + `UPPER_SNAKE_CASE` 常量 | `MODE_GRASP = 1` |
| 话题 | `/<模块>/<语义>` | `/motion/move_to_pose` |
| 服务 | `/<模块>/<动词或名词>` | `/world_model/get_object` |
| 动作 | `/<模块>/<动宾>` | `/decision/execute_task` |
| 参数 | `snake_case` | `max_joint_speed_rad_s` |

- 驱动既有 `/control/*`、`/feedback/*` 保持原名，用 remap 接入。
- 禁止 `data`、`info`、`msg`、`value` 这类无语义字段名（`DiagnosticArray` 的 `KeyValue` 除外）。

---

## 2. 单位与坐标

### 2.1 单位（SI，字段带单位后缀）

| 物理量 | 单位 | 后缀 |
| ------ | ---- | ---- |
| 长度 | m | `_m` |
| 角度 | rad | `_rad` |
| 线速度 | m/s | `_mps` |
| 角速度 | rad/s | `_radps` |
| 加速度 | m/s² | `_mps2` |
| 力 | N | `_n` |
| 力矩 | N·m | `_nm` |
| 质量 | kg | `_kg` |
| 时间 | s | `_s` |
| 频率 | Hz | `_hz` |
| 百分比 | %（0–100） | `_percent` |

- 禁止 mm/cm/deg 混用；SDK 单位不同时在适配层转换。
- 浮点字段写 `1.0`，不写 `1`。

### 2.2 坐标

- REP-103：x 前、y 左、z 上，右手系；四元数必须归一化，禁止用欧拉角传位姿。
- 所有位姿带 `header.frame_id`；跨坐标系由消费方 `tf2` 转换。
- 常用坐标系：`map`、`odom`、`base_link`、`tool0`、`tcp`、`camera_link`、`camera_optical_frame`。
- `tcp_offset` 只允许一份（驱动配置或 L8 标定配置）。

---

## 3. 时间

- 所有消息带 `header.stamp`；时间戳取采集/生成时刻。
- 使用节点时钟；仿真统一 `use_sim_time`。
- 超时/周期参数化：`timeout_s`、`publish_rate_hz`、`watchdog_timeout_s`。
- 禁止业务代码用 `time.time()` / `sleep()` 做逻辑；超时判断用单调时钟。
- 收到消息先检查时间戳有效性；过期数据丢弃并记录。

---

## 4. 参数与配置

### 4.1 参数化与 launch 赋值（强制）

**能手动配置的值必须参数化；参数的实际赋值统一放在 `*.launch.py`。**

适合参数化：端口/设备 ID、话题名、坐标系、频率、超时、重试次数、速度/加速度/力限、
阈值、路径（模型/标定/配置）、使能开关、算法可调项。

规则：

1. 节点内用 `declare_parameter` 声明参数，给**安全默认值**；禁止对未声明参数 `get_parameter`。
2. `*.launch.py` 是赋值入口：用 `DeclareLaunchArgument` 暴露用户可覆盖项，用
   `Node(parameters=[{...}])` 赋实际值。
3. 环境相关值（CAN 口/IP/序列号）从 `config/environments/` 读入 launch，不写死在代码。
4. 大块/嵌套参数可用 YAML，但必须由 launch 加载并传入；YAML 不是第二套赋值逻辑。
5. 不在回调/算法里硬编码可配置值；新增可调项必须同时补参数声明、launch 赋值、README 说明。
6. 不参数化真正的物理常量/算法不变量；避免无意义参数和“参数爆炸”，相关参数可成组。
7. 启动时校验类型、范围、单位，非法值**启动失败**并给出明确原因。
8. 参数改名/改单位视为接口变更：更新 README、代码索引与下游 launch。

示例：

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    can_port = LaunchConfiguration('can_port')
    speed = LaunchConfiguration('speed_percent')
    return LaunchDescription([
        DeclareLaunchArgument('can_port', default_value='can0'),
        DeclareLaunchArgument('speed_percent', default_value='30'),
        Node(
            package='agx_arm_ctrl',
            executable='agx_arm_ctrl_single',
            parameters=[{
                'can_port': can_port,
                'speed_percent': speed,
                'tcp_offset': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            }],
        ),
    ])
```

### 4.2 参数归属

| 类型 | 位置 |
| ---- | ---- |
| 驱动私有 | `drivers/<ws>/src/<pkg>/config/` |
| 协议适配私有 | `communicate/src/<device>_bridge/config/` |
| 单层算法 | `<layer>/src/<pkg>/config/` |
| 跨层组合 | `bringup/src/robot_bringup/config/` 或 launch 参数 |
| 标定结果 | `config/calibration/` |
| 环境配置（CAN/IP/序列号） | `config/environments/` |

### 4.3 规则

- 不在代码里硬编码速度、偏移、超时、阈值。
- 每个参数写清单位、范围、默认值、影响。
- launch 只装配：选配置、传参、remap，不写算法。
- 新增依赖先记录到根目录 [`readme.md`](../readme.md) 台账，再安装。

---

## 5. 日志与诊断

- 用 ROS 2 logger，不用裸 `print`（脚本工具除外）。
- 级别：DEBUG 细节 / INFO 状态变化 / WARN 可恢复 / ERROR 失败 / FATAL 不可继续。
- 日志带上下文：谁、什么对象、什么值、原因。反例 `ERROR: failed`；正例 `grasp failed, task_id=..., object=cup_1, reason=timeout 5.0s`。
- 高频日志限流（THROTTLE）。
- 健康状态用 `diagnostic_msgs/DiagnosticArray`：`name`、`hardware_id`、`level`、`message`、`values`。
- 复用驱动 `/feedback/arm_status`、`/feedback/motor_status`、`/feedback/gripper_status`、`/feedback/hand_status`。

---

## 6. 错误与超时

| 场景 | 处理 |
| ---- | ---- |
| 服务不存在 | 退避重试，超上限上报，不无限阻塞 |
| action 无反馈 | 超时取消，返回明确原因 |
| 传感器断流 | 标记不可用，降级；禁止旧数据冒充新数据 |
| CAN/通信断连 | 驱动处理并上报；上层不自动发危险指令 |
| 规划失败 | 返回失败码 + 原因；L5 决定重试/换策略 |
| 取消请求 | 停止执行、清理状态、返回 `CANCELED` |

- 失败必须可枚举（错误码/枚举），禁止只返回 `false`。
- 危险动作必须有前置检查与后置确认。

---

## 7. 代码与仓库

- Python：PEP 8、行宽 100、公共接口类型注解、回调短小、不在回调里阻塞。
- C++：C++17、ROS 2 风格、智能指针、回调不抛异常、实时路径不分配内存。
- 不提交 `build/ install/ log/`、`__pycache__`、`*.pyc`、编辑器临时文件。
- 大文件（bag/模型/标定原图）不入库，记录路径与下载命令。
- 第三方代码保留 LICENSE 与来源。

---

## 8. 禁止事项

- ❌ `interfaces/` 放节点/launch/算法/参数。
- ❌ 复制 `agx_arm_msgs` 等厂商消息到 `interfaces/`。
- ❌ 多处缓存同一状态并写回。
- ❌ `std_msgs/String` 拼 JSON 做契约。
- ❌ 用 topic 表达需要成功/失败确认的长任务。
- ❌ 代码里写死话题名、坐标系、速度、TCP 偏移。
- ❌ 旧传感器数据冒充新数据。
- ❌ 静默吞异常/超时。
- ❌ 绕过 L6 直接控制机械臂（调试脚本需显式标注）。
