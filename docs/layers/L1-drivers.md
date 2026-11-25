# L1 驱动层（Drivers）

> 索引：[`commen_readme.md`](../../commen_readme.md) · [`architecture.md`](../architecture.md)
> 第三方驱动：[`drivers/readme.md`](../../drivers/readme.md) · 自研协议：[`communicate/readme.md`](../../communicate/readme.md)

## 定位

把硬件协议翻译成稳定、标准、可观测的 ROS 2 话题/服务。**只翻译，不决策。**

- `drivers/`：下载的第三方 ROS 2 驱动，保持上游原状。
- `communicate/`：自研协议适配，外部设备字节流 ↔ 话题。

## 做

- 连接、初始化、使能/失能、断线重连。
- 定时发布反馈：关节、TCP、设备健康、电机诊断。
- 执行控制话题：关节/位姿/直线/圆弧/夹爪/灵巧手/MIT。
- 底层安全：看门狗、反馈过期、速度/位置/力限幅。
- 厂商错误码 → ROS 诊断；设备参数外置。

## 不做

- ❌ 任务语义（抓不抓、抓哪个）、规划、感知、UI。
- ❌ 跨坐标系业务变换（只按 `tcp_offset` 算 TCP）。
- ❌ 订阅决策 action、反向调用 L5/L6/L7。
- ❌ 把多设备耦合成一个业务节点。

## 接口（AgileX 已接入）

输入：

| 话题 | 类型 |
| ---- | ---- |
| `/control/joint_states`、`/control/move_j`、`/control/move_js`、`/control/move_cpv` | `sensor_msgs/JointState` |
| `/control/move_p`、`/control/move_l` | `geometry_msgs/PoseStamped` |
| `/control/move_c` | `geometry_msgs/PoseArray` |
| `/control/move_mit` | `agx_arm_msgs/MoveMITMsg` |
| `/control/hand`、`/control/hand_position_time` | `agx_arm_msgs/*` |
| `/control_enable` | `std_srvs/SetBool` |

输出：

| 话题 | 类型 | 频率 |
| ---- | ---- | ---- |
| `/feedback/joint_states` | `sensor_msgs/JointState` | `pub_rate`（默认 200 Hz） |
| `/feedback/tcp_pose` | `geometry_msgs/PoseStamped` | `pub_rate` |
| `/feedback/arm_status`、`/feedback/motor_status` | `diagnostic_msgs/DiagnosticArray` | 30 Hz |
| `/feedback/gripper_status`、`/feedback/hand_status` | `diagnostic_msgs/DiagnosticArray` | 30 Hz |
| `/feedback/leader_joint_states` | `sensor_msgs/JointState` | `pub_rate` |

字段/范围/诊断 key 以驱动 README 为准，本文件只记边界。

## 依赖

- 依赖：L0、`pyAgxArm`、L2、`agx_arm_msgs`（厂商私有）。
- 被依赖：L6（控制+反馈）、L3/L5/L7（只读状态）。
- 禁止：依赖业务层、L4；订阅决策 action。

## 关键约束

- **控制权在 L6**：L1 接受 `/control/*`，但不判断谁有资格发；`control_enabled:=false` 可只读。
- **反馈唯一真源**：上层不得积分/估算关节角或 TCP 替代反馈；滤波在 L6/L3 做。
- **失效行为**：

| 事件 | 驱动 | 上层 |
| ---- | ---- | ---- |
| CAN 断开 | feedback lost，停止接受新指令 | L6 取消 action，L5 上报 |
| 反馈恢复 | 低速刷新后尝试重新使能 | 人工确认后恢复任务 |
| 使能失败/电机故障 | 返回错误/诊断 | 停止任务并告警 |
| 反馈过期 | `motor_status` 置 `STALE` | 丢弃过期数据 |

## 验收

- [ ] 单臂 launch 启动，`/feedback/*` 可订阅且时间戳有效。
- [ ] 主要控制话题各有一个可重复调试样例。
- [ ] `tcp_offset` 后 TCP 与 RViz/实测一致。
- [ ] 断连/恢复/使能失败/故障均有可见诊断。
- [ ] `control_enabled:=false` 拒绝控制、仅反馈。
- [ ] 无业务逻辑进入驱动包。

## 常见越界

| 越界 | 正确 |
| ---- | ---- |
| 驱动判断抓取成功 | L6/L5 根据误差、力、感知判断 |
| 驱动做 IK | L6 |
| 驱动订阅任务 action | L6 订阅并转 `/control/*` |
| 复制 `agx_arm_msgs` 到 `interfaces` | `depend` |
