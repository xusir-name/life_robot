# L6 运动层（Motion）

> 索引：[`commen_readme.md`](../../commen_readme.md) · [`architecture.md`](../architecture.md)

## 定位

把目标（位姿/关节/抓取/放置）变成合法、安全、可执行、可取消的运动。
**全系统唯一允许发布 `/control/*` 的模块**，也是运动安全的第一道软件闸门。

## 做

- 原语 action：`move_to_pose`、`move_linear`、`move_joints`、`move_arc`、`home`。
- 末端原语：`grasp`、`release`、`set_gripper`、`hand_pose`。
- 遥操作接口（仅 `MANUAL` 由 L7 调用）。
- 坐标统一、IK、轨迹规划/时间参数化（优先 MoveIt2）。
- 校验：工作空间、关节限位、速度/加速度、奇异点、碰撞（只读 L4 障碍）。
- 执行与监控：发 `/control/*`、订阅 `/feedback/*`、判断到位/超时/失败。
- 取消/急停：平滑停止并回到可控状态。
- 发布运动状态/诊断。

## 不做

- ❌ 决定抓哪个、先做什么（L5）。
- ❌ 检测/位姿/点云（L3）。
- ❌ 维护场景生命周期（只读 L4）。
- ❌ 解析 UI/自然语言（L7）。
- ❌ 硬件协议/CAN（L1）；不绕过 L1 限幅/看门狗。
- ❌ 假设反馈一定及时。

## IO

输入：L5/L7 运动 action、取消、L4 障碍（可缺省）、`/feedback/joint_states`、`/feedback/tcp_pose`、
`/feedback/*_status`、TF/标定、参数。
输出：`/control/joint_states`、`/control/move_j`、`/control/move_js`、`/control/move_p`、
`/control/move_l`、`/control/move_c`、`/control/hand`；action feedback/result；运动状态/诊断。

## 依赖

- 依赖：L1（控制+反馈）、L2、L4（只读）、TF、MoveIt2（可选）、`agx_arm_msgs`（灵巧手时）。
- 被依赖：L5、L7（MANUAL 遥操作）。
- 禁止：依赖 L5/L7 内部；写 L4。

## 控制单出口

```
L5 ──action──► L6 ──/control/*──► L1
L7 ──action──► L6（仅 MANUAL）
```

- L6 内部可多个规划器，但只有一个组件写 `/control/*`。
- 多目标按优先级/抢占仲裁；禁止两个节点同时写控制话题。

## 执行状态机

```
IDLE → VALIDATING → PLANNING → EXECUTING → SETTLING → SUCCEEDED
              │          │
              ├──► FAILED（不可达/规划失败）
              └──► CANCELED（preempt）
任意状态 ──急停/故障──► ABORTED
```

- `SETTLING` 等位置误差+速度阈值稳定后再判成功。
- `ABORTED` 后回安全状态，禁止自动继续。

## 必须校验

目标可达（IK 有解）、关节限位、速度/加速度、工作空间、碰撞、状态健康（无故障/反馈未过期/已使能）。
任一项失败 → 拒绝 + 明确错误码，不静默降级。

## 取消与到位

- 取消：停止轨迹生成 → 减速/保持 → 等稳定 → 返回 `CANCELED`；禁止走完当前轨迹。
- 急停/故障 → `ABORTED`，不自动恢复。
- 到位阈值/速度/稳定时间参数化；基于 L1 最新反馈并检查新鲜度。
- 抓取成功需宽度/力反馈 + 感知结果（由 L5 综合）。

## 与 MoveIt

- 可用 MoveIt 规划/碰撞，但 **MoveIt 不得绕过 L6 直接发控制话题**。
- `FollowJointTrajectory` 由 L6 桥接，不把 MoveIt 执行器直接暴露给 L5。
- 规划器可替换，action 契约保持稳定。

## 验收

- [ ] `move_to_pose/joints/linear/grasp/release/home` 稳定。
- [ ] 每个 action 支持 feedback/cancel/result，错误码可枚举。
- [ ] 取消后停止并回可控状态，返回 `CANCELED`。
- [ ] 不可达/超限/碰撞被拒且不产生运动。
- [ ] 到位判断不误判；断连/故障停止且不自动继续。
- [ ] 全系统只有 L6 写 `/control/*`（调试除外）。
- [ ] 无任务编排/感知/UI。

## 越界

| 越界 | 正确 |
| ---- | ---- |
| 决定先抓杯子再放盘子 | L5 |
| 订阅相机做检测 | 用 L4/L5 目标 |
| 多臂无仲裁直接写控制 | 实例命名空间 + 仲裁 |
| MoveIt 直接控制驱动 | 经 L6 |
| 取消只停本地定时器 | 停实际运动 |
| 到位只看目标关节角 | 反馈误差 + 稳定时间 |
