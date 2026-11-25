# bringup —— 启动编排与健康门禁

> 入口：[`commen_readme.md`](../commen_readme.md) · 依赖台账：[`readme.md`](../readme.md)
> 分层：[`docs/layers/L8-integration.md`](../docs/layers/L8-integration.md)

## 定位

按版本/场景把其他目录的节点**按依赖顺序拉起**，确认前一步成功后再拉起下一步。
**不写节点，只决定：起哪些、按什么顺序、起没起来、起不来怎么办。**

## 核心概念

| 概念 | 含义 |
| ---- | ---- |
| Profile | 版本/场景的节点组合：`arm_only/teleop/perception_only/full_auto/sim/debug` |
| Stage | 有共同前置条件的一组节点 |
| Readiness | 进程+节点+接口+首帧/心跳+TF，按节点类型定义 |
| Gate | 前一阶段全就绪才进入下一阶段 |
| Failure Policy | `fail-fast/retry/degrade/abort` |
| Supervision | 启动后存活、心跳、崩溃循环检测 |

## 做

- 版本化启动：profile 决定节点组合，不复制代码。
- 依赖排序、启动前预检、就绪判定、失败处理、回滚。
- 进程看护、顺序关闭、启动报告。
- 命名空间/remap/profile 参数/环境配置。
- 一键启动/停止。

## 不做

- ❌ 节点/算法/驱动/协议实现。
- ❌ 发 `/control/*`、直接控制硬件。
- ❌ 替各层做运行时业务决策。
- ❌ 隐藏失败、无限 respawn 掩盖崩溃。
- ❌ 把设备参数写死在 launch。
- ❌ 修改其他包代码适配启动（用参数/remap）。
- ❌ 用 `sleep 10` 代替就绪检查。
- ❌ 长期保留临时调试脚本作为正式入口。

## 启动成功 = 什么

进程活着 ≠ 成功。按类型选最小条件：

| 检查 | 适用 |
| ---- | ---- |
| 进程存活 | 所有 |
| 节点注册（`ros2 node list`） | 所有 ROS 节点 |
| 关键 publisher / 首帧 | 驱动、感知、运动 |
| 关键 service 可发现/调用 | 世界模型、使能 |
| 关键 action server 可发现 | 运动、决策 |
| 关键 TF 可查询 | 驱动、RSP |
| lifecycle `active` | 使用 lifecycle 的节点 |
| 心跳/健康话题 | 关键长期节点 |

建议：

| 节点 | 最小就绪 |
| ---- | -------- |
| `robot_state_publisher` | 节点 + `/tf_static` + 收到 `/joint_states` |
| 机械臂驱动 | 节点 + `/feedback/joint_states` 首帧 + `arm_status` 无 ERROR |
| 运动执行器 | 节点 + 关键 action + 能读最新反馈 |
| 相机/雷达 | 节点 + 图像/点云首帧 + 时间戳有效 |
| 感知 | 节点 + 输入就绪 + 输出首帧（可空结果） |
| 世界模型 | 节点 + 查询 service |
| 决策 | 节点 + 任务 action + 依赖接口可达 |
| 应用 | 节点 + 能连 L5 任务 action |

判定只读、无副作用；禁止为检测发控制指令。

## 阶段

```
0 Preflight → 1 Core/Hardware → 2 Motion → 3 Perception+World → 4 Decision → 5 Applications → 6 Monitoring
```

| Stage | 内容 | 失败策略 |
| ----- | ---- | -------- |
| 0 | 环境/配置/设备/端口/标定/重复进程 | 中止，不启动 |
| 1 | 驱动、RSP、TF | 重试 N 次，仍失败中止 |
| 2 | 运动执行器 | 中止自动任务，可保留只读 |
| 3 | 感知、世界模型 | 可选失败→降级（禁用自动抓取） |
| 4 | 任务服务、行为树 | 失败→禁用自动任务 |
| 5 | UI/CLI/语音 | 只影响交互 |
| 6 | 诊断/日志 | 告警，不阻断主链 |

- 关键链 fail-fast；增强能力可降级并标记不可用；每阶段产出启动报告。

## Profile

```yaml
profile: full_auto
robot: { arm_type: piper, effector_type: agx_gripper, arm_count: 1 }
stages:
  - name: core
    required: true
    nodes:
      - pkg: agx_arm_ctrl
        exec: agx_arm_ctrl_single
        params: { can_port: can0, arm_type: piper }
        ready: { topics: [/feedback/joint_states, /feedback/arm_status] }
  - name: perception
    required: false
    nodes: []
```

- profile 只描述「起哪些/怎么起/怎么判定」，不写算法。
- 参数赋值统一在 `*.launch.py`；节点声明安全默认值，profile 只提供值。
- 设备参数来自 `config/environments/`；同一代码靠 profile 切换版本，禁止 `bringup_v1/v2` 分叉。

## 抗风险

- **Preflight**：ROS 环境、包/配置/标定、设备、端口、重复进程、日志磁盘、仿真标志、时间同步。
- **重试**：`startup_timeout_s`、`max_attempts`、`backoff_s`；重试前重新检查依赖。
- **失败策略**：预检失败→中止；驱动/运动失败→中止或只读；感知/世界模型失败→降级；
  决策失败→禁用自动任务；应用失败→告警；启动后崩溃→有限重启，超阈值→降级/中止。
- **回滚/停止**：中途失败按已启动逆序停止；Ctrl-C 先取消 L5 任务→应用→决策→运动→感知→驱动。
- **看护**：有限 respawn；统计单位时间重启次数，超阈值判崩溃循环；关键节点退出触发上层降级。
- **报告**：profile、起止时间、每节点就绪耗时/状态/原因、每阶段重试、降级项、退出码。

## 目录

```
bringup/src/robot_bringup/
├── launch/{robot,stage_core,stage_motion,stage_perception,stage_decision,stage_apps,stage_monitoring,sim}.launch.py
├── config/profiles/{arm_only,teleop,perception_only,full_auto,sim,debug}.yaml
├── config/gates.yaml
├── scripts/{preflight,wait_for_node,wait_for_topic,wait_for_service,wait_for_action,startup_report}.py
└── test/
```

- `robot_bringup` 是 launch-only 包，不放节点/算法。
- 阶段 launch 可独立运行；profile/gates 是 YAML，可 review/diff。

## 就绪检查实现

- 用 `OnProcessStart/OnProcessExit` + `RegisterEventHandler` 做基础事件。
- 门禁用独立 `ExecuteProcess`（`wait_for_*.py`）：满足返回 0，超时非 0，由 launch 决策。
- 或轻量 `startup_orchestrator` 状态机统一日志/超时/报告。
- 只读、幂等、可重复；不在 launch 主线程长时间阻塞。
- 建议：poll 0.2–0.5 s；驱动超时 10–30 s；算法 5–15 s；attempts 2–3；backoff 1–5 s。

## 配置与关闭

- `ros2 launch robot_bringup robot.launch.py profile:=full_auto`
- 优先级：`包默认 < profile < 环境配置 < launch 覆盖`
- 校验 profile schema：未知字段/缺失必填/类型错误都报错。
- 关闭：停止新任务 → 取消进行中 → 应用 → 决策 → 运动 → 感知 → 驱动 → 监控；
  `SIGINT` → 超时 `SIGTERM` → 最后 `SIGKILL`；重复停止安全。

## 验收

- [ ] profile 一键启动对应版本，参数可覆盖
- [ ] 每阶段有就绪条件，前置未就绪不启动后继
- [ ] 关键链 fail-fast；可选链可降级并标记
- [ ] 启动失败逆序回滚；启停有报告
- [ ] 无代码分叉；无节点/算法实现；无 `/control/*`
- [ ] 降级/回滚/崩溃循环测试通过

## 常见错误

| 错误 | 后果 | 正确 |
| ---- | ---- | ---- |
| 一次性启动所有 | 随机失败 | 分 stage + gate |
| `sleep 10` | 慢机失败/快机浪费 | 轮询就绪 |
| 只看进程存活 | 接口不可用 | 进程+节点+接口+首帧 |
| 无限 respawn | 崩溃被掩盖 | 有限重启+阈值告警 |
| 静默跳过失败 | 半启动 | 报告 + 明确降级 |
| 设备参数写死 launch | 换现场改代码 | 环境配置 |
| bringup 写业务 | 越界 | 只装配/门禁 |
| 不逆序停止 | 上层还在发指令 | 逆序关闭 |
| `kill -9` 收尾 | 资源泄漏 | SIGINT→SIGTERM→强杀 |

## 代码文件索引

> 当前无手写代码。新增 `src/robot_bringup/` 后必须在下表登记；路径相对仓库根目录。

| 文件 | 作用 | 使用到的文件/资源 | 订阅/调用 | 发布/提供 |
| ---- | ---- | ----------------- | --------- | --------- |
| _（暂无）_ | — | — | — | — |
