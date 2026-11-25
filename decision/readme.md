# decision —— 行为编排（行为树）

> 入口：[`commen_readme.md`](../commen_readme.md) · 依赖台账：[`readme.md`](../readme.md)
> 分层：[`docs/layers/L5-decision.md`](../docs/layers/L5-decision.md)

## 定位

用行为树把各种行为（技能节点）串联成完整任务：
顺序、条件、回退、重试、超时、并行、取消、失败恢复。
**决定「先做什么、失败怎么办」，不决定「关节怎么动、图像怎么识别」。**

## 行为 = BT 节点

| 类型 | 作用 | 示例 |
| ---- | ---- | ---- |
| Action | 调用下层，返回成功/失败/运行中 | `MoveToPose`、`GraspObject`、`ReleaseObject` |
| Condition | 条件判断 | `IsObjectVisible`、`IsGrasped` |
| Control | 组合 | `Sequence`、`Fallback`、`Parallel` |
| Decorator | 修饰 | `Retry`、`Timeout`、`RateLimit` |

节点只编排/调用，不实现 IK、轨迹、碰撞、检测、协议；不发 `/control/*`。

## 做

- 任务 action（如 `/decision/execute_task`）：类型/目标/约束/超时/优先级。
- 任务类型 → 行为树 XML；tick 执行；黑板维护参数/中间结果/失败原因。
- 技能节点调用 L6 action、L4 查询、L1 只读状态。
- 控制流/失败恢复/取消与 halt；资源互斥与模式检查。
- 任务状态/进度/当前节点/错误码；树可视化与测试。

## 不做

- ❌ IK/轨迹/碰撞（L6）；发 `/control/*`。
- ❌ 检测/位姿/点云（L3）；协议解析（`communicate/`）。
- ❌ 维护场景状态（L4）；UI（L7）；底层限幅/看门狗（L1/L6）。
- ❌ 业务算法藏进 `if/else`；流程必须在树里。
- ❌ 阻塞 tick；无限重试；吞失败原因。

## IO

输入：任务 action、取消、L4 查询、L6 feedback/result、L1 只读状态、模式、配置。
输出：L6 action、任务状态/feedback/result、告警/诊断、树日志。

## 目录

```
decision/src/decision_bt/
├── task_server.py        # /decision/execute_task
├── bt_engine.py          # 加载树、tick、halt、状态发布
├── blackboard.py         # 黑板 schema
├── nodes/{actions,conditions,decorators}/
├── recovery.py
├── safety.py
├── behavior_trees/*.xml  # 一任务一棵树
├── config/
└── test/
```

## 节点规范

- 命名：Action `<动词><对象>`（`MoveToPose`）；Condition `Is/Has...`；禁止 `Node1/Step2`。
- 异步 action client；`tick()` 快速返回；返回 `SUCCESS/FAILURE/RUNNING`；异常转 `FAILURE`。
- 进入节点重置状态；`halt()` 取消下游、释放资源、可重复调用。
- 超时由 `Timeout`/参数控制；失败原因写黑板 + 诊断。

## 黑板

| Key | 类型 | 生产者 | 消费者 |
| --- | ---- | ------ | ------ |
| `task_id` | string | TaskServer | 所有节点 |
| `target_object_id` | string | TaskServer/L4 | 查询/抓取 |
| `target_pose` | PoseStamped | L4 查询 | MoveToPose |
| `place_pose` | PoseStamped | L4 查询 | MoveToPose |
| `grasp_force_n` | float | TaskServer | GraspObject |
| `timeout_s` | float | TaskServer | Timeout |
| `last_error` | string | 失败节点 | Recovery/Result |

所有 key 在 `blackboard.py` 声明类型/默认值；禁止未声明 key、禁止全局变量代替黑板。

## 控制流

- `Sequence`：任一失败即失败；`Fallback`：失败则尝试下一个。
- `Retry`：仅安全失败，次数有限；`Timeout`：超时 halt 子节点。
- `Parallel`：仅独立动作；禁止无限循环，必须有次数上限或超时。

## 失败与取消

| 失败 | 处理 |
| ---- | ---- |
| 未看到目标 | Fallback 重新查询/观测，有限次数 |
| 不可达 | Fallback 调整预抓取位姿，有限重试 |
| 空抓/滑落 | Retry N 次后上报 |
| 超时 | Timeout → halt → FAILURE |
| 取消 | halt 整树 → 取消下游 → `CANCELED` |
| 急停/硬件故障 | 立即 halt → ABORTED/FAILURE，禁止自动重试 |
| 未知异常 | 捕获 → FAILURE + 诊断 + 上报 |

取消：停止 tick → 递归 halt → 取消未完成 action → 清理黑板 → 返回 `CANCELED`。
`ESTOP/FAULT` 拒绝新任务并 halt；`MANUAL` 拒绝自动任务；同一机械臂资源互斥。

## 任务 action

字段：`task_id`（幂等）、`task_type`、`target`、`destination`、`constraints`、`timeout_s`、`priority`。
feedback：阶段/当前节点/进度/消息；result：成功/失败/取消 + 错误码 + 消息 + 耗时。

## 引擎

BehaviorTree.CPP（C++，Humble/Nav2 生态）或 py_trees（Python，快速原型）。
**一个包只用一种**；XML 与节点解耦，换引擎只换适配层。

## 测试与验收

- [ ] mock 节点可脱离 L6 测树逻辑
- [ ] 成功/失败/重试/超时/回退/取消
- [ ] 故障注入：L6 失败、action 超时、取消无响应、急停
- [ ] 黑板 schema 校验（缺 key/类型错报错）
- [ ] XML 可加载/可视化
- [ ] 仿真/真机跑通一条完整任务
- [ ] 取消后整树停止、下游取消、返回 `CANCELED`
- [ ] 模式约束生效；无 `/control/*`、无 IK/轨迹/感知/协议

## 常见错误

| 错误 | 正确 |
| ---- | ---- |
| 节点里写 IK/轨迹/检测 | 调 L6/L4 |
| `sleep`/同步等待 action | 异步 client + `RUNNING` |
| 无 `halt()` | 递归 halt + 取消下游 |
| 无限重试 | 次数上限 + 超时 |
| 失败只返回 `FAILURE` | 黑板写原因 + 诊断 |
| 全局变量传状态 | 黑板 + schema |
| 每棵树复制流程 | SubTree |
| 超时/速度写死在 XML | 参数化/黑板注入 |
| 用 BT 做底层控制 | 控制归 L6 |

## 代码文件索引

> 当前无手写代码。新增 `src/decision_bt/` 后必须在下表登记；路径相对仓库根目录。

| 文件 | 作用 | 使用到的文件/资源 | 订阅/调用 | 发布/提供 |
| ---- | ---- | ----------------- | --------- | --------- |
| _（暂无）_ | — | — | — | — |
