# L2 接口契约层（Interfaces）

> 索引：[`commen_readme.md`](../../commen_readme.md) · **唯一真源**：[`interfaces/readme.md`](../../interfaces/readme.md)

## 定位

全系统唯一的跨模块接口仓库：只回答「数据长什么样」，不回答「谁来算」。
包名固定 `interfaces`，全系统只允许一个。

## 做

- 定义跨 ≥2 个模块的 `msg/srv/action`；规定字段语义、单位、坐标系、命名、默认值。
- 维护话题/服务/动作契约登记表（`interfaces/readme.md` 第 7 节）。
- 保证编译期类型安全与兼容性（只加字段，不删/不改语义）。

## 不做

- ❌ 节点、launch、参数、RViz、算法。
- ❌ 单节点内部临时结构。
- ❌ 复制 `agx_arm_msgs` 等厂商消息；需要时 `depend`。
- ❌ 依赖业务包；被 L1–L7 反向修改。
- ❌ 发布者名字、日志字符串等元信息。

## 策略：标准消息优先

1. 能用标准消息就用（`geometry_msgs`、`sensor_msgs`、`diagnostic_msgs`、`trajectory_msgs`、`std_srvs`、`vision_msgs` 等）。
2. 能复用驱动 `/control/*`、`/feedback/*` 就不新增。
3. 只有满足任一条才自定义：
   - 标准消息无法一次原子表达（目标+力+超时+优先级+任务 ID）；
   - 需要强类型枚举/状态机；
   - 数组化批语义会丢字段；
   - 跨机/跨进程需要稳定 ABI。

## 接口生命周期

- **新增**：走 `interfaces/readme.md` 9 步；必须登记 `INTERFACE_FILES`、补 `package.xml` 依赖、编译验证、更新第 7 节、下游重编。
- **修改**：✅ 只加字段且默认值合理；❌ 删字段/改类型/改单位/改坐标系/复用字段含义。
- **不兼容**：新建 `XxxV2` 或升包版本，通知所有订阅方。
- **废弃**：先标 deprecated + 替代者，下游迁移完再删，登记移除日期。

## 验收

- [ ] `colcon build --packages-select interfaces` 通过。
- [ ] `ros2 interface show interfaces/msg/<Name>` 字段/注释正确。
- [ ] Python 可 `from interfaces.msg import <Name>`。
- [ ] 第 7 节已登记（名称/类型/话题/方向/日期）。
- [ ] 下游已补 `interfaces` 依赖并在同一环境重编。
- [ ] 包内无节点/launch/算法/参数。

## 常见错误

| 错误 | 后果 | 正确 |
| ---- | ---- | ---- |
| 未登记 `INTERFACE_FILES` | 不生成 | 登记 |
| 缺 `package.xml` 依赖 | 类型未定义 | 补 `<depend>` |
| 两个工作区各有一个 `interfaces` | 串包 | 只保留一个 |
| 改字段不升版本 | 下游崩溃/错位 | V2 或升版本 |
| `float64` 传枚举 | 类型不匹配 | `uint8/uint16` |
