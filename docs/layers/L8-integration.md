# L8 集成层（Integration）

> 索引：[`commen_readme.md`](../../commen_readme.md) · **启动规范**：[`bringup/readme.md`](../../bringup/readme.md)

## 定位

把各层装配成可启动、可配置、可仿真、可诊断的系统。**只做组合，不生产算法。**

## 做

- `bringup/`：profile、分阶段 launch、就绪门禁、失败策略、回滚、关闭。
- `config/`：跨层参数、标定出口、环境配置（CAN/IP/序列号）。
- `sim/`：仿真世界、mock 驱动、场景与回归。
- `tools/`：统一构建、依赖检查、录包/回放。
- 可观测：诊断聚合、日志、RViz/Foxglove 配置。
- 部署：Docker/CI、版本锁定、安装说明。

## 不做

- ❌ 感知/决策/运动/控制/协议算法。
- ❌ 发 `/control/*`（mock 驱动除外且需明确标记）。
- ❌ 替各层做运行时业务决策。
- ❌ 把某层私有参数复制到全局；只做组合/覆盖。
- ❌ launch 里写复杂逻辑。

## 目录

```
bringup/src/robot_bringup/{launch, config/profiles, config/gates.yaml, scripts, test}
config/{calibration, robot, environments}
sim/{worlds, models, mock_drivers, scenarios}
tools/{build.sh, check.sh, record.sh, replay.sh}
```

## 启动阶段

| Stage | 内容 | 失败策略 |
| ----- | ---- | -------- |
| 0 Preflight | 环境/配置/设备/端口/标定/重复进程 | 中止，不启动 |
| 1 Core | 驱动、RSP、TF | 重试 N 次，仍失败中止 |
| 2 Motion | 运动执行器 | 中止自动任务，可保留只读 |
| 3 Perception | 感知、世界模型 | 可选失败→降级（禁用自动抓取） |
| 4 Decision | 任务服务、行为树 | 失败→禁用自动任务 |
| 5 Applications | UI/CLI/语音 | 失败只影响交互 |
| 6 Monitoring | 诊断/日志 | 告警，不阻断主链 |

- **就绪 ≠ 进程活着**：进程 + 节点注册 + 关键接口 + 首帧/心跳 + TF（按节点类型）。
- 前置未就绪不启动后继；禁止 `sleep` 代替检查。
- 启动失败逆序回滚；关闭逆序：应用→决策→运动→感知→驱动。

## 参数优先级

`包默认 < 层配置 < profile < 环境配置 < launch 覆盖`

- 环境相关只在 `config/environments/`；标定只读并校验版本。
- 参数赋值统一在 `*.launch.py`；节点只声明安全默认值，profile/YAML 作为 launch 的输入。
- 多实例用 namespace + remap，不改代码。

## 仿真

- mock 驱动话题/类型/QoS/频率与真实一致，可注入延迟/丢帧/故障。
- 同一套 L3–L7 代码接仿真；差异只在 L1/mock 与 L8 配置。
- 每个关键任务至少一个仿真场景 + 断言脚本。

## 构建与检查

```bash
./tools/build.sh    # 覆盖 interfaces、drivers、communicate、各层、bringup、sim
./tools/check.sh    # 依赖/接口登记/重复包/硬编码
```

## 验收

- [ ] profile 一键启动对应版本，参数可覆盖。
- [ ] 每阶段有就绪条件，前置未就绪不启动后继。
- [ ] 关键链 fail-fast；可选链可降级并标记不可用。
- [ ] 启动失败逆序回滚；启停都有报告。
- [ ] 多实例靠 namespace/remap。
- [ ] mock 可注入故障；集成测试可自动跑通一个场景。
- [ ] launch 无算法逻辑、无硬编码环境参数。

## 越界

| 越界 | 正确 |
| ---- | ---- |
| launch 写 IK/感知 | 放对应层 |
| 私有参数抄进全局 config | 覆盖/remap，单一来源 |
| 仿真/真机两套代码 | 只换 L1/mock 与配置 |
| 调试脚本被 bringup 调用 | 独立，默认不启动 |
| 多实例改代码区分 | namespace + remap |
