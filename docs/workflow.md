# life_robot 开发流程

> 入口：[`../commen_readme.md`](../commen_readme.md)。目标是「读文档 → 改契约 → 写代码 → 验证 → 回写文档」闭环。

---

## 1. 闭环

```
定层/读文档 → 改契约（若跨层）→ 写实现 + 单测 → mock → 仿真 → 真机 → 更新文档/索引/台账 → 提交
```

硬规则：**文档/索引/台账未同步 = 未完成。**

---

## 2. 改动前自检

1. 属于哪一层？对照 `docs/layers/` 的「做/不做」。
2. 是否跨层？跨层必须走 `interfaces/`，不能 import 其他层内部模块。
3. 是否新增话题/服务/动作？先走 `interfaces/readme.md` 9 步流程。
4. 是否改变状态所有者？先改总纲 §3.2 与 `docs/architecture.md` §3。
5. 是否新硬件/新依赖？先补层文档；依赖先记录到根 `readme.md`。
6. 是否影响安全链？补 `docs/architecture.md` §5 与安全测试。
7. 是否在目录下写/改代码？更新该目录 `readme.md` 与「代码文件索引」（文件、引用、话题/服务/动作）。
8. 是否安装/下载依赖？记录到根 `readme.md` 台账（版本/用途/完整命令）。

---

## 3. 新增模块

| 类型 | 位置 |
| ---- | ---- |
| 第三方驱动 | `drivers/<hw>_ws/src/<pkg>` |
| 自研协议适配 | `communicate/src/<device>_bridge` |
| 感知 | `perception/src/<pkg>` |
| 世界模型 | `world_model/src/<pkg>` |
| 决策/行为树 | `decision/src/<pkg>` |
| 运动/规划 | `motion/src/<pkg>` |
| 应用 | `applications/src/<app>` |
| 集成 | `bringup/`、`config/`、`sim/`、`tools/` |

清单：

- [ ] 目录 `readme.md` 已写定位/边界/目录/入口/IO/配置
- [ ] 「代码文件索引」已登记（文件、作用、引用、订阅/发布）
- [ ] `package.xml` / `CMakeLists.txt` / `setup.py` 依赖完整
- [ ] 话题/参数/命名符合 `docs/conventions.md`
- [ ] 参数外置，单位/坐标系/时间戳正确
- [ ] 可配置项已参数化：节点声明默认值，`*.launch.py` 赋值（见 `docs/conventions.md` §4）
- [ ] 超时、错误码、取消处理
- [ ] 单测或可重复验证脚本
- [ ] 不越层（尤其不发布 `/control/*`）
- [ ] 新依赖已记录到根 `readme.md`

---

## 4. 接口变更

唯一流程：[`interfaces/readme.md`](../interfaces/readme.md) 9 步。

要点：能复用标准消息/驱动接口就不新增；新增必须登记 `INTERFACE_FILES`、补依赖、编译验证、
更新 `interfaces/readme.md` 第 7 节、下游补 `interfaces` 依赖并重编。

---

## 5. 编译与运行

```bash
source /opt/ros/humble/setup.bash

# 接口
cd /home/xu/life_robot/interfaces
colcon build --packages-select interfaces --symlink-install
source install/setup.bash

# 驱动
cd /home/xu/life_robot/drivers/agx_arm_ws
colcon build --symlink-install
source install/setup.bash

# 目标形态（统一环境）
cd /home/xu/life_robot && ./tools/build.sh && source install/setup.bash
ros2 launch bringup ...
```

> 联调必须同一份 install 环境；不要混用多套工作区。

---

## 6. 测试

| 层级 | 内容 | 手段 |
| ---- | ---- | ---- |
| 接口 | 字段/依赖 | `ros2 interface show`、Python import |
| 单元 | 纯逻辑/边界/异常 | pytest / gtest |
| 组件 | 单节点 IO | `ros2 topic pub`、launch_testing |
| 集成 | 跨层闭环 | 仿真 + 脚本化任务 |
| 硬件在环 | 时延/限幅/急停/恢复 | 真机受控场景 |

联调命令：

```bash
ros2 topic list -t
ros2 topic echo /feedback/joint_states --once
ros2 topic echo /feedback/tcp_pose --once
ros2 topic info -v /control/joint_states
ros2 action list -t
ros2 run tf2_ros tf2_echo base_link tcp
```

涉及控制链路必测：超时停止、取消停止、断连/恢复、急停不复位、超限拒绝。

调试顺序：**mock → 仿真 → 真机**；禁止跳过前两步直接上真机试新控制逻辑。

---

## 7. 文档更新清单

- [ ] 层文档 `docs/layers/L*.md` 的职责/边界/接口
- [ ] 目标目录 `readme.md` + **代码文件索引**
- [ ] `interfaces/readme.md` 第 7 节登记表（若改接口）
- [ ] 根 `readme.md` 依赖台账（若装/下依赖）
- [ ] `commen_readme.md` 目录/状态/变更记录（若架构级改动）

---

## 8. 提交与完成标准

**每次完成一个可验证的修改，必须提交。** 一个逻辑改动一个 commit，不要攒大批。

提交前：

```bash
cd /home/xu/life_robot
git status --short                 # 只应包含预期文件
git add <文件...>
git commit -m "<范围>: <动词> <对象>"
git log -1 --stat                  # 确认提交内容
```

- 消息格式：`<范围>: <动词> <对象>`，例：`motion: add move_to_pose action`、
  `docs: tighten L5/L6 boundary`、`communicate: add plc frame parser`。
- 提交前确认：层文档 + 目录 README + 代码索引 + 依赖台账 + 接口登记已同步。
- **不提交**：`build/ install/ log/`、`__pycache__`、密钥、bag、模型权重、大数据集。
- 第三方驱动工作区按根 `.gitignore` 排除，来源与版本记录在 `drivers/readme.md` 与根 `readme.md`。
- 若根仓库尚无提交，首次提交包含项目骨架与文档。
- AI 在回复「完成」前必须已完成提交；除非用户明确要求不提交。

Definition of Done：

- [ ] 行为符合所在层边界
- [ ] 跨层走契约，接口已登记并验证
- [ ] 参数外置、单位/坐标/时间戳正确；可配置项已在 `*.launch.py` 赋值
- [ ] 超时/取消/失败路径有测试
- [ ] 单元测试通过；涉及控制完成安全测试
- [ ] 层文档 + 目录 README + 代码索引 + 依赖台账 + 总纲状态已同步
- [ ] **已 `git commit`**（提交信息可追溯，未遗漏文件）
- [ ] 未越控制单出口/状态所有权红线
