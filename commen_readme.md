# life_robot 开发总纲（AI 唯一入口）

> **只以本仓库文档为准。** 第三方 README 仅作驱动使用参考，不把外部项目的架构、命名、流程引入本项目。
> **阅读顺序**：本文件 → `docs/layers/Lx-*.md` → 目标目录 `readme.md` → 该目录「代码文件索引」→ 目标文件。
> **单一真源**：架构/边界 = 本文件 + `docs/`；接口 = `interfaces/readme.md`；依赖 = `readme.md`。

---

## 0. AI 自循环协议（每次阅读必须执行）

1. **先定层**：判断改动属于哪一层，读对应层文档与目录 README；越界即停，先改文档再改代码。
2. **顺手体检**：每次阅读时检查重复、过期、矛盾、失效链接；**优先删减/合并**，不追加长文。优化只做有依据的收敛，不制造无意义改写。
3. **闭环同步**：代码/接口/依赖/目录变化 → 同步目录 README、代码文件索引、接口登记、依赖台账（见 §5）。
4. **自包含**：所有规则、路径、命令必须能在 `life_robot/` 内闭环；需要外部信息时，把结论写入本仓库，不只给链接。
5. **记录变更**：架构或规则改动，在本文件 §7 追加一行。
6. **依赖先行**：缺依赖时先查 `readme.md` 台账，再记录、再安装（见 §5.3）。
7. **完成标准**：文档未同步 = 任务未完成。
8. **反重复**：同一规则只在**一个真源展开**，其他处一句话 + 链接；发现重复展开就合并/删除。
9. **体积控制**：单文档 >300 行必须压缩/拆分；目录 README 尽量 <180 行；优先表格与短句，目录 README 只留定位、边界、入口、代码索引。
10. **必须提交**：每次完成一个可验证的修改，必须 `git add` + `git commit`；提交前确认文档/索引/台账已同步；不提交构建产物、密钥、大文件。

---

## 1. 定位

**目标**：面向生活/实验室场景的机器人系统。已确认核心为 AgileX 机械臂 + 夹爪/Revo2 灵巧手；底盘、相机、升降等扩展待定。

**当前阶段**：先把架构、边界、接口、启动编排定清楚，再逐层实现；不堆功能。

**非目标**：不在 `interfaces/` 写节点；不在 `drivers/` 写业务；不在 `applications/` 写算法；不引入第二套外部架构。

---

## 2. 分层与边界

```
L7 applications/  人/UI/语音/遥操作
      │ action/service
L5 decision/      行为树：做什么、顺序、失败恢复
      │ action                 │ 查询
L6 motion/  唯一控制出口        L4 world_model/  场景状态（可按需引入）
      │ /control/*             ▲ 观测
L1 drivers/（第三方） + communicate/（自研协议）   L3 perception/
      │ CAN/USB/串口/网口        ▲ 图像/点云
L0 硬件（臂、手、相机、底盘、急停）

横切：L2 interfaces/（契约，只被依赖） · L8 bringup/sim/config/tools（只装配）
```

| 层 | 目录 | 职责 | 状态 |
| -- | ---- | ---- | ---- |
| L0 | 现场 | 物理设备、急停、安全回路 | 已存在 |
| L1 | `drivers/`、`communicate/` | 协议 ↔ ROS2 话题；驱动看门狗与限幅 | ✅ 驱动已接入；协议规范已定 |
| L2 | `interfaces/` | 只定义跨模块 msg/srv/action | ✅ 骨架，暂无自定义接口 |
| L3 | `perception/` | 标定、检测、位姿、点云；只输出观测 | 规则已定，待实现 |
| L4 | `world_model/` | 场景状态唯一真源；被动查询 | 规则已定，待实现 |
| L5 | `decision/` | 行为树编排、超时/取消/恢复、安全仲裁 | 规则已定，待实现 |
| L6 | `motion/` | IK/轨迹/校验/执行；**唯一发 `/control/*`** | 规则已定，待实现 |
| L7 | `applications/` | 完整运行节点：意图入口与展示 | 规则已定，待实现 |
| L8 | `bringup/ sim/ config/ tools/` | profile、分阶段拉起、就绪门禁、回滚 | 规则已定，待实现 |

### 2.1 依赖铁律

1. 单向：`L7 → L5 → L6/L3 → L1`；L2 被所有层依赖，自己不依赖业务。
2. 只有 **L6** 发布 `/control/*`；调试脚本例外且必须显式标注、默认不启动。
3. L5 不做几何（IK/轨迹/碰撞）；L3 不发控制；L1 不做业务判断。
4. 跨层只走 ROS2 topic/service/action + `interfaces/` 或厂商公开消息；禁止 import 其他层内部模块。
5. 同层模块不互相直调，通过话题或由上层编排。
6. L4 只被动接收/回答查询，不指挥 L5/L6；L6 可只读 L4。
7. L8 只做装配（launch/参数/remap/仿真/工具），不写算法、不替各层决策。
8. 全系统只有一个 `interfaces` 包；`agx_arm_msgs` 等只 `depend`，不复制。

### 2.2 允许 / 禁止依赖

| 调用方 ↓ \ 被调方 → | L1 | L2 | L3 | L4 | L5 | L6 | L7 |
| --- | --- | --- | --- | --- | --- | --- | --- |
| L1 | — | ✅消息 | ❌ | ❌ | ❌ | ❌ | ❌ |
| L3 | ✅只读 | ✅ | — | ❌ | ❌ | ❌ | ❌ |
| L4 | ⚠️只读 | ✅ | ✅订阅 | — | ❌ | ❌ | ❌ |
| L5 | ❌ | ✅ | ✅订阅 | ✅查询 | — | ✅action | ❌ |
| L6 | ✅控制+反馈 | ✅ | ⚠️只读障碍 | ⚠️只读 | ❌ | — | ❌ |
| L7 | ❌ | ✅ | ⚠️展示 | ⚠️展示 | ✅action | ✅仅 MANUAL 遥操作 | — |
| L8 | ✅装配 | ✅ | ✅装配 | ✅装配 | ✅装配 | ✅装配 | ✅装配 |

`⚠️` = 只读数据依赖，不得形成调用/控制回路。

---

## 3. 数据流、状态所有权与安全

### 3.1 端到端（抓取放置）

```
L7 意图 → L5 行为树拆技能 → L6 校验/规划 → /control/* → L1 驱动 → 硬件
                ↑ 查询 L4 ← L3 观测 ← 相机/点云
```

- 人给意图，不给关节角；L5 只编排；L6 唯一控制出口；L1 忠实执行并反馈。

### 3.2 状态唯一所有者

| 状态 | 所有者 | 对外接口 |
| ---- | ------ | -------- |
| 关节/TCP/设备健康 | L1 | `/feedback/*`、`/control_enable` |
| 检测/位姿/点云 | L3 | 感知结果话题（带 stamp/frame_id/confidence） |
| 场景物体与生命周期 | L4 | 查询 service + 场景快照 |
| 任务状态/进度 | L5 | 任务 action feedback/result + 状态话题 |
| 运动执行状态 | L6 | 运动 action feedback/result |
| 模式 `AUTO/MANUAL/ESTOP/FAULT` | L5 仲裁 + L8 配置 | 模式话题 |

**禁止**：同一状态多处缓存后写回；上层自行积分关节角/TCP 替代反馈。

### 3.3 TF 与坐标

| TF 段 | 发布者 |
| ----- | ------ |
| `map → odom` | 定位/导航（有底盘时） |
| `odom → base_link` | 底盘驱动；无底盘由 L8 发静态单位变换 |
| `base_link → 连杆` | `robot_state_publisher`（L8 启动，URDF + `/joint_states`） |
| `tool0 → tcp` | 驱动或 L8 静态 TF，必须与 `tcp_offset` 一致 |
| `base_link → camera_link` | L3 标定模块（静态 TF） |

- REP-103：x 前、y 左、z 上；单位 SI；位姿必须带 `header.frame_id`；跨坐标系由消费方 `tf2` 转换。
- 同一 TF 段只能一个发布者；标定结果放 `config/calibration/`，不写死代码。

### 3.4 安全链（高 → 低）

`硬件急停 > 驱动看门狗/限幅 > L6 可行性校验 > L5 任务互锁 > L7 人机互锁 > L8 监控`

- 失效必须可见（诊断）、必须收敛（停止/降级/上报）；急停/碰撞后禁止自动恢复。
- 任何 action 必须可取消；取消后必须停止实际运动并返回 `CANCELED`。
- 启动默认保守：低速、默认不自动使能（可配置）。

---

## 4. 目录、构建与运行

### 4.1 顶层目录

```
life_robot/
├── commen_readme.md      # 本文件：唯一入口 + AI 协议
├── readme.md             # 环境与依赖台账（安装/下载记录）
├── docs/                 # architecture / conventions / workflow / layers
├── interfaces/           # L2 契约
├── drivers/              # L1 第三方/下载驱动（drivers/readme.md）
├── communicate/          # L1 自研协议适配（communicate/readme.md）
├── perception/           # L3（待建）
├── world_model/          # L4（待建，可延后）
├── decision/             # L5 行为树（decision/readme.md）
├── motion/               # L6（待建）
├── applications/         # L7 完整运行节点（applications/readme.md）
├── bringup/              # L8 启动编排（bringup/readme.md）
├── sim/ config/ tools/   # L8 仿真 / 配置 / 工具（待建）
└── build/ install/ log/  # 构建产物，不入库
```

### 4.2 层内布局

```
<layer>/src/<pkg>/{package.xml, CMakeLists.txt|setup.py, <pkg>/, config/, launch/, test/}
```

- 一个包只做一件事；包名 `snake_case`。
- **可配置项必须参数化**：节点 `declare_parameter` 声明安全默认值，实际值在 `*.launch.py` 赋值；不在代码里硬编码（见 `docs/conventions.md` §4）。
- `drivers/` 保持厂商原状；`interfaces/` 用现有 `interfaces/interfaces` 包。
- 仿真与真机只替换 L1/mock 与 L8 配置，L3–L7 代码不分叉。

### 4.3 构建（当前可用）

```bash
source /opt/ros/humble/setup.bash
cd /home/xu/life_robot/interfaces && colcon build --packages-select interfaces --symlink-install && source install/setup.bash
cd /home/xu/life_robot/drivers/agx_arm_ws && colcon build --symlink-install && source install/setup.bash
```

目标形态：根目录 `tools/build.sh` 统一构建，单一 `install/setup.bash`；落地前跨层联调必须同一环境。

### 4.4 命名空间

- 单臂可用全局 `/control/*`、`/feedback/*`；多臂用 `namespace:=arm_left/right` + remap，**不改代码**。
- 跨层公共话题 `/<模块>/<语义>`；模块私有 `~/...` 或 `/<模块>/internal/...`。
- 节点名 = 包名 + 语义后缀；所有实例差异由 launch 参数注入。

---

## 5. 文档体系与维护（闭环）

| 文档 | 回答 |
| ---- | ---- |
| `commen_readme.md` | 入口、分层、边界、目录、维护规则 |
| `readme.md` | 依赖台账：装了什么、版本、命令、状态 |
| `docs/architecture.md` | 通信/状态/TF/QoS/安全/部署/演进 |
| `docs/conventions.md` | 命名、单位、时间、参数、日志、禁止项 |
| `docs/workflow.md` | 开发闭环、检查清单、测试、DoD |
| `docs/layers/L*.md` | 单层职责/边界/IO/验收 |
| `<dir>/readme.md` | 该目录定位、边界、目录、入口、**代码文件索引** |
| `interfaces/readme.md` | 接口唯一真源 + 登记表 + 新增流程 |

### 5.1 目录 README 与代码文件索引（强制）

- 改哪个目录的代码，就更新哪个目录的 `readme.md`；**先有 README 再写代码**。
- 每个目录维护「代码文件索引」：

| 文件 | 作用 | 使用到的文件/资源 | 订阅/调用 | 发布/提供 |
| ---- | ---- | ----------------- | --------- | --------- |

- 一个手写文件一行，入口用 `★`；路径用**相对仓库根目录**的真实路径；话题/服务/动作写全名与类型。
- 新增/删除/重命名文件，或改变引用、话题、服务、动作时，同步索引。
- 厂商代码（`drivers/`）可按包/入口索引，不要求逐文件。
- **AI 查找顺序**：目录 README → 代码文件索引 → 目标文件；禁止全仓库盲搜猜用途。

### 5.2 接口变更

一律按 `interfaces/readme.md` 的 9 步流程：先判断是否真需要 → 设计字段 → 建文件 → 登记 `INTERFACE_FILES` → 补依赖 → 编译验证 → 更新第 7 节登记表 → 下游补依赖重编。

### 5.3 依赖安装与下载（强制）

- 任何 `apt/pip/git clone/模型/数据集` 安装或下载，必须记录到根目录 `readme.md` 台账。
- 记录：名称、精确版本/分支/commit、用途、来源、**完整命令**、日期、作用域、验证/回滚。
- 先查后装；命令可复现；版本锁定；失败也记录；有 `tools/install_deps.sh` 时同步更新。
- 大文件不入库，只记录下载命令与存放路径。
- **没记录 = 环境不可复现 = 未完成**。

### 5.4 单一真源

- 架构/边界：本文件 + `docs/`；接口：`interfaces/readme.md`；依赖/版本：`readme.md`。
- 冲突时先改真源，再改代码，不允许三处各说各话。

---

## 6. 当前状态与路线

### 6.1 已完成

- 驱动接入（AgileX ROS2，CAN/URDF/MoveIt/夹爪/灵巧手）。
- L7 标定应用：`applications/src/hand_eye_calibration` 已实现眼在手上标定（ArUco + PnP + 时间戳对齐 + YAML 输出），仿真验证通过。
- L7 仿真抓取应用（C++）：`applications/src/graspnet_pick` 已实现“图像框 → 最高分抓取 → 服务触发抓取”，
  手眼外参读取/时间戳对齐/候选朝向重试/夹空检测；估计→服务→运动→夹爪→夹空检测链路已在
  Gazebo + MoveIt 跑通，抓取接触点参数仍需按夹爪模型/实物继续整定。
- 架构、约定、流程、L0–L8 分层文档、各目录 README 与代码索引规则。
- 依赖台账（含已安装/待安装状态与命令）。

### 6.2 推进顺序

| 阶段 | 目标 | 验收 |
| ---- | ---- | ---- |
| P0 | 驱动跑通 | CAN 激活、单臂 launch、`/feedback/*` 可读、RViz/MoveIt 可视化 |
| P1 | 控制闭环 | L6 提供 `move_to_pose/joints/grasp/release/home` action，可取消/超时 |
| P2 | 感知闭环 | L3 输出稳定目标位姿；L4 可查询 |
| P3 | 任务闭环 | L5 行为树打通「抓取-放置」，失败可恢复/上报 |
| P4 | 产品化 | L7 入口 + L8 一键 bringup/仿真/回归 |

### 6.3 待确认（未定前不扩写实现）

- [ ] 是否含移动底盘？导航归 L6 还是独立 `navigation/`。
- [ ] 真实相机型号/安装方式与手眼标定（仿真已跑通眼在手上流程，输出 `config/calibration/hand_eye_calibration.yaml`）。
- [ ] 是否引入 L4，或先由 L5 直连 L3。
- [ ] 单臂/双臂、单机/多机、ROS 2 Humble 是否唯一基线。

---

## 7. 变更记录

| 日期 | 变更 | 说明 |
| ---- | ---- | ---- |
| 2026-11-25 | 建立总纲/分层文档 | L0–L8、目录、依赖铁律、文档体系 |
| 2026-11-25 | 增加 README 同步与依赖台账规则 | 代码文件索引、安装命令必须登记 |
| 2026-11-25 | 文档整体收敛 | 精简重复内容，加入 AI 自循环协议，统一索引与模板 |
| 2026-11-25 | 新增参数化规则 | 可配置项必须参数化，节点声明默认值，`*.launch.py` 统一赋值 |
| 2026-11-25 | 新增 Git 提交规则 | 每次 AI 修改完成后必须提交，提交前同步文档/索引/台账 |
| 2026-11-25 | 新增手眼标定应用 | `applications/src/hand_eye_calibration`，ArUco + PnP + 时间戳对齐 + YAML |
| 2026-11-25 | 标定结束回原位 | 启动记录初始位姿，成功/失败/异常退出均自动恢复 |
| 2026-11-25 | 标定话题可配置 + 纯外参输出 | launch 可改全部话题/坐标系名；额外生成 `camera_tcp_extrinsics.yaml` |
| 2026-11-25 | 新增仿真抓取应用（C++） | `graspnet_pick`：GraspCandidate/ExecuteGrasp 接口、服务触发抓取、手眼与候选朝向重试、夹空检测 |
| 2026-11-26 | 抓取应用加力控/软闭合 | 位置模式闭爪到 `宽度 - squeeze` 不再夹 0；`gazebo_force` 用 Gazebo 恒定力；`force` 用 AgileX `/control/joint_states.effort` |
| 2026-11-26 | 抓取评分对齐下压 | 垂直权重提高到 0.30；垂直候选用框中心表面点下探 `grasp_depth_from_top_m` |
