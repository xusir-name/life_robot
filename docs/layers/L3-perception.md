# L3 感知层（Perception）

> 索引：[`commen_readme.md`](../../commen_readme.md) · [`architecture.md`](../architecture.md)

## 定位

把传感器数据变成**带时间戳、坐标系、置信度**的观测：看到什么、在哪、多确定。
不回答「该怎么做」。

## 做

- 相机/点云/IMU 标定：内参、外参、手眼；结果落盘 `config/calibration/`。
- 2D/3D 检测、分割、分类、关键点、6D 位姿。
- 点云滤波/配准/支撑面/障碍聚类。
- 短时目标跟踪（观测级 `track_id`）。
- 发布标定静态 TF（`base_link → camera_link`）。
- 健康诊断：帧率、延迟、标定有效性、数据是否过期。

## 不做

- ❌ 发 `/control/*`、直接控制任何硬件。
- ❌ 任务决策（抓哪个、先抓哪个、失败怎么办）。
- ❌ IK/轨迹/运动规划。
- ❌ 长期场景记忆/物体生命周期（L4）。
- ❌ 直接给 L6 下发目标（经 L4/L5）。
- ❌ 修改驱动原始语义；增强输出新话题。

## IO

输入：`sensor_msgs/Image`、`CameraInfo`、`PointCloud2`、`Imu`；L1 关节/TCP（眼在手上）；
`/tf`、`/tf_static`；标定文件。

输出（优先标准/社区消息）：

| 输出 | 类型 |
| ---- | ---- |
| 2D 检测 | `vision_msgs/Detection2DArray`（评估后使用） |
| 3D 检测/位姿 | `vision_msgs/Detection3DArray` 或自定义 |
| 分割掩码 | `sensor_msgs/Image` |
| 滤波点云 | `sensor_msgs/PointCloud2` |
| 支撑面/障碍 | 自定义或 `vision_msgs` 扩展 |
| 标定 TF | `tf2_msgs/TFMessage` |
| 健康 | `diagnostic_msgs/DiagnosticArray` |

自定义消息必须走 `interfaces/`：Header 第一行、带 `confidence/frame_id/stamp`、类别用枚举、位姿 `Pose`、尺寸 m。

## 依赖

- 依赖：L1 传感器、L2、TF、标定配置。
- 被依赖：L4（订阅）、L5（经 L4 或只读）、L7（只读展示）。
- 禁止：依赖 L5/L6/L7；给 L6 直接发目标；写 L4。

## 关键约束

- 所有输出带采集时刻 `stamp` 与 `frame_id`；用 `message_filters`/`tf2` 同步，不假设同时到达。
- 标定版本化；失效要上报。
- 允许延迟，但消费方能判断过期；断流/低帧率标 `STALE/UNAVAILABLE`，禁止旧结果冒充新数据。
- 重计算不阻塞回调；单帧耗时超阈值告警。
- 关键算法有离线回放测试；模型/权重不入库，记录路径与版本。

## 验收

- [ ] 标定可重复，启动加载成功。
- [ ] 输出带 `stamp/frame_id/confidence`。
- [ ] `base_link` 下目标位姿误差在约定范围。
- [ ] 断流/低帧率/标定失效有诊断，不输出伪装数据。
- [ ] 有离线回放与至少一个真机场景。
- [ ] 无控制发布、无任务逻辑。

## 越界

| 越界 | 正确 |
| ---- | ---- |
| 感知直接发 `/control/move_p` | 输出观测给 L4/L5，由 L6 执行 |
| 感知维护「杯子还在不在」 | L4 |
| 感知决定先抓哪个 | L5 |
| 旧帧继续发布 | 标记过期并停止 |
| 写死相机外参 | 从 `config/calibration/` 加载 |
