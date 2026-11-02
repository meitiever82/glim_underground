# 阶段二 总纲：RTK-IMU-LO 融合定位 + 语义 VoxelMap + 100m Submap

> 本文只定义阶段二的顶层约定：阶段目标、数据流、跨模块契约、集成验收、与阶段三/四的交接。模块级设计、接口、参数与单模块验收拆到两篇子 spec：
>
> - [2.1 定位子spec：RTK-IMU-LO 融合定位](阶段二_2.1_Spec_RTK_IMU_LO融合定位.md)
> - [2.2 语义地图子spec：语义 VoxelMap + 100m Submap](阶段二_2.2_Spec_语义VoxelMap_100mSubmap.md)
>
> 2.1 的定位升级路径（位置级 RTK → GNSS 原始观测紧耦合）见 [GNSS 原始观测紧耦合总方案](../../gnss/specs/2026-10-31-gnss-tight-coupling-master-plan.md)，本文不重复其内容。

## 1. 阶段目标

在现有 GLIM + RTK Odometry 插件基础上，完成以下能力闭环：

- **定位轨迹**（2.1）：RTK / IMU / 激光里程计融合，输出实时 6DoF 轨迹；
- **语义 VoxelMap**（2.2）：从 Point Cloud Submap 升级为带地面/非地面语义的体素地图；
- **100m Submap 管理**（2.2）：以车体为中心维护局部语义子地图；
- **退化保护**（2.1）：RTK 退化、激光退化时可降级运行，不跳变；
- **为阶段三预留接口**（本文 §9）：检测头、分割对齐、外参/时间校正。

## 2. 输入输出

### 2.1 输入

| 传感器 | 频率 | 用途 | 归属 |
|---|---|---|---|
| 4 路激光雷达 | 10–20Hz | 点云建图、地面分割、LO | 2.1（LO）/ 2.2（分割） |
| IMU | ≥100Hz | 去畸变、位姿传播、RTK 质量校验 | 2.1 |
| 北云 RTK | 1–20Hz | 全局位置/航向约束 | 2.1 |
| 车辆时间同步 | — | 统一时间基准 | 2.1 前置条件 |

雷达侧需要先做外参标定，统一转换到 `base_link`，再合并成一帧送入 GLIM。RTK 原始观测（伪距/多普勒/载波）模式的数据链路与解析器不在阶段二交付范围，见 GNSS 总方案 S0/S1。

### 2.2 输出

| 输出项 | 内容 | 频率 | 归属 | 消费方 |
|---|---|---|---|---|
| 实时轨迹 | position, orientation, velocity, covariance | ≥10Hz | 2.1 | 2.2、阶段三 |
| 传感器状态 | RTK 状态、LO 置信度、退化标志 | ≥5Hz | 2.1 | 阶段三、诊断 |
| 语义 VoxelMap | occupancy + ground/non-ground + timestamp | ≥10Hz | 2.2 | 阶段三 |
| 100m Submap | 局部体素地图 | 持续更新 | 2.2 | 阶段三 |
| 地面点云 | ground / non-ground 分离结果 | 每帧 | 2.2 | 2.2、阶段三 |

## 3. 架构设计

整体分为四层，括号内为归属子 spec：

```
Sensor Layer:  RTK / IMU / 4 LiDAR
Preprocess:    时间对齐、外参变换、IMU 去畸变、点云合并        (2.1 前置)
GLIM Core:     GPU Odometry + Submap + Pose Graph              (不修改)
Phase2 Extension:
   - RTK/IMU/LO 融合因子                                        (2.1)
   - RTK 质量门控 / 退化检测与权重管理                          (2.1)
   - 地面分割模块                                               (2.2)
   - 语义 VoxelMap                                             (2.2)
   - 100m Submap 管理器                                        (2.2)
Output Layer:  轨迹、语义地图、状态诊断
```

GLIM 的里程计层保留不动，RTK Odometry 插件继续作为里程计模块接入。阶段二新增模块以插件/回调方式挂在 GLIM 外部或内部，不重写核心配准逻辑。

## 4. 子 spec 边界

| 编号 | 文档 | 负责 | 不包含 |
|---|---|---|---|
| 2.1 | [定位子spec](阶段二_2.1_Spec_RTK_IMU_LO融合定位.md) | 位姿/速度、RTK 门控、退化检测与降级、定位状态输出 | 语义地图、地面分割、Submap |
| 2.2 | [语义地图子spec](阶段二_2.2_Spec_语义VoxelMap_100mSubmap.md) | 地面分割、语义 VoxelMap、100m Submap 管理 | 位姿估计、视觉、目标检测跟踪 |
| — | [GNSS 紧耦合总方案](../../gnss/specs/2026-10-31-gnss-tight-coupling-master-plan.md) | 2.1 的原始观测升级路线、实验计划、风险登记 | 阶段二验收本身 |

阶段二范围外：视觉 BEV/检测/在线标定属阶段三；GNSS v4 车车动基线属阶段四远期。

## 5. 跨模块数据契约

### 5.1 2.1 → 2.2

2.1 每个有效历元输出一个融合结果，2.2 只用位姿与时间戳写体素：

```cpp
struct FusionResult {
    double stamp;
    Eigen::Isometry3d T_world_base;
    Eigen::Matrix<double, 6, 6> covariance;
    Eigen::Vector3d velocity;
    FusionStatus status;   // RTK/LO/IMU 置信度、退化标志
};
```

约束：2.2 不重新估计位姿；位姿质量下降（退化标志置位）时，2.2 只降低体素更新频率/权重，不改变几何写入逻辑。

### 5.2 2.1 / 2.2 → 阶段三

- 2.1 提供 `T_world_base` + covariance + status（≥10Hz / ≥5Hz），供 BEV 投影与感知增强定位；
- 2.2 提供 `get_submap(center, radius)`、`semantic_label`、动态掩码回写接口，详见 [2.2 §9](阶段二_2.2_Spec_语义VoxelMap_100mSubmap.md#9-与阶段三的交接)。

### 5.3 阶段三 → 阶段二回灌（只留槽位）

- 动态掩码：阶段三的 `ObjectTrack` 列表调用 2.2 的 `remove_dynamic_objects`；
- 视觉地面/静态结构软约束：走 2.1 的 `insert_ground_plane` 与 2.2 的体素权重更新槽位；
- 阶段二只声明接口，不实现视觉逻辑。

## 6. 集成验收

| 指标 | 目标 |
|---|---|
| 轨迹输出频率 | ≥10Hz |
| 轨迹延迟 | <100ms |
| VoxelMap 更新频率 | ≥10Hz |
| 退化切换（RTK/LO/IMU） | 无位姿跳变，有状态告警 |
| Submap 显存占用 | Orin 上稳定，不持续增长 |
| 阶段二 baseline | 可复现，供阶段三 §9.2 对比 |

单模块指标（定位精度、地面分割时延等）见各子 spec 末节。

## 7. 测试计划

### 7.1 数据集测试

- 开阔矿区直道；
- 深坑/边坡多路径区域；
- 空旷无结构区域；
- 其他车辆/人员干扰场景；
- RTK 频繁退化场景。

### 7.2 对比实验

1. 纯 RTK/IMU（2.1 M-A 模式）；
2. 纯 GLIM LO；
3. RTK + IMU + LO + 地面约束（2.1 + 2.2 全量）。

评估指标：ATE、RPE、轨迹平滑度、RTK 退化段漂移、地面分割稳定性、Submap 切换连续性。

### 7.3 实车测试

- 连续运行 30 分钟以上；
- 记录 RTK 状态切换次数、退化触发次数；
- 检查显存和 CPU/GPU 占用；
- 检查轨迹是否有跳变。

## 8. 交付物

- 2.1：RTK/IMU/LO 融合模块、RTK 质量门控、退化检测与权重管理、配置参数；
- 2.2：地面分割模块、语义 VoxelMap、100m Submap 管理器、配置参数；
- 总纲：跨模块数据契约、日志与可视化工具、阶段二测试报告。

## 9. 与阶段三的接口交接

阶段三见 [阶段三 Spec](阶段三_Spec_视觉激光跨模态融合_动态感知_在线自标定.md)。下表是双方必须对齐的契约：

| 阶段三需求 | 阶段二提供 | 文档 |
|---|---|---|
| BEV 投影需要位姿 | `T_world_base` + covariance，≥10Hz | [2.1](阶段二_2.1_Spec_RTK_IMU_LO融合定位.md) |
| 跨模态语义对齐需要体素与标签 | `get_submap(center, radius)`、`semantic_label` | [2.2](阶段二_2.2_Spec_语义VoxelMap_100mSubmap.md) |
| 动态掩码写回地图 | `remove_dynamic_objects(std::vector<ObjectTrack>)` | [2.2](阶段二_2.2_Spec_语义VoxelMap_100mSubmap.md) |
| 多类别语义 | `semantic_label` 枚举已含 VEHICLE/PEDESTRIAN/VEGETATION/ORE_PILE/DYNAMIC_UNKNOWN | [2.2](阶段二_2.2_Spec_语义VoxelMap_100mSubmap.md) |
| 感知增强定位 | 动态 mask → `static_points`；视觉地面/静态软约束槽位 | 2.1 + 2.2 |
| 体素结构扩展 | 基础字段稳定，阶段三追加 `dynamic_prob` / `associated_track_id` / `visual_semantic_confidence` | [2.2](阶段二_2.2_Spec_语义VoxelMap_100mSubmap.md) |
| 外参/时间偏移 | 阶段二只提供统一到 `base_link` 的离线外参；在线微调留阶段三 | — |

阶段三不需要阶段二实现任何视觉/检测/标定逻辑；阶段二只需保证上述类型与接口存在且不破坏。阶段四（全局回环、多车协同建图、离线高精地图）由阶段三输出继续演进，不在本文范围。
