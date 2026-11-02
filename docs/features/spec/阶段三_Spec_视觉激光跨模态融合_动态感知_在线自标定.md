# 阶段三：视觉-激光跨模态融合 + 动态感知 + 在线自标定

## 1. 阶段目标

在阶段二稳定的语义 VoxelMap 和融合定位基础上，引入 4 相机视觉感知，完成：

- **跨模态语义对齐**：视觉 BEV 分割与激光语义 VoxelMap 对齐；
- **动态感知**：3D 目标检测 + 跟踪，输出动态物体掩码；
- **在线自标定**：外参微调 + 时间偏移估计；
- **感知增强定位**：动态物体 mask 掉，静态结构参与配准；
- **闭环输出**：实时轨迹 + 语义地图 + 目标列表 + 诊断信息。

## 2. 输入输出

### 2.1 输入（新增）

| 传感器 | 频率 | 用途 |
|---|---|---|
| 4 路相机 | 10–30Hz | BEV 分割、目标检测 |
| [阶段二输出](阶段二_Spec_RTK_IMU_LO融合定位_语义VoxelMap.md) | ≥10Hz | 轨迹（[2.1](阶段二_2.1_Spec_RTK_IMU_LO融合定位.md)）、语义 VoxelMap + submap（[2.2](阶段二_2.2_Spec_语义VoxelMap_100mSubmap.md)） |

相机需要预先标定内参和外参，阶段三支持在线微调。

### 2.2 输出（新增）

| 输出项 | 内容 | 频率 |
|---|---|---|
| 3D 目标列表 | id, bbox, velocity, class, confidence | ≥10Hz |
| 动态掩码 | 体素级 dynamic mask | 每帧 |
| 跨模态对齐损失 | segmentation alignment loss | 每帧 |
| 外参/时间偏移 | lidar-camera extrinsic delta, time offset | 低频更新 |
| 融合诊断 | 视觉置信度、对齐质量、跟踪状态 | ≥5Hz |

## 3. 架构设计

```
Phase2 Output: 轨迹 + 语义 VoxelMap + Submap
                    │
        ┌───────────┼───────────┐
        ▼           ▼           ▼
   Camera Raw    相机去畸变    外参投影
        │
        ▼
   BEV Encoder (DETR-style / BEVFormer-style)
        │
        ├──► BEV Segmentation Head ──► 语义对齐
        ├──► 3D Detection Head ──► 目标列表
        └──► Depth Estimation (可选)
                    │
        ┌───────────┼───────────┐
        ▼           ▼           ▼
   投影到激光坐标系   动态掩码生成   外参/时间校正
        │
        ▼
   更新语义 VoxelMap（多类别）
        │
        ▼
   动态物体 mask → 静态点参与 LO 配准
        │
        ▼
   融合定位（RTK/IMU/LO + 视觉约束）
```

## 4. 模块分解

### 4.1 BEV 感知模块

推荐采用 **DETR-style BEV 感知框架**，单 backbone 同时输出分割和检测：

| 子模块 | 功能 | 备注 |
|---|---|---|
| Image Backbone | 多相机特征提取 | ResNet / Swin |
| BEV Encoder | 图像特征 → BEV 特征 | Cross-attention 或 LSS |
| Segmentation Head | BEV 级语义分割 | 输出类别概率图 |
| Detection Head | 3D 检测框 | class + bbox + velocity |
| Depth Head（可选） | 深度估计辅助 | 提升 LSS 类方法精度 |

类别设计直接复用阶段二扩展的语义标签（定义见 [2.2 §4](阶段二_2.2_Spec_语义VoxelMap_100mSubmap.md#4-语义-voxelmap)）：

```
GROUND, NON_GROUND_STATIC, VEHICLE, PEDESTRIAN,
VEGETATION, ORE_PILE, DYNAMIC_UNKNOWN
```

### 4.2 跨模态语义对齐

核心思想：**把视觉 BEV 分割结果投影到激光坐标系，与语义 VoxelMap 做对齐**。

流程：

1. 视觉 BEV 分割输出每类的概率图；
2. 用当前外参 + [2.1 输出的轨迹](阶段二_2.1_Spec_RTK_IMU_LO融合定位.md)，将 BEV 像素投影到激光坐标系的体素网格；
3. 对每个体素，比较视觉预测的语义分布与激光侧已有的语义标签；
4. 计算对齐损失，用于：
   - 监督外参/时间偏移的在线校正；
   - 更新体素语义置信度；
   - 检测跨模态不一致区域（可能是动态物体或标定漂移）。

对齐损失设计：

```
L_align = Σ_voxel CE(semantic_visual, semantic_lidar)
```

只在对齐质量高的区域计算损失，低置信度区域不参与。

### 4.3 动态感知与跟踪

**检测**：BEV Detection Head 输出 3D 检测框。

**跟踪**：用简单但鲁棒的跟踪器：

- AB3DMOT / ByteTrack 3D 变体；
- 状态变量：position, velocity, size, class；
- 用 Kalman Filter 预测 + Hungarian 匹配。

**动态掩码生成**：

1. 检测框内的点云标记为动态候选；
2. 跟踪器确认连续多帧存在的物体，确认为动态；
3. 高变化但无检测框的体素，标记为 `DYNAMIC_UNKNOWN`；
4. 动态掩码通过 [2.2 的 `remove_dynamic_objects` / `apply_dynamic_mask`](阶段二_2.2_Spec_语义VoxelMap_100mSubmap.md#6-接口) 写入语义 VoxelMap，供 LO 配准时 mask 掉。

### 4.4 在线自标定

#### 外参微调

状态变量：每个相机相对于 `base_link` 的 SE3 小量偏移 `δT`。

观测来源：

- 跨模态语义对齐残差；
- 视觉特征点重投影误差（可选）；
- 检测框与激光点云的 IoU（可选）。

优化方式：在因子图中加入 **外参微调因子**，做小范围 BA。更新频率不宜过高，建议 1–5Hz，避免与主定位耦合过紧。

#### 时间偏移估计

状态变量：每个相机与激光/IMU 的时间偏移 `δt`。

观测来源：

- 快速运动段中，视觉 BEV 特征与激光结构的对齐残差随 `δt` 的变化；
- 用数值微分或解析雅可比估计最优 `δt`。

同样低频更新，只在运动激励足够时激活。

### 4.5 感知增强定位

动态物体 mask 后，LO 配准只用静态点：

```
static_points = raw_points - dynamic_mask
```

这能显著减少其他车辆、人员、装载机对配准的污染。

同时，视觉 BEV 分割结果可以作为 **软约束** 进入融合器：

- 视觉认为某区域是地面 → 增强地面约束；
- 视觉认为某区域是静态结构 → 提高该区域体素在配准中的权重；
- 视觉置信度低时，不引入额外约束，避免污染。

## 5. 接口定义

### 5.1 BEV 感知接口

```cpp
class BEVPerception {
public:
    void insert_images(double stamp,
                       const std::vector<cv::Mat>& images,
                       const std::vector<CameraIntrinsic>& intrinsics);

    BEVResult get_result() const;
};

struct BEVResult {
    double stamp;
    // BEV 网格上的语义概率 [H_bev, W_bev, N_classes]
    Eigen::Tensor<float, 3> bev_semantic;
    // 3D 检测框
    std::vector<Object3D> objects;
    // 置信度
    float alignment_confidence;
};
```

### 5.2 动态跟踪接口

```cpp
class ObjectTracker {
public:
    void update(const std::vector<Object3D>& detections,
                double stamp);

    std::vector<ObjectTrack> get_tracks() const;

    DynamicMask generate_mask(const PreprocessedFrame& frame) const;
};
```

### 5.3 在线标定接口

```cpp
class OnlineCalibration {
public:
    void insert_alignment_residual(double stamp,
                                   const Eigen::VectorXd& residual,
                                   const Eigen::MatrixXd& information);

    void optimize();

    Eigen::Isometry3d get_extrinsic_delta(int camera_id) const;
    double get_time_offset(int sensor_id) const;
};
```

## 6. 关键参数

| 参数 | 默认值 | 说明 |
|---|---|---|
| bev_grid_size | 100m × 100m | BEV 感知范围 |
| bev_resolution | 0.2m | BEV 网格分辨率 |
| detection_conf_threshold | 0.3 | 检测置信度阈值 |
| track_confirm_frames | 3 | 确认跟踪所需连续帧数 |
| track_lost_frames | 5 | 跟踪丢失容忍帧数 |
| extrinsic_update_rate | 1Hz | 外参微调频率 |
| time_offset_update_rate | 0.5Hz | 时间偏移更新频率 |
| alignment_min_confidence | 0.5 | 对齐质量最低阈值 |
| dynamic_mask_dilation | 0.3m | 动态掩码膨胀半径 |

## 7. 验收指标

| 指标 | 目标 |
|---|---|
| 3D 检测 mAP | 矿区场景下可用（具体阈值待数据集确定） |
| 跟踪 ID 切换率 | 低频，无明显 ID 抖动 |
| 动态物体 mask 准确率 | 视觉 + 激光一致性高 |
| 跨模态对齐稳定性 | 无周期性漂移 |
| 外参在线校正精度 | 相对离线标定 <5cm / <0.5° |
| 时间偏移估计精度 | <10ms |
| LO 配准内点率 | 动态 mask 后提升 |
| 端到端延迟 | <150ms |
| Orin 资源占用 | GPU/CPU 稳定，不 OOM |

## 8. 交付物

- BEV 感知模块（分割 + 检测）；
- 3D 目标跟踪模块；
- 跨模态语义对齐模块；
- 动态掩码生成模块；
- 在线外参/时间标定模块；
- 感知增强 LO 配准逻辑；
- 配置参数与模型权重；
- 阶段三测试报告。

## 9. 测试计划

### 9.1 数据集测试

- 多车辆交汇场景；
- 人员穿行场景；
- 装载机作业区域；
- 光照变化（逆光、黄昏、夜间如有数据）；
- 标定轻微漂移场景（用于验证在线校正）。

### 9.2 对比实验

1. 阶段二 baseline vs 阶段三（有无动态 mask）；
2. 离线标定 vs 在线标定；
3. 有无视觉语义约束的定位对比；
4. 跟踪稳定性评估（ID switch, fragment）。

### 9.3 实车测试

- 连续运行 1 小时以上；
- 记录检测/跟踪质量；
- 记录在线标定收敛情况；
- 检查动态 mask 对轨迹平滑度的改善；
- 监控 Orin 资源占用。

## 10. 与阶段四的接口预留

阶段三完成后，阶段四可以考虑：

- **全局回环检测**：语义描述子 + 几何描述子融合；
- **多车协同建图**：语义 VoxelMap 合并与冲突消解；
- **离线高精地图生成**：从语义 VoxelMap 导出结构化地图；
- **预测模块**：基于跟踪轨迹做短期运动预测；
- **规划接口**：输出可通行区域、动态障碍物轨迹。

语义 VoxelMap 的体素结构可以在 [2.2 的基础结构](阶段二_2.2_Spec_语义VoxelMap_100mSubmap.md#4-语义-voxelmap) 上进一步扩展（基础字段布局保持不变）：

```cpp
struct SemanticVoxel {
    float occupancy_prob;
    uint8_t semantic_label;
    Eigen::Vector3f plane_normal;
    float plane_d;
    uint32_t update_count;
    double last_update_time;
    float height_mean;
    float height_var;
    // 阶段三新增
    float dynamic_prob;
    uint32_t associated_track_id;
    float visual_semantic_confidence;
    // 阶段四预留
    float reflectance_mean;
    float traversability;
};
```
