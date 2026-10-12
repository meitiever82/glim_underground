# 激光 SLAM 在线外参标定：文献调研与 GLIM 落地方案

> 调研时间：2026-10-10（v2 细化版，结合 glim_ws 代码实况）。
> 覆盖 LiDAR-IMU / 多 LiDAR / LiDAR-GNSS(杆臂) 三类传感器对的在线外参（含时间偏移）标定。
> 引用编号见文末 Sources，由引用账本机械生成；代码引用均给出文件:行号。

## 1. 问题定义与动机

在线外参标定（online extrinsic calibration）指将传感器间的空间变换 $T \in SE(3)$（6 自由度外参）和时间偏移 $t_d$（temporal offset）作为**待估状态**，在 SLAM/里程计运行中与位姿、速度、IMU 零偏一起估计，而非离线标定后冻结为常量。

动机：

1. **标定会漂移**：机械振动、温度、拆装导致外参变化；LIC-Fusion 明确将在线标定定位为"compensate for possible calibration variations"[12]。
2. **离线标定成本高且不可持续**：MINS 将其定位为补偿"poor prior calibration and/or degraded calibration varying over time"[14]。
3. **免靶标部署**：多 LiDAR 系统常无公共视场，离线靶标法失效，在线方法成为唯一选择[4][2]。
4. **外参误差是硬误差**：旋转外参偏差 $\delta\theta$ 在距离 $r$ 处产生 $r\cdot\sin\delta\theta$ 的点云错位，直接污染配准对应关系；时间偏移在角速度 $\omega$ 下等效为角度误差 $\omega \cdot t_d$。2025 年 MDPI Sensors 综述指出：在线标定的本质是把时空参数估计与实时状态估计耦合，用数据驱动的参数更新持续补偿时空失配[18]。

## 2. 文献调研

### 2.1 状态增广 + 滤波（MSCKF/EKF 系）

思路：把外参（旋转+平移）和时间偏移放入滤波状态向量，测量残差对外参求雅可比，随 EKF/MSCKF 一起传播更新。

| 工作 | 框架 | 状态中的标定量 | 特点 |
|---|---|---|---|
| LIC-Fusion (IROS'19) [12] | MSCKF | IMU-Cam、LiDAR-IMU 时空外参 | 首个三传感器全在线标定的紧耦合 LIC 里程计 |
| LIC-Fusion 2.0 (2020) [13] | 滑窗滤波 | LiDAR-IMU 时空外参 | 滑窗平面特征跟踪 + LiDAR-IMU 子系统可观测性分析 + 退化运动枚举 |
| MINS (RPNG'23) [14] | MSCKF | **所有**传感器的内参+时空外参 | 五模态（IMU/轮速/相机/GNSS/LiDAR），流形插值处理异步，动态 clone 控算力 |
| LION (DARPA SubT) [8] | 固定滞后平滑 | LiDAR-IMU 外参 | 不要求已知安装关系，可观测性指标自评几何病态；地下矿井实测 |
| Target-free EKF [5] | EKF | LiDAR-IMU 6DoF | 旋转用运动约束最小二乘、平移用 EKF，两段式；已开源 |

要点：LIC-Fusion 2.0 的对照实验表明，从扰动初值出发若**不**做在线标定，系统输出不一致（ATE、NEES 显著变大）[13]——这定量说明了外参在线估计对估计一致性的作用。

### 2.2 因子图 / 滑窗优化系

思路：外参作为图中的一个变量节点，配准因子/预积分因子同时连接位姿节点与外参节点，再加先验因子约束。

| 工作 | 框架 | 要点 |
|---|---|---|
| M-LOAM (T-RO'21) [4] | 滑窗非线性优化 | 多 LiDAR：运动序列对齐（hand-eye）初始化 → 滑窗内在线精化 → **收敛判据自动冻结**外参转纯里程计，全程无监督 |
| 连续时间固定滞后平滑 (2023) [17] | B 样条 CT-FGO | 连续时间轨迹天然支持异步测量与在线时间偏移估计；IMU 时间偏移多在 3s 内收敛（均值 -2.0ms，std 2.7ms） |
| GNSS 原始观测紧耦合 FGO (2025) [10] | FGO | IMU+里程计+GNSS 伪距/载波/多普勒同图；可观测性分析：一般运动下水平两方向平移和三方向旋转可观，**垂向平移不可观** |
| GNSS/INS 杆臂 FGO (ION) [11] | FGO | 杆臂作为图内变量在线估计；车/火车实测：比 KF 精度高、收敛快、重复性好 |

### 2.3 初始化式标定（粗标定 → 交给在线系统）

- **LI-Init**（IROS'22, HKU MaRS）[3]：FAST-LIO2 的初始化模块。流程：① 角速度幅值互相关粗估 $t_d$（免硬件同步）→ ② 统一时空优化精化 $t_d$ + 外参 + 重力 + IMU 零偏 → ③ 激励度评估（雅可比奇异值）决定结果是否可信。工程经验：刚性安装的外参标一次写入配置即可，$t_d$ 若每次上电漂移则需每次初始化[3]。
- **iCalib**（UDel, ICRA'21 workshop）[19]：惯性辅助的统一 NLS 框架，一次标定相机内参 + camera-IMU / LiDAR-IMU / wheel-IMU 时空参数。

### 2.4 可观测性分析与退化处理（成败关键）

在线标定能不能用，取决于这一层：

1. **可观测性理论**：Yang et al. 对 VINS 全参数标定做了完整可观测性分析——充分激励的 6 轴运动下所有标定参数可观（仅全局 yaw + 平移不可观），并首次系统枚举了使标定参数退化的基本运动模式，且退化运动的任意组合仍退化[15]。LIC-Fusion 2.0 对 LiDAR-IMU 平面特征标定给出了对应的退化情形列表[13]。
2. **工程守卫（三件套）**：
   - **激励度评估**：LI-Init 用雅可比奇异值判断数据激励是否充分[3]；
   - **信息论数据选择**：OA-LICalib 只保留信息量最大的数据段参与标定[7]；Scania 的多 LiDAR 标定用互信息选位姿子集[2]；
   - **截断更新**：OA-LICalib 用 TSVD 只更新状态空间中可辨识的方向，退化数据下也不发散[7]。
3. **收敛判据与自动冻结**：M-LOAM 的收敛判据满足后自动终止标定转纯里程计[4]；Scania 方案给出标定完成的停止准则[2]。
4. **主动标定**（2025, IEEE Sensors J.）[9]：不只被动选数据——以 FIM 最小特征值为目标在线规划 B 样条轨迹，**主动产生激励**，EKF 实时更新外参。这是"地下巷道直线行驶激励不足"痛点的直接答案。
5. **地面约束补偿**：GRIL-Calib 用地面平面运动约束（GPM）使理论不可观方向变得可约束，并把分步标定改为单次联合优化[6]。

### 2.5 时间偏移标定

- VINS 系把 $t_d$ 建模进优化状态，与位姿、特征联合优化[16]；
- 连续时间轨迹用轨迹导数天然支持 $t_d$ 在线估计[17]；
- LI-Init 用互相关粗估 + 统一时空优化精化[3]（见 2.3）。

### 2.6 按传感器对索引

| 传感器对 | 代表工作 | 备注 |
|---|---|---|
| LiDAR-IMU（空间） | LIC-Fusion 2.0 [13]、OA-LICalib [7]、MINS [14]、LION [8]、GRIL-Calib [6]、EKF [5] | 最成熟，滤波/优化两派均有开源 |
| LiDAR-IMU（时间） | LI-Init [3]、CT 平滑 [17] | 固态/机械雷达均需 |
| 多 LiDAR | M-LOAM [4]、IMU 信号匹配 [1]、GNSS 基准 [2] | 无公共 FoV 刚需 |
| GNSS-IMU 杆臂 | FGO 杆臂 [11]、GNSS 原始观测紧耦合 [10] | 杆臂可与导航状态同图估计；垂向弱可观 [10] |
| LiDAR-GNSS（位姿基准） | Scania 方案 [2] | GNSS 位姿作基准 + 里程计位姿匹配 |

## 3. GLIM 现状分析（代码实况）

### 3.1 外参在 GLIM 中的注入链路

当前 GLIM（glim_underground）中 `T_lidar_imu` 是**纯常量**，生命周期如下：

1. **读取**：`odometry_estimation_imu.cpp:39` 从 `config_sensors.json` 读入（TUM 格式 `[x,y,z,qx,qy,qz,qw]`，语义为 `p_lidar = T_lidar_imu * p_imu`，见 `config/config_sensors.json:33-34`）；
2. **点云预处理即固化**：`odometry_estimation_imu.cpp:198-201` 在送入优化前把所有点变换到 IMU 系：
   ```cpp
   points_imu[i] = T_imu_lidar * raw_frame->points[i];
   ```
   因此后续 VGICP 因子（`odometry_estimation_gpu.cpp:144` 的 `IntegratedVGICPFactorGPU(X(target), X(current), ...)`）操作的是 **IMU 系点云**，外参在因子图里**完全不可见**；
3. **去畸变依赖外参**：`sub_mapping.cpp:363-370` deskew 用 `T_lidar_imu.inverse()` 把 IMU 系轨迹换回 LiDAR 系；
4. **状态向量**：`X(i) = T_world_imu`、`V(i) = v_world_imu`、`B(i) = imu_bias`（`odometry_estimation_imu.cpp:32-34`），没有外参节点；
5. **每帧冗余存储**：`EstimationFrame` 持有 `T_lidar_imu` 成员（`estimation_frame.hpp:88`），submap 序列化时每个子图保存一份（`sub_map.cpp:33`）——数据结构上是 per-frame 的，但全程赋的都是同一个常量，**这个设计意外地给在线标定留了口子**；
6. **初始化不估外参**：`RobustInitialStateEstimation`（`robust_initial_state_estimation.hpp:24-30`）只做 GICP frame-to-model 里程计 + 陀螺零偏估计 + 重力/速度/加计零偏的线性求解，`T_lidar_imu` 是构造时的 `const` 输入（第 51 行），不估计。

### 3.2 GNSS/RTK 侧

`glim_ext` 的 RTK shell（`glim_ext/include/glim_ext/rtk/rtk_shell_common.hpp:54`）从配置读 `T_imu_gnss`（GNSS 天线在 IMU 系的杆臂），同样是固定常量；算法核心在 `gnss_core::AnchorPipeline`，以 anchor 位置约束进图。

### 3.3 结论

GLIM 的外参是"配置 → 预处理固化"模式，因子图中无外参变量。在线标定的本质是**把外参从预处理常量变回图内变量**，并处理好与固定滞后平滑器、deskew、submap 的联动。

## 4. GLIM 在线标定实施方案（分四阶段）

### Phase 0：离线粗标定工具（1–2 周，独立交付物）

目标：不依赖 GLIM 主系统的标定工具，产出 `T_lidar_imu` 初值 + $t_d$ + 激励度报告。

做法（复刻 LI-Init 流水线[3]）：

1. GLIM 已有可复用件：`RobustInitialStateEstimation` 的 GICP frame-to-model 里程计（`robust_initial_state_estimation.hpp:26`）正好充当 LI-Init 中 FAST-LO 的角色；
2. 在里程计角速度序列与 IMU 角速度序列间做互相关 → $t_d$ 粗值；
3. 对齐 LiDAR 里程计状态与 IMU 预积分状态，统一优化外参 + 重力 + 零偏；
4. 输出雅可比奇异值谱作为激励度报告（LI-Init 同款[3]）。

价值：即使在线标定不上，这个工具也立刻解决"新车/新支架外参靠手量"的现状。

### Phase 1：外参节点化（核心改动，2–4 周）

**状态增广**：在里程计固定滞后平滑器中引入持久节点 `E(0)`（`gtsam::Pose3`，语义 $T_{li}$，与 GLIM 现有 `T_lidar_imu` 语义一致）：

- **先验因子**：`PriorFactor<Pose3>(E(0), T_lidar_imu_config, σ)`，σ 取手量/Phase 0 精度（如旋转 0.5°、平移 5mm）——这是收敛与一致性的锚；
- **配准因子改造**：不再在预处理把点转到 IMU 系（`odometry_estimation_imu.cpp:198-201` 改为保留 LiDAR 系点云），新增 `IntegratedVGICPFactorWithExtrinsic(X(i), X(j), E(0))`：残差内部用 $T_{wi} \cdot T_{il}$ 把 LiDAR 系点变换到目标帧，并对外参求雅可比。gtsam_points 的 `IntegratedVGICPFactor` 已支持自定义点变换，改动集中在雅可比链；
- **IMU 因子不变**：`ImuFactor` 只连 X/V/B，与外参无耦合项；
- **deskew 联动**：`cloud_deskewing` 与 `sub_mapping.cpp:363` 的外参改为从当前最优 `E(0)` 取，每轮优化后刷新——`EstimationFrame::T_lidar_imu` 成员正好用来缓存每帧当时的外参估计值，下游（submap 序列化、global_mapping 的 `T_world_lidar = T_world_imu * T_lidar_imu.inverse()`，`global_mapping.cpp:623-624`）**无需改动**。

**固定滞后平滑器的坑**：GLIM 用 `IncrementalFixedLagSmootherExtWithFallback`（`odometry_estimation_imu.hpp:117-118`），超出滞后的节点会被边缘化。外参是全局变量，必须保证 `E(0)` **永不被边缘化**——做法是每个关键帧都向外参节点挂因子（配准因子天然满足），且在 `new_stamps` 中不给 `E(0)` 赋时间戳或始终赋最新时间戳，使其留在窗内。这是本阶段最需要原型验证的点。

**收敛后冻结**：参考 M-LOAM 收敛判据[4]——外参边际协方差对角线低于阈值且连续 N 帧变化量小于阈值后，把 `E(0)` 换成极强先验（等效冻结），避免长期运行中的一致性漂移。

### Phase 2：可观测性守卫（与 Phase 1 并行）

没有守卫的在线标定在退化段必然发散[13][15]。三层防护：

1. **激励度监控**：维护滑窗内角速度/加速度的激励度指标——LI-Init 式雅可比奇异值[3]，或直接监控外参 Fisher 信息增量的最小特征值（OA-LICalib 的信息论准则[7]）；
2. **截断更新**：对外参增量做 SVD，只保留信息矩阵特征值超过阈值方向上的更新分量（OA-LICalib 的 TSVD 机制[7]）；激励不足时整体冻结，回退先验值；
3. **健康度输出**：外参估计值、协方差、激励度发到 ROS topic / viewer，现场可观测（offline_viewer 已有类似基础设施可复用）。

### Phase 3：GNSS 杆臂在线估计（RTK 侧，独立可做）

`glim_ext` 的 `T_imu_gnss` 杆臂目前手量。方案：

1. **松耦合版（先做）**：Scania 范式[2]——GNSS 位置序列与里程计位姿序列做带杆臂参数的轨迹对齐，批优化求杆臂；可完全离线/准在线，不动主图；
2. **紧耦合版**：杆臂作为全局建图因子图中的节点，RTK 位置因子同时连位姿节点与杆臂节点。FGO 路线已有车/火车实测验证优于 KF[11]；
3. **已知限制**：一般运动下 GNSS 相关外参的垂向平移分量弱可观[10]——杆臂 z 分量要么靠俯仰/起伏激励，要么固定手量值、只在线估水平分量。

### 4.5 验证计划

| 验证项 | 方法 | 判据 |
|---|---|---|
| 收敛性 | 离线 bag 回放，外参初值加 1°/2°/5° 扰动 | 收敛到真值 ±0.1° 内，且不依赖初值 |
| 一致性 | 多 bag Monte-Carlo，NEES/协方差检查（LIC-Fusion 2.0 同款[13]） | NEES 在置信区间内 |
| 退化鲁棒性 | 直线匀速段注入错误外参 | 守卫冻结外参，位姿不退化 |
| 端对端精度 | 扰动外参+在线标定 vs 真值外参，比较建图 ATE | 差距 < 5% |
| 算力开销 | 外参节点前后 odometry 单帧耗时对比 | 增幅 < 20% |

### 4.6 风险清单

1. **固定滞后边缘化与外参持久性的冲突**（Phase 1 最大技术风险，需原型验证）；
2. **旋转外参与 IMU 零偏耦合**：大激励初期二者可能互相吸收，先验 σ 不能太松；
3. **CT 模式（`odometry_estimation_ct`）**：连续时间轨迹下外参进图方式不同，若项目用 CT 模式需单独评估[17]；
4. **外参在线变化与子地图一致性的矛盾**：submap 用当时外参构建，外参后续更新不会重建 submap——长期运行需评估是否接受，或在 global mapping 层补偿。

## 5. 开源资源

| 项目 | 内容 |
|---|---|
| github.com/hku-mars/LiDAR_IMU_Init | LI-Init：初始化+时间偏移+激励度评估，ROS1（有社区 ROS2 fork）[3] |
| github.com/APRIL-ZJU/OA-LICalib | 可观测性感知 LiDAR-IMU 内外参标定（TSVD）[7] |
| github.com/rpng/MINS | 五模态在线标定导航系统[14] |
| github.com/unmannedlab/imu_lidar_calibration | EKF 式 LiDAR-IMU 标定[5] |
| ram-lab.com/file/site/m-loam | 多 LiDAR 标定+SLAM 系统与数据[4] |
| github.com/AISLAB-sustech/Multisensor-Calibration | 主动标定（轨迹优化）[9] |

<!-- SOURCES -->

## Sources

[1] https://arxiv.org/abs/2302.14735
[2] https://arxiv.org/abs/2212.09579
[3] https://arxiv.org/abs/2202.11006
[4] https://arxiv.org/abs/2010.14294
[5] https://arxiv.org/abs/2104.12280
[6] https://arxiv.org/abs/2312.14035
[7] https://arxiv.org/abs/2205.03276
[8] https://arxiv.org/abs/2102.03443
[9] https://arxiv.org/abs/2506.13420
[10] https://arxiv.org/abs/2510.08880
[11] https://www.ion.org/publications/abstract.cfm?articleID=20256
[12] https://arxiv.org/abs/1909.04102
[13] https://arxiv.org/abs/2008.07196
[14] https://arxiv.org/abs/2309.15390
[15] https://arxiv.org/abs/2201.09170
[16] https://arxiv.org/abs/1808.00692
[17] https://arxiv.org/abs/2302.07456
[18] https://www.mdpi.com/1424-8220/25/17/5409
[19] https://udel.edu/~ghuang/icra21-vins-workshop/papers/01-Yang_iCalib.pdf
