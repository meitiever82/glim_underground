# 激光 SLAM 在线外参标定：文献调研

> 调研时间：2026-10-10。覆盖 LiDAR-IMU / 多 LiDAR / LiDAR-GNSS(杆臂) 三类传感器对的在线外参（含时间偏移）标定工作。
> 引用编号见文末 Sources，全部由检索时注册的账本生成。

## 1. 问题定义与动机

在线外参标定（online extrinsic calibration）指将传感器间的空间变换（SE(3) 外参）和时间偏移（temporal offset）作为待估状态，在 SLAM/里程计运行过程中与位姿一起估计，而不是离线一次性标定后视为常量。动机有三：

1. **标定会漂移**：机械振动、温度、拆装都会使外参变化；LIC-Fusion 明确指出在线标定是为了"compensate for possible calibration variations"[12]。
2. **离线标定成本高**：MINS 将在线标定定位为补偿"poor prior calibration and/or degraded calibration varying over time"[14]。
3. **免靶标部署**：多 LiDAR 系统往往没有公共视场，离线靶标法失效，在线方法成为唯一选择[4][2]。

2025 年 MDPI Sensors 上的综述《Camera, LiDAR, and IMU Spatiotemporal Calibration: Methodological Review and Research Perspectives》指出：在线标定的本质是把时空参数的估计与实时状态估计任务（SLAM/里程计）耦合，用数据驱动的参数更新持续补偿传感器间的时空失配[18]。

## 2. 技术路线

### 2.1 状态增广 + 滤波（MSCKF/EKF 系）

把外参和时间偏移直接放入滤波状态向量，随状态一起传播和更新：

- **LIC-Fusion**（IROS 2019）：MSCKF 框架下紧耦合 IMU + 视觉特征 + LiDAR 边缘/平面特征，状态中含 IMU-Camera 与 LiDAR-IMU 的时空标定量，实现在线空间+时间标定[12]。
- **LIC-Fusion 2.0**（2020）：引入滑窗平面特征跟踪，并对 LiDAR-IMU 子系统做了完整的可观测性分析，报告了使用平面特征做时空标定时的退化运动情形[13]。实验显示：从扰动初值出发若不做在线标定，系统输出不一致（ATE 和 NEES 明显变大）[13]。
- **MINS**（RPNG, 2023）：目前最完整的滤波系多传感器方案——IMU/轮速/相机/GNSS/LiDAR 五模态紧耦合，对**所有** onboard 传感器做在线内参+外参（时空）标定，并处理了异步融合的流形插值与计算复杂度问题[14]。
- **LION**（DARPA SubT 亚军/冠军系统）：固定滞后滑窗平滑器融合 IMU 与 LiDAR 相对位姿，**不要求已知 LiDAR-IMU 相对安装关系**，外参完全在线估计，并用可观测性指标自评位姿是否几何病态[8]。这是地下矿井场景（与你的 underground 场景直接相关）验证过的方案。
- **Target-free 3D-Lidar-IMU EKF 标定**（2021）：先用运动激励下旋转约束的最小二乘求旋转，再用 EKF 估计平移，代码开源（unmannedlab/imu_lidar_calibration）[5]。

### 2.2 因子图 / 滑窗优化系

- **M-LOAM**（T-RO 2021）：多 LiDAR 在线标定+里程计+建图完整系统。运动序列对齐（hand-eye 式）初始化外参 → 滑窗内在线精化 → 收敛判据自动终止标定转纯里程计，全程无监督[4]。
- **连续时间固定滞后平滑**（2023）：B 样条连续时间轨迹天然支持异步测量和**在线时间偏移估计**（IMU 时间偏移多在 3 秒内收敛，均值 -2.0ms、std 2.7ms）[17]。
- **GNSS 原始观测紧耦合 FGO 标定**（2025-10）：在因子图内融合 IMU、里程计与 GNSS 伪距/载波/多普勒，联合估计导航状态与 IMU-里程计外参；可观测性分析证明一般运动下水平两方向平移和三方向旋转可观，**垂向平移不可观**[10]。
- **GNSS/INS 杆臂在线估计（ION, FGO）**：因子图 GNSS/INS 组合中在线估计杆臂，车和火车实测表明比 KF 方法精度更高、收敛更快、重复性更好[11]。

### 2.3 初始化式标定（粗标定 → 交给在线系统）

- **LI-Init**（IROS 2022, HKU MaRS）：与 FAST-LIO2 无缝衔接的初始化模块。先用角速度幅值互相关粗估时间偏移（免硬件同步），再统一优化精化时间偏移+外参+重力+IMU 零偏，并自动评估数据激励度[3]。工程启示：同一刚性安装外参只需标一次写入配置，时间偏移若每次上电变化则需每次标定[3]。
- **iCalib**（UDel, ICRA21 VINS workshop）：惯性辅助的统一 NLS 标定框架，一次标定相机内参、camera-IMU/LiDAR-IMU/wheel-IMU 时空参数[19]。

### 2.4 可观测性分析与退化处理（成败关键）

这一支线是在线标定能不能用的核心：

- **Yang et al.（2022）**：对 VINS 全参数标定做完整可观测性分析，证明充分激励的 6 轴运动下所有标定参数可观（仅全局 yaw 和平移不可观），并**首次系统枚举了使标定参数退化的基本运动模式**，且退化运动的任意组合仍退化[15]。方法论对 LiDAR-IMU 同样适用。
- **OA-LICalib**（TRO 2022，浙大 APRIL）：连续时间批优化标定 LiDAR/IMU 内外参，两个可观测性模块：(i) 信息论数据选择——只用信息量最大的数据段；(ii) **TSVD 截断奇异值分解的状态更新**——只更新状态空间中可辨识的方向，退化数据下也能得到准确结果[7]。代码开源（APRIL-ZJU/OA-LICalib）。
- **可观测性感知的多 LiDAR 在线标定**（RA-L 2023, Scania 卡车实测）：以 GNSS 位姿为基准实时匹配 LiDAR 里程计位姿，用互信息选最具信息量的位姿子集，并给出标定完成停止判据[2]。同一团队后续工作进一步只用各 LiDAR 内置 IMU 原始信号匹配做初值，GICP+特征精化，并在线补偿 IMU 零偏[1]。
- **主动标定**（2025, IEEE Sensors J.）：更进一步——不只被动选数据，而是用 FIM 最小特征值作为目标，**在线规划 B 样条轨迹主动产生激励**，EKF 实时更新外参[9]。对"地下环境直线行驶激励不足"这类痛点是直接答案。
- **GRIL-Calib**（2024）：针对地面机器人，用地面平面约束（GPM）解决单目标优化中不可观方向的问题，把标定建模为单次联合优化而非分步优化[6]。

### 2.5 时间偏移标定

- **Qin & Shen**（2018）：VINS 中把 $t_d$ 建模进优化状态，与位姿、特征联合优化[16]。
- 连续时间方法用轨迹求导天然支持时间偏移在线估计[17]；LI-Init 用互相关+统一时空优化[3]（见 2.3）。

## 3. 按传感器对索引

| 传感器对 | 代表工作 | 备注 |
|---|---|---|
| LiDAR-IMU（空间） | LIC-Fusion 2.0 [13]、OA-LICalib [7]、MINS [14]、LION [8]、GRIL-Calib [6]、[5] | 最成熟，滤波/优化两派都有开源 |
| LiDAR-IMU（时间） | LI-Init [3]、连续时间平滑 [17] | 固态雷达上尤为重要 |
| 多 LiDAR | M-LOAM [4]、[1]、[2] | 无公共 FoV 场景的刚需 |
| GNSS-IMU 杆臂 | FGO 杆臂估计 [11]、GNSS 原始观测紧耦合 [10] | 杆臂可与导航状态同图估计；垂向分量弱可观[10] |
| LiDAR-GNSS（经位姿基准） | [2] | GNSS 提供全局位姿基准做外参回归 |

## 4. 对 GLIM 项目的切入点

结合 glim_ws（VGICP 前端 + 子地图滑窗 + GTSAM 因子图全局建图 + glim_ext RTK 扩展）：

1. **LiDAR-IMU 外参节点化**：GLIM 的预积分因子和 VGICP 因子都已对外参求导链路，把 $T_{IL}$ 加成因子图节点+先验因子是结构自然的做法；可参考 M-LOAM 的"初始化→在线精化→收敛后冻结"流程[4]。
2. **可观测性守卫必须做**：参考 OA-LICalib 的 TSVD 更新[7]或 [2] 的互信息数据选择，激励不足时冻结外参——退化运动下继续更新标定参数会导致估计不一致[13]。
3. **GNSS 杆臂**：若 RTK 以绝对位置因子进图，杆臂可作为图内节点在线估计，FGO 路线已有实测验证[10][11]；Scania 的多 LiDAR-GNSS 标定范式[2]说明"GNSS 位姿作基准 + 里程计位姿匹配"在车上可行。
4. **初始化先行**：LI-Init 式初始化模块[3]可以独立于主系统先做，成本低、收益直接（外参粗值+时间偏移+激励度评估三合一）。

## 5. 开源资源

| 项目 | 内容 |
|---|---|
| github.com/hku-mars/LiDAR_IMU_Init | LI-Init，初始化+时间偏移，ROS1（有社区 ROS2 fork）[3] |
| github.com/APRIL-ZJU/OA-LICalib | 可观测性感知 LiDAR-IMU 内外参标定[7] |
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
