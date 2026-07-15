# 基于点云·IMU 约束滑动窗口优化的先验地图上 6DoF 位姿估计

> **原文标题（日）**：点群・IMU 制約のウィンドウ最適化に基づく既知地図上での 6DoF 姿勢推定
> **英文标题**：Window-Optimization-based Range-IMU Localization on a 3D Prior Environmental Map
> **出处**：ROBOMECH 2023
> **作者**：○小出健司（小出健司，Kenji Koide，产综研 AIST）、大石修士（Shuji Oishi，AIST）、横塚将志（Masashi Yokozuka，AIST）、阪野贵彦（Atsuhiko Banno，AIST）
>
> *本文为日文原文的中文译文，供项目内部参考；参考文献保留原文。*

---

## 摘要

本文提出一种在三维先验环境地图上进行鲁棒传感器定位的方法，其核心是对紧耦合的 LiDAR 与 IMU 约束进行滑动窗口优化。所提方法在一个滑动窗口因子图上，直接融合三类约束：相邻帧之间的点云配准约束、相对于地图点云的配准约束，以及预积分的 IMU 约束。实验结果表明，所提方法对快速的传感器运动以及不完整的地图均具有极强的鲁棒性。

**关键词**：定位（Localization）、传感器融合（Sensor fusion）、点云（Point Cloud）、IMU

---

## 1. 绪言

自身定位（self-localization）是自主系统的重要环节，对于自动驾驶车辆、服务机器人等众多应用而言，稳定的自身定位不可或缺。近年来，随着廉价三维距离传感器（例如 LiDAR、RGBD 相机）的普及，利用三维点云在先验地图上进行自身定位的研究开发尤为活跃。

现有的自身定位方法中，许多是将 NDT、GICP 等扫描匹配（scan matching）应用于当前传感器点云与环境地图模型之间，并在位姿空间或直接在状态估计滤波器上最小化其误差 [1, 2, 3]。这类方法基本上每一帧只考虑当前传感器点云与环境点云的匹配，因而能够进行高速的状态估计。然而，当传感器点云所含几何特征不足、致使点云匹配误差函数发生**退化（縮退）**的情形，或地图存在缺损与变化时，估计精度会大幅劣化。此外，为实现更稳定的状态估计，也有人提出将粒子滤波与基于距离场的优化相结合的蒙特卡洛位姿估计方法 [4]，但其存在基于距离场的优化在地图缺损区域变得不稳定等问题。

本研究为实现高精度且鲁棒的先验地图上自身定位，**同时考虑时间上前后点云帧之间的匹配与对地图的匹配**，并进一步基于包含 IMU 约束的复合代价，对优化窗口内的多个传感器状态变量进行**联合优化**。由此，所提方法相对于既有方法具有以下优点：

1. 通过与前后帧的匹配可估计自身运动量（自我运动），因此即便在地图部分缺失乃至完全缺失的区域，也能进行稳定的状态估计。
2. 由于与前后帧的匹配和与地图的匹配采用**完全相同的误差函数**处理，因此可在地图内外之间实现平滑过渡。
3. 通过包含 IMU 约束的窗口优化，即使面对剧烈的传感器运动也能极其鲁棒地应对。

此外，所提方法在设计上不包含依赖特定传感器的处理，因此不仅适用于 Ouster OS1-64、Livox Avia 之类的 LiDAR，也适用于包括 Microsoft Azure Kinect 在内的 RGBD 相机——即可应用于任意的 Range-IMU（距离 + 惯性）传感器。

---

## 2. 提案方法

所提方法以基于固定滞后平滑（Fixed-lag smoothing）的 Range-IMU 融合运动量估计方法 [5] 为基础，并纳入对先验地图的点云匹配，进行因子图优化。

**图 1（Fig. 1）：所提因子图。** 所提方法在一个滑动窗口因子图上，直接融合相邻帧之间的点云配准约束、相对于地图点云的配准约束，以及 IMU 约束。
（图例：○ Sensor state 传感器状态；▢ Map point cloud 地图点云；红箭头 Matching cost factor 匹配代价因子；绿箭头 IMU preintegration factor IMU 预积分因子。状态节点 x₁…x₅ 之间由匹配代价因子与 IMU 预积分因子连接，并各自与地图点云 m 之间连有匹配代价因子。）

图 1 给出了所提方法优化所用的因子图。该因子图主要由两类因子构成：评价点云间一致性的**匹配代价因子（matching cost factor）**，以及基于 IMU 观测、评价运动量一致性的 **IMU 因子**。

匹配代价因子采用 **Voxelized GICP（VGICP）[6]**。它是将基于分布对分布匹配的 Generalized ICP [7] 用体素对应关系加以扩展的方法，在保持 GICP 精度的同时，能够在 GPU 上高速计算匹配误差。为估计自身运动量，对当前点云帧与过去 N 帧（例如 2 帧）之间生成匹配代价因子；同时，为校正运动量估计的漂移，也对当前帧与地图点云之间生成匹配代价因子。IMU 约束采用预积分（preintegration）方法 [8]，通过省去线性化时的重积分处理来高效计算误差。

优化的目标函数表示为：

```
F(X) = Σ_{xi∈X} ( Σ_{j=i-2}^{i-1} f_P(xi, xj) )  +  Σ_{xi∈X} f_P(xi, m)          (1)
       + Σ_{xi∈X} f_I(xi-1, xi)  +  C(X).                                       (2)
```

其中，`X = [xi, ···, xi+N]` 为优化窗口（例如 5 s）内存在的传感器状态变量，`m` 为地图点云，`f_P` 为点云匹配误差函数，`f_I` 为 IMU 误差函数，`C` 为对已被边缘化（周辺化）的变量与因子进行补偿的项。

由于所提方法以**完全相同的误差函数**处理对地图的匹配与对过去帧的匹配，因此即便在地图存在大面积缺损的情形下，也能基于对过去帧匹配得到的自身运动量估计来进行稳定的状态估计。此外，由于在因子图上直接整合点云与 IMU 的误差函数（即**紧耦合，tight coupling**），故对剧烈的传感器运动也极为鲁棒。

优化以及对离开优化窗口的变量的边缘化（marginalization），采用基于贝叶斯树的高效优化方法 **iSAM2 [9]**。实现使用了 **GTSAM**[^1]。

---

## 3. 实验

**图 2（Fig. 2）：实验环境。** 为评价对地图点缺失的鲁棒性，将虚线矩形所示区域以外的点云全部删除（即仅保留虚线框内区域作为环境地图）。

图 2 给出实验环境。为衡量对地图缺损的鲁棒性，仅保留虚线框所围区域、删除其余区域的点云，将其作为环境地图使用。作为对比方法，采用了：基于 NDT 扫描匹配与无迹卡尔曼滤波（Unscented Kalman Filter）的 **hdl_localization [10]**，以及基于粒子滤波与距离场优化的 **mcl3d_ros [4]**。图 3 给出各方法估计出的传感器轨迹。

可以看出，hdl_localization 与 mcl3d_ros 在进入地图缺损区域的时刻分别产生了较大的估计误差。这是因为两种方法的状态估计都高度依赖于对地图的匹配，当无法获得足够的匹配点时，估计随即破绽（崩溃）。

另一方面，所提方法能够不依赖于对地图的匹配来进行运动量估计，因此即便置身于地图缺损区域，也能实现稳定的传感器状态估计。而且，当从缺损区域返回原先地点时，对地图的匹配会再次生效，从而在抑制估计漂移的同时实现高精度的运动量估计。

**图 3（Fig. 3）：估计轨迹。** 既有方法在传感器移出地图区域时发生崩溃；所提方法成功地估计出了穿越点云被完全删除区域的传感器轨迹。（图中曲线：mcl3d_ros、hdl_localization、Proposed；横轴 X[m]、纵轴 Y[m]。）

---

## 4. 结言

本研究提出了一种通过对点云配准误差与 IMU 误差进行窗口联合最小化、从而在先验地图上实现高精度且鲁棒的自身定位方法。所提方法以同一误差函数处理对地图的匹配与前后帧的匹配，使得对地图缺损与变化具有鲁棒的状态估计成为可能。实验结果证实，所提方法即便在地图缺损区域也能实现极其稳定的自身定位。

---

## 致谢

本研究为国立研究开发法人新能源・产业技术综合开发机构（NEDO）资助项目的成果。

---

## 参考文献

> 以下保留原文。

[1] F. Pomerleau, F. Colas, R. Siegwart, and S. Magnenat, "Comparing icp variants on real-world data sets," *Autonomous Robots*, no. 34, pp. 133–148, 2013.

[2] C. Bai, T. Xiao, Y. Chen, H. Wang, F. Zhang, and X. Gao, "Faster-LIO: Lightweight tightly coupled lidar-inertial odometry using parallel sparse incremental voxels," *IEEE Robotics and Automation Letters*, vol. 7, no. 2, pp. 4861–4868, Apr. 2022.

[3] R. David and M. A. L., "Lol: Lidar-only odometry and localization in 3d point cloud maps," in *IEEE International Conference on Robotics and Automation*, 2020, pp. 4379–4385.

[4] N. Akai, "Efficient solution to 3d-lidar-based monte carlo localization with fusion of measurement model optimization via importance sampling," 2023.

[5] K. Koide, M. Yokozuka, S. Oishi, and A. Banno, "Globally consistent and tightly coupled 3d LiDAR inertial mapping," in *2022 International Conference on Robotics and Automation (ICRA)*. IEEE, may 2022.

[6] ——, "Globally consistent 3d LiDAR mapping with GPU-accelerated GICP matching cost factors," *IEEE Robotics and Automation Letters*, vol. 6, no. 4, pp. 8591–8598, oct 2021.

[7] A. Segal, D. Haehnel, and S. Thrun, "Generalized-ICP," in *Robotics: Science and Systems V*. Robotics: Science and Systems Foundation, jun 2009.

[8] C. Forster, L. Carlone, F. Dellaert, and D. Scaramuzza, "IMU preintegration on manifold for efficient visual-inertial maximum-a-posteriori estimation," in *Robotics: Science and Systems XI*. Robotics: Science and Systems Foundation, jul 2015.

[9] M. Kaess, H. Johannsson, R. Roberts, V. Ila, J. J. Leonard, and F. Dellaert, "iSAM2: Incremental smoothing and mapping using the bayes tree," *The International Journal of Robotics Research*, vol. 31, no. 2, pp. 216–235, dec 2011.

[10] K. Koide, J. Miura, and E. Menegatti, "A portable 3d lidar-based system for long-term and wide-area people behavior measurement," *International Journal of Advanced Robotic Systems*, vol. 16, pp. 1–16, Apr. 2019.

---

## 译注：与本项目的关系

本文是 GLIM 框架在 **先验地图定位** 方向上的早期工作，其"S2S + S2M + IMU 同窗紧耦合优化、地图缺损时平滑退化为纯 LIO"的思想，正是本项目定位子系统（见 [glim_localization_proposal.md](glim_localization_proposal.md) 与详细设计 §4.3）的理论依据。后续期刊版工作即 [Tightly Coupled Range Inertial Localization on a 3D Prior Map Based on Sliding Window Factor Graph Optimization](Tightly%20Coupled%20Range%20Inertial%20Localization%20on%20a%203D%20Prior%20Based%20on%20Sliding%20Window%20Factor%20Graph%20Optimization/)。

[^1]: https://gtsam.org/
