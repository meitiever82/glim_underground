# 一期设计：RTK 双模块实车验证 + GNSS 杆臂图内在线估计

> 2026-10-10，boonraygrn 机器。上游 spec：`docs/gnss/specs/2026-09-03-gnss-glim-modules-design.md`（下称"总设计"）；执行计划：`docs/gnss/plans/2026-10-10-phase1-boonray-execution.md`。
> 相关调研（二期用）：`docs/features/spec/online_extrinsic_calibration_survey.md`。

## 1. 背景与目标

总设计轮 1/轮 1.5 的代码（`rtk_global`、`rtk_odometry`、`gnss_core` 工具链）已写完，但**从未在实车数据上闭环验证**（轮 5 计划 2026-10-07 制定，未执行）。本期的直接动机：外参标定系列工作（见在线标定调研）以"GNSS 全局约束可信"为前提——RTK 约束没验证过，后续一切免谈。

**目标（按序）：**

1. **G1 验证闭环**：在 boonraygrn 机器上执行轮 5 验证——`rtk_global` 被 GLIM 真实加载并产出因子（总设计 §12.2）、合成注入四用例（§12.3）、`quality_sigma_scale` 与 `T_imu_gnss`/`time_offset` 实测标定（§9.2/§9.3）、`rtk_global` vs `gnss_global` A/B 对比（§12.4）；全部通过后开 `rtk_odometry` 启用门（§10 轮 1.5）。
2. **G2 建图质量判定**：回答"挂 RTK 约束后建图是否更好"——轨迹 RMSE（对 INS 真值）、重复路段点云厚度、优化器残差分布三项指标，LiDAR-only 基线（run_C）作对照。
3. **G3 杆臂图内估计**：把 `T_imu_gnss` 从配置常量提升为因子图变量，在线估计，验证是否优于"离线估计+写回配置"。

**非目标（明确排除）：**

- LiDAR-IMU 外参（`T_lidar_imu`）在线标定——**二期**，见调研文档 §4 Phase 1；
- RTK–LiDAR 时间偏移 `time_offset` 的图内估计（本期仍用 §9.3 离线工具估计后写配置）；
- `T_world_enu` 联合优化（总设计 §7.乙，仍冻结）；
- 真车（天准小车）部署——本期只用红沙泉数据离线回放，真车是 G1 通过后的下一步；
- 任何对 `finder_ros` 驱动的改动。

**反目标（做到了也算失败）：**

- 为通过验收而放宽指标或挑数据段；
- `rtk_global` 与 `gnss_global` 同时启用（总设计 §7.6：二选一，同开即重复约束）。

## 2. 现状事实（2026-10-10 本机核实）

- boonraygrn 机器：ROS 2 Humble ✓、GTSAM+gtsam_points（`/usr/local/lib/cmake`）✓、Iridescence ✓、GeographicLib 1.52 ✓、CUDA 12.9 + RTX 5060 Laptop ✓、磁盘余量 565G ✓。
- `glim_ws/src/` 含 `glim_ext`（master `fb21d3f`，轮 4a 已合并）、`glim_underground`（`743131a`）、`glim_ros2`、`gnss_core`、`gnss_msgs`、`gnss_bringup` 等。
- `rtk_global` / `rtk_odometry` 源码在 `glim_ext/modules/{mapping,odometry}/`，CMake `ENABLE_RTK_GLOBAL` / `ENABLE_RTK_ODOMETRY` 默认 ON。
- **但当前 `install/glim_ext/lib/` 无 `librtk_global.so` / `librtk_odometry.so`**——install 是旧构建，必须重建。
- `gnss_core/tools/` 齐备：`synth_rtk_fix`、`estimate_lever_arm`、`calibrate_sigma_scale`、`pos_to_rtkfix_bag.py`、`run_injection_suite.sh`、`traj_rmse.py`。
- 数据：`/home/boonraygrn/Documents/Datasets/tage/hongshaquan/20260915/`（用户负责拷贝，结构不变，只读使用）。

## 3. 数据与数值（逐字沿用轮 5 计划，路径换前缀）

- 段 A（主用）：`…/20260915/seg_164931_165748`——497 s / 1329 m / RTK 事后固定率 97.6%。
- 段 B（注入套件）：`…/20260915/seg_153000_153351`——231 s / 1364 m / 固定率 93.9%。
- `T_lidar_imu` = `[-1.886806, 1.006244, 0.651553, 0,0,0,1]`（GLIM 约定 `p_lidar = T_lidar_imu * p_imu`）。
- `T_imu_gnss` 手量值 = `[1.600, 0.850, 2.880]`（注意 `cgi610_imu_flu` 原点为推算值未验证——§9.3 的测量同时是对它的验证）。
- 杆臂未标定期间 `sigma_floor` 保持 `[1.0,1.0,1.0]`；杆臂填入后才降 cm 级（总设计 §7.3 v2）。
- 轮 5 已测的第一组数（待正式留档）：FIXED 档 486 历元，rmse_h 0.003 m / rmse_v 0.009 m，ratio 中位数 0.430。

## 4. G3 设计：杆臂作为图内变量

### 4.1 问题

总设计 §7.3 的 `AntennaPriorFactor`：`h(X) = X.transformFrom(body_point)`，`body_point` 是常量（`rtk_global` 轮 1 取 `lever_imu` 配置值）。杆臂错 → 转弯段系统性残差 → 要么污染轨迹、要么被 Huber 误杀好解（总设计 §7.3 v2 已分析）。离线 §9.3 能估一次，但支架拆装/振动后需重估。

### 4.2 方案

在 `rtk_global`（global mapping 图）中把杆臂提升为变量节点：

```
L(0) : gtsam::Point3        # 天线在 IMU 系的位置
新因子: AntennaLeverFactor(X(submap_id), L(0), measured_world, model)
        h(X, L) = X.transformFrom(T_origin_frame(t) · L)
先验:   PriorFactor<Point3>(L(0), [1.600, 0.850, 2.880], σ=0.3 m)
```

- **只做 `rtk_global` 侧**。odometry 侧（`rtk_odometry`）是固定滞后平滑器，持久变量与边缘化的交互风险大（同调研文档 §4 Phase 1 的坑），一期不碰——global 图是全量 iSAM2，无此问题。
- **初值**：§9.3 离线估计值（没有就用 `[1.600, 0.850, 2.880]`）。
- **可观测性守卫**：文献证明一般运动下 GNSS 外参的水平两方向可观、**垂向弱可观**（见调研文档 [10]）。默认配置 `lever_online_axes: [x, y]`——z 分量固定取先验值不更新；`[x, y, z]` 仅在轨迹含明显起伏时手动开。
- **冻结判据**：`L(0)` 边际协方差对角元 < 1 cm² 且连续 10 个 submap 变化 < 5 mm → 换强先验冻结（M-LOAM 收敛冻结同款思路）。
- **开关**：`config_rtk_global.json` 加 `lever_mode: "fixed" | "online"`，默认 `fixed`（行为与现状完全一致）。

### 4.3 验收

1. 单元测试：`AntennaLeverFactor` 解析雅可比过 `numericalDerivative21`；`L` 固定为先验值时退化为现 `AntennaPriorFactor` 行为。
2. 段 A 回放：online 模式收敛值与 §9.3 离线估计值差异 < 5 cm（水平）；
3. 建图指标不劣于 fixed 模式（轨迹 RMSE、点云厚度）；
4. 转弯段 RTK 因子残差 RMS：online ≤ fixed（零杆臂）模式的 50%。

## 5. 验收总表

| # | 项 | 判据 | 来源 |
|---|---|---|---|
| V1 | 模块真实加载 | 段 A 回放日志含模块初始化 + `T_world_enu=` + `RtkFix` 计数 > 0 | §12.2 |
| V2 | 注入四用例 | wrong_fix/stale/float/lever 各达 §12.3 期望行为 | §12.3 |
| V3 | 系数标定 | `quality_sigma_scale` FIXED 档实测写回 `tage_at128/config_rtk_global.json` | §9.2 |
| V4 | 杆臂/时间偏移 | `estimate_lever_arm` 输出落手量值附近（水平分量），残差 RMS 报告 | §9.3 |
| V5 | A/B/C 对比 | 段 A+B：`rtk_global` 不差于 `gnss_global`、优于或持平 LiDAR-only | §12.4* |
| V6 | 轮 1.5 门 | `rtk_odometry` 启用后日志 `factors_added > 0`，轨迹不劣化 | §10 |
| V7 | G3 杆臂在线 | §4.3 四条全过 | 本 spec |

\* §12.4 正式验收（含浮动/差分中断路段）仍阻塞于 `mdc-20260915-153000` 数据段，本期用段 B（6.1% 浮动）做尽量接近的近似，并在结果文档写明局限。

## 6. 风险

| 项 | 处理 |
|---|---|
| 本机 colcon 被 Hermes Python 3.14 劫持 | 构建加 `-DPYTHON_EXECUTABLE=/usr/bin/python3 -DPython3_EXECUTABLE=/usr/bin/python3` |
| 轮 5 计划路径全部硬编码 `/home/steve` | 执行计划统一替换为 `/home/boonraygrn`；数据集只读，产出写 `<seg>/validation_20261010/` |
| `glim_underground` 有未提交的 `time_keeper` 改动 | 不 stage、不还原，原样保留（沿用轮 5 约束） |
| 杆臂图内变量与 gauge 冻结的时序 | `T_world_enu` bootstrap 冻结前，杆臂节点只挂强先验不加量测因子 |
| 段 A/B 固定率都偏高，浮动档标定样本不足 | 只写回 FIXED 档系数，其余档留默认并在结果文档标注 |
