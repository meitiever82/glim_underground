# 20261007 RTK run 毁图根因诊断与修复验证（boonraygrn 机器，段 A）

> 日期：2026-10-16 ｜ 执行：Hermes（boonraygrn 机器）｜ 数据：`seg_164931_165748`
> 结论一句话：**毁图与 RTK 模块无关——是前端 CT 里程计在 100m 裁距下于远角急弯（t≈107s）发散；RTK 模块逻辑全部验证无罪。修复 = `distance_far_thresh` 100→200 + 杆臂按雷达系填。**

## 1. 起始症状（20261007, steve 机器）

| run | 配置 | RMSE |
|---|---|---|
| run_C | CPU LIO 基线 | 1.83m ✓ |
| run_ct_only | CT 纯 LiDAR | 2.15m ✓ |
| run_rtkglobal_ct | CT + rtk_global | 81.6m ❌ |
| run_rtkodom_ct | CT + rtk_odometry | 76.9m ❌ |
| run_rtkodom_ct_strong | CT + rtk_odometry 强约束 | segfault ❌ |

## 2. 排查过程（按排除顺序）

| 嫌疑 | 验证方法 | 结果 |
|---|---|---|
| 时间基准（GPST/UTC 18s） | 读 bag 内 rtk_fix：`gnss_time` 与 `header.stamp` 同基，恒差 50ms | 排除 |
| RTK 测量质量 | `rtkrcv.enu` 对 gt：系统差 3.4m，z 差 2.78m ≈ 杆臂 z=2.88 | 排除（数据健康） |
| gauge bootstrap 错误 | 用干净段（前 110s）独立重解 2D Umeyama：yaw=-22.49° 残差 0.28m；模块打印 -23.30°；理论 -23.9° | 排除（三者吻合） |
| AntennaPriorFactor 实现 | 读码（GTSAM 内建 transformFrom 雅可比）+ 独立 LM 集成测试（X(1) 被正确拉动） | 排除 |
| 锚点（stamp, pose）配对 | submap origin = 中间帧（`sub_map.hpp:31` frames[size/2]），pose 与 stamp 一致 | 排除 |
| 噪声策略/门控 | 静态读码与 spec §7.2 一致；bag 内 σ 字段诚实 | 排除 |
| enable_imu 的 1e6 零值因子 | `enable_imu=false` 与 true 的 run 输出 **bit 级相同**（md5 一致）——零速度零偏的自洽约束，数值惰性 | 排除（但见 §5 建议） |
| **前端发散** | v_base（无 RTK）odom 轨迹同样崩 80.9m；与 steve 干净 run 点对点对比：**t<107.2s 差异 ≤5cm，之后瞬间 141m** | **坐实** |

**关键反直觉证据**：`v_base`（完全无 RTK 模块）崩了 79.9m，与挂 RTK 的 run 同为坏模态——RTK 因子从头到尾是旁观者。

## 3. 根因

**主因：`config_preprocess.json` 的 `distance_far_thresh=100m`。** 该场地 ~200×200m，水平约束全靠远墙（本数据集 KISS 调参报告结论：max_range 100→200 = ATE 1.45m→0.57m）。车辆在远角（t≈107s，(110,-52) 附近）以 3.9m/s 急转弯时，100m 裁距把约束墙全部裁掉，CT 纯 LiDAR 前端配准发散。发散对构建/调度数值敏感（steve 机器同配置幸免，本机确定性复现），表现为"双模态"。

**次因（真实但非本次元凶）**：杆臂坐标系。CT/纯 LiD 模式下 GLIM 估计系 = 雷达系（CT 代码中 `T_lidar_imu` 恒单位阵），`T_imu_gnss` 必须填雷达系值：`gnss_antenna − at128_flu = [-0.286806, 1.856244, 3.531553]`。填 IMU 系值 `[1.6, 0.85, 2.88]` 引入 2.2m 系错；带杆臂因子残差含旋转项 `R·lever`，错杆臂会迫使优化器旋转子图凑残差。spec §7.3 v2 的 "sigma_floor=杆臂量级(~1m)" 规则在本车实际杆臂模长（4.0m）下失效，应改为"≥杆臂模长"。

## 4. 修复验证（本机，段 A，同一 bag）

| run | 配置要点 | odom RMSE | traj RMSE |
|---|---|---|---|
| v_base | CT，无 RTK，裁距 100m | 80.9m ❌ | 79.9m ❌ |
| v_base_1x | 同上 + 1 倍速回放 | bit 级相同 ❌ | 排除回放超速嫌疑 |
| **v_base_r200** | CT，无 RTK，**裁距 200m** | **2.65m** ✓ | **2.27m** ✓ |
| **v_both_r200** | CT + rtk_odometry + rtk_global，雷达系杆臂，裁距 200m | **2.65m** ✓ | **2.25m** ✓ |

`v_both_r200` 中：rtk_odometry 注入 4250 因子、rtk_global 注入 43 因子，全程无撕裂。RTK 的净收益在本段（高固定率、场地良性）尚不显著（2.27→2.25m），其价值要在退化/差分中断段验证（待 mdc 段数据）。

## 4.1 统一评估口径（evo，对 gt_at128_flu）与定权扫参（2026-10-16 补）

**口径修正**：§4 的 RMSE 是把雷达系轨迹对到 cgi610 系真值的结果，含 2.3m 设备杆臂污染。统一成调研同款 evo 方法（associate 0.06s + SE3 对齐 + ATE translation）并对 `gt_at128_flu` 后：

| 配置 | ATE (m) |
|---|---|
| 调研独立实验（KISS+GTSAM 融合，复现验证） | 0.155 |
| 无 RTK（裁距 200m） | 0.542 |
| RTK 双模块 + floor=1m | 0.604（负收益） |
| **RTK 双模块 + floor=4m + 手量杆臂** | **0.290** ⭐（RTK 净收益 1.9×） |
| 同上但杆臂置零 | 0.359（杆臂值在真实做功） |
| 估计器杆臂 + floor=1/2/4m | 0.582 / 0.369 / 0.383 |

定权结论：
- **×3 重复后（2026-10-16 晚）结论修正**：前端 run 间方差 ±0.15~0.35m，上述单跑排名全部降级。补齐重复后：v_base_r200 = 0.542/0.406/0.434（mean 0.461）；v_both_r200_f4 = 0.290/0.461/0.432（mean 0.394）。**RTK 净收益真实量级 ≈15%（0.461→0.394），不是单跑显示的 1.9 倍**，n=3 下区间仍重叠，统计上弱
- "floor=4 是甜点""手量杆臂优于估计值"等结论均为单跑对比，差值落在噪声界内，**撤回撤不量化表述**；floor/杆臂两轴的扫描需确定性前端后重做
- `estimate_lever_arm` 输出（[-0.406, 1.885]）不敌手量值：其 1.15m 残差警告为真——地图倾斜污染 + z 不可观。**结论：沿用 `extrinsics_tage_truck.yaml` 手量值，at128 原点独立验证仍挂起**
- 距调研 0.155 的剩余差距归因：融合层噪声建模（调研用逐帧信息矩阵+批量融合）与前端差异，属二期议题；**当前瓶颈是前端非确定性，不是 RTK 定权**

## 5. 修复与建议清单

### 配置层（立即，一期目标形态）
1. `distance_far_thresh`: 100 → **200**（该场地必须；其它场地按尺度评）
2. `T_imu_gnss` 填**雷达系**值 `[-0.286806, 1.856244, 3.531553]`（两个 rtk 配置文件都改）
3. `config_global_mapping_cpu.json`: `enable_imu=false`（CT 模式下必须，虽本轮证其数值惰性）
4. `sigma_floor` 规则修订：**杆臂模长量级**（本车 4m），不是固定 1m

### 代码层（建议，小改）
5. `global_mapping.cpp:170` 与 `sub_mapping.cpp:180`：frame_id≠IMU 时**跳过** IMU 因子块而不是 warn-and-continue（消除退化的 1e6 零值因子，以及 5000 行警告刷屏）
6. rtk_odometry 排障日志 `insert N RTK prior factors` 保持 info 级（本次诊断加的，建议保留）
7. 已知架构限制（一期内不做）：CT 里程计的 RTK 因子进的是**外层**固定滞后平滑器，内层 LM 看不到 RTK——前端内层发散时外层救不回来（探针实验证明：σ=1mm 的因子也只在 87m 处拔河）。要让帧级 RTK 真正兜底，需把因子注入 CT 内层 LM（二期设计议题）

### 待办（不阻塞）
8. 双模态的数值根（为何 steve 机器幸免）：两边 gtsam_points/GLIM 提交与编译选项对比
9. mdc 段（含遮挡/差分中断）到位后做 §12.4 正式验收
10. 杠杆语义文档化：`config_rtk_*.json` 的 `T_imu_gnss` 键注释改为"GLIM 估计系"

## 6. 本次诊断产生的资产

- 复现/二分/确认脚本：`~/.hermes/cache/scratch/{run_bisect,run_decisive,run_confirm,run_rtkodom,run_base}.sh`（一次性，scratch 24h 后清除；如需保留请告知，挪进 gnss_core/tools）
- 全部 run 的配置快照与 dump：`seg_164931_165748/validation_20261013_boonray/`
- 未提交代码改动（glim_ext）：rtk_odometry drain 日志 debug→info（建议保留）
