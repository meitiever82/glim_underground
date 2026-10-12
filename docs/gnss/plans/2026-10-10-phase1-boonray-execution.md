# 一期执行计划：RTK 双模块验证 + 杆臂图内估计（boonraygrn 机器）

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在 boonraygrn 机器上完成 RTK 双模块实车数据验证闭环（V1–V6），并实现并验证 GNSS 杆臂的图内在线估计（V7）。

**Architecture:** Part A（Task 0–6）是轮 5 验证闭计划的本机适配执行——不新增算法代码，只建配置集、跑 run 矩阵、回填数值；轮 5 计划（`docs/gnss/plans/2026-10-07-round5-validation-closure.md`）仍是详细步骤的唯一来源，本计划只写**差异**和任务序。Part B（Task 7）是新开发：`gnss_core` 加 `AntennaLeverFactor`，`rtk_global` 加 `lever_mode`。

**Tech Stack:** ROS 2 Humble、colcon、GTSAM 4.3 + gtsam_points（CUDA 12.9）、`gnss_core`（C++17 + GeographicLib 1.52）、GTest、rosbag2（sqlite3）。

**Spec:** `docs/gnss/specs/2026-10-10-phase1-rtk-validation-and-leverarm-design.md`（一期 spec，数值全在其中 §3）；总设计 `2026-09-03-gnss-glim-modules-design.md`。

## Global Constraints

- **绝不 `git push`，绝不创建 PR**；提交末尾加 `Co-Authored-By` 行。
- **数据集只读**：`/home/boonraygrn/Documents/Datasets/` 下只允许写 `<seg>/validation_20261010/` 这一个新建子目录。
- 不改 `finder_ros`；不 stage `glim_underground` 未提交的 `time_keeper.{hpp,cpp}`。
- 跑任何 colcon/ctest/ros2 前：`source /opt/ros/humble/setup.bash && source /home/boonraygrn/glim_ws/install/setup.bash`。
- colcon 构建必须加：`--cmake-args -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPython3_EXECUTABLE=/usr/bin/python3`（本机 Hermes Python 3.14 会劫持 python3）。
- 新增 Python 测试用 `unittest`，不用 pytest。
- 每个新测试先见 RED（或变异验证），证据写进提交信息。
- 所有路径相对 `GLIM_WS=/home/boonraygrn/glim_ws`、`DATA=/home/boonraygrn/Documents/Datasets/tage/hongshaquan/20260915`。

## Review Focus

1. **bag 内时间是 UTC unix 而 `.pos` 是 GPST**——配对错 18 秒就全空：Task 3 的配对脚本必须走 `gnss_core` 的 `pos_io`（识别头部时间系统），不许自写解析。
2. **`quality_sigma_scale` 只有 FIXED 档样本**——只写回 FIXED 档，其余档留默认（spec §5 注）；任务 3 的结果文档必须写明此外推局限。
3. **`sigma_floor` 降档时机**——杆臂填入前保持 1.0 m，降档是独立一次 run（run_A2），两步不许合并。
4. **`rtk_global` 与 `gnss_global` 互斥**——所有 run 的配置快照落盘 `<seg>/validation_20261010/config/`，每个快照里二者只出现一个。
5. **杆臂图内变量与 gauge 时序**——`T_world_enu` 冻结前 `L(0)` 只挂先验；Task 7 的单元测试必须覆盖"bootstrap 前不产因子"这一行为。

---

### Task 0: 数据到位与重建基线

**Files:**
- Modify: 无（只读检查 + 构建）
- 产出：`<segA>/validation_20261010/`、`<segB>/validation_20261010/` 空目录

- [ ] **Step 1: 数据完整性检查**

```bash
DATA=/home/boonraygrn/Documents/Datasets/tage/hongshaquan/20260915
for s in seg_164931_165748 seg_153000_153351; do
  ls $DATA/$s/bag_lio $DATA/$s/gt/enu_origin.txt $DATA/$s/gnss/rtk_check.pos
done
ls $DATA/seg_164931_165748/gt/gt_cgi610_imu_flu.tum $DATA/seg_164931_165748/integration_20260916/run_B/pos/20260915/rtkrcv.pos
```
Expected: 全部存在；`gt_*.tum` 四段齐全。

- [ ] **Step 2: 确认 gnss_core 符号链接与构建**

```bash
ls -la $GLIM_WS/src/gnss_core   # 若不存在：bash $GLIM_WS/src/glim_ext/setup_workspace.sh
cd $GLIM_WS && colcon build --cmake-args -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPython3_EXECUTABLE=/usr/bin/python3
```
Expected: 构建成功；`ls install/glim_ext/lib/librtk_global.so install/glim_ext/lib/librtk_odometry.so` 两个文件**存在**（当前缺失，本步的核心产出）。

- [ ] **Step 3: 测试基线**

Run: `colcon test --packages-select gnss_core glim_ext && colcon test-result --verbose`
Expected: 全绿；若有红，先修再往下（记录红的原因）。

### Task 1: `tage_at128` 配置集 + LiDAR-only 基线（run_C）

**Files:**
- Create: `$GLIM_WS/src/glim_underground/config/tage_at128/*.json`
- 产出：`<segA>/validation_20261010/run_C/{traj_imu.txt,traj_lidar.txt,glim.log}`

按轮 5 计划 Task 1 执行（设计决定 1/2/5 不变：从 `mid360/` 拷贝、CPU 变体、ENU 原点取 `gt/enu_origin.txt`），数值用一期 spec §3（`T_lidar_imu`、话题 `/at128/points` + `/cgi610/imu`）。**差异**：轮 5 计划里所有 `/home/steve` 路径换 `/home/boonraygrn`；产出目录名 `validation_20261010`。

- [ ] Step 1–4：按轮 5 计划 Task 1 的 Step 序列执行（配置集创建 → run_C 回放 → 基线轨迹对真值 RMSE 留档）
- [ ] Step 5: Commit `config/tage_at128/`（只 add 该目录）

### Task 2: run_A（rtk_global 首跑：floor=1m、杆臂 0）→ V1

**Files:**
- Modify: `config/tage_at128/config_ros.json`（`extension_modules` 加 `librtk_global.so`）
- 产出：`<segA>/validation_20261010/run_A/`

- [ ] Step 1–3：按轮 5 计划对应 Task 执行——回放段 A，断言日志含模块初始化行、`T_world_enu=`、`RtkFix` 计数 > 0、`factors_added > 0`（V1）
- [ ] Step 4：轨迹对真值 RMSE 入结果表

### Task 3: §9.3 杆臂/时间偏移 + §9.2 系数标定 → run_A2 → V3/V4

**Files:**
- Modify: `config/tage_at128/config_rtk_global.json`（`T_imu_gnss`、`time_offset`、`quality_sigma_scale`、`sigma_floor`）
- 产出：`<segA>/validation_20261010/eval/`、`<segA>/validation_20261010/run_A2/`

- [ ] Step 1：`estimate_lever_arm` 跑 run_C 轨迹 × `rtk_check.pos`（经 `pos_io` 统一时基）→ 记录 `lever_imu`、`time_offset`、残差 RMS、可观性提示（V4；判据：水平分量落 `[1.600, 0.850]` 附近 ±0.3 m）
- [ ] Step 2：`calibrate_sigma_scale`（ref=`rtk_check.pos`，test=`rtkrcv.pos`）→ FIXED 档系数写回 config（V3，局限写明）
- [ ] Step 3：填杆臂 + `sigma_floor=[0.05,0.05,0.1]`，跑 run_A2；与 run_A 对比（差值即杆臂建模有效性的实车证据）

### Task 4: run_B（gnss_global）+ A/B/C 对比 → V5

- [ ] Step 1–3：按轮 5 计划——合并 bag（`/rtkrcv_node/rtk_fix` + `/gnss/pose` 同包，设计决定 3），`gnss_global` 跑段 A、段 B
- [ ] Step 4：三组（run_C / run_A2 / run_B）× 两段数据的 RMSE 表 + 重复路段点云厚度对比 + 残差分布，写入结果文档；写明"段 A/B 均为高固定率，正式 §12.4 验收仍待 mdc 段"

### Task 5: 注入套件（段 B）→ V2

- [ ] Step 1–2：按轮 5 计划 Task 6——先修 `run_injection_suite.sh` 的三处 `### CHECK` 与 JSONC 解析（`glim_config_patch.py`，若轮 5 未做则带上该工具的 unittest），段 B 跑四用例（wrong_fix / stale / float / lever），`summary.txt` 落 `<segB>/validation_20261010/injection/`

### Task 6: 轮 1.5 门 → V6

前置：V1、V5 已通过。

- [ ] Step 1：`tage_at128/config_ros.json` 的 `extension_modules` 加 `librtk_odometry.so`，回放段 A
- [ ] Step 2：断言日志 `rtk_odometry stats: … factors_added>0`；轨迹 RMSE 不劣于 run_A2（±5% 内）

### Task 7: 杆臂图内变量（G3）→ V7

**Files:**
- Create: `glim_ext/gnss_core/include/gnss_core/antenna_lever_factor.hpp`、`src/antenna_lever_factor.cpp`、`test/test_antenna_lever_factor.cpp`
- Modify: `glim_ext/modules/mapping/rtk_global/`（壳：`lever_mode` 配置、L(0) 节点管理、bootstrap 时序守卫、冻结判据）
- Modify: `config/tage_at128/config_rtk_global.json`（`lever_mode`、`lever_online_axes`）
- Test: gtest `test_antenna_lever_factor.cpp`

**Interfaces:**
- Consumes: `gnss_core::AnchorPipeline::Constraint{id, stamp, p_world, model}`（现有）；总设计 §7.3 的 `body_point` 语义
- Produces: `gtsam::NonlinearFactor` 子类 `AntennaLeverFactor(gtsam::Key poseKey, gtsam::Key leverKey, gtsam::Point3 measured_world, gtsam::Point3 frame_offset, SharedNoiseModel)`，`h(X,L)=X.transformFrom(frame_offset + R_origin_frame·L)`（`frame_offset` 为 `T_origin_frame` 平移部分，调用方算好）；壳新增配置键 `lever_mode`（默认 `fixed`）、`lever_online_axes`（默认 `[x,y]`）

- [ ] **Step 1: 写失败测试**——雅可比数值校验（`numericalDerivative21` 对 X 与 L 两块）；L 锁先验时与 `AntennaPriorFactor` 残差一致；`lever_online_axes=[x,y]` 时 z 增量为零
- [ ] **Step 2: 跑测试确认 RED**
- [ ] **Step 3: 实现 `AntennaLeverFactor`**（解析雅可比，参考 `AntennaPriorFactor` 的 `transformFrom` 雅可比链 + 对 L 的 `X.rotation()` 块）
- [ ] **Step 4: 测试 GREEN，commit**
- [ ] **Step 5: 壳接入**——`lever_mode=online` 时：gauge 冻结前 L(0) 只挂先验（σ=0.3 m）；冻结后每个 submap 加 `AntennaLeverFactor`；协方差/变化量冻结判据（spec §4.2）；z 轴守卫
- [ ] **Step 6: 段 A 回放 online vs fixed**——验收 spec §4.3 第 2–4 条（收敛差 <5 cm、指标不劣化、转弯段残差 ≤50%）
- [ ] **Step 7: Commit + 结果写入 field 文档**

### Task 8: 结果文档与回填

- [ ] Step 1：写 `docs/gnss/field/2026-10-10-phase1-validation-boonray.md`——V1–V7 逐项过/不过 + 全部数值表 + 已知局限（固定率偏高、垂向弱可观、mdc 段缺失）
- [ ] Step 2：回填总设计 §13 风险表（杆臂、时间偏移两项）与轮 5 计划验收勾选；commit（只 add 这两个文档与本计划打勾版）
