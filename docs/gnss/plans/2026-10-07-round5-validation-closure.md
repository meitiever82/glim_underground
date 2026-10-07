# 轮 5：验证闭环 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 把 spec 里"代码写完但从未验证"的五项一次性闭合——`librtk_global.so` 被 GLIM 真实加载并产出因子（§12.2）、合成注入套件跑出结果表（§12.3）、`quality_sigma_scale` 与 `T_imu_gnss`/`time_offset` 由实测写回 config（§9.2/§9.3）、`rtk_global` 与 `gnss_global` 在同一段实车数据上 A/B 对比（§12.4），并据此打开轮 1.5 `rtk_odometry` 的启用门。

**Architecture:** 不新增模块。用 2026-09-15 红沙泉两段数据（`bag_lio` 的 AT128 + CGI-610 IMU，已有 INS 真值与后处理 RTK）新建一套 GLIM 配置集 `config/tage_at128/`，把既有离线工具（`synth_rtk_fix` / `pos_to_rtkfix_bag.py` / `estimate_lever_arm` / `calibrate_sigma_scale` / `run_injection_suite.sh` / `traj_rmse.py`）真正跑起来。新代码只有三小块：`gnss_core` 的 ENU 轨迹导出（给 `gnss_global` 当输入）、`pos_to_rtkfix_bag.py` 的两项扩展（GPS 周/周内秒时间列、写 `PoseWithCovarianceStamped`）、以及一个带注释容错的 GLIM 配置补丁工具（把 `run_injection_suite.sh` 里三处 `### CHECK` 和一个会直接崩的 `json.load` 修掉）。其余产出是数值结果与文档回填。

**Tech Stack:** ROS 2 Humble、colcon、GLIM（`glim` / `glim_ros` / `glim_ext`）、GTSAM 4.3 + gtsam_points 1.2.2（本机含 CUDA）、`gnss_core`（纯 C++17 + GeographicLib 1.52）、GTest、Python 3.10 `unittest`、rosbag2（sqlite3）、RTKLIB-EX 2.5.1。

**Spec:** `src/glim_underground/docs/gnss/specs/2026-09-03-gnss-glim-modules-design.md`（本计划闭合其 §9.2、§9.3、§12.2、§12.3、§12.4，以及 §10 轮 1 的验收项 V4–V10、轮 1.5 的启用门）

**前置审计：** 本计划的缺口清单来自 2026-10-05 的 spec/plan 完成度审计。该审计有三条结论经复核为错或已过时，这里按复核后的事实写：
- `librtk_global.so` **已经**出现在 `config/casbot/config_ros.json` 的 `extension_modules` 里（审计说"任何 config_ros.json 里都没有"是错的），但 `casbot` 配置集的话题是 `/rslidar_points` + `/rslidar_imu_data`，与本数据集不符，且**从未被实际运行过**——§12.2 仍然没有任何一次通过记录。
- 工作区没有 `build/`/`install/` 是因为距离上次工作已过约三周、工作区被清理，不是"从未构建"。
- `ctest` 全红是本机 pyenv 的 python shim 顶掉系统 python3 导致 `ModuleNotFoundError: No module named 'ament_cmake_test'`；`source /opt/ros/humble/setup.bash` 后 `/home/steve/worktrees/build-round4a` 25 个测试全绿。**凡本计划里跑 ctest/colcon test 的步骤，都必须先 source ROS。**

---

## Global Constraints

以下为项目级约束，每个 Task 的要求都隐含包含本节。数值均从 spec 与数据集文档逐字抄来。

**流程**
- **绝不执行 `git push`，绝不创建 PR。** 分支在本地做完、提交完整即止，推送与开 PR 由维护者本人完成。
- 提交信息末尾加 `Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>`。
- 每个新增测试都必须**亲眼见过它失败**（先写桩让它 RED，或对实现做一次变异让它红），并把 RED/变异证据写进该提交的 message。做不到的，在 message 里如实写明原因。
- **数据集只读**：`/home/steve/Documents/Datasets/` 下只允许在 `<seg>/validation_20261007/` 这一个新建子目录里写东西。已有的 `bag/`、`bag_lio/`、`gnss/`、`gt/`、`integration_20260916/` 一律不得改动或删除。
- **不改 `finder_ros`**（`/home/steve/Documents/GitHub/ztpilot/finder_ros`）的任何驱动源码；本计划不需要它。
- `glim_underground` 仓库里只 `git add` 本计划明确列出的路径；**绝不 stage** 未提交的 `include/glim/util/time_keeper.hpp` 与 `src/glim/util/time_keeper.cpp`。
- 本机 `~/.local` 的 pytest 是 9.1.1，与 ROS Humble 的 `launch_testing` pytest 插件不兼容。**新增 Python 测试一律用 `unittest`**，不要用 pytest、不要用 `ament_add_pytest_test`。
- 跑任何 `ctest` / `colcon test` / `ros2 run` 之前先 `source /opt/ros/humble/setup.bash && source /home/steve/glim_ws/install/setup.bash`。

**数值与路径（逐字）**
- 数据段 A（主用，§12.2/§9.2/§9.3/§12.4）：`/home/steve/Documents/Datasets/tage/hongshaquan/20260915/seg_164931_165748`
  —— 497 s，行驶 1329 m，最高 5.5 m/s，航向累计 1778°，RTK 事后固定率 97.6%；前静止 53 s、后静止 93 s。
- 数据段 B（短，§12.3 注入套件）：`/home/steve/Documents/Datasets/tage/hongshaquan/20260915/seg_153000_153351`
  —— 231 s，行驶 1364 m，最高 8.0 m/s，航向累计 1731°，固定率 93.9%。
- ENU 原点（段 A）：`1789462171.000 44.470257133 90.294514984 610.6395`（`<segA>/gt/enu_origin.txt`）
- ENU 原点（段 B）：`1789457400.000 44.470267482 90.294436345 610.5679`（`<segB>/gt/enu_origin.txt`）
- 各坐标系原点在 `base_link` 中的位置（m，`README_LIO.md` §3）：`at128_flu (8.736806, -0.006244, 1.628447)`、`jt128_flu (-0.92, -0.10, 1.82)`、`cgi610_imu_flu (6.85, 1.00, 2.28)`、`gnss_antenna (8.45, 1.85, 5.16)`。**所有 `*_flu` 系轴向与 `base_link` 相同（x 前、y 左、z 上），只有原点不同；因此任意"雷达+IMU"组合外参旋转都是单位阵。**
- 由此导出、本计划要写进配置的两个量：
  - `T_lidar_imu`（GLIM 约定 `p_lidar = T_lidar_imu * p_imu`，TUM 格式 `[x,y,z,qx,qy,qz,qw]`）= `cgi610_imu_flu − at128_flu` = **`[-1.886806, 1.006244, 0.651553, 0.0, 0.0, 0.0, 1.0]`**
  - `T_imu_gnss`（天线位置在 IMU 系下，`config_rtk_global.json`）= `gnss_antenna − cgi610_imu_flu` = **`[1.600, 0.850, 2.880]`**，§9.3 的实测应落在它附近。注意 `cgi610_imu_flu` 的原点在 `extrinsics_tage_truck.yaml` 里标注为**推算值、未验证**，所以 §9.3 的测量同时是对它的验证。
- `bag_lio` 话题（两段相同）：`/at128/points`（PointCloud2，4966 条，10 Hz，velodyne 字段 `x y z intensity` float32 + `ring` uint16 + `time` float32 相对秒）、`/cgi610/imu`（Imu，49731 条，100 Hz，m/s² 与 rad/s，静止时 acc.z ≈ +9.8，无姿态）、`/jt128/points`、`/jt128/imu`、`/cgi610/odom`、`/cgi610/fix`、`/tf_static`。**全部时间为 UTC unix。**
- 真值：`<seg>/gt/gt_cgi610_imu_flu.tum`、`gt_at128_flu.tum`、`gt_base_link.tum`、`gt_jt128_flu.tum`，格式 `t x y z qx qy qz qw`，在 ENU 系下，10 Hz 4974 行。`/cgi610/odom` 位置 1σ：水平约 1.1 cm、高程约 3.3 cm；航向约 0.09°。
- 实时 RTK 解（段 A，来自现场联调 run_B）：`<segA>/integration_20260916/run_B/pos/20260915/rtkrcv.pos`，497 历元、Q1 489 / Q2 8，表头 `time=GPST`，时间列是日历格式。
- 后处理基准：`<seg>/gnss/rtk_check.pos`，表头 `time=GPST`，时间列是 **GPS 周 + 周内秒**（`2436 204589.000`）；段 A 为 498 历元 Q1 486 / Q2 12，段 B 为 231 历元 Q1 217 / Q2 14。**这是天线处位置。**
- 已测得的 §9.2 第一组数字（段 A，ref=`rtk_check.pos`，test=run_B 的 `rtkrcv.pos`，待本计划正式留档）：
  ```
  quality      n  rmse_h(m)  rmse_v(m)  ratio_median  ratio_rms  ratio_mean
  FIXED      486      0.003      0.009         0.430      0.614       0.492
  ```
- spec §12.3 的四条期望行为（逐字）：`wrong_fix`：开鲁棒核轨迹 RMSE 基本不变、关鲁棒核明显恶化；`stale`：该段不产生因子；`float`：因子仍加但权重降 5 倍；`lever`：不设杆臂时先验有系统性偏移、设置后消失。
- spec §12.4 验收标准（逐字）：**在含浮动解或差分中断的路段，`rtk_global` 的轨迹 RMSE 应优于 `gnss_global`，且不出现被单个坏历元拽偏的情形。**
- spec §10 轮 1.5 启用门（逐字）：`rtk_odometry` 的**启用**须待 `rtk_global` 实车验证通过。
- `sigma_floor` 规则（spec §7.3 v2，`config_rtk_global.json` 注释逐字）：`WHILE T_imu_gnss IS UNCALIBRATED KEEP THIS AT LEVER-ARM MAGNITUDE (~1.0 m)`；杆臂填进去之后才降到 cm 级。

**环境事实（已核实，2026-10-07）**
- `/opt/ros/humble` 存在；GTSAM 与 gtsam_points 1.2.2（`GTSAM_POINTS_USE_CUDA 1`）在 `/usr/local/lib/cmake/`；Iridescence 在 `/usr/local/lib/cmake/iridescence`；`libgeographic-dev 1.52` 已装；CUDA 12.6 + RTX 4060 Laptop 可用；`/home` 剩余 2.1 TB。
- `glim_ws/src/` 下有 `glim_underground`、`glim_ros2`、`glim_ext`、`gnss_core`、`gnss_msgs`、`gnss_bringup`、`glim_ext`、`hesai_ros_driver2`、`rslidar_*`、`timesync_diag`。`glim_ws/build`、`glim_ws/install` 目前**不存在**，Task 1 负责重建。
- `glim_ext` 的 `master` 已是 `fb21d3f`（PR #7、#8 已合并），工作区干净。

---

## 文件结构

| 文件 | 责任 | Task |
|---|---|---|
| `glim_underground/config/tage_at128/*.json` | 本车（踏歌宽体车 AT128P + CGI-610）的 GLIM 配置集：话题、外参、预处理、扩展模块、`config_rtk_global.json` / `config_rtk_odometry.json` / `config_gnss_global.json` | 1 |
| `glim_ext/gnss_core/include/gnss_core/enu_track.hpp` + `src/enu_track.cpp` | `.pos` 记录 → 局部 ENU 轨迹（含质量门限、σ 换序、原点选取）与文本序列化。纯函数，不碰文件系统 | 2 |
| `glim_ext/gnss_core/tools/pos_to_enu.cpp` | 上面那层的 CLI 外壳：读 `.pos` 写 `.enu` | 2 |
| `glim_ext/gnss_core/test/test_enu_track.cpp` | `enu_track` 的 gtest | 2 |
| `glim_ext/gnss_core/tools/pos_to_rtkfix_bag.py` | 加 GPS 周/周内秒时间列解析；加 `--pose-enu`/`--pose-topic` 写 `PoseWithCovarianceStamped` | 3 |
| `glim_ext/gnss_core/tests/test_pos_to_rtkfix_bag.py` | 上面两项的 unittest（纯解析函数，不需要 ROS） | 3 |
| `glim_ext/gnss_core/tools/glim_config_patch.py` | 注释容错的 JSONC 读取 + GLIM 配置集拷贝改写（dump_path / extension_modules / rtk_global 参数） | 4 |
| `glim_ext/gnss_core/tests/test_glim_config_patch.py` | 上面的 unittest | 4 |
| `glim_ext/gnss_core/tools/run_injection_suite.sh` | 改用 `glim_config_patch.py`、修三处 `### CHECK`、每用例跑完即删合成 bag | 6 |
| `glim_ext/gnss_core/tools/README.md` | §2 的合成示例换成实测、§6 现场待办勾选、新增 `pos_to_enu` 与注入套件结果 | 5, 6 |
| `glim_underground/docs/gnss/field/2026-10-07-round5-validation-closure.md` | 本轮全部数值结果与结论（给维护者看的主产出） | 1–7 |
| `glim_underground/docs/gnss/specs/2026-09-03-gnss-glim-modules-design.md` | §13 风险表按实测结果更新（杆臂、时间偏移两项） | 7 |
| `glim_underground/docs/gnss/plans/2026-09-03-round1-rtk-global.md` | V4–V10 验收项按实测勾选或写明未达原因 | 7 |

数据产物（**不进 git**，全部写在数据集的新建子目录下）：

```
<segA>/validation_20261007/
├── config/            # 每次 run 实际用的 GLIM 配置集快照(由 glim_config_patch.py 生成)
├── run_C/             # LiDAR-only 基线:traj_imu.txt / traj_lidar.txt / glim.log
├── run_A/             # rtk_global
├── run_A2/            # rtk_global + 标定后的杆臂与 sigma_floor
├── run_B/             # gnss_global
├── run_A_odom/        # rtk_global + rtk_odometry(轮 1.5 门)
├── gnss/              # rtkrcv.enu / rtk_check.enu / 合并后的 bag
└── eval/              # RMSE 表、calibrate_sigma_scale 与 estimate_lever_arm 输出、结论
<segB>/validation_20261007/
└── injection/         # run_injection_suite.sh 的工作目录与 summary.txt
```

---

## 设计决定

1. **新建 `config/tage_at128/` 而不是改 `casbot/`。** `casbot` 是另一台车的配置集（`/rslidar_points`、`T_lidar_imu` 带 90° 旋转），改它会毁掉一份已有配置。新配置集从 `mid360/` 拷贝（它同时带 `*_cpu.json` 与 `*_gpu.json` 三件套），只改必须改的键。
2. **里程计/建图走 CPU 变体（`config_odometry_cpu.json` / `config_sub_mapping_cpu.json` / `config_global_mapping_cpu.json`）。** 本机有 CUDA，但 A/B/C 三组只在"全局约束"上不同，CPU 路径让结论可在无 CUDA 的机器上复现；GPU 变体文件照样拷进配置集，换一行 `config.json` 即可切。工作区仍按 `BUILD_WITH_CUDA=ON` 默认构建（后续 GPU 工作复用同一份 install）；若 CUDA 编译失败，退回 `-DBUILD_WITH_CUDA=OFF` 并在结果文档记一行——本计划没有任何一步依赖 CUDA。
3. **`gnss_global` 的输入用离线转好的 `PoseWithCovarianceStamped` 写进同一个 bag，不写在线转换节点。** `glim_rosbag` 只回放 `filter.topics`（imu/points/image + 各扩展模块声明的订阅），所以消息必须在包里。把 `/rtkrcv_node/rtk_fix`（`gnss_msgs/RtkFix`，给 `rtk_global`）与 `/gnss/pose`（`PoseWithCovarianceStamped`，给 `gnss_global`）**同时**写进一个合并 bag，A/B/C 三组共用一份输入，差异只剩模块本身——这是 A/B 公平性的前提。
4. **经纬高→ENU 的换算放 C++（`pos_to_enu`），不在 Python 里重写测地学。** 本机既无 `geographiclib` 也无 `pyproj` 的 Python 绑定；`gnss_core::LlaToEnu` 是已测过的 GeographicLib `LocalCartesian` 封装。`pos_to_rtkfix_bag.py` 只读 `pos_to_enu` 产出的 `.enu` 文本，不做任何坐标换算。
5. **ENU 原点一律取 `<seg>/gt/enu_origin.txt` 的三个数**，不取 `.pos` 第一条。这样 `gnss_global` 看到的坐标与 `gt/*.tum` 同原点，轨迹图可以直接叠；`rtk_global` 则仍自己估 `T_world_enu`（它的 `enu_origin` 留空）。
6. **§12.4 的 A/B 之外加一组 C（LiDAR-only）。** spec 只要求 A vs B，但没有 C 就无法判断"谁更好"里有多少是 GNSS 的功劳；C 同时是 §12.3 注入套件的参考轨迹。
7. **§12.4 跑两段数据。** 段 A 有真实实时解（run_B 的 `rtkrcv.pos`，489/497 固定）但 98.4% 固定、无差分中断，**不满足 spec §12.4 "含浮动解或差分中断的路段"的前提**；段 B 的后处理解有 14/231 浮动（6.1%）。两段都跑，并在结果文档里明确写出：段 A 的结论只能说明"好解下不劣化"，真正的验收仍需一段含遮挡/隧道/长差分中断的数据（现场文档建议同日 `mdc-20260915-153000` 整段）。
8. **§9.2 的标定结果写进 `tage_at128/config_rtk_global.json`，不动 `glim_ext/config/config_rtk_global.json` 的默认值。** 实测只拿到 FIXED 一档（段 A 98.4% 固定、段 B 93.9%），FLOAT/DGPS/SINGLE 三档样本不足以定系数；把单档结果推成全局默认是过度外推。默认文件只改注释，指向实测结论的位置。
9. **`sigma_floor` 的降档必须在 §9.3 之后、且作为独立一次 run（run_A2）。** 杆臂未填时 floor 必须留 1.0 m（spec §7.3 v2）；先 run_A（floor 1.0、杆臂 0）拿到 §12.2 的通过记录和 §9.3 的输入，填完杆臂再 run_A2（floor `[0.05,0.05,0.1]`）。两次都留档，差值本身就是杆臂建模有效性的实车证据。
10. **`run_injection_suite.sh` 的 `make_config` 当前会直接崩**：它对 GLIM 的 `config_ros.json` 调 `json.load`，而 GLIM 的配置是 JSON-with-comments（`//` 与 `/* */`）。Task 4 先把注释容错的改写逻辑做成带测试的 `glim_config_patch.py`，Task 6 再让脚本改用它——顺带修掉 `GLIM_CONFIG_SRC=${GLIM_WS}/src/glim/config`（本仓是 `src/glim_underground/config`）与 `src = ~/glim_ws/src/glim_ext/config/config_rtk_global.json` 两处写死路径。
11. **注入套件跑段 B 而不是段 A。** 231 s、航向累计 1731°（转弯多 → `lever` 用例可观）、bag 4.0 GB（11 次 glim run 的时间与磁盘都省一半以上）。合成 bag 每个用例跑完即删（`--keep-bags` 可保留）。
12. **轮 1.5 的门只在 §12.2 与 §12.4 都通过后才开**，且开门动作限于：在 `tage_at128/config_ros.json` 的 `extension_modules` 里加 `librtk_odometry.so`、跑一遍、确认日志里 `rtk_odometry stats: … factors_added>0`。不改 `rtk_odometry` 的任何代码，不改它的默认 `ENABLE_RTK_ODOMETRY=ON`。
13. **`enable_rtkrcv` 的 launch 默认保持 `"false"`**（与轮 2/3b 一致）。本计划不碰 `gnss_bringup` 的 launch。

---

### Task 1: `tage_at128` 配置集、工作区重建与 LiDAR-only 基线（§12.4 的 C 组）

**Files:**
- Create: `src/glim_underground/config/tage_at128/config.json`
- Create: `src/glim_underground/config/tage_at128/config_ros.json`
- Create: `src/glim_underground/config/tage_at128/config_sensors.json`
- Create: `src/glim_underground/config/tage_at128/config_preprocess.json`
- Create: `src/glim_underground/config/tage_at128/config_rtk_global.json`
- Create: `src/glim_underground/config/tage_at128/config_rtk_odometry.json`
- Create: `src/glim_underground/config/tage_at128/config_gnss_global.json`
- Copy unchanged from `mid360/`: `config_odometry_cpu.json`、`config_odometry_gpu.json`、`config_odometry_ct.json`、`config_sub_mapping_cpu.json`、`config_sub_mapping_gpu.json`、`config_sub_mapping_passthrough.json`、`config_global_mapping_cpu.json`、`config_global_mapping_gpu.json`、`config_global_mapping_pose_graph.json`、`config_viewer.json`
- Create: `src/glim_underground/docs/gnss/field/2026-10-07-round5-validation-closure.md`（本 Task 只写 §1 环境与 §2 基线两节）

**Interfaces:**
- Consumes: 无（本 Task 是起点）
- Produces:
  - 配置集目录名 `tage_at128`（后续所有 `ros2 run` 用 `-p config_path:=/home/steve/glim_ws/src/glim_underground/config/tage_at128`，**绝对路径**；相对路径会被解析到 `glim` 包的 share 目录）
  - `<segA>/validation_20261007/run_C/traj_imu.txt`、`traj_lidar.txt`（TUM，`t x y z qx qy qz qw`，IMU 轨迹即 `cgi610_imu_flu` 在 GLIM world 系下的位姿）
  - `<segA>/validation_20261007/eval/C.md`（C 组 RMSE）

- [ ] **Step 1: 建工作区、构建**

```bash
cd /home/steve/glim_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to glim_ros glim_ext gnss_bringup 2>&1 | tail -30
```

若出现 CUDA/nvcc 相关编译错误，改用：

```bash
cd /home/steve/glim_ws && source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to glim_ros glim_ext gnss_bringup \
  --cmake-args -DBUILD_WITH_CUDA=OFF 2>&1 | tail -30
```

两种情况都把实际用的那条命令和结果（成功/失败包数）记进结果文档 §1。

- [ ] **Step 2: 确认构建产物齐全**

```bash
cd /home/steve/glim_ws && source install/setup.bash
ls install/glim_ros/lib/glim_ros/glim_rosbag
ls install/glim_ext/lib/librtk_global.so install/glim_ext/lib/libgnss_global.so install/glim_ext/lib/librtk_odometry.so
ls install/gnss_core/lib/gnss_core/{calibrate_sigma_scale,estimate_lever_arm,synth_rtk_fix,gnss_report}
ls install/gnss_core/lib/gnss_core/{pos_to_rtkfix_bag.py,run_injection_suite.sh,traj_rmse.py}
```

Expected: 十一个路径全部存在。缺 `librtk_global.so` 说明 `ENABLE_RTK_GLOBAL` 被关掉了（默认 ON），重新构建 `glim_ext` 并带 `--cmake-args -DENABLE_RTK_GLOBAL=ON -DENABLE_RTK_ODOMETRY=ON -DENABLE_GNSS=ON`。

- [ ] **Step 3: 跑既有测试，确认基线全绿**

```bash
cd /home/steve/glim_ws && source /opt/ros/humble/setup.bash
colcon test --packages-select gnss_core gnss_bringup --event-handlers console_direct+ 2>&1 | tail -20
colcon test-result --verbose 2>&1 | tail -10
```

Expected: 0 failures。（PR #7 + #8 合并后的基线是 607 tests / 0 failures；数字可能因环境略有出入，失败数必须是 0。）**若出现 `ModuleNotFoundError: No module named 'ament_cmake_test'`**，是本机 pyenv shim 的问题：用 `which python3` 确认，必要时 `PATH=/usr/bin:$PATH` 重跑。

- [ ] **Step 4: 建配置集骨架**

```bash
cd /home/steve/glim_ws/src/glim_underground/config
mkdir -p tage_at128
for f in config_odometry_cpu.json config_odometry_gpu.json config_odometry_ct.json \
         config_sub_mapping_cpu.json config_sub_mapping_gpu.json config_sub_mapping_passthrough.json \
         config_global_mapping_cpu.json config_global_mapping_gpu.json config_global_mapping_pose_graph.json \
         config_viewer.json; do cp mid360/$f tage_at128/$f; done
ls tage_at128
```

- [ ] **Step 5: 写 `tage_at128/config.json`**

```json
{
  /*** Global Configuration ***/
  // 踏歌宽体车(红沙泉 2026-09-15):AT128P 主雷达 + CGI-610 IMU,喂 bag_lio。
  // CPU 变体是本配置集的默认(轮 5 计划设计决定 2):A/B/C 三组只在全局约束上不同,
  // CPU 路径让结论可在无 CUDA 的机器上复现。要换 GPU 就把下面三行的 _cpu 改成 _gpu。
  "global": {
    "config_path": "",
    "config_ros": "config_ros.json",
    "config_viewer": "config_viewer.json",
    "config_sensors": "config_sensors.json",
    "config_preprocess": "config_preprocess.json",
    "config_odometry": "config_odometry_cpu.json",
    "config_sub_mapping": "config_sub_mapping_cpu.json",
    "config_global_mapping": "config_global_mapping_cpu.json",
    // glim_ext 扩展模块的配置:这里的键优先于 <config_ext>/config_ext.json
    // (GlobalConfigExt::get_config_path(),glim_ext/include/glim_ext/util/config_ext.hpp:36-46)。
    // 本配置集不提供 config_ext.json,所以三个模块的配置都必须列在这里。
    "config_rtk_global": "config_rtk_global.json",
    "config_rtk_odometry": "config_rtk_odometry.json",
    "config_gnss_global": "config_gnss_global.json"
  }
}
```

- [ ] **Step 6: 写 `tage_at128/config_ros.json`**

```json
{
  /*** ROS-related config ***/
  // 话题与外参按 Datasets/tage/hongshaquan/20260915/README_LIO.md §3。
  // extension_modules 故意留空:轮 5 的每一次 run 由 glim_config_patch.py 按用例注入
  // (C 组空、A 组 librtk_global.so、B 组 libgnss_global.so),避免"配置里写了什么"
  // 和"这次跑的是什么"对不上。手动跑 viewer 时自己加 libstandard_viewer.so。
  // playback_speed=0.0:glim_rosbag 不做回放节流(src/glim_rosbag.cpp:346 的
  // `playback_speed > 0.0` 守卫),离线批跑尽可能快;里程计落后时仍有 workload 节流。
  "glim_ros": {
    "enable_local_mapping": true,
    "enable_global_mapping": true,
    "keep_raw_points": false,
    "imu_time_offset": 0.0,
    "points_time_offset": 0.0,
    // bag_lio 的 IMU 是 m/s²(README_LIO.md §3「IMU 单位」),显式写 1.0,
    // 不依赖 glim_ros.cpp:240 的自动判断。
    "acc_scale": 1.0,
    "playback_speed": 0.0,
    "imu_frame_id": "cgi610_imu_flu",
    "lidar_frame_id": "at128_flu",
    "base_frame_id": "",
    "odom_frame_id": "odom",
    "map_frame_id": "map",
    "publish_imu2lidar": true,
    "tf_time_offset": 1e-6,
    "extension_modules": [],
    "imu_topic": "/cgi610/imu",
    "points_topic": "/at128/points",
    "image_topic": "/image"
  }
}
```

- [ ] **Step 7: 写 `tage_at128/config_sensors.json`**

```json
{
  /*** Sensor configurations ***/
  // T_lidar_imu 把 imu 系的点变到 lidar 系:p_lidar = T_lidar_imu * p_imu,TUM 格式
  // [x,y,z,qx,qy,qz,qw](见 mid360/config_sensors.json 的注释块)。
  // README_LIO.md §3:所有 *_flu 系轴向与 base_link 相同 → 旋转是单位阵;
  // 平移 = imu 原点 − lidar 原点 = cgi610_imu_flu(6.85, 1.00, 2.28)
  //                               − at128_flu(8.736806, -0.006244, 1.628447)
  //                             = (-1.886806, 1.006244, 0.651553)
  // 与 README_LIO.md §3 给 FAST-LIO 的 extrinsic_T(雷达原点在 IMU 系下)
  // [1.886806, -1.006244, -0.651553] 恰好反号,互为校验。
  // 注意 cgi610_imu_flu 的原点是网页杆臂推算值、未验证(extrinsics_tage_truck.yaml),
  // README_LIO.md §6 建议打开外参在线估计;轮 5 的 §9.3 会独立测一次 IMU→天线杆臂。
  "sensors": {
    "imu_acc_noise": 0.05,
    "imu_gyro_noise": 0.02,
    "imu_int_noise": 0.001,
    // 车载组合导航级 IMU,偏置随机游走比手持设备小一个量级(与 casbot 一致)
    "imu_bias_noise": 1e-5,
    "global_shutter_lidar": false,
    "T_lidar_imu": [
      -1.886806,
      1.006244,
      0.651553,
      0.0,
      0.0,
      0.0,
      1.0
    ],
    // bag_lio 的点云是 velodyne 风格:intensity float32 + ring uint16 + time float32(相对 header.stamp 的秒)
    "intensity_field": "intensity",
    "ring_field": "ring",
    "autoconf_perpoint_times": true,
    "autoconf_prefer_frame_time": false,
    "perpoint_relative_time": true,
    "perpoint_time_scale": 1.0,
    // 本次没录相机(README_LIO.md §6),以下保留 mid360 的占位值,不参与计算
    "global_shutter_camera": true,
    "image_size": [
      752,
      480
    ],
    "T_lidar_camera": [
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      1.0
    ],
    "intrinsics": [
      815.498895336274,
      817.4904316140473,
      830.3149255906893,
      639.4531613962971
    ],
    "distortion_model": "plumb_bob",
    "distortion_coeffs": [
      0.0,
      0.0,
      0.0,
      0.0,
      0.0
    ]
  }
}
```

- [ ] **Step 8: 写 `tage_at128/config_preprocess.json`**

```json
{
  /*** Preprocessing parameters ***/
  // AT128P:120° 前向视场,10 Hz,每帧约 7.5 万点,帧内时间 0–0.052 s。
  // bag_lio 已剔除 NaN 与 0.5 m 内的点(README_LIO.md §3),near_thresh 保持 0.5。
  // 场地是露天停车场、绕圈行驶,far_thresh 给 100 m。
  "preprocess": {
    "distance_near_thresh": 0.5,
    "distance_far_thresh": 100.0,
    "use_random_grid_downsampling": true,
    "downsample_resolution": 0.5,
    "random_downsample_target": 15000,
    "random_downsample_rate": 0.1,
    "enable_outlier_removal": false,
    "outlier_removal_k": 10,
    "outlier_std_mul_factor": 1.0,
    "k_correspondences": 10,
    "num_threads": 4
  }
}
```

- [ ] **Step 9: 写三个扩展模块配置**

`tage_at128/config_rtk_global.json` —— 从 `glim_ext/config/config_rtk_global.json` 整份拷来（**连注释一起**，那段注释是这些键唯一的文档），只改三处值：

```
  "rtk_fix_topic": "/rtkrcv_node/rtk_fix",   // 现场联调 run_B 的 bag 里就是这个话题名
  "enu_origin": [],                          // 留空:让 FrameAligner 自己估 T_world_enu(轮 5 Task 4 验的就是它)
  "T_imu_gnss": [0.0, 0.0, 0.0],             // Task 5 用 estimate_lever_arm 的实测填回
```

其余键保持默认（`stamp_source` `gnss_time`、`time_offset` 0.0、`min_quality` 3、`max_diff_age` 15.0、`min_sats` 6、`fix_max_gap` 2.5、`quality_sigma_scale` `[0.0, 50.0, 20.0, 5.0, 1.0]`、`sigma_floor` `[1.0, 1.0, 1.0]`、`vertical_scale` 3.0、`robust_kernel` `huber`、`robust_delta` 1.345、`min_baseline` 50.0、`fix_buffer_horizon` 600.0、`max_stamp_skew` 5.0）。在文件顶部加一行本车注释：

```
  // 踏歌宽体车。sigma_floor 保持 1.0 m 直到 T_imu_gnss 标定完成(spec §7.3 v2);
  // 实测值与降档后的结果见 docs/gnss/field/2026-10-07-round5-validation-closure.md。
```

`tage_at128/config_rtk_odometry.json` —— 同样从 `glim_ext/config/config_rtk_odometry.json` 整份拷来，只把 `rtk_fix_topic` 改成 `/rtkrcv_node/rtk_fix`。

`tage_at128/config_gnss_global.json`：

```json
{
  /*** gnss_global(上游朴素 GNSS 约束模块,§12.4 的 B 组对照)***/
  // 这个模块直接把 pose.pose.position 当成"UTM 米"用,忽略协方差、忽略 IMU-GNSS 杆臂
  // (modules/mapping/gnss_global/include/glim_ext/gnss_global_module.hpp:89-95 与该文件
  //  头部的 @note)。轮 5 Task 7 喂给它的是 pos_to_enu 产出的局部 ENU 米,原点取
  //  <seg>/gt/enu_origin.txt,与 gt/*.tum 同原点。
  // prior_inf_scale 的 z 为 0 = 不约束高程,保留上游默认值,不为了对比而调它。
  "gnss": {
    "gnss_topic": "/gnss/pose",
    "min_baseline": 10.0,
    "prior_inf_scale": [
      1e3,
      1e3,
      0.0
    ]
  }
}
```

- [ ] **Step 10: 提交配置集**

```bash
cd /home/steve/glim_ws/src/glim_underground
git checkout -b feat/round5-validation-closure
git add config/tage_at128
git commit -F - <<'EOF'
config: add tage_at128 config set (AT128P + CGI-610, bag_lio)

轮 5 验证闭环需要一套本车的 GLIM 配置集:现有的 mid360 / casbot 话题与外参都
不对(casbot 是 /rslidar_points 且 T_lidar_imu 带 90° 旋转),改它们会毁掉已有配置。

- T_lidar_imu [-1.886806, 1.006244, 0.651553, 0,0,0,1]
  = cgi610_imu_flu(6.85,1.00,2.28) − at128_flu(8.736806,-0.006244,1.628447),
  旋转为单位阵(README_LIO.md §3: 所有 *_flu 系轴向与 base_link 相同);
  与该文档给 FAST-LIO 的 extrinsic_T 恰好反号,互为校验。
- 话题 /at128/points + /cgi610/imu,acc_scale 1.0(bag 里是 m/s²),
  ring_field "ring"、perpoint_relative_time(velodyne 风格 time 字段)。
- extension_modules 留空,每次 run 由 glim_config_patch.py 按用例注入。
- config.json 里显式列出 config_rtk_global / config_rtk_odometry /
  config_gnss_global:本配置集不提供 config_ext.json,GlobalConfigExt
  只能从主 config.json 的 "global" 块找到它们(config_ext.hpp:36-46)。
- CPU 变体为默认(_gpu 文件一并拷入,换一行即可切)。

验证:无新代码,下一步 Task 1 Step 11 用这套配置跑 LiDAR-only 基线,
日志里 T_lidar_imu / 话题 / 帧数正确即为通过。

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>
EOF
```

- [ ] **Step 11: 跑 LiDAR-only 基线（C 组）**

```bash
SEGA=/home/steve/Documents/Datasets/tage/hongshaquan/20260915/seg_164931_165748
OUT=$SEGA/validation_20261007
mkdir -p $OUT/run_C $OUT/eval
cd /home/steve/glim_ws && source install/setup.bash
ros2 run glim_ros glim_rosbag $SEGA/bag_lio \
  --ros-args \
  -p config_path:=/home/steve/glim_ws/src/glim_underground/config/tage_at128 \
  -p auto_quit:=true \
  -p dump_path:=$OUT/run_C \
  2>&1 | tee $OUT/run_C/glim.log
```

Expected（在 `glim.log` 里逐条确认）：
- `config_path: /home/steve/glim_ws/src/glim_underground/config/tage_at128`
- `topics:` 列出 `/cgi610/imu`、`/at128/points`
- `detected storage_id=sqlite3 from metadata.yaml`
- 结尾出现 `traj_imu.txt` 对应的 dump（`glim->save()` 在 `global_mapping.cpp:601-627` 写 `traj_lidar.txt` 与 `traj_imu.txt`）
- `wc -l $OUT/run_C/traj_imu.txt` ≥ 4000（AT128 帧数 4966；submap 抽稀后更少也可接受，但不能是 0）

**跑不动的两种常见原因**：① 点云字段名不对 → 日志里会有 intensity/ring 相关的警告，核对 `config_sensors.json` 的 `intensity_field`/`ring_field`；② IMU 落后 → 日志反复 `throttling (waiting for odometry estimation)`，把 `config_preprocess.json` 的 `random_downsample_target` 降到 8000 重跑，并在结果文档记下改动。

- [ ] **Step 12: C 组 RMSE（本 Task 的验收断言）**

```bash
SEGA=/home/steve/Documents/Datasets/tage/hongshaquan/20260915/seg_164931_165748
OUT=$SEGA/validation_20261007
cd /home/steve/glim_ws && source install/setup.bash
python3 install/gnss_core/lib/gnss_core/traj_rmse.py \
  $SEGA/gt/gt_cgi610_imu_flu.tum $OUT/run_C/traj_imu.txt --align --tol 0.05 \
  | tee $OUT/eval/C.md
```

Expected: 输出里 `n_pairs` > 1000，`rmse` < 2.0 m。参照物：同段 FAST-LIO 的结果在 `<segA>/fastlio_eval/`，可一并跑一次 `traj_rmse.py` 作横向比较并记进结果文档。

**断言不过怎么办**：rmse > 2 m 说明配置有问题（最可能是 `T_lidar_imu` 符号反了或 `acc_scale` 不对），**必须先修到过**——C 组是后面所有对比的分母。把排查过程记进结果文档 §2。

- [ ] **Step 13: 起结果文档并提交**

新建 `src/glim_underground/docs/gnss/field/2026-10-07-round5-validation-closure.md`，本 Task 只写两节：

- `## 1. 环境与构建`：实际用的 colcon 命令、是否开 CUDA、`colcon test` 结果、工具可执行路径。
- `## 2. LiDAR-only 基线(C 组)`：命令、`traj_imu.txt` 行数、`traj_rmse.py` 输出全文、与 FAST-LIO 的横向对比、以及为了跑通做过的任何配置改动。

```bash
cd /home/steve/glim_ws/src/glim_underground
git add docs/gnss/field/2026-10-07-round5-validation-closure.md
git commit -F - <<'EOF'
docs(gnss): round 5 results — environment and LiDAR-only baseline

C 组(无 GNSS)是 §12.4 A/B 的分母,也是 §12.3 注入套件的参考轨迹。
记录实际构建命令、既有测试基线,以及 traj_imu.txt 对 gt_cgi610_imu_flu.tum
的对齐 RMSE。

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>
EOF
```

---

### Task 2: `gnss_core` 的 ENU 轨迹导出（`enu_track` + `pos_to_enu`）

**Files:**
- Create: `src/glim_ext/gnss_core/include/gnss_core/enu_track.hpp`
- Create: `src/glim_ext/gnss_core/src/enu_track.cpp`
- Create: `src/glim_ext/gnss_core/test/test_enu_track.cpp`
- Create: `src/glim_ext/gnss_core/tools/pos_to_enu.cpp`
- Modify: `src/glim_ext/gnss_core/CMakeLists.txt`（加库源文件、加 `gnss_core_add_test(test_enu_track)`、加 `pos_to_enu` 可执行与 install）

**Interfaces:**
- Consumes: `gnss_core::PosRecord`、`gnss_core::read_pos`、`gnss_core::PosReadOptions`、`gnss_core::PosTimeSystem`（`pos_io.hpp`）；`gnss_core::LlaToEnu`（`geodetic.hpp`）；`gnss_core::Quality`、`gnss_core::q_to_quality`（`types.hpp`、`pos_io.hpp:195`）
- Produces:
  ```cpp
  namespace gnss_core {
  struct EnuSample {
    double stamp = 0.0;                                 // UTC unix 秒
    Eigen::Vector3d enu = Eigen::Vector3d::Zero();      // [E, N, U] m
    Eigen::Vector3d sigma_enu = Eigen::Vector3d::Zero();// [sde, sdn, sdu] m
    Quality quality = Quality::NONE;
  };
  struct EnuTrack {
    Eigen::Vector3d origin_lla = Eigen::Vector3d::Zero();  // [lat, lon, alt] deg/deg/m
    std::vector<EnuSample> samples;
  };
  struct EnuTrackOptions {
    Quality min_quality = Quality::FLOAT;
    bool has_origin = false;
    Eigen::Vector3d origin_lla = Eigen::Vector3d::Zero();
  };
  EnuTrack make_enu_track(const std::vector<PosRecord>& recs, const EnuTrackOptions& opt);
  void write_enu_track(std::ostream& out, const EnuTrack& track);
  EnuTrack read_enu_track(std::istream& in);   // 供测试与 Python 侧格式对齐用
  }  // namespace gnss_core
  ```
  `.enu` 文本格式（Task 3 的 Python 侧按这个解析）：
  ```
  # origin_lla 44.47025713 90.29451498 610.640
  # t e n u sde sdn sdu q
  1789462172.000 7.412 -3.118 0.353 0.764 0.857 1.929 3
  ```
  `q` 列是**归一化质量枚举**（0 NONE / 1 SINGLE / 2 DGPS / 3 FLOAT / 4 FIXED），不是 RTKLIB 的 Q。

- [ ] **Step 1: 写失败的测试**

`src/glim_ext/gnss_core/test/test_enu_track.cpp`：

```cpp
#include <gtest/gtest.h>
#include <sstream>
#include <vector>
#include "gnss_core/enu_track.hpp"
#include "gnss_core/geodetic.hpp"

using gnss_core::EnuTrack;
using gnss_core::EnuTrackOptions;
using gnss_core::make_enu_track;
using gnss_core::PosRecord;
using gnss_core::Quality;

namespace {

// 红沙泉段 A 的 ENU 原点(gt/enu_origin.txt)
constexpr double kLat0 = 44.470257133;
constexpr double kLon0 = 90.294514984;
constexpr double kAlt0 = 610.6395;

PosRecord rec(double stamp, double lat, double lon, double h, int q) {
  PosRecord r;
  r.stamp = stamp;
  r.lat = lat;
  r.lon = lon;
  r.height = h;
  r.q = q;
  r.ns = 25;
  r.sdne = Eigen::Vector3d(0.11, 0.07, 0.19);  // sdn, sde, sdu —— 注意 .pos 的列序
  return r;
}

}  // namespace

TEST(EnuTrack, ExplicitOriginMapsToZero) {
  const std::vector<PosRecord> recs = {rec(100.0, kLat0, kLon0, kAlt0, 1)};
  EnuTrackOptions opt;
  opt.has_origin = true;
  opt.origin_lla = Eigen::Vector3d(kLat0, kLon0, kAlt0);
  const EnuTrack t = make_enu_track(recs, opt);
  ASSERT_EQ(t.samples.size(), 1u);
  EXPECT_LT(t.samples[0].enu.norm(), 1e-6);
  EXPECT_EQ(t.samples[0].quality, Quality::FIXED);
}

TEST(EnuTrack, DefaultOriginIsFirstQualifyingRecord) {
  // 第一条是 SINGLE(Q=5),被 min_quality=FLOAT 挡掉 → 原点应取第二条
  const std::vector<PosRecord> recs = {rec(100.0, kLat0 + 1e-3, kLon0, kAlt0, 5),
                                       rec(101.0, kLat0, kLon0, kAlt0, 1)};
  const EnuTrack t = make_enu_track(recs, EnuTrackOptions{});
  ASSERT_EQ(t.samples.size(), 1u);
  EXPECT_NEAR(t.origin_lla.x(), kLat0, 1e-12);
  EXPECT_LT(t.samples[0].enu.norm(), 1e-6);
}

TEST(EnuTrack, QualityGateDropsRecordsBelowThreshold) {
  const std::vector<PosRecord> recs = {rec(100.0, kLat0, kLon0, kAlt0, 1),   // FIXED
                                       rec(101.0, kLat0, kLon0, kAlt0, 2),   // FLOAT
                                       rec(102.0, kLat0, kLon0, kAlt0, 4),   // DGPS
                                       rec(103.0, kLat0, kLon0, kAlt0, 0)};  // NONE
  EnuTrackOptions opt;
  opt.has_origin = true;
  opt.origin_lla = Eigen::Vector3d(kLat0, kLon0, kAlt0);
  opt.min_quality = Quality::FLOAT;
  const EnuTrack t = make_enu_track(recs, opt);
  ASSERT_EQ(t.samples.size(), 2u);
  EXPECT_EQ(t.samples[0].quality, Quality::FIXED);
  EXPECT_EQ(t.samples[1].quality, Quality::FLOAT);
}

TEST(EnuTrack, SigmaIsReorderedToEnu) {
  // PosRecord::sdne 的列序是 (sdn, sde, sdu);EnuSample::sigma_enu 必须是 (sde, sdn, sdu)
  const std::vector<PosRecord> recs = {rec(100.0, kLat0, kLon0, kAlt0, 1)};
  EnuTrackOptions opt;
  opt.has_origin = true;
  opt.origin_lla = Eigen::Vector3d(kLat0, kLon0, kAlt0);
  const EnuTrack t = make_enu_track(recs, opt);
  ASSERT_EQ(t.samples.size(), 1u);
  EXPECT_NEAR(t.samples[0].sigma_enu.x(), 0.07, 1e-12);  // sde
  EXPECT_NEAR(t.samples[0].sigma_enu.y(), 0.11, 1e-12);  // sdn
  EXPECT_NEAR(t.samples[0].sigma_enu.z(), 0.19, 1e-12);  // sdu
}

TEST(EnuTrack, MatchesLlaToEnuDirectly) {
  // 与已测过的 LlaToEnu 交叉验证:同一个点,两条路径必须毫米内一致
  const double lat = kLat0 + 2e-4, lon = kLon0 - 3e-4, h = kAlt0 + 1.5;
  const std::vector<PosRecord> recs = {rec(100.0, lat, lon, h, 1)};
  EnuTrackOptions opt;
  opt.has_origin = true;
  opt.origin_lla = Eigen::Vector3d(kLat0, kLon0, kAlt0);
  const EnuTrack t = make_enu_track(recs, opt);
  const gnss_core::LlaToEnu conv(kLat0, kLon0, kAlt0);
  const Eigen::Vector3d expected = conv.forward(lat, lon, h);
  ASSERT_EQ(t.samples.size(), 1u);
  EXPECT_LT((t.samples[0].enu - expected).norm(), 1e-3);
}

TEST(EnuTrack, EmptyInputYieldsEmptyTrack) {
  const EnuTrack t = make_enu_track({}, EnuTrackOptions{});
  EXPECT_TRUE(t.samples.empty());
}

TEST(EnuTrack, NoQualifyingRecordWithoutExplicitOriginYieldsEmptyTrack) {
  const std::vector<PosRecord> recs = {rec(100.0, kLat0, kLon0, kAlt0, 5)};  // SINGLE < FLOAT
  const EnuTrack t = make_enu_track(recs, EnuTrackOptions{});
  EXPECT_TRUE(t.samples.empty());
}

TEST(EnuTrack, WriteReadRoundTrip) {
  const std::vector<PosRecord> recs = {rec(1789462172.0, kLat0 + 1e-4, kLon0, kAlt0, 1),
                                       rec(1789462173.0, kLat0, kLon0 + 1e-4, kAlt0 + 0.5, 2)};
  EnuTrackOptions opt;
  opt.has_origin = true;
  opt.origin_lla = Eigen::Vector3d(kLat0, kLon0, kAlt0);
  const EnuTrack t = make_enu_track(recs, opt);

  std::ostringstream os;
  gnss_core::write_enu_track(os, t);
  std::istringstream is(os.str());
  const EnuTrack back = gnss_core::read_enu_track(is);

  ASSERT_EQ(back.samples.size(), t.samples.size());
  EXPECT_NEAR(back.origin_lla.x(), t.origin_lla.x(), 1e-7);
  EXPECT_NEAR(back.origin_lla.y(), t.origin_lla.y(), 1e-7);
  for (size_t i = 0; i < t.samples.size(); ++i) {
    EXPECT_NEAR(back.samples[i].stamp, t.samples[i].stamp, 1e-3);
    EXPECT_LT((back.samples[i].enu - t.samples[i].enu).norm(), 1e-3);
    EXPECT_LT((back.samples[i].sigma_enu - t.samples[i].sigma_enu).norm(), 1e-3);
    EXPECT_EQ(back.samples[i].quality, t.samples[i].quality);
  }
}

TEST(EnuTrack, ReadSkipsCommentsAndMalformedLines) {
  const std::string text =
    "# origin_lla 44.47025713 90.29451498 610.640\n"
    "# t e n u sde sdn sdu q\n"
    "1789462172.000 1.0 2.0 3.0 0.1 0.2 0.3 4\n"
    "garbage line\n"
    "1789462173.000 4.0 5.0\n"            // 列数不足
    "1789462174.000 7.0 8.0 9.0 0.1 0.2 0.3 3\n";
  std::istringstream is(text);
  const EnuTrack t = gnss_core::read_enu_track(is);
  ASSERT_EQ(t.samples.size(), 2u);
  EXPECT_NEAR(t.samples[0].enu.x(), 1.0, 1e-9);
  EXPECT_EQ(t.samples[1].quality, Quality::FLOAT);
}
```

- [ ] **Step 2: 加桩让它编译并失败**

`src/glim_ext/gnss_core/include/gnss_core/enu_track.hpp` 先只写声明（如 Interfaces 节所示），`src/enu_track.cpp` 写空实现：

```cpp
#include "gnss_core/enu_track.hpp"

namespace gnss_core {

EnuTrack make_enu_track(const std::vector<PosRecord>&, const EnuTrackOptions&) { return {}; }
void write_enu_track(std::ostream&, const EnuTrack&) {}
EnuTrack read_enu_track(std::istream&) { return {}; }

}  // namespace gnss_core
```

`CMakeLists.txt` 的 `add_library(gnss_core SHARED ...)` 列表里加 `src/enu_track.cpp`，`if(BUILD_TESTING)` 块里加 `gnss_core_add_test(test_enu_track)`。

- [ ] **Step 3: 运行测试确认它失败**

```bash
cd /home/steve/glim_ws && source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select gnss_core
colcon test --packages-select gnss_core --ctest-args -R test_enu_track --event-handlers console_direct+ 2>&1 | tail -30
```

Expected: FAIL —— `ExplicitOriginMapsToZero` 等用例全部红（`samples.size()` 为 0）。**把这段输出抄进提交 message。**

- [ ] **Step 4: 写实现**

`src/glim_ext/gnss_core/src/enu_track.cpp`：

```cpp
#include "gnss_core/enu_track.hpp"

#include <cstdio>
#include <iomanip>
#include <sstream>
#include <string>

#include "gnss_core/geodetic.hpp"
#include "gnss_core/pos_io.hpp"

namespace gnss_core {

EnuTrack make_enu_track(const std::vector<PosRecord>& recs, const EnuTrackOptions& opt) {
  EnuTrack track;

  // 原点:显式给定优先;否则取第一条达到 min_quality 的记录。两者都没有 → 空轨迹。
  if (opt.has_origin) {
    track.origin_lla = opt.origin_lla;
  } else {
    bool found = false;
    for (const auto& r : recs) {
      if (q_to_quality(r.q) >= opt.min_quality) {
        track.origin_lla = Eigen::Vector3d(r.lat, r.lon, r.height);
        found = true;
        break;
      }
    }
    if (!found) {
      return track;
    }
  }

  const LlaToEnu conv(track.origin_lla.x(), track.origin_lla.y(), track.origin_lla.z());
  track.samples.reserve(recs.size());
  for (const auto& r : recs) {
    const Quality q = q_to_quality(r.q);
    if (q < opt.min_quality) {
      continue;
    }
    EnuSample s;
    s.stamp = r.stamp;
    s.enu = conv.forward(r.lat, r.lon, r.height);
    // PosRecord::sdne 的列序是 (sdn, sde, sdu);输出按 ENU 换序
    s.sigma_enu = Eigen::Vector3d(r.sdne.y(), r.sdne.x(), r.sdne.z());
    s.quality = q;
    track.samples.push_back(s);
  }
  return track;
}

void write_enu_track(std::ostream& out, const EnuTrack& track) {
  char buf[256];
  std::snprintf(buf, sizeof(buf), "# origin_lla %.8f %.8f %.3f\n", track.origin_lla.x(),
                track.origin_lla.y(), track.origin_lla.z());
  out << buf;
  out << "# t e n u sde sdn sdu q\n";
  for (const auto& s : track.samples) {
    std::snprintf(buf, sizeof(buf), "%.3f %.3f %.3f %.3f %.4f %.4f %.4f %d\n", s.stamp, s.enu.x(),
                  s.enu.y(), s.enu.z(), s.sigma_enu.x(), s.sigma_enu.y(), s.sigma_enu.z(),
                  static_cast<int>(s.quality));
    out << buf;
  }
}

EnuTrack read_enu_track(std::istream& in) {
  EnuTrack track;
  std::string line;
  while (std::getline(in, line)) {
    if (!line.empty() && line.back() == '\r') {
      line.pop_back();   // CRLF 容错(与 report_inputs 一致)
    }
    if (line.empty()) {
      continue;
    }
    if (line[0] == '#') {
      std::istringstream hs(line.substr(1));
      std::string key;
      double a = 0.0, b = 0.0, c = 0.0;
      if ((hs >> key >> a >> b >> c) && key == "origin_lla") {
        track.origin_lla = Eigen::Vector3d(a, b, c);
      }
      continue;
    }
    std::istringstream ls(line);
    double t = 0.0, e = 0.0, n = 0.0, u = 0.0, sde = 0.0, sdn = 0.0, sdu = 0.0;
    int q = 0;
    if (!(ls >> t >> e >> n >> u >> sde >> sdn >> sdu >> q)) {
      continue;   // 列数不足或非数值:跳过,不中断
    }
    if (q < 0 || q > static_cast<int>(Quality::FIXED)) {
      continue;
    }
    EnuSample s;
    s.stamp = t;
    s.enu = Eigen::Vector3d(e, n, u);
    s.sigma_enu = Eigen::Vector3d(sde, sdn, sdu);
    s.quality = static_cast<Quality>(q);
    track.samples.push_back(s);
  }
  return track;
}

}  // namespace gnss_core
```

- [ ] **Step 5: 运行测试确认通过**

```bash
cd /home/steve/glim_ws && source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select gnss_core
colcon test --packages-select gnss_core --ctest-args -R test_enu_track --event-handlers console_direct+ 2>&1 | tail -20
```

Expected: PASS，9 个用例全绿。

- [ ] **Step 6: 做一次变异确认测试有牙**

把 `s.sigma_enu` 那行的 `r.sdne.y(), r.sdne.x()` 换回 `r.sdne.x(), r.sdne.y()`，重跑 → `SigmaIsReorderedToEnu` 必须红。改回来。**变异输出抄进提交 message。**

- [ ] **Step 7: 写 CLI 外壳 `pos_to_enu`**

`src/glim_ext/gnss_core/tools/pos_to_enu.cpp`：

```cpp
// pos_to_enu —— RTKLIB .pos → 局部 ENU 轨迹文本(轮 5 Task 2;spec §12.4 的 gnss_global 输入)
//
// usage: pos_to_enu <in.pos> <out.enu> [--origin lat lon alt] [--min-quality 3]
//                   [--pos-time GPST|UTC] [--leap 18]
//
// - --origin 缺省取 .pos 首个达标历元;§12.4 要求与 gt/*.tum 同原点,所以实际使用时
//   务必显式传 <seg>/gt/enu_origin.txt 里的三个数。
// - --min-quality 用归一化枚举:4=FIXED 3=FLOAT 2=DGPS 1=SINGLE 0=不过滤。
// - --pos-time 只在 .pos 没有 "% (... time=GPST|UTC)" 头时生效(RTKLIB 约定默认 GPST)。
// - 输出格式见 gnss_core/enu_track.hpp;时间列是 UTC unix 秒。
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <string>

#include "gnss_core/enu_track.hpp"
#include "gnss_core/pos_io.hpp"

namespace {

void usage() {
  std::fprintf(stderr,
               "usage: pos_to_enu <in.pos> <out.enu> [--origin lat lon alt] [--min-quality 3]\n"
               "                  [--pos-time GPST|UTC] [--leap 18]\n");
}

bool need(int argc, int i, int n) {
  if (i + n >= argc) {
    usage();
    std::exit(2);
  }
  return true;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 3) {
    usage();
    return 2;
  }
  const std::string in_path = argv[1], out_path = argv[2];

  gnss_core::EnuTrackOptions opt;
  gnss_core::PosReadOptions ropt;
  for (int i = 3; i < argc; ++i) {
    const std::string a = argv[i];
    if (a == "--origin" && need(argc, i, 3)) {
      opt.has_origin = true;
      opt.origin_lla = Eigen::Vector3d(std::atof(argv[i + 1]), std::atof(argv[i + 2]), std::atof(argv[i + 3]));
      i += 3;
    } else if (a == "--min-quality" && need(argc, i, 1)) {
      opt.min_quality = static_cast<gnss_core::Quality>(std::atoi(argv[++i]));
    } else if (a == "--pos-time" && need(argc, i, 1)) {
      const std::string v = argv[++i];
      ropt.default_time_system = (v == "UTC") ? gnss_core::PosTimeSystem::UTC : gnss_core::PosTimeSystem::GPST;
    } else if (a == "--leap" && need(argc, i, 1)) {
      ropt.leap_seconds = std::atoi(argv[++i]);
    } else {
      std::fprintf(stderr, "unknown argument: %s\n", a.c_str());
      usage();
      return 2;
    }
  }

  std::vector<gnss_core::PosRecord> recs;
  try {
    recs = gnss_core::read_pos(in_path, ropt);
  } catch (const std::exception& e) {
    std::fprintf(stderr, "error: %s\n", e.what());
    return 1;
  }
  if (recs.empty()) {
    std::fprintf(stderr, "error: no epochs in %s (check the time column / header time system)\n", in_path.c_str());
    return 1;
  }

  const gnss_core::EnuTrack track = gnss_core::make_enu_track(recs, opt);
  if (track.samples.empty()) {
    std::fprintf(stderr, "error: no epoch reaches min_quality=%d (read %zu epochs)\n",
                 static_cast<int>(opt.min_quality), recs.size());
    return 1;
  }

  std::ofstream ofs(out_path);
  if (!ofs) {
    std::fprintf(stderr, "error: cannot write %s\n", out_path.c_str());
    return 1;
  }
  gnss_core::write_enu_track(ofs, track);
  ofs.close();
  if (!ofs) {
    std::fprintf(stderr, "error: write failed on %s\n", out_path.c_str());
    return 1;
  }

  std::printf("read %zu epochs, wrote %zu samples (min_quality=%d)\n", recs.size(),
              track.samples.size(), static_cast<int>(opt.min_quality));
  std::printf("origin: lat=%.8f lon=%.8f alt=%.3f\n", track.origin_lla.x(), track.origin_lla.y(),
              track.origin_lla.z());
  return 0;
}
```

`CMakeLists.txt` 在 `estimate_lever_arm` 那组后面加：

```cmake
add_executable(pos_to_enu tools/pos_to_enu.cpp)
target_link_libraries(pos_to_enu gnss_core)
install(TARGETS pos_to_enu DESTINATION lib/${PROJECT_NAME})
```

- [ ] **Step 8: 在真实数据上跑一遍（两段共四个文件）**

```bash
SEGA=/home/steve/Documents/Datasets/tage/hongshaquan/20260915/seg_164931_165748
SEGB=/home/steve/Documents/Datasets/tage/hongshaquan/20260915/seg_153000_153351
cd /home/steve/glim_ws && source install/setup.bash
P=install/gnss_core/lib/gnss_core/pos_to_enu
mkdir -p $SEGA/validation_20261007/gnss $SEGB/validation_20261007/gnss

# 段 A:实时解(现场联调 run_B)与后处理解
$P $SEGA/integration_20260916/run_B/pos/20260915/rtkrcv.pos \
   $SEGA/validation_20261007/gnss/rtkrcv.enu \
   --origin 44.470257133 90.294514984 610.6395 --min-quality 3
$P $SEGA/gnss/rtk_check.pos \
   $SEGA/validation_20261007/gnss/rtk_check.enu \
   --origin 44.470257133 90.294514984 610.6395 --min-quality 3
# 段 B:只有后处理解
$P $SEGB/gnss/rtk_check.pos \
   $SEGB/validation_20261007/gnss/rtk_check.enu \
   --origin 44.470267482 90.294436345 610.5679 --min-quality 3
```

Expected:
- `rtkrcv.enu`：`read 497 epochs, wrote 497 samples`
- 段 A `rtk_check.enu`：`read 498 epochs, wrote 498 samples` ——**这是 `read_pos` 对 "GPS 周 + 周内秒" 时间列支持（PR #8 `8fefcdc`）的第一次真实数据验证**；若读出 0 条，说明那个修复没有覆盖这个文件，必须在这里停下来排查，不要绕过。
- 段 B `rtk_check.enu`：`read 231 epochs, wrote 231 samples`
- 三个文件的第一条 ENU 范数都应在百米量级以内（原点就在场地里）：`head -3` 看一眼。

- [ ] **Step 9: 提交**

```bash
cd /home/steve/glim_ws/src/glim_ext
git checkout -b feat/round5-validation-closure
git add gnss_core/include/gnss_core/enu_track.hpp gnss_core/src/enu_track.cpp \
        gnss_core/test/test_enu_track.cpp gnss_core/tools/pos_to_enu.cpp gnss_core/CMakeLists.txt
git commit -F - <<'EOF'
gnss_core: add enu_track + pos_to_enu (ENU input for gnss_global)

spec §12.4 的 B 组(libgnss_global.so)只认 geometry_msgs/PoseWithCovarianceStamped,
而且直接把 pose.position 当米用(gnss_global_module.hpp:89-95)。要在同一段数据上
公平对比 A/B,必须把 .pos 换算成局部 ENU 米。

换算放在 C++ 而不是 Python:本机既没有 geographiclib 也没有 pyproj 的 Python 绑定,
而 gnss_core::LlaToEnu 是已有测试覆盖的 GeographicLib LocalCartesian 封装。
Task 3 的 pos_to_rtkfix_bag.py 只读这里产出的 .enu 文本,不做任何坐标换算。

- make_enu_track:质量门限、σ 按 (sdn,sde,sdu) → (sde,sdn,sdu) 换序、
  原点显式给定或取首个达标历元;无达标历元且无显式原点时返回空轨迹。
- write/read_enu_track:注释行 + 8 列文本,CRLF 与坏行容错(与 report_inputs 一致)。

RED:桩实现(全部返回空)下 9 个用例里 7 个红,输出:
  [  FAILED  ] EnuTrack.ExplicitOriginMapsToZero
  [  FAILED  ] EnuTrack.DefaultOriginIsFirstQualifyingRecord
  [  FAILED  ] EnuTrack.QualityGateDropsRecordsBelowThreshold
  [  FAILED  ] EnuTrack.SigmaIsReorderedToEnu
  [  FAILED  ] EnuTrack.MatchesLlaToEnuDirectly
  [  FAILED  ] EnuTrack.WriteReadRoundTrip
  [  FAILED  ] EnuTrack.ReadSkipsCommentsAndMalformedLines
变异:把 sigma_enu 的换序改回 (sdne.x, sdne.y) → SigmaIsReorderedToEnu 红(期望 0.07 实得 0.11)。

真实数据:段 A rtkrcv.pos 497/497、rtk_check.pos 498/498、段 B rtk_check.pos 231/231。
其中 rtk_check.pos 是 "GPS 周 + 周内秒" 时间列,这是 PR #8 的 8fefcdc 第一次在真实
数据上被验证。

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>
EOF
```

---

### Task 3: `pos_to_rtkfix_bag.py` —— 周/周内秒时间列与 `PoseWithCovarianceStamped`

**Files:**
- Modify: `src/glim_ext/gnss_core/tools/pos_to_rtkfix_bag.py`
- Create: `src/glim_ext/gnss_core/tests/test_pos_to_rtkfix_bag.py`
- Modify: `src/glim_ext/gnss_core/CMakeLists.txt`（把 Python 测试的 `add_test` 名字从 `test_export_bag_to_pos` 改成 `test_tools_python`，因为它 discover 整个 `tests/` 目录，名字已经名不副实）

**Interfaces:**
- Consumes: `.enu` 文本格式（Task 2 的 `write_enu_track` 定义）
- Produces（Python 函数，供测试与 Task 7 使用）：
  - `parse_pos(path, default_time="GPST", leap=18) -> list[dict]` ——**行为扩展**：时间列除 `YYYY/MM/DD HH:MM:SS.sss` 外，也接受 `WWWW SSSSSS.SSS`（GPS 周 + 周内秒）
  - `parse_enu(path) -> (origin_lla: tuple[float,float,float] | None, samples: list[dict])`，`samples` 每项 `{"t","e","n","u","sde","sdn","sdu","q"}`
  - 新 CLI 参数：`--pose-enu <file.enu>`、`--pose-topic /gnss/pose`、`--pose-frame enu`
  - 一次调用可同时写 `--topic`（`gnss_msgs/RtkFix`，来自 `.pos`）与 `--pose-topic`（`geometry_msgs/PoseWithCovarianceStamped`，来自 `.enu`），两者与 `--merge` 的源 bag 消息按接收时间归并进同一个输出 bag

- [ ] **Step 1: 写失败的测试**

`src/glim_ext/gnss_core/tests/test_pos_to_rtkfix_bag.py`：

```python
#!/usr/bin/env python3
"""pos_to_rtkfix_bag.py 的纯解析函数测试(不需要 ROS;轮 5 Task 3)。

本文件被 gnss_core/CMakeLists.txt 的 `unittest discover -s tests` 自动收集。
用 unittest 而非 pytest:见该 CMakeLists 里的注释(新版 pytest 与 ROS Humble 的
launch_testing 插件不兼容)。
"""
import os
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "tools"))

import pos_to_rtkfix_bag as mod  # noqa: E402


def write(tmpdir, name, text):
    path = os.path.join(tmpdir, name)
    with open(path, "w") as f:
        f.write(text)
    return path


class ParsePosCalendar(unittest.TestCase):
    HEADER_GPST = (
        "% program   : gnss_core write_pos\n"
        "% (lat/lon/height=WGS84/ellipsoidal,Q=1:fix,2:float,4:dgps,5:single, time=GPST)\n"
        "%  GPST                  latitude(deg) longitude(deg)  height(m)   Q  ns"
        "   sdn(m)   sde(m)   sdu(m)  sdne(m)  sdeu(m)  sdun(m) age(s)  ratio\n"
    )
    ROW = ("2026/09/15 08:49:50.000   44.470305335   90.294609181   610.9932   2  25"
           "   0.8573   0.7637   1.9293   0.0000   0.0000   0.0000   1.50    0.0\n")

    def test_gpst_header_subtracts_leap_seconds(self):
        with tempfile.TemporaryDirectory() as d:
            p = write(d, "a.pos", self.HEADER_GPST + self.ROW)
            recs = mod.parse_pos(p)
            self.assertEqual(len(recs), 1)
            # 2026/09/15 08:49:50 UTC = 1789462190;GPST 头 → 减 18 s
            self.assertAlmostEqual(recs[0]["t"], 1789462190.0 - 18.0, places=3)
            self.assertEqual(recs[0]["q"], 2)
            self.assertEqual(recs[0]["ns"], 25)
            self.assertAlmostEqual(recs[0]["sdn"], 0.8573, places=6)
            self.assertAlmostEqual(recs[0]["sde"], 0.7637, places=6)
            self.assertAlmostEqual(recs[0]["age"], 1.50, places=3)

    def test_utc_header_keeps_stamp(self):
        with tempfile.TemporaryDirectory() as d:
            header = self.HEADER_GPST.replace("time=GPST", "time=UTC")
            p = write(d, "a.pos", header + self.ROW)
            recs = mod.parse_pos(p)
            self.assertAlmostEqual(recs[0]["t"], 1789462190.0, places=3)


class ParsePosWeekTow(unittest.TestCase):
    """rnx2rtkp 默认输出的 "GPS 周 + 周内秒" 时间列(gnss/rtk_check.pos 就是这个格式)。

    gnss_core::read_pos 已支持(PR #8 8fefcdc),Python 侧此前只认日历格式,
    对 rtk_check.pos 会静默解析出 0 条。
    """

    HEADER = (
        "% program   : rnx2rtkp ver.EX 2.5.1\n"
        "% obs start : 2026/09/15 08:49:49.0 GPST\n"
        "%  GPST      latitude(deg) longitude(deg)  height(m)   Q  ns"
        "   sdn(m)   sde(m)   sdu(m)  sdne(m)  sdeu(m)  sdun(m) age(s)  ratio\n"
    )
    # GPS 周 2436、周内秒 204589.000 → GPST 2026/09/15 08:49:49 → UTC 08:49:31
    ROW = ("2436 204589.000   44.470303520   90.294603461   613.3524   2  22"
           "   0.0123   0.0098   0.0345   0.0000   0.0000   0.0000   0.90    1.2\n")

    def test_week_tow_is_parsed_as_gpst(self):
        with tempfile.TemporaryDirectory() as d:
            p = write(d, "a.pos", self.HEADER + self.ROW)
            recs = mod.parse_pos(p)
            self.assertEqual(len(recs), 1)
            # GPS epoch 1980-01-06T00:00:00Z = 315964800
            expected_gpst = 315964800.0 + 2436 * 604800.0 + 204589.000
            self.assertAlmostEqual(recs[0]["t"], expected_gpst - 18.0, places=3)

    def test_week_tow_honours_utc_header(self):
        with tempfile.TemporaryDirectory() as d:
            p = write(d, "a.pos", self.HEADER.replace("GPST", "UTC") + self.ROW)
            recs = mod.parse_pos(p)
            expected = 315964800.0 + 2436 * 604800.0 + 204589.000
            self.assertAlmostEqual(recs[0]["t"], expected, places=3)

    def test_week_tow_rejects_out_of_range_tow(self):
        with tempfile.TemporaryDirectory() as d:
            bad = self.ROW.replace("204589.000", "999999.000")   # > 604800
            p = write(d, "a.pos", self.HEADER + bad)
            self.assertEqual(mod.parse_pos(p), [])

    def test_real_rtk_check_pos_is_not_silently_empty(self):
        """真实文件存在时顺带跑一次;不存在就跳过(测试不依赖数据集)。"""
        path = ("/home/steve/Documents/Datasets/tage/hongshaquan/20260915/"
                "seg_164931_165748/gnss/rtk_check.pos")
        if not os.path.exists(path):
            self.skipTest("dataset not present")
        recs = mod.parse_pos(path)
        self.assertEqual(len(recs), 498)


class ParseEnu(unittest.TestCase):
    TEXT = (
        "# origin_lla 44.47025713 90.29451498 610.640\n"
        "# t e n u sde sdn sdu q\n"
        "1789462172.000 7.412 -3.118 0.353 0.0764 0.0857 0.1929 4\n"
        "garbage\n"
        "1789462173.000 8.0 -3.0\n"
        "1789462174.000 9.100 -2.900 0.400 0.0800 0.0900 0.2000 3\n"
    )

    def test_parses_origin_and_skips_bad_lines(self):
        with tempfile.TemporaryDirectory() as d:
            p = write(d, "a.enu", self.TEXT)
            origin, samples = mod.parse_enu(p)
            self.assertAlmostEqual(origin[0], 44.47025713, places=8)
            self.assertAlmostEqual(origin[2], 610.640, places=3)
            self.assertEqual(len(samples), 2)
            self.assertAlmostEqual(samples[0]["e"], 7.412, places=3)
            self.assertAlmostEqual(samples[0]["sdu"], 0.1929, places=4)
            self.assertEqual(samples[0]["q"], 4)
            self.assertEqual(samples[1]["q"], 3)

    def test_crlf_lines(self):
        with tempfile.TemporaryDirectory() as d:
            p = write(d, "a.enu", self.TEXT.replace("\n", "\r\n"))
            _, samples = mod.parse_enu(p)
            self.assertEqual(len(samples), 2)

    def test_missing_origin_returns_none(self):
        with tempfile.TemporaryDirectory() as d:
            p = write(d, "a.enu", "1789462172.000 1 2 3 0.1 0.1 0.1 4\n")
            origin, samples = mod.parse_enu(p)
            self.assertIsNone(origin)
            self.assertEqual(len(samples), 1)


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: 运行测试确认它失败**

```bash
cd /home/steve/glim_ws/src/glim_ext/gnss_core
python3 -m unittest discover -s tests -p "test_*.py" -v 2>&1 | tail -30
```

Expected: `ParsePosWeekTow` 四个用例全红（`len(recs)` 为 0，因为 `parse_pos` 只认日历格式），`ParseEnu` 三个用例报 `AttributeError: module 'pos_to_rtkfix_bag' has no attribute 'parse_enu'`。**抄进提交 message。**

- [ ] **Step 3: 改 `parse_pos` 支持周/周内秒**

把 `pos_to_rtkfix_bag.py` 里 `parse_pos` 的时间解析那段（当前是单一的 `datetime.strptime(c[0] + " " + c[1][:12], ...)`）替换成：

```python
GPS_EPOCH_UNIX = 315964800.0        # 1980-01-06T00:00:00Z
SECONDS_PER_WEEK = 604800.0


def _parse_stamp(c0, c1):
    """时间两列 → unix 秒(尚未做 GPST→UTC 换算);解析不出来返回 None。

    两种格式(与 gnss_core::read_pos 一致):
      - 日历:  "2026/09/15" "08:49:50.000"
      - 周/tow: "2436" "204589.000"(rnx2rtkp 默认)
    """
    if "/" in c0:
        try:
            dt = datetime.strptime(c0 + " " + c1[:12], "%Y/%m/%d %H:%M:%S.%f")
        except ValueError:
            return None
        return dt.replace(tzinfo=timezone.utc).timestamp()
    try:
        week = int(c0)
        tow = float(c1)
    except ValueError:
        return None
    if week < 0 or not (0.0 <= tow < SECONDS_PER_WEEK):
        return None
    return GPS_EPOCH_UNIX + week * SECONDS_PER_WEEK + tow
```

`parse_pos` 的循环体里改成：

```python
            t = _parse_stamp(c[0], c[1])
            if t is None:
                continue
            if ts == "GPST":
                t -= leap
```

（原来的 `try/except ValueError: continue` 整块由 `_parse_stamp` 返回 `None` 取代。）

- [ ] **Step 4: 加 `parse_enu`**

在 `parse_pos` 之后加：

```python
def parse_enu(path):
    """读 pos_to_enu 写出的 .enu(格式见 gnss_core/enu_track.hpp)。

    返回 (origin_lla | None, samples);samples 每项
    {"t","e","n","u","sde","sdn","sdu","q"},q 是归一化质量枚举(0..4)。
    注释行、列数不足的行、非数值行一律跳过,不中断。
    """
    origin = None
    samples = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            if line.startswith("#"):
                tok = line[1:].split()
                if len(tok) >= 4 and tok[0] == "origin_lla":
                    try:
                        origin = (float(tok[1]), float(tok[2]), float(tok[3]))
                    except ValueError:
                        origin = None
                continue
            c = line.split()
            if len(c) < 8:
                continue
            try:
                s = {"t": float(c[0]), "e": float(c[1]), "n": float(c[2]), "u": float(c[3]),
                     "sde": float(c[4]), "sdn": float(c[5]), "sdu": float(c[6]), "q": int(c[7])}
            except ValueError:
                continue
            if not 0 <= s["q"] <= 4:
                continue
            samples.append(s)
    return origin, samples
```

- [ ] **Step 5: 运行测试确认通过**

```bash
cd /home/steve/glim_ws/src/glim_ext/gnss_core
python3 -m unittest discover -s tests -p "test_*.py" -v 2>&1 | tail -20
```

Expected: 全部 PASS（含既有的 `test_export_bag_to_pos.py`）。

- [ ] **Step 6: 变异确认**

把 `_parse_stamp` 里 `GPS_EPOCH_UNIX` 改成 `315964818.0`（故意把闰秒算两次），重跑 → `test_week_tow_is_parsed_as_gpst` 与 `test_week_tow_honours_utc_header` 必须红。改回来。**抄进提交 message。**

- [ ] **Step 7: 加 `--pose-enu` / `--pose-topic` / `--pose-frame`**

`main()` 的 `argparse` 后面加三个参数：

```python
    ap.add_argument("--pose-enu", default=None,
                    help="pos_to_enu 产出的 .enu;给上游 gnss_global 写 PoseWithCovarianceStamped")
    ap.add_argument("--pose-topic", default="/gnss/pose")
    ap.add_argument("--pose-frame", default="enu")
```

`pos` 位置参数改成可选（只写 pose 时不需要 `.pos`）：把 `ap.add_argument("pos")` 改成 `ap.add_argument("pos", nargs="?", default=None)`，并在解析后加：

```python
    if args.pos is None and args.pose_enu is None:
        print("需要 <in.pos> 或 --pose-enu 至少一个", file=sys.stderr)
        return 2
```

`recs = parse_pos(...)` 那段改成只在 `args.pos` 非空时执行，且"解析出 0 条"的早退也只在那时生效。

ROS 依赖的 `try: import` 块里加：

```python
        from geometry_msgs.msg import PoseWithCovarianceStamped
```

在 `make_msg` 之后加：

```python
    def make_pose(s):
        m = PoseWithCovarianceStamped()
        m.header.stamp = to_time(s["t"] + args.header_delay)
        m.header.frame_id = args.pose_frame
        m.pose.pose.position.x = s["e"]
        m.pose.pose.position.y = s["n"]
        m.pose.pose.position.z = s["u"]
        m.pose.pose.orientation.w = 1.0
        # gnss_global 完全忽略协方差(gnss_global_module.hpp:89-95),这里照样填上,
        # 方便别的消费者与肉眼核对;姿态部分给一个大值表示"未知"。
        cov = [0.0] * 36
        cov[0] = s["sde"] ** 2
        cov[7] = s["sdn"] ** 2
        cov[14] = s["sdu"] ** 2
        cov[21] = cov[28] = cov[35] = 1e6
        m.pose.covariance = cov
        return m
```

写出端：`create_topic` 里为 pose 话题再注册一个（`type="geometry_msgs/msg/PoseWithCovarianceStamped"`，`id` 用一个不与 merge 来源冲突的值——把 merge 的 `enumerate(..., start=1)` 改成 `start=2`，`0` 给 `--topic`、`1` 给 `--pose-topic`）。归并迭代器加第三路：

```python
    def pose_iter():
        for s in sorted(pose_samples, key=lambda x: x["t"]):
            t_ns = int(round((s["t"] + args.header_delay) * 1e9))
            yield (t_ns, 1, args.pose_topic, serialize_message(make_pose(s)))
```

`heapq.merge` 把三路一起并（`lidar_iter()` 的次序键仍是 0，两路合成的是 1，保证同一时刻 LiDAR 先写）。计数行相应改成：

```python
    print(f"wrote {n_rtk} RtkFix on {args.topic}, {n_pose} poses on {args.pose_topic}"
          + (f" + {n_merge} merged messages" if args.merge else "") + f" -> {args.out}")
```

把文件头的 docstring 更新：usage 行加上 `[--pose-enu <f.enu> --pose-topic /gnss/pose]`，并说明"`--merge` 的源 bag 里与 `--topic`/`--pose-topic` 同名的话题会被丢弃并打警告"（已有逻辑，对 pose 话题也要同样生效——在 merge 的话题循环里把判断从 `tm.name == args.topic` 改成 `tm.name in (args.topic, args.pose_topic)`）。

- [ ] **Step 8: 冒烟验证（需要 ROS）**

```bash
cd /home/steve/glim_ws && source install/setup.bash
SEGA=/home/steve/Documents/Datasets/tage/hongshaquan/20260915/seg_164931_165748
OUT=$SEGA/validation_20261007
rm -rf /tmp/round5_smoke
head -40 $OUT/gnss/rtkrcv.enu > /tmp/smoke.enu
head -5 $SEGA/integration_20260916/run_B/pos/20260915/rtkrcv.pos > /tmp/smoke.pos
python3 install/gnss_core/lib/gnss_core/pos_to_rtkfix_bag.py /tmp/smoke.pos \
  --out /tmp/round5_smoke --topic /rtkrcv_node/rtk_fix \
  --pose-enu /tmp/smoke.enu --pose-topic /gnss/pose
ros2 bag info /tmp/round5_smoke
```

Expected: `ros2 bag info` 里两个话题都在——`/rtkrcv_node/rtk_fix`（`gnss_msgs/msg/RtkFix`，2 条：`head -5` 只含 1 行数据 + 4 行头，按实际条数核对）与 `/gnss/pose`（`geometry_msgs/msg/PoseWithCovarianceStamped`，38 条）。条数与输入行数对不上就停下来查。

**这是 `pos_to_rtkfix_bag.py` 第一次被实际运行**（它的 docstring 原话：「本脚本在无 ROS 的云端环境编写，未在本环境运行过」）。把 `ros2 bag info` 的完整输出记进结果文档 §3。

- [ ] **Step 9: 改 CMake 测试名并提交**

`CMakeLists.txt` 里把

```cmake
  add_test(NAME test_export_bag_to_pos
```

改成

```cmake
  # discover 整个 tests/ 目录(目前是 test_export_bag_to_pos.py 与 test_pos_to_rtkfix_bag.py),
  # 所以名字不跟单个文件绑定。
  add_test(NAME test_tools_python
```

```bash
cd /home/steve/glim_ws/src/glim_ext
git add gnss_core/tools/pos_to_rtkfix_bag.py gnss_core/tests/test_pos_to_rtkfix_bag.py gnss_core/CMakeLists.txt
git commit -F - <<'EOF'
gnss_core: pos_to_rtkfix_bag.py — week/tow time column + pose output

两处缺口:
1. 时间列只认日历格式,对 rnx2rtkp 默认输出的 "GPS 周 + 周内秒" 静默解析出 0 条。
   数据集的后处理基准 gnss/rtk_check.pos 正是这个格式,§12.3/§12.4 都要用它。
   gnss_core::read_pos 早已支持(PR #8 8fefcdc),Python 侧没跟上。
2. spec §12.4 的 B 组 libgnss_global.so 只订阅 PoseWithCovarianceStamped。
   现在一次调用就能把 RtkFix(来自 .pos)与 Pose(来自 pos_to_enu 的 .enu)写进
   同一个合并 bag,A/B/C 三组共用一份输入,差异只剩模块本身。

另:把 Python 测试的 add_test 名字从 test_export_bag_to_pos 改成 test_tools_python
——它 discover 整个 tests/ 目录,名字早就名不副实了。

RED:
  ParsePosWeekTow.test_week_tow_is_parsed_as_gpst            FAIL (0 != 1)
  ParsePosWeekTow.test_week_tow_honours_utc_header           FAIL (0 != 1)
  ParsePosWeekTow.test_week_tow_rejects_out_of_range_tow     FAIL
  ParsePosWeekTow.test_real_rtk_check_pos_is_not_silently_empty FAIL (0 != 498)
  ParseEnu.*                                                 ERROR (no attribute 'parse_enu')
变异:GPS_EPOCH_UNIX 改成 315964818.0(闰秒算两次)→ 两个 week/tow 用例红。

冒烟:本脚本此前从未运行过(docstring 自述"未在本环境运行过")。
ros2 bag info 确认两个话题与条数,输出记在
docs/gnss/field/2026-10-07-round5-validation-closure.md §3。

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>
EOF
```

---

### Task 4: §12.2 —— `librtk_global.so` 被 GLIM 加载并产出因子（A 组首跑）

**Files:**
- Create: `src/glim_ext/gnss_core/tools/glim_config_patch.py`
- Create: `src/glim_ext/gnss_core/tests/test_glim_config_patch.py`
- Modify: `src/glim_ext/gnss_core/CMakeLists.txt`（`install(PROGRAMS ...)` 里加 `tools/glim_config_patch.py`）
- Modify: `src/glim_underground/docs/gnss/field/2026-10-07-round5-validation-closure.md`（加 §3 工具冒烟、§4 §12.2 结果两节）

**Interfaces:**
- Consumes: Task 1 的 `config/tage_at128/`；Task 2 的 `.enu`；Task 3 的 `pos_to_rtkfix_bag.py`
- Produces:
  ```python
  # glim_config_patch.py
  strip_jsonc(text: str) -> str          # 去掉 // 与 /* */ 注释,保留字符串里的同形字符
  load_jsonc(path: str) -> dict
  patch_config_set(src_dir, dst_dir, *, dump_path=None, extension_modules=None,
                   rtk_global=None, rtk_odometry=None, gnss_global=None) -> str
      # 把 src_dir 整份拷到 dst_dir(dst_dir 已存在则先删),按参数改写:
      #   config_ros.json      : glim_ros.dump_path / glim_ros.extension_modules
      #   config_rtk_global.json / config_rtk_odometry.json / config_gnss_global.json
      #     : 对应顶层块里的键逐个覆盖(传 dict,只改给出的键)
      # 返回 dst_dir。写出的 JSON 不带注释(生成物,原配置集不动)。
  ```
  CLI：`glim_config_patch.py <src_dir> <dst_dir> [--dump-path P] [--modules a.so,b.so] [--rtk-global k=v ...] [--rtk-odometry k=v ...] [--gnss-global k=v ...]`，`v` 按 JSON 解析（`15.0`、`"huber"`、`[1.6,0.85,2.88]` 都可）。

- [ ] **Step 1: 写失败的测试**

`src/glim_ext/gnss_core/tests/test_glim_config_patch.py`：

```python
#!/usr/bin/env python3
"""glim_config_patch.py 的测试(轮 5 Task 4)。

GLIM 的配置是 JSON-with-comments。run_injection_suite.sh 原来的 make_config 直接
json.load(config_ros.json),一定抛 JSONDecodeError —— 这组测试先把注释容错的读取
与改写固定下来,Task 6 再让脚本改用它。
"""
import json
import os
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "tools"))

import glim_config_patch as mod  # noqa: E402


class StripJsonc(unittest.TestCase):
    def test_line_comment(self):
        self.assertEqual(json.loads(mod.strip_jsonc('{"a": 1 // note\n}')), {"a": 1})

    def test_block_comment_multiline(self):
        text = '{\n/*** header\n   more\n ***/\n"a": 2\n}'
        self.assertEqual(json.loads(mod.strip_jsonc(text)), {"a": 2})

    def test_keeps_slashes_inside_strings(self):
        text = '{"p": "a//b", "q": "x/*y*/z"}'
        self.assertEqual(json.loads(mod.strip_jsonc(text)), {"p": "a//b", "q": "x/*y*/z"})

    def test_keeps_escaped_quote_inside_string(self):
        text = '{"p": "he said \\"hi\\" // not a comment"}'
        self.assertEqual(json.loads(mod.strip_jsonc(text))["p"], 'he said "hi" // not a comment')

    def test_real_glim_config_loads(self):
        path = "/home/steve/glim_ws/src/glim_underground/config/tage_at128/config_ros.json"
        if not os.path.exists(path):
            self.skipTest("tage_at128 config set not present")
        cfg = mod.load_jsonc(path)
        self.assertEqual(cfg["glim_ros"]["points_topic"], "/at128/points")
        self.assertEqual(cfg["glim_ros"]["imu_topic"], "/cgi610/imu")


class PatchConfigSet(unittest.TestCase):
    def _src(self, d):
        src = os.path.join(d, "src_cfg")
        os.makedirs(src)
        with open(os.path.join(src, "config_ros.json"), "w") as f:
            f.write('{\n // comment\n "glim_ros": {"extension_modules": ["libx.so"],'
                    ' "imu_topic": "/cgi610/imu"}\n}')
        with open(os.path.join(src, "config_rtk_global.json"), "w") as f:
            f.write('{\n /*** doc ***/\n "rtk_global": {"sigma_floor": [1.0, 1.0, 1.0],'
                    ' "T_imu_gnss": [0.0, 0.0, 0.0], "robust_kernel": "huber"}\n}')
        with open(os.path.join(src, "config_other.json"), "w") as f:
            f.write('{"keep": true}')
        return src

    def test_copies_everything_and_patches_requested_keys(self):
        with tempfile.TemporaryDirectory() as d:
            src = self._src(d)
            dst = os.path.join(d, "dst_cfg")
            out = mod.patch_config_set(
                src, dst, dump_path="/tmp/dump_x",
                extension_modules=["librtk_global.so"],
                rtk_global={"sigma_floor": [0.05, 0.05, 0.1], "T_imu_gnss": [1.6, 0.85, 2.88]})
            self.assertEqual(out, dst)
            ros = mod.load_jsonc(os.path.join(dst, "config_ros.json"))
            self.assertEqual(ros["glim_ros"]["dump_path"], "/tmp/dump_x")
            self.assertEqual(ros["glim_ros"]["extension_modules"], ["librtk_global.so"])
            self.assertEqual(ros["glim_ros"]["imu_topic"], "/cgi610/imu")   # 未动的键保留
            rtk = mod.load_jsonc(os.path.join(dst, "config_rtk_global.json"))
            self.assertEqual(rtk["rtk_global"]["sigma_floor"], [0.05, 0.05, 0.1])
            self.assertEqual(rtk["rtk_global"]["T_imu_gnss"], [1.6, 0.85, 2.88])
            self.assertEqual(rtk["rtk_global"]["robust_kernel"], "huber")   # 未动的键保留
            # 不相关的文件原样拷过来
            self.assertTrue(os.path.exists(os.path.join(dst, "config_other.json")))

    def test_source_is_not_modified(self):
        with tempfile.TemporaryDirectory() as d:
            src = self._src(d)
            before = open(os.path.join(src, "config_ros.json")).read()
            mod.patch_config_set(src, os.path.join(d, "dst"), dump_path="/tmp/x")
            self.assertEqual(open(os.path.join(src, "config_ros.json")).read(), before)

    def test_existing_dst_is_replaced(self):
        with tempfile.TemporaryDirectory() as d:
            src = self._src(d)
            dst = os.path.join(d, "dst")
            os.makedirs(dst)
            with open(os.path.join(dst, "stale.json"), "w") as f:
                f.write("{}")
            mod.patch_config_set(src, dst, dump_path="/tmp/x")
            self.assertFalse(os.path.exists(os.path.join(dst, "stale.json")))

    def test_unknown_key_raises(self):
        with tempfile.TemporaryDirectory() as d:
            src = self._src(d)
            with self.assertRaises(KeyError):
                mod.patch_config_set(src, os.path.join(d, "dst"),
                                     rtk_global={"no_such_key": 1})

    def test_missing_module_config_raises(self):
        """要改 gnss_global 但配置集里没有这个文件 → 必须报错,不能静默跳过。"""
        with tempfile.TemporaryDirectory() as d:
            src = self._src(d)
            with self.assertRaises(FileNotFoundError):
                mod.patch_config_set(src, os.path.join(d, "dst"),
                                     gnss_global={"gnss_topic": "/gnss/pose"})


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: 运行测试确认它失败**

```bash
cd /home/steve/glim_ws/src/glim_ext/gnss_core
python3 -m unittest discover -s tests -p "test_*.py" -v 2>&1 | tail -20
```

Expected: `ModuleNotFoundError: No module named 'glim_config_patch'`。

- [ ] **Step 3: 写实现**

`src/glim_ext/gnss_core/tools/glim_config_patch.py`：

```python
#!/usr/bin/env python3
"""glim_config_patch.py —— 拷一份 GLIM 配置集并改写几个键(轮 5 Task 4)。

GLIM 的配置是 JSON-with-comments(`//` 与 `/* */`),标准 json 模块读不了;
run_injection_suite.sh 原来的 make_config 直接 json.load 一定崩。这里把注释剥离
与改写做成带测试的函数,供注入套件与 §12.4 的 A/B/C 共用。

生成物不带注释(它是 run 的快照,原配置集一个字节都不动)。

usage:
  glim_config_patch.py <src_dir> <dst_dir> [--dump-path P] [--modules a.so,b.so]
      [--rtk-global k=v ...] [--rtk-odometry k=v ...] [--gnss-global k=v ...]
  v 按 JSON 解析:15.0 / "huber" / [1.6,0.85,2.88] / true 都可。
"""
import argparse
import json
import os
import shutil
import sys

# 顶层块名 → 文件名。改这些块的键时文件必须存在,不存在就报错(静默跳过会让
# "我以为我改了" 变成一次无声的错误对比)。
MODULE_FILES = {
    "rtk_global": "config_rtk_global.json",
    "rtk_odometry": "config_rtk_odometry.json",
    "gnss": "config_gnss_global.json",     # gnss_global 的顶层块名就是 "gnss"
}


def strip_jsonc(text):
    """去掉 // 行注释与 /* */ 块注释,字符串字面量里的同形字符保留。"""
    out = []
    i, n = 0, len(text)
    in_str = False
    while i < n:
        c = text[i]
        if in_str:
            out.append(c)
            if c == "\\" and i + 1 < n:
                out.append(text[i + 1])
                i += 2
                continue
            if c == '"':
                in_str = False
            i += 1
            continue
        if c == '"':
            in_str = True
            out.append(c)
            i += 1
            continue
        if c == "/" and i + 1 < n and text[i + 1] == "/":
            while i < n and text[i] != "\n":
                i += 1
            continue
        if c == "/" and i + 1 < n and text[i + 1] == "*":
            i += 2
            while i + 1 < n and not (text[i] == "*" and text[i + 1] == "/"):
                if text[i] == "\n":
                    out.append("\n")   # 保留行号,方便 JSONDecodeError 定位
                i += 1
            i += 2
            continue
        out.append(c)
        i += 1
    return "".join(out)


def load_jsonc(path):
    with open(path) as f:
        return json.loads(strip_jsonc(f.read()))


def _save(path, obj):
    with open(path, "w") as f:
        json.dump(obj, f, indent=2)
        f.write("\n")


def _patch_block(dst_dir, block, updates):
    if not updates:
        return
    filename = MODULE_FILES[block]
    path = os.path.join(dst_dir, filename)
    if not os.path.exists(path):
        raise FileNotFoundError(f"{filename} not in config set {dst_dir} (needed to patch {block})")
    cfg = load_jsonc(path)
    body = cfg.get(block)
    if body is None:
        raise KeyError(f'{filename} has no top-level "{block}" block')
    for k, v in updates.items():
        if k not in body:
            raise KeyError(f'{filename}: "{block}" has no key "{k}"')
        body[k] = v
    _save(path, cfg)


def patch_config_set(src_dir, dst_dir, *, dump_path=None, extension_modules=None,
                     rtk_global=None, rtk_odometry=None, gnss_global=None):
    if os.path.exists(dst_dir):
        shutil.rmtree(dst_dir)
    shutil.copytree(src_dir, dst_dir)

    if dump_path is not None or extension_modules is not None:
        path = os.path.join(dst_dir, "config_ros.json")
        cfg = load_jsonc(path)
        body = cfg.get("glim_ros")
        if body is None:
            raise KeyError('config_ros.json has no top-level "glim_ros" block')
        if dump_path is not None:
            body["dump_path"] = dump_path       # 新键:glim_rosbag 读的是 ROS 参数,
                                                # 这里写进 config 只为留档"这次 run 用了哪个 dump 目录"
        if extension_modules is not None:
            body["extension_modules"] = list(extension_modules)
        _save(path, cfg)

    _patch_block(dst_dir, "rtk_global", rtk_global)
    _patch_block(dst_dir, "rtk_odometry", rtk_odometry)
    _patch_block(dst_dir, "gnss", gnss_global)
    return dst_dir


def _kv(pairs):
    out = {}
    for p in pairs or []:
        if "=" not in p:
            raise SystemExit(f"expected key=value, got {p!r}")
        k, v = p.split("=", 1)
        try:
            out[k] = json.loads(v)
        except json.JSONDecodeError:
            out[k] = v          # 裸字符串(不带引号的 huber 之类)
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("src_dir")
    ap.add_argument("dst_dir")
    ap.add_argument("--dump-path", default=None)
    ap.add_argument("--modules", default=None, help="逗号分隔;空串表示不加载任何扩展模块")
    ap.add_argument("--rtk-global", nargs="*", default=None, metavar="k=v")
    ap.add_argument("--rtk-odometry", nargs="*", default=None, metavar="k=v")
    ap.add_argument("--gnss-global", nargs="*", default=None, metavar="k=v")
    a = ap.parse_args(argv)

    modules = None
    if a.modules is not None:
        modules = [m for m in a.modules.split(",") if m]
    out = patch_config_set(a.src_dir, a.dst_dir, dump_path=a.dump_path,
                           extension_modules=modules,
                           rtk_global=_kv(a.rtk_global) if a.rtk_global is not None else None,
                           rtk_odometry=_kv(a.rtk_odometry) if a.rtk_odometry is not None else None,
                           gnss_global=_kv(a.gnss_global) if a.gnss_global is not None else None)
    print(out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
```

`CMakeLists.txt` 的 `install(PROGRAMS tools/pos_to_rtkfix_bag.py tools/run_injection_suite.sh tools/traj_rmse.py` 这一行加上 `tools/glim_config_patch.py`。

- [ ] **Step 4: 运行测试确认通过 + 变异**

```bash
cd /home/steve/glim_ws/src/glim_ext/gnss_core
python3 -m unittest discover -s tests -p "test_*.py" -v 2>&1 | tail -20
```

Expected: 全绿。变异：把 `strip_jsonc` 里 `if c == '"': in_str = True` 那个分支删掉 → `test_keeps_slashes_inside_strings` 必须红。改回来。两段输出都抄进提交 message。

- [ ] **Step 5: 做 A 组的合并 bag**

```bash
cd /home/steve/glim_ws && source install/setup.bash
SEGA=/home/steve/Documents/Datasets/tage/hongshaquan/20260915/seg_164931_165748
OUT=$SEGA/validation_20261007
python3 install/gnss_core/lib/gnss_core/pos_to_rtkfix_bag.py \
  $SEGA/integration_20260916/run_B/pos/20260915/rtkrcv.pos \
  --out $OUT/gnss/bag_lio_gnss \
  --topic /rtkrcv_node/rtk_fix \
  --pose-enu $OUT/gnss/rtkrcv.enu --pose-topic /gnss/pose \
  --merge $SEGA/bag_lio
ros2 bag info $OUT/gnss/bag_lio_gnss | tee $OUT/gnss/bag_lio_gnss.info
```

Expected: `/at128/points` 4966、`/cgi610/imu` 49731、`/jt128/points` 4956、`/jt128/imu` 99398、`/cgi610/odom` 4974、`/cgi610/fix` 4974、`/tf_static` 1、`/rtkrcv_node/rtk_fix` 497、`/gnss/pose` 497。总计 169000 + 994。磁盘约 10 GB。

- [ ] **Step 6: 生成 A 组配置并跑（§12.2 的验证动作）**

```bash
cd /home/steve/glim_ws && source install/setup.bash
SEGA=/home/steve/Documents/Datasets/tage/hongshaquan/20260915/seg_164931_165748
OUT=$SEGA/validation_20261007
mkdir -p $OUT/config $OUT/run_A
python3 install/gnss_core/lib/gnss_core/glim_config_patch.py \
  src/glim_underground/config/tage_at128 $OUT/config/run_A \
  --dump-path $OUT/run_A --modules librtk_global.so
ros2 run glim_ros glim_rosbag $OUT/gnss/bag_lio_gnss \
  --ros-args -p config_path:=$OUT/config/run_A -p auto_quit:=true -p dump_path:=$OUT/run_A \
  2>&1 | tee $OUT/run_A/glim.log
```

- [ ] **Step 7: 跑 `check_rtk_global_log.sh`（本 Task 的验收断言）**

```bash
cd /home/steve/glim_ws
SEGA=/home/steve/Documents/Datasets/tage/hongshaquan/20260915/seg_164931_165748
src/glim_ext/modules/mapping/rtk_global/tools/check_rtk_global_log.sh \
  $SEGA/validation_20261007/run_A/glim.log \
  | tee $SEGA/validation_20261007/eval/check_rtk_global.txt
```

Expected —— 八行全 `ok`，退出码 0：

```
ok    module initialised
ok    backend thread started
ok    T_world_enu bootstrapped
ok    fixes received (>0)
ok    factors added (>0)
ok    no msg type mismatch
ok    no deserialisation failure
ok    no config load failure
rtk_global stats: fix_received=497 fix_dropped=0 fix_pushed=497 anchors=… factors_added=… …
```

**这是 spec §12.2 的全部验证内容，也是轮 1 验收项 V5。**

常见失败与对策（逐条写进结果文档）：
- `FAIL fixes received (>0)`：bag 里的话题名与 `config_rtk_global.json` 的 `rtk_fix_topic` 不一致，或 `glim_rosbag` 的 `filter.topics` 没包含它（它来自 `extension_subscriptions()`，所以模块没加载时根本不会读这个话题）→ 先看 `module initialised` 是否 ok。
- `FAIL T_world_enu bootstrapped`：行驶里程没到 `min_baseline`（50 m）。本段 1329 m，不应发生；若真发生，检查 `.enu`/`.pos` 的 ENU 是否退化。
- 日志里出现 `submap anchor older than the RTK buffer`：`stamp_source`/`time_offset` 或时间系统有问题（`fix_buffer_horizon` 是 600 s，段长 497 s，正常不会触发）。
- `factors_added=0` 而 `fix_pushed>0`：看 stats 行的 `gated`/`gap`/`too_old` 哪个在涨——`gated` 高说明 `min_quality`/`min_sats`/`max_diff_age` 挡住了。

- [ ] **Step 8: A 组 RMSE，与 C 组对比**

```bash
cd /home/steve/glim_ws && source install/setup.bash
SEGA=/home/steve/Documents/Datasets/tage/hongshaquan/20260915/seg_164931_165748
OUT=$SEGA/validation_20261007
python3 install/gnss_core/lib/gnss_core/traj_rmse.py \
  $SEGA/gt/gt_cgi610_imu_flu.tum $OUT/run_A/traj_imu.txt --align --tol 0.05 \
  | tee $OUT/eval/A.md
```

记录 A 与 C 的 RMSE 对比。**此时 `T_imu_gnss` 还是 0、`sigma_floor` 还是 1.0 m**，所以 A 不一定比 C 好——这正是 Task 5 要解决的；把这个事实写进结果文档，不要在这里调参。

- [ ] **Step 9: 提交**

```bash
cd /home/steve/glim_ws/src/glim_ext
git add gnss_core/tools/glim_config_patch.py gnss_core/tests/test_glim_config_patch.py gnss_core/CMakeLists.txt
git commit -F - <<'EOF'
gnss_core: add glim_config_patch.py (JSONC-aware GLIM config set patcher)

GLIM 的配置是 JSON-with-comments;run_injection_suite.sh 原来的 make_config 直接
json.load(config_ros.json),一定抛 JSONDecodeError —— 那个脚本从未被运行过,所以
这个必崩的 bug 一直留着。这里把"剥注释 + 拷配置集 + 改几个键"做成带测试的函数,
§12.2/§12.3/§12.4 三处共用;Task 6 让注入套件改用它。

改不存在的键或块抛 KeyError、要改的模块配置文件缺失抛 FileNotFoundError:
静默跳过会让"我以为我改了"变成一次无声的错误对比。

RED:ModuleNotFoundError: No module named 'glim_config_patch'(11 个用例全 ERROR)。
变异:删掉 strip_jsonc 里进入字符串状态的分支 → test_keeps_slashes_inside_strings 红。

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>
EOF
```

然后把 §3（工具冒烟：`pos_to_rtkfix_bag.py` 的 `ros2 bag info`、合并 bag 的条数表）与 §4（§12.2：`check_rtk_global_log.sh` 八行输出、`rtk_global stats` 整行、A vs C 的 RMSE）写进结果文档并提交到 `glim_underground`。

---

### Task 5: §9.3 杆臂与时间偏移、§9.2 定权系数 —— 实测并写回配置

**Files:**
- Modify: `src/glim_underground/config/tage_at128/config_rtk_global.json`（`T_imu_gnss`、`time_offset`、`sigma_floor`、可能的 `quality_sigma_scale`）
- Modify: `src/glim_underground/config/tage_at128/config_rtk_odometry.json`（同上三项，保持两模块一致）
- Modify: `src/glim_ext/config/config_rtk_global.json`（只改注释，指向实测结论；默认值不动——设计决定 8）
- Modify: `src/glim_ext/gnss_core/tools/README.md`（§2 合成示例换成实测、§6 待办勾选、表格加 `pos_to_enu`）
- Modify: `src/glim_underground/docs/gnss/field/2026-10-07-round5-validation-closure.md`（加 §5 §9.3、§6 §9.2 两节）

**Interfaces:**
- Consumes: Task 4 的 `run_A/traj_imu.txt`；`<segA>/integration_20260916/run_B/pos/20260915/rtkrcv.pos`；`<segA>/gnss/rtk_check.pos`
- Produces: `<segA>/validation_20261007/eval/lever_arm.txt`、`sigma_scale.txt`、`run_A2/`（降档后的 run）与 `eval/A2.md`

- [ ] **Step 1: 跑 `estimate_lever_arm`**

```bash
cd /home/steve/glim_ws && source install/setup.bash
SEGA=/home/steve/Documents/Datasets/tage/hongshaquan/20260915/seg_164931_165748
OUT=$SEGA/validation_20261007
install/gnss_core/lib/gnss_core/estimate_lever_arm \
  $OUT/run_A/traj_imu.txt \
  $SEGA/integration_20260916/run_B/pos/20260915/rtkrcv.pos \
  --origin 44.470257133 90.294514984 610.6395 \
  --min-quality 4 --pair-tol 0.05 --dt-range -0.5 0.5 --dt-step 0.01 \
  | tee $OUT/eval/lever_arm.txt
```

Expected（逐条核对，不符合就在结果文档里写清楚为什么）：
- `n_pairs` 应在 400 左右（497 个 1 Hz 历元 × 与 submap 轨迹的重叠；`traj_imu.txt` 是 submap 级，配对数可能明显更少 —— 若 `n_pairs < 50`，改用 `run_A/traj_lidar.txt` 不行（那是雷达系），应改成用 `--pair-tol 0.6` 放宽，或改用帧级轨迹；无论怎么调都要记进文档）
- `lever_imu` 应落在 **`[1.600, 0.850, 2.880]`** 附近。水平两维（x 前、y 左）的差值直接就是对 `extrinsics_tage_truck.yaml` 里"未验证"的 `T_base_link_cgi610_imu = (6.85, 1.00, 2.28)` 的检验。
- 航向累计 1778° → `yaw range` 应 ≫ 30°，**不应**出现 `lever NOT observable`。
- 车辆在平地绕圈 → 很可能出现 `NOTE: lever.z not observable`，此时 `lever.z` 被钉成 0、由 `T_world_enu.z` 吸收。**这是预期行为**：z 杆臂取设计值 2.88，只把 x/y 换成实测，并在结果文档写明哪一维是测的、哪一维是抄的。
- `time_offset`：AT128 用 PTP、CGI-610 用 GPS 时间减 18 s（`README_LIO.md` §3），期望接近 0（|Δt| < 50 ms）。出现 `time_offset NOT observable` 就保持 0。
- `rms_residual` 应是 cm 量级；> 0.5 m 说明轨迹/`.pos` 对不上，先查时间系统。

- [ ] **Step 2: 写回配置**

按 `estimate_lever_arm` 输出末尾的 `--- paste into config_rtk_global.json ---` 两行，改 `tage_at128/config_rtk_global.json`：

```
  "time_offset": <实测值,不可观则 0.0>,
  "T_imu_gnss": [<实测 x>, <实测 y>, <实测或设计值 z>],
  "sigma_floor": [0.05, 0.05, 0.1],
```

`sigma_floor` 的降档依据是 `config_rtk_global.json` 自带的注释（逐字）：`After the lever arm is filled in, lower to e.g. [0.02, 0.02, 0.05]`，以及 round1 plan 的 V8（`[0.05,0.05,0.1]`）。**取 V8 的 `[0.05, 0.05, 0.1]`**，与 round1 验收项一致。

同样三项改进 `tage_at128/config_rtk_odometry.json`（两个模块同一套外参，不一致会在轮 1.5 开门时出怪事）。

- [ ] **Step 3: 跑 run_A2（降档后复跑）**

```bash
cd /home/steve/glim_ws && source install/setup.bash
SEGA=/home/steve/Documents/Datasets/tage/hongshaquan/20260915/seg_164931_165748
OUT=$SEGA/validation_20261007
mkdir -p $OUT/run_A2
python3 install/gnss_core/lib/gnss_core/glim_config_patch.py \
  src/glim_underground/config/tage_at128 $OUT/config/run_A2 \
  --dump-path $OUT/run_A2 --modules librtk_global.so
ros2 run glim_ros glim_rosbag $OUT/gnss/bag_lio_gnss \
  --ros-args -p config_path:=$OUT/config/run_A2 -p auto_quit:=true -p dump_path:=$OUT/run_A2 \
  2>&1 | tee $OUT/run_A2/glim.log
src/glim_ext/modules/mapping/rtk_global/tools/check_rtk_global_log.sh $OUT/run_A2/glim.log
python3 install/gnss_core/lib/gnss_core/traj_rmse.py \
  $SEGA/gt/gt_cgi610_imu_flu.tum $OUT/run_A2/traj_imu.txt --align --tol 0.05 \
  | tee $OUT/eval/A2.md
```

Expected: `check_rtk_global_log.sh` 仍八行 ok；**A2 的 RMSE 应不差于 A**。A2 明显变差（> A 的 1.5 倍）说明杆臂填错了或 floor 降得过头——回到 Step 1 复查，别直接接受。

- [ ] **Step 4: 跑 `calibrate_sigma_scale`（§9.2）**

```bash
cd /home/steve/glim_ws && source install/setup.bash
SEGA=/home/steve/Documents/Datasets/tage/hongshaquan/20260915/seg_164931_165748
OUT=$SEGA/validation_20261007
install/gnss_core/lib/gnss_core/calibrate_sigma_scale \
  $SEGA/gnss/rtk_check.pos \
  $SEGA/integration_20260916/run_B/pos/20260915/rtkrcv.pos \
  0.1 1 | tee $OUT/eval/sigma_scale.txt
# 段 B(后处理解自比,只为看浮动档有没有样本;ref 与 test 同源,结论仅供参考)
SEGB=/home/steve/Documents/Datasets/tage/hongshaquan/20260915/seg_153000_153351
install/gnss_core/lib/gnss_core/calibrate_sigma_scale \
  $SEGB/gnss/rtk_check.pos $SEGB/gnss/rtk_check.pos 0.1 1 \
  | tee $OUT/eval/sigma_scale_segB_selfcheck.txt
```

Expected（段 A，已预跑过一次）：

```
quality      n  rmse_h(m)  rmse_v(m)  ratio_median  ratio_rms  ratio_mean
FIXED      486      0.003      0.009         0.430      0.614       0.492
note: FIXED tier raw median ratio = 0.430
```

- [ ] **Step 5: 判断能否写进 `quality_sigma_scale`，并记录理由**

按 `tools/README.md` §3 的两条规则判：
1. `suggested quality_sigma_scale` 已归一到 FIXED=1，**只有 FIXED 一档有样本时这张表给不出任何有效的分档信息** → `quality_sigma_scale` **保持默认 `[0.0, 50.0, 20.0, 5.0, 1.0]` 不动**，在 `tage_at128/config_rtk_global.json` 里加一行注释说明为什么。
2. FIXED 档的**原始**中位数比值 0.430 明显偏离良态期望 0.83（README §2「读数注意」：σ 完全可信时 `err_h/σ_h` 服从 Rayleigh，中位数 ≈ 0.83），说明**板卡 σ 整体偏悲观约 2 倍**。按 README §3 第 1 条，这种情况应"同时把 `sigma_floor` 或全局 σ 放大该倍数"——**但这里是偏悲观（实际误差小于报告的 σ），方向是缩小而不是放大**，而且 ref 与 test 同为 RTKLIB 同一引擎、只差实时/后处理（现场文档 §1.2 明说"不说明绝对精度"）。**结论：不据此改 σ，只记录。** 把这段推理原文写进结果文档 §6 ——它是下一段含遮挡数据回来后的判据。

- [ ] **Step 6: 更新 `tools/README.md`**

- 表格加一行：`| pos_to_enu.cpp | pos_to_enu <in.pos> <out.enu> [--origin lat lon alt] [--min-quality 3]:.pos → 局部 ENU 轨迹文本,给上游 gnss_global 当输入;spec §12.4,轮 5 |`
- §2 末尾"**待现场 ref/test 成对数据回来后重跑，并把结果贴到本节下方替换合成示例。**"那句之后，补一节 `### 现场实测(2026-09-15 红沙泉 seg_164931_165748)`，贴 Step 4 的实际输出，并把 Step 5 的两条判断原文写上。
- §3 加一段：说明本次为什么没改 `quality_sigma_scale`（只有 FIXED 一档）。
- §6 现场待办：
  - 第 3 条（跑 `calibrate_sigma_scale` 并替换 §2 合成示例）改 `[x]`
  - 第 4 条（结果写入 config 的 `quality_sigma_scale`）**保持 `[ ]`**，后面加 `—— 轮 5 实测只拿到 FIXED 一档,FLOAT/DGPS/SINGLE 无样本,需含遮挡的数据段(建议 mdc-20260915-153000)`
  - 第 5 条（Orin 上首次运行 `pos_to_rtkfix_bag.py` + `run_injection_suite.sh`）在 Task 6 之后改 `[x]`
  - 第 6 条（`estimate_lever_arm` 结果填 config、`sigma_floor` 降到 `[0.05,0.05,0.1]`）改 `[x]`，附实测值

- [ ] **Step 7: 提交（两个仓库）**

```bash
cd /home/steve/glim_ws/src/glim_ext
git add gnss_core/tools/README.md config/config_rtk_global.json
git commit -F - <<'EOF'
gnss_core: record the first real-data sigma calibration; point config at it

spec §9.2 的实测:段 A(红沙泉 2026-09-15 seg_164931_165748),ref 为 rnx2rtkp 后处理
rtk_check.pos、test 为现场联调 run_B 的 rtkrcv.pos。

  FIXED n=486  rmse_h 0.003 m  rmse_v 0.009 m  ratio_median 0.430  ratio_rms 0.614

两条结论都写进 README §2/§3:
- 只有 FIXED 一档有样本(98.4% 固定),分档表给不出有效信息 → quality_sigma_scale
  的默认值不动,现场待办第 4 条保持未勾,并写明需要含遮挡的数据段。
- FIXED 原始中位比值 0.430 vs 良态期望 0.83(Rayleigh 中位数) → 板卡 σ 偏悲观约 2 倍,
  方向是缩小而不是 README §3 写的放大;且 ref/test 同为 RTKLIB 同一引擎,
  不说明绝对精度。结论:只记录,不据此改 σ。

glim_ext/config/config_rtk_global.json 只改注释(指向结果文档),默认值一字未动。

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>
EOF

cd /home/steve/glim_ws/src/glim_underground
git add config/tage_at128/config_rtk_global.json config/tage_at128/config_rtk_odometry.json \
        docs/gnss/field/2026-10-07-round5-validation-closure.md
git commit -F - <<'EOF'
config(tage_at128): fill T_imu_gnss / time_offset from §9.3, lower sigma_floor

spec §9.3 的实测(estimate_lever_arm,run_A 的 traj_imu.txt + run_B 的 rtkrcv.pos):
  lever_imu   = <实测值>
  time_offset = <实测值>
  rms_residual= <实测值>,n_pairs=<实测值>,yaw range=<实测值>

设计值(README_LIO.md §3 的 gnss_antenna − cgi610_imu_flu)是 [1.600, 0.850, 2.880]。
水平两维的实测同时是对 extrinsics_tage_truck.yaml 里标注"未验证"的
T_base_link_cgi610_imu=(6.85,1.00,2.28) 的检验;<平地绕圈 → lever.z 不可观时写明
z 用的是设计值>。

杆臂填进来后按 spec §7.3 v2 与 round1 plan 的 V8 把 sigma_floor 从 [1,1,1]
降到 [0.05,0.05,0.1],复跑一遍(run_A2):check_rtk_global_log.sh 仍八行 ok,
RMSE <A2 vs A 的对比>。两次 run 都留档在数据段的 validation_20261007/。

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>
EOF
```

---

### Task 6: §12.3 —— 合成注入套件首次运行

**Files:**
- Modify: `src/glim_ext/gnss_core/tools/run_injection_suite.sh`
- Modify: `src/glim_ext/gnss_core/tools/README.md`（§5 贴 `summary.txt`、§6 第 5 条勾选）
- Modify: `src/glim_underground/docs/gnss/field/2026-10-07-round5-validation-closure.md`（加 §7）

**Interfaces:**
- Consumes: Task 4 的 `glim_config_patch.py`；Task 1 的 `config/tage_at128/`；Task 3 的 `pos_to_rtkfix_bag.py`
- Produces: `<segB>/validation_20261007/injection/summary.txt` 与各用例的 `traj_*.txt`、`glim.log`

- [ ] **Step 1: 改脚本——换掉会崩的 `make_config`**

把 `make_config()` 整个函数体换成调用 `glim_config_patch.py`：

```bash
# make_config <name> <robust_kernel> <lever_x> <lever_y> <lever_z> <enable_rtk 0|1> → 输出 config 目录
# 原实现用 python3 json.load 读 GLIM 的 config_ros.json —— 那是 JSON-with-comments,
# 必崩(这个脚本从未被运行过,所以这个 bug 一直留着)。现在交给带测试的
# glim_config_patch.py(轮 5 Task 4)。
make_config() {
  local name=$1 kernel=$2 lx=$3 ly=$4 lz=$5 enable=$6
  local dir="$WORK/config_$name"
  local modules=""
  local -a rtk_args=()
  if [[ "$enable" == "1" ]]; then
    modules="librtk_global.so"
    rtk_args=(--rtk-global "robust_kernel=\"$kernel\"" "T_imu_gnss=[$lx,$ly,$lz]"
              "rtk_fix_topic=\"$TOPIC\"" 'stamp_source="gnss_time"' "time_offset=0.0")
  fi
  "$PATCH" "$GLIM_CONFIG_SRC" "$dir" --dump-path "$WORK/dump_$name" \
           --modules "$modules" "${rtk_args[@]}" >/dev/null
  echo "$dir"
}
```

并在工具定位那段加：

```bash
PATCH=""
for p in "$(command -v glim_config_patch.py || true)" \
         "${GLIM_WS}/install/gnss_core/lib/gnss_core/glim_config_patch.py" \
         "${HERE}/glim_config_patch.py"; do
  [[ -n "$p" && -r "$p" ]] && PATCH="python3 $p" && break
done
[[ -n "$PATCH" ]] || { echo "glim_config_patch.py not found"; exit 1; }
```

- [ ] **Step 2: 改脚本——修三处写死路径与一处参数名**

1. `GLIM_CONFIG_SRC=${GLIM_WS}/src/glim/config` → `GLIM_CONFIG_SRC=${GLIM_WS}/src/glim_underground/config/tage_at128`，注释里的 `### CHECK` 换成说明"默认是踏歌宽体车配置集；别的车用 `--glim-config` 覆盖"。
2. 原 `make_config` 里 `src = os.path.expanduser("~/glim_ws/src/glim_ext/config/config_rtk_global.json")` 的回退逻辑随函数替换一起消失——新逻辑要求配置集**自带** `config_rtk_global.json`，缺了 `glim_config_patch.py` 会抛 `FileNotFoundError`，这是想要的行为（静默回退到别处的配置会让"这次跑的到底是哪份参数"说不清）。在脚本顶部的前提说明里写明这条要求。
3. `run_glim()` 的 `### CHECK: 参数名` —— 已核对 `glim_rosbag` 的 ROS 参数就是 `config_path`、`auto_quit`、`dump_path`（`src/glim_ros/glim_ros.cpp:70`、`src/glim_rosbag.cpp:434-441`），把 `### CHECK` 去掉并补一行源码位置。顺带把 `run_glim` 的 bag 参数从 `"$2"`（函数只收一个参数时是空的，等于把空串传给 `glim_rosbag`）修成 `local bag=$2`，调用处已经传了两个参数，但 `local dump; dump=$(...)` 后面用的是 `"$2"`，在 `set -u` 下调用 `run_glim "$cfg"`（Step 1 的参考运行只传一个）会直接退出——统一成显式两参数：

```bash
run_glim() {   # run_glim <config_dir> <bag_dir> → 轨迹文件路径
  local cfg=$1 bag=$2
  local dump; dump=$(python3 -c "import json,sys;sys.path.insert(0,'$(dirname "$PATCH_PY")');import glim_config_patch as m;print(m.load_jsonc('$cfg/config_ros.json')['glim_ros']['dump_path'])")
  rm -rf "$dump"; mkdir -p "$dump"
  ros2 run glim_ros glim_rosbag "$bag" --ros-args \
    -p config_path:="$cfg" -p auto_quit:=true -p dump_path:="$dump" >"$dump/glim.log" 2>&1
  local traj="$dump/traj_imu.txt"
  [[ -s "$traj" ]] || { echo "no traj at $traj (see $dump/glim.log)"; exit 1; }
  echo "$traj"
}
```

（`PATCH_PY` 是不带 `python3 ` 前缀的脚本路径变量，与 `PATCH` 一起设好。）

- [ ] **Step 3: 改脚本——每个用例跑完即删合成 bag**

把第 2 段（合成全部 5 个 bag）与第 3 段（跑 glim）合成一个按用例的循环：

```bash
# ---------- 2+3. 逐用例:合成 .pos → 合并 bag → 跑 glim × {huber,none} → 删 bag ----------
# 原来是先造 5 个 bag 再跑;每个合并 bag 和源 bag 一样大(段 B 约 4 GB),
# 逐个造完即删把峰值占用从 5 份降到 1 份,也便于中断后续跑。--keep-bags 保留。
KEEP_BAGS=${KEEP_BAGS:-0}
for name in clean wrong_fix stale float lever; do
  "$SYNTH" traj_ref.txt "synth_$name.pos" ${SYNTH_ARGS[$name]} --truth "truth_$name.pos" --seed 42
  rm -rf "bag_$name"
  python3 "$POS2BAG" "synth_$name.pos" --out "bag_$name" --topic "$TOPIC" --merge "$BAG"
  case "$name" in
    lever) run_case lever huber 0 0 0; run_case lever huber 0 1 0 ;;
    *)     run_case "$name" huber 0 0 0; run_case "$name" none 0 0 0 ;;
  esac
  [[ "$KEEP_BAGS" == "1" ]] || rm -rf "bag_$name"
done
```

`run_case` 里 `run_glim "$cfg" "bag_$name"` 保持两参数调用；参考运行那处改成 `run_glim "$CFG_REF" "$BAG"`。

- [ ] **Step 4: 先用一个用例试跑，确认脚本能走通**

```bash
cd /home/steve/glim_ws && source install/setup.bash
SEGB=/home/steve/Documents/Datasets/tage/hongshaquan/20260915/seg_153000_153351
mkdir -p $SEGB/validation_20261007/injection
# 单独验一次 make_config + run_glim:参考(无 GNSS)那一跑
GLIM_WS=/home/steve/glim_ws TOPIC=/rtkrcv_node/rtk_fix \
  bash -x src/glim_ext/gnss_core/tools/run_injection_suite.sh \
  $SEGB/bag_lio $SEGB/validation_20261007/injection 2>&1 \
  | tee $SEGB/validation_20261007/injection/run.log | tail -40
```

**先只看 `== [1/4] reference run (no GNSS)` 这一段是否跑过去**（它等价于 Task 1 的 C 组，只是换成段 B）。跑不过去就按日志修，修完再放开整套。
`traj_ref.txt` 的位姿数与 `DUR`（应约 231 s）记进结果文档。

- [ ] **Step 5: 跑完整套**

同一条命令跑到底（11 次 glim run，段 B 231 s/4 GB，CPU 路径下预计 1–2 小时）。

Expected: `summary.txt` 生成，形如

```
injection     | huber                                    | none
--------------|------------------------------------------|------------------------------------------
clean         | rmse=0.0xx n_rejected=0                  | rmse=0.0xx n_rejected=0
wrong_fix     | rmse=0.0xx n_rejected=NN                 | rmse=0.xxx n_rejected=0
stale         | rmse=0.0xx n_rejected=NN                 | rmse=0.0xx ...
float         | rmse=0.0xx ...                           | rmse=0.0xx ...
lever T=0     | rmse=0.xxx ...                           | -
lever T=010   | rmse=0.0xx ...                           | -
```

- [ ] **Step 6: 把结果逐条对照 spec §12.3 的四条期望（本 Task 的验收断言）**

在结果文档 §7 做一张四行表，每行写「spec 期望 / 实测 / 结论（符合 / 不符合 / 无法判定）」：

| 注入 | spec 期望（逐字） | 判据 |
|---|---|---|
| `wrong_fix` | 开鲁棒核：轨迹 RMSE 基本不变；关鲁棒核：明显恶化 | `rmse(wrong_fix,huber) ≈ rmse(clean,huber)`（相对差 < 20%）且 `rmse(wrong_fix,none) > 1.5 × rmse(clean,none)` |
| `stale` | 该段不产生因子 | `stale` 用例的 `glim.log` 里 `rtk_global stats` 的 `gated` 计数 > 0，且 `factors_added` 明显少于 `clean` |
| `float` | 因子仍加但权重降 5 倍 | `float` 的 `factors_added` 与 `clean` 同量级（不是 0）；权重 5 倍来自 `quality_sigma_scale[FLOAT]=5.0`，在日志里看不到，改为核对 `rmse(float) ≥ rmse(clean)` 且两者都远小于 `wrong_fix,none` |
| `lever` | 不设杆臂时先验有系统性偏移，设置后消失 | `rmse(lever, T=[0,1,0]) < rmse(lever, T=0)`，且差值在转弯段更明显（用 `traj_rmse.py --relative-time --from/--to` 卡一个转弯窗口单独算） |

**任何一条不符合，都要在结果文档里写明实测数字与可能原因，不要为了让表好看去调参数。** `stale`/`float` 两条的判据依赖 stats 行，若 `rtk_global` 没打出足够信息，就在文档里记"当前日志不足以判定"，并作为后续改进项列出（不在本轮改模块代码）。

- [ ] **Step 7: 更新 README 并提交**

`tools/README.md` §5 末尾贴 `summary.txt` 全文 + §6 第 5 条改 `[x]`。

```bash
cd /home/steve/glim_ws/src/glim_ext
git add gnss_core/tools/run_injection_suite.sh gnss_core/tools/README.md
git commit -F - <<'EOF'
gnss_core: make run_injection_suite.sh actually runnable, and run it (§12.3)

这个脚本自 2026-09-03 写好起从未被执行过,四处问题都是"没跑过"才留得住的:
1. make_config 用 json.load 读 GLIM 的 config_ros.json —— JSON-with-comments,必崩。
   换成带测试的 glim_config_patch.py(Task 4)。
2. GLIM_CONFIG_SRC 指向不存在的 ${GLIM_WS}/src/glim/config(本仓是
   src/glim_underground/config),默认改为 tage_at128 配置集。
3. run_glim 的 bag 参数用 "$2" 而函数只声明收一个 —— set -u 下参考那一跑直接退出。
   改成显式 run_glim <config_dir> <bag_dir>。
4. 先造 5 个合并 bag 再跑,峰值占用是源 bag 的 5 倍。改成逐用例造完即删(KEEP_BAGS=1 保留)。
另:确认 glim_rosbag 的参数名就是 config_path / auto_quit / dump_path
(glim_ros.cpp:70、glim_rosbag.cpp:434-441),去掉那处 ### CHECK 并标注源码位置。

首次运行结果(段 B seg_153000_153351,231 s,航向累计 1731°,CPU 路径,11 次 run):
summary.txt 全文贴在 tools/README.md §5,逐条对照 spec §12.3 四条期望的判定表在
docs/gnss/field/2026-10-07-round5-validation-closure.md §7。

RED:脚本改前跑参考那一跑即 JSONDecodeError(make_config)——输出见结果文档 §7 开头。
本次改动没有新增单元测试:改的是编排脚本,它依赖 glim_rosbag 与 10 GB 级 bag,
行为由 Task 4 的 glim_config_patch.py 测试 + 这次真实运行共同覆盖。

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>
EOF
```

---

### Task 7: §12.4 实车 A/B、轮 1.5 启用门、文档回填

**Files:**
- Modify: `src/glim_underground/config/tage_at128/config_ros.json`（轮 1.5 开门时加 `librtk_odometry.so` —— 见 Step 5 的条件）
- Modify: `src/glim_underground/docs/gnss/field/2026-10-07-round5-validation-closure.md`（加 §8 §12.4、§9 轮 1.5、§10 结论与下一步）
- Modify: `src/glim_underground/docs/gnss/specs/2026-09-03-gnss-glim-modules-design.md`（§13 风险表两行）
- Modify: `src/glim_underground/docs/gnss/plans/2026-09-03-round1-rtk-global.md`（V4–V10 勾选）
- Modify: `src/glim_underground/docs/gnss/field/2026-09-16-hongshaquan-seg164931-integration.md`（§7 问题 11 回填「已修复 `8fefcdc`」）

**Interfaces:**
- Consumes: Task 1 的 run_C、Task 4 的 run_A、Task 5 的 run_A2 与配置、Task 2/3 的 `.enu` 与合并 bag
- Produces: `<segA>/validation_20261007/run_B/`、`<segB>/validation_20261007/`（A/B/C 三组）、`eval/ab_segA.md`、`eval/ab_segB.md`、`run_A_odom/`

- [ ] **Step 1: 跑段 A 的 B 组（`gnss_global`）**

```bash
cd /home/steve/glim_ws && source install/setup.bash
SEGA=/home/steve/Documents/Datasets/tage/hongshaquan/20260915/seg_164931_165748
OUT=$SEGA/validation_20261007
mkdir -p $OUT/run_B
python3 install/gnss_core/lib/gnss_core/glim_config_patch.py \
  src/glim_underground/config/tage_at128 $OUT/config/run_B \
  --dump-path $OUT/run_B --modules libgnss_global.so
ros2 run glim_ros glim_rosbag $OUT/gnss/bag_lio_gnss \
  --ros-args -p config_path:=$OUT/config/run_B -p auto_quit:=true -p dump_path:=$OUT/run_B \
  2>&1 | tee $OUT/run_B/glim.log
grep -E "initializing GNSS global|starting GNSS global thread|insert .* GNSS prior|invalid condition" \
  $OUT/run_B/glim.log | head -20
```

Expected: 日志里有 `initializing GNSS global constraints`、`gnss_global_config_path=.../config_gnss_global.json`、`starting GNSS global thread`。**`gnss_global` 的订阅来自 `create_subscriptions()`，`glim_rosbag` 只有加载了它才会读 `/gnss/pose`** —— 没看到初始化行就说明 `config.json` 里 `config_gnss_global` 的键没生效（`GlobalConfigExt::get_config_path` 先查主 config 的 `global` 块，`config_ext.hpp:36-46`）。
注意 `gnss_global` 的"插入了多少先验因子"只在 debug 级别打（`logger->debug("insert {} GNSS prior factors", ...)`），要数因子就加 `-p debug:=true`（`glim_ros.cpp:48` 会把 trace 级别打开并写 `/tmp/glim_log.log`）。**B 组要加 `-p debug:=true` 重跑一次或直接带上**，否则没法和 A 组的 `factors_added` 比。

- [ ] **Step 2: 段 A 的 A/B/C 对比表**

```bash
cd /home/steve/glim_ws && source install/setup.bash
SEGA=/home/steve/Documents/Datasets/tage/hongshaquan/20260915/seg_164931_165748
OUT=$SEGA/validation_20261007
for r in run_C run_A run_A2 run_B; do
  echo "=== $r ==="
  python3 install/gnss_core/lib/gnss_core/traj_rmse.py \
    $SEGA/gt/gt_cgi610_imu_flu.tum $OUT/$r/traj_imu.txt --align --tol 0.05
done | tee $OUT/eval/ab_segA.md
```

- [ ] **Step 3: 段 B 跑同样的三组**

段 B 没有实时解，用后处理 `rtk_check.pos`（含 14/231 浮动历元，这是 spec §12.4 "含浮动解路段"前提唯一能部分满足的地方）：

```bash
cd /home/steve/glim_ws && source install/setup.bash
SEGB=/home/steve/Documents/Datasets/tage/hongshaquan/20260915/seg_153000_153351
OUT=$SEGB/validation_20261007
mkdir -p $OUT/gnss $OUT/config $OUT/run_C $OUT/run_A $OUT/run_B $OUT/eval
python3 install/gnss_core/lib/gnss_core/pos_to_rtkfix_bag.py \
  $SEGB/gnss/rtk_check.pos --out $OUT/gnss/bag_lio_gnss \
  --topic /rtkrcv_node/rtk_fix \
  --pose-enu $OUT/gnss/rtk_check.enu --pose-topic /gnss/pose \
  --merge $SEGB/bag_lio
ros2 bag info $OUT/gnss/bag_lio_gnss | tee $OUT/gnss/bag_lio_gnss.info

for case in C:"" A:librtk_global.so B:libgnss_global.so; do
  name=${case%%:*}; mods=${case#*:}
  python3 install/gnss_core/lib/gnss_core/glim_config_patch.py \
    src/glim_underground/config/tage_at128 $OUT/config/run_$name \
    --dump-path $OUT/run_$name --modules "$mods"
  bag=$OUT/gnss/bag_lio_gnss; [[ "$name" == "C" ]] && bag=$SEGB/bag_lio
  ros2 run glim_ros glim_rosbag $bag --ros-args \
    -p config_path:=$OUT/config/run_$name -p auto_quit:=true \
    -p dump_path:=$OUT/run_$name -p debug:=true 2>&1 | tee $OUT/run_$name/glim.log
done

for r in run_C run_A run_B; do
  echo "=== $r ==="
  python3 install/gnss_core/lib/gnss_core/traj_rmse.py \
    $SEGB/gt/gt_cgi610_imu_flu.tum $OUT/$r/traj_imu.txt --align --tol 0.05
done | tee $OUT/eval/ab_segB.md
```

- [ ] **Step 4: 按 spec §12.4 验收标准下结论**

在结果文档 §8 写三件事，一件都不能省：

1. **数字**：两段各三组（段 A 四组，含 A2）的 `n_pairs` 与 `rmse`，A 组与 B 组的因子数（A 看 `rtk_global stats` 的 `factors_added`，B 看 debug 日志里 `insert N GNSS prior factors` 的累加）。
2. **逐条对标 spec §12.4 验收标准**（逐字：「在含浮动解或差分中断的路段，`rtk_global` 的轨迹 RMSE 应优于 `gnss_global`，且不出现被单个坏历元拽偏的情形」）：
   - 「优于」：给出两段的 RMSE 对比。
   - 「不被单个坏历元拽偏」：看 A 组轨迹相对 gt 的**最大**单点误差（`traj_rmse.py` 的输出里有则直接用；没有就用 `--relative-time` 扫窗口找最差段），与 B 组对比。
   - **「含浮动解或差分中断的路段」这个前提**：段 A 98.4% 固定、无差分中断，**不满足**；段 B 有 6.1% 浮动但没有差分中断，**部分满足**。所以结论必须写成"在当前可得的两段数据上 X"，而不是"§12.4 通过"。
3. **明确写出还缺什么**：一段含遮挡/出入隧道/长差分中断的数据（现场文档 §9 建议 6：≥600 s、含遮挡，候选 `mdc-20260915-153000` 整段）。这是 §12.4 正式验收的唯一剩余阻塞。

- [ ] **Step 5: 轮 1.5 的启用门**

**只有当 Step 4 的结论是"A 不劣于 B，且 §12.2 八项全 ok"时才做这一步**；否则跳过，并在结果文档 §9 写明门未开的原因（spec §10 的门就是"待 `rtk_global` 实车验证通过"）。

```bash
cd /home/steve/glim_ws && source install/setup.bash
SEGA=/home/steve/Documents/Datasets/tage/hongshaquan/20260915/seg_164931_165748
OUT=$SEGA/validation_20261007
mkdir -p $OUT/run_A_odom
python3 install/gnss_core/lib/gnss_core/glim_config_patch.py \
  src/glim_underground/config/tage_at128 $OUT/config/run_A_odom \
  --dump-path $OUT/run_A_odom --modules librtk_global.so,librtk_odometry.so
ros2 run glim_ros glim_rosbag $OUT/gnss/bag_lio_gnss \
  --ros-args -p config_path:=$OUT/config/run_A_odom -p auto_quit:=true \
  -p dump_path:=$OUT/run_A_odom 2>&1 | tee $OUT/run_A_odom/glim.log
grep -E "initializing rtk_odometry|rtk_odometry stats:" $OUT/run_A_odom/glim.log
python3 install/gnss_core/lib/gnss_core/traj_rmse.py \
  $SEGA/gt/gt_cgi610_imu_flu.tum $OUT/run_A_odom/traj_imu.txt --align --tol 0.05 \
  | tee $OUT/eval/A_odom.md
```

Expected: 日志里 `rtk_odometry stats:` 一行且其中 `factors_added` > 0；RMSE 不差于 run_A2。
门开成功后，把 `tage_at128/config_ros.json` 的 `extension_modules` 从 `[]` 改成

```json
    "extension_modules": [
      "librtk_global.so",
      "librtk_odometry.so"
    ],
```

并把那段"故意留空"的注释改成说明"轮 5 §12.4 通过后打开；离线批跑仍由 `glim_config_patch.py` 按用例覆盖"。
`rtk_odometry` 的 `smoother_lag_margin` 默认 1.0 s，与 `config_odometry_cpu.json` 的 `smoother_lag` 配合；日志里若出现大量"key already marginalized"类告警，记进文档作为后续项，不在本轮改代码。

- [ ] **Step 6: 文档回填**

1. **spec §13 风险表**两行按实测更新（保留原文，追加「[v5] 2026-10-07 实测」）：
   - 「IMU-GNSS 杆臂未标定 [v2]」→ 填入 Task 5 的实测值、说明 z 维可观性、给出结果文档路径；若 x/y 与设计值吻合到 cm 级，这一项可标为**关闭**（并写明 z 用的是设计值）。
   - 「RTK 与 LiDAR 时间戳不同源 [v2]」→ 填入 `time_offset` 实测值与可观性结论。
   - 「踏歌现场无同步 LiDAR+RTK 录包」→ 标为**关闭**（2026-09-15 两段数据已有同步 LiDAR+IMU+RTK 与 INS 真值）。
2. **round1 plan 的 V4–V10**（`2026-09-03-round1-rtk-global.md` 第 1980–1988 行附近）逐条勾选或写明未达原因：
   - V5（`glim_rosbag` + `check_rtk_global_log.sh`）→ `[x]`，附 Task 4 的八行输出位置
   - V6（实车 `.pos` + `calibrate_sigma_scale` 分档表）→ `[x]` 但注明只有 FIXED 一档
   - V7（合成注入全套）→ `[x]`，附 `summary.txt` 位置
   - V8（`estimate_lever_arm` 填 config、`sigma_floor` 降到 `[0.05,0.05,0.1]`）→ `[x]`
   - V9（打开 `librtk_odometry.so`）→ 按 Step 5 结果勾或不勾
   - V10（§12.4 A/B）→ **不勾**，写明"前提『含浮动解或差分中断路段』未满足，需另一段数据"，并指向结果文档 §8
   - V4（有 CAN 的 bag 回放核对 quality 变化）→ **不勾**，原因是 CAN σ 帧缺失（现场文档 §7 问题 1），本轮未涉及
   - aarch64/Orin 上 V1–V3 的复核 → 仍未做，保持原状
3. **现场文档回填**：`2026-09-16-hongshaquan-seg164931-integration.md` §7 问题 11（`read_pos` 不认"周+周内秒"）标「已修复 `8fefcdc`（PR #8），轮 5 Task 2 在真实 `rtk_check.pos` 上验证读出 498 条」。

- [ ] **Step 7: 写结果文档 §10 结论与下一步，提交**

§10 必须包含：
- 本轮关闭了什么（逐条对 spec 条目：§12.2 / §12.3 / §9.2 部分 / §9.3 / §12.4 部分 / 轮 1.5 门）
- 还开着什么，每条给最小下一步：§12.4 的正式验收（需含遮挡数据段）、`quality_sigma_scale` 的 FLOAT/DGPS/SINGLE 三档、E1–E8 与 `gnss_diag` 模块壳（轮 4b，完全未开始）、现场五条阻断（CAN σ 帧、GPS 星历、`device_divergence` 同点比较、`ambiguity` 死规则、`base_shift` 基线）、aarch64/Orin 复核
- 一句话给维护者的判断：`rtk_global` 现在是"可加载、可产因子、在好解数据上不劣化、杆臂已实测"的状态；"优于 `gnss_global`"这个命题在当前数据上只能部分支持。

```bash
cd /home/steve/glim_ws/src/glim_underground
git add docs/gnss/field/2026-10-07-round5-validation-closure.md \
        docs/gnss/field/2026-09-16-hongshaquan-seg164931-integration.md \
        docs/gnss/specs/2026-09-03-gnss-glim-modules-design.md \
        docs/gnss/plans/2026-09-03-round1-rtk-global.md \
        config/tage_at128/config_ros.json
git commit -F - <<'EOF'
docs(gnss): round 5 — close §12.2/§12.3/§9.3, partially close §9.2/§12.4

A/B/C 三组在 2026-09-15 红沙泉两段数据上跑完,数字与逐条对标见
docs/gnss/field/2026-10-07-round5-validation-closure.md。

关闭:
- §12.2 librtk_global.so 被 GLIM 加载并产出因子(check_rtk_global_log.sh 八项全 ok)
- §12.3 合成注入套件首次运行,四条期望的判定表
- §9.3 杆臂与时间偏移实测并写回 tage_at128 配置,sigma_floor 降到 [0.05,0.05,0.1]
- spec §13「踏歌现场无同步 LiDAR+RTK 录包」
部分关闭:
- §9.2 只拿到 FIXED 一档(两段固定率 98.4% / 93.9%),分档表仍缺 FLOAT/DGPS/SINGLE
- §12.4 A/B 跑通并有数字,但验收前提「含浮动解或差分中断的路段」两段都不满足
  (段 A 98.4% 固定无中断、段 B 6.1% 浮动无中断)→ round1 plan 的 V10 不勾
轮 1.5:<按实测结果写 门已开/门未开及原因>

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>
EOF
```

---

## 自查

**1. spec 覆盖**

| spec 条目 | 落在哪 |
|---|---|
| §9.2 系数标定（中位数口径、写回 config） | Task 5 Step 4–6（结论：只有 FIXED 一档，默认值不动并写明理由） |
| §9.3 杆臂与时间偏移（网格搜 Δt、可观性提示、填 config） | Task 5 Step 1–3 |
| §12.2 模块加载与订阅（短 bag 跑 `glim_rosbag`、断言初始化与 `T_world_enu=`、`RtkFix` 计数 > 0） | Task 4 Step 6–7 |
| §12.3 合成注入四用例（`wrong_fix`/`stale`/`float`/`lever` × huber/none） | Task 6 全部 |
| §12.4 实车 A/B（同一包挂 `libgnss_global.so` 与 `librtk_global.so`，比 RMSE） | Task 7 Step 1–4（含 C 组与两段数据；验收前提未满足已明示） |
| §10 轮 1.5 启用门 | Task 7 Step 5（带条件） |
| §13 风险表（杆臂、时间偏移、无同步录包） | Task 7 Step 6 |
| 轮 1 验收 V4–V10 | Task 7 Step 6（逐条勾选或写明未达原因） |
| §11 / E1–E8 / `gnss_diag` 模块壳（轮 4b） | **不在本计划范围**；Task 7 Step 7 把它列进"还开着什么" |
| §12.1 纯单元测试 | 既有，Task 1 Step 3 只做基线确认 |

**2. 占位符扫描** —— 计划里出现 `<实测值>` 的地方都在**提交 message 模板**里，是执行时必须填入真实数字的槽位，不是设计上的 TBD；每处都在相邻步骤给了产生该数字的命令与期望范围。Task 5 Step 2 的 `T_imu_gnss` 三个分量、Task 7 Step 5 的门开/未开，都给了明确判据。

**3. 类型一致性** —— `make_enu_track` / `write_enu_track` / `read_enu_track`（Task 2 的 Interfaces、测试、实现、`pos_to_enu.cpp`）签名一致；`.enu` 的 8 列格式在 Task 2 的 `write_enu_track` 与 Task 3 的 `parse_enu` 两侧一致（`t e n u sde sdn sdu q`，`q` 是归一化枚举 0–4）；`patch_config_set` 的关键字参数在 Task 4 的 Interfaces、测试、实现、Task 6 的 `make_config`、Task 7 的 CLI 调用里一致（`--modules` 空串 = 不加载任何模块）；`glim_rosbag` 的三个 ROS 参数名 `config_path` / `auto_quit` / `dump_path` 在 Task 1/4/5/6/7 里一致，且已核对源码。
