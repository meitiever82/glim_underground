# 轮 3b：诊断节点、清理节点与 rtkrcv 健康信号 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 把轮 3a 的 `gnss_core::DiagnosisEngine` 接进车上：`gnss_diag_node` 订阅差分流、两路 `RtkFix`、`$SAT` 流，写 `events.log` / `base.pos` 并发布 ROS 标准诊断；`gnss_cleanup_node` 按保留天数与磁盘水位清理录包与 `.pos`；`rtkrcv_node` 发布自身健康状态；同时补上 ratio 传递与 3a 遗留的偏差学习缺陷。

**Architecture:** 所有判定逻辑仍在 `gnss_core`（ROS 无关）；`gnss_bringup` 里新增的纯函数头文件负责 ROS 消息与 `gnss_core` 类型之间的转换、按天文件路径、时钟回跳检测与健康评估，节点本身只做参数、订阅、定时器与落盘接线。状态输出统一用 `diagnostic_msgs/DiagnosticArray`。节点级行为用"gtest 起子进程"的方式测试（沿用轮 2 的 `node_process_harness.hpp`）。

**Tech Stack:** C++17、ROS 2 Humble（rclcpp、diagnostic_msgs、std_srvs；测试用 rosgraph_msgs）、gnss_msgs、gnss_core、gtest。

**Spec:**
- `docs/gnss/specs/2026-09-03-gnss-glim-modules-design.md`（§3 A6/C1–C10/D2–D4、§5.2、§5.3 目录布局、§8 `gnss_diag_node`、§10 轮 3）
- `docs/gnss/specs/2026-09-15-round3a-followups.md`（A 节两项偏差学习决定、B 节 3b 接口事项——本计划逐条落实，见「设计决定」）
- 3a 已合并的 `gnss_core` 诊断部件：`glim_ext/gnss_core/include/gnss_core/{diagnosis,divergence_monitor,event_book,base_station_monitor,diag_io,retention,diagnosis_engine}.hpp`

## Global Constraints

- **代码仓库一**：`/home/steve/glim_ws/src/glim_ext`（git）。开工前 `git ls-remote origin` 确认 `master`（写作时为 `ed307f6`），从最新 `master` 开 `feat/round3b-diag-node`。以 `git ls-remote` 为准（仓库有外部 git 自动同步）。
- **代码仓库二**：`/home/steve/Documents/GitHub/ztpilot/finder_ros`（git，远端 `code.ztpilot.com`，当前分支 `ros2`）。`gnss_msgs` 实体在 `finder_ros/drivers/gnss_msgs`，`glim_ws/src/gnss_msgs` 与 `driver_ws/src/gnss_msgs` 都是指向它的符号链接。只在 Task 1 改动，从 `ros2` 开本地分支 `feat/rtkfix-ratio`，**只提交 `drivers/gnss_msgs/msg/RtkFix.msg`**；该仓库有维护者未跟踪的 `CLAUDE.md`，绝不暂存。
- **绝不 `git push`**（任何仓库）。
- 每个 commit message 以 `Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>` 结尾。
- `glim_underground` 里维护者未提交的工作（`include/glim/util/time_keeper.hpp`、`src/glim/util/time_keeper.cpp` 及其他未跟踪文件）绝不触碰、绝不暂存；在该仓库只 `git add` 明确的文档路径。
- `gnss_core` 保持不依赖 ROS / GLIM。
- `launch/gnss_bringup.launch.py` 的 `enable_rtkrcv` 默认值保持 `"false"`。
- 注释中文、与周边一致；标识符英文；诊断消息文本沿用 `gnss_core` 已有字符串。
- 测试一律 gtest（开发机 pytest 9.1.1 与 launch_testing 插件不兼容，禁止 `ament_add_pytest_test` / launch_testing）。
- **诚实测试**：每个新测试在实现前必须亲眼看到失败——对旧代码失败，或对桩实现失败，或对写明的变异失败（CMake "找不到源文件"不算行为 RED）；证据写进 commit message。
- 测试文件一律放 `mkdtemp` 临时目录（基于 `$TMPDIR`，缺省 `/tmp`），含失败路径在内删除；起 ROS 节点的测试给子进程独立 `ROS_DOMAIN_ID` 与指向临时目录的 `ROS_LOG_DIR`。
- 构建：`cd /home/steve/glim_ws && source /opt/ros/humble/setup.bash && colcon build --symlink-install --packages-up-to gnss_bringup`（Task 1 改了消息，首次需要 `--packages-select gnss_msgs gnss_core gnss_bringup` 全部重编）
- 单测：`cd /home/steve/glim_ws && source /opt/ros/humble/setup.bash && source install/setup.bash && ./build/<package>/<test> --gtest_filter='<Suite>.<Name>'`
- 全量：`cd /home/steve/glim_ws && source /opt/ros/humble/setup.bash && colcon test --packages-select gnss_core gnss_bringup && colcon test-result --all`
- 基线：`colcon test-result --all` = **477 tests, 0 errors, 0 failures, 1 skipped**。每个 Task 结束 0 failures。
- 已安装：RTKLIB-EX 2.5.1（`/usr/local/bin/rtkrcv`）、libfaketime、`diagnostic_msgs` / `std_srvs` / `rosgraph_msgs`（`/opt/ros/humble`）。
- 节点级测试的 `ROS_DOMAIN_ID` 按文件错开：健康 20–29、清理 30–39、既有 rtkrcv 40–89、诊断 90–99。

## 设计决定

维护者 2026-09-15 确认：

1. **ratio 走 `RtkFix`**：`gnss_msgs/RtkFix.msg` 末尾加 `float32 ratio`（0 = 源不提供）。`rtkrcv_node` 填 RTKLIB 的 AR ratio，610 驱动不改（默认 0）。**代价**：消息定义变了，`glim_ws` 与 `driver_ws` 都要重编；用旧定义录的含 `RtkFix` 的 bag，新代码按新类型反序列化会失败（CDR 末尾少 4 字节）——README 要写明。
2. **时钟用 ROS 时间 + 回跳检测**：`gnss_diag_node` 的引擎时间、`events.log` / `base.pos` 时间戳都用 `node->now()`（回放 bag 时配 `use_sim_time` 保持一致）；某次 tick 发现时间比上次回退超过 `clock_jump_tolerance_s`（1 s），就把已开事件以 `shutdown` 原因关闭并写日志，重建引擎（基线从文件重新读），重新开始启动宽限期。
3. **状态输出用 `diagnostic_msgs/DiagnosticArray`**：`gnss_diag_node` 发 `/gnss/diagnostics`，`rtkrcv_node` 发 `~/diagnostics`（即 `/rtkrcv_node/diagnostics`），不新增消息类型。
4. **偏差学习只用固定解，σ 设上限**：只有两路配对样本都是 `FIXED` 时才允许进入经验窗口（其余样本照常判定、不学习），解决浮点解拉宽阈值；学到的 σ（窗口 RMS 与持有基线）上限 `divergence_sigma_max_m = 0.10`，阈值中来自学习的部分最多 0.30 m，更大的持续偏差不会被学成正常。rtkrcv 当前 σ 仍然无上限地抬高阈值。

控制者裁定（本计划内，代价可控）：

5. **启动宽限期** `startup_grace_s = 60`：节点起来（或时钟回跳重建引擎）后前 60 s 只接收数据、不调用 `tick`，避免 rtkrcv 收敛前每次开机都记一条 `no_solution`。代价：宽限期内真实的差分中断等故障晚 60 s 才记录。
6. **基线文件** 放 `<root>/base_baseline`；**`last_history`** 启动时从 `<root>` 下日期最新、含有效数据行的 `YYYYMMDD/base.pos` 取最后一行，找不到就为空（首次运行多写一行历史，无害）。
7. **控制点参数** 用三个等长数组 `control_points.names` / `.lat` / `.lon`（ROS 参数不支持结构体数组）。
8. **清理节点** 用两个根目录参数 `bag_root`（`gnss_YYYYMMDD_HHMMSS`）与 `pos_root`（`YYYYMMDD`），可各自为空但不能都空；先扫录包根目录再扫 `.pos` 根目录（录包占盘大，水位共享）；定时器用墙钟（删数据按真实日期，不跟随 sim time）；启动时先跑一次，之后每 `interval_s`（3600 s）一次；每轮结束若任一根目录所在盘仍高于水位，打 WARN。`launch` 默认启用，`bag_root` 由 launch 注入（`$GNSS_BAG_ROOT`，未设置时 `$HOME/gnss_bags`，与 `record_gnss.sh` 默认一致），`.pos` 根目录取 yaml。根目录不存在（还没录过包）视为跳过，不算错误。
9. **`rtkrcv_node` 健康**：1 Hz 发布；`ERROR` = 子进程未在运行；`WARN` = 超过 `health_no_solution_warn_s`（30 s）没有解算行（消息区分"上行无数据"与"上行有数据但无解——检查 base_pos_type / obs_format"）；否则 `OK`。
10. **`DiagnosisEngine` 增加 `open_event_codes()`**（3b 发布诊断时要列出当前已开事件）。
11. **基站基线重置做成 `std_srvs/Trigger` 服务** `/gnss_diag/reset_base_baseline`（3a 遗留 B 要求暴露给运维），新增 `std_srvs` 依赖，不新增服务类型。
12. **配对持续失败打 WARN**：两路解都在按时到达、却连续 `unpaired_warn_s`（60 s）没配上对时打一次 WARN（3a 遗留 B：历元始终差 > 0.1 s 时 `device_divergence` 静默失效）。
13. **`solver_enabled` 由 launch 按 `enable_rtkrcv` 注入**：rtkrcv 没启用时 `no_solution` 记为 info、不开事件，yaml 里不重复写。

## File Structure

`finder_ros`（Task 1）
- Modify `drivers/gnss_msgs/msg/RtkFix.msg` — 末尾加 `ratio`

`glim_ext/gnss_core`（Task 2、Task 6 的一个小函数）
- Modify `include/gnss_core/diagnosis.hpp`、`src/diagnosis.cpp` — `divergence_sigma_max_m` 配置与校验
- Modify `include/gnss_core/divergence_monitor.hpp`、`src/divergence_monitor.cpp` — `learnable` 参数、σ 上限、注释修正
- Modify `include/gnss_core/diagnosis_engine.hpp`、`src/diagnosis_engine.cpp` — 传 `learnable`、`open_event_codes()`
- Modify `include/gnss_core/retention.hpp`、`src/retention.cpp` — `disk_used_pct()`（Task 6）
- Test `test/test_divergence_monitor.cpp`、`test/test_diagnosis_engine.cpp`、`test/test_diagnosis.cpp`、`test/test_retention.cpp`

`glim_ext/gnss_bringup`
- Modify `include/gnss_bringup/rtk_fix_mapping.hpp`、`test/test_rtk_fix_mapping.cpp` — ratio 双向映射（Task 1）
- Modify `include/gnss_bringup/pos_rotation.hpp`、`test/test_pos_rotation.cpp` — `day_file_path()`，`pos_path_for` 改为委托（Task 3）
- Create `include/gnss_bringup/diag_node_support.hpp`、`test/test_diag_node_support.cpp` — 消息转换、诊断状态、按天追加写、历史读取、时钟回跳、控制点（Task 3）
- Create `src/gnss_diag_node.cpp`、`test/test_gnss_diag_node_process.cpp` — 诊断节点与节点级测试（Task 4）
- Create `include/gnss_bringup/rtkrcv_health.hpp`、`test/test_rtkrcv_health.cpp`、`test/test_rtkrcv_node_health_process.cpp`；Modify `src/rtkrcv_node.cpp`、`include/gnss_bringup/process_supervisor.hpp`（`child_running()`）、`test/test_process_supervisor.cpp`、`config/gnss_bringup.yaml`（一项参数）— 健康信号（Task 5）
- Create `include/gnss_bringup/cleanup_params.hpp`、`src/gnss_cleanup_node.cpp`、`test/test_cleanup_params.cpp`、`test/test_gnss_cleanup_node_process.cpp` — 清理节点（Task 6）
- Modify `CMakeLists.txt`、`package.xml`（`diagnostic_msgs` 依赖 Task 3 起；`std_srvs` 与测试依赖 `rosgraph_msgs` Task 4 起）
- Modify `launch/gnss_bringup.launch.py`、`config/gnss_bringup.yaml`、`README.md`、`scripts/record_gnss.sh`（文件头注释）（Task 7）

`glim_underground`（Task 7）
- Modify `docs/gnss/specs/2026-09-15-round3a-followups.md` — 处理状态

依赖顺序：1 → 2 → 3 → 4 → 5 → 6 → 7。

---
### Task 1: `RtkFix` 增加 ratio，并在 `gnss_bringup` 双向映射

**Files:**
- Modify: `/home/steve/Documents/GitHub/ztpilot/finder_ros/drivers/gnss_msgs/msg/RtkFix.msg`（finder_ros 仓库，分支 `feat/rtkfix-ratio`）
- Modify: `glim_ext/gnss_bringup/include/gnss_bringup/rtk_fix_mapping.hpp`（`to_rtk_fix`、`to_pos_record` 及其上方注释）
- Test: `glim_ext/gnss_bringup/test/test_rtk_fix_mapping.cpp`

**Interfaces:**
- Produces: `gnss_msgs::msg::RtkFix::ratio`（`float`，0 表示源不提供）；`to_rtk_fix(r).ratio == r.ratio`；`to_pos_record(m).ratio == m.ratio`

- [ ] **Step 1: 改消息定义**

在 finder_ros 仓库：

```bash
cd /home/steve/Documents/GitHub/ztpilot/finder_ros
git status --short           # 记录:应只有维护者的 ?? CLAUDE.md
git checkout -b feat/rtkfix-ratio
```

在 `drivers/gnss_msgs/msg/RtkFix.msg` 末尾（`bool heading_valid ...` 行之后）追加：

```
float32 ratio             # 模糊度固定检验 ratio（RTKLIB AR ratio）；0 = 源不提供（如 610 板卡）
```

- [ ] **Step 2: 写失败测试**

`glim_ext/gnss_bringup/test/test_rtk_fix_mapping.cpp`：删除 `TEST(ToPosRecord, RatioHasNoCounterpartInRtkFixSoItIsZero)`，追加：

```cpp
TEST(RtkFixMapping, RatioIsCarriedIntoRtkFix) {
  EXPECT_FLOAT_EQ(to_rtk_fix(sample()).ratio, 20.5f);
}

TEST(ToPosRecord, RatioIsCarriedBackFromRtkFix) {
  auto m = fix_sample();
  m.ratio = 7.25f;
  EXPECT_DOUBLE_EQ(to_pos_record(m).ratio, 7.25);
}

TEST(ToPosRecord, SourcesWithoutRatioStayZero) {
  // 610 板卡不提供 ratio,消息默认值 0 原样落进 .pos 的 ratio 列
  EXPECT_DOUBLE_EQ(to_pos_record(fix_sample()).ratio, 0.0);
}
```

并在 `TEST(ToPosRecord, RoundTripsThroughToRtkFix)` 里 `const auto original = fix_sample();` 改成可修改的 `auto original = fix_sample(); original.ratio = 12.5f;`，末尾追加 `EXPECT_FLOAT_EQ(back.ratio, original.ratio);`。

- [ ] **Step 3: 确认 RED**

Run: `cd /home/steve/glim_ws && source /opt/ros/humble/setup.bash && colcon build --symlink-install --packages-select gnss_msgs gnss_core gnss_bringup && source install/setup.bash && ./build/gnss_bringup/test_rtk_fix_mapping`
Expected: 编译通过（消息已有字段），`RatioIsCarriedIntoRtkFix`、`RatioIsCarriedBackFromRtkFix`、`RoundTripsThroughToRtkFix` FAIL（映射还没填 ratio）；`SourcesWithoutRatioStayZero` 在旧代码上会通过——为它记录变异：Step 4 之后把 `to_pos_record` 的 ratio 改成 `1.0`，它必须 FAIL。

- [ ] **Step 4: 实现**

`rtk_fix_mapping.hpp`：
- `to_rtk_fix` 在 `m.heading_valid = false;` 之后加 `m.ratio = static_cast<float>(r.ratio);   // RTKLIB AR ratio,供诊断的 ambiguity 规则使用`。
- `to_pos_record` 把 `r.ratio = 0.0;  // RtkFix 不携带 AR ratio,见上面注释` 改成 `r.ratio = m.ratio;   // 0 表示源不提供`。
- 删除 `to_pos_record` 上方"ratio(AR ratio)在 RtkFix 里没有对应字段……"那三行注释，改为一行：`// ratio 为 0 表示源不提供(RtkFix.msg 字段注释),原样写进 .pos。`

- [ ] **Step 5: GREEN + 变异 + 全量**

Run: `./build/gnss_bringup/test_rtk_fix_mapping`（PASS）；执行 Step 3 写明的变异并恢复；全量测试。
其他 `RtkFix` 用户（字段加在末尾、默认 0，源码不需要改，但都要重编）：
- `glim_ws` 里的 `glim_ext` 包（`modules/odometry/rtk_odometry`、`modules/mapping/rtk_global`）：`colcon build --symlink-install --packages-select glim_ext --cmake-args -DENABLE_GNSS=ON` 编译通过即可（若本机 `glim_ext` 构建依赖缺失，如实记录，不要为此改构建配置）。
- `driver_ws` 的 `gnss_CGI610`：**不要**在 `driver_ws` 里构建或改任何东西（维护者可能正在用它的 install）。只做静态检查：`grep -rn "== *m\b\|operator==\|RtkFix{" /home/steve/Documents/GitHub/ztpilot/finder_ros/drivers/gnss_CGI610` 确认没有整条消息比较/聚合初始化会受新字段影响，结果写进 commit message；`driver_ws` 的重编留给维护者（README 写明，见 Task 7）。

- [ ] **Step 6: 提交（两个仓库分别提交）**

```bash
cd /home/steve/Documents/GitHub/ztpilot/finder_ros
git add drivers/gnss_msgs/msg/RtkFix.msg
git status --short    # 暂存区只能有 RtkFix.msg
git commit -m "feat(gnss_msgs): carry the AR ratio in RtkFix

rtkrcv's solution stream has an ambiguity-resolution ratio that the GNSS
diagnostics (ambiguity rule) need; RtkFix dropped it. Appended as the last
field, 0 meaning the source does not provide it (the CGI-610 driver leaves it 0).
Bags recorded with the previous RtkFix definition will not deserialize with the
new type.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"

cd /home/steve/glim_ws/src/glim_ext
git add gnss_bringup/include/gnss_bringup/rtk_fix_mapping.hpp gnss_bringup/test/test_rtk_fix_mapping.cpp
git commit -m "feat(gnss_bringup): map the new RtkFix ratio in both directions

<RED 与变异结果>

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 2: 偏差学习只用固定解、学到的 σ 设上限；引擎暴露已开事件

落实设计决定 4、10，以及 3a 遗留 C 节"偏差监测"的注释修正与缺失测试。

**Files:**
- Modify: `glim_ext/gnss_core/include/gnss_core/diagnosis.hpp`（`DiagnosisConfig::divergence_sigma_max_m`）
- Modify: `glim_ext/gnss_core/src/diagnosis.cpp`（校验）
- Modify: `glim_ext/gnss_core/include/gnss_core/divergence_monitor.hpp`、`src/divergence_monitor.cpp`
- Modify: `glim_ext/gnss_core/include/gnss_core/diagnosis_engine.hpp`、`src/diagnosis_engine.cpp`
- Test: `glim_ext/gnss_core/test/test_divergence_monitor.cpp`、`test/test_diagnosis_engine.cpp`、`test/test_diagnosis.cpp`

**Interfaces:**
- Produces:
  - `DiagnosisConfig::divergence_sigma_max_m`（`double`，默认 `0.10`，必须 `>= divergence_sigma_floor_m`）
  - `DivergenceState DivergenceMonitor::update(double t, std::optional<double> divergence_m, double current_sigma_m, bool learnable = true)`
  - `std::vector<std::string> DiagnosisEngine::open_event_codes() const`

- [ ] **Step 1: 写失败测试（配置）**

`test/test_diagnosis.cpp`：
- `DefaultsAreValidAndMatchRtkMonitor` 末尾加 `EXPECT_DOUBLE_EQ(c.divergence_sigma_max_m, 0.10);`
- `RejectsNonsenseWithTheFieldName` 里 `c.divergence_epoch_max_dt_s = 0.0; expect_rejected(...)` 这一行末尾补 `c = {};`，再加：

```cpp
  c.divergence_sigma_max_m = 0.04;  expect_rejected(c, "divergence_sigma_max_m");   // 低于 5 cm 下限
```

- [ ] **Step 2: 写失败测试（偏差监测）**

`test/test_divergence_monitor.cpp` 的 `cfg_small()` 在 `c.divergence_sigma_floor_m = 0.05;` 后加：

```cpp
  // 既有用例钉的是不设上限时的窗口/预热/held 行为(基线常取 0.2 m),上限另有专门用例
  c.divergence_sigma_max_m = 10.0;
```

文件末尾追加：

```cpp
// 设计决定 4:不可学习的样本(任一路不是 FIXED)照常判定、计时,但不进经验窗口——
// 预热期与预热结束后都一样。
TEST(DivergenceMonitor, NonLearnableSamplesAreJudgedButNotAdmitted) {
  DivergenceMonitor m(cfg_small());
  auto s = m.update(0.0, 0.02, 0.001, false);
  EXPECT_EQ(m.window_size(), 0u) << "预热期里不可学习的样本也不能入窗口";
  s = m.update(1.0, 0.5, 0.001, false);
  ASSERT_TRUE(s.since.has_value()) << "不可学习不等于不判定:0.5 > 0.15";
  EXPECT_DOUBLE_EQ(*s.since, 1.0);

  DivergenceMonitor w(cfg_small());
  for (int t = 0; t < 10; ++t) w.update(t, 0.02, 0.001);   // 窗口攒满 10 个
  s = w.update(10.0, 0.03, 0.001, false);                   // 预热在这一拍结束;0.03 < 0.15
  EXPECT_FALSE(s.since.has_value());
  EXPECT_EQ(w.window_size(), 10u) << "未超限但不可学习:不入窗口";
  s = w.update(11.0, 0.5, 0.001, false);
  ASSERT_TRUE(s.since.has_value());
  EXPECT_DOUBLE_EQ(*s.since, 11.0);
}

// 设计决定 4:学到的 σ 最多 divergence_sigma_max_m,阈值里来自学习的部分最多 3 × 上限
TEST(DivergenceMonitor, LearnedSigmaIsCapped) {
  auto cfg = cfg_small();
  cfg.divergence_sigma_max_m = 0.10;
  DivergenceMonitor m(cfg);
  for (int t = 0; t < 10; ++t) m.update(t, 0.2, 0.001);   // 窗口 RMS 0.2
  const auto s = m.update(10.0, 0.35, 0.001);
  EXPECT_TRUE(s.empirical);
  EXPECT_NEAR(s.threshold_m, 0.30, 1e-9) << "不设上限时是 0.6";
  ASSERT_TRUE(s.since.has_value()) << "0.35 > 0.30";
  EXPECT_DOUBLE_EQ(*s.since, 10.0);
}

// 3a 遗留 A.1:启动时就存在的 0.5 m 持续偏差,预热期会被收进窗口,但上限让它学不成"正常"
TEST(DivergenceMonitor, AnOffsetPresentFromStartIsStillReportedAfterWarmUp) {
  auto cfg = cfg_small();
  cfg.divergence_sigma_max_m = 0.10;
  DivergenceMonitor m(cfg);
  for (int t = 0; t < 10; ++t) m.update(t, 0.5, 0.001);
  for (int t = 10; t <= 400; ++t) {
    const auto s = m.update(t, 0.5, 0.001);
    ASSERT_TRUE(s.since.has_value()) << "t=" << t;
    EXPECT_DOUBLE_EQ(*s.since, 10.0) << "t=" << t << ":预热结束那一拍重新起算,之后一直保持";
    EXPECT_LE(s.threshold_m, 0.30 + 1e-9) << "t=" << t;
  }
}

// 3a 遗留 C:held 基线只存窗口 RMS(带下限、上限),从不存本拍的当前 σ
TEST(DivergenceMonitor, HeldBaselineNeverStoresTheCurrentSigma) {
  DivergenceMonitor m(cfg_small());
  for (int t = 0; t < 10; ++t) m.update(t, 0.02, 0.001);   // 基线 = 下限 0.05
  const auto raised = m.update(10.0, 0.02, 0.3);             // 当前 σ 0.3 抬高本拍阈值
  EXPECT_NEAR(raised.threshold_m, 0.9, 1e-9);
  // t=105:t<=5 的样本出窗,剩 6..10 共 5 个 < 10,走 held 基线;105-10=95 < 100 不算缺口
  const auto s = m.update(105.0, 0.5, 0.001);
  EXPECT_TRUE(s.empirical);
  EXPECT_NEAR(s.threshold_m, 0.15, 1e-9) << "held 基线若存了当前 σ,这里会是 0.9";
}
```

- [ ] **Step 3: 写失败测试（引擎）**

`test/test_diagnosis_engine.cpp` 末尾追加：

```cpp
// 3a 遗留 A.2:rtkrcv 掉到 FLOAT 的 90 s 里,偏差在被 σ 抬高的阈值之下,旧实现会把这些
// 样本学进窗口,之后 0.3 m 的真实偏差报不出来。只用两路都 FIXED 的样本学习后必须报出。
TEST(DiagnosisEngine, FloatSamplesDoNotWidenTheLearnedThreshold) {
  auto e = make_engine();
  const auto north = [](double m) { return 44.5 + m / 111132.0; };
  TickResult r;
  for (int i = 0; i < 70; ++i) {   // FIXED,两路相差 2 cm:学到 5 cm 下限
    const double t = 100.0 + i;
    corrections(e, t);
    e.on_solution(t, fixed());
    e.on_device_solution(t, fixed(north(0.02)));
    r = e.tick(t + 0.1);
  }
  EXPECT_NEAR(r.divergence.threshold_m, 0.15, 1e-6);
  for (int i = 70; i < 160; ++i) {   // rtkrcv FLOAT,σ 0.14 m → 阈值 0.42;偏差 0.3 m 不超限
    const double t = 100.0 + i;
    corrections(e, t);
    SolutionSample fl = fixed();
    fl.quality = Quality::FLOAT;
    fl.sdn = fl.sde = 0.1;
    e.on_solution(t, fl);
    e.on_device_solution(t, fixed(north(0.3)));
    r = e.tick(t + 0.1);
    ASSERT_FALSE(r.divergence.since.has_value()) << "t=" << t;
  }
  for (int i = 160; i <= 170; ++i) {   // 回到 FIXED,偏差仍是 0.3 m
    const double t = 100.0 + i;
    corrections(e, t);
    e.on_solution(t, fixed());
    e.on_device_solution(t, fixed(north(0.3)));
    r = e.tick(t + 0.1);
  }
  EXPECT_LT(r.divergence.threshold_m, 0.2) << "FLOAT 期间的样本不能拉宽学到的阈值";
  EXPECT_TRUE(has_code(r, "device_divergence"));
}

TEST(DiagnosisEngine, OpenEventCodesListsWhatIsOpen) {
  auto e = make_engine();
  EXPECT_TRUE(e.open_event_codes().empty());
  corrections(e, 100.0);
  e.tick(104.0);   // 差分中断 4 s + 无解:corr_outage 与 no_solution
  const auto codes = e.open_event_codes();
  EXPECT_NE(std::find(codes.begin(), codes.end(), "corr_outage"), codes.end());
  EXPECT_NE(std::find(codes.begin(), codes.end(), "no_solution"), codes.end());
  e.shutdown(105.0);
  EXPECT_TRUE(e.open_event_codes().empty());
}
```

- [ ] **Step 4: 确认 RED（对桩实现）**

先只做"能编译"的桩：`DiagnosisConfig` 加字段 `double divergence_sigma_max_m = 0.10;`（不加校验）；`update` 加 `bool learnable = true` 参数但函数体完全不用它；`open_event_codes()` 返回 `{}`。

Run: `cd /home/steve/glim_ws && source /opt/ros/humble/setup.bash && colcon build --symlink-install --packages-select gnss_core && ./build/gnss_core/test_divergence_monitor && ./build/gnss_core/test_diagnosis_engine; ./build/gnss_core/test_diagnosis`
Expected FAIL：`RejectsNonsenseWithTheFieldName`、`NonLearnableSamplesAreJudgedButNotAdmitted`、`LearnedSigmaIsCapped`、`AnOffsetPresentFromStartIsStillReportedAfterWarmUp`、`FloatSamplesDoNotWidenTheLearnedThreshold`、`OpenEventCodesListsWhatIsOpen`。
`HeldBaselineNeverStoresTheCurrentSigma` 与默认值断言对桩会 PASS——前者在 Step 6 用变异验证，后者是纯默认值钉子。

- [ ] **Step 5: 实现**

`diagnosis.hpp`，`divergence_sigma_floor_m` 之后：

```cpp
  // 学到的 σ(窗口 RMS 与 held 基线)上限:阈值里来自学习的部分最多 divergence_sigma × 此值,
  // 更大的持续偏差不会被学成正常。rtkrcv 当前自报 σ 不受此限。
  double divergence_sigma_max_m = 0.10;
```

`diagnosis.cpp` 的 `validate_diagnosis_config`，`divergence_sigma_floor_m` 那行之后：

```cpp
  require(c.divergence_sigma_max_m >= c.divergence_sigma_floor_m, "divergence_sigma_max_m",
          "必须 >= divergence_sigma_floor_m");
```

`divergence_monitor.hpp`：
- 文件头第 5–6 行"经验基线 = 已入窗样本……"改为：
  `//   - 经验基线 = min(divergence_sigma_max_m, max(下限, 已入窗样本的 RMS))。只有 learnable 的样本(调用方:两路都是 FIXED)才可能入窗;`
- 规则 1、3 各补一句"且 learnable"；删除"已知代价"整段，换成：

```cpp
// 学习约束(维护者 2026-09-15 决定):只有 learnable 的样本入窗,rtkrcv 浮点解期间被当前 σ
// 抬高的阈值不会反过来拉宽经验基线;学到的 σ 设上限,启动时或长缺口后重新预热期间存在的
// 持续偏移最多被学成 divergence_sigma_max_m,超过 divergence_sigma × 上限的偏移仍会报出。
```

- `update` 声明改为 `DivergenceState update(double t, std::optional<double> divergence_m, double current_sigma_m, bool learnable = true);`，注释补"learnable=false:照常判定与计时,但不入经验窗口"。
- 私有成员 `double sigma_mult_, window_s_, floor_m_;` 改为 `double sigma_mult_, window_s_, floor_m_, cap_m_;`

`divergence_monitor.cpp`：
- 构造函数初始化列表 `floor_m_(...)` 后加 `cap_m_(cfg.divergence_sigma_max_m),`
- 签名同步加 `bool learnable`。
- 规则 5 注释改为 `// 规则 5:非有限值(NaN/inf)一律当作没配上处理——不入窗口、清零 since、`（3a 遗留 C：代码本来就清零 since）。
- 规则 A 的 `base = std::max(floor_m_, ...)` 改为：

```cpp
    base = std::min(cap_m_, std::max(floor_m_, std::sqrt(sum_sq / static_cast<double>(window_.size()))));
```

- 预热分支的 `window_.emplace_back(t, *divergence_m);` 改为 `if (learnable) window_.emplace_back(t, *divergence_m);`；预热结束分支 `else` 块里的同一句同样改。

`diagnosis_engine.hpp`，`baseline()` 之后：

```cpp
  // 当前已开事件的规则码(字典序),壳发布状态时列出
  std::vector<std::string> open_event_codes() const { return events_.open_codes(); }
```

`diagnosis_engine.cpp` 的 `tick`：`std::optional<double> d;` 后加 `bool learnable = false;`；`if (partner) d = geodesic_distance_m(...);` 改为

```cpp
    if (partner) {
      d = geodesic_distance_m(sol->lat, sol->lon, partner->lat, partner->lon);
      // 只有两路都是固定解的偏差才代表"正常水平",才允许学进经验窗口(设计决定 4)
      learnable = sol->quality == Quality::FIXED && partner->quality == Quality::FIXED;
    }
```

`divergence_.update(now, d, sol ? std::hypot(sol->sdn, sol->sde) : 0.0)` 改为追加实参 `, learnable`。

- [ ] **Step 6: GREEN + 变异**

Run: Step 4 的三个测试二进制全部 PASS。
变异（逐个做、逐个恢复，记录结果）：
1. `divergence_monitor.cpp` 规则 A 的 `baseline_sigma_ = base;` 改成 `baseline_sigma_ = std::max(base, current_sigma_m);` → `HeldBaselineNeverStoresTheCurrentSigma` 必须 FAIL。
2. 引擎里 `learnable = ...` 改成 `learnable = true;` → `FloatSamplesDoNotWidenTheLearnedThreshold` 必须 FAIL。
3. 规则 A 去掉 `std::min(cap_m_, ...)` → `LearnedSigmaIsCapped`、`AnOffsetPresentFromStartIsStillReportedAfterWarmUp` 必须 FAIL。

- [ ] **Step 7: 全量 + 提交**

Run: 全量测试，0 failures。

```bash
cd /home/steve/glim_ws/src/glim_ext
git add gnss_core/include/gnss_core/diagnosis.hpp gnss_core/src/diagnosis.cpp \
  gnss_core/include/gnss_core/divergence_monitor.hpp gnss_core/src/divergence_monitor.cpp \
  gnss_core/include/gnss_core/diagnosis_engine.hpp gnss_core/src/diagnosis_engine.cpp \
  gnss_core/test/test_divergence_monitor.cpp gnss_core/test/test_diagnosis_engine.cpp gnss_core/test/test_diagnosis.cpp
git commit -m "feat(gnss_core): learn divergence only from FIXED pairs and cap the learned sigma

<RED(桩)与三个变异的结果>

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 3: 诊断节点的纯逻辑支撑（`diag_node_support.hpp`、`day_file_path`）

**Files:**
- Modify: `glim_ext/gnss_bringup/include/gnss_bringup/pos_rotation.hpp`（抽出 `day_file_path`）
- Test: `glim_ext/gnss_bringup/test/test_pos_rotation.cpp`
- Create: `glim_ext/gnss_bringup/include/gnss_bringup/diag_node_support.hpp`
- Create: `glim_ext/gnss_bringup/test/test_diag_node_support.cpp`
- Modify: `glim_ext/gnss_bringup/CMakeLists.txt`、`package.xml`

**Interfaces:**
- Consumes: Task 1 的 `RtkFix::ratio`；Task 2 的 `DiagnosisEngine::open_event_codes()`（这里只用 `TickResult`）
- Produces（全部在 `namespace gnss_bringup`）:
  - `std::string day_file_path(const std::string& root, const std::string& filename, double utc_stamp)`
  - `gnss_core::SolutionSample to_solution_sample(const gnss_msgs::msg::RtkFix&)`
  - `uint8_t to_diagnostic_level(gnss_core::Level)`
  - `diagnostic_msgs::msg::DiagnosticStatus make_diagnostic_status(const gnss_core::TickResult&, const std::vector<std::string>& open_codes)`
  - `diagnostic_msgs::msg::DiagnosticStatus make_startup_grace_status(double remaining_s)`
  - `class DayFileAppender { DayFileAppender(root, filename, header); bool append(double t, const std::string& line); void close(); const std::string& current_path() const; }`
  - `std::optional<gnss_core::Ecef> parse_base_history_line(const std::string&)`、`std::optional<gnss_core::Ecef> read_last_base_history(const std::string& root)`
  - `struct ClockStep { double t; bool jumped; }`、`class MonotonicClockGuard { explicit MonotonicClockGuard(double tolerance_s); ClockStep step(double now); std::optional<double> last() const; }`
  - `std::vector<gnss_core::ControlPoint> control_points_from_params(names, lat, lon)`（非法抛 `std::invalid_argument`，消息以 `control_points` 开头）
  - `class UnpairedWatch { explicit UnpairedWatch(int streak_ticks); bool update(bool both_streams_live, bool paired); }`
  - 状态名常量：`inline constexpr const char* kDiagStatusName = "gnss_diag";`

- [ ] **Step 1: 写失败测试（`day_file_path`）**

`test/test_pos_rotation.cpp` 顶部 `using` 区加 `using gnss_bringup::day_file_path;`，末尾追加：

```cpp
TEST(DayFilePath, SameDateRulesAsPosFiles) {
  EXPECT_EQ(day_file_path("/data/gnss/", "events.log", 1789208625.0), "/data/gnss/20260912/events.log");
  EXPECT_EQ(day_file_path("/d", "base.pos", 1789257600.0), "/d/20260913/base.pos");
}

TEST(DayFilePath, RejectsUnsafeInput) {
  EXPECT_EQ(day_file_path("", "events.log", 1789208625.0), "");
  EXPECT_EQ(day_file_path("/d", "", 1789208625.0), "");
  EXPECT_EQ(day_file_path("/d", "a/b", 1789208625.0), "");
  EXPECT_EQ(day_file_path("/d", "..", 1789208625.0), "") << "\"..\" 会逃出日期目录";
  EXPECT_EQ(day_file_path("/d", "events.log", std::numeric_limits<double>::quiet_NaN()), "");
}
```

- [ ] **Step 2: 写失败测试（`diag_node_support`）**

Create `test/test_diag_node_support.cpp`：

```cpp
#include <gtest/gtest.h>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include "gnss_bringup/diag_node_support.hpp"
#include "node_process_harness.hpp"   // make_temp_dir、read_file、count_occurrences
using namespace gnss_bringup;
using gnss_bringup_test::count_occurrences;
using gnss_bringup_test::make_temp_dir;
using gnss_bringup_test::read_file;
namespace fs = std::filesystem;

namespace {
gnss_msgs::msg::RtkFix fix() {
  gnss_msgs::msg::RtkFix m;
  m.quality = gnss_msgs::msg::RtkFix::QUALITY_FIXED;
  m.latitude = 44.5;
  m.longitude = 90.28;
  m.sigma_enu = {0.022, 0.011, 0.033};   // E, N, U
  m.diff_age = 0.8f;
  m.sats_used = 20;
  return m;
}

std::string value_of(const diagnostic_msgs::msg::DiagnosticStatus& s, const std::string& key) {
  for (const auto& kv : s.values) {
    if (kv.key == key) return kv.value;
  }
  return "<missing>";
}

gnss_core::TickResult tick_with(std::vector<gnss_core::Verdict> verdicts) {
  gnss_core::TickResult r;
  r.result.verdicts = std::move(verdicts);
  r.divergence.threshold_m = 0.15;
  return r;
}
}  // namespace

TEST(ToSolutionSample, MapsFieldsAndSwapsSigmaOrder) {
  const auto s = to_solution_sample(fix());
  EXPECT_EQ(s.quality, gnss_core::Quality::FIXED);
  EXPECT_DOUBLE_EQ(s.lat, 44.5);
  EXPECT_DOUBLE_EQ(s.lon, 90.28);
  EXPECT_EQ(s.ns, 20);
  EXPECT_DOUBLE_EQ(s.sdn, 0.011) << "N 取 sigma_enu[1]";
  EXPECT_DOUBLE_EQ(s.sde, 0.022) << "E 取 sigma_enu[0]";
  EXPECT_NEAR(s.age, 0.8, 1e-6);
}

TEST(ToSolutionSample, ZeroRatioAndZeroGnssTimeMeanNotProvided) {
  auto m = fix();
  auto s = to_solution_sample(m);
  EXPECT_FALSE(s.ratio.has_value()) << "ratio 0 = 源不提供,ambiguity 规则不能拿 0 去判";
  EXPECT_FALSE(s.epoch_t.has_value()) << "gnss_time 0 = 源不提供,只能按到达时刻配对";
  m.ratio = 7.5f;
  m.gnss_time = 1789208625.25;
  s = to_solution_sample(m);
  ASSERT_TRUE(s.ratio.has_value());
  EXPECT_DOUBLE_EQ(*s.ratio, 7.5);
  ASSERT_TRUE(s.epoch_t.has_value());
  EXPECT_DOUBLE_EQ(*s.epoch_t, 1789208625.25);
}

TEST(ToSolutionSample, UnknownQualityIsNone) {
  auto m = fix();
  m.quality = 9;
  EXPECT_EQ(to_solution_sample(m).quality, gnss_core::Quality::NONE);
}

TEST(DiagnosticStatus, LevelIsTheWorstVerdictAndMessageIsTheStatusVerdict) {
  using S = diagnostic_msgs::msg::DiagnosticStatus;
  const auto r = tick_with({{gnss_core::Level::Warning, "multipath", "G01 残差异常——疑似多路径"},
                            {gnss_core::Level::Serious, "device_divergence", "610 输出与独立解算偏差 0.40m"},
                            {gnss_core::Level::Ok, "rtk_fixed", "RTK 固定"}});
  const auto st = make_diagnostic_status(r, {"device_divergence", "multipath"});
  EXPECT_EQ(st.name, "gnss_diag");
  EXPECT_EQ(st.level, S::ERROR) << "级别取全部结论里最严重的,不是第一条";
  EXPECT_EQ(st.message, "G01 残差异常——疑似多路径");
  EXPECT_EQ(value_of(st, "status_code"), "multipath");
  EXPECT_EQ(value_of(st, "open_events"), "device_divergence,multipath");
  EXPECT_EQ(value_of(st, "divergence_m"), "-");
  EXPECT_EQ(value_of(st, "divergence_threshold_m"), "0.150");
  EXPECT_EQ(value_of(st, "verdict.device_divergence"), "serious 610 输出与独立解算偏差 0.40m");
}

TEST(DiagnosticStatus, OkAndInfoAreOkWarningIsWarn) {
  using S = diagnostic_msgs::msg::DiagnosticStatus;
  EXPECT_EQ(to_diagnostic_level(gnss_core::Level::Ok), S::OK);
  EXPECT_EQ(to_diagnostic_level(gnss_core::Level::Info), S::OK);
  EXPECT_EQ(to_diagnostic_level(gnss_core::Level::Warning), S::WARN);
  EXPECT_EQ(to_diagnostic_level(gnss_core::Level::Critical), S::ERROR);
  auto r = tick_with({{gnss_core::Level::Info, "no_solution", "独立解算未启用"}});
  r.divergence.divergence_m = 0.0234;
  const auto st = make_diagnostic_status(r, {});
  EXPECT_EQ(st.level, S::OK);
  EXPECT_EQ(value_of(st, "open_events"), "-");
  EXPECT_EQ(value_of(st, "divergence_m"), "0.023");
}

TEST(DiagnosticStatus, EmptyVerdictsAreAnErrorNotACrash) {
  const auto st = make_diagnostic_status(tick_with({}), {});
  EXPECT_EQ(st.level, diagnostic_msgs::msg::DiagnosticStatus::ERROR);
}

TEST(DiagnosticStatus, StartupGraceIsOk) {
  const auto st = make_startup_grace_status(42.4);
  EXPECT_EQ(st.name, "gnss_diag");
  EXPECT_EQ(st.level, diagnostic_msgs::msg::DiagnosticStatus::OK);
  EXPECT_NE(st.message.find("启动宽限期"), std::string::npos) << st.message;
  EXPECT_NE(st.message.find("42"), std::string::npos) << st.message;
}

TEST(DayFileAppender, WritesTheHeaderOnceAndRollsOverAtUtcMidnight) {
  const auto root = make_temp_dir("day_appender_");
  ASSERT_FALSE(root.empty());
  {
    DayFileAppender out(root, "events.log", "% header\n");
    EXPECT_TRUE(out.append(1789257599.0, "a"));   // 2026-09-12 23:59:59 UTC
    EXPECT_TRUE(out.append(1789257599.5, "a2"));
    EXPECT_TRUE(out.append(1789257600.0, "b"));   // 2026-09-13 00:00:00 UTC
    EXPECT_EQ(out.current_path(), root + "/20260913/events.log");
  }
  EXPECT_EQ(read_file(root + "/20260912/events.log"), "% header\na\na2\n");
  EXPECT_EQ(read_file(root + "/20260913/events.log"), "% header\nb\n");
  {
    DayFileAppender again(root, "events.log", "% header\n");   // 重启后追加,不重复写头
    EXPECT_TRUE(again.append(1789257601.0, "c"));
  }
  EXPECT_EQ(count_occurrences(read_file(root + "/20260913/events.log"), "% header"), 1u);
  fs::remove_all(root);
}

TEST(DayFileAppender, FailureIsReportedAndTheNextLineRetries) {
  const auto base = make_temp_dir("day_appender_fail_");
  ASSERT_FALSE(base.empty());
  const std::string root = base + "/root";
  std::ofstream(root) << "a regular file where the root directory should be";
  DayFileAppender out(root, "events.log", "% header\n");
  EXPECT_FALSE(out.append(1789208625.0, "lost"));
  fs::remove(root);
  fs::create_directories(root);
  EXPECT_TRUE(out.append(1789208626.0, "kept"));
  EXPECT_EQ(read_file(root + "/20260912/events.log"), "% header\nkept\n");
  fs::remove_all(base);
}

TEST(BaseHistory, ParsesDataLinesOnly) {
  const auto p = parse_base_history_line("2026/09/12 10:23:45.000  -2148744.1000 4426641.2000 4044655.9000");
  ASSERT_TRUE(p.has_value());
  EXPECT_DOUBLE_EQ(p->x, -2148744.1);
  EXPECT_DOUBLE_EQ(p->z, 4044655.9);
  EXPECT_FALSE(parse_base_history_line("% time=UTC").has_value());
  EXPECT_FALSE(parse_base_history_line("2026/09/12 10:23:45.000  -2148744.1").has_value()) << "掉电留下的半行";
  EXPECT_FALSE(parse_base_history_line("").has_value());
}

TEST(BaseHistory, ReadsTheLastValidLineOfTheNewestDayThatHasOne) {
  const auto root = make_temp_dir("base_history_");
  ASSERT_FALSE(root.empty());
  const auto write = [&root](const std::string& rel, const std::string& text) {
    fs::create_directories(fs::path(root + "/" + rel).parent_path());
    std::ofstream(root + "/" + rel) << text;
  };
  write("20260910/base.pos", "% h\n2026/09/10 01:00:00.000  1.0000 2.0000 3.0000\n");
  write("20260912/base.pos", "% h\n2026/09/12 01:00:00.000  4.0000 5.0000 6.0000\n"
                             "2026/09/12 02:00:00.000  7.0000 8.0000 9.0000\n"
                             "2026/09/12 03:00:00.000  10.0\n");
  write("20260913/base.pos", "% only a header\n");
  fs::create_directories(root + "/20260914");                     // 没有 base.pos
  write("notadate/base.pos", "2026/09/20 01:00:00.000  0.0000 0.0000 0.0000\n");

  const auto last = read_last_base_history(root);
  ASSERT_TRUE(last.has_value());
  EXPECT_DOUBLE_EQ(last->x, 7.0);
  EXPECT_DOUBLE_EQ(last->z, 9.0);
  EXPECT_FALSE(read_last_base_history(root + "/absent").has_value());
  fs::remove_all(root);
}

TEST(MonotonicClockGuard, ClampsSmallJitterAndReportsLargeBackwardJumps) {
  MonotonicClockGuard g(1.0);
  EXPECT_FALSE(g.last().has_value());
  auto s = g.step(100.0);
  EXPECT_FALSE(s.jumped);
  EXPECT_DOUBLE_EQ(s.t, 100.0);
  s = g.step(99.5);                     // 回退 0.5 s:夹住
  EXPECT_FALSE(s.jumped);
  EXPECT_DOUBLE_EQ(s.t, 100.0);
  s = g.step(101.0);
  EXPECT_DOUBLE_EQ(s.t, 101.0);
  s = g.step(50.0);                     // 回退 51 s:回跳,新起点
  EXPECT_TRUE(s.jumped);
  EXPECT_DOUBLE_EQ(s.t, 50.0);
  s = g.step(50.5);
  EXPECT_FALSE(s.jumped);
  EXPECT_DOUBLE_EQ(*g.last(), 50.5);
  EXPECT_FALSE(g.step(5000.0).jumped) << "向前跳不算回跳";
}

TEST(ControlPoints, ParsesParallelArrays) {
  const auto cps = control_points_from_params({"K1", "K2"}, {44.5, 44.6}, {90.1, 90.2});
  ASSERT_EQ(cps.size(), 2u);
  EXPECT_EQ(cps[1].name, "K2");
  EXPECT_DOUBLE_EQ(cps[1].lat, 44.6);
  EXPECT_DOUBLE_EQ(cps[1].lon, 90.2);
  EXPECT_TRUE(control_points_from_params({}, {}, {}).empty());
}

TEST(ControlPoints, RejectsMismatchedEmptyOrOutOfRange) {
  const auto rejected = [](std::vector<std::string> n, std::vector<double> la, std::vector<double> lo) {
    try {
      control_points_from_params(n, la, lo);
    } catch (const std::invalid_argument& e) {
      return std::string(e.what()).rfind("control_points", 0) == 0;
    }
    return false;
  };
  EXPECT_TRUE(rejected({"K1"}, {}, {90.0}));
  EXPECT_TRUE(rejected({""}, {44.0}, {90.0}));
  EXPECT_TRUE(rejected({"K1"}, {91.0}, {90.0}));
  EXPECT_TRUE(rejected({"K1"}, {44.0}, {-181.0}));
  EXPECT_TRUE(rejected({"K1"}, {std::numeric_limits<double>::quiet_NaN()}, {90.0}));
}

TEST(UnpairedWatch, WarnsOnceAfterTheStreakAndResetsWhenPaired) {
  UnpairedWatch w(3);
  EXPECT_FALSE(w.update(true, false));
  EXPECT_FALSE(w.update(true, false));
  EXPECT_TRUE(w.update(true, false)) << "连续 3 拍两路都在却没配上";
  EXPECT_FALSE(w.update(true, false)) << "同一段只提醒一次";
  EXPECT_FALSE(w.update(true, true));
  EXPECT_FALSE(w.update(true, false));
  EXPECT_FALSE(w.update(false, false)) << "有一路没到不算配对失败,计数清零";
  EXPECT_FALSE(w.update(true, false));
  EXPECT_FALSE(w.update(true, false));
  EXPECT_TRUE(w.update(true, false));
}
```

`CMakeLists.txt`：`find_package(gnss_core REQUIRED)` 之后加 `find_package(diagnostic_msgs REQUIRED)`；`BUILD_TESTING` 块里 `test_pos_rotation` 之后加：

```cmake
  # diag_node_support.hpp:gnss_diag_node 的纯逻辑(消息转换、状态组装、按天追加写、
  # base.pos 末行、时钟回跳、控制点参数),不起节点
  ament_add_gtest(test_diag_node_support test/test_diag_node_support.cpp)
  target_link_libraries(test_diag_node_support gnss_core::gnss_core)
  target_include_directories(test_diag_node_support PRIVATE include test)
  ament_target_dependencies(test_diag_node_support gnss_msgs diagnostic_msgs)
```

`package.xml`：`<depend>gnss_core</depend>` 之后加 `<depend>diagnostic_msgs</depend>`。

- [ ] **Step 3: 确认 RED（对桩实现）**

先写桩：`pos_rotation.hpp` 加 `inline std::string day_file_path(const std::string&, const std::string&, double) { return ""; }`；`diag_node_support.hpp` 按 Interfaces 写出全部声明，函数体一律返回默认值（空状态、`std::nullopt`、`{}`、`false`、`ClockStep{now, false}`），`control_points_from_params` 返回空 vector。

Run: `cd /home/steve/glim_ws && source /opt/ros/humble/setup.bash && colcon build --symlink-install --packages-select gnss_bringup && ./build/gnss_bringup/test_pos_rotation; ./build/gnss_bringup/test_diag_node_support`
Expected: 除 `DayFilePath.RejectsUnsafeInput`（桩恰好返回空串）以外，Step 1–2 的新测试全部 FAIL；`RejectsUnsafeInput` 在 Step 5 用变异验证。

- [ ] **Step 4: 实现**

`pos_rotation.hpp`：把 `pos_path_for` 的函数体原样搬进新函数 `day_file_path`（放在 `pos_path_for` 之前），只改三处：参数 `source` 改名 `filename`；校验改为 `if (!detail::is_valid_pos_source(filename) || filename == "." || filename == "..") return "";`；两处 `"/" + source + ".pos"` 改为 `"/" + filename`。函数上方注释写：

```cpp
// <root>/YYYYMMDD/<filename>,YYYYMMDD 按 UTC。拒绝规则与下面 pos_path_for 的说明相同
// (空 root、filename 为空/带 '/'/是 "." 或 ".."、时间非有限或超出 time_t),失败返回空串。
// .pos、events.log、base.pos 共用这一套日期目录规则(spec §5.3)。
```

`pos_path_for` 保留原注释，函数体改为：

```cpp
  if (!detail::is_valid_pos_source(source)) return "";
  return day_file_path(root, source + ".pos", utc_stamp);
```

Create `include/gnss_bringup/diag_node_support.hpp`：

```cpp
#pragma once
// gnss_diag_node 的纯逻辑(轮 3b):ROS 消息 → gnss_core 类型、DiagnosticStatus 组装、按天追加写、
// 跨日期目录读 base.pos 末行、时钟回跳检测、控制点参数、配对失败提醒。节点只负责接线。
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <gnss_msgs/msg/rtk_fix.hpp>

#include "gnss_bringup/pos_rotation.hpp"
#include "gnss_core/base_station_monitor.hpp"
#include "gnss_core/diag_io.hpp"
#include "gnss_core/diagnosis_engine.hpp"
#include "gnss_core/retention.hpp"

namespace gnss_bringup {

inline constexpr const char* kDiagStatusName = "gnss_diag";

// RtkFix → 诊断用的解样本。0 表示源不提供的字段(ratio、gnss_time,见 RtkFix.msg)映射为空,
// 不能当成真实的 0 去判 ambiguity 或按历元配对。
inline gnss_core::SolutionSample to_solution_sample(const gnss_msgs::msg::RtkFix& m) {
  gnss_core::SolutionSample s;
  s.quality = m.quality <= static_cast<uint8_t>(gnss_core::Quality::FIXED)
                  ? static_cast<gnss_core::Quality>(m.quality)
                  : gnss_core::Quality::NONE;
  s.lat = m.latitude;
  s.lon = m.longitude;
  s.ns = m.sats_used;
  s.sdn = m.sigma_enu[1];   // sigma_enu 是 E/N/U
  s.sde = m.sigma_enu[0];
  s.age = m.diff_age;
  if (std::isfinite(m.ratio) && m.ratio > 0.0f) s.ratio = m.ratio;
  if (std::isfinite(m.gnss_time) && m.gnss_time > 0.0) s.epoch_t = m.gnss_time;
  return s;
}

inline uint8_t to_diagnostic_level(gnss_core::Level level) {
  using S = diagnostic_msgs::msg::DiagnosticStatus;
  switch (level) {
    case gnss_core::Level::Ok:
    case gnss_core::Level::Info: return S::OK;
    case gnss_core::Level::Warning: return S::WARN;
    case gnss_core::Level::Serious:
    case gnss_core::Level::Critical: return S::ERROR;
  }
  return S::ERROR;
}

namespace detail {
inline void add_value(diagnostic_msgs::msg::DiagnosticStatus& st, std::string key, std::string value) {
  diagnostic_msgs::msg::KeyValue kv;
  kv.key = std::move(key);
  kv.value = std::move(value);
  st.values.push_back(std::move(kv));
}

inline std::string fixed3(double v) {
  char buf[48];
  std::snprintf(buf, sizeof(buf), "%.3f", v);
  return buf;
}
}  // namespace detail

// 一拍的诊断状态:level 取全部结论里最严重的一条,message/status_code 取优先级最高的一条
// (DiagnosisResult::status());values 列出已开事件、偏差、阈值与每条结论。
inline diagnostic_msgs::msg::DiagnosticStatus make_diagnostic_status(const gnss_core::TickResult& r,
                                                                     const std::vector<std::string>& open_codes) {
  diagnostic_msgs::msg::DiagnosticStatus st;
  st.name = kDiagStatusName;
  st.hardware_id = "gnss";
  if (r.result.verdicts.empty()) {   // evaluate_rules 保证非空;这里不依赖它(status() 对空结果是未定义行为)
    st.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    st.message = "规则链没有输出";
    return st;
  }
  gnss_core::Level worst = gnss_core::Level::Ok;
  for (const auto& v : r.result.verdicts) worst = std::max(worst, v.level);
  st.level = to_diagnostic_level(worst);
  const auto& head = r.result.status();
  st.message = head.message;
  detail::add_value(st, "status_code", head.code);
  std::string open;
  for (const auto& c : open_codes) open += (open.empty() ? "" : ",") + c;
  detail::add_value(st, "open_events", open.empty() ? "-" : open);
  detail::add_value(st, "divergence_m",
                    r.divergence.divergence_m ? detail::fixed3(*r.divergence.divergence_m) : "-");
  detail::add_value(st, "divergence_threshold_m", detail::fixed3(r.divergence.threshold_m));
  for (const auto& v : r.result.verdicts) {
    detail::add_value(st, "verdict." + v.code, std::string(gnss_core::level_name(v.level)) + " " + v.message);
  }
  return st;
}

inline diagnostic_msgs::msg::DiagnosticStatus make_startup_grace_status(double remaining_s) {
  diagnostic_msgs::msg::DiagnosticStatus st;
  st.name = kDiagStatusName;
  st.hardware_id = "gnss";
  st.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  char buf[96];
  std::snprintf(buf, sizeof(buf), "启动宽限期(剩余 %.0f s),暂不判定", std::max(0.0, remaining_s));
  st.message = buf;
  return st;
}

// <root>/YYYYMMDD/<filename> 的逐行追加写,跨 UTC 零点自动换文件,新文件先写 header。
// 打开或写入失败时本行丢弃并返回 false(调用方节流报错),并关闭文件,下一行重新尝试打开。
class DayFileAppender {
public:
  DayFileAppender(std::string root, std::string filename, std::string header)
      : root_(std::move(root)), filename_(std::move(filename)), header_(std::move(header)) {}

  bool append(double t, const std::string& line) {
    const std::string path = day_file_path(root_, filename_, t);
    if (path.empty()) return false;
    if (!out_.is_open() || out_.path() != path) {
      if (!out_.open(path, header_)) return false;
    }
    if (!out_.append(line)) {
      out_.close();
      return false;
    }
    return true;
  }
  void close() { out_.close(); }
  const std::string& current_path() const { return out_.path(); }

private:
  std::string root_, filename_, header_;
  gnss_core::LineAppender out_;
};

// base.pos 数据行:"YYYY/MM/DD HH:MM:SS.sss x y z"(gnss_core::format_base_history_line)。
// 注释行、空行、字段不全(掉电留下的半行)、非有限坐标都返回空。
inline std::optional<gnss_core::Ecef> parse_base_history_line(const std::string& line) {
  if (line.empty() || line[0] == '%') return std::nullopt;
  std::istringstream in(line);
  std::string date, time;
  gnss_core::Ecef p;
  if (!(in >> date >> time >> p.x >> p.y >> p.z)) return std::nullopt;
  if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) return std::nullopt;
  return p;
}

// 设计决定 6:从 <root> 下日期最新、含有效数据行的 YYYYMMDD/base.pos 取最后一个有效行;
// 找不到(首次运行、root 不存在)返回空——引擎因此会多写一行历史,无害。
inline std::optional<gnss_core::Ecef> read_last_base_history(const std::string& root) {
  namespace fs = std::filesystem;
  std::vector<std::pair<int, std::string>> days;
  std::error_code ec;
  for (fs::directory_iterator it(root, ec), end; !ec && it != end; it.increment(ec)) {
    const std::string name = it->path().filename().string();
    if (const auto d = gnss_core::parse_day_dir_date(name)) days.emplace_back(*d, name);
  }
  std::sort(days.rbegin(), days.rend());
  for (const auto& [date, name] : days) {
    std::ifstream in(fs::path(root) / name / "base.pos");
    std::optional<gnss_core::Ecef> last;
    for (std::string line; std::getline(in, line);) {
      if (const auto p = parse_base_history_line(line)) last = p;
    }
    if (last) return last;
  }
  return std::nullopt;
}

// 引擎要求时间单调不减(3a 遗留 B),ROS 时间(尤其 use_sim_time 回放)可能回退:
//   - 回退不超过 tolerance_s:夹到上一次的值,不算事件;
//   - 回退超过 tolerance_s:jumped=true,本次时间成为新起点,调用方关闭事件并重建引擎(设计决定 2)。
// 向前跳不处理(引擎按新鲜度自然过期)。
struct ClockStep {
  double t = 0.0;
  bool jumped = false;
};

class MonotonicClockGuard {
public:
  explicit MonotonicClockGuard(double tolerance_s) : tolerance_s_(tolerance_s) {}
  ClockStep step(double now) {
    if (!last_) {
      last_ = now;
      return {now, false};
    }
    if (now < *last_ - tolerance_s_) {
      last_ = now;
      return {now, true};
    }
    last_ = std::max(*last_, now);
    return {*last_, false};
  }
  std::optional<double> last() const { return last_; }

private:
  double tolerance_s_;
  std::optional<double> last_;
};

// 设计决定 7:控制点参数是三个等长数组(ROS 参数不支持结构体数组)
inline std::vector<gnss_core::ControlPoint> control_points_from_params(const std::vector<std::string>& names,
                                                                       const std::vector<double>& lat,
                                                                       const std::vector<double>& lon) {
  if (names.size() != lat.size() || names.size() != lon.size()) {
    throw std::invalid_argument("control_points.names/.lat/.lon 长度必须相同(收到 " +
                                std::to_string(names.size()) + "/" + std::to_string(lat.size()) + "/" +
                                std::to_string(lon.size()) + ")");
  }
  std::vector<gnss_core::ControlPoint> out;
  for (size_t i = 0; i < names.size(); ++i) {
    const std::string idx = "[" + std::to_string(i) + "]";
    if (names[i].empty()) throw std::invalid_argument("control_points.names" + idx + " 不能为空");
    if (!std::isfinite(lat[i]) || std::abs(lat[i]) > 90.0) {
      throw std::invalid_argument("control_points.lat" + idx + " 必须在 [-90, 90]");
    }
    if (!std::isfinite(lon[i]) || std::abs(lon[i]) > 180.0) {
      throw std::invalid_argument("control_points.lon" + idx + " 必须在 [-180, 180]");
    }
    out.push_back(gnss_core::ControlPoint{names[i], lat[i], lon[i]});
  }
  return out;
}

// 3a 遗留 B:两路解都在按时到达、却连续 streak_ticks 拍没配上对(历元始终相差超过
// divergence_epoch_max_dt_s,或一路缺历元时刻而到达时刻相差太大)时,device_divergence 会
// 静默失效。update 在连续失败达到门限的那一拍返回 true(每段只一次);配上或有一路没到时复位。
class UnpairedWatch {
public:
  explicit UnpairedWatch(int streak_ticks) : limit_(streak_ticks) {}
  bool update(bool both_streams_live, bool paired) {
    if (!both_streams_live || paired) {
      streak_ = 0;
      warned_ = false;
      return false;
    }
    if (++streak_ >= limit_ && !warned_) {
      warned_ = true;
      return true;
    }
    return false;
  }

private:
  int limit_;
  int streak_ = 0;
  bool warned_ = false;
};

}  // namespace gnss_bringup
```

注意：`ControlPoint` 是聚合体（`name` + 带默认成员初始化的 `lat`/`lon`），C++17 下花括号初始化合法；若编译器报错，改成逐字段赋值。

- [ ] **Step 5: GREEN + 变异 + 全量**

Run: 两个测试二进制全部 PASS，然后全量。
变异（逐个做、逐个恢复）：
1. `day_file_path` 去掉 `|| filename == ".."` → `DayFilePath.RejectsUnsafeInput` 必须 FAIL。
2. `DayFileAppender::append` 里去掉 `out_.path() != path` 条件 → `WritesTheHeaderOnceAndRollsOverAtUtcMidnight` 必须 FAIL。
3. `read_last_base_history` 里 `if (last) return last;` 改成无条件 `return last;` → `ReadsTheLastValidLineOfTheNewestDayThatHasOne` 必须 FAIL。
4. `make_diagnostic_status` 里 `worst` 改成 `head.level` → `LevelIsTheWorstVerdict...` 必须 FAIL。

- [ ] **Step 6: 提交**

```bash
cd /home/steve/glim_ws/src/glim_ext
git add gnss_bringup/include/gnss_bringup/pos_rotation.hpp gnss_bringup/test/test_pos_rotation.cpp \
  gnss_bringup/include/gnss_bringup/diag_node_support.hpp gnss_bringup/test/test_diag_node_support.cpp \
  gnss_bringup/CMakeLists.txt gnss_bringup/package.xml
git commit -m "feat(gnss_bringup): pure helpers for the diagnosis node

<RED(桩)与四个变异的结果>

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 4: `gnss_diag_node`

**Files:**
- Create: `glim_ext/gnss_bringup/src/gnss_diag_node.cpp`
- Create: `glim_ext/gnss_bringup/test/test_gnss_diag_node_process.cpp`
- Modify: `glim_ext/gnss_bringup/CMakeLists.txt`、`package.xml`

**Interfaces:**
- Consumes: Task 2 的引擎接口（`open_event_codes`、`learnable` 在引擎内部），Task 3 的全部支撑函数，`gnss_core::{DiagnosisEngine, read_base_baseline, write_base_baseline, events_log_header, base_pos_header, format_event_line, format_base_history_line}`，`gnss_bringup::{LineSplitter, is_positive_finite_seconds}`（`rtk_fix_mapping.hpp`）
- Produces（节点名 `gnss_diag`）:
  - 订阅：`corrections_topic`（默认 `/gnss/rtcm_corrections`，`RawStream`）、`solution_topic`（`/rtkrcv_node/rtk_fix`）、`device_topic`（`/gnss_cgi610/rtk_fix`）、`stat_topic`（`/rtkrcv_node/stat`），全部 reliable depth 100
  - 发布：`/gnss/diagnostics`（`diagnostic_msgs/DiagnosticArray`，每秒一条，`status[0].name == "gnss_diag"`）
  - 服务：`~/reset_base_baseline`（`std_srvs/Trigger`，即 `/gnss_diag/reset_base_baseline`）
  - 文件：`<root>/YYYYMMDD/events.log`、`<root>/YYYYMMDD/base.pos`、`<root>/base_baseline`
  - 参数：`root`（必填）、四个话题、`solver_enabled`（true）、`startup_grace_s`（60.0）、`clock_jump_tolerance_s`（1.0）、`unpaired_warn_s`（60.0）、`control_points.{names,lat,lon}`、`diagnosis.<DiagnosisConfig 字段名>`（全部 22 项，默认取 `DiagnosisConfig{}`）
  - 日志锚点（测试用）：启动成功 `gnss_diag 已启动`；配置错误 `启动失败,配置有误: <原因>` 并退出码 1；时钟回跳 `时钟回跳`

- [ ] **Step 1: 写失败的节点级测试**

Create `test/test_gnss_diag_node_process.cpp`：

```cpp
#include <gtest/gtest.h>
#include <unistd.h>

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <gnss_msgs/msg/raw_stream.hpp>
#include <gnss_msgs/msg/rtk_fix.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "diag_test_fixtures.hpp"     // gnss_core/test:make_1005_frame
#include "node_process_harness.hpp"
using namespace gnss_bringup_test;
using namespace std::chrono_literals;
namespace fs = std::filesystem;

namespace {
std::string domain_id() { return std::to_string(90 + ::getpid() % 10); }

std::vector<std::pair<std::string, std::string>> isolated_env(const std::string& dir) {
  return {{"ROS_DOMAIN_ID", domain_id()}, {"ROS_LOG_DIR", dir + "/roslog"}};
}

// 本测试进程自己也要起 rclcpp 时调用,必须在 NodeProcess 构造之后、rclcpp::init 之前
void join_isolated_domain(const std::string& dir) {
  ::setenv("ROS_DOMAIN_ID", domain_id().c_str(), 1);
  ::setenv("ROS_LOG_DIR", (dir + "/roslog").c_str(), 1);
}

std::vector<std::string> diag_args(const std::string& root, double grace_s, const std::vector<std::string>& extra = {}) {
  std::vector<std::string> a{"--ros-args", "-p", "root:=" + root, "-p", "startup_grace_s:=" + std::to_string(grace_s)};
  for (const auto& e : extra) {
    a.push_back("-p");
    a.push_back(e);
  }
  return a;
}

// 所有日期目录下同名文件的内容拼起来(测试跨 UTC 零点也不漏)
std::string all_day_files(const std::string& root, const std::string& filename) {
  std::string out;
  std::error_code ec;
  for (fs::directory_iterator it(root, ec), end; !ec && it != end; it.increment(ec)) {
    const auto p = it->path() / filename;
    if (fs::is_regular_file(p)) out += read_file(p.string());
  }
  return out;
}
}  // namespace

TEST(GnssDiagNodeProcess, RefusesToStartOnInvalidConfig) {
  const auto dir = make_temp_dir("diag_node_bad_");
  ASSERT_FALSE(dir.empty());
  const auto expect_refused = [&](const std::vector<std::string>& args, const std::string& needle) {
    NodeProcess node(GNSS_DIAG_NODE_PATH, args, dir + "/node.log", isolated_env(dir));
    EXPECT_EQ(node.wait_exit(20.0), 1) << node.log();
    EXPECT_NE(node.log().find("启动失败,配置有误"), std::string::npos) << node.log();
    EXPECT_NE(node.log().find(needle), std::string::npos) << node.log();
  };
  expect_refused({"--ros-args"}, "root");
  expect_refused(diag_args(dir + "/r", 0.0, {"control_points.names:=['K1']"}), "control_points");
  expect_refused(diag_args(dir + "/r", 0.0, {"diagnosis.divergence_sigma_max_m:=0.01"}), "divergence_sigma_max_m");
  expect_refused(diag_args(dir + "/r", 0.0, {"clock_jump_tolerance_s:=0.0"}), "clock_jump_tolerance_s");
  fs::remove_all(dir);
}

TEST(GnssDiagNodeProcess, NoInputOpensNoDataAndShutdownClosesIt) {
  const auto dir = make_temp_dir("diag_node_nodata_");
  ASSERT_FALSE(dir.empty());
  const std::string root = dir + "/diag";
  NodeProcess node(GNSS_DIAG_NODE_PATH, diag_args(root, 0.0), dir + "/node.log", isolated_env(dir));
  ASSERT_TRUE(wait_until([&] { return all_day_files(root, "events.log").find(" OPEN warning no_data ") != std::string::npos; }, 20.0))
      << node.log();
  node.interrupt();
  EXPECT_EQ(node.wait_exit(20.0), 0) << node.log();
  const auto ev = all_day_files(root, "events.log");
  EXPECT_NE(ev.find(" CLOSE warning no_data "), std::string::npos) << ev;
  EXPECT_NE(ev.find("reason=shutdown"), std::string::npos) << "停机必须先写关闭行再关文件\n" << ev;
  EXPECT_EQ(count_occurrences(ev, "% gnss_core events.log"), 1u) << ev;
  fs::remove_all(dir);
}

TEST(GnssDiagNodeProcess, StartupGraceDelaysTheFirstJudgement) {
  const auto dir = make_temp_dir("diag_node_grace_");
  ASSERT_FALSE(dir.empty());
  const std::string root = dir + "/diag";
  NodeProcess node(GNSS_DIAG_NODE_PATH, diag_args(root, 5.0), dir + "/node.log", isolated_env(dir));
  ASSERT_TRUE(wait_until([&] { return node.log().find("gnss_diag 已启动") != std::string::npos; }, 20.0)) << node.log();
  std::this_thread::sleep_for(3s);
  EXPECT_EQ(all_day_files(root, "events.log").find(" OPEN "), std::string::npos) << "宽限期内不判定";
  EXPECT_TRUE(wait_until([&] { return all_day_files(root, "events.log").find(" OPEN warning no_data ") != std::string::npos; }, 15.0))
      << node.log();
  node.interrupt();
  EXPECT_EQ(node.wait_exit(20.0), 0) << node.log();
  fs::remove_all(dir);
}

TEST(GnssDiagNodeProcess, WiresInputsToDiagnosticsBaseHistoryAndTheResetService) {
  const auto dir = make_temp_dir("diag_node_wire_");
  ASSERT_FALSE(dir.empty());
  const std::string root = dir + "/diag";
  NodeProcess node(GNSS_DIAG_NODE_PATH, diag_args(root, 0.0, {"diagnosis.base_warmup_s:=1.0"}), dir + "/node.log",
                   isolated_env(dir));
  join_isolated_domain(dir);
  rclcpp::init(0, nullptr);
  {
    auto n = std::make_shared<rclcpp::Node>("diag_node_test_driver");
    const auto qos = rclcpp::QoS(100).reliable();
    auto corr = n->create_publisher<gnss_msgs::msg::RawStream>("/gnss/rtcm_corrections", qos);
    auto sol = n->create_publisher<gnss_msgs::msg::RtkFix>("/rtkrcv_node/rtk_fix", qos);
    auto dev = n->create_publisher<gnss_msgs::msg::RtkFix>("/gnss_cgi610/rtk_fix", qos);
    diagnostic_msgs::msg::DiagnosticStatus last;
    bool fixed_seen = false;
    auto sub = n->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
        "/gnss/diagnostics", 10, [&](const diagnostic_msgs::msg::DiagnosticArray& a) {
          for (const auto& s : a.status) {
            if (s.name != "gnss_diag") continue;
            last = s;
            for (const auto& kv : s.values) {
              if (kv.key == "status_code" && kv.value == "rtk_fixed") fixed_seen = true;
            }
          }
        });
    auto reset = n->create_client<std_srvs::srv::Trigger>("/gnss_diag/reset_base_baseline");
    rclcpp::executors::SingleThreadedExecutor ex;
    ex.add_node(n);

    const auto frame = gnss_core::test_fixtures::make_1005_frame(1, -2148744.1, 4426641.2, 4044655.9);
    const auto feed_once = [&] {
      gnss_msgs::msg::RawStream raw;
      raw.data = frame;
      corr->publish(raw);
      gnss_msgs::msg::RtkFix f;
      f.quality = gnss_msgs::msg::RtkFix::QUALITY_FIXED;
      f.latitude = 44.5;
      f.longitude = 90.28;
      f.sigma_enu = {0.012, 0.011, 0.03};
      f.diff_age = 0.8f;
      f.sats_used = 20;
      f.ratio = 25.0f;
      sol->publish(f);
      dev->publish(f);
    };
    const auto spin_feeding_until = [&](const std::function<bool()>& pred, double timeout_s) {
      const auto end = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout_s);
      auto next_feed = std::chrono::steady_clock::now();
      while (std::chrono::steady_clock::now() < end) {
        if (std::chrono::steady_clock::now() >= next_feed) {
          feed_once();
          next_feed += 200ms;
        }
        ex.spin_some(20ms);
        if (pred()) return true;
      }
      return pred();
    };

    EXPECT_TRUE(spin_feeding_until([&] { return fixed_seen; }, 30.0)) << "最后状态: " << last.message << "\n" << node.log();
    EXPECT_EQ(last.level, diagnostic_msgs::msg::DiagnosticStatus::OK);
    EXPECT_TRUE(spin_feeding_until([&] { return fs::exists(root + "/base_baseline"); }, 15.0)) << node.log();
    EXPECT_NE(all_day_files(root, "base.pos").find("-2148744.1000"), std::string::npos) << all_day_files(root, "base.pos");

    ASSERT_TRUE(reset->wait_for_service(10s));
    auto fut = reset->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
    ASSERT_EQ(ex.spin_until_future_complete(fut, 10s), rclcpp::FutureReturnCode::SUCCESS);
    const auto resp = fut.get();
    EXPECT_TRUE(resp->success) << resp->message;
  }
  rclcpp::shutdown();
  node.interrupt();
  EXPECT_EQ(node.wait_exit(20.0), 0) << node.log();
  fs::remove_all(dir);
}

TEST(GnssDiagNodeProcess, BackwardClockJumpClosesOpenEventsAndRebuildsTheEngine) {
  const auto dir = make_temp_dir("diag_node_jump_");
  ASSERT_FALSE(dir.empty());
  const std::string root = dir + "/diag";
  NodeProcess node(GNSS_DIAG_NODE_PATH, diag_args(root, 0.0, {"use_sim_time:=true"}), dir + "/node.log",
                   isolated_env(dir));
  join_isolated_domain(dir);
  rclcpp::init(0, nullptr);
  {
    auto n = std::make_shared<rclcpp::Node>("diag_clock_driver");
    auto clock = n->create_publisher<rosgraph_msgs::msg::Clock>("/clock", rclcpp::ClockQoS());
    double sim = 1789000000.0;   // 2026-09-10 UTC
    const auto run_clock_until = [&](const std::function<bool()>& pred, double timeout_s) {
      const auto end = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout_s);
      while (std::chrono::steady_clock::now() < end) {
        rosgraph_msgs::msg::Clock c;
        c.clock.sec = static_cast<int32_t>(std::floor(sim));
        c.clock.nanosec = static_cast<uint32_t>((sim - std::floor(sim)) * 1e9);
        clock->publish(c);
        sim += 0.05;
        std::this_thread::sleep_for(50ms);
        if (pred()) return true;
      }
      return pred();
    };
    ASSERT_TRUE(run_clock_until([&] { return all_day_files(root, "events.log").find(" OPEN warning no_data ") != std::string::npos; }, 30.0))
        << node.log();
    sim -= 100.0;
    EXPECT_TRUE(run_clock_until([&] { return node.log().find("时钟回跳") != std::string::npos; }, 15.0)) << node.log();
    EXPECT_NE(all_day_files(root, "events.log").find("reason=shutdown"), std::string::npos) << all_day_files(root, "events.log");
    EXPECT_TRUE(run_clock_until([&] { return count_occurrences(all_day_files(root, "events.log"), " OPEN warning no_data ") >= 2; }, 15.0))
        << "重建后的引擎重新判定\n" << all_day_files(root, "events.log");
  }
  rclcpp::shutdown();
  node.interrupt();
  EXPECT_EQ(node.wait_exit(20.0), 0) << node.log();
  fs::remove_all(dir);
}
```

`CMakeLists.txt`：
- 顶部 `find_package(diagnostic_msgs REQUIRED)` 之后加 `find_package(std_srvs REQUIRED)`。
- `pos_writer` 的 `install` 之后加：

```cmake
add_executable(gnss_diag_node src/gnss_diag_node.cpp)
target_link_libraries(gnss_diag_node gnss_core::gnss_core)
ament_target_dependencies(gnss_diag_node rclcpp gnss_msgs diagnostic_msgs std_srvs)
install(TARGETS gnss_diag_node DESTINATION lib/${PROJECT_NAME})
```

- `BUILD_TESTING` 块末尾（`test_rtkrcv_real_binary` 之后）加：

```cmake
  # gnss_diag_node 节点级测试:起子进程,本进程用 rclcpp 喂数据/收诊断/调服务/发 /clock。
  # 1005 帧构造复用 gnss_core 的测试夹具(同一仓库的兄弟包,只在测试里引用)。
  find_package(rosgraph_msgs REQUIRED)
  ament_add_gtest(test_gnss_diag_node_process test/test_gnss_diag_node_process.cpp TIMEOUT 240)
  target_link_libraries(test_gnss_diag_node_process gnss_core::gnss_core)
  target_include_directories(test_gnss_diag_node_process PRIVATE include test
    ${CMAKE_CURRENT_SOURCE_DIR}/../gnss_core/test)
  ament_target_dependencies(test_gnss_diag_node_process rclcpp gnss_msgs diagnostic_msgs std_srvs rosgraph_msgs)
  add_dependencies(test_gnss_diag_node_process gnss_diag_node)
  target_compile_definitions(test_gnss_diag_node_process PRIVATE
    GNSS_DIAG_NODE_PATH="$<TARGET_FILE:gnss_diag_node>")
```

`package.xml`：`<depend>diagnostic_msgs</depend>` 之后加 `<depend>std_srvs</depend>`；`<test_depend>ament_cmake_gtest</test_depend>` 之后加 `<test_depend>rosgraph_msgs</test_depend>`。

- [ ] **Step 2: 确认 RED（对桩节点）**

先写桩 `src/gnss_diag_node.cpp`：`main` 里 `rclcpp::init` → 建名为 `gnss_diag` 的节点 → 声明 `root` 参数并打印 `gnss_diag 已启动` → `rclcpp::spin` → `rclcpp::shutdown` → 返回 0（不校验、不订阅、不写文件）。

Run: `cd /home/steve/glim_ws && source /opt/ros/humble/setup.bash && colcon build --symlink-install --packages-select gnss_bringup && source install/setup.bash && ./build/gnss_bringup/test_gnss_diag_node_process`
Expected: 5 个用例全部 FAIL（`RefusesToStartOnInvalidConfig` 因桩不退出而超时返回 -1，其余等不到文件/话题/服务）。

- [ ] **Step 3: 实现节点**

用下面的完整实现替换桩：

```cpp
// gnss_diag_node:把 gnss_core::DiagnosisEngine 接到车上(spec §8,轮 3b)。
// 订阅差分裸流、rtkrcv 独立解、610 融合解、rtkrcv $SAT 流;每秒 tick 一次,事件写
// <root>/YYYYMMDD/events.log,基站坐标史写同目录 base.pos,基线持久化到 <root>/base_baseline,
// 状态发布到 /gnss/diagnostics。判定逻辑全部在 gnss_core,这里只做参数、订阅、定时与落盘接线。
//
// 线程:只用 rclcpp::spin(单线程 executor),订阅回调、定时器、服务串行执行,引擎不加锁。
// 时间:引擎与落盘都用 node->now()(回放 bag 时配 use_sim_time),经 MonotonicClockGuard 保证单调;
//   回退超过 clock_jump_tolerance_s 时以 shutdown 关闭已开事件、重建引擎(基线从文件重读)、
//   重新开始启动宽限期。use_sim_time 下还没收到 /clock(now()==0)时整拍跳过。
// 启动宽限期:前 startup_grace_s 秒只接收数据、不 tick,避免 rtkrcv 收敛前每次开机记一条 no_solution。
// 停机:spin 返回后 shutdown(now) → 写关闭行 → 关文件。
#include <algorithm>
#include <chrono>
#include <climits>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <gnss_msgs/msg/raw_stream.hpp>
#include <gnss_msgs/msg/rtk_fix.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "gnss_bringup/diag_node_support.hpp"
#include "gnss_bringup/rtk_fix_mapping.hpp"   // LineSplitter、is_positive_finite_seconds

namespace gnss_bringup {

class GnssDiagNode {
public:
  explicit GnssDiagNode(rclcpp::Node* node) : node_(node) {
    read_params();
    std::error_code ec;
    std::filesystem::create_directories(root_, ec);
    if (ec) throw std::invalid_argument("root 目录建不出来: " + root_ + ": " + ec.message());
    baseline_path_ = (std::filesystem::path(root_) / "base_baseline").string();
    events_ = std::make_unique<DayFileAppender>(root_, "events.log", gnss_core::events_log_header());
    base_history_ = std::make_unique<DayFileAppender>(root_, "base.pos", gnss_core::base_pos_header());
    build_engine();

    diag_pub_ = node_->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/gnss/diagnostics", 10);
    const auto qos = rclcpp::QoS(100).reliable();   // 与 rtcm_bridge / rtkrcv_node / 610 驱动的 reliable 发布者匹配
    corr_sub_ = node_->create_subscription<gnss_msgs::msg::RawStream>(
        corr_topic_, qos, [this](gnss_msgs::msg::RawStream::ConstSharedPtr m) { on_corrections(*m); });
    sol_sub_ = node_->create_subscription<gnss_msgs::msg::RtkFix>(
        sol_topic_, qos, [this](gnss_msgs::msg::RtkFix::ConstSharedPtr m) { on_solution(*m); });
    dev_sub_ = node_->create_subscription<gnss_msgs::msg::RtkFix>(
        dev_topic_, qos, [this](gnss_msgs::msg::RtkFix::ConstSharedPtr m) { on_device_solution(*m); });
    stat_sub_ = node_->create_subscription<gnss_msgs::msg::RawStream>(
        stat_topic_, qos, [this](gnss_msgs::msg::RawStream::ConstSharedPtr m) { on_stat(*m); });
    reset_srv_ = node_->create_service<std_srvs::srv::Trigger>(
        "~/reset_base_baseline",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
               std::shared_ptr<std_srvs::srv::Trigger::Response> resp) { on_reset_base_baseline(*resp); });
    // 墙钟定时器:回放暂停(sim time 不走)时也照常检查;判定用的时间仍取 node->now()
    timer_ = node_->create_wall_timer(std::chrono::seconds(1), [this] { on_tick(); });

    RCLCPP_INFO(node_->get_logger(),
                "gnss_diag 已启动: root=%s corr=%s sol=%s dev=%s stat=%s solver_enabled=%s 宽限期 %.0f s 控制点 %zu 个",
                root_.c_str(), corr_topic_.c_str(), sol_topic_.c_str(), dev_topic_.c_str(), stat_topic_.c_str(),
                solver_enabled_ ? "true" : "false", startup_grace_s_, control_points_.size());
  }

  void shutdown() {
    if (const auto last = clock_.last()) {
      write_transitions(engine_->shutdown(std::max(*last, node_->now().seconds())));
    }
    events_->close();
    base_history_->close();
  }

private:
  void read_params() {
    root_ = node_->declare_parameter<std::string>("root", "");
    if (root_.empty()) throw std::invalid_argument("root 不能为空");
    corr_topic_ = node_->declare_parameter<std::string>("corrections_topic", "/gnss/rtcm_corrections");
    sol_topic_ = node_->declare_parameter<std::string>("solution_topic", "/rtkrcv_node/rtk_fix");
    dev_topic_ = node_->declare_parameter<std::string>("device_topic", "/gnss_cgi610/rtk_fix");
    stat_topic_ = node_->declare_parameter<std::string>("stat_topic", "/rtkrcv_node/stat");
    solver_enabled_ = node_->declare_parameter<bool>("solver_enabled", true);

    startup_grace_s_ = node_->declare_parameter<double>("startup_grace_s", 60.0);
    if (!std::isfinite(startup_grace_s_) || startup_grace_s_ < 0.0) {
      throw std::invalid_argument("startup_grace_s 必须是 >= 0 的有限秒数");
    }
    const double tolerance_s = node_->declare_parameter<double>("clock_jump_tolerance_s", 1.0);
    if (!is_positive_finite_seconds(tolerance_s)) throw std::invalid_argument("clock_jump_tolerance_s 必须 > 0");
    clock_ = MonotonicClockGuard(tolerance_s);
    const double unpaired_warn_s = node_->declare_parameter<double>("unpaired_warn_s", 60.0);
    if (!std::isfinite(unpaired_warn_s) || unpaired_warn_s < 1.0 || unpaired_warn_s > 86400.0) {
      throw std::invalid_argument("unpaired_warn_s 必须在 [1, 86400]");
    }
    unpaired_ticks_ = static_cast<int>(unpaired_warn_s);   // 每秒一拍

    const auto names = node_->declare_parameter<std::vector<std::string>>("control_points.names", std::vector<std::string>{});
    const auto lat = node_->declare_parameter<std::vector<double>>("control_points.lat", std::vector<double>{});
    const auto lon = node_->declare_parameter<std::vector<double>>("control_points.lon", std::vector<double>{});
    control_points_ = control_points_from_params(names, lat, lon);

    cfg_ = read_diagnosis_config();
  }

  gnss_core::DiagnosisConfig read_diagnosis_config() {
    gnss_core::DiagnosisConfig c;
    const auto d = [this](const char* name, double& field) {
      field = node_->declare_parameter<double>(std::string("diagnosis.") + name, field);
    };
    const auto i = [this](const char* name, int& field) {
      // declare_parameter<int> 实际按 int64_t 取值(与 rtkrcv_node.cpp 同样的注意事项)
      const int64_t v = node_->declare_parameter<int>(std::string("diagnosis.") + name, field);
      if (v < INT_MIN || v > INT_MAX) throw std::invalid_argument(std::string(name) + ": 超出 int 范围");
      field = static_cast<int>(v);
    };
    d("corr_gap_s", c.corr_gap_s);
    d("age_max_s", c.age_max_s);
    d("base_shift_m", c.base_shift_m);
    i("min_sats", c.min_sats);
    d("resid_max_m", c.resid_max_m);
    d("low_el_deg", c.low_el_deg);
    d("low_snr_dbhz", c.low_snr_dbhz);
    d("min_ratio", c.min_ratio);
    i("slip_max_per_30s", c.slip_max_per_30s);
    d("divergence_sigma", c.divergence_sigma);
    d("divergence_hold_s", c.divergence_hold_s);
    d("close_hysteresis_s", c.close_hysteresis_s);
    d("sol_stale_s", c.sol_stale_s);
    d("abs_ref_max_m", c.abs_ref_max_m);
    d("abs_ref_radius_m", c.abs_ref_radius_m);
    d("divergence_window_s", c.divergence_window_s);
    i("divergence_min_samples", c.divergence_min_samples);
    d("divergence_sigma_floor_m", c.divergence_sigma_floor_m);
    d("divergence_sigma_max_m", c.divergence_sigma_max_m);
    d("divergence_pair_max_dt_s", c.divergence_pair_max_dt_s);
    d("divergence_epoch_max_dt_s", c.divergence_epoch_max_dt_s);
    d("base_warmup_s", c.base_warmup_s);
    gnss_core::validate_diagnosis_config(c);   // 非法时抛,消息以字段名开头
    return c;
  }

  // 新建(或回跳后重建)引擎:基线从文件读,base.pos 末行跨日期目录读(设计决定 6)
  void build_engine() {
    engine_ = std::make_unique<gnss_core::DiagnosisEngine>(cfg_, control_points_, solver_enabled_,
                                                           gnss_core::read_base_baseline(baseline_path_),
                                                           read_last_base_history(root_));
    grace_start_.reset();
    last_sol_t_.reset();
    last_dev_t_.reset();
    unpaired_ = UnpairedWatch(unpaired_ticks_);
    stat_splitter_.reset();
  }

  // 每次调用引擎前取时间。返回空:时间尚不可用。
  std::optional<double> engine_time() {
    const double now = node_->now().seconds();
    if (!(now > 0.0)) return std::nullopt;
    const auto prev = clock_.last();
    const auto step = clock_.step(now);
    if (step.jumped) {
      RCLCPP_WARN(node_->get_logger(),
                  "时钟回跳 %.3f s(%.3f → %.3f):以 shutdown 关闭已开事件,重建诊断引擎,重新开始 %.0f s 启动宽限期",
                  *prev - now, *prev, now, startup_grace_s_);
      write_transitions(engine_->shutdown(*prev));
      events_->close();
      base_history_->close();
      build_engine();
    }
    if (!grace_start_) grace_start_ = step.t;
    return step.t;
  }

  void on_corrections(const gnss_msgs::msg::RawStream& m) {
    const auto t = engine_time();
    if (!t) return;
    for (const auto& u : engine_->on_corrections(*t, m.data.data(), m.data.size())) handle_base_update(u);
  }

  void on_solution(const gnss_msgs::msg::RtkFix& m) {
    const auto t = engine_time();
    if (!t) return;
    engine_->on_solution(*t, to_solution_sample(m));
    last_sol_t_ = *t;
  }

  void on_device_solution(const gnss_msgs::msg::RtkFix& m) {
    const auto t = engine_time();
    if (!t) return;
    engine_->on_device_solution(*t, to_solution_sample(m));
    last_dev_t_ = *t;
  }

  // rtkrcv_node 按文件块转发 .stat,块边界不是行边界
  void on_stat(const gnss_msgs::msg::RawStream& m) {
    const auto t = engine_time();
    if (!t) return;
    for (const auto& line : stat_splitter_.feed(m.data.data(), m.data.size())) {
      if (line.rfind("$SAT", 0) == 0) engine_->on_stat_line(*t, line);
    }
  }

  void handle_base_update(const gnss_core::BaseUpdate& u) {
    if (u.feed.baseline_learned) {
      const auto b = engine_->baseline();
      if (b && gnss_core::write_base_baseline(baseline_path_, *b)) {
        RCLCPP_INFO(node_->get_logger(), "基站基线已持久化: %.4f %.4f %.4f -> %s", b->x, b->y, b->z,
                    baseline_path_.c_str());
      } else {
        RCLCPP_ERROR(node_->get_logger(), "基站基线写入失败: %s(重启后会重新预热)", baseline_path_.c_str());
      }
    }
    if (u.feed.history_changed &&
        !base_history_->append(u.t, gnss_core::format_base_history_line(u.t, gnss_core::Ecef{u.coords.x, u.coords.y, u.coords.z}))) {
      RCLCPP_ERROR_THROTTLE(node_->get_logger(), steady_clock_, 10000, "base.pos 写入失败(root=%s)", root_.c_str());
    }
  }

  void write_transitions(const std::vector<gnss_core::EventTransition>& transitions) {
    for (const auto& e : transitions) {
      const std::string line = gnss_core::format_event_line(e);
      if (e.kind == gnss_core::EventKind::Open) {
        RCLCPP_WARN(node_->get_logger(), "诊断事件 %s", line.c_str());
      } else {
        RCLCPP_INFO(node_->get_logger(), "诊断事件 %s", line.c_str());
      }
      if (!events_->append(e.t, line)) {
        RCLCPP_ERROR_THROTTLE(node_->get_logger(), steady_clock_, 10000, "events.log 写入失败(root=%s)", root_.c_str());
      }
    }
  }

  void on_tick() {
    const auto t = engine_time();
    if (!t) return;
    diagnostic_msgs::msg::DiagnosticArray arr;
    arr.header.stamp = node_->now();
    const double elapsed = *t - *grace_start_;
    if (elapsed < startup_grace_s_) {
      arr.status.push_back(make_startup_grace_status(startup_grace_s_ - elapsed));
      diag_pub_->publish(arr);
      return;
    }
    const auto r = engine_->tick(*t);
    write_transitions(r.transitions);
    arr.status.push_back(make_diagnostic_status(r, engine_->open_event_codes()));
    diag_pub_->publish(arr);

    const auto live = [&](const std::optional<double>& last) { return last && *t - *last < cfg_.sol_stale_s; };
    if (unpaired_.update(live(last_sol_t_) && live(last_dev_t_), r.divergence.divergence_m.has_value())) {
      RCLCPP_WARN(node_->get_logger(),
                  "独立解与 610 解都在到达,但已连续 %d s 没配上对,device_divergence 无法判定——"
                  "检查两路 gnss_time 是否都是 UTC(历元相差须 <= %.2f s)",
                  unpaired_ticks_, cfg_.divergence_epoch_max_dt_s);
    }
  }

  void on_reset_base_baseline(std_srvs::srv::Trigger::Response& resp) {
    const auto t = engine_time();
    if (!t) {
      resp.success = false;
      resp.message = "ROS 时间尚不可用(use_sim_time 下还没收到 /clock)";
      return;
    }
    const auto u = engine_->reset_base_baseline(*t);
    if (!u) {
      resp.success = false;
      resp.message = "还没收到过 RTCM 1005/1006,无法重置基站基线";
      return;
    }
    handle_base_update(*u);
    char buf[160];
    std::snprintf(buf, sizeof(buf), "基站基线已重置为 %.4f %.4f %.4f", u->coords.x, u->coords.y, u->coords.z);
    resp.success = true;
    resp.message = buf;
    RCLCPP_WARN(node_->get_logger(), "运维重置: %s", buf);
  }

  rclcpp::Node* node_;
  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};   // 日志节流用,不受 sim time 回跳影响

  std::string root_, baseline_path_;
  std::string corr_topic_, sol_topic_, dev_topic_, stat_topic_;
  bool solver_enabled_ = true;
  double startup_grace_s_ = 60.0;
  int unpaired_ticks_ = 60;
  gnss_core::DiagnosisConfig cfg_;
  std::vector<gnss_core::ControlPoint> control_points_;

  std::unique_ptr<gnss_core::DiagnosisEngine> engine_;
  std::unique_ptr<DayFileAppender> events_, base_history_;
  MonotonicClockGuard clock_{1.0};
  UnpairedWatch unpaired_{60};
  std::optional<double> grace_start_, last_sol_t_, last_dev_t_;
  LineSplitter stat_splitter_;

  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;
  rclcpp::Subscription<gnss_msgs::msg::RawStream>::SharedPtr corr_sub_, stat_sub_;
  rclcpp::Subscription<gnss_msgs::msg::RtkFix>::SharedPtr sol_sub_, dev_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_srv_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace gnss_bringup

int main(int argc, char** argv) {
  std::shared_ptr<rclcpp::Node> node;
  std::unique_ptr<gnss_bringup::GnssDiagNode> diag;
  // 与 pos_writer_node.cpp 相同:rclcpp::init / 节点构造 / 参数声明的异常统一收口成一行错误 + 退出码 1
  try {
    rclcpp::init(argc, argv);
    node = std::make_shared<rclcpp::Node>("gnss_diag");
    diag = std::make_unique<gnss_bringup::GnssDiagNode>(node.get());
  } catch (const std::exception& e) {
    if (node) {
      RCLCPP_ERROR(node->get_logger(), "启动失败,配置有误: %s", e.what());
    } else {
      std::fprintf(stderr, "gnss_diag: 启动失败,配置有误: %s\n", e.what());
    }
    diag.reset();
    if (rclcpp::ok()) rclcpp::shutdown();
    return 1;
  }

  rclcpp::spin(node);
  diag->shutdown();   // node 仍存活:now()、日志都可用
  diag.reset();
  rclcpp::shutdown();
  return 0;
}
```

实现要点（写进 commit message 以外、执行者必须自查）：
- `LineSplitter::feed` 的签名以 `rtk_fix_mapping.hpp` 为准；`reset()` 已存在。
- `RCLCPP_ERROR_THROTTLE` 的时钟参数要可变引用，`steady_clock_` 不能是 `const`。
- ROS 参数类型严格：yaml 里 double 参数必须写成 `60.0` 而不是 `60`（Task 7 的 yaml 注释要写明）。

- [ ] **Step 4: GREEN + 变异**

Run: `./build/gnss_bringup/test_gnss_diag_node_process`，5 个用例 PASS；再连续跑 3 次确认稳定（记录耗时）。
变异（逐个做、逐个恢复）：
1. `main` 里删掉 `diag->shutdown();` → `NoInputOpensNoDataAndShutdownClosesIt` 必须 FAIL。
2. `engine_time()` 里删掉 `if (step.jumped) {...}` 整块 → `BackwardClockJumpClosesOpenEventsAndRebuildsTheEngine` 必须 FAIL。
3. `on_tick()` 里把宽限期判断改成 `if (false)` → `StartupGraceDelaysTheFirstJudgement` 必须 FAIL。

- [ ] **Step 5: 全量 + 提交**

```bash
cd /home/steve/glim_ws/src/glim_ext
git add gnss_bringup/src/gnss_diag_node.cpp gnss_bringup/test/test_gnss_diag_node_process.cpp \
  gnss_bringup/CMakeLists.txt gnss_bringup/package.xml
git commit -m "feat(gnss_bringup): gnss_diag_node wires the diagnosis engine onto the vehicle

<RED(桩)、三个变异、3 次连跑耗时>

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 5: `rtkrcv_node` 健康信号

落实设计决定 9。

**Files:**
- Create: `glim_ext/gnss_bringup/include/gnss_bringup/rtkrcv_health.hpp`、`test/test_rtkrcv_health.cpp`
- Modify: `glim_ext/gnss_bringup/include/gnss_bringup/process_supervisor.hpp`（`child_running()`）
- Test: `glim_ext/gnss_bringup/test/test_process_supervisor.cpp`
- Modify: `glim_ext/gnss_bringup/src/rtkrcv_node.cpp`
- Create: `glim_ext/gnss_bringup/test/test_rtkrcv_node_health_process.cpp`
- Modify: `glim_ext/gnss_bringup/CMakeLists.txt`

**Interfaces:**
- Produces:
  - `bool ProcessSupervisor::child_running() const`（子进程已派生且尚未被回收）
  - `struct RtkrcvHealthInput { bool child_running; double since_start_s; std::optional<double> since_solution_s; std::optional<double> since_uplink_s; }`
  - `diagnostic_msgs::msg::DiagnosticStatus evaluate_rtkrcv_health(const RtkrcvHealthInput&, double no_solution_warn_s)`（`name == "rtkrcv_node"`）
  - `rtkrcv_node` 发布 `~/diagnostics`（`/rtkrcv_node/diagnostics`，`DiagnosticArray`，1 Hz，墙钟）；新参数 `health_no_solution_warn_s`（30.0，必须 > 0）

- [ ] **Step 1: 写失败测试（纯函数）**

Create `test/test_rtkrcv_health.cpp`：

```cpp
#include <gtest/gtest.h>

#include <string>

#include "gnss_bringup/rtkrcv_health.hpp"
using namespace gnss_bringup;
using S = diagnostic_msgs::msg::DiagnosticStatus;

namespace {
RtkrcvHealthInput healthy() {
  RtkrcvHealthInput in;
  in.child_running = true;
  in.since_start_s = 600.0;
  in.since_solution_s = 0.4;
  in.since_uplink_s = 0.1;
  return in;
}
bool contains(const std::string& s, const char* needle) { return s.find(needle) != std::string::npos; }
}  // namespace

TEST(RtkrcvHealth, RecentSolutionIsOk) {
  const auto st = evaluate_rtkrcv_health(healthy(), 30.0);
  EXPECT_EQ(st.name, "rtkrcv_node");
  EXPECT_EQ(st.level, S::OK);
}

TEST(RtkrcvHealth, ChildNotRunningIsErrorEvenWithAFreshSolution) {
  auto in = healthy();
  in.child_running = false;
  const auto st = evaluate_rtkrcv_health(in, 30.0);
  EXPECT_EQ(st.level, S::ERROR);
  EXPECT_TRUE(contains(st.message, "未在运行")) << st.message;
}

TEST(RtkrcvHealth, SilentSolverWithoutUplinkPointsAtTheLink) {
  auto in = healthy();
  in.since_solution_s = 31.0;
  in.since_uplink_s.reset();
  auto st = evaluate_rtkrcv_health(in, 30.0);
  EXPECT_EQ(st.level, S::WARN);
  EXPECT_TRUE(contains(st.message, "上行无数据")) << st.message;
  in.since_uplink_s = 45.0;   // 有过上行,但也停了
  st = evaluate_rtkrcv_health(in, 30.0);
  EXPECT_TRUE(contains(st.message, "上行无数据")) << st.message;
}

TEST(RtkrcvHealth, SilentSolverWithUplinkPointsAtTheConf) {
  auto in = healthy();
  in.since_solution_s = 31.0;
  const auto st = evaluate_rtkrcv_health(in, 30.0);
  EXPECT_EQ(st.level, S::WARN);
  EXPECT_TRUE(contains(st.message, "base_pos_type")) << st.message;
}

TEST(RtkrcvHealth, NeverSolvedCountsFromStartAndTheBoundaryIsStrict) {
  auto in = healthy();
  in.since_solution_s.reset();
  in.since_start_s = 30.0;
  auto st = evaluate_rtkrcv_health(in, 30.0);
  EXPECT_EQ(st.level, S::OK) << "恰好等于门限不报";
  EXPECT_TRUE(contains(st.message, "等待")) << st.message;
  in.since_start_s = 30.5;
  st = evaluate_rtkrcv_health(in, 30.0);
  EXPECT_EQ(st.level, S::WARN);
}
```

`test/test_process_supervisor.cpp` 末尾追加（沿用文件里的 `cfg_for`、`stop_async`）：

```cpp
TEST(ProcessSupervisor, ChildRunningTracksTheChildLifetime) {
  auto s = std::make_shared<ProcessSupervisor>(cfg_for("live", 0.1));
  EXPECT_FALSE(s->child_running());
  s->start();
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (!s->child_running() && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  EXPECT_TRUE(s->child_running());
  ASSERT_EQ(stop_async(s).wait_for(std::chrono::seconds(10)), std::future_status::ready);
  EXPECT_FALSE(s->child_running());
}

TEST(ProcessSupervisor, ChildRunningIsFalseWhileWaitingToRestart) {
  auto c = cfg_for("die", 5.0);
  c.max_restart_delay_s = 5.0;
  auto s = std::make_shared<ProcessSupervisor>(c);
  s->start();
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (s->spawn_count() < 1 && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));   // 立即退出的子进程已被回收,正在退避
  EXPECT_EQ(s->spawn_count(), 1);
  EXPECT_FALSE(s->child_running());
  ASSERT_EQ(stop_async(s).wait_for(std::chrono::seconds(10)), std::future_status::ready);
}
```

- [ ] **Step 2: 写失败测试（节点级）**

Create `test/test_rtkrcv_node_health_process.cpp`：

```cpp
#include <gtest/gtest.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <gnss_msgs/msg/raw_stream.hpp>
#include <rclcpp/rclcpp.hpp>

#include "node_process_harness.hpp"
using namespace gnss_bringup_test;
using namespace std::chrono_literals;
namespace fs = std::filesystem;

namespace {
std::string domain_id() { return std::to_string(20 + ::getpid() % 10); }

struct Restart {
  double delay_s = 0.1, crash_loop_life_s = 5.0, max_delay_s = 0.4;
};

std::vector<std::string> node_args(const std::string& dir, int sol_port, const std::string& mode, Restart r) {
  return {"--ros-args",
          "-p", std::string("binary:=") + FAKE_RTKRCV_PATH,
          "-p", "run_dir:=" + dir + "/run",
          "-p", "sol_port:=" + std::to_string(sol_port),
          "-p", "corr_port:=" + std::to_string(pick_free_port()),
          "-p", "obs_port:=" + std::to_string(pick_free_port()),
          "-p", "restart_delay_s:=" + std::to_string(r.delay_s),
          "-p", "crash_loop_life_s:=" + std::to_string(r.crash_loop_life_s),
          "-p", "max_restart_delay_s:=" + std::to_string(r.max_delay_s),
          "-p", "sol_initial_backoff_s:=0.2",
          "-p", "sol_max_backoff_s:=0.5",
          "-p", "health_no_solution_warn_s:=2.0",
          "-p", "args:=['" + mode + "']"};
}

// 起节点子进程 + 本进程 rclcpp,收集 /rtkrcv_node/diagnostics
class HealthFixture {
public:
  HealthFixture(const std::string& dir, int sol_port, const std::string& mode, Restart r)
      : node_(RTKRCV_NODE_PATH, node_args(dir, sol_port, mode, r), dir + "/node.log",
              {{"ROS_DOMAIN_ID", domain_id()}, {"ROS_LOG_DIR", dir + "/roslog"}}) {
    ::setenv("ROS_DOMAIN_ID", domain_id().c_str(), 1);
    ::setenv("ROS_LOG_DIR", (dir + "/roslog").c_str(), 1);
    rclcpp::init(0, nullptr);
    n_ = std::make_shared<rclcpp::Node>("rtkrcv_health_test_driver");
    sub_ = n_->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
        "/rtkrcv_node/diagnostics", 10, [this](const diagnostic_msgs::msg::DiagnosticArray& a) {
          for (const auto& s : a.status) {
            if (s.name == "rtkrcv_node") last_ = s;
          }
        });
    corr_ = n_->create_publisher<gnss_msgs::msg::RawStream>("/gnss/rtcm_corrections", rclcpp::QoS(100).reliable());
    // executor 要在 rclcpp::init 之后构造(它的 guard condition 绑定默认 context)
    ex_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    ex_->add_node(n_);
  }
  ~HealthFixture() {
    ex_->remove_node(n_);
    ex_.reset();
    sub_.reset();
    corr_.reset();
    n_.reset();
    rclcpp::shutdown();
  }

  // 边 spin 边检查;feed_uplink 为真时每 200 ms 发一块差分字节
  bool spin_until(const std::function<bool(const diagnostic_msgs::msg::DiagnosticStatus&)>& pred, double timeout_s,
                  bool feed_uplink = false) {
    const auto end = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout_s);
    auto next_feed = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() < end) {
      if (feed_uplink && std::chrono::steady_clock::now() >= next_feed) {
        gnss_msgs::msg::RawStream m;
        m.data = {0xD3, 0x00, 0x00};
        corr_->publish(m);
        next_feed += 200ms;
      }
      ex_->spin_some(20ms);
      if (pred(last_)) return true;
    }
    return false;
  }

  NodeProcess& node() { return node_; }
  const diagnostic_msgs::msg::DiagnosticStatus& last() const { return last_; }

private:
  NodeProcess node_;
  std::shared_ptr<rclcpp::Node> n_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr sub_;
  rclcpp::Publisher<gnss_msgs::msg::RawStream>::SharedPtr corr_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> ex_;
  diagnostic_msgs::msg::DiagnosticStatus last_;
};

// 在 sol_port 上当 rtkrcv 的 outstr1(tcpsvr):接受连接后每 200 ms 写一行 llh 解
class FakeSolutionServer {
public:
  explicit FakeSolutionServer(int port) {
    fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
    const int one = 1;
    ::setsockopt(fd_, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    addr.sin_port = htons(static_cast<uint16_t>(port));
    ok_ = ::bind(fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) == 0 && ::listen(fd_, 4) == 0;
    thread_ = std::thread([this] { run(); });
  }
  ~FakeSolutionServer() {
    running_ = false;
    thread_.join();
    ::close(fd_);
  }
  bool ok() const { return ok_; }

private:
  void run() {
    int client = -1;
    while (running_) {
      if (client < 0) {
        pollfd p{fd_, POLLIN, 0};
        if (ok_ && ::poll(&p, 1, 100) > 0) client = ::accept(fd_, nullptr, nullptr);
        continue;
      }
      static const char kLine[] =
          "2026/09/12 10:23:45.000   44.501234560   90.287654320   617.1230   1  20   0.0110   0.0120   0.0330"
          "   0.0000   0.0000   0.0000   0.80   25.0\n";
      if (::send(client, kLine, sizeof(kLine) - 1, MSG_NOSIGNAL) < 0) {
        ::close(client);
        client = -1;
      }
      std::this_thread::sleep_for(200ms);
    }
    if (client >= 0) ::close(client);
  }
  int fd_ = -1;
  bool ok_ = false;
  std::atomic<bool> running_{true};
  std::thread thread_;
};

bool has(const std::string& s, const char* needle) { return s.find(needle) != std::string::npos; }
}  // namespace

TEST(RtkrcvNodeHealthProcess, DeadChildIsReportedAsError) {
  const auto dir = make_temp_dir("rtkrcv_health_dead_");
  ASSERT_FALSE(dir.empty());
  fs::create_directories(dir + "/run");
  {
    HealthFixture f(dir, pick_free_port(), "die", Restart{30.0, 1.0, 60.0});
    EXPECT_TRUE(f.spin_until([](const auto& s) { return s.level == diagnostic_msgs::msg::DiagnosticStatus::ERROR; }, 20.0))
        << "最后状态: " << f.last().message << "\n" << f.node().log();
    EXPECT_TRUE(has(f.last().message, "未在运行")) << f.last().message;
    f.node().interrupt();
    EXPECT_EQ(f.node().wait_exit(20.0), 0) << f.node().log();
  }
  fs::remove_all(dir);
}

TEST(RtkrcvNodeHealthProcess, SilentSolverIsWarnAndTellsWhetherUplinkArrives) {
  const auto dir = make_temp_dir("rtkrcv_health_silent_");
  ASSERT_FALSE(dir.empty());
  fs::create_directories(dir + "/run");
  {
    HealthFixture f(dir, pick_free_port(), "live", Restart{});
    EXPECT_TRUE(f.spin_until([](const auto& s) { return has(s.message, "上行无数据"); }, 20.0))
        << "最后状态: " << f.last().message << "\n" << f.node().log();
    EXPECT_EQ(f.last().level, diagnostic_msgs::msg::DiagnosticStatus::WARN);
    EXPECT_TRUE(f.spin_until([](const auto& s) { return has(s.message, "base_pos_type"); }, 15.0, true))
        << "最后状态: " << f.last().message;
    f.node().interrupt();
    EXPECT_EQ(f.node().wait_exit(20.0), 0) << f.node().log();
  }
  fs::remove_all(dir);
}

TEST(RtkrcvNodeHealthProcess, SolutionLinesMakeItOk) {
  const auto dir = make_temp_dir("rtkrcv_health_ok_");
  ASSERT_FALSE(dir.empty());
  fs::create_directories(dir + "/run");
  const int sol_port = pick_free_port();
  {
    HealthFixture f(dir, sol_port, "live", Restart{});
    // 节点启动时会探测 sol_port 是否被占用(孤儿检测),假解算服务必须在它启动之后再监听
    ASSERT_TRUE(wait_until([&] { return has(f.node().log(), "已启动 rtkrcv 监管"); }, 20.0)) << f.node().log();
    FakeSolutionServer server(sol_port);
    ASSERT_TRUE(server.ok());
    EXPECT_TRUE(f.spin_until([](const auto& s) { return has(s.message, "解算输出正常"); }, 20.0))
        << "最后状态: " << f.last().message << "\n" << f.node().log();
    EXPECT_EQ(f.last().level, diagnostic_msgs::msg::DiagnosticStatus::OK);
    f.node().interrupt();
    EXPECT_EQ(f.node().wait_exit(20.0), 0) << f.node().log();
  }
  fs::remove_all(dir);
}
```

说明：`HealthFixture` 里 `NodeProcess` 作为第一个成员先构造（先 fork 再 `rclcpp::init`，满足 harness 的 PDEATHSIG 约束）；域号 20–29，与其他节点测试（40–89、90–99）错开。`FakeSolutionServer` 写的 llh 行在实现前用 `gnss_core::parse_llh_solution` 手工核对一次能解析（列数 ≥ 10）。

`CMakeLists.txt`：
- `rtkrcv_node` 的 `ament_target_dependencies(rtkrcv_node rclcpp gnss_msgs)` 改为 `ament_target_dependencies(rtkrcv_node rclcpp gnss_msgs diagnostic_msgs)`。
- `BUILD_TESTING` 块里 `test_sol_stream_log_gate` 之后加：

```cmake
  # rtkrcv_health.hpp:健康评估纯函数
  ament_add_gtest(test_rtkrcv_health test/test_rtkrcv_health.cpp)
  target_include_directories(test_rtkrcv_health PRIVATE include)
  ament_target_dependencies(test_rtkrcv_health diagnostic_msgs)
```

- `test_rtkrcv_node_process` 之后加：

```cmake
  # rtkrcv_node 健康话题的节点级测试:假 rtkrcv + 本进程 rclcpp 订阅 ~/diagnostics,
  # 并在 sol_port 上扮演 rtkrcv 的解算输出
  ament_add_gtest(test_rtkrcv_node_health_process test/test_rtkrcv_node_health_process.cpp TIMEOUT 180)
  target_include_directories(test_rtkrcv_node_health_process PRIVATE include test)
  ament_target_dependencies(test_rtkrcv_node_health_process rclcpp gnss_msgs diagnostic_msgs)
  add_dependencies(test_rtkrcv_node_health_process rtkrcv_node)
  target_compile_definitions(test_rtkrcv_node_health_process PRIVATE
    RTKRCV_NODE_PATH="$<TARGET_FILE:rtkrcv_node>"
    FAKE_RTKRCV_PATH="${CMAKE_CURRENT_SOURCE_DIR}/test/fake_rtkrcv.sh")
```

- [ ] **Step 3: 确认 RED（对桩）**

桩：`child_running()` 返回 `false`；`rtkrcv_health.hpp` 的 `evaluate_rtkrcv_health` 返回默认构造的状态（level 0 = OK、空 message）；`rtkrcv_node.cpp` 不改。

Run: `colcon build --symlink-install --packages-select gnss_bringup && ./build/gnss_bringup/test_rtkrcv_health; ./build/gnss_bringup/test_process_supervisor --gtest_filter='*ChildRunning*'; ./build/gnss_bringup/test_rtkrcv_node_health_process`
Expected: `RecentSolutionIsOk`（name 断言）FAIL，其余健康纯函数用例 FAIL；`ChildRunningTracksTheChildLifetime` FAIL；`ChildRunningIsFalseWhileWaitingToRestart` 对桩 PASS（Step 5 变异验证）；3 个节点级用例 FAIL（没有话题）。

- [ ] **Step 4: 实现**

`process_supervisor.hpp`，`start_failure_count()` 之后：

```cpp
  // 子进程已派生且尚未被回收(退避等待重启期间为 false)。供健康检查用;
  // 与 child_pid_ 同一个原子量,只是一瞬间的快照。
  bool child_running() const { return child_pid_.load() > 0; }
```

Create `include/gnss_bringup/rtkrcv_health.hpp`：

```cpp
#pragma once
// rtkrcv_node 自身健康(轮 3b 设计决定 9),1 Hz 发布到 ~/diagnostics。纯函数:节点只负责采样输入。
//   ERROR:rtkrcv 子进程未在运行(崩溃后退避等待重启,或解析不到二进制)
//   WARN :超过 no_solution_warn_s 没有解算行;区分"上行也没数据"(链路问题)与
//          "上行有数据但无解"(conf 问题:base_pos_type 所需的 1005/1006、obs_format)
//   OK   :其余;还没出过解且未超过门限时提示"等待首条解算输出"
#include <cstdio>
#include <optional>
#include <string>
#include <utility>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>

namespace gnss_bringup {

struct RtkrcvHealthInput {
  bool child_running = false;
  double since_start_s = 0.0;                // 节点起来多久
  std::optional<double> since_solution_s;    // 距最近一条解算行;从未收到为空
  std::optional<double> since_uplink_s;      // 距最近一次上行(差分或观测)字节;从未收到为空
};

inline diagnostic_msgs::msg::DiagnosticStatus evaluate_rtkrcv_health(const RtkrcvHealthInput& in,
                                                                     double no_solution_warn_s) {
  using S = diagnostic_msgs::msg::DiagnosticStatus;
  S st;
  st.name = "rtkrcv_node";
  st.hardware_id = "rtkrcv";
  const auto add = [&st](const char* key, const std::optional<double>& v) {
    diagnostic_msgs::msg::KeyValue kv;
    kv.key = key;
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%.1f", v.value_or(0.0));
    kv.value = v ? buf : "-";
    st.values.push_back(std::move(kv));
  };
  diagnostic_msgs::msg::KeyValue running;
  running.key = "child_running";
  running.value = in.child_running ? "true" : "false";
  st.values.push_back(running);
  add("since_solution_s", in.since_solution_s);
  add("since_uplink_s", in.since_uplink_s);

  char msg[256];
  if (!in.child_running) {
    st.level = S::ERROR;
    st.message = "rtkrcv 子进程未在运行——见节点日志里的退出记录";
    return st;
  }
  const double quiet_s = in.since_solution_s.value_or(in.since_start_s);
  if (quiet_s > no_solution_warn_s) {
    st.level = S::WARN;
    const bool uplink = in.since_uplink_s && *in.since_uplink_s <= no_solution_warn_s;
    std::snprintf(msg, sizeof(msg),
                  uplink ? "%.0f s 没有解算输出,上行有数据但无解——检查 base_pos_type 所需的 RTCM 1005/1006 与 obs_format"
                         : "%.0f s 没有解算输出,上行无数据——检查 rtcm_bridge 与平台/板卡链路",
                  quiet_s);
    st.message = msg;
    return st;
  }
  st.level = S::OK;
  st.message = in.since_solution_s ? "解算输出正常" : "等待首条解算输出";
  return st;
}

}  // namespace gnss_bringup
```

`src/rtkrcv_node.cpp`：
1. include 区加 `#include <diagnostic_msgs/msg/diagnostic_array.hpp>` 与 `#include "gnss_bringup/rtkrcv_health.hpp"`。
2. 文件头"接线顺序"列表末尾加一项 `//   9. 1 Hz 发布 ~/diagnostics:子进程是否在跑、多久没有解算行、上行是否有数据(rtkrcv_health.hpp)`。
3. 匿名命名空间（或类内 static）加：

```cpp
inline int64_t steady_now_ns() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             std::chrono::steady_clock::now().time_since_epoch()).count();
}
```

4. `read_params()` 里 `stat_poll_interval_s_ = ...` 之后加 `health_no_solution_warn_s_ = declare_positive_seconds("health_no_solution_warn_s", 30.0);`
5. 构造函数 `try` 块最前面（`read_params();` 之前）加 `started_ns_ = steady_now_ns();`；`start_stat_tailer();` 之后加 `start_health_publisher();`。
6. `on_solution_bytes` 里 `parse_llh_solution` 成功之后（`sol_log_gate_.on_solution_line()` 那个块之前）加 `last_solution_ns_.store(steady_now_ns());`。
7. `subscribe_uplink_streams` 两个回调里 `broadcast(...)` 之前各加 `if (!msg->data.empty()) last_uplink_ns_.store(steady_now_ns());`。
8. `connect_solution_stream` 里 terminal 分支注释"这个节点不会自动重启它,也没有健康检查话题,现场只能靠这行日志发现——必须是 ERROR。"改成"这个节点不会自动重启它;~/diagnostics 随后会因为没有解算行转为 WARN,但原因只在这行日志里——必须是 ERROR。"
9. 新增 Step（放在 Step 7 之后）：

```cpp
  // ---------- Step 8: 健康状态 ----------
  // 墙钟定时器:健康看的是真实流逝的时间,不跟随 sim time。回调在 spin 线程上,只读原子量。
  void start_health_publisher() {
    health_pub_ = node_->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("~/diagnostics", 10);
    health_timer_ = node_->create_wall_timer(std::chrono::seconds(1), [this] { publish_health(); });
  }

  void publish_health() {
    const int64_t now_ns = steady_now_ns();
    const auto since = [now_ns](int64_t t_ns) -> std::optional<double> {
      if (t_ns == 0) return std::nullopt;
      return static_cast<double>(now_ns - t_ns) * 1e-9;
    };
    RtkrcvHealthInput in;
    in.child_running = supervisor_ && supervisor_->child_running();
    in.since_start_s = static_cast<double>(now_ns - started_ns_) * 1e-9;
    in.since_solution_s = since(last_solution_ns_.load());
    in.since_uplink_s = since(last_uplink_ns_.load());
    diagnostic_msgs::msg::DiagnosticArray arr;
    arr.header.stamp = node_->now();
    arr.status.push_back(evaluate_rtkrcv_health(in, health_no_solution_warn_s_));
    health_pub_->publish(arr);
  }
```

10. 成员区加：

```cpp
  double health_no_solution_warn_s_ = 30.0;
  int64_t started_ns_ = 0;
  std::atomic<int64_t> last_solution_ns_{0};   // TcpStream 线程写,spin 线程读;0 = 从未
  std::atomic<int64_t> last_uplink_ns_{0};     // 订阅回调写
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr health_pub_;
  rclcpp::TimerBase::SharedPtr health_timer_;
```

`gnss_bringup.yaml` 的 `rtkrcv_node` 段、`stat_poll_interval_s` 之后加（yaml 在 Task 7 统一整理，这里先加这一项让默认配置可用）：

```yaml
    # ~/diagnostics(1 Hz)在超过这么多秒没有解算行时转为 WARN。隧道内没有解属正常,
    # 这个 WARN 在隧道里是预期的;开阔天空下持续 WARN 才需要排查。
    health_no_solution_warn_s: 30.0
```

- [ ] **Step 5: GREEN + 变异 + 全量**

Run: Step 3 的三个测试二进制全部 PASS；节点级测试连跑 3 次。
变异（逐个做、逐个恢复）：
1. `child_running()` 改成 `return child_pid_.load() != 0;` → `ChildRunningIsFalseWhileWaitingToRestart` 必须 FAIL（回收后是 -1）。
2. 健康纯函数里 `*in.since_uplink_s <= no_solution_warn_s` 改成 `true` → `SilentSolverWithoutUplinkPointsAtTheLink` 必须 FAIL。
3. `on_solution_bytes` 里删掉 `last_solution_ns_.store(...)` → `SolutionLinesMakeItOk` 必须 FAIL。

- [ ] **Step 6: 提交**

```bash
cd /home/steve/glim_ws/src/glim_ext
git add gnss_bringup/include/gnss_bringup/rtkrcv_health.hpp gnss_bringup/test/test_rtkrcv_health.cpp \
  gnss_bringup/include/gnss_bringup/process_supervisor.hpp gnss_bringup/test/test_process_supervisor.cpp \
  gnss_bringup/src/rtkrcv_node.cpp gnss_bringup/test/test_rtkrcv_node_health_process.cpp \
  gnss_bringup/CMakeLists.txt gnss_bringup/config/gnss_bringup.yaml
git commit -m "feat(gnss_bringup): rtkrcv_node publishes its own health on ~/diagnostics

<RED(桩)、三个变异、3 次连跑耗时>

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 6: `gnss_cleanup_node`

落实设计决定 8 与 3a 遗留 B"清理"。

**Files:**
- Modify: `glim_ext/gnss_core/include/gnss_core/retention.hpp`、`src/retention.cpp`（`disk_used_pct`）
- Test: `glim_ext/gnss_core/test/test_retention.cpp`
- Create: `glim_ext/gnss_bringup/include/gnss_bringup/cleanup_params.hpp`、`test/test_cleanup_params.cpp`
- Create: `glim_ext/gnss_bringup/src/gnss_cleanup_node.cpp`、`test/test_gnss_cleanup_node_process.cpp`
- Modify: `glim_ext/gnss_bringup/CMakeLists.txt`

**Interfaces:**
- Produces:
  - `std::optional<double> gnss_core::disk_used_pct(const std::string& path)`（0–100；`fs::space` 失败或容量为 0 时为空）
  - `struct CleanupParams { std::string bag_root, pos_root; int retention_days = 14; double watermark_pct = 85.0; double interval_s = 3600.0; }`
  - `void validate_cleanup_params(const CleanupParams&)`（抛 `std::invalid_argument`，消息以参数名开头）
  - `struct RootCleanupResult { std::string kind; std::string root; bool missing; gnss_core::CleanupReport report; std::optional<double> used_pct_after; }`
  - `std::vector<RootCleanupResult> run_cleanup_pass(const CleanupParams&, int today_yyyymmdd, const std::function<std::optional<double>(const std::string&)>& used_pct = gnss_core::disk_used_pct)`
  - `bool over_watermark(const RootCleanupResult&, double watermark_pct)`
  - 节点 `gnss_cleanup`，参数 `bag_root`、`pos_root`、`retention_days`、`watermark_pct`、`interval_s`；日志锚点 `已删除 <root>/<name>`、`清理完成一轮`、`启动失败,配置有误`

- [ ] **Step 1: 写失败测试（`disk_used_pct`）**

`gnss_core/test/test_retention.cpp` 末尾追加：

```cpp
TEST(Retention, DiskUsedPctOfARealPathAndOfAMissingOne) {
  const char* base = std::getenv("TMPDIR");
  const std::string dir = base ? base : "/tmp";
  const auto pct = disk_used_pct(dir);
  ASSERT_TRUE(pct.has_value());
  EXPECT_GE(*pct, 0.0);
  EXPECT_LE(*pct, 100.0);
  EXPECT_FALSE(disk_used_pct(dir + "/definitely_absent_retention_dir_xyz").has_value())
      << "查不到用量时要让调用方知道,而不是当成 0%";
}
```

- [ ] **Step 2: 写失败测试（`cleanup_params`）**

Create `gnss_bringup/test/test_cleanup_params.cpp`：

```cpp
#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include "gnss_bringup/cleanup_params.hpp"
#include "node_process_harness.hpp"   // make_temp_dir
using namespace gnss_bringup;
using gnss_bringup_test::make_temp_dir;
namespace fs = std::filesystem;

namespace {
const int TODAY = 20260910;

bool rejected(const CleanupParams& p, const std::string& field) {
  try {
    validate_cleanup_params(p);
  } catch (const std::invalid_argument& e) {
    return std::string(e.what()).rfind(field, 0) == 0;
  }
  return false;
}

std::optional<double> low_usage(const std::string&) { return 10.0; }
}  // namespace

TEST(CleanupParams, ValidatesEveryField) {
  CleanupParams ok;
  ok.bag_root = "/b";
  EXPECT_NO_THROW(validate_cleanup_params(ok));
  CleanupParams p = ok;
  p.bag_root.clear();
  EXPECT_TRUE(rejected(p, "bag_root")) << "两个根目录不能都为空";
  p = ok; p.retention_days = 0;      EXPECT_TRUE(rejected(p, "retention_days"));
  p = ok; p.watermark_pct = 0.0;     EXPECT_TRUE(rejected(p, "watermark_pct"));
  p = ok; p.watermark_pct = 100.5;   EXPECT_TRUE(rejected(p, "watermark_pct"));
  p = ok; p.watermark_pct = std::numeric_limits<double>::quiet_NaN(); EXPECT_TRUE(rejected(p, "watermark_pct"));
  p = ok; p.interval_s = 0.5;        EXPECT_TRUE(rejected(p, "interval_s"));
}

TEST(CleanupPass, BagRootFirstThenPosRootAndEmptyRootsAreSkipped) {
  const auto base = make_temp_dir("cleanup_pass_");
  ASSERT_FALSE(base.empty());
  for (const char* d : {"bags/gnss_20260801_000000", "bags/gnss_20260910_120000", "pos/20260801", "pos/20260910"}) {
    fs::create_directories(base + "/" + d);
  }
  CleanupParams p;
  p.bag_root = base + "/bags";
  p.pos_root = base + "/pos";
  p.watermark_pct = 100.0;

  std::vector<std::string> usage_queries;
  const auto results = run_cleanup_pass(p, TODAY, [&](const std::string& root) {
    usage_queries.push_back(root);
    return std::optional<double>(10.0);
  });
  ASSERT_EQ(results.size(), 2u);
  EXPECT_EQ(results[0].kind, "bag");
  EXPECT_EQ(results[0].report.deleted, (std::vector<std::string>{"gnss_20260801_000000"}));
  EXPECT_EQ(results[1].kind, "pos");
  EXPECT_EQ(results[1].report.deleted, (std::vector<std::string>{"20260801"}));
  EXPECT_EQ(usage_queries, (std::vector<std::string>{p.bag_root, p.pos_root})) << "录包占盘大,先扫录包根目录";
  EXPECT_TRUE(fs::exists(base + "/bags/gnss_20260910_120000"));
  EXPECT_TRUE(fs::exists(base + "/pos/20260910"));

  p.pos_root.clear();
  EXPECT_EQ(run_cleanup_pass(p, TODAY, low_usage).size(), 1u);
  fs::remove_all(base);
}

TEST(CleanupPass, MissingRootIsSkippedButABrokenOneIsAnError) {
  const auto base = make_temp_dir("cleanup_pass_missing_");
  ASSERT_FALSE(base.empty());
  fs::create_directories(base + "/pos/20260801");
  fs::create_directories(base + "/pos/20260910");
  std::ofstream(base + "/not_a_dir") << "x";
  CleanupParams p;
  p.bag_root = base + "/never_recorded";
  p.pos_root = base + "/pos";
  p.watermark_pct = 100.0;
  auto results = run_cleanup_pass(p, TODAY, low_usage);
  ASSERT_EQ(results.size(), 2u);
  EXPECT_TRUE(results[0].missing) << "还没录过包时根目录不存在是正常的";
  EXPECT_TRUE(results[0].report.error.empty());
  EXPECT_EQ(results[1].report.deleted, (std::vector<std::string>{"20260801"})) << "一个根目录的问题不影响另一个";

  p.bag_root = base + "/not_a_dir";
  results = run_cleanup_pass(p, TODAY, low_usage);
  EXPECT_FALSE(results[0].missing);
  EXPECT_FALSE(results[0].report.error.empty());
  fs::remove_all(base);
}

TEST(CleanupPass, OverWatermarkUsesTheUsageAfterCleaning) {
  RootCleanupResult r;
  r.used_pct_after = 90.0;
  EXPECT_TRUE(over_watermark(r, 85.0));
  r.used_pct_after = 85.0;
  EXPECT_FALSE(over_watermark(r, 85.0));
  r.used_pct_after.reset();
  EXPECT_FALSE(over_watermark(r, 85.0)) << "查不到用量不报超水位";
}
```

- [ ] **Step 3: 写失败测试（节点级）**

Create `gnss_bringup/test/test_gnss_cleanup_node_process.cpp`：

```cpp
#include <gtest/gtest.h>
#include <unistd.h>

#include <ctime>
#include <filesystem>
#include <string>
#include <vector>

#include "gnss_core/retention.hpp"
#include "node_process_harness.hpp"
using namespace gnss_bringup_test;
namespace fs = std::filesystem;

namespace {
std::vector<std::pair<std::string, std::string>> isolated_env(const std::string& dir) {
  return {{"ROS_DOMAIN_ID", std::to_string(30 + ::getpid() % 10)}, {"ROS_LOG_DIR", dir + "/roslog"}};
}
}  // namespace

TEST(GnssCleanupNodeProcess, RefusesToStartWithoutAnyRoot) {
  const auto dir = make_temp_dir("cleanup_node_bad_");
  ASSERT_FALSE(dir.empty());
  NodeProcess node(GNSS_CLEANUP_NODE_PATH, {"--ros-args"}, dir + "/node.log", isolated_env(dir));
  EXPECT_EQ(node.wait_exit(20.0), 1) << node.log();
  EXPECT_NE(node.log().find("启动失败,配置有误"), std::string::npos) << node.log();
  EXPECT_NE(node.log().find("bag_root"), std::string::npos) << node.log();
  fs::remove_all(dir);
}

TEST(GnssCleanupNodeProcess, CleansOnStartupAndKeepsRecentEntries) {
  const auto dir = make_temp_dir("cleanup_node_run_");
  ASSERT_FALSE(dir.empty());
  const std::string today = std::to_string(gnss_core::utc_yyyymmdd(static_cast<double>(std::time(nullptr))));
  for (const auto& d : std::vector<std::string>{"bags/gnss_20200101_000000", "bags/gnss_" + today + "_000000",
                                                "pos/20200101", "pos/" + today}) {
    fs::create_directories(dir + "/" + d);
  }
  NodeProcess node(GNSS_CLEANUP_NODE_PATH,
                   {"--ros-args", "-p", "bag_root:=" + dir + "/bags", "-p", "pos_root:=" + dir + "/pos",
                    "-p", "watermark_pct:=100.0"},
                   dir + "/node.log", isolated_env(dir));
  EXPECT_TRUE(wait_until([&] { return node.log().find("清理完成一轮") != std::string::npos; }, 20.0)) << node.log();
  EXPECT_FALSE(fs::exists(dir + "/bags/gnss_20200101_000000")) << node.log();
  EXPECT_FALSE(fs::exists(dir + "/pos/20200101")) << node.log();
  EXPECT_TRUE(fs::exists(dir + "/bags/gnss_" + today + "_000000"));
  EXPECT_TRUE(fs::exists(dir + "/pos/" + today));
  EXPECT_NE(node.log().find("已删除 " + dir + "/bags/gnss_20200101_000000"), std::string::npos) << node.log();
  node.interrupt();
  EXPECT_EQ(node.wait_exit(20.0), 0) << node.log();
  fs::remove_all(dir);
}
```

`CMakeLists.txt`：
- `gnss_diag_node` 的 `install` 之后加：

```cmake
add_executable(gnss_cleanup_node src/gnss_cleanup_node.cpp)
target_link_libraries(gnss_cleanup_node gnss_core::gnss_core)
ament_target_dependencies(gnss_cleanup_node rclcpp)
install(TARGETS gnss_cleanup_node DESTINATION lib/${PROJECT_NAME})
```

- `BUILD_TESTING` 块末尾加：

```cmake
  # cleanup_params.hpp:清理节点的参数校验与一轮清理(先录包根目录,再 .pos 根目录)
  ament_add_gtest(test_cleanup_params test/test_cleanup_params.cpp)
  target_link_libraries(test_cleanup_params gnss_core::gnss_core)
  target_include_directories(test_cleanup_params PRIVATE include test)

  ament_add_gtest(test_gnss_cleanup_node_process test/test_gnss_cleanup_node_process.cpp TIMEOUT 120)
  target_link_libraries(test_gnss_cleanup_node_process gnss_core::gnss_core)
  target_include_directories(test_gnss_cleanup_node_process PRIVATE include test)
  add_dependencies(test_gnss_cleanup_node_process gnss_cleanup_node)
  target_compile_definitions(test_gnss_cleanup_node_process PRIVATE
    GNSS_CLEANUP_NODE_PATH="$<TARGET_FILE:gnss_cleanup_node>")
```

- [ ] **Step 4: 确认 RED（对桩）**

桩：`disk_used_pct` 返回 `std::nullopt`；`cleanup_params.hpp` 按 Interfaces 声明，`validate_cleanup_params` 空函数体、`run_cleanup_pass` 返回 `{}`、`over_watermark` 返回 `false`；`gnss_cleanup_node.cpp` 只 `init` → `spin` → `shutdown`。

Run: `colcon build --symlink-install --packages-select gnss_core gnss_bringup && ./build/gnss_core/test_retention; ./build/gnss_bringup/test_cleanup_params; ./build/gnss_bringup/test_gnss_cleanup_node_process`
Expected: `DiskUsedPctOfARealPathAndOfAMissingOne` FAIL；`test_cleanup_params` 4 个用例全部 FAIL；两个节点级用例 FAIL（桩不退出/不删除）。

- [ ] **Step 5: 实现**

`retention.hpp`，`cleanup_dated_root` 声明之前：

```cpp
// path 所在文件系统的已用百分比(0–100)。fs::space 失败或容量为 0 时为空——
// 调用方要能区分"查不到"与"用了 0%"(3a 遗留 C:以前静默退化为只按天数删除)。
std::optional<double> disk_used_pct(const std::string& path);
```

`retention.cpp`：

```cpp
std::optional<double> disk_used_pct(const std::string& path) {
  std::error_code ec;
  const auto info = std::filesystem::space(path, ec);
  if (ec || info.capacity == 0) return std::nullopt;
  return static_cast<double>(info.capacity - info.free) / static_cast<double>(info.capacity) * 100.0;
}
```

`cleanup_dated_root` 里的 `used_pct` lambda 改为：

```cpp
  const auto used_pct = [&root]() {
    return disk_used_pct(root).value_or(0.0);   // 查不到用量时只按保留天数删(节点清理后会复查并报出)
  };
```

Create `gnss_bringup/include/gnss_bringup/cleanup_params.hpp`：

```cpp
#pragma once
// gnss_cleanup_node 的参数校验与一轮清理(轮 3b 设计决定 8)。不碰 ROS:节点只负责定时与打日志。
//   - 两个根目录:bag_root(rosbag2 录包,gnss_YYYYMMDD_HHMMSS)与 pos_root(.pos 与诊断文件,YYYYMMDD);
//     各自可为空(不清),不能都空。
//   - 先扫录包根目录再扫 pos 根目录:两者通常在同一块盘上共用水位,录包占盘大,先删它。
//   - 根目录不存在(比如还没录过包)视为跳过,不算错误;存在但遍历失败(比如是个普通文件)才是错误。
//   - 每个根目录清完后复查所在盘的用量,仍高于水位由调用方报警(最新一项与今天的数据永不删)。
#include <cmath>
#include <filesystem>
#include <functional>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "gnss_core/retention.hpp"

namespace gnss_bringup {

struct CleanupParams {
  std::string bag_root;
  std::string pos_root;
  int retention_days = 14;
  double watermark_pct = 85.0;
  double interval_s = 3600.0;
};

inline void validate_cleanup_params(const CleanupParams& p) {
  if (p.bag_root.empty() && p.pos_root.empty()) {
    throw std::invalid_argument("bag_root 与 pos_root 不能都为空");
  }
  if (p.retention_days < 1) throw std::invalid_argument("retention_days 必须 >= 1");
  if (!std::isfinite(p.watermark_pct) || p.watermark_pct <= 0.0 || p.watermark_pct > 100.0) {
    throw std::invalid_argument("watermark_pct 必须在 (0, 100]");
  }
  if (!std::isfinite(p.interval_s) || p.interval_s < 1.0) throw std::invalid_argument("interval_s 必须 >= 1");
}

struct RootCleanupResult {
  std::string kind;     // "bag" 或 "pos"
  std::string root;
  bool missing = false; // 根目录不存在,本轮跳过
  gnss_core::CleanupReport report;
  std::optional<double> used_pct_after;   // 清完后所在盘的用量;查不到为空
};

inline std::vector<RootCleanupResult> run_cleanup_pass(
    const CleanupParams& p, int today_yyyymmdd,
    const std::function<std::optional<double>(const std::string&)>& used_pct = gnss_core::disk_used_pct) {
  std::vector<RootCleanupResult> out;
  const auto one = [&](const char* kind, const std::string& root,
                       const std::function<std::optional<int>(const std::string&)>& parse_date) {
    if (root.empty()) return;
    RootCleanupResult r;
    r.kind = kind;
    r.root = root;
    std::error_code ec;
    if (!std::filesystem::exists(root, ec) && !ec) {
      r.missing = true;
    } else {
      r.report = gnss_core::cleanup_dated_root(root, parse_date, today_yyyymmdd, p.retention_days, p.watermark_pct);
      r.used_pct_after = used_pct(root);
    }
    out.push_back(std::move(r));
  };
  one("bag", p.bag_root, gnss_core::parse_bag_dir_date);
  one("pos", p.pos_root, gnss_core::parse_day_dir_date);
  return out;
}

inline bool over_watermark(const RootCleanupResult& r, double watermark_pct) {
  return r.used_pct_after && *r.used_pct_after > watermark_pct;
}

}  // namespace gnss_bringup
```

Create `gnss_bringup/src/gnss_cleanup_node.cpp`：

```cpp
// gnss_cleanup_node:按保留天数与磁盘水位清理录包与 .pos/诊断目录(spec §3 A6/D4、§5.2,轮 3b)。
// 启动时先跑一轮,之后每 interval_s 一次。定时器与"今天"都用墙钟:删数据按真实日期,
// 不跟随回放时的 sim time。判定逻辑在 gnss_core::retention 与 cleanup_params.hpp。
#include <chrono>
#include <climits>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <stdexcept>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "gnss_bringup/cleanup_params.hpp"

namespace {

void run_pass(rclcpp::Node& node, const gnss_bringup::CleanupParams& p) {
  const double now_s =
      std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
  const int today = gnss_core::utc_yyyymmdd(now_s);
  size_t deleted = 0;
  for (const auto& r : gnss_bringup::run_cleanup_pass(p, today)) {
    if (r.missing) {
      RCLCPP_INFO(node.get_logger(), "%s 根目录不存在,跳过: %s", r.kind.c_str(), r.root.c_str());
      continue;
    }
    if (!r.report.error.empty()) {
      RCLCPP_WARN(node.get_logger(), "清理 %s 根目录没有完成: %s", r.kind.c_str(), r.report.error.c_str());
    }
    for (const auto& name : r.report.deleted) {
      RCLCPP_INFO(node.get_logger(), "已删除 %s/%s", r.root.c_str(), name.c_str());
    }
    deleted += r.report.deleted.size();
    if (gnss_bringup::over_watermark(r, p.watermark_pct)) {
      RCLCPP_WARN(node.get_logger(),
                  "%s 所在磁盘清理后仍占用 %.1f%%(水位 %.1f%%)——今天的数据与最新一项不删,需要人工处理",
                  r.root.c_str(), *r.used_pct_after, p.watermark_pct);
    }
  }
  RCLCPP_INFO(node.get_logger(), "清理完成一轮:删除 %zu 项", deleted);
}

}  // namespace

int main(int argc, char** argv) {
  std::shared_ptr<rclcpp::Node> node;
  gnss_bringup::CleanupParams p;
  try {
    rclcpp::init(argc, argv);
    node = std::make_shared<rclcpp::Node>("gnss_cleanup");
    p.bag_root = node->declare_parameter<std::string>("bag_root", "");
    p.pos_root = node->declare_parameter<std::string>("pos_root", "");
    const int64_t days = node->declare_parameter<int>("retention_days", p.retention_days);
    if (days < INT_MIN || days > INT_MAX) throw std::invalid_argument("retention_days 超出范围");
    p.retention_days = static_cast<int>(days);
    p.watermark_pct = node->declare_parameter<double>("watermark_pct", p.watermark_pct);
    p.interval_s = node->declare_parameter<double>("interval_s", p.interval_s);
    gnss_bringup::validate_cleanup_params(p);
  } catch (const std::exception& e) {
    if (node) {
      RCLCPP_ERROR(node->get_logger(), "启动失败,配置有误: %s", e.what());
    } else {
      std::fprintf(stderr, "gnss_cleanup: 启动失败,配置有误: %s\n", e.what());
    }
    if (rclcpp::ok()) rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(node->get_logger(), "gnss_cleanup 已启动: bag_root=%s pos_root=%s 保留 %d 天 水位 %.1f%% 间隔 %.0f s",
              p.bag_root.empty() ? "-" : p.bag_root.c_str(), p.pos_root.empty() ? "-" : p.pos_root.c_str(),
              p.retention_days, p.watermark_pct, p.interval_s);
  run_pass(*node, p);
  const auto timer = node->create_wall_timer(
      std::chrono::milliseconds(static_cast<int64_t>(p.interval_s * 1000.0)), [&node, &p] { run_pass(*node, p); });
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
```

- [ ] **Step 6: GREEN + 变异 + 全量**

Run: Step 4 的三个测试二进制 PASS，然后全量。
变异（逐个做、逐个恢复）：
1. `run_cleanup_pass` 里交换 `one("bag", ...)` 与 `one("pos", ...)` 两行 → `BagRootFirstThenPosRootAndEmptyRootsAreSkipped` 必须 FAIL。
2. `run_cleanup_pass` 里去掉 `r.missing = true;` 分支（一律调用 `cleanup_dated_root`）→ `MissingRootIsSkippedButABrokenOneIsAnError` 必须 FAIL。
3. `over_watermark` 里 `>` 改成 `>=` → `OverWatermarkUsesTheUsageAfterCleaning` 必须 FAIL。

- [ ] **Step 7: 提交**

```bash
cd /home/steve/glim_ws/src/glim_ext
git add gnss_core/include/gnss_core/retention.hpp gnss_core/src/retention.cpp gnss_core/test/test_retention.cpp \
  gnss_bringup/include/gnss_bringup/cleanup_params.hpp gnss_bringup/test/test_cleanup_params.cpp \
  gnss_bringup/src/gnss_cleanup_node.cpp gnss_bringup/test/test_gnss_cleanup_node_process.cpp \
  gnss_bringup/CMakeLists.txt
git commit -m "feat(gnss_bringup): gnss_cleanup_node sweeps bags then .pos by retention and watermark

<RED(桩)、三个变异的结果>

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 7: launch、yaml、README 与遗留文档

**Files:**
- Modify: `glim_ext/gnss_bringup/launch/gnss_bringup.launch.py`
- Modify: `glim_ext/gnss_bringup/config/gnss_bringup.yaml`
- Modify: `glim_ext/gnss_bringup/README.md`
- Modify: `glim_ext/gnss_bringup/scripts/record_gnss.sh`（文件头"缺口"注释）
- Modify: `glim_underground/docs/gnss/specs/2026-09-15-round3a-followups.md`

**Interfaces:**
- Consumes: Task 4 的 `gnss_diag_node`（节点名 `gnss_diag`）、Task 5 的 `health_no_solution_warn_s`、Task 6 的 `gnss_cleanup_node`（节点名 `gnss_cleanup`）
- Produces: launch 参数 `enable_diag`（默认 `"true"`）、`enable_cleanup`（默认 `"true"`）、`bag_root`（默认 `$GNSS_BAG_ROOT`，未设置时 `$HOME/gnss_bags`）；`enable_rtkrcv` 默认仍为 `"false"`

本 Task 没有 gtest（launch 文件无法用 gtest 覆盖，launch_testing 在开发机不可用），验证靠 Step 4 的冒烟运行，输出贴进 commit message。

- [ ] **Step 1: launch**

`launch/gnss_bringup.launch.py`：
- import 改为：

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
```

- `DeclareLaunchArgument("enable_pos_writer", ...)` 之后加：

```python
        # 诊断与清理默认随桥一起起来(与 pos_writer 相同):events.log / base.pos 与磁盘清理
        # 是记录面常态开启的一部分。rtkrcv 没启用时诊断把 no_solution 记成 info,不开事件。
        DeclareLaunchArgument("enable_diag", default_value="true"),
        DeclareLaunchArgument("enable_cleanup", default_value="true"),
        # 录包根目录:与 record_gnss.sh 同一套默认(先看 GNSS_BAG_ROOT,再退回 $HOME/gnss_bags)
        DeclareLaunchArgument(
            "bag_root",
            default_value=EnvironmentVariable(
                "GNSS_BAG_ROOT", default_value=PathJoinSubstitution([EnvironmentVariable("HOME"), "gnss_bags"]))),
```

- `pos_writer` 的 `Node(...)` 之后加：

```python
        # solver_enabled 跟随 enable_rtkrcv,不在 yaml 里重复写一份(两处容易改漏)
        Node(package="gnss_bringup", executable="gnss_diag_node", name="gnss_diag",
             parameters=[LaunchConfiguration("params_file"),
                         {"solver_enabled": ParameterValue(LaunchConfiguration("enable_rtkrcv"), value_type=bool)}],
             output="screen",
             condition=IfCondition(LaunchConfiguration("enable_diag"))),
        Node(package="gnss_bringup", executable="gnss_cleanup_node", name="gnss_cleanup",
             parameters=[LaunchConfiguration("params_file"), {"bag_root": LaunchConfiguration("bag_root")}],
             output="screen",
             condition=IfCondition(LaunchConfiguration("enable_cleanup"))),
```

- [ ] **Step 2: yaml**

`config/gnss_bringup.yaml` 末尾追加：

```yaml

gnss_diag:
  ros__parameters:
    # 诊断落盘根目录:<root>/YYYYMMDD/events.log(事件)、<root>/YYYYMMDD/base.pos(基站坐标史)、
    # <root>/base_baseline(基站基线)。与 pos_writer.root 相同,诊断文件与 .pos 落在同一个
    # 日期目录里(spec §5.3),清理节点也按同一个目录一起清。
    root: "/data/gnss/pos"
    # 四路输入。solution_topic 需要 enable_rtkrcv:=true 才有数据;stat_topic 同样来自 rtkrcv_node。
    corrections_topic: "/gnss/rtcm_corrections"
    solution_topic: "/rtkrcv_node/rtk_fix"
    device_topic: "/gnss_cgi610/rtk_fix"
    stat_topic: "/rtkrcv_node/stat"
    # solver_enabled 由 launch 按 enable_rtkrcv 注入,这里不写。
    #
    # 注意:下面的秒数/米数一律写成带小数点的形式(60.0 而不是 60)——ROS 参数类型严格,
    # 写成整数会在启动时因类型不符拒绝启动。
    #
    # 启动(以及时钟回跳重建引擎)后的宽限期:只接收数据、不判定,避免 rtkrcv 收敛前每次开机
    # 都记一条 no_solution。代价:宽限期内真实的差分中断等故障晚这么多秒才记录。
    startup_grace_s: 60.0
    # ROS 时间回退超过这么多秒视为时钟回跳(回放 bag 重新开始、系统校时):关闭已开事件并重建引擎
    clock_jump_tolerance_s: 1.0
    # 两路解都在到达、却连续这么多秒没配上对时打一次 WARN(device_divergence 此时无法判定)
    unpaired_warn_s: 60.0
    # 控制点(abs_ref_shift 规则):三个等长数组。没有控制点时整段保持注释——
    # ROS 无法从空列表推断类型(同 rtkrcv_node.args 的说明)。经纬度写成带小数点的数。
    # control_points:
    #   names: ["K1"]
    #   lat: [44.50123456]
    #   lon: [90.28765432]
    # 规则阈值,默认值与 gnss_core::DiagnosisConfig 相同(移植自 rtk-monitor,
    # 本项目新增项见 docs/gnss/plans/2026-09-15-round3a-diagnosis-core.md「设计决定」)
    diagnosis:
      corr_gap_s: 3.0
      age_max_s: 10.0
      base_shift_m: 0.1
      min_sats: 6
      resid_max_m: 2.0
      low_el_deg: 20.0
      low_snr_dbhz: 35.0
      min_ratio: 3.0
      slip_max_per_30s: 5
      divergence_sigma: 3.0
      divergence_hold_s: 5.0
      close_hysteresis_s: 10.0
      sol_stale_s: 5.0
      abs_ref_max_m: 0.2
      abs_ref_radius_m: 3.0
      divergence_window_s: 600.0
      divergence_min_samples: 60
      divergence_sigma_floor_m: 0.05
      # 学到的偏差 σ 上限(维护者 2026-09-15 决定):学习部分给出的阈值最多 3 × 0.10 = 0.30 m,
      # 更大的持续偏差不会被学成正常;rtkrcv 浮点解时的当前 σ 不受此限
      divergence_sigma_max_m: 0.10
      divergence_pair_max_dt_s: 2.0
      divergence_epoch_max_dt_s: 0.1
      base_warmup_s: 600.0

gnss_cleanup:
  ros__parameters:
    # bag_root 由 launch 注入(默认 $GNSS_BAG_ROOT,未设置时 $HOME/gnss_bags,与 record_gnss.sh 一致)。
    # .pos 与诊断文件的根目录,与 pos_writer.root / gnss_diag.root 相同
    pos_root: "/data/gnss/pos"
    # 超过这么多天的日期目录/录包删除;今天的、以及每个根目录下最新的一项永不删
    retention_days: 14
    # 所在盘用量高于这个百分比时,从最旧的开始继续删,直到低于水位(同样不删今天与最新一项)
    watermark_pct: 85.0
    # 启动时先清一轮,之后每隔这么多秒一轮
    interval_s: 3600.0
```

- [ ] **Step 3: README、脚本注释、遗留文档**

`README.md`：
1. 「rosbag2 录制」一节里"**缺口(spec A6,留给轮 3,这里只记录不实现)**……"整段替换为：
   `**清理**:rosbag2 本身不删旧 bag;由 \`gnss_cleanup_node\`(默认随 launch 启动)按保留天数与磁盘水位清理 \`bag_root\` 下的 \`gnss_YYYYMMDD_HHMMSS\` 与 \`.pos\` 根目录下的 \`YYYYMMDD\`,见「清理节点」一节。`
2. 在「rosbag2 录制」与「已验证 / 未验证项」之间新增四节：

```markdown
## 诊断节点 `gnss_diag_node`

订阅差分裸流、rtkrcv 独立解(`/rtkrcv_node/rtk_fix`)、610 融合解(`/gnss_cgi610/rtk_fix`)与 rtkrcv `$SAT` 流
(`/rtkrcv_node/stat`),每秒按九条规则判定一次(规则与阈值见 `gnss_core/diagnosis.hpp`)。

| 输出 | 位置 |
| --- | --- |
| 事件(开/关、峰值、持续时间) | `<root>/YYYYMMDD/events.log` |
| 基站坐标史(RTCM 1005/1006 坐标变化时追加) | `<root>/YYYYMMDD/base.pos` |
| 基站基线(预热 600 s 取中位数) | `<root>/base_baseline` |
| 状态 | `/gnss/diagnostics`(`diagnostic_msgs/DiagnosticArray`,`name: gnss_diag`) |

- **启动宽限期** `startup_grace_s`(60 s):开机或时钟回跳后先只收数据,避免 rtkrcv 收敛前每次记一条 `no_solution`。
- **时间**:判定与落盘用 ROS 时间;回放 bag 时加 `use_sim_time:=true`。ROS 时间回退超过 1 s 时,已开事件以
  `reason=shutdown` 关闭、引擎重建。
- **基站搬迁**:确认基站确实搬了之后,`ros2 service call /gnss_diag/reset_base_baseline std_srvs/srv/Trigger`
  以最近一次 1005/1006 坐标为新基线。
- **偏差规则 `device_divergence`** 只用两路都 FIXED 的样本学习正常偏差水平,学到的 σ 最多 0.10 m
  (`diagnosis.divergence_sigma_max_m`)。两路 `gnss_time` 必须都是 UTC,否则按历元配不上对,节点会打 WARN。
- 不需要诊断时 `enable_diag:=false`。

## 清理节点 `gnss_cleanup_node`

启动时清一轮,之后每 `interval_s`(3600 s)一轮:先 `bag_root`(录包,占盘大)再 `pos_root`;超过
`retention_days`(14 天)的删除,所在盘高于 `watermark_pct`(85%)时从最旧的继续删。今天的数据与每个根目录下
最新的一项永不删;清完仍高于水位会打 WARN,需要人工处理。`bag_root` 由 launch 参数给(默认
`$GNSS_BAG_ROOT` → `$HOME/gnss_bags`),与 `record_gnss.sh` 一致。不需要时 `enable_cleanup:=false`。

## `rtkrcv_node` 健康话题

`/rtkrcv_node/diagnostics`(1 Hz):`ERROR` = rtkrcv 子进程没在运行;`WARN` = 超过
`health_no_solution_warn_s`(30 s)没有解算行,消息里区分"上行无数据"(查链路)与"上行有数据但无解"
(查 `base_pos_type` 所需的 1005/1006、`obs_format`);其余 `OK`。隧道内 WARN 属预期。

## `RtkFix` 增加 `ratio`(2026-09 轮 3b)

`gnss_msgs/RtkFix` 末尾加了 `float32 ratio`(RTKLIB AR ratio,0 = 源不提供,610 驱动为 0)。
- **`glim_ws` 与 `driver_ws` 都要重编** `gnss_msgs` 及其下游(`glim_ws`:`gnss_bringup`、`glim_ext`;`driver_ws`:`gnss_CGI610`。
  `gnss_msgs` 是两边共用的符号链接,改一处两边都变);
  只重编一边会在话题上出现类型哈希不一致、收不到消息。
- **旧 bag 不兼容**:改动之前录的含 `RtkFix` 的 bag,用新定义回放时反序列化失败(CDR 末尾少 4 字节)。
  需要回放旧 bag 时,切回改动前的 `gnss_msgs` 构建。
```

3. 「参数说明」一节若有 rtkrcv_node 参数表，补一行 `health_no_solution_warn_s`；若没有表格则跳过（不要为此新建表）。

`scripts/record_gnss.sh` 文件头"缺口(spec A6,轮 3 做,这里只记录,不实现)……详见 README「rosbag2 录制」一节。"整段替换为：

```bash
# 清理:本脚本只管"怎么录"。旧 bag 由 gnss_cleanup_node(随 gnss_bringup.launch.py 默认启动)
# 按保留天数与磁盘水位删除,它的 bag_root 默认与这里的 GNSS_BAG_ROOT 一致。详见 README「清理节点」。
```

`glim_underground/docs/gnss/specs/2026-09-15-round3a-followups.md`：文件末尾追加：

```markdown

## 轮 3b 处理状态(2026-09-15,计划 `docs/gnss/plans/2026-09-15-round3b-diag-node.md`)

- A.1、A.2:已处理——只用两路都 FIXED 的样本学习,学到的 σ 上限 `divergence_sigma_max_m = 0.10`(Task 2)。
- B 全部已处理:ratio 加进 `RtkFix`(Task 1);`epoch_t` 由 `gnss_time > 0` 填入、配对持续失败打 WARN;
  ROS 时间 + 回跳检测重建引擎;单线程 executor;停机顺序;基线持久化、`last_history` 跨日期读取、
  `reset_base_baseline` 服务;清理先录包后 `.pos`、清完仍超水位打 WARN;启动宽限期 60 s(Task 3–6)。
- C 已顺带处理:偏差监测规则 5 注释、held 基线不存当前 σ 的测试(Task 2);`fs::space` 失败不再静默
  (`disk_used_pct`,Task 6)。其余 C 项仍未处理。
```

- [ ] **Step 4: 构建、全量与冒烟**

Run（全量）: `cd /home/steve/glim_ws && source /opt/ros/humble/setup.bash && colcon build --symlink-install --packages-up-to gnss_bringup && colcon test --packages-select gnss_core gnss_bringup && colcon test-result --all`
Expected: 0 failures。

Run（launch 参数）: `source install/setup.bash && ros2 launch gnss_bringup gnss_bringup.launch.py --show-args`
Expected: 列出 `enable_diag`、`enable_cleanup`、`bag_root`，`enable_rtkrcv` 默认 `'false'`。

Run（冒烟，隔离域与临时目录，不写 `/data`）:

```bash
cd /home/steve/glim_ws && source /opt/ros/humble/setup.bash && source install/setup.bash
SMOKE=$(mktemp -d)
export ROS_DOMAIN_ID=77 ROS_LOG_DIR=$SMOKE/roslog
sed -e "s#/data/gnss/pos#$SMOKE/pos#g" src/glim_ext/gnss_bringup/config/gnss_bringup.yaml > $SMOKE/params.yaml
mkdir -p $SMOKE/bags/gnss_20200101_000000
timeout -s INT 15 ros2 launch gnss_bringup gnss_bringup.launch.py params_file:=$SMOKE/params.yaml \
  bag_root:=$SMOKE/bags > $SMOKE/launch.log 2>&1 &
sleep 8
ros2 node list | sort
ros2 topic echo --once /gnss/diagnostics diagnostic_msgs/msg/DiagnosticArray | head -20
wait
grep -E "gnss_diag 已启动|gnss_cleanup 已启动|已删除|清理完成一轮|启动失败" $SMOKE/launch.log
ls $SMOKE/bags
rm -rf "$SMOKE"
```

Expected：
- 节点列表含 `/gnss_diag`、`/gnss_cleanup`、`/rtcm_bridge`、`/pos_writer`，不含 `/rtkrcv_node`；
- `/gnss/diagnostics` 收到一条（宽限期内 message 为"启动宽限期……"）；
- 日志里有 `gnss_diag 已启动`、`gnss_cleanup 已启动`、`清理完成一轮`，没有 `启动失败`、没有 `已删除`；
- `ls $SMOKE/bags` 仍列出 `gnss_20200101_000000`——它虽然超期，但是根目录下唯一（最新）的一项，永不删。若被删了，说明判定有误，停下报告。

- [ ] **Step 5: 提交（两个仓库）**

```bash
cd /home/steve/glim_ws/src/glim_ext
git add gnss_bringup/launch/gnss_bringup.launch.py gnss_bringup/config/gnss_bringup.yaml \
  gnss_bringup/README.md gnss_bringup/scripts/record_gnss.sh
git commit -m "feat(gnss_bringup): launch the diagnosis and cleanup nodes by default

<--show-args 与冒烟输出摘要>

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"

cd /home/steve/glim_ws/src/glim_underground
git add docs/gnss/specs/2026-09-15-round3a-followups.md
git status --short    # 暂存区只能有这一个文件;time_keeper.* 等维护者文件不能出现在暂存区
git commit -m "docs(gnss): record how round 3b resolved the round 3a follow-ups

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

## 完成标准

- 两个代码仓库各自的分支（`glim_ext: feat/round3b-diag-node`、`finder_ros: feat/rtkfix-ratio`）上提交齐全，未 push。
- `colcon test-result --all`：0 errors、0 failures；测试总数 = 477 + 本计划新增。
- 每个 Task 的 commit message 里有 RED（桩或旧代码）与变异证据。
- 冒烟运行输出与预期一致；`enable_rtkrcv` 默认仍为 `false`。
