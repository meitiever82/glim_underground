# 轮 3a:`gnss_core` 诊断核心 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在 ROS 无关的 `gnss_core` 里实现诊断全部算法：规则链（C1–C9）、按规则码独立的事件状态机（C10）、基站坐标监测、`events.log` / `base.pos` 文本落盘（D2/D3）、保留天数与磁盘水位清理判定（A6/D4），以及把这些部件串起来的 `DiagnosisEngine`，供轮 3b 的 ROS 节点做薄壳。

**Architecture:** 规则与参考实现 rtk-monitor 的 `diagnose()` 同优先级、同阈值、同中文消息，但返回**全部命中**的结论（状态取第一条）；事件机按规则码各一个迟滞跟踪器，纯函数式地返回开/关事件，不直接写文件；`DiagnosisEngine` 接收时间戳化的原始输入（差分字节、两路解、`$SAT` 行），每秒 `tick()` 一次产出结论与事件变更，落盘与定时由壳负责。

**Tech Stack:** C++17、Eigen、GeographicLib（已是 `gnss_core` 依赖）、gtest（经 `gnss_core_add_test`）。

**Spec:**
- `docs/gnss/specs/2026-09-03-gnss-glim-modules-design.md`（§3 C1–C10、D2–D4、A6；§5.3 目录布局；§8；§10 轮 3；§12.1「规则链 / 事件机沿用 rtk-monitor 既有用例」）
- 参考实现：`/home/steve/Documents/GitHub/gnss-alg/rtk-monitor`（HEAD `8a218515`）——`src/rtk_monitor/diagnosis/{rules,events,base_station}.py`、`storage/cleanup.py`、`config.py:24-41`、`tests/test_{rules,event_machine,base_station,cleanup,app_plan2}.py`
- 下面「设计决定」一节是本计划对 spec 与参考实现分歧处的裁定，执行者必须先读

## Global Constraints

- 代码仓库：`/home/steve/glim_ws/src/glim_ext`（独立 git）。开工前 `git ls-remote origin` 确认 `master`（本计划写作时为 `d40e77a`），从最新 `master` 开 `feat/round3a-diagnosis-core`。以 `git ls-remote` 为准（仓库有外部 git 自动同步）。
- **绝不 `git push`**。
- 每个 commit message 以 `Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>` 结尾。
- `glim_underground` 里 `include/glim/util/time_keeper.hpp`、`src/glim/util/time_keeper.cpp` 是维护者未提交的工作，绝不触碰、绝不暂存。
- **`gnss_core` 严格不依赖 ROS、不依赖 GLIM**；只允许 Eigen / GTSAM / GeographicLib 与标准库。
- 注释用中文、与周边代码一致；标识符英文；**诊断消息文本必须与本计划给出的字符串逐字一致**（全角括号、破折号「——」、顿号「、」、`⚠`）。
- 测试一律 gtest，经 `gnss_core/CMakeLists.txt` 里的 `gnss_core_add_test(<name>)` 注册（测试文件名即 `test/<name>.cpp`）。
- **诚实测试**：每个新测试在实现之前必须先亲眼看到失败（RED）——对旧代码失败（含编译失败），或对一个写明的变异失败；RED 证据写进 commit message。此前出现过多次"错误分支被执行、但错误本身从没发生"的假测试，评审会专门查。
- 测试产生的文件一律放在 `mkdtemp` 建出的临时目录（基于 `$TMPDIR`，缺省 `/tmp`），测试结束（含失败路径）删除。
- 构建：`cd /home/steve/glim_ws && source /opt/ros/humble/setup.bash && colcon build --symlink-install --packages-select gnss_core`（本机 colcon 必须先 source `/opt/ros/humble/setup.bash`，否则 pyenv shim 找不到 `ament_package`）
- 单测快速迭代：`cd /home/steve/glim_ws && source /opt/ros/humble/setup.bash && source install/setup.bash && ./build/gnss_core/<test_name> --gtest_filter='<Suite>.<Name>'`
- 全量：`cd /home/steve/glim_ws && source /opt/ros/humble/setup.bash && colcon test --packages-select gnss_core gnss_bringup && colcon test-result --all`
- 基线：`colcon test-result --all` = **376 tests, 0 errors, 0 failures, 1 skipped**。每个 Task 结束时 0 failures。
- 时间一律是 **UTC unix 秒（double）**，由壳传入；`gnss_core` 内部不读系统时钟（清理的 `utc_today_yyyymmdd` 也由调用方传入时间）。

## 设计决定（spec 与参考实现分歧处的裁定）

维护者 2026-09-15 确认的四项：

1. **事件按规则码独立**。rtk-monitor 设计文档 §4.3 要求"同一历元所有命中的规则及各自指标全部入事件"，但其代码只把第一条命中的结论喂给唯一的事件机。本计划：`evaluate_rules()` 返回全部命中结论（优先级序），**状态 = 第一条**（与 rtk-monitor `diagnose()` 完全一致，旧用例原样适用）；`EventBook` 为每个规则码各维护一个迟滞跟踪器，多条同时命中就同时开多个事件。
2. **`device_divergence` 的 σ 用经验值**。设计文档要求"最近 10 分钟 610-vs-自解差值的经验标准差（排除当前偏差窗口）"，代码却用 rtkrcv 自报 `sdn/sde`。本计划：`DivergenceMonitor` 维护最近 `divergence_window_s`（600 s）内**未超限**时的偏差样本，σ = `max(divergence_sigma_floor_m, RMS(样本))`；样本数不足 `divergence_min_samples`（60）时回退到 rtkrcv 自报 `max(1e-3, hypot(sdn, sde))`（即 rtk-monitor 现行行为）；阈值 = `divergence_sigma × σ`。偏差量是非负的水平距离，所以用 RMS 而不是围绕均值的标准差（二维零均值高斯差的 RMS 正好对应 σ 量级）。
3. **轮 3 拆两份**：本计划（3a，`gnss_core`）与 3b（`gnss_diag_node`、清理节点、`rtkrcv_node` 健康信号）。**`glim_ext` 的 `gnss_diag` 模块壳挪到轮 4** 与界面一起做（没有界面时它只是空壳）。
4. **清理做成独立 ROS 节点，每小时一次**（3b）；判定逻辑在本计划 Task 6 做成纯函数。

控制者另行裁定（3a 内，代价可控）：

5. **`multipath` 用残差绝对值**：RTKLIB `$SAT` 的 `resp` 带符号，rtk-monitor 用 `resp > resid_max_m` 会漏掉大的负残差；本计划用 `|resp| > resid_max_m`。
6. **消息去掉矿区用语**：rtk-monitor 面向露天矿（"全矿"、"高帮/坑底"）；本项目是隧道/井下巡检，相关措辞改为通用表述，规则码与数字格式不变（见 Task 1 消息表）。`abs_ref_shift` 消息带上控制点名。
7. **`not_fixed` 消息用归一化质量名**（`FLOAT` 等），因为输入是 `gnss_core::Quality` 而不是 RTKLIB 数字 Q。
8. **`ambiguity` 需要 ratio**：`SolutionSample::ratio` 是 `std::optional<double>`，缺失时该规则不触发。现有 `gnss_msgs/RtkFix` 没有 ratio 字段——怎么把 ratio 带到诊断节点由 3b 决定。
9. **停机时显式关闭所有事件**（`close_reason = shutdown`）。rtk-monitor 停机不关、事件在库里永远 open；文本日志无法原地改，故关闭并注明原因，重启后若故障仍在会重新开事件。
10. **`sats_min` 指标只在有解时才上报**（rtk-monitor 无解时报 0.0，会把"最少卫星数"峰值拉成 0）。
11. **`$SAT` 卫星列表按到达时刻做新鲜度门限**（`sol_stale_s`）：rtkrcv 死掉后不能让最后一个历元的多路径结论永远挂着。
12. **距离用 WGS-84 测地线**（GeographicLib `Geodesic`），不用 rtk-monitor 的球面 haversine；基站位移仍用 ECEF 欧氏距离（与 rtk-monitor 一致）。
13. **清理永不删除最新的一项**：录包目录名形如 `gnss_YYYYMMDD_HHMMSS`，跨天录制时最新目录可能早于今天但仍在写。

## File Structure

全部在 `/home/steve/glim_ws/src/glim_ext/gnss_core/` 下：

| 文件 | 职责 | Task |
|---|---|---|
| `include/gnss_core/geodetic.hpp`、`src/geodetic.cpp`（改） | 新增 `geodesic_distance_m()` | 1 |
| `include/gnss_core/diagnosis.hpp`、`src/diagnosis.cpp`（新） | `Level`/`Verdict`/`DiagnosisConfig`/`DiagnosisInput`/`SolutionSample`/`ControlPoint`、`evaluate_rules()`、`validate_diagnosis_config()` | 1 |
| `test/test_diagnosis.cpp`（新） | 规则链用例（移植 `test_rules.py` + 全命中 + 配置校验） | 1 |
| `include/gnss_core/divergence_monitor.hpp`、`src/divergence_monitor.cpp`（新） | 经验 σ 窗口与超限计时 | 2 |
| `test/test_divergence_monitor.cpp`（新） | | 2 |
| `include/gnss_core/event_book.hpp`、`src/event_book.cpp`（新） | 按码独立的事件迟滞跟踪、峰值指标、停机关闭 | 3 |
| `test/test_event_book.cpp`（新） | 移植 `test_event_machine.py` + 并发事件 | 3 |
| `include/gnss_core/base_station_monitor.hpp`、`src/base_station_monitor.cpp`（新） | 基站坐标预热中位数基线、偏移、变更判定、基线文件读写 | 4 |
| `test/test_base_station_monitor.cpp`（新） | 移植 `test_base_station.py` | 4 |
| `include/gnss_core/diag_io.hpp`、`src/diag_io.cpp`（新） | UTC 时间戳、`events.log` 行、`base.pos` 表头与行、`LineAppender` | 5 |
| `test/test_diag_io.cpp`（新） | | 5 |
| `include/gnss_core/retention.hpp`、`src/retention.cpp`（新） | 日期目录解析、清理判定、按根目录执行 | 6 |
| `test/test_retention.cpp`（新） | 移植 `test_cleanup.py` | 6 |
| `include/gnss_core/diagnosis_engine.hpp`、`src/diagnosis_engine.cpp`（新） | 串起全部部件的引擎 | 7 |
| `test/diag_test_fixtures.hpp`（新）、`test/test_diagnosis_engine.cpp`（新）、`test/test_rtkstat.cpp`（改，改用共享 `sat_line`） | | 7 |
| `CMakeLists.txt`（改） | 每个 Task 追加源文件与测试注册 | 1–7 |

依赖顺序：1 → 2 → 3 → 4 → 5 → 6 → 7（2–6 只依赖 1 的类型或互不依赖，但按序执行避免 CMakeLists 冲突）。

---
### Task 1: 规则链（C1–C9）与测地线距离

**Files:**
- Modify: `gnss_core/include/gnss_core/geodetic.hpp`、`gnss_core/src/geodetic.cpp`
- Create: `gnss_core/include/gnss_core/diagnosis.hpp`、`gnss_core/src/diagnosis.cpp`
- Test: `gnss_core/test/test_diagnosis.cpp`（新）、`gnss_core/test/test_geodetic.cpp`（追加）
- Modify: `gnss_core/CMakeLists.txt`

**Interfaces:**
- Consumes: `gnss_core::Quality`（`types.hpp`）、`gnss_core::SatStat`（`rtkstat.hpp`）
- Produces:
  - `double gnss_core::geodesic_distance_m(double lat1, double lon1, double lat2, double lon2);`
  - `enum class gnss_core::Level { Ok, Info, Warning, Serious, Critical };`、`const char* level_name(Level)`（`"ok" "info" "warning" "serious" "critical"`）、`bool level_opens_event(Level)`（Warning/Serious/Critical 为 true）、`const char* quality_name(Quality)`（`"NONE" "SINGLE" "DGPS" "FLOAT" "FIXED"`）
  - `struct Verdict { Level level; std::string code; std::string message; };`
  - `struct ControlPoint { std::string name; double lat; double lon; };`
  - `struct SolutionSample { Quality quality; double lat; double lon; int ns; double sdn; double sde; double age; std::optional<double> ratio; };`
  - `struct DiagnosisConfig`（字段与默认值见 Step 4）
  - `struct DiagnosisInput`（字段见 Step 4）
  - `struct DiagnosisResult { std::vector<Verdict> verdicts; const Verdict& status() const; };`
  - `DiagnosisResult evaluate_rules(const DiagnosisInput&, const DiagnosisConfig&);`
  - `void validate_diagnosis_config(const DiagnosisConfig&);`（非法时抛 `std::invalid_argument`，消息以字段名开头）

规则优先级（`verdicts` 的顺序）与消息（逐字）：

| 序 | code | 条件 | level | message |
|---|---|---|---|---|
| 0 | `no_data` | `!sol && !corr_last_t`；**命中后立即返回，不评其他规则** | warning | `无数据——检查采集链路与设备连接` |
| 1 | `corr_outage` | `gap = now - *corr_last_t`（有 corr_last_t 时）`> corr_gap_s`，或 `corr_age > age_max_s` | serious | `差分中断 %ds——5G 链路或平台转发问题`，`%d` = `int(gap 命中 ? gap : corr_age)` |
| 2 | `base_shift` | `base_offset_m > base_shift_m` | critical | `⚠ 基站坐标变动 %.2fm——所有定位结果将整体平移` |
| 3 | `abs_ref_shift` | `sol` 且 `quality == FIXED` 且有控制点；取测地线距离最近的控制点 d，`d <= abs_ref_radius_m && d > abs_ref_max_m` | critical | `⚠ 绝对基准偏差 %.2fm@控制点 %s——疑似整体基准平移` |
| 4 | `low_sats` | `sol && sol->ns < min_sats` | serious | `卫星数不足（%d 颗）——疑似遮挡` |
| 5 | `multipath` | `sats` 中 `|resp| > resid_max_m && (el < low_el_deg || snr < low_snr_dbhz)` 的卫星 ≥ 2 颗 | warning | `<前 4 颗星号用「、」连接> 残差异常——疑似多路径` |
| 6 | `ambiguity` | `sol && quality == FLOAT && ratio 有值 && *ratio < min_ratio` | warning | `模糊度无法固定（ratio=%.1f）——遮挡过渡区常见` |
| 7 | `cycle_slip` | `slip_count_30s > slip_max_per_30s` | warning | `载波频繁失锁——动态遮挡或天线/馈线问题` |
| 8 | `device_divergence` | `divergence_m && divergence_since && sol && *divergence_m > divergence_threshold_m && now - *divergence_since >= divergence_hold_s` | serious | `610 输出与独立解算偏差 %.2fm——疑似 610 融合问题` |
| 9 | 状态（恰好一条） | `!sol` → `no_solution`（`solver_enabled` 时 warning，否则 info）；`quality != FIXED` → `not_fixed`；否则 `rtk_fixed` | 见左 / info / ok | `独立解算无输出——rtkrcv 未运行或未收敛` / `独立解算未启用` / `非固定解（%s）`（`quality_name`）/ `RTK 固定` |

- [ ] **Step 1: 写测地线距离的失败测试**

追加到 `gnss_core/test/test_geodetic.cpp`：

```cpp
TEST(GeodesicDistance, ZeroForIdenticalPoints) {
  EXPECT_NEAR(geodesic_distance_m(44.5, 90.28, 44.5, 90.28), 0.0, 1e-9);
}

TEST(GeodesicDistance, MatchesLocalEnuHorizontalNormForShortBaselines) {
  // 与既有 LlaToEnu(GeographicLib LocalCartesian)交叉验证:几米的基线上两者应在毫米内一致
  const double lat0 = 44.5, lon0 = 90.28;
  LlaToEnu enu(lat0, lon0, 0.0);
  for (const auto& [dlat, dlon] : {std::pair{0.5 / 111000.0, 0.0}, std::pair{0.0, 1e-5},
                                   std::pair{2e-5, -3e-5}}) {
    const Eigen::Vector3d e = enu.forward(lat0 + dlat, lon0 + dlon, 0.0);
    EXPECT_NEAR(geodesic_distance_m(lat0, lon0, lat0 + dlat, lon0 + dlon), e.head<2>().norm(), 1e-3);
  }
}
```

- [ ] **Step 2: 写规则链的失败测试**

`gnss_core/test/test_diagnosis.cpp`：

```cpp
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include "gnss_core/diagnosis.hpp"
using namespace gnss_core;

namespace {
// 移植 rtk-monitor tests/test_rules.py 的 _sol / _inp
SolutionSample sol(Quality q = Quality::FIXED, int ns = 38, std::optional<double> ratio = 25.0) {
  SolutionSample s;
  s.quality = q;
  s.lat = 44.5;
  s.lon = 90.28;
  s.ns = ns;
  s.sdn = 0.011;
  s.sde = 0.012;
  s.age = 0.8;
  s.ratio = ratio;
  return s;
}

DiagnosisInput inp() {
  DiagnosisInput in;
  in.now = 1000.0;
  in.corr_last_t = 999.5;
  in.corr_age = 0.8;
  in.base_offset_m = 0.0;
  in.sol = sol();
  in.divergence_threshold_m = 3.0 * std::hypot(0.011, 0.012);
  return in;
}

SatStat sat(const char* name, double el, double resp, double snr) {
  SatStat s;
  s.sat = name;
  s.el = el;
  s.resp = resp;
  s.snr = snr;
  s.valid = true;
  return s;
}

bool has_code(const DiagnosisResult& r, const std::string& code) {
  return std::any_of(r.verdicts.begin(), r.verdicts.end(),
                     [&](const Verdict& v) { return v.code == code; });
}

const DiagnosisConfig kCfg{};
}  // namespace

TEST(Rules, AllGoodIsFixed) {
  const auto r = evaluate_rules(inp(), kCfg);
  EXPECT_EQ(r.status().code, "rtk_fixed");
  EXPECT_EQ(r.status().level, Level::Ok);
  EXPECT_EQ(r.status().message, "RTK 固定");
  EXPECT_EQ(r.verdicts.size(), 1u);
}

TEST(Rules, CorrOutageWinsStatusButAmbiguityIsStillReported) {
  auto in = inp();
  in.corr_last_t = 990.0;               // gap 10 s > 3 s
  in.sol = sol(Quality::FLOAT, 38, 1.5);
  const auto r = evaluate_rules(in, kCfg);
  EXPECT_EQ(r.status().code, "corr_outage");
  EXPECT_EQ(r.status().level, Level::Serious);
  EXPECT_EQ(r.status().message, "差分中断 10s——5G 链路或平台转发问题");
  EXPECT_TRUE(has_code(r, "ambiguity")) << "全命中:优先级低的并发故障也必须报告";
  EXPECT_EQ(r.verdicts.back().code, "not_fixed");
}

TEST(Rules, CorrOutageByAgeOverrun) {
  auto in = inp();
  in.corr_age = 15.0;
  const auto r = evaluate_rules(in, kCfg);
  EXPECT_EQ(r.status().code, "corr_outage");
  EXPECT_EQ(r.status().message, "差分中断 15s——5G 链路或平台转发问题");
}

TEST(Rules, BaseShift) {
  auto in = inp();
  in.base_offset_m = 0.8;
  const auto r = evaluate_rules(in, kCfg);
  EXPECT_EQ(r.status().code, "base_shift");
  EXPECT_EQ(r.status().level, Level::Critical);
  EXPECT_EQ(r.status().message, "⚠ 基站坐标变动 0.80m——所有定位结果将整体平移");
}

TEST(Rules, LowSats) {
  auto in = inp();
  in.sol = sol(Quality::FIXED, 4);
  const auto r = evaluate_rules(in, kCfg);
  EXPECT_EQ(r.status().code, "low_sats");
  EXPECT_EQ(r.status().message, "卫星数不足（4 颗）——疑似遮挡");
}

TEST(Rules, MultipathNeedsTwoBadSatellitesAndNamesThem) {
  auto in = inp();
  in.sats = {sat("C08", 15, 3.5, 30), sat("G17", 12, 2.8, 33)};
  in.sol = sol(Quality::FLOAT, 38, 1.5);
  auto r = evaluate_rules(in, kCfg);
  EXPECT_EQ(r.status().code, "multipath");
  EXPECT_EQ(r.status().message, "C08、G17 残差异常——疑似多路径");
  EXPECT_TRUE(has_code(r, "ambiguity"));

  in.sats.pop_back();
  r = evaluate_rules(in, kCfg);
  EXPECT_FALSE(has_code(r, "multipath")) << "只有一颗坏星不报多路径";
}

TEST(Rules, MultipathUsesAbsoluteResidual) {
  auto in = inp();
  in.sats = {sat("C08", 15, -3.5, 30), sat("G17", 12, -2.8, 33)};
  EXPECT_EQ(evaluate_rules(in, kCfg).status().code, "multipath")
      << "RTKLIB 残差带符号,大的负残差同样是异常";
}

TEST(Rules, MultipathNamesAtMostFourSatellites) {
  auto in = inp();
  in.sats = {sat("G01", 10, 3, 30), sat("G02", 10, 3, 30), sat("G03", 10, 3, 30),
             sat("G04", 10, 3, 30), sat("G05", 10, 3, 30)};
  EXPECT_EQ(evaluate_rules(in, kCfg).status().message, "G01、G02、G03、G04 残差异常——疑似多路径");
}

TEST(Rules, AbsRefShiftWhenNearControlPointAndDeviated) {
  auto in = inp();
  in.control_points = {{"CP1", 44.5 + 0.5 / 111000.0, 90.28}};
  const auto r = evaluate_rules(in, kCfg);
  EXPECT_EQ(r.status().code, "abs_ref_shift");
  EXPECT_EQ(r.status().level, Level::Critical);
  EXPECT_EQ(r.status().message.rfind("⚠ 绝对基准偏差 0.50m@控制点 CP1", 0), 0u) << r.status().message;
}

TEST(Rules, NoAbsRefAlarmOnControlPointFarAwayWithoutPointsOrOnFloat) {
  auto in = inp();
  in.control_points = {{"CP1", 44.5, 90.28}};
  EXPECT_EQ(evaluate_rules(in, kCfg).status().code, "rtk_fixed");
  in.control_points = {{"CP1", 44.6, 90.28}};
  EXPECT_EQ(evaluate_rules(in, kCfg).status().code, "rtk_fixed");
  in.control_points = {};
  EXPECT_EQ(evaluate_rules(in, kCfg).status().code, "rtk_fixed");
  in.control_points = {{"CP1", 44.5 + 0.5 / 111000.0, 90.28}};
  in.sol = sol(Quality::FLOAT, 38, 25.0);
  EXPECT_FALSE(has_code(evaluate_rules(in, kCfg), "abs_ref_shift"));
}

TEST(Rules, AbsRefJudgesOnlyTheNearestControlPoint) {
  auto in = inp();
  // 最近点在 0.05 m(未超限),另一点 0.5 m(超限但不是最近点)→ 不报
  in.control_points = {{"FAR", 44.5 + 0.5 / 111000.0, 90.28}, {"NEAR", 44.5 + 0.05 / 111000.0, 90.28}};
  EXPECT_FALSE(has_code(evaluate_rules(in, kCfg), "abs_ref_shift"));
}

TEST(Rules, AmbiguityOnFloatLowRatio) {
  auto in = inp();
  in.sol = sol(Quality::FLOAT, 38, 1.8);
  const auto r = evaluate_rules(in, kCfg);
  EXPECT_EQ(r.status().code, "ambiguity");
  EXPECT_EQ(r.status().message, "模糊度无法固定（ratio=1.8）——遮挡过渡区常见");
}

TEST(Rules, AmbiguityNeedsARatio) {
  auto in = inp();
  in.sol = sol(Quality::FLOAT, 38, std::nullopt);
  const auto r = evaluate_rules(in, kCfg);
  EXPECT_FALSE(has_code(r, "ambiguity"));
  EXPECT_EQ(r.status().code, "not_fixed");
  EXPECT_EQ(r.status().message, "非固定解（FLOAT）");
}

TEST(Rules, CycleSlips) {
  auto in = inp();
  in.slip_count_30s = 9;
  EXPECT_EQ(evaluate_rules(in, kCfg).status().code, "cycle_slip");
  in.slip_count_30s = 5;
  EXPECT_EQ(evaluate_rules(in, kCfg).status().code, "rtk_fixed") << "等于门限不报";
}

TEST(Rules, DivergenceNeedsHold) {
  auto in = inp();
  in.divergence_m = 0.5;
  in.divergence_since = 998.0;   // 已持续 2 s < 5 s
  EXPECT_EQ(evaluate_rules(in, kCfg).status().code, "rtk_fixed");
  in.divergence_since = 990.0;   // 10 s
  const auto r = evaluate_rules(in, kCfg);
  EXPECT_EQ(r.status().code, "device_divergence");
  EXPECT_EQ(r.status().message, "610 输出与独立解算偏差 0.50m——疑似 610 融合问题");
}

TEST(Rules, DivergenceUsesTheSuppliedThreshold) {
  auto in = inp();
  in.divergence_m = 0.5;
  in.divergence_since = 990.0;
  in.divergence_threshold_m = 0.6;
  EXPECT_FALSE(has_code(evaluate_rules(in, kCfg), "device_divergence"));
}

TEST(Rules, NoDataAtAllShortCircuits) {
  auto in = inp();
  in.sol.reset();
  in.corr_last_t.reset();
  in.corr_age.reset();
  in.base_offset_m = 0.8;   // 即便有基站偏移,也只报 no_data
  const auto r = evaluate_rules(in, kCfg);
  ASSERT_EQ(r.verdicts.size(), 1u);
  EXPECT_EQ(r.status().code, "no_data");
  EXPECT_EQ(r.status().level, Level::Warning);
  EXPECT_EQ(r.status().message, "无数据——检查采集链路与设备连接");
}

TEST(Rules, SolverDeadIsNotReportedFixed) {
  auto in = inp();
  in.sol.reset();
  const auto r = evaluate_rules(in, kCfg);
  EXPECT_EQ(r.status().code, "no_solution");
  EXPECT_EQ(r.status().level, Level::Warning);
  EXPECT_EQ(r.status().message, "独立解算无输出——rtkrcv 未运行或未收敛");
}

TEST(Rules, SolverDisabledIsInfo) {
  auto in = inp();
  in.sol.reset();
  in.solver_enabled = false;
  const auto r = evaluate_rules(in, kCfg);
  EXPECT_EQ(r.status().code, "no_solution");
  EXPECT_EQ(r.status().level, Level::Info);
  EXPECT_EQ(r.status().message, "独立解算未启用");
}

TEST(Rules, VerdictsAreInPriorityOrderWhenManyFire) {
  auto in = inp();
  in.corr_last_t = 990.0;
  in.base_offset_m = 0.8;
  in.sol = sol(Quality::FLOAT, 4, 1.0);
  in.slip_count_30s = 9;
  std::vector<std::string> codes;
  for (const auto& v : evaluate_rules(in, kCfg).verdicts) codes.push_back(v.code);
  EXPECT_EQ(codes, (std::vector<std::string>{"corr_outage", "base_shift", "low_sats", "ambiguity",
                                             "cycle_slip", "not_fixed"}));
}

TEST(Rules, LevelHelpers) {
  EXPECT_STREQ(level_name(Level::Serious), "serious");
  EXPECT_TRUE(level_opens_event(Level::Warning));
  EXPECT_FALSE(level_opens_event(Level::Info));
  EXPECT_FALSE(level_opens_event(Level::Ok));
  EXPECT_STREQ(quality_name(Quality::DGPS), "DGPS");
}

TEST(DiagnosisConfigValidation, DefaultsAreValidAndMatchRtkMonitor) {
  const DiagnosisConfig c;
  EXPECT_NO_THROW(validate_diagnosis_config(c));
  EXPECT_DOUBLE_EQ(c.corr_gap_s, 3.0);
  EXPECT_DOUBLE_EQ(c.age_max_s, 10.0);
  EXPECT_DOUBLE_EQ(c.base_shift_m, 0.1);
  EXPECT_EQ(c.min_sats, 6);
  EXPECT_DOUBLE_EQ(c.close_hysteresis_s, 10.0);
  EXPECT_DOUBLE_EQ(c.abs_ref_radius_m, 3.0);
}

TEST(DiagnosisConfigValidation, RejectsNonsenseWithTheFieldName) {
  const auto expect_rejected = [](DiagnosisConfig c, const std::string& field) {
    try {
      validate_diagnosis_config(c);
      ADD_FAILURE() << field << " 没有被拒绝";
    } catch (const std::invalid_argument& e) {
      EXPECT_EQ(std::string(e.what()).rfind(field, 0), 0u) << e.what();
    }
  };
  DiagnosisConfig c;
  c.corr_gap_s = 0.0;           expect_rejected(c, "corr_gap_s");       c = {};
  c.sol_stale_s = -1.0;         expect_rejected(c, "sol_stale_s");      c = {};
  c.min_sats = -1;              expect_rejected(c, "min_sats");         c = {};
  c.low_el_deg = 91.0;          expect_rejected(c, "low_el_deg");       c = {};
  c.abs_ref_radius_m = 0.1;     expect_rejected(c, "abs_ref_radius_m"); c = {};   // 必须大于 abs_ref_max_m
  c.divergence_min_samples = 1; expect_rejected(c, "divergence_min_samples");
}
```

在 `gnss_core/CMakeLists.txt` 的 `add_library(gnss_core SHARED ...)` 列表追加 `src/diagnosis.cpp`，在 `if(BUILD_TESTING)` 里追加 `gnss_core_add_test(test_diagnosis)`。

- [ ] **Step 3: 确认 RED**

Run: 按 Global Constraints 构建 `gnss_core`。
Expected: 编译失败（`diagnosis.hpp`、`geodesic_distance_m` 不存在）。这就是 RED；把编译错误的关键行记进报告。

- [ ] **Step 4: 实现**

`gnss_core/include/gnss_core/geodetic.hpp` 追加声明：

```cpp
// 两点间 WGS-84 椭球测地线距离(m),只看水平位置。
double geodesic_distance_m(double lat1, double lon1, double lat2, double lon2);
```

`gnss_core/src/geodetic.cpp` 追加（include `<GeographicLib/Geodesic.hpp>`）：

```cpp
double geodesic_distance_m(double lat1, double lon1, double lat2, double lon2) {
  double s12 = 0.0;
  GeographicLib::Geodesic::WGS84().Inverse(lat1, lon1, lat2, lon2, s12);
  return s12;
}
```

`gnss_core/include/gnss_core/diagnosis.hpp`：

```cpp
#pragma once
// 诊断规则链(spec §3 C1–C9、§8)。纯函数,无 ROS / GLIM 依赖。
// 优先级、阈值默认值与消息语义移植自 rtk-monitor diagnosis/rules.py;
// 与参考实现的差异见 docs/gnss/plans/2026-09-15-round3a-diagnosis-core.md「设计决定」。
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "gnss_core/rtkstat.hpp"
#include "gnss_core/types.hpp"

namespace gnss_core {

enum class Level { Ok, Info, Warning, Serious, Critical };
const char* level_name(Level level);
// 只有 warning / serious / critical 会开事件;ok / info 只是状态
bool level_opens_event(Level level);
const char* quality_name(Quality q);

struct Verdict {
  Level level = Level::Ok;
  std::string code;
  std::string message;
};

struct ControlPoint {
  std::string name;
  double lat = 0.0, lon = 0.0;   // WGS-84 deg;只做水平比较
};

// 一路定位解(rtkrcv 独立解或 610 融合解)在诊断里用到的字段
struct SolutionSample {
  Quality quality = Quality::NONE;
  double lat = 0.0, lon = 0.0;   // deg
  int ns = 0;                    // 参与解算卫星数
  double sdn = 0.0, sde = 0.0;   // m
  double age = 0.0;              // 差分龄期 s
  std::optional<double> ratio;   // AR ratio;源不提供时为空,ambiguity 规则不触发
};

struct DiagnosisConfig {
  double corr_gap_s = 3.0;
  double age_max_s = 10.0;
  double base_shift_m = 0.1;
  int min_sats = 6;
  double resid_max_m = 2.0;
  double low_el_deg = 20.0;
  double low_snr_dbhz = 35.0;
  double min_ratio = 3.0;
  int slip_max_per_30s = 5;
  double divergence_sigma = 3.0;
  double divergence_hold_s = 5.0;
  double close_hysteresis_s = 10.0;
  double sol_stale_s = 5.0;
  double abs_ref_max_m = 0.2;
  double abs_ref_radius_m = 3.0;
  // 以下为本项目新增(rtk-monitor 没有)
  double divergence_window_s = 600.0;       // 经验 σ 的滑动窗口
  int divergence_min_samples = 60;          // 样本不足时回退到 rtkrcv 自报 σ
  double divergence_sigma_floor_m = 0.05;   // 经验 σ 下限,两路几乎重合时防误报
  double divergence_pair_max_dt_s = 2.0;    // 两路解到达时刻相差超过此值不配对
  double base_warmup_s = 600.0;             // 基站基线预热时长(取中位数)
};

// 任何字段非法时抛 std::invalid_argument,消息以字段名开头
void validate_diagnosis_config(const DiagnosisConfig& cfg);

// 每秒一次的规则输入。时间一律是 UTC unix 秒。
struct DiagnosisInput {
  double now = 0.0;
  std::optional<double> corr_last_t;         // 最近一次收到任何差分字节的时刻
  std::optional<double> corr_age;            // 差分龄期 s
  std::optional<double> base_offset_m;       // 基站相对基线的 ECEF 位移
  std::optional<SolutionSample> sol;         // 独立解(调用方已按 sol_stale_s 过滤)
  std::vector<SatStat> sats;                 // 当前 $SAT 历元(调用方已过滤新鲜度)
  int slip_count_30s = 0;
  std::optional<double> divergence_m;        // 610 与独立解的水平偏差
  std::optional<double> divergence_since;    // 偏差首次超限时刻(DivergenceMonitor 给)
  double divergence_threshold_m = 0.0;       // 当前超限阈值(DivergenceMonitor 给)
  bool solver_enabled = true;
  std::vector<ControlPoint> control_points;
};

struct DiagnosisResult {
  // 全部命中的结论,按优先级排序;最后一条总是状态(no_solution / not_fixed / rtk_fixed),
  // 例外:no_data 命中时只有这一条。
  std::vector<Verdict> verdicts;
  const Verdict& status() const { return verdicts.front(); }
};

DiagnosisResult evaluate_rules(const DiagnosisInput& in, const DiagnosisConfig& cfg);

}  // namespace gnss_core
```

`gnss_core/src/diagnosis.cpp`：

```cpp
#include "gnss_core/diagnosis.hpp"

#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <limits>
#include <utility>

#include "gnss_core/geodetic.hpp"

namespace gnss_core {

namespace {
std::string format(const char* fmt, ...) {
  char buf[256];
  va_list args;
  va_start(args, fmt);
  std::vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);
  return buf;
}

void require(bool ok, const char* field, const char* why) {
  if (!ok) throw std::invalid_argument(std::string(field) + ": " + why);
}
}  // namespace

const char* level_name(Level level) {
  switch (level) {
    case Level::Ok: return "ok";
    case Level::Info: return "info";
    case Level::Warning: return "warning";
    case Level::Serious: return "serious";
    case Level::Critical: return "critical";
  }
  return "ok";
}

bool level_opens_event(Level level) {
  return level == Level::Warning || level == Level::Serious || level == Level::Critical;
}

const char* quality_name(Quality q) {
  switch (q) {
    case Quality::NONE: return "NONE";
    case Quality::SINGLE: return "SINGLE";
    case Quality::DGPS: return "DGPS";
    case Quality::FLOAT: return "FLOAT";
    case Quality::FIXED: return "FIXED";
  }
  return "NONE";
}

void validate_diagnosis_config(const DiagnosisConfig& c) {
  require(c.corr_gap_s > 0.0, "corr_gap_s", "必须 > 0");
  require(c.age_max_s > 0.0, "age_max_s", "必须 > 0");
  require(c.base_shift_m > 0.0, "base_shift_m", "必须 > 0");
  require(c.min_sats >= 0, "min_sats", "必须 >= 0");
  require(c.resid_max_m > 0.0, "resid_max_m", "必须 > 0");
  require(c.low_el_deg >= 0.0 && c.low_el_deg <= 90.0, "low_el_deg", "必须在 [0, 90]");
  require(c.low_snr_dbhz >= 0.0, "low_snr_dbhz", "必须 >= 0");
  require(c.min_ratio > 0.0, "min_ratio", "必须 > 0");
  require(c.slip_max_per_30s >= 0, "slip_max_per_30s", "必须 >= 0");
  require(c.divergence_sigma > 0.0, "divergence_sigma", "必须 > 0");
  require(c.divergence_hold_s >= 0.0, "divergence_hold_s", "必须 >= 0");
  require(c.close_hysteresis_s >= 0.0, "close_hysteresis_s", "必须 >= 0");
  require(c.sol_stale_s > 0.0, "sol_stale_s", "必须 > 0");
  require(c.abs_ref_max_m > 0.0, "abs_ref_max_m", "必须 > 0");
  require(c.abs_ref_radius_m > c.abs_ref_max_m, "abs_ref_radius_m", "必须大于 abs_ref_max_m,否则该规则永远不会触发");
  require(c.divergence_window_s > 0.0, "divergence_window_s", "必须 > 0");
  require(c.divergence_min_samples >= 2, "divergence_min_samples", "必须 >= 2");
  require(c.divergence_sigma_floor_m > 0.0, "divergence_sigma_floor_m", "必须 > 0");
  require(c.divergence_pair_max_dt_s > 0.0, "divergence_pair_max_dt_s", "必须 > 0");
  require(c.base_warmup_s >= 0.0, "base_warmup_s", "必须 >= 0");
}

DiagnosisResult evaluate_rules(const DiagnosisInput& in, const DiagnosisConfig& cfg) {
  DiagnosisResult r;
  const auto push = [&r](Level level, const char* code, std::string message) {
    r.verdicts.push_back(Verdict{level, code, std::move(message)});
  };

  // 0 no_data:什么都没有时其他规则没有意义,直接返回
  if (!in.sol && !in.corr_last_t) {
    push(Level::Warning, "no_data", "无数据——检查采集链路与设备连接");
    return r;
  }

  // 1 corr_outage
  std::optional<double> gap;
  if (in.corr_last_t) gap = in.now - *in.corr_last_t;
  const bool gap_hit = gap && *gap > cfg.corr_gap_s;
  const bool age_hit = in.corr_age && *in.corr_age > cfg.age_max_s;
  if (gap_hit || age_hit) {
    const int n = static_cast<int>(gap_hit ? *gap : *in.corr_age);
    push(Level::Serious, "corr_outage", format("差分中断 %ds——5G 链路或平台转发问题", n));
  }

  // 2 base_shift
  if (in.base_offset_m && *in.base_offset_m > cfg.base_shift_m) {
    push(Level::Critical, "base_shift",
         format("⚠ 基站坐标变动 %.2fm——所有定位结果将整体平移", *in.base_offset_m));
  }

  // 3 abs_ref_shift:只在固定解时判(0.2 m 门限只对 cm 级固定解有意义),只判最近的控制点
  if (in.sol && in.sol->quality == Quality::FIXED && !in.control_points.empty()) {
    const ControlPoint* nearest = nullptr;
    double best = std::numeric_limits<double>::infinity();
    for (const auto& cp : in.control_points) {
      const double d = geodesic_distance_m(in.sol->lat, in.sol->lon, cp.lat, cp.lon);
      if (d < best) {
        best = d;
        nearest = &cp;
      }
    }
    if (nearest && best <= cfg.abs_ref_radius_m && best > cfg.abs_ref_max_m) {
      push(Level::Critical, "abs_ref_shift",
           format("⚠ 绝对基准偏差 %.2fm@控制点 %s——疑似整体基准平移", best, nearest->name.c_str()));
    }
  }

  // 4 low_sats
  if (in.sol && in.sol->ns < cfg.min_sats) {
    push(Level::Serious, "low_sats", format("卫星数不足（%d 颗）——疑似遮挡", in.sol->ns));
  }

  // 5 multipath:残差取绝对值(RTKLIB resp 带符号)
  std::vector<const SatStat*> bad;
  for (const auto& s : in.sats) {
    if (std::abs(s.resp) > cfg.resid_max_m && (s.el < cfg.low_el_deg || s.snr < cfg.low_snr_dbhz)) {
      bad.push_back(&s);
    }
  }
  if (bad.size() >= 2) {
    std::string names;
    for (size_t i = 0; i < bad.size() && i < 4; ++i) {
      if (i > 0) names += "、";
      names += bad[i]->sat;
    }
    push(Level::Warning, "multipath", names + " 残差异常——疑似多路径");
  }

  // 6 ambiguity
  if (in.sol && in.sol->quality == Quality::FLOAT && in.sol->ratio && *in.sol->ratio < cfg.min_ratio) {
    push(Level::Warning, "ambiguity",
         format("模糊度无法固定（ratio=%.1f）——遮挡过渡区常见", *in.sol->ratio));
  }

  // 7 cycle_slip
  if (in.slip_count_30s > cfg.slip_max_per_30s) {
    push(Level::Warning, "cycle_slip", "载波频繁失锁——动态遮挡或天线/馈线问题");
  }

  // 8 device_divergence:阈值与起始时刻由 DivergenceMonitor 给出,这里不再重算
  if (in.divergence_m && in.divergence_since && in.sol &&
      *in.divergence_m > in.divergence_threshold_m &&
      in.now - *in.divergence_since >= cfg.divergence_hold_s) {
    push(Level::Serious, "device_divergence",
         format("610 输出与独立解算偏差 %.2fm——疑似 610 融合问题", *in.divergence_m));
  }

  // 9 状态
  if (!in.sol) {
    if (in.solver_enabled) {
      push(Level::Warning, "no_solution", "独立解算无输出——rtkrcv 未运行或未收敛");
    } else {
      push(Level::Info, "no_solution", "独立解算未启用");
    }
  } else if (in.sol->quality != Quality::FIXED) {
    push(Level::Info, "not_fixed", format("非固定解（%s）", quality_name(in.sol->quality)));
  } else {
    push(Level::Ok, "rtk_fixed", "RTK 固定");
  }
  return r;
}

}  // namespace gnss_core
```

- [ ] **Step 5: GREEN + 变异检查**

Run: 构建后 `./build/gnss_core/test_diagnosis && ./build/gnss_core/test_geodetic`
Expected: 全部 PASS。

变异（做完恢复）：
- `multipath` 条件里 `std::abs(s.resp)` 改成 `s.resp` → `MultipathUsesAbsoluteResidual` 必须 FAIL
- 删掉 `no_data` 分支里的 `return r;` → `NoDataAtAllShortCircuits` 必须 FAIL
- `abs_ref_shift` 改成对每个控制点都判（去掉"最近点"）→ `AbsRefJudgesOnlyTheNearestControlPoint` 必须 FAIL
- 删掉 `abs_ref_radius_m > abs_ref_max_m` 的校验 → `RejectsNonsenseWithTheFieldName` 必须 FAIL

- [ ] **Step 6: 全量测试并提交**

Run: 全量命令。Expected: 0 failures。

```bash
cd /home/steve/glim_ws/src/glim_ext
git add gnss_core/include/gnss_core/geodetic.hpp gnss_core/src/geodetic.cpp gnss_core/include/gnss_core/diagnosis.hpp gnss_core/src/diagnosis.cpp gnss_core/test/test_diagnosis.cpp gnss_core/test/test_geodetic.cpp gnss_core/CMakeLists.txt
git commit -m "feat(gnss_core): diagnosis rule chain reporting every matching rule

<RED 编译失败证据;四个变异结果>

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---
### Task 2: `DivergenceMonitor`（610 与独立解偏差的经验 σ）

**Files:**
- Create: `gnss_core/include/gnss_core/divergence_monitor.hpp`、`gnss_core/src/divergence_monitor.cpp`
- Test: `gnss_core/test/test_divergence_monitor.cpp`
- Modify: `gnss_core/CMakeLists.txt`（源文件 `src/divergence_monitor.cpp`；`gnss_core_add_test(test_divergence_monitor)`）

**Interfaces:**
- Consumes: `DiagnosisConfig` 的 `divergence_sigma`、`divergence_window_s`、`divergence_min_samples`、`divergence_sigma_floor_m`（Task 1）
- Produces:
  - `struct DivergenceState { std::optional<double> divergence_m; std::optional<double> since; double threshold_m; bool empirical; };`
  - `class DivergenceMonitor { public: explicit DivergenceMonitor(const DiagnosisConfig& cfg); DivergenceState update(double t, std::optional<double> divergence_m, double fallback_sigma_m); size_t window_size() const; };`

行为（每 tick 调用一次）：
1. 丢弃窗口里时刻 `< t - divergence_window_s` 的样本。
2. 窗口样本数 `>= divergence_min_samples` 时：`sigma = max(divergence_sigma_floor_m, sqrt(mean(d²)))`，`empirical = true`；否则 `sigma = max(1e-3, fallback_sigma_m)`，`empirical = false`。`threshold_m = divergence_sigma * sigma`。
3. `divergence_m` 为空（本 tick 两路没配上）：清空 `since`，返回 `{nullopt, nullopt, threshold_m, empirical}`，不入窗口。
4. `*divergence_m > threshold_m`：`since` 为空则置为 `t`（已有则保持）；**该样本不入窗口**（排除当前偏差段）。
5. 否则：清空 `since`，样本 `(t, d)` 入窗口。
6. 返回 `{divergence_m, since, threshold_m, empirical}`。

- [ ] **Step 1: 写失败测试**

`gnss_core/test/test_divergence_monitor.cpp`：

```cpp
#include <gtest/gtest.h>

#include <cmath>

#include "gnss_core/divergence_monitor.hpp"
using namespace gnss_core;

namespace {
DiagnosisConfig cfg_small() {
  DiagnosisConfig c;
  c.divergence_sigma = 3.0;
  c.divergence_window_s = 100.0;
  c.divergence_min_samples = 10;
  c.divergence_sigma_floor_m = 0.05;
  return c;
}
}  // namespace

TEST(DivergenceMonitor, FallsBackToSolverSigmaUntilTheWindowHasEnoughSamples) {
  DivergenceMonitor m(cfg_small());
  const auto s = m.update(0.0, 0.02, std::hypot(0.011, 0.012));
  EXPECT_FALSE(s.empirical);
  EXPECT_NEAR(s.threshold_m, 3.0 * std::hypot(0.011, 0.012), 1e-12);
  EXPECT_FALSE(s.since.has_value());
}

TEST(DivergenceMonitor, FallbackSigmaHasAMillimetreFloor) {
  DivergenceMonitor m(cfg_small());
  EXPECT_NEAR(m.update(0.0, std::nullopt, 0.0).threshold_m, 3.0 * 1e-3, 1e-12);
}

TEST(DivergenceMonitor, UsesRmsOfTheWindowOnceFull) {
  DivergenceMonitor m(cfg_small());
  // 10 个 0.2 m 样本(610 与独立解长期稳定相差 0.2 m)→ RMS 0.2,阈值 0.6
  for (int i = 0; i < 10; ++i) m.update(i, 0.2, 0.001);
  const auto s = m.update(10.0, 0.25, 0.001);
  EXPECT_TRUE(s.empirical);
  EXPECT_NEAR(s.threshold_m, 0.6, 1e-9);
  EXPECT_FALSE(s.since.has_value()) << "0.25 < 0.6,不算超限";
}

TEST(DivergenceMonitor, EmpiricalSigmaHasAFloor) {
  DivergenceMonitor m(cfg_small());
  for (int i = 0; i < 10; ++i) m.update(i, 0.001, 0.001);   // 两路几乎重合
  EXPECT_NEAR(m.update(10.0, 0.001, 0.001).threshold_m, 3.0 * 0.05, 1e-9);
}

TEST(DivergenceMonitor, ExceedingSamplesStartTheClockAndStayOutOfTheWindow) {
  DivergenceMonitor m(cfg_small());
  for (int i = 0; i < 10; ++i) m.update(i, 0.1, 0.001);   // 阈值 0.3
  const size_t before = m.window_size();
  auto s = m.update(10.0, 1.0, 0.001);
  ASSERT_TRUE(s.since.has_value());
  EXPECT_DOUBLE_EQ(*s.since, 10.0);
  s = m.update(11.0, 1.2, 0.001);
  EXPECT_DOUBLE_EQ(*s.since, 10.0) << "持续超限时起始时刻保持";
  EXPECT_EQ(m.window_size(), before) << "超限段不能拉高自己的阈值";
  EXPECT_NEAR(s.threshold_m, 0.3, 1e-9);
}

TEST(DivergenceMonitor, RecoveryOrLostPairingClearsTheClock) {
  DivergenceMonitor m(cfg_small());
  m.update(0.0, 1.0, 0.01);
  EXPECT_FALSE(m.update(1.0, 0.0, 0.01).since.has_value());
  m.update(2.0, 1.0, 0.01);
  const auto s = m.update(3.0, std::nullopt, 0.01);
  EXPECT_FALSE(s.since.has_value());
  EXPECT_FALSE(s.divergence_m.has_value());
}

TEST(DivergenceMonitor, OldSamplesAgeOutOfTheWindow) {
  DivergenceMonitor m(cfg_small());
  for (int i = 0; i < 10; ++i) m.update(i, 0.2, 0.001);
  EXPECT_TRUE(m.update(50.0, 0.2, 0.001).empirical);
  const auto s = m.update(150.0, std::nullopt, 0.02);   // 窗口 100 s,之前的样本全部过期
  EXPECT_FALSE(s.empirical);
  EXPECT_EQ(m.window_size(), 0u);
}
```

- [ ] **Step 2: 确认 RED**（编译失败：头文件不存在），记录。

- [ ] **Step 3: 实现**

`gnss_core/include/gnss_core/divergence_monitor.hpp`：

```cpp
#pragma once
// device_divergence 规则的阈值与计时(spec §3 C8)。
// σ 用最近 divergence_window_s 内"未超限"时的偏差 RMS——设计文档要求经验 σ 且排除当前偏差段,
// 这样 610 "自信地错"(自报 σ 很小)也能被抓到;样本不足时回退到 rtkrcv 自报 σ。
#include <cstddef>
#include <deque>
#include <optional>
#include <utility>

#include "gnss_core/diagnosis.hpp"

namespace gnss_core {

struct DivergenceState {
  std::optional<double> divergence_m;   // 本 tick 的偏差;两路没配上时为空
  std::optional<double> since;          // 持续超限的起始时刻
  double threshold_m = 0.0;
  bool empirical = false;               // true:阈值来自窗口 RMS;false:来自回退 σ
};

class DivergenceMonitor {
public:
  explicit DivergenceMonitor(const DiagnosisConfig& cfg);
  // fallback_sigma_m:窗口样本不足时使用的 σ(调用方传 hypot(sdn, sde),无独立解时传 0)
  DivergenceState update(double t, std::optional<double> divergence_m, double fallback_sigma_m);
  size_t window_size() const { return window_.size(); }

private:
  double sigma_mult_, window_s_, floor_m_;
  size_t min_samples_;
  std::deque<std::pair<double, double>> window_;   // (t, d)
  std::optional<double> since_;
};

}  // namespace gnss_core
```

`gnss_core/src/divergence_monitor.cpp`：

```cpp
#include "gnss_core/divergence_monitor.hpp"

#include <algorithm>
#include <cmath>

namespace gnss_core {

DivergenceMonitor::DivergenceMonitor(const DiagnosisConfig& cfg)
    : sigma_mult_(cfg.divergence_sigma),
      window_s_(cfg.divergence_window_s),
      floor_m_(cfg.divergence_sigma_floor_m),
      min_samples_(static_cast<size_t>(std::max(cfg.divergence_min_samples, 0))) {}

DivergenceState DivergenceMonitor::update(double t, std::optional<double> divergence_m,
                                          double fallback_sigma_m) {
  while (!window_.empty() && window_.front().first < t - window_s_) window_.pop_front();

  DivergenceState s;
  double sigma = 0.0;
  if (window_.size() >= min_samples_ && !window_.empty()) {
    double sum_sq = 0.0;
    for (const auto& [ts, d] : window_) sum_sq += d * d;
    sigma = std::max(floor_m_, std::sqrt(sum_sq / static_cast<double>(window_.size())));
    s.empirical = true;
  } else {
    sigma = std::max(1e-3, fallback_sigma_m);
  }
  s.threshold_m = sigma_mult_ * sigma;

  if (!divergence_m) {
    since_.reset();
    return s;
  }
  s.divergence_m = divergence_m;
  if (*divergence_m > s.threshold_m) {
    if (!since_) since_ = t;   // 超限样本不入窗口:不让偏差段拉高自己的阈值
  } else {
    since_.reset();
    window_.emplace_back(t, *divergence_m);
  }
  s.since = since_;
  return s;
}

}  // namespace gnss_core
```

- [ ] **Step 4: GREEN + 变异**

Run: 构建后 `./build/gnss_core/test_divergence_monitor`。Expected: PASS。
变异（做完恢复）：让超限样本也入窗口 → `ExceedingSamplesStartTheClockAndStayOutOfTheWindow` 必须 FAIL；去掉 `std::max(floor_m_, ...)` → `EmpiricalSigmaHasAFloor` 必须 FAIL；去掉窗口过期循环 → `OldSamplesAgeOutOfTheWindow` 必须 FAIL。

- [ ] **Step 5: 全量测试并提交**

```bash
cd /home/steve/glim_ws/src/glim_ext
git add gnss_core/include/gnss_core/divergence_monitor.hpp gnss_core/src/divergence_monitor.cpp gnss_core/test/test_divergence_monitor.cpp gnss_core/CMakeLists.txt
git commit -m "feat(gnss_core): empirical-sigma threshold and hold clock for device divergence

<RED 与变异结果>

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---
### Task 3: `EventBook`（按规则码独立的事件迟滞状态机）

**Files:**
- Create: `gnss_core/include/gnss_core/event_book.hpp`、`gnss_core/src/event_book.cpp`
- Test: `gnss_core/test/test_event_book.cpp`
- Modify: `gnss_core/CMakeLists.txt`（源文件；`gnss_core_add_test(test_event_book)`）

**Interfaces:**
- Consumes: `Verdict`、`Level`、`level_opens_event`（Task 1）
- Produces:
  - `struct LatLon { double lat; double lon; };`
  - `enum class EventKind { Open, Close };`、`enum class CloseReason { Recovered, Shutdown };`、`const char* close_reason_name(CloseReason)`（`"recovered"` / `"shutdown"`）
  - `struct EventTransition { EventKind kind; double t; double t_open; std::string code; Level level; std::string message; std::optional<LatLon> pos; CloseReason reason; std::map<std::string, double> peak; };`
  - `class EventBook { public: explicit EventBook(double close_hysteresis_s); std::vector<EventTransition> update(double t, const std::vector<Verdict>& verdicts, std::optional<LatLon> pos, const std::map<std::string, double>& metrics); std::vector<EventTransition> close_all(double t); std::vector<std::string> open_codes() const; };`

每个规则码一个跟踪器，语义逐条对应 rtk-monitor `diagnosis/events.py`：

- 本 tick `verdicts` 里 `level_opens_event` 为真的码是"活跃"的（同一码出现多次只算第一次）。
- **已开且活跃**：清除恢复计时；把 `metrics` 并入峰值；`pos` 有值时更新"最近位置"。不产生变更。
- **已开但不活跃**：恢复计时为空则置为 `t`（本 tick 不关）；否则若 `t - 恢复起点 >= close_hysteresis_s` 则关闭，产生 `Close`（`t` 为本 tick，`t_open`、`level`、`message` 取开事件时的结论，`pos` 为最近位置，`reason = Recovered`，`peak` 为累计峰值）。再次活跃会清空恢复计时（relapse）。
- **未开且活跃**：开事件，产生 `Open`（`t = t_open = 本 tick`，`pos` 为本 tick 位置，`peak` 为空）；新跟踪器的峰值从本 tick 的 `metrics` 开始累计，最近位置 = 本 tick 位置。
- 峰值合并：键以 `_min` 结尾的取最小值（不存在时直接存入）；其余键取绝对值最大者、保留符号，且只有 `|v| > |已有值(不存在视为 0)|` 才写入（与 rtk-monitor 一致：值为 0 的指标从未写入）。
- `ok` / `info` 的结论从不开事件。
- 同一 tick 内返回顺序：先所有 `Close`（按码的字典序），再所有 `Open`（按 `verdicts` 顺序）。
- `close_all(t)`：关闭所有已开事件，`reason = Shutdown`，按码字典序返回；之后 `open_codes()` 为空。
- `open_codes()`：按字典序返回当前已开事件的码。

- [ ] **Step 1: 写失败测试**

`gnss_core/test/test_event_book.cpp`：

```cpp
#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "gnss_core/event_book.hpp"
using namespace gnss_core;

namespace {
const Verdict OK{Level::Ok, "rtk_fixed", "RTK 固定"};
const Verdict OUT{Level::Serious, "corr_outage", "差分中断 5s——5G 链路或平台转发问题"};
const Verdict FLOAT_{Level::Warning, "ambiguity", "模糊度无法固定（ratio=1.5）——遮挡过渡区常见"};
const Verdict INFO{Level::Info, "not_fixed", "非固定解（FLOAT）"};

std::vector<Verdict> v(std::initializer_list<Verdict> l) { return l; }

std::vector<std::string> describe(const std::vector<EventTransition>& ts) {
  std::vector<std::string> out;
  for (const auto& t : ts) {
    out.push_back(std::string(t.kind == EventKind::Open ? "open " : "close ") + t.code + "@" +
                  std::to_string(static_cast<int>(t.t)));
  }
  return out;
}
}  // namespace

TEST(EventBook, OpenCloseWithHysteresis) {   // 移植 test_open_close_with_hysteresis
  EventBook b(10.0);
  EXPECT_TRUE(b.update(100, v({OK}), std::nullopt, {}).empty());
  auto t = b.update(101, v({OUT, OK}), LatLon{44.5, 90.2}, {});
  ASSERT_EQ(describe(t), (std::vector<std::string>{"open corr_outage@101"}));
  EXPECT_EQ(t[0].level, Level::Serious);
  ASSERT_TRUE(t[0].pos.has_value());
  EXPECT_DOUBLE_EQ(t[0].pos->lat, 44.5);
  EXPECT_TRUE(b.update(105, v({OUT, OK}), std::nullopt, {}).empty());
  EXPECT_TRUE(b.update(106, v({OK}), std::nullopt, {}).empty());
  EXPECT_TRUE(b.update(110, v({OK}), std::nullopt, {}).empty()) << "恢复 4 s,未满 10 s";
  t = b.update(117, v({OK}), std::nullopt, {});
  ASSERT_EQ(describe(t), (std::vector<std::string>{"close corr_outage@117"}));
  EXPECT_DOUBLE_EQ(t[0].t_open, 101.0);
  EXPECT_EQ(t[0].reason, CloseReason::Recovered);
  EXPECT_EQ(t[0].message, OUT.message);
  EXPECT_TRUE(b.open_codes().empty());
}

TEST(EventBook, DifferentCodesAreTrackedIndependently) {
  // rtk-monitor 在这里会"关旧开新";按码独立后两个事件并存,旧的按自己的迟滞关闭
  EventBook b(10.0);
  b.update(100, v({OUT}), std::nullopt, {});
  auto t = b.update(101, v({FLOAT_}), std::nullopt, {});
  EXPECT_EQ(describe(t), (std::vector<std::string>{"open ambiguity@101"}));
  EXPECT_EQ(b.open_codes(), (std::vector<std::string>{"ambiguity", "corr_outage"}));
  t = b.update(111, v({FLOAT_}), std::nullopt, {});
  EXPECT_EQ(describe(t), (std::vector<std::string>{"close corr_outage@111"}));
  EXPECT_EQ(b.open_codes(), (std::vector<std::string>{"ambiguity"}));
}

TEST(EventBook, SimultaneousHitsOpenSimultaneousEventsInVerdictOrder) {
  EventBook b(10.0);
  const auto t = b.update(100, v({OUT, FLOAT_, INFO}), std::nullopt, {});
  EXPECT_EQ(describe(t), (std::vector<std::string>{"open corr_outage@100", "open ambiguity@100"}));
}

TEST(EventBook, ClosesComeBeforeOpensWithinATick) {
  EventBook b(0.0);
  b.update(100, v({OUT}), std::nullopt, {});
  b.update(101, v({OK}), std::nullopt, {});    // 恢复计时开始
  const auto t = b.update(102, v({FLOAT_}), std::nullopt, {});
  EXPECT_EQ(describe(t), (std::vector<std::string>{"close corr_outage@102", "open ambiguity@102"}));
}

TEST(EventBook, RelapseResetsHysteresis) {   // 移植 test_relapse_resets_hysteresis
  EventBook b(10.0);
  b.update(100, v({OUT}), std::nullopt, {});
  b.update(101, v({OK}), std::nullopt, {});
  b.update(105, v({OUT}), std::nullopt, {});
  EXPECT_TRUE(b.update(120, v({OK}), std::nullopt, {}).empty()) << "复发后恢复计时从 120 重新开始";
  EXPECT_EQ(describe(b.update(131, v({OK}), std::nullopt, {})),
            (std::vector<std::string>{"close corr_outage@131"}));
}

TEST(EventBook, InfoAndOkNeverOpen) {   // 移植 test_info_does_not_open
  EventBook b(10.0);
  EXPECT_TRUE(b.update(100, v({INFO, OK}), std::nullopt, {}).empty());
  EXPECT_TRUE(b.open_codes().empty());
}

TEST(EventBook, SameCodeReopensAfterClosing) {
  EventBook b(1.0);
  b.update(100, v({OUT}), std::nullopt, {});
  b.update(101, v({OK}), std::nullopt, {});
  b.update(102, v({OK}), std::nullopt, {});
  EXPECT_EQ(describe(b.update(114, v({OUT}), std::nullopt, {})),
            (std::vector<std::string>{"open corr_outage@114"}));
}

TEST(EventBook, PeakMetricsAccumulateAndCloseUsesTheLastPosition) {   // 移植 test_peak_metrics_accumulate_and_persist
  EventBook b(1.0);
  b.update(100, v({OUT}), LatLon{44.0, 90.0}, {{"corr_gap_s", 5.0}});
  b.update(101, v({OUT}), LatLon{44.1, 90.1}, {{"corr_gap_s", 12.0}});
  b.update(102, v({OK}), LatLon{44.2, 90.2}, {{"corr_gap_s", 0.0}});   // 不活跃的 tick 不更新峰值与位置
  const auto t = b.update(104, v({OK}), std::nullopt, {});
  ASSERT_EQ(t.size(), 1u);
  EXPECT_DOUBLE_EQ(t[0].peak.at("corr_gap_s"), 12.0);
  ASSERT_TRUE(t[0].pos.has_value());
  EXPECT_DOUBLE_EQ(t[0].pos->lat, 44.1);
  EXPECT_DOUBLE_EQ(t[0].pos->lon, 90.1);
}

TEST(EventBook, MinSuffixMetricsAggregateMinOthersByAbsoluteValue) {   // 移植 test_min_suffix_metrics_aggregate_min
  EventBook b(1.0);
  b.update(100, v({OUT}), std::nullopt, {{"sats_min", 12.0}, {"corr_gap_s", 3.0}, {"divergence_m", 0.0}});
  b.update(101, v({OUT}), std::nullopt, {{"sats_min", 4.0}, {"corr_gap_s", -9.0}});
  b.update(102, v({OUT}), std::nullopt, {{"sats_min", 8.0}, {"corr_gap_s", 5.0}});
  b.update(103, v({OK}), std::nullopt, {});
  const auto t = b.update(105, v({OK}), std::nullopt, {});
  ASSERT_EQ(t.size(), 1u);
  EXPECT_DOUBLE_EQ(t[0].peak.at("sats_min"), 4.0);
  EXPECT_DOUBLE_EQ(t[0].peak.at("corr_gap_s"), -9.0) << "取绝对值最大者并保留符号";
  EXPECT_EQ(t[0].peak.count("divergence_m"), 0u) << "值为 0 的非 _min 指标从未写入";
}

TEST(EventBook, CloseAllAtShutdown) {
  EventBook b(10.0);
  b.update(100, v({OUT, FLOAT_}), LatLon{1.0, 2.0}, {{"corr_gap_s", 4.0}});
  const auto t = b.close_all(130);
  EXPECT_EQ(describe(t), (std::vector<std::string>{"close ambiguity@130", "close corr_outage@130"}));
  for (const auto& e : t) EXPECT_EQ(e.reason, CloseReason::Shutdown);
  EXPECT_TRUE(b.open_codes().empty());
  EXPECT_TRUE(b.close_all(131).empty());
  EXPECT_STREQ(close_reason_name(CloseReason::Shutdown), "shutdown");
}
```

- [ ] **Step 2: 确认 RED**（编译失败），记录。

- [ ] **Step 3: 实现**

`gnss_core/include/gnss_core/event_book.hpp`：

```cpp
#pragma once
// 诊断事件状态机(spec §3 C10)。每个规则码一个带关闭迟滞的跟踪器,多条同时命中就同时开多个事件。
// 迟滞与峰值指标语义移植自 rtk-monitor diagnosis/events.py;不直接写文件,只返回开/关变更。
#include <map>
#include <optional>
#include <string>
#include <vector>

#include "gnss_core/diagnosis.hpp"

namespace gnss_core {

struct LatLon {
  double lat = 0.0, lon = 0.0;
};

enum class EventKind { Open, Close };
enum class CloseReason { Recovered, Shutdown };
const char* close_reason_name(CloseReason reason);

struct EventTransition {
  EventKind kind = EventKind::Open;
  double t = 0.0;        // 本次变更的时刻
  double t_open = 0.0;   // 事件开启时刻(Open 时等于 t)
  std::string code;
  Level level = Level::Warning;   // 开事件时的结论
  std::string message;            // 开事件时的结论
  std::optional<LatLon> pos;      // Open:开启时位置;Close:事件期间最后一个位置
  CloseReason reason = CloseReason::Recovered;   // 仅 Close 有意义
  std::map<std::string, double> peak;            // 仅 Close 有意义
};

class EventBook {
public:
  explicit EventBook(double close_hysteresis_s);
  std::vector<EventTransition> update(double t, const std::vector<Verdict>& verdicts,
                                      std::optional<LatLon> pos,
                                      const std::map<std::string, double>& metrics);
  // 停机:关闭全部已开事件(reason = Shutdown)
  std::vector<EventTransition> close_all(double t);
  std::vector<std::string> open_codes() const;

private:
  struct Tracker {
    Verdict verdict;
    double t_open = 0.0;
    std::optional<LatLon> last_pos;
    std::optional<double> ok_since;
    std::map<std::string, double> peak;
  };
  static EventTransition make_close(const std::string& code, const Tracker& tr, double t, CloseReason reason);
  double hysteresis_s_;
  std::map<std::string, Tracker> open_;
};

}  // namespace gnss_core
```

`gnss_core/src/event_book.cpp`：

```cpp
#include "gnss_core/event_book.hpp"

#include <algorithm>
#include <cmath>

namespace gnss_core {

namespace {
void fold_metrics(std::map<std::string, double>& peak, const std::map<std::string, double>& metrics) {
  for (const auto& [key, value] : metrics) {
    const bool is_min = key.size() >= 4 && key.compare(key.size() - 4, 4, "_min") == 0;
    const auto it = peak.find(key);
    if (is_min) {
      if (it == peak.end() || value < it->second) peak[key] = value;
    } else {
      const double current = it == peak.end() ? 0.0 : it->second;
      if (std::abs(value) > std::abs(current)) peak[key] = value;
    }
  }
}
}  // namespace

const char* close_reason_name(CloseReason reason) {
  return reason == CloseReason::Shutdown ? "shutdown" : "recovered";
}

EventBook::EventBook(double close_hysteresis_s) : hysteresis_s_(close_hysteresis_s) {}

EventTransition EventBook::make_close(const std::string& code, const Tracker& tr, double t,
                                      CloseReason reason) {
  EventTransition e;
  e.kind = EventKind::Close;
  e.t = t;
  e.t_open = tr.t_open;
  e.code = code;
  e.level = tr.verdict.level;
  e.message = tr.verdict.message;
  e.pos = tr.last_pos;
  e.reason = reason;
  e.peak = tr.peak;
  return e;
}

std::vector<EventTransition> EventBook::update(double t, const std::vector<Verdict>& verdicts,
                                               std::optional<LatLon> pos,
                                               const std::map<std::string, double>& metrics) {
  std::vector<const Verdict*> active;
  for (const auto& v : verdicts) {
    if (!level_opens_event(v.level)) continue;
    const bool dup = std::any_of(active.begin(), active.end(),
                                 [&](const Verdict* a) { return a->code == v.code; });
    if (!dup) active.push_back(&v);
  }
  const auto is_active = [&](const std::string& code) {
    return std::any_of(active.begin(), active.end(), [&](const Verdict* a) { return a->code == code; });
  };

  std::vector<EventTransition> out;
  for (auto it = open_.begin(); it != open_.end();) {
    Tracker& tr = it->second;
    if (is_active(it->first)) {
      tr.ok_since.reset();
      fold_metrics(tr.peak, metrics);
      if (pos) tr.last_pos = pos;
      ++it;
    } else if (!tr.ok_since) {
      tr.ok_since = t;
      ++it;
    } else if (t - *tr.ok_since >= hysteresis_s_) {
      out.push_back(make_close(it->first, tr, t, CloseReason::Recovered));
      it = open_.erase(it);
    } else {
      ++it;
    }
  }

  for (const Verdict* v : active) {
    if (open_.count(v->code)) continue;
    Tracker tr;
    tr.verdict = *v;
    tr.t_open = t;
    tr.last_pos = pos;
    fold_metrics(tr.peak, metrics);
    open_.emplace(v->code, tr);

    EventTransition e;
    e.kind = EventKind::Open;
    e.t = t;
    e.t_open = t;
    e.code = v->code;
    e.level = v->level;
    e.message = v->message;
    e.pos = pos;
    out.push_back(std::move(e));
  }
  return out;
}

std::vector<EventTransition> EventBook::close_all(double t) {
  std::vector<EventTransition> out;
  for (const auto& [code, tr] : open_) out.push_back(make_close(code, tr, t, CloseReason::Shutdown));
  open_.clear();
  return out;
}

std::vector<std::string> EventBook::open_codes() const {
  std::vector<std::string> codes;
  for (const auto& [code, tr] : open_) codes.push_back(code);
  return codes;
}

}  // namespace gnss_core
```

- [ ] **Step 4: GREEN + 变异**

Run: `./build/gnss_core/test_event_book`。Expected: PASS。
变异（做完恢复）：
- 首个不活跃 tick 置好 `ok_since` 后不 `continue`，而是在同一 tick 继续做迟滞比较（`hysteresis 0` 时当 tick 就关）→ `ClosesComeBeforeOpensWithinATick` 必须 FAIL
- 活跃时不清 `ok_since` → `RelapseResetsHysteresis` 必须 FAIL
- 非 `_min` 键改成直接覆盖 → `MinSuffixMetricsAggregateMinOthersByAbsoluteValue` 必须 FAIL
- 开新码前先关闭所有其他已开事件（rtk-monitor 的旧行为）→ `DifferentCodesAreTrackedIndependently` 必须 FAIL

- [ ] **Step 5: 全量测试并提交**

```bash
cd /home/steve/glim_ws/src/glim_ext
git add gnss_core/include/gnss_core/event_book.hpp gnss_core/src/event_book.cpp gnss_core/test/test_event_book.cpp gnss_core/CMakeLists.txt
git commit -m "feat(gnss_core): per-code diagnosis event tracking with close hysteresis

<RED 与变异结果>

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---
### Task 4: `BaseStationMonitor`（基站坐标基线与位移，D3 的变更判定）

**Files:**
- Create: `gnss_core/include/gnss_core/base_station_monitor.hpp`、`gnss_core/src/base_station_monitor.cpp`
- Test: `gnss_core/test/test_base_station_monitor.cpp`
- Modify: `gnss_core/CMakeLists.txt`（源文件；`gnss_core_add_test(test_base_station_monitor)`）

**Interfaces:**
- Produces:
  - `struct Ecef { double x; double y; double z; };`
  - `struct BaseFeedResult { std::optional<double> offset_m; bool baseline_learned; bool history_changed; };`
  - `class BaseStationMonitor { public: BaseStationMonitor(double warmup_s, std::optional<Ecef> baseline, std::optional<Ecef> last_history); BaseFeedResult feed(double t, const Ecef& p); BaseFeedResult reset(const Ecef& p); std::optional<Ecef> baseline() const; };`
  - `std::optional<Ecef> read_base_baseline(const std::string& path);`（文件不存在、读失败、格式不对都返回空，不抛）
  - `bool write_base_baseline(const std::string& path, const Ecef& p);`（先写同目录临时文件再 `rename`，失败返回 false，不抛；自动创建父目录）

语义逐条对应 rtk-monitor `diagnosis/base_station.py`：
- `feed`：先判变更——还没有历史坐标，或任一轴与上次历史坐标相差 `> 1e-3` m，则 `history_changed = true` 并更新历史坐标。然后若尚无基线：把 `(t, p)` 放进预热样本；若 `t - 第一个样本时刻 >= warmup_s`，基线 = 各轴中位数（偶数个样本取中间两个的平均），清空样本，`baseline_learned = true`；否则返回（`offset_m` 为空）。有基线时 `offset_m = |p - baseline|`（ECEF 欧氏距离）。
- `reset(p)`：运维确认新基线——基线 = `p`，清空预热样本，历史坐标 = `p`，返回 `{offset_m = 0, baseline_learned = true, history_changed = true}`。
- 基线文件格式：一行 `%.4f,%.4f,%.4f\n`（与 rtk-monitor KV 值相同）。

- [ ] **Step 1: 写失败测试**

`gnss_core/test/test_base_station_monitor.cpp`：

```cpp
#include <gtest/gtest.h>
#include <unistd.h>

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include "gnss_core/base_station_monitor.hpp"
using namespace gnss_core;

namespace {
const Ecef XYZ{-2148744.1, 4426641.2, 4044655.9};
Ecef shifted(double dx) { return Ecef{XYZ.x + dx, XYZ.y, XYZ.z}; }

class TempDir {
public:
  TempDir() {
    const char* base = std::getenv("TMPDIR");
    std::string tmpl = std::string(base ? base : "/tmp") + "/base_monitor_XXXXXX";
    std::vector<char> buf(tmpl.begin(), tmpl.end());
    buf.push_back('\0');
    if (::mkdtemp(buf.data()) != nullptr) path_ = buf.data();
  }
  ~TempDir() {
    if (!path_.empty()) std::filesystem::remove_all(path_);
  }
  const std::string& path() const { return path_; }

private:
  std::string path_;
};
}  // namespace

TEST(BaseStationMonitor, LearnsBaselineThenReportsOffset) {   // 移植 test_learns_baseline_then_reports_offset
  BaseStationMonitor m(100.0, std::nullopt, std::nullopt);
  auto r = m.feed(0, XYZ);
  EXPECT_FALSE(r.offset_m.has_value());
  EXPECT_TRUE(r.history_changed);
  r = m.feed(50, shifted(0.001));
  EXPECT_FALSE(r.offset_m.has_value()) << "仍在预热";
  r = m.feed(101, XYZ);
  ASSERT_TRUE(r.offset_m.has_value());
  EXPECT_TRUE(r.baseline_learned);
  EXPECT_LT(*r.offset_m, 0.002);
  ASSERT_TRUE(m.baseline().has_value());
  EXPECT_NEAR(m.baseline()->x, XYZ.x, 1e-9) << "x 轴三个样本 [x, x+0.001, x] 的中位数是 x";
  r = m.feed(102, shifted(0.5));
  ASSERT_TRUE(r.offset_m.has_value());
  EXPECT_NEAR(*r.offset_m, 0.5, 0.01);
  EXPECT_FALSE(r.baseline_learned);
}

TEST(BaseStationMonitor, EvenSampleCountMedianAveragesTheMiddlePair) {
  BaseStationMonitor m(10.0, std::nullopt, std::nullopt);
  m.feed(0, shifted(0.0));
  m.feed(4, shifted(1.0));
  m.feed(8, shifted(3.0));
  m.feed(10, shifted(10.0));   // 4 个样本 [0, 1, 3, 10] → 中位数 2
  ASSERT_TRUE(m.baseline().has_value());
  EXPECT_NEAR(m.baseline()->x - XYZ.x, 2.0, 1e-6);
}

TEST(BaseStationMonitor, PersistedBaselineSkipsWarmup) {   // 移植 test_baseline_persists_across_restart
  BaseStationMonitor m(600.0, XYZ, XYZ);
  const auto r = m.feed(10, XYZ);
  ASSERT_TRUE(r.offset_m.has_value());
  EXPECT_FALSE(r.history_changed) << "与上次历史坐标相同";
}

TEST(BaseStationMonitor, HistoryRecordsChangesOnly) {   // 移植 test_history_records_changes_only
  BaseStationMonitor m(1.0, std::nullopt, std::nullopt);
  EXPECT_TRUE(m.feed(0, XYZ).history_changed);
  EXPECT_FALSE(m.feed(2, shifted(0.0009)).history_changed) << "1 mm 容差内不算变化";
  EXPECT_TRUE(m.feed(3, shifted(0.5)).history_changed);
}

TEST(BaseStationMonitor, ResetUpdatesBaseline) {   // 移植 test_reset_updates_baseline
  BaseStationMonitor m(1.0, std::nullopt, std::nullopt);
  m.feed(0, XYZ);
  m.feed(2, XYZ);
  const auto rr = m.reset(shifted(0.5));
  EXPECT_TRUE(rr.baseline_learned);
  EXPECT_TRUE(rr.history_changed);
  const auto r = m.feed(6, shifted(0.5));
  ASSERT_TRUE(r.offset_m.has_value());
  EXPECT_LT(*r.offset_m, 0.01);
}

TEST(BaseBaselineFile, RoundTripsAndCreatesParentDirectories) {
  TempDir dir;
  ASSERT_FALSE(dir.path().empty());
  const std::string path = dir.path() + "/sub/base_baseline";
  ASSERT_TRUE(write_base_baseline(path, XYZ));
  const auto back = read_base_baseline(path);
  ASSERT_TRUE(back.has_value());
  EXPECT_NEAR(back->x, XYZ.x, 1e-4);
  EXPECT_NEAR(back->y, XYZ.y, 1e-4);
  EXPECT_NEAR(back->z, XYZ.z, 1e-4);
  std::ifstream in(path);
  std::string line;
  std::getline(in, line);
  EXPECT_EQ(line, "-2148744.1000,4426641.2000,4044655.9000");
}

TEST(BaseBaselineFile, MissingOrCorruptFileMeansNoBaseline) {   // 移植 test_corrupt_kv_falls_back_to_rewarmup
  TempDir dir;
  ASSERT_FALSE(dir.path().empty());
  EXPECT_FALSE(read_base_baseline(dir.path() + "/absent").has_value());
  const std::string path = dir.path() + "/corrupt";
  std::ofstream(path) << "garbage";
  EXPECT_FALSE(read_base_baseline(path).has_value());
  std::ofstream(path) << "1.0,2.0";
  EXPECT_FALSE(read_base_baseline(path).has_value());
  std::ofstream(path) << "1.0,2.0,3.0,4.0";
  EXPECT_FALSE(read_base_baseline(path).has_value());
}

TEST(BaseBaselineFile, WriteFailureReturnsFalse) {
  TempDir dir;
  ASSERT_FALSE(dir.path().empty());
  const std::string blocker = dir.path() + "/file";
  std::ofstream(blocker) << "x";
  EXPECT_FALSE(write_base_baseline(blocker + "/base_baseline", XYZ)) << "父路径是普通文件,写不进去";
}
```

- [ ] **Step 2: 确认 RED**（编译失败），记录。

- [ ] **Step 3: 实现**

`gnss_core/include/gnss_core/base_station_monitor.hpp`：

```cpp
#pragma once
// 基站坐标监测(spec §3 C2 base_shift、D3 base.pos)。移植自 rtk-monitor diagnosis/base_station.py:
// 首次运行取 warmup_s 内 1005/1006 坐标的各轴中位数作基线,之后报告当前坐标相对基线的 ECEF 位移;
// 坐标变化(任一轴 > 1 mm)时标记 history_changed,由调用方写 base.pos。基线持久化到一个小文件。
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace gnss_core {

struct Ecef {
  double x = 0.0, y = 0.0, z = 0.0;   // m
};

struct BaseFeedResult {
  std::optional<double> offset_m;   // 基线尚未建立时为空
  bool baseline_learned = false;    // 本次调用刚建立/更新了基线(调用方应持久化)
  bool history_changed = false;     // 坐标与上次记录不同(调用方应写 base.pos)
};

class BaseStationMonitor {
public:
  BaseStationMonitor(double warmup_s, std::optional<Ecef> baseline, std::optional<Ecef> last_history);
  BaseFeedResult feed(double t, const Ecef& p);
  // 运维确认基站确实搬迁后,以当前坐标为新基线
  BaseFeedResult reset(const Ecef& p);
  std::optional<Ecef> baseline() const { return baseline_; }

private:
  double warmup_s_;
  std::optional<Ecef> baseline_;
  std::optional<Ecef> last_history_;
  std::vector<std::pair<double, Ecef>> samples_;
};

std::optional<Ecef> read_base_baseline(const std::string& path);
bool write_base_baseline(const std::string& path, const Ecef& p);

}  // namespace gnss_core
```

`gnss_core/src/base_station_monitor.cpp`：

```cpp
#include "gnss_core/base_station_monitor.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <sstream>

namespace gnss_core {

namespace {
double median(std::vector<double> v) {
  std::sort(v.begin(), v.end());
  const size_t n = v.size();
  return n % 2 == 1 ? v[n / 2] : 0.5 * (v[n / 2 - 1] + v[n / 2]);
}

double distance(const Ecef& a, const Ecef& b) {
  return std::sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y) + (a.z - b.z) * (a.z - b.z));
}
}  // namespace

BaseStationMonitor::BaseStationMonitor(double warmup_s, std::optional<Ecef> baseline,
                                       std::optional<Ecef> last_history)
    : warmup_s_(warmup_s), baseline_(baseline), last_history_(last_history) {}

BaseFeedResult BaseStationMonitor::feed(double t, const Ecef& p) {
  BaseFeedResult r;
  if (!last_history_ || std::abs(p.x - last_history_->x) > 1e-3 ||
      std::abs(p.y - last_history_->y) > 1e-3 || std::abs(p.z - last_history_->z) > 1e-3) {
    r.history_changed = true;
    last_history_ = p;
  }
  if (!baseline_) {
    samples_.emplace_back(t, p);
    if (t - samples_.front().first < warmup_s_) return r;
    std::vector<double> xs, ys, zs;
    for (const auto& [ts, s] : samples_) {
      xs.push_back(s.x);
      ys.push_back(s.y);
      zs.push_back(s.z);
    }
    baseline_ = Ecef{median(xs), median(ys), median(zs)};
    samples_.clear();
    r.baseline_learned = true;
  }
  r.offset_m = distance(p, *baseline_);
  return r;
}

BaseFeedResult BaseStationMonitor::reset(const Ecef& p) {
  baseline_ = p;
  last_history_ = p;
  samples_.clear();
  BaseFeedResult r;
  r.offset_m = 0.0;
  r.baseline_learned = true;
  r.history_changed = true;
  return r;
}

std::optional<Ecef> read_base_baseline(const std::string& path) {
  std::ifstream in(path);
  if (!in) return std::nullopt;
  std::string line;
  if (!std::getline(in, line)) return std::nullopt;
  std::istringstream ss(line);
  Ecef p;
  char c1 = 0, c2 = 0;
  if (!(ss >> p.x >> c1 >> p.y >> c2 >> p.z) || c1 != ',' || c2 != ',') return std::nullopt;
  std::string rest;
  if (ss >> rest) return std::nullopt;   // 多余字段(比如第四个值)视为损坏
  return p;
}

bool write_base_baseline(const std::string& path, const Ecef& p) {
  std::error_code ec;
  const std::filesystem::path fp(path);
  if (fp.has_parent_path()) std::filesystem::create_directories(fp.parent_path(), ec);
  const std::string tmp = path + ".tmp";
  {
    std::ofstream out(tmp, std::ios::trunc);
    if (!out) return false;
    char buf[128];
    std::snprintf(buf, sizeof(buf), "%.4f,%.4f,%.4f\n", p.x, p.y, p.z);
    out << buf;
    out.flush();
    if (!out) return false;
  }
  std::filesystem::rename(tmp, fp, ec);   // 原子替换:崩溃时要么旧基线要么新基线
  if (ec) {
    std::filesystem::remove(tmp, ec);
    return false;
  }
  return true;
}

}  // namespace gnss_core
```

- [ ] **Step 4: GREEN + 变异**

Run: `./build/gnss_core/test_base_station_monitor`。Expected: PASS。
变异（做完恢复）：变更容差 `1e-3` 改成 `0.0` → `HistoryRecordsChangesOnly` 必须 FAIL；偶数中位数改成取 `v[n/2]` → `EvenSampleCountMedianAveragesTheMiddlePair` 必须 FAIL；删掉 `if (ss >> rest)` 检查 → `MissingOrCorruptFileMeansNoBaseline` 必须 FAIL。

- [ ] **Step 5: 全量测试并提交**

```bash
cd /home/steve/glim_ws/src/glim_ext
git add gnss_core/include/gnss_core/base_station_monitor.hpp gnss_core/src/base_station_monitor.cpp gnss_core/test/test_base_station_monitor.cpp gnss_core/CMakeLists.txt
git commit -m "feat(gnss_core): base station baseline, offset and change detection

<RED 与变异结果>

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---
### Task 5: `diag_io`（`events.log` / `base.pos` 文本格式与追加写入）

**Files:**
- Create: `gnss_core/include/gnss_core/diag_io.hpp`、`gnss_core/src/diag_io.cpp`
- Test: `gnss_core/test/test_diag_io.cpp`
- Modify: `gnss_core/CMakeLists.txt`（源文件；`gnss_core_add_test(test_diag_io)`）

**Interfaces:**
- Consumes: `EventTransition`、`CloseReason`、`close_reason_name`（Task 3）；`level_name`（Task 1）；`Ecef`（Task 4）
- Produces:
  - `std::string format_utc_timestamp(double unix_s);` → `YYYY/MM/DD HH:MM:SS.mmm`（四舍五入到毫秒，进位跨秒/跨天正确）
  - `std::string events_log_header();`
  - `std::string format_event_line(const EventTransition& e);`（不含换行）
  - `std::string base_pos_header();`
  - `std::string format_base_history_line(double t, const Ecef& p);`（不含换行）
  - `class LineAppender { public: bool open(const std::string& path, const std::string& header); bool append(const std::string& line); void close(); bool is_open() const; const std::string& path() const; };`

`events.log` 格式（spec §5.3 "时间 级别 代码 消息"，本计划定死字段）：

```
% gnss_core events.log (time=UTC)
% OPEN : <time> OPEN <level> <code> lat=<deg|-> lon=<deg|-> <message>
% CLOSE: <time> CLOSE <level> <code> lat=<deg|-> lon=<deg|-> opened=<time> duration_s=<s> reason=<recovered|shutdown> peak=<k=v;...|-> <message>
```

- `<time>` 用 `format_utc_timestamp`；`lat`/`lon` 为 `%.9f`，无位置时写 `-`；`duration_s` 为 `%.1f`；`peak` 按键字典序 `key=%.3f` 以 `;` 连接，空时写 `-`；消息里的 `\r`、`\n` 替换为空格。

`base.pos` 格式（spec D3，与 `.pos` 同目录）：

```
% program : gnss_core base history
% time=UTC
%  UTC                    x-ecef(m)        y-ecef(m)        z-ecef(m)
2026/09/14 08:00:00.500  -2148744.1000   4426641.2000   4044655.9000
```

行格式：`<time>  %.4f %.4f %.4f`（时间后两个空格，坐标之间一个空格）。

`LineAppender` 语义（崩溃安全的追加写）：
- `open`：自动创建父目录（失败交给后面的打开判定）；文件不存在或为空时先写 `header`（可为空串）；文件非空且最后一个字节不是 `\n`（上次掉电写了半行）时，先补一个 `\n`，**绝不截断**；打开或写入失败返回 false，不抛。
- `append(line)`：未打开返回 false；`line` 含 `\n` 或 `\r` 返回 false 且不写；否则写 `line + "\n"` 并 `flush`，返回流状态。

- [ ] **Step 1: 写失败测试**

`gnss_core/test/test_diag_io.cpp`：

```cpp
#include <gtest/gtest.h>
#include <unistd.h>

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "gnss_core/diag_io.hpp"
using namespace gnss_core;

namespace {
class TempDir {
public:
  TempDir() {
    const char* base = std::getenv("TMPDIR");
    std::string tmpl = std::string(base ? base : "/tmp") + "/diag_io_XXXXXX";
    std::vector<char> buf(tmpl.begin(), tmpl.end());
    buf.push_back('\0');
    if (::mkdtemp(buf.data()) != nullptr) path_ = buf.data();
  }
  ~TempDir() {
    if (!path_.empty()) std::filesystem::remove_all(path_);
  }
  const std::string& path() const { return path_; }

private:
  std::string path_;
};

std::string slurp(const std::string& path) {
  std::ifstream in(path, std::ios::binary);
  std::ostringstream ss;
  ss << in.rdbuf();
  return ss.str();
}

EventTransition open_event() {
  EventTransition e;
  e.kind = EventKind::Open;
  e.t = 1789372800.5;
  e.t_open = e.t;
  e.code = "corr_outage";
  e.level = Level::Serious;
  e.message = "差分中断 10s——5G 链路或平台转发问题";
  e.pos = LatLon{44.5, 90.28};
  return e;
}
}  // namespace

TEST(DiagIo, UtcTimestampRoundsToMillisecondsAndCarries) {
  EXPECT_EQ(format_utc_timestamp(1789372800.5), "2026/09/14 08:00:00.500");
  EXPECT_EQ(format_utc_timestamp(1789372817.25), "2026/09/14 08:00:17.250");
  EXPECT_EQ(format_utc_timestamp(1789459199.9996), "2026/09/15 08:00:00.000") << "四舍五入进位跨秒";
}

TEST(DiagIo, OpenEventLine) {
  EXPECT_EQ(format_event_line(open_event()),
            "2026/09/14 08:00:00.500 OPEN serious corr_outage lat=44.500000000 lon=90.280000000 "
            "差分中断 10s——5G 链路或平台转发问题");
}

TEST(DiagIo, CloseEventLineCarriesDurationReasonAndSortedPeak) {
  EventTransition e = open_event();
  e.kind = EventKind::Close;
  e.t = 1789372817.25;
  e.pos.reset();
  e.reason = CloseReason::Shutdown;
  e.peak = {{"sats_min", 4.0}, {"corr_gap_s", 12.0}};
  EXPECT_EQ(format_event_line(e),
            "2026/09/14 08:00:17.250 CLOSE serious corr_outage lat=- lon=- "
            "opened=2026/09/14 08:00:00.500 duration_s=16.8 reason=shutdown "
            "peak=corr_gap_s=12.000;sats_min=4.000 差分中断 10s——5G 链路或平台转发问题");
}

TEST(DiagIo, EmptyPeakAndNewlinesInMessage) {
  EventTransition e = open_event();
  e.kind = EventKind::Close;
  e.message = "a\nb\rc";
  const std::string line = format_event_line(e);
  EXPECT_NE(line.find(" peak=- "), std::string::npos) << line;
  EXPECT_NE(line.find("a b c"), std::string::npos) << line;
  EXPECT_EQ(line.find('\n'), std::string::npos);
}

TEST(DiagIo, HeadersAndBaseHistoryLine) {
  EXPECT_EQ(events_log_header().rfind("% gnss_core events.log (time=UTC)\n", 0), 0u);
  EXPECT_EQ(base_pos_header(),
            "% program : gnss_core base history\n"
            "% time=UTC\n"
            "%  UTC                    x-ecef(m)        y-ecef(m)        z-ecef(m)\n");
  EXPECT_EQ(format_base_history_line(1789372800.5, Ecef{-2148744.1, 4426641.2, 4044655.9}),
            "2026/09/14 08:00:00.500  -2148744.1000 4426641.2000 4044655.9000");
}

TEST(LineAppender, WritesHeaderOnceAndAppendsAcrossReopen) {
  TempDir dir;
  ASSERT_FALSE(dir.path().empty());
  const std::string path = dir.path() + "/20260914/events.log";
  {
    LineAppender a;
    ASSERT_TRUE(a.open(path, "% H\n"));
    ASSERT_TRUE(a.append("one"));
  }
  {
    LineAppender a;
    ASSERT_TRUE(a.open(path, "% H\n"));
    ASSERT_TRUE(a.append("two"));
  }
  EXPECT_EQ(slurp(path), "% H\none\ntwo\n");
}

TEST(LineAppender, RepairsAHalfWrittenLastLineWithoutTruncating) {
  TempDir dir;
  ASSERT_FALSE(dir.path().empty());
  const std::string path = dir.path() + "/events.log";
  std::ofstream(path, std::ios::binary) << "% H\nhalf";
  LineAppender a;
  ASSERT_TRUE(a.open(path, "% H\n"));
  ASSERT_TRUE(a.append("next"));
  EXPECT_EQ(slurp(path), "% H\nhalf\nnext\n");
}

TEST(LineAppender, EachLineIsFlushed) {
  TempDir dir;
  ASSERT_FALSE(dir.path().empty());
  const std::string path = dir.path() + "/events.log";
  LineAppender a;
  ASSERT_TRUE(a.open(path, ""));
  ASSERT_TRUE(a.append("x"));
  EXPECT_EQ(slurp(path), "x\n") << "appender 仍打开时内容已经落盘";
}

TEST(LineAppender, RejectsEmbeddedNewlinesAndReportsFailures) {
  TempDir dir;
  ASSERT_FALSE(dir.path().empty());
  LineAppender closed;
  EXPECT_FALSE(closed.append("x"));

  const std::string path = dir.path() + "/events.log";
  LineAppender a;
  ASSERT_TRUE(a.open(path, ""));
  EXPECT_FALSE(a.append("a\nb"));
  EXPECT_EQ(slurp(path), "");

  const std::string blocker = dir.path() + "/file";
  std::ofstream(blocker) << "x";
  LineAppender b;
  EXPECT_FALSE(b.open(blocker + "/events.log", ""));
  EXPECT_FALSE(b.is_open());
}
```

- [ ] **Step 2: 确认 RED**（编译失败），记录。

- [ ] **Step 3: 实现**

`gnss_core/include/gnss_core/diag_io.hpp`：

```cpp
#pragma once
// 诊断落盘格式(spec §5.3 目录布局、D2 events.log、D3 base.pos)与崩溃安全的逐行追加写。
#include <fstream>
#include <string>

#include "gnss_core/base_station_monitor.hpp"
#include "gnss_core/event_book.hpp"

namespace gnss_core {

std::string format_utc_timestamp(double unix_s);

std::string events_log_header();
std::string format_event_line(const EventTransition& e);

std::string base_pos_header();
std::string format_base_history_line(double t, const Ecef& p);

class LineAppender {
public:
  bool open(const std::string& path, const std::string& header);
  bool append(const std::string& line);
  void close();
  bool is_open() const { return out_.is_open(); }
  const std::string& path() const { return path_; }

private:
  std::ofstream out_;
  std::string path_;
};

}  // namespace gnss_core
```

`gnss_core/src/diag_io.cpp`：

```cpp
#include "gnss_core/diag_io.hpp"

#include <cmath>
#include <cstdio>
#include <ctime>
#include <filesystem>

namespace gnss_core {

namespace {
std::string latlon_fields(const std::optional<LatLon>& pos) {
  if (!pos) return "lat=- lon=-";
  char buf[96];
  std::snprintf(buf, sizeof(buf), "lat=%.9f lon=%.9f", pos->lat, pos->lon);
  return buf;
}

std::string single_line(std::string s) {
  for (char& c : s) {
    if (c == '\n' || c == '\r') c = ' ';
  }
  return s;
}
}  // namespace

std::string format_utc_timestamp(double unix_s) {
  const long long total_ms = std::llround(unix_s * 1000.0);
  long long secs = total_ms / 1000;
  long long ms = total_ms % 1000;
  if (ms < 0) {
    ms += 1000;
    secs -= 1;
  }
  const std::time_t tt = static_cast<std::time_t>(secs);
  std::tm tm{};
  gmtime_r(&tt, &tm);
  char buf[40];
  std::snprintf(buf, sizeof(buf), "%04d/%02d/%02d %02d:%02d:%02d.%03lld", tm.tm_year + 1900,
                tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec, ms);
  return buf;
}

std::string events_log_header() {
  return "% gnss_core events.log (time=UTC)\n"
         "% OPEN : <time> OPEN <level> <code> lat=<deg|-> lon=<deg|-> <message>\n"
         "% CLOSE: <time> CLOSE <level> <code> lat=<deg|-> lon=<deg|-> opened=<time> duration_s=<s> "
         "reason=<recovered|shutdown> peak=<k=v;...|-> <message>\n";
}

std::string format_event_line(const EventTransition& e) {
  std::string line = format_utc_timestamp(e.t);
  line += e.kind == EventKind::Open ? " OPEN " : " CLOSE ";
  line += level_name(e.level);
  line += " " + e.code + " " + latlon_fields(e.pos);
  if (e.kind == EventKind::Close) {
    char dur[48];
    std::snprintf(dur, sizeof(dur), "%.1f", e.t - e.t_open);
    line += " opened=" + format_utc_timestamp(e.t_open) + " duration_s=" + dur +
            " reason=" + close_reason_name(e.reason) + " peak=";
    if (e.peak.empty()) {
      line += "-";
    } else {
      bool first = true;
      for (const auto& [key, value] : e.peak) {   // std::map:键字典序
        char kv[128];
        std::snprintf(kv, sizeof(kv), "%s%s=%.3f", first ? "" : ";", key.c_str(), value);
        line += kv;
        first = false;
      }
    }
  }
  line += " " + single_line(e.message);
  return line;
}

std::string base_pos_header() {
  return "% program : gnss_core base history\n"
         "% time=UTC\n"
         "%  UTC                    x-ecef(m)        y-ecef(m)        z-ecef(m)\n";
}

std::string format_base_history_line(double t, const Ecef& p) {
  char buf[128];
  std::snprintf(buf, sizeof(buf), "  %.4f %.4f %.4f", p.x, p.y, p.z);
  return format_utc_timestamp(t) + buf;
}

bool LineAppender::open(const std::string& path, const std::string& header) {
  close();
  std::error_code ec;
  const std::filesystem::path fp(path);
  if (fp.has_parent_path()) std::filesystem::create_directories(fp.parent_path(), ec);

  bool is_new = true;
  bool needs_newline = false;
  const auto size = std::filesystem::file_size(fp, ec);
  if (!ec && size > 0) {
    is_new = false;
    std::ifstream in(path, std::ios::binary);
    in.seekg(static_cast<std::streamoff>(size) - 1);
    char last = 0;
    if (!in.get(last)) return false;   // 读不到最后一个字节:宁可不写,也不往可能损坏的文件里追加
    needs_newline = last != '\n';
  }

  out_.open(path, std::ios::app | std::ios::binary);
  if (!out_.is_open()) return false;
  path_ = path;
  if (is_new) out_ << header;
  if (needs_newline) out_ << '\n';   // 上次掉电留下半行:补换行,不截断
  out_.flush();
  if (!out_) {
    close();
    return false;
  }
  return true;
}

bool LineAppender::append(const std::string& line) {
  if (!out_.is_open()) return false;
  if (line.find('\n') != std::string::npos || line.find('\r') != std::string::npos) return false;
  out_ << line << '\n';
  out_.flush();
  return static_cast<bool>(out_);
}

void LineAppender::close() {
  if (out_.is_open()) out_.close();
  out_.clear();
  path_.clear();
}

}  // namespace gnss_core
```

- [ ] **Step 4: GREEN + 变异**

Run: `./build/gnss_core/test_diag_io`。Expected: PASS。
变异（做完恢复）：`format_utc_timestamp` 改成截断而非四舍五入（`static_cast<long long>(unix_s * 1000.0)`）→ `UtcTimestampRoundsToMillisecondsAndCarries` 必须 FAIL；去掉 `needs_newline` 补换行 → `RepairsAHalfWrittenLastLineWithoutTruncating` 必须 FAIL；`append` 去掉 `flush()` → `EachLineIsFlushed` 必须 FAIL（若 libstdc++ 缓冲使该变异侥幸通过，如实记录并改用多行更大的写入量复测）。

- [ ] **Step 5: 全量测试并提交**

```bash
cd /home/steve/glim_ws/src/glim_ext
git add gnss_core/include/gnss_core/diag_io.hpp gnss_core/src/diag_io.cpp gnss_core/test/test_diag_io.cpp gnss_core/CMakeLists.txt
git commit -m "feat(gnss_core): events.log and base.pos line formats with crash-safe appending

<RED 与变异结果>

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---
### Task 6: `retention`（保留天数 / 磁盘水位清理判定，A6/D4）

**Files:**
- Create: `gnss_core/include/gnss_core/retention.hpp`、`gnss_core/src/retention.cpp`
- Test: `gnss_core/test/test_retention.cpp`
- Modify: `gnss_core/CMakeLists.txt`（源文件；`gnss_core_add_test(test_retention)`）

**Interfaces:**
- Produces:
  - `std::optional<int> parse_day_dir_date(const std::string& name);` —— `.pos` 根目录下的 `YYYYMMDD`（恰好 8 位数字且是合法日期）
  - `std::optional<int> parse_bag_dir_date(const std::string& name);` —— 录包目录 `gnss_YYYYMMDD_HHMMSS`（`record_gnss.sh` 用 `date -u +%Y%m%d_%H%M%S` 命名）
  - `int utc_yyyymmdd(double unix_s);`
  - `int days_between(int from_yyyymmdd, int to_yyyymmdd);`（`to - from` 的天数，可为负）
  - `struct DatedEntry { std::string name; int yyyymmdd; };`
  - `std::vector<std::string> sweep_dated_entries(std::vector<DatedEntry> entries, int today_yyyymmdd, int retention_days, double watermark_pct, const std::function<double()>& used_pct, const std::function<bool(const std::string&)>& remove);`
  - `struct CleanupReport { std::vector<std::string> deleted; std::string error; };`
  - `CleanupReport cleanup_dated_root(const std::string& root, const std::function<std::optional<int>(const std::string&)>& parse_date, int today_yyyymmdd, int retention_days, double watermark_pct);`

`sweep_dated_entries` 语义（移植 rtk-monitor `storage/cleanup.py`，外加"最新一项永不删"）：
1. 按 `(yyyymmdd, name)` 升序排序。
2. 只遍历到倒数第二项（**最新一项永不删**：可能是正在写的录包目录）。
3. 对每项：日期 `>= today` 时停止；每项都调用一次 `used_pct()`；`too_old = days_between(日期, today) > retention_days`，`over = used_pct() > watermark_pct`；两者都不成立时停止；`remove(name)` 返回 false 时停止（不在删不掉的项上反复尝试）；否则记入已删除。
4. 返回已删除的名字（按删除顺序）。

`cleanup_dated_root`：只看 `root` 下的**目录**，名字能被 `parse_date` 解析的才参与；`used_pct` = `(capacity - free) / capacity * 100`（`std::filesystem::space`，与 Python `shutil.disk_usage` 的 used/total 一致）；`remove` = `std::filesystem::remove_all(root/name)` 不出错即成功；`root` 不存在或无法遍历时 `error` 写明原因、`deleted` 为空；不抛。

- [ ] **Step 1: 写失败测试**

`gnss_core/test/test_retention.cpp`：

```cpp
#include <gtest/gtest.h>
#include <unistd.h>

#include <algorithm>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "gnss_core/retention.hpp"
using namespace gnss_core;

namespace {
const int TODAY = 20260910;

// 按调用次序返回序列值,用完后一直返回最后一个值(配合"删一个就降到水位下"的用例)
std::function<double()> usage_sequence(std::vector<double> seq) {
  auto next = std::make_shared<size_t>(0);
  return [seq, next]() {
    const double v = seq[std::min(*next, seq.size() - 1)];
    ++*next;
    return v;
  };
}

std::function<bool(const std::string&)> record_removal(std::vector<std::string>& removed) {
  return [&removed](const std::string& name) {
    removed.push_back(name);
    return true;
  };
}
}  // namespace

TEST(Retention, ParsesDayAndBagDirectoryNames) {
  EXPECT_EQ(parse_day_dir_date("20260914"), 20260914);
  EXPECT_FALSE(parse_day_dir_date("2026091").has_value());
  EXPECT_FALSE(parse_day_dir_date("20261301").has_value()) << "13 月";
  EXPECT_FALSE(parse_day_dir_date("20260230").has_value()) << "2 月 30 日";
  EXPECT_FALSE(parse_day_dir_date("gnss_20260914_101010").has_value());
  EXPECT_EQ(parse_bag_dir_date("gnss_20260914_101010"), 20260914);
  EXPECT_FALSE(parse_bag_dir_date("gnss_20260914").has_value());
  EXPECT_FALSE(parse_bag_dir_date("20260914").has_value());
}

TEST(Retention, DateArithmetic) {
  EXPECT_EQ(days_between(20260820, 20260910), 21);
  EXPECT_EQ(days_between(20241231, 20250101), 1);
  EXPECT_EQ(days_between(20240228, 20240301), 2) << "闰年";
  EXPECT_EQ(days_between(20260910, 20260901), -9);
  EXPECT_EQ(utc_yyyymmdd(1789372800.5), 20260914);
}

TEST(Retention, DeletesBeyondRetention) {   // 移植 test_deletes_beyond_retention
  std::vector<std::string> removed;
  const auto deleted = sweep_dated_entries(
      {{"20260910", 20260910}, {"20260820", 20260820}, {"20260901", 20260901}}, TODAY, 14, 85.0,
      usage_sequence({10.0}), record_removal(removed));
  EXPECT_EQ(deleted, (std::vector<std::string>{"20260820"}));
  EXPECT_EQ(removed, deleted);
}

TEST(Retention, DeletesOldestUntilUnderWatermark) {   // 移植 test_deletes_oldest_when_over_watermark
  std::vector<std::string> removed;
  const auto deleted = sweep_dated_entries(
      {{"20260908", 20260908}, {"20260909", 20260909}, {"20260910", 20260910}}, TODAY, 14, 85.0,
      usage_sequence({90.0, 80.0}), record_removal(removed));
  EXPECT_EQ(deleted, (std::vector<std::string>{"20260908"}));
}

TEST(Retention, NeverDeletesToday) {   // 移植 test_never_deletes_today
  std::vector<std::string> removed;
  EXPECT_TRUE(sweep_dated_entries({{"20260910", 20260910}, {"20260911", 20260911}}, TODAY, 0, 0.0,
                                  usage_sequence({100.0}), record_removal(removed))
                  .empty());
}

TEST(Retention, NeverDeletesTheNewestEntryEvenIfOld) {
  // 跨天录包:最新目录名字是昨天的日期,但仍在写
  std::vector<std::string> removed;
  const auto deleted = sweep_dated_entries(
      {{"gnss_20260801_000000", 20260801}, {"gnss_20260802_000000", 20260802}}, TODAY, 14, 85.0,
      usage_sequence({10.0}), record_removal(removed));
  EXPECT_EQ(deleted, (std::vector<std::string>{"gnss_20260801_000000"}));
}

TEST(Retention, StopsWhenARemovalFails) {
  int calls = 0;
  const auto deleted = sweep_dated_entries(
      {{"20260801", 20260801}, {"20260802", 20260802}, {"20260803", 20260803}}, TODAY, 14, 85.0,
      usage_sequence({10.0}), [&calls](const std::string&) {
        ++calls;
        return false;
      });
  EXPECT_TRUE(deleted.empty());
  EXPECT_EQ(calls, 1);
}

TEST(Retention, CleanupDatedRootOnARealDirectory) {
  const char* base = std::getenv("TMPDIR");
  std::string tmpl = std::string(base ? base : "/tmp") + "/retention_XXXXXX";
  std::vector<char> buf(tmpl.begin(), tmpl.end());
  buf.push_back('\0');
  ASSERT_NE(::mkdtemp(buf.data()), nullptr);
  const std::string root = buf.data();
  for (const char* d : {"20260801", "20260909", "20260910", "notadate"}) {
    std::filesystem::create_directories(root + "/" + d);
    std::ofstream(root + "/" + d + "/can.pos") << "x";
  }
  std::ofstream(root + "/20260802") << "a regular file, not a directory";

  const auto report = cleanup_dated_root(root, parse_day_dir_date, TODAY, 14, 100.0);
  EXPECT_TRUE(report.error.empty()) << report.error;
  EXPECT_EQ(report.deleted, (std::vector<std::string>{"20260801"}));
  EXPECT_FALSE(std::filesystem::exists(root + "/20260801"));
  EXPECT_TRUE(std::filesystem::exists(root + "/20260909"));
  EXPECT_TRUE(std::filesystem::exists(root + "/notadate"));
  EXPECT_TRUE(std::filesystem::exists(root + "/20260802")) << "普通文件不参与";

  const auto missing = cleanup_dated_root(root + "/absent", parse_day_dir_date, TODAY, 14, 100.0);
  EXPECT_FALSE(missing.error.empty());
  EXPECT_TRUE(missing.deleted.empty());
  std::filesystem::remove_all(root);
}
```

- [ ] **Step 2: 确认 RED**（编译失败），记录。

- [ ] **Step 3: 实现**

`gnss_core/include/gnss_core/retention.hpp`：

```cpp
#pragma once
// 保留天数与磁盘水位清理(spec §3 A6/D4、§5.2)。移植自 rtk-monitor storage/cleanup.py:
// 按日期从旧到新逐个删除,直到"既不超期也不超水位";今天及以后的不删;
// 另加一条:最新的一项永不删(跨天录包时最新目录可能早于今天但仍在写)。
#include <functional>
#include <optional>
#include <string>
#include <vector>

namespace gnss_core {

std::optional<int> parse_day_dir_date(const std::string& name);
std::optional<int> parse_bag_dir_date(const std::string& name);
int utc_yyyymmdd(double unix_s);
int days_between(int from_yyyymmdd, int to_yyyymmdd);

struct DatedEntry {
  std::string name;
  int yyyymmdd = 0;
};

std::vector<std::string> sweep_dated_entries(std::vector<DatedEntry> entries, int today_yyyymmdd,
                                             int retention_days, double watermark_pct,
                                             const std::function<double()>& used_pct,
                                             const std::function<bool(const std::string&)>& remove);

struct CleanupReport {
  std::vector<std::string> deleted;
  std::string error;   // 非空表示没能完成(比如 root 不存在)
};

CleanupReport cleanup_dated_root(const std::string& root,
                                 const std::function<std::optional<int>(const std::string&)>& parse_date,
                                 int today_yyyymmdd, int retention_days, double watermark_pct);

}  // namespace gnss_core
```

`gnss_core/src/retention.cpp`：

```cpp
#include "gnss_core/retention.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <ctime>
#include <filesystem>

namespace gnss_core {

namespace {
// Howard Hinnant 的 days_from_civil:公历日期 → 自 1970-01-01 起的天数
long long days_from_civil(int y, unsigned m, unsigned d) {
  y -= m <= 2;
  const long long era = (y >= 0 ? y : y - 399) / 400;
  const unsigned yoe = static_cast<unsigned>(y - era * 400);
  const unsigned doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1;
  const unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
  return era * 146097 + static_cast<long long>(doe) - 719468;
}

bool all_digits(const std::string& s) {
  return !s.empty() && std::all_of(s.begin(), s.end(), [](unsigned char c) { return std::isdigit(c); });
}

std::optional<int> parse_yyyymmdd(const std::string& s) {
  if (s.size() != 8 || !all_digits(s)) return std::nullopt;
  const int v = std::stoi(s);
  const int y = v / 10000, m = (v / 100) % 100, d = v % 100;
  if (m < 1 || m > 12 || d < 1) return std::nullopt;
  static const int kDays[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
  const bool leap = (y % 4 == 0 && y % 100 != 0) || y % 400 == 0;
  const int max_day = kDays[m - 1] + (m == 2 && leap ? 1 : 0);
  if (d > max_day) return std::nullopt;
  return v;
}

long long to_days(int yyyymmdd) {
  return days_from_civil(yyyymmdd / 10000, static_cast<unsigned>((yyyymmdd / 100) % 100),
                         static_cast<unsigned>(yyyymmdd % 100));
}
}  // namespace

std::optional<int> parse_day_dir_date(const std::string& name) { return parse_yyyymmdd(name); }

std::optional<int> parse_bag_dir_date(const std::string& name) {
  // gnss_YYYYMMDD_HHMMSS
  if (name.size() != 20 || name.compare(0, 5, "gnss_") != 0 || name[13] != '_') return std::nullopt;
  if (!all_digits(name.substr(14, 6))) return std::nullopt;
  return parse_yyyymmdd(name.substr(5, 8));
}

int utc_yyyymmdd(double unix_s) {
  const std::time_t tt = static_cast<std::time_t>(std::floor(unix_s));
  std::tm tm{};
  gmtime_r(&tt, &tm);
  return (tm.tm_year + 1900) * 10000 + (tm.tm_mon + 1) * 100 + tm.tm_mday;
}

int days_between(int from_yyyymmdd, int to_yyyymmdd) {
  return static_cast<int>(to_days(to_yyyymmdd) - to_days(from_yyyymmdd));
}

std::vector<std::string> sweep_dated_entries(std::vector<DatedEntry> entries, int today_yyyymmdd,
                                             int retention_days, double watermark_pct,
                                             const std::function<double()>& used_pct,
                                             const std::function<bool(const std::string&)>& remove) {
  std::sort(entries.begin(), entries.end(), [](const DatedEntry& a, const DatedEntry& b) {
    return a.yyyymmdd != b.yyyymmdd ? a.yyyymmdd < b.yyyymmdd : a.name < b.name;
  });
  std::vector<std::string> deleted;
  for (size_t i = 0; i + 1 < entries.size(); ++i) {   // 最新一项永不删
    const DatedEntry& e = entries[i];
    if (e.yyyymmdd >= today_yyyymmdd) break;
    const bool over = used_pct() > watermark_pct;
    const bool too_old = days_between(e.yyyymmdd, today_yyyymmdd) > retention_days;
    if (!(too_old || over)) break;
    if (!remove(e.name)) break;
    deleted.push_back(e.name);
  }
  return deleted;
}

CleanupReport cleanup_dated_root(const std::string& root,
                                 const std::function<std::optional<int>(const std::string&)>& parse_date,
                                 int today_yyyymmdd, int retention_days, double watermark_pct) {
  namespace fs = std::filesystem;
  CleanupReport report;
  std::error_code ec;
  std::vector<DatedEntry> entries;
  fs::directory_iterator it(root, ec);
  if (ec) {
    report.error = "无法遍历 " + root + ": " + ec.message();
    return report;
  }
  for (const auto& entry : it) {
    std::error_code type_ec;
    if (!entry.is_directory(type_ec) || type_ec) continue;
    const std::string name = entry.path().filename().string();
    if (const auto date = parse_date(name)) entries.push_back({name, *date});
  }
  const auto used_pct = [&root]() {
    std::error_code space_ec;
    const auto info = fs::space(root, space_ec);
    if (space_ec || info.capacity == 0) return 0.0;   // 查不到用量时只按保留天数删
    return static_cast<double>(info.capacity - info.free) / static_cast<double>(info.capacity) * 100.0;
  };
  const auto remove = [&root](const std::string& name) {
    std::error_code rm_ec;
    fs::remove_all(fs::path(root) / name, rm_ec);
    return !rm_ec;
  };
  report.deleted = sweep_dated_entries(std::move(entries), today_yyyymmdd, retention_days, watermark_pct,
                                       used_pct, remove);
  return report;
}

}  // namespace gnss_core
```

- [ ] **Step 4: GREEN + 变异**

Run: `./build/gnss_core/test_retention`。Expected: PASS。
变异（做完恢复）：循环条件改回 `i < entries.size()`（去掉"最新一项永不删"）→ `NeverDeletesTheNewestEntryEvenIfOld` 必须 FAIL；`remove` 失败后 `continue` 而不是 `break` → `StopsWhenARemovalFails` 必须 FAIL；日期校验去掉闰年/月天数判断 → `ParsesDayAndBagDirectoryNames` 必须 FAIL。

- [ ] **Step 5: 全量测试并提交**

```bash
cd /home/steve/glim_ws/src/glim_ext
git add gnss_core/include/gnss_core/retention.hpp gnss_core/src/retention.cpp gnss_core/test/test_retention.cpp gnss_core/CMakeLists.txt
git commit -m "feat(gnss_core): retention-days and disk-watermark sweep for dated data directories

<RED 与变异结果>

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---
### Task 7: `DiagnosisEngine`（串起全部部件）

**Files:**
- Create: `gnss_core/include/gnss_core/diagnosis_engine.hpp`、`gnss_core/src/diagnosis_engine.cpp`
- Create: `gnss_core/test/diag_test_fixtures.hpp`
- Modify: `gnss_core/test/test_rtkstat.cpp`（删除本地 `sat_line`，改用共享头文件）
- Test: `gnss_core/test/test_diagnosis_engine.cpp`
- Modify: `gnss_core/CMakeLists.txt`（源文件；`gnss_core_add_test(test_diagnosis_engine)`）

**Interfaces:**
- Consumes: Task 1–4 的全部类型；既有 `RtcmFramer`、`parse_base_station`、`BaseStationCoords`（`rtcm.hpp`），`StatEpochAccumulator`、`SlipWindow`、`parse_sat_line`（`rtkstat.hpp`）
- Produces（轮 3b 的 `gnss_diag_node` 只用这些）：
  - `struct BaseUpdate { double t; BaseStationCoords coords; BaseFeedResult feed; };`
  - `struct TickResult { DiagnosisResult result; std::vector<EventTransition> transitions; DivergenceState divergence; };`
  - `class DiagnosisEngine { public: DiagnosisEngine(DiagnosisConfig cfg, std::vector<ControlPoint> control_points, bool solver_enabled, std::optional<Ecef> persisted_baseline, std::optional<Ecef> last_history); std::vector<BaseUpdate> on_corrections(double t, const uint8_t* data, size_t len); void on_solution(double t, const SolutionSample& s); void on_device_solution(double t, const SolutionSample& s); void on_stat_line(double t, const std::string& line); TickResult tick(double now); std::vector<EventTransition> shutdown(double now); std::optional<Ecef> baseline() const; };`
  - 测试夹具（命名空间 `gnss_core::test_fixtures`）：`std::string sat_line(const char* sat, double tow, int frq, double el, double resp, double snr, int vsat, int slipc, int rejc);`、`std::vector<uint8_t> make_1005_frame(int station_id, double x, double y, double z);`

引擎语义（对应 rtk-monitor `main.py` 的 `_diagnosis_tick` 与各 `_on_*` 回调）：
- 构造时 `validate_diagnosis_config(cfg)`，非法抛 `std::invalid_argument`。
- `on_corrections(t, data, len)`：`len > 0` 时 `corr_last_t = t`（任何差分字节都算链路存活）；把字节喂给 `RtcmFramer`，每条能解析成 1005/1006 的消息喂给 `BaseStationMonitor`，`offset_m` 有值时更新"当前基站位移"（保持到下一条 1005/1006）；返回本次的全部 `BaseUpdate`（调用方据 `history_changed` 写 `base.pos`、据 `baseline_learned` 持久化基线）。
- `on_solution` / `on_device_solution`：记下样本与到达时刻 `t`（前者是 rtkrcv 独立解，后者是 610 融合解）。
- `on_stat_line(t, line)`：喂给 `StatEpochAccumulator`；能解析为 `$SAT` 时同时喂 `SlipWindow::feed(t, sat, slipc)`（用到达时刻，不用 `$SAT` 的 tow），并记 `stat_t = t`。
- `tick(now)`：
  1. 新鲜度：`fresh(x) = x 有值 && now - x < sol_stale_s`。`sol` = 新鲜时的独立解，`dev` = 新鲜时的 610 解。
  2. `corr_age` = `sol ? sol->age : (dev ? dev->age : 空)`。
  3. 偏差：`sol && dev && |sol_t - dev_t| < divergence_pair_max_dt_s` 时 `d = geodesic_distance_m(sol, dev)`，否则空；`DivergenceMonitor::update(now, d, sol ? hypot(sol->sdn, sol->sde) : 0.0)`。
  4. `sats` = `stat_t` 新鲜时的 `StatEpochAccumulator::epoch()`，否则空；`slip_count_30s = SlipWindow::count(now)`。
  5. 组 `DiagnosisInput` 调 `evaluate_rules`。
  6. 事件位置 `pos` = `sol` 的经纬度，否则 `dev` 的，否则空。指标：`divergence_m = d 或 0`；`sats_min = sol->ns`（**仅有 sol 时**）；`corr_gap_s = corr_last_t ? now - corr_last_t : 0`。
  7. `EventBook::update(now, verdicts, pos, metrics)`，结果连同诊断结论、偏差状态一起返回。
- `shutdown(now)` = `EventBook::close_all(now)`。

- [ ] **Step 1: 共享测试夹具**

`gnss_core/test/diag_test_fixtures.hpp`：

```cpp
#pragma once
// 诊断相关测试共用的输入构造:rtkrcv $SAT 行、RTCM 1005 帧。
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

#include "gnss_core/rtcm.hpp"

namespace gnss_core::test_fixtures {

// RTKLIB 列序:$SAT,week,tow,sat,frq,az,el,resp,resc,vsat,snr,fix,slip,lock,outc,slipc,rejc
inline std::string sat_line(const char* sat, double tow, int frq, double el, double resp, double snr,
                            int vsat, int slipc, int rejc) {
  return "$SAT," + std::to_string(2380) + "," + std::to_string(tow) + "," + sat + "," +
         std::to_string(frq) + ",123.4," + std::to_string(el) + "," + std::to_string(resp) +
         ",0.001," + std::to_string(vsat) + "," + std::to_string(snr) + ",1,0,100,0," +
         std::to_string(slipc) + "," + std::to_string(rejc);
}

inline void set_bits(std::vector<uint8_t>& buf, int pos, int len, int64_t value) {
  const uint64_t v = static_cast<uint64_t>(value);   // 负数取二进制补码的低 len 位
  for (int i = 0; i < len; ++i) {
    if ((v >> (len - 1 - i)) & 1u) {
      const int idx = pos + i;
      buf[static_cast<size_t>(idx / 8)] |= static_cast<uint8_t>(0x80 >> (idx % 8));
    }
  }
}

// RTCM3 1005:DF002(12) DF003(12) DF021(6) DF022-024+DF141(4) DF025 X(38) DF142+DF001(2)
//            DF026 Y(38) DF364(2) DF027 Z(38),共 152 bit = 19 字节;坐标单位 0.1 mm
inline std::vector<uint8_t> make_1005_frame(int station_id, double x, double y, double z) {
  std::vector<uint8_t> payload(19, 0);
  set_bits(payload, 0, 12, 1005);
  set_bits(payload, 12, 12, station_id);
  set_bits(payload, 34, 38, std::llround(x * 10000.0));
  set_bits(payload, 74, 38, std::llround(y * 10000.0));
  set_bits(payload, 114, 38, std::llround(z * 10000.0));
  std::vector<uint8_t> frame = {0xD3, 0x00, static_cast<uint8_t>(payload.size())};
  frame.insert(frame.end(), payload.begin(), payload.end());
  const uint32_t crc = crc24q(frame.data(), frame.size());
  frame.push_back(static_cast<uint8_t>((crc >> 16) & 0xFF));
  frame.push_back(static_cast<uint8_t>((crc >> 8) & 0xFF));
  frame.push_back(static_cast<uint8_t>(crc & 0xFF));
  return frame;
}

}  // namespace gnss_core::test_fixtures
```

修改 `gnss_core/test/test_rtkstat.cpp`：删除匿名命名空间里的 `sat_line` 定义（若匿名命名空间因此为空则一并删除），在 include 区加 `#include "diag_test_fixtures.hpp"` 与 `using gnss_core::test_fixtures::sat_line;`。`./build/gnss_core/test_rtkstat` 必须照常全部通过。

- [ ] **Step 2: 写失败测试**

`gnss_core/test/test_diagnosis_engine.cpp`：

```cpp
#include <gtest/gtest.h>

#include <algorithm>
#include <string>
#include <vector>

#include "diag_test_fixtures.hpp"
#include "gnss_core/diagnosis_engine.hpp"
using namespace gnss_core;
using gnss_core::test_fixtures::make_1005_frame;
using gnss_core::test_fixtures::sat_line;

namespace {
const double X = -2148744.1, Y = 4426641.2, Z = 4044655.9;

SolutionSample fixed(double lat = 44.5, double lon = 90.28) {
  SolutionSample s;
  s.quality = Quality::FIXED;
  s.lat = lat;
  s.lon = lon;
  s.ns = 20;
  s.sdn = 0.011;
  s.sde = 0.012;
  s.age = 0.8;
  s.ratio = 25.0;
  return s;
}

void corrections(DiagnosisEngine& e, double t) {
  const auto f = make_1005_frame(1, X, Y, Z);
  e.on_corrections(t, f.data(), f.size());
}

bool has_code(const TickResult& r, const std::string& code) {
  return std::any_of(r.result.verdicts.begin(), r.result.verdicts.end(),
                     [&](const Verdict& v) { return v.code == code; });
}

bool opened(const TickResult& r, const std::string& code) {
  return std::any_of(r.transitions.begin(), r.transitions.end(), [&](const EventTransition& t) {
    return t.kind == EventKind::Open && t.code == code;
  });
}

DiagnosisEngine make_engine(DiagnosisConfig cfg = {}) {
  return DiagnosisEngine(cfg, {}, true, std::nullopt, std::nullopt);
}
}  // namespace

TEST(DiagTestFixtures, Make1005FrameRoundTripsThroughTheParser) {
  const auto frame = make_1005_frame(1234, -22458.1234, 4548123.4567, 4451234.8901);
  RtcmFramer framer;
  const auto msgs = framer.feed(frame);
  ASSERT_EQ(msgs.size(), 1u);
  BaseStationCoords c;
  ASSERT_TRUE(parse_base_station(msgs[0], c));
  EXPECT_EQ(c.station_id, 1234);
  EXPECT_NEAR(c.x, -22458.1234, 1e-4);
  EXPECT_NEAR(c.y, 4548123.4567, 1e-4);
  EXPECT_NEAR(c.z, 4451234.8901, 1e-4);
}

TEST(DiagnosisEngine, RejectsAnInvalidConfig) {
  DiagnosisConfig cfg;
  cfg.corr_gap_s = 0.0;
  EXPECT_THROW(make_engine(cfg), std::invalid_argument);
}

TEST(DiagnosisEngine, CorrectionsGapOpensCorrOutage) {   // 移植 test_epochs_and_corr_outage_event
  auto e = make_engine();
  corrections(e, 100.0);
  e.on_solution(100.0, fixed());
  auto r = e.tick(101.0);
  EXPECT_EQ(r.result.status().code, "rtk_fixed");
  EXPECT_TRUE(r.transitions.empty());
  e.on_solution(103.5, fixed());
  r = e.tick(104.0);
  EXPECT_EQ(r.result.status().code, "corr_outage");
  EXPECT_TRUE(opened(r, "corr_outage"));
}

TEST(DiagnosisEngine, AnyCorrectionBytesCountAsLinkAlive) {
  auto e = make_engine();
  e.on_solution(100.0, fixed());
  const std::vector<uint8_t> garbage = {0x01, 0x02, 0x03};
  EXPECT_TRUE(e.on_corrections(100.0, garbage.data(), garbage.size()).empty());
  EXPECT_FALSE(has_code(e.tick(102.0), "corr_outage"));
}

TEST(DiagnosisEngine, StaleSolutionDegradesToNoSolution) {   // 移植 test_stale_solution_degrades_to_no_solution
  auto e = make_engine();
  corrections(e, 100.0);
  e.on_solution(100.0, fixed());
  EXPECT_FALSE(has_code(e.tick(102.0), "no_solution"));
  corrections(e, 105.5);
  const auto r = e.tick(106.0);   // 独立解已 6 s 没更新
  EXPECT_TRUE(has_code(r, "no_solution"));
  EXPECT_TRUE(opened(r, "no_solution"));
  EXPECT_FALSE(has_code(r, "corr_outage"));
}

TEST(DiagnosisEngine, StaleDeviceFixDoesNotFeedTheCorrAgeFallback) {   // 移植 test_stale_can_epoch_gated_from_fallbacks
  auto e = make_engine();
  SolutionSample dev = fixed();
  dev.age = 99.0;
  e.on_device_solution(40.0, dev);
  corrections(e, 100.0);
  EXPECT_FALSE(has_code(e.tick(100.5), "corr_outage")) << "60 s 前的 610 龄期不能拿来判差分中断";
  e.on_device_solution(100.4, dev);
  EXPECT_TRUE(has_code(e.tick(100.6), "corr_outage")) << "新鲜的 610 龄期作为回退";
}

TEST(DiagnosisEngine, DivergenceNeedsHoldAndItsClockResetsWhenPairingIsLost) {   // 移植 test_div_since_cleared_when_inputs_vanish
  auto e = make_engine();
  const double far_lat = 44.5 + 0.5 / 111000.0;   // 610 解偏北约 0.5 m
  TickResult last;
  for (int i = 0; i <= 6; ++i) {
    const double t = 100.0 + i;
    corrections(e, t);
    e.on_solution(t, fixed());
    e.on_device_solution(t, fixed(far_lat));
    last = e.tick(t + 0.1);
    if (i == 4) EXPECT_FALSE(has_code(last, "device_divergence")) << "只持续了 4 s";
  }
  EXPECT_TRUE(has_code(last, "device_divergence"));

  for (int i = 7; i <= 12; ++i) {   // 独立解断流:两路到达时刻差 >= 2 s 后不再配对
    const double t = 100.0 + i;
    corrections(e, t);
    e.on_device_solution(t, fixed(far_lat));
    last = e.tick(t + 0.1);
  }
  EXPECT_FALSE(last.divergence.since.has_value());

  corrections(e, 113.0);
  e.on_solution(113.0, fixed());
  e.on_device_solution(113.0, fixed(far_lat));
  const auto r = e.tick(113.1);
  EXPECT_FALSE(has_code(r, "device_divergence")) << "恢复配对后必须重新累计持续时间";
  ASSERT_TRUE(r.divergence.since.has_value());
  EXPECT_DOUBLE_EQ(*r.divergence.since, 113.1);
}

TEST(DiagnosisEngine, BaseStationShiftFromRtcm1005) {
  DiagnosisConfig cfg;
  cfg.base_warmup_s = 10.0;
  auto e = make_engine(cfg);
  const auto base = make_1005_frame(7, X, Y, Z);
  auto u = e.on_corrections(0.0, base.data(), base.size());
  ASSERT_EQ(u.size(), 1u);
  EXPECT_EQ(u[0].coords.station_id, 7);
  EXPECT_TRUE(u[0].feed.history_changed);
  EXPECT_FALSE(u[0].feed.offset_m.has_value());

  u = e.on_corrections(10.0, base.data(), base.size());
  ASSERT_EQ(u.size(), 1u);
  EXPECT_TRUE(u[0].feed.baseline_learned);
  ASSERT_TRUE(e.baseline().has_value());

  const auto moved = make_1005_frame(7, X + 0.8, Y, Z);
  u = e.on_corrections(11.0, moved.data(), moved.size());
  ASSERT_EQ(u.size(), 1u);
  EXPECT_TRUE(u[0].feed.history_changed);
  ASSERT_TRUE(u[0].feed.offset_m.has_value());
  EXPECT_NEAR(*u[0].feed.offset_m, 0.8, 1e-3);

  e.on_solution(11.0, fixed());
  const auto r = e.tick(11.5);
  EXPECT_EQ(r.result.status().code, "base_shift");
  EXPECT_TRUE(opened(r, "base_shift"));
}

TEST(DiagnosisEngine, PersistedBaselineIsUsedImmediately) {
  auto e = DiagnosisEngine(DiagnosisConfig{}, {}, true, Ecef{X, Y, Z}, Ecef{X, Y, Z});
  const auto moved = make_1005_frame(7, X, Y + 0.3, Z);
  const auto u = e.on_corrections(0.0, moved.data(), moved.size());
  ASSERT_EQ(u.size(), 1u);
  ASSERT_TRUE(u[0].feed.offset_m.has_value());
  EXPECT_NEAR(*u[0].feed.offset_m, 0.3, 1e-3);
}

TEST(DiagnosisEngine, MultipathFromStatLinesExpiresWhenTheStreamStops) {
  auto e = make_engine();
  corrections(e, 100.0);
  e.on_solution(100.0, fixed());
  e.on_stat_line(100.0, sat_line("C08", 1000.0, 1, 15.0, 3.5, 30.0, 1, 0, 0));
  e.on_stat_line(100.2, sat_line("G17", 1000.0, 1, 12.0, -2.8, 33.0, 1, 0, 0));
  EXPECT_TRUE(has_code(e.tick(100.5), "multipath"));

  corrections(e, 106.0);
  e.on_solution(106.0, fixed());
  EXPECT_FALSE(has_code(e.tick(106.0), "multipath")) << "$SAT 流 5.8 s 没更新,多路径结论不能一直挂着";
}

TEST(DiagnosisEngine, CycleSlipsCountedOnArrivalTime) {
  auto e = make_engine();
  corrections(e, 100.0);
  e.on_solution(100.0, fixed());
  e.on_stat_line(100.0, sat_line("G05", 1000.0, 1, 60.0, 0.1, 45.0, 1, 0, 0));
  e.on_stat_line(101.0, sat_line("G05", 1001.0, 1, 60.0, 0.1, 45.0, 1, 9, 0));
  EXPECT_TRUE(has_code(e.tick(101.5), "cycle_slip"));
  corrections(e, 140.0);
  e.on_solution(140.0, fixed());
  EXPECT_FALSE(has_code(e.tick(140.0), "cycle_slip")) << "30 s 窗口过后不再计数";
}

TEST(DiagnosisEngine, SatsMinMetricOnlyWhenASolutionExists) {
  DiagnosisConfig cfg;
  cfg.close_hysteresis_s = 0.0;
  auto e = make_engine(cfg);
  corrections(e, 100.0);
  e.tick(104.0);                     // 无解 + 差分中断:开 corr_outage 与 no_solution
  corrections(e, 105.0);
  e.on_solution(105.0, fixed());
  e.tick(105.1);                     // 两者恢复计时开始
  corrections(e, 105.2);
  const auto r = e.tick(105.2);
  const auto it = std::find_if(r.transitions.begin(), r.transitions.end(),
                               [](const EventTransition& t) { return t.code == "corr_outage"; });
  ASSERT_NE(it, r.transitions.end());
  EXPECT_EQ(it->kind, EventKind::Close);
  EXPECT_EQ(it->peak.count("sats_min"), 0u) << "无解时不报 sats_min,不能把峰值拉成 0";
  EXPECT_GT(it->peak.at("corr_gap_s"), 3.0);
}

TEST(DiagnosisEngine, ShutdownClosesEverythingOpen) {
  auto e = make_engine();
  corrections(e, 100.0);
  e.tick(104.0);
  const auto closed = e.shutdown(105.0);
  ASSERT_FALSE(closed.empty());
  for (const auto& t : closed) {
    EXPECT_EQ(t.kind, EventKind::Close);
    EXPECT_EQ(t.reason, CloseReason::Shutdown);
  }
}
```

- [ ] **Step 3: 确认 RED**

Run: 构建 `gnss_core`。Expected: 编译失败（`diagnosis_engine.hpp` 不存在）；`test_rtkstat` 在夹具改动后单独构建运行必须仍全部通过（夹具重构不改变其行为）。记录。

- [ ] **Step 4: 实现**

`gnss_core/include/gnss_core/diagnosis_engine.hpp`：

```cpp
#pragma once
// 诊断引擎:把规则链、偏差监测、事件机、基站监测串起来(spec §8)。
// 输入是带到达时刻(UTC unix 秒)的原始数据,每秒 tick() 一次;落盘、定时、参数加载由壳(轮 3b 的
// gnss_diag_node)负责。语义对应 rtk-monitor main.py 的 _diagnosis_tick 与各 _on_* 回调。
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "gnss_core/base_station_monitor.hpp"
#include "gnss_core/diagnosis.hpp"
#include "gnss_core/divergence_monitor.hpp"
#include "gnss_core/event_book.hpp"
#include "gnss_core/rtcm.hpp"
#include "gnss_core/rtkstat.hpp"

namespace gnss_core {

struct BaseUpdate {
  double t = 0.0;
  BaseStationCoords coords;
  BaseFeedResult feed;   // history_changed → 写 base.pos;baseline_learned → 持久化基线
};

struct TickResult {
  DiagnosisResult result;
  std::vector<EventTransition> transitions;
  DivergenceState divergence;
};

class DiagnosisEngine {
public:
  // cfg 非法时抛 std::invalid_argument
  DiagnosisEngine(DiagnosisConfig cfg, std::vector<ControlPoint> control_points, bool solver_enabled,
                  std::optional<Ecef> persisted_baseline, std::optional<Ecef> last_history);

  std::vector<BaseUpdate> on_corrections(double t, const uint8_t* data, size_t len);
  void on_solution(double t, const SolutionSample& s);          // rtkrcv 独立解
  void on_device_solution(double t, const SolutionSample& s);   // 610 融合解
  void on_stat_line(double t, const std::string& line);         // rtkrcv $SAT 行

  TickResult tick(double now);
  std::vector<EventTransition> shutdown(double now);
  std::optional<Ecef> baseline() const { return base_.baseline(); }

private:
  bool fresh(const std::optional<double>& t, double now) const;

  DiagnosisConfig cfg_;
  std::vector<ControlPoint> control_points_;
  bool solver_enabled_;

  RtcmFramer framer_;
  BaseStationMonitor base_;
  DivergenceMonitor divergence_;
  EventBook events_;
  StatEpochAccumulator stat_epoch_;
  SlipWindow slips_;

  std::optional<double> corr_last_t_;
  std::optional<double> base_offset_m_;
  std::optional<SolutionSample> sol_;
  std::optional<double> sol_t_;
  std::optional<SolutionSample> dev_;
  std::optional<double> dev_t_;
  std::optional<double> stat_t_;
};

}  // namespace gnss_core
```

`gnss_core/src/diagnosis_engine.cpp`：

```cpp
#include "gnss_core/diagnosis_engine.hpp"

#include <cmath>
#include <map>
#include <utility>

#include "gnss_core/geodetic.hpp"

namespace gnss_core {

namespace {
const DiagnosisConfig& validated(const DiagnosisConfig& cfg) {
  validate_diagnosis_config(cfg);
  return cfg;
}
}  // namespace

DiagnosisEngine::DiagnosisEngine(DiagnosisConfig cfg, std::vector<ControlPoint> control_points,
                                 bool solver_enabled, std::optional<Ecef> persisted_baseline,
                                 std::optional<Ecef> last_history)
    : cfg_(validated(cfg)),
      control_points_(std::move(control_points)),
      solver_enabled_(solver_enabled),
      base_(cfg_.base_warmup_s, persisted_baseline, last_history),
      divergence_(cfg_),
      events_(cfg_.close_hysteresis_s) {}

bool DiagnosisEngine::fresh(const std::optional<double>& t, double now) const {
  return t && now - *t < cfg_.sol_stale_s;
}

std::vector<BaseUpdate> DiagnosisEngine::on_corrections(double t, const uint8_t* data, size_t len) {
  std::vector<BaseUpdate> out;
  if (len == 0) return out;
  corr_last_t_ = t;   // 任何差分字节都算链路存活,不只是 1005/1006
  for (const auto& msg : framer_.feed(data, len)) {
    BaseStationCoords coords;
    if (!parse_base_station(msg, coords)) continue;
    BaseUpdate u;
    u.t = t;
    u.coords = coords;
    u.feed = base_.feed(t, Ecef{coords.x, coords.y, coords.z});
    if (u.feed.offset_m) base_offset_m_ = u.feed.offset_m;   // 位移保持到下一条 1005/1006
    out.push_back(u);
  }
  return out;
}

void DiagnosisEngine::on_solution(double t, const SolutionSample& s) {
  sol_ = s;
  sol_t_ = t;
}

void DiagnosisEngine::on_device_solution(double t, const SolutionSample& s) {
  dev_ = s;
  dev_t_ = t;
}

void DiagnosisEngine::on_stat_line(double t, const std::string& line) {
  SatStat s;
  if (!parse_sat_line(line, s)) return;
  stat_epoch_.feed(line);
  slips_.feed(t, s.sat, s.slipc);   // 周跳窗口按到达时刻计,不用 $SAT 的 tow
  stat_t_ = t;
}

TickResult DiagnosisEngine::tick(double now) {
  const std::optional<SolutionSample> sol = fresh(sol_t_, now) ? sol_ : std::nullopt;
  const std::optional<SolutionSample> dev = fresh(dev_t_, now) ? dev_ : std::nullopt;

  DiagnosisInput in;
  in.now = now;
  in.corr_last_t = corr_last_t_;
  if (sol) {
    in.corr_age = sol->age;
  } else if (dev) {
    in.corr_age = dev->age;
  }
  in.base_offset_m = base_offset_m_;
  in.sol = sol;
  if (fresh(stat_t_, now)) in.sats = stat_epoch_.epoch();
  in.slip_count_30s = slips_.count(now);

  std::optional<double> d;
  if (sol && dev && std::abs(*sol_t_ - *dev_t_) < cfg_.divergence_pair_max_dt_s) {
    d = geodesic_distance_m(sol->lat, sol->lon, dev->lat, dev->lon);
  }
  TickResult out;
  out.divergence = divergence_.update(now, d, sol ? std::hypot(sol->sdn, sol->sde) : 0.0);
  in.divergence_m = out.divergence.divergence_m;
  in.divergence_since = out.divergence.since;
  in.divergence_threshold_m = out.divergence.threshold_m;
  in.solver_enabled = solver_enabled_;
  in.control_points = control_points_;

  out.result = evaluate_rules(in, cfg_);

  std::optional<LatLon> pos;
  if (sol) {
    pos = LatLon{sol->lat, sol->lon};
  } else if (dev) {
    pos = LatLon{dev->lat, dev->lon};
  }
  std::map<std::string, double> metrics;
  metrics["divergence_m"] = d.value_or(0.0);
  if (sol) metrics["sats_min"] = static_cast<double>(sol->ns);   // 无解时不报,免得峰值被拉成 0
  metrics["corr_gap_s"] = corr_last_t_ ? now - *corr_last_t_ : 0.0;
  out.transitions = events_.update(now, out.result.verdicts, pos, metrics);
  return out;
}

std::vector<EventTransition> DiagnosisEngine::shutdown(double now) { return events_.close_all(now); }

}  // namespace gnss_core
```

注意 `cfg_` 必须在 `base_`、`divergence_`、`events_` 之前声明（上面的头文件已按此顺序），它们在初始化列表里读 `cfg_`。

- [ ] **Step 5: GREEN + 变异**

Run: 构建后 `./build/gnss_core/test_diagnosis_engine && ./build/gnss_core/test_rtkstat`。Expected: 全部 PASS。

变异（做完恢复）：
- `tick` 里 `dev` 不做新鲜度过滤（直接用 `dev_`）→ `StaleDeviceFixDoesNotFeedTheCorrAgeFallback` 必须 FAIL
- 去掉 `|sol_t - dev_t| < divergence_pair_max_dt_s` 条件 → `DivergenceNeedsHoldAndItsClockResetsWhenPairingIsLost` 必须 FAIL
- `in.sats` 不做 `stat_t_` 新鲜度过滤 → `MultipathFromStatLinesExpiresWhenTheStreamStops` 必须 FAIL
- 无解时也写 `metrics["sats_min"] = 0.0` → `SatsMinMetricOnlyWhenASolutionExists` 必须 FAIL
- `on_corrections` 只在解出 1005/1006 时才更新 `corr_last_t_` → `AnyCorrectionBytesCountAsLinkAlive` 必须 FAIL

- [ ] **Step 6: 全量测试并提交**

Run: 全量命令。Expected: 0 failures。确认 `$TMPDIR`（或 `/tmp`）下没有本计划测试留下的 `base_monitor_*`、`diag_io_*`、`retention_*` 目录。

```bash
cd /home/steve/glim_ws/src/glim_ext
git add gnss_core/include/gnss_core/diagnosis_engine.hpp gnss_core/src/diagnosis_engine.cpp gnss_core/test/diag_test_fixtures.hpp gnss_core/test/test_diagnosis_engine.cpp gnss_core/test/test_rtkstat.cpp gnss_core/CMakeLists.txt
git commit -m "feat(gnss_core): diagnosis engine wiring rules, divergence, events and base monitoring

<RED 与五个变异结果>

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

## 轮 3b 预告（不在本计划内）

3a 合并后再写 3b 计划，范围：

- **`gnss_diag_node`**（`gnss_bringup`）：订阅 `/gnss/rtcm_corrections`、`/rtkrcv_node/rtk_fix`、610 的 `RtkFix`、`/rtkrcv_node/stat`；1 Hz 调 `DiagnosisEngine::tick`；按 UTC 日轮转写 `<root>/<YYYYMMDD>/events.log` 与 `base.pos`；持久化基线文件；发布诊断状态与事件话题；停机时写 `shutdown` 关闭行。
- **ratio 的传递**（设计决定 8）：`gnss_msgs/RtkFix` 没有 ratio。选项：给 `RtkFix` 加字段（两个工作区都要重编）、或 `rtkrcv_node` 另发一个带 ratio 的话题——3b 开工前定。
- **清理节点**：每小时对 `.pos` 根目录（`parse_day_dir_date`）与录包根目录（`parse_bag_dir_date`）各调一次 `cleanup_dated_root`。
- **`rtkrcv_node` 健康信号**：距上一条解算的时长、上行字节计数（遗留清单里的轮 3 首项），用来区分"在隧道里"和"配置错误"。
- launch / yaml / README 接线。
