# 轮 4a：GNSS 报告离线工具（F2/F3）Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 新增离线命令行工具 `gnss_report`：读取 `<root>/YYYYMMDD/` 下的 `.pos`、`events.log`、`base.pos`，按一个 UTC 自然日或任意时间窗生成一份自包含 HTML 报告（内嵌 SVG 图表），用浏览器"打印为 PDF"。报告覆盖 rtk-monitor 报告的全部内容：固定率（总体与分小时）、事件表、问题路段、基站坐标稳定性、610 与独立解偏差、绝对基准校验。

**Architecture:** 全部放在 ROS 无关的 `gnss_core`，分四层，每层纯函数、单独可测：
1. `diag_io` 补上与现有写出格式互逆的解析函数；
2. `report_inputs` 负责按时间窗装载文件、配对事件；
3. `report_stats` 负责全部统计；
4. `report_svg` / `report_html` 负责渲染。

`report_cli` 负责参数解析与执行，`tools/gnss_report.cpp` 只剩一个 `main`。

**Tech Stack:** C++17、Eigen、GeographicLib（已有依赖，用于测地线距离与局部 ENU 投影）、gtest。不新增任何外部依赖。

**Spec:**
- `docs/gnss/specs/2026-09-03-gnss-glim-modules-design.md` §3 F2/F3（报告内容清单与归宿"离线工具，读 `.pos` + `events.log`"）、§5.3（目录布局）、§10 轮 4
- 参照实现（口径的来源）：`/home/steve/Documents/GitHub/gnss-alg/rtk-monitor/src/rtk_monitor/report.py`、`api.py` 的 `/report`，测试 `tests/test_report.py`；rtk-monitor 设计文档 `/home/steve/Documents/GitHub/gnss-alg/rtk-monitor/docs/superpowers/specs/2026-08-31-rtk-monitor-design.md` §6
- 输入格式：`glim_ext/gnss_core/include/gnss_core/{pos_io,diag_io,retention}.hpp`

## Global Constraints

- **代码仓库**：`/home/steve/glim_ws/src/glim_ext`（git）。
  - 开工前 `git ls-remote origin master` 确认远端 `master`（写作时为 `dbd2986`）。
  - 确认工作区干净，且没有其他会话正在改这个仓库（`git status --short` 为空、`git worktree list` 只有主工作区）。
  - 从最新 `master` 开 `feat/round4a-report-tool`。以 `git ls-remote` 为准（仓库有外部 git 自动同步）。
- **绝不 `git push`**。
- 每个 commit message 以 `Co-Authored-By:` 行结尾，写实际执行提交的模型。
- `glim_underground` 里维护者未提交的工作（`include/glim/util/time_keeper.hpp`、`src/glim/util/time_keeper.cpp`）绝不触碰、绝不暂存。
- `gnss_core` 保持不依赖 ROS / GLIM；不新增外部库（不引入 yaml、PDF、图表、模板库）。
- **报告 HTML 必须自包含**：内联 CSS 与 SVG，不含 `<script>`、`<link>`、`http://`、`https://`。现场离线用任何浏览器打开都完整显示。
- 报告里所有来自文件或命令行的文本（源名、事件代码与结论、控制点名、根目录）一律经 `html_escape`。
- 报告界面文字用中文；时间一律 UTC，格式与 `format_utc_timestamp` 一致（`YYYY/MM/DD HH:MM:SS.mmm`）。
- 注释中文、与周边一致；标识符英文。
- 测试一律 gtest，经 `gnss_core/CMakeLists.txt` 的 `gnss_core_add_test` 注册；禁止 pytest / launch_testing。
- **诚实测试**：每个新测试在实现前必须亲眼看到失败——对旧代码失败，或对桩实现失败，或对写明的变异失败（CMake"找不到源文件"不算行为 RED）；证据写进 commit message。
- 测试需要文件时一律用 `mkdtemp` 临时目录（基于 `$TMPDIR`，缺省 `/tmp`），用 RAII 在析构里删除，失败路径也要删。
- 构建：`cd /home/steve/glim_ws && source /opt/ros/humble/setup.bash && colcon build --symlink-install --packages-up-to gnss_bringup`
- 单测：`cd /home/steve/glim_ws && source /opt/ros/humble/setup.bash && source install/setup.bash && ./build/gnss_core/<test> --gtest_filter='<Suite>.<Name>'`
- 全量：`cd /home/steve/glim_ws && source /opt/ros/humble/setup.bash && colcon test --packages-select gnss_core gnss_bringup && colcon test-result --all`
  - 开工前先跑一次并记下基线（写作时为 530 tests、0 failures，但 `dbd2986` 可能改变数量）。每个 Task 结束 0 failures。
  - `test_rtkrcv_node_process` 的 `WrittenConfTakesTheBasePositionFromRtcm` 已知会在整套测试刚跑完时偶发失败，若出现，单独重跑确认后照实记录，不在本计划内修。

## 设计决定

维护者 2026-09-16 确认：

1. **C++ 实现，放 `gnss_core`**。复用 `read_pos`（自动识别 GPST/UTC）与 `events.log`/`base.pos` 的格式代码，避免第二份解析器漂移。
2. **输出单个 HTML，浏览器打印为 PDF**（与 rtk-monitor 一致）。HTML 带 `@page A4` 与 `@media print` 样式。
3. **内嵌 SVG 图表**，共四张：
   - 轨迹图：按解状态着色，标出事件位置，即"问题路段"；无底图。
   - 分小时固定率柱状图。
   - 基站偏移曲线。
   - 绝对基准偏差曲线。
4. **每个存在的数据源各算一份**：`can` / `gpchc` / `rtkrcv` / `ref` 及目录里其他 `*.pos`（`base.pos` 除外）并列展示。

控制者裁定（本计划内，代价可控）：

5. **固定解统一按 RTKLIB `Q == 1` 判定**。本项目的 `.pos` 已归一到 RTKLIB Q（`pos_writer` 把 `QUALITY_FIXED` 写成 1）。rtk-monitor 里 `can` 用 `q == 4` 是它自己 SQLite 存储的约定，不适用。
   - 分母是时间窗内该源全部记录（含非固定），与 rtk-monitor 一致。
   - 分小时桶按 UTC 整点切，时间窗内没有记录的小时显示"-"。
6. **610 与独立解偏差**：`can`、`gpchc` 各自与 `rtkrcv` 比较。
   - 对每个 `rtkrcv` 历元，取时间最近的设备历元；`|Δt| <= pair_tol_s`（默认 0.5 s，沿用 rtk-monitor `_DEV_PAIR_TOL_S`）才配对。
   - 算测地线水平距离，输出配对数、最大、均值。
   - 缺 `rtkrcv` 或两个 610 源都缺时，这一节显示"无法比较"。
7. **绝对基准校验**：对每个源的固定解，找最近的控制点，距离 `<= abs_ref_radius_m`（默认 3.0 m）才计入偏差序列。
   - 最大偏差 `> abs_ref_max_m`（默认 0.2 m）时标"超阈值——全矿整体平移嫌疑"。
   - 只取最近的一个控制点，与诊断规则 C3（`AbsRefJudgesOnlyTheNearestControlPoint`）一致；rtk-monitor 是对所有半径内控制点各算一次。
   - 控制点用命令行 `--control-point NAME,LAT,LON`（可重复）给出，不读 ROS yaml（`gnss_core` 不引入 yaml 依赖）。
8. **基站坐标稳定性**：以时间窗内第一条 `base.pos` 记录为基准，输出 ECEF 欧氏偏移序列与最大值（与 rtk-monitor 一致）。最大值 `> base_shift_m`（默认 0.1 m，同诊断 `base_shift_m`）时标警告。
9. **事件配对**：
   - 按日期目录顺序逐行读 `events.log`：OPEN 行记为待关闭；CLOSE 行按 `code` 与 `opened=` 时刻（误差 < 2 ms）配对。
   - 配不上的 CLOSE 行单独成一条事件，位置取 CLOSE 行。
   - 同一 `code` 又出现 OPEN 时，前一条视为未关闭（进程中断后重启）。
   - 扫描范围从 `t0` 所在日的前一天开始，用来找窗口开始前开启、窗口内关闭的事件的 OPEN 行。更早开启的事件没有 OPEN 行时，位置取 CLOSE 行。
   - 与时间窗有交集（`t_open < t1` 且（未关闭或 `t_close >= t0`））的事件入报告。
   - 问题路段 = 事件开启位置：在轨迹图上标事件编号，事件表同时列经纬度。
10. **时间窗**：`--day YYYYMMDD`（UTC 自然日）或 `--from/--to "YYYY/MM/DD HH:MM:SS"`（UTC），半开区间 `[t0, t1)`，最长 31 天。`.pos` 只从 `[t0 所在日, t1 所在日]` 的目录读。
11. **轨迹图**：
    - 所有源共用一个局部 ENU 原点（按源顺序第一个非空源的首条记录；都没有时取第一个带位置的事件）。
    - 相邻记录间隔 `> track_gap_s`（5 s）时断开折线。
    - 每源最多保留 `max_track_points`（4000）个点：按步长抽稀，每段首尾点保留。
    - 同一段内按解状态分成若干等色子段；源之间用线宽与虚线样式区分。
12. **`parse_base_history_line` 挪到 `gnss_core`**。`gnss_bringup/diag_node_support.hpp` 里的同名函数改为调用它，行为不变（多出一条：时间戳必须能解析）。

## File Structure

`glim_ext/gnss_core`
- Modify `include/gnss_core/pos_io.hpp`、`src/pos_io.cpp` — 公开 `parse_utc_date_time`（原匿名命名空间里的 `parse_date_time`）（Task 1）
- Modify `include/gnss_core/diag_io.hpp`、`src/diag_io.cpp` — `EventLogLine`、`parse_event_line`、`parse_base_history_line`（Task 1）
- Modify `test/test_diag_io.cpp`（Task 1）
- Create `include/gnss_core/report_inputs.hpp`、`src/report_inputs.cpp`、`test/test_report_inputs.cpp` — 按时间窗装载与事件配对（Task 2）
- Create `include/gnss_core/report_stats.hpp`、`src/report_stats.cpp`、`test/test_report_stats.cpp` — 统计（Task 3）
- Create `include/gnss_core/report_svg.hpp`、`src/report_svg.cpp`、`test/test_report_svg.cpp` — SVG 图表与 `html_escape`（Task 4）
- Create `include/gnss_core/report_html.hpp`、`src/report_html.cpp`、`test/test_report_html.cpp` — HTML 渲染（Task 5）
- Create `include/gnss_core/report_cli.hpp`、`src/report_cli.cpp`、`test/test_report_cli.cpp`、`tools/gnss_report.cpp` — 命令行（Task 6）
- Modify `CMakeLists.txt`（每个 Task 加自己的源文件与测试）、`tools/README.md`（Task 6）

`glim_ext/gnss_bringup`
- Modify `include/gnss_bringup/diag_node_support.hpp` — `parse_base_history_line` 委托给 `gnss_core`（Task 1）

依赖顺序：1 → 2 → 3 → 4 → 5 → 6。

---
### Task 1: 读回 `events.log` 与 `base.pos` 的解析函数

**Files:**
- Modify: `glim_ext/gnss_core/include/gnss_core/pos_io.hpp`、`src/pos_io.cpp`
- Modify: `glim_ext/gnss_core/include/gnss_core/diag_io.hpp`、`src/diag_io.cpp`
- Test: `glim_ext/gnss_core/test/test_diag_io.cpp`
- Modify: `glim_ext/gnss_bringup/include/gnss_bringup/diag_node_support.hpp`

**Interfaces:**
- Produces（`namespace gnss_core`）:
  - `bool parse_utc_date_time(const std::string& date, const std::string& time, double& out)`（pos_io.hpp）
  - `struct EventLogLine { EventKind kind; double t; std::string level; std::string code; std::optional<LatLon> pos; std::string message; double t_open; double duration_s; std::string reason; std::map<std::string, double> peak; }`
  - `std::optional<EventLogLine> parse_event_line(const std::string& line)`
  - `std::optional<std::pair<double, Ecef>> parse_base_history_line(const std::string& line)`

- [ ] **Step 1: 写失败测试**

`test/test_diag_io.cpp`：include 区补 `#include <optional>`；在 `TEST(LineAppender, WritesHeaderOnceAndAppendsAcrossReopen)` 之前插入：

```cpp
namespace {
EventTransition close_event_with_peak() {
  EventTransition e = open_event();
  e.kind = EventKind::Close;
  e.t = 1789372810.0;
  e.pos = LatLon{44.501, 90.281};
  e.reason = CloseReason::Recovered;
  e.peak = {{"corr_gap_s", 12.345}, {"sats_min", 4.0}};
  return e;
}
}  // namespace

// 轮 4a:报告工具要把 events.log 读回来,解析必须与 format_event_line 互逆
TEST(EventLineParsing, OpenLineRoundTrips) {
  const auto p = parse_event_line(format_event_line(open_event()));
  ASSERT_TRUE(p.has_value());
  EXPECT_EQ(p->kind, EventKind::Open);
  EXPECT_NEAR(p->t, 1789372800.5, 1e-6);
  EXPECT_NEAR(p->t_open, 1789372800.5, 1e-6) << "OPEN 行的开启时刻就是行时刻";
  EXPECT_EQ(p->level, "serious");
  EXPECT_EQ(p->code, "corr_outage");
  ASSERT_TRUE(p->pos.has_value());
  EXPECT_NEAR(p->pos->lat, 44.5, 1e-9);
  EXPECT_NEAR(p->pos->lon, 90.28, 1e-9);
  EXPECT_EQ(p->message, "差分中断 10s——5G 链路或平台转发问题");
}

TEST(EventLineParsing, CloseLineRoundTripsWithOpenedDurationReasonAndPeak) {
  const auto p = parse_event_line(format_event_line(close_event_with_peak()));
  ASSERT_TRUE(p.has_value());
  EXPECT_EQ(p->kind, EventKind::Close);
  EXPECT_NEAR(p->t, 1789372810.0, 1e-6);
  EXPECT_NEAR(p->t_open, 1789372800.5, 1e-6);
  EXPECT_NEAR(p->duration_s, 9.5, 1e-9);
  EXPECT_EQ(p->reason, "recovered");
  ASSERT_EQ(p->peak.size(), 2u);
  EXPECT_NEAR(p->peak.at("corr_gap_s"), 12.345, 1e-9);
  EXPECT_NEAR(p->peak.at("sats_min"), 4.0, 1e-9);
  ASSERT_TRUE(p->pos.has_value());
  EXPECT_NEAR(p->pos->lat, 44.501, 1e-9);
  EXPECT_EQ(p->message, "差分中断 10s——5G 链路或平台转发问题");
}

TEST(EventLineParsing, NoPositionEmptyPeakAndEmptyMessage) {
  EventTransition e = close_event_with_peak();
  e.pos.reset();
  e.peak.clear();
  e.message.clear();
  e.reason = CloseReason::Shutdown;
  const auto p = parse_event_line(format_event_line(e));
  ASSERT_TRUE(p.has_value()) << format_event_line(e);
  EXPECT_FALSE(p->pos.has_value());
  EXPECT_TRUE(p->peak.empty());
  EXPECT_EQ(p->message, "");
  EXPECT_EQ(p->reason, "shutdown");
}

TEST(EventLineParsing, RejectsCommentsHalfLinesAndUnknownWords) {
  std::istringstream header(events_log_header());
  for (std::string line; std::getline(header, line);) {
    EXPECT_FALSE(parse_event_line(line).has_value()) << line;
  }
  EXPECT_FALSE(parse_event_line("").has_value());
  const std::string close = format_event_line(close_event_with_peak());
  EXPECT_FALSE(parse_event_line(close.substr(0, 60)).has_value()) << "掉电留下的半行";
  std::string bad_kind = format_event_line(open_event());
  bad_kind.replace(bad_kind.find(" OPEN "), 6, " MAYBE ");
  EXPECT_FALSE(parse_event_line(bad_kind).has_value());
  std::string bad_level = format_event_line(open_event());
  bad_level.replace(bad_level.find(" serious "), 9, " fatal ");
  EXPECT_FALSE(parse_event_line(bad_level).has_value());
  std::string bad_reason = close;
  bad_reason.replace(bad_reason.find("reason=recovered"), 16, "reason=whatever");
  EXPECT_FALSE(parse_event_line(bad_reason).has_value());
}

TEST(BaseHistoryParsing, RoundTripsAndRejectsBrokenLines) {
  const Ecef p{-2148744.1, 4426641.2, 4044655.9};
  const auto r = parse_base_history_line(format_base_history_line(1789372800.25, p));
  ASSERT_TRUE(r.has_value());
  EXPECT_NEAR(r->first, 1789372800.25, 1e-6);
  EXPECT_NEAR(r->second.x, p.x, 1e-4);
  EXPECT_NEAR(r->second.y, p.y, 1e-4);
  EXPECT_NEAR(r->second.z, p.z, 1e-4);
  std::istringstream header(base_pos_header());
  for (std::string line; std::getline(header, line);) {
    EXPECT_FALSE(parse_base_history_line(line).has_value()) << line;
  }
  EXPECT_FALSE(parse_base_history_line("2026/09/14 08:00:00.500  -2148744.1").has_value()) << "半行";
  EXPECT_FALSE(parse_base_history_line("2026/09/14 08:00:00.500  1 2 3 4").has_value()) << "多余字段视为损坏";
  EXPECT_FALSE(parse_base_history_line("not-a-date 08:00:00.500  1 2 3").has_value());
  EXPECT_FALSE(parse_base_history_line("2026/09/14 08:00:00.500  nan 2 3").has_value());
}
```

- [ ] **Step 2: 确认 RED（对桩）**

先做能编译的桩：
- `pos_io.hpp` 声明 `parse_utc_date_time`；`pos_io.cpp` 暂时加一个返回 `false` 的定义（原匿名命名空间里的 `parse_date_time` 先不动）。
- `diag_io.hpp` 按 Interfaces 加上 `EventLogLine` 与两个函数声明；`diag_io.cpp` 两个函数返回 `std::nullopt`。

Run: `cd /home/steve/glim_ws && source /opt/ros/humble/setup.bash && colcon build --symlink-install --packages-select gnss_core && ./build/gnss_core/test_diag_io`
Expected: `OpenLineRoundTrips`、`CloseLineRoundTripsWithOpenedDurationReasonAndPeak`、`NoPositionEmptyPeakAndEmptyMessage`、`BaseHistoryParsing.RoundTripsAndRejectsBrokenLines` FAIL；`RejectsCommentsHalfLinesAndUnknownWords` 对桩 PASS（Step 4 用变异验证）。

- [ ] **Step 3: 实现**

`pos_io.hpp`，`read_pos` 声明之前加：

```cpp
// "YYYY/MM/DD" + "HH:MM:SS.sss" 按 UTC 日历合成 unix 秒(timegm,不受本机 TZ 影响);
// 字段读不出来返回 false。.pos、events.log、base.pos 的时间列共用。
bool parse_utc_date_time(const std::string& date, const std::string& time, double& out);
```

`pos_io.cpp`：把匿名命名空间里的 `parse_date_time` 整个函数（含上方两行注释）移到匿名命名空间之外（`namespace gnss_core` 内），改名为 `parse_utc_date_time`，函数体不变；把 `parse_llh_solution` 里的 `parse_date_time(date, time, stamp)` 改为 `parse_utc_date_time(date, time, stamp)`。删掉桩定义。

`diag_io.hpp`：include 区改为

```cpp
#include <fstream>
#include <map>
#include <optional>
#include <string>
#include <utility>

#include "gnss_core/base_station_monitor.hpp"
#include "gnss_core/event_book.hpp"
```

文件头注释第一行末尾补 `;读回函数供轮 4a 报告工具使用`。`format_base_history_line` 声明之后加：

```cpp
// ---- 读回(与上面的写出格式互逆) ----
// events.log 的一行。注释行(%)、空行、字段不全(掉电留下的半行)、类型/级别/关闭原因不认识时解析失败。
struct EventLogLine {
  EventKind kind = EventKind::Open;
  double t = 0.0;                        // 行时刻(UTC unix 秒)
  std::string level;                     // ok/info/warning/serious/critical
  std::string code;
  std::optional<LatLon> pos;
  std::string message;
  double t_open = 0.0;                   // OPEN 行等于 t;CLOSE 行取 opened=
  double duration_s = 0.0;               // 仅 CLOSE
  std::string reason;                    // 仅 CLOSE:recovered / shutdown
  std::map<std::string, double> peak;    // 仅 CLOSE
};
std::optional<EventLogLine> parse_event_line(const std::string& line);

// base.pos 的一行 → (UTC unix 秒, ECEF);注释行、半行、多余字段、非有限坐标返回空
std::optional<std::pair<double, Ecef>> parse_base_history_line(const std::string& line);
```

`diag_io.cpp`：include 区补 `<algorithm>`、`<cerrno>`、`<cstdlib>`、`<cstring>`，并 `#include "gnss_core/pos_io.hpp"`（`parse_utc_date_time`）。在已有匿名命名空间里（`single_line` 之后）加：

```cpp
// 取下一个以空格/制表符分隔的词,pos 前进到词尾
bool next_token(const std::string& s, size_t& pos, std::string& tok) {
  while (pos < s.size() && (s[pos] == ' ' || s[pos] == '\t')) ++pos;
  if (pos >= s.size()) return false;
  const size_t start = pos;
  while (pos < s.size() && s[pos] != ' ' && s[pos] != '\t') ++pos;
  tok = s.substr(start, pos - start);
  return true;
}

bool strip_prefix(const std::string& tok, const char* prefix, std::string& rest) {
  const size_t n = std::strlen(prefix);
  if (tok.compare(0, n, prefix) != 0) return false;
  rest = tok.substr(n);
  return true;
}

// 整个字符串必须是一个有限浮点数
bool parse_finite(const std::string& s, double& out) {
  if (s.empty()) return false;
  char* end = nullptr;
  errno = 0;
  out = std::strtod(s.c_str(), &end);
  return end == s.c_str() + s.size() && errno == 0 && std::isfinite(out);
}

bool known_level(const std::string& s) {
  return s == "ok" || s == "info" || s == "warning" || s == "serious" || s == "critical";
}

// "k=v;k=v" → map;"-" 表示空
bool parse_peak(const std::string& s, std::map<std::string, double>& out) {
  if (s == "-") return true;
  size_t start = 0;
  while (start <= s.size()) {
    const size_t end = std::min(s.find(';', start), s.size());
    const std::string kv = s.substr(start, end - start);
    const size_t eq = kv.find('=');
    double v = 0.0;
    if (eq == std::string::npos || eq == 0 || !parse_finite(kv.substr(eq + 1), v)) return false;
    out[kv.substr(0, eq)] = v;
    start = end + 1;
  }
  return true;
}
```

文件末尾（`LineAppender::close` 之后、命名空间结束前）加：

```cpp
std::optional<EventLogLine> parse_event_line(const std::string& line) {
  if (line.empty() || line[0] == '%') return std::nullopt;
  EventLogLine e;
  size_t pos = 0;
  std::string date, time, kind, lat_tok, lon_tok, lat_s, lon_s;
  if (!next_token(line, pos, date) || !next_token(line, pos, time) || !parse_utc_date_time(date, time, e.t)) {
    return std::nullopt;
  }
  if (!next_token(line, pos, kind)) return std::nullopt;
  if (kind == "OPEN") {
    e.kind = EventKind::Open;
  } else if (kind == "CLOSE") {
    e.kind = EventKind::Close;
  } else {
    return std::nullopt;
  }
  if (!next_token(line, pos, e.level) || !known_level(e.level)) return std::nullopt;
  if (!next_token(line, pos, e.code)) return std::nullopt;
  if (!next_token(line, pos, lat_tok) || !next_token(line, pos, lon_tok) ||
      !strip_prefix(lat_tok, "lat=", lat_s) || !strip_prefix(lon_tok, "lon=", lon_s)) {
    return std::nullopt;
  }
  if (lat_s != "-" || lon_s != "-") {
    double lat = 0.0, lon = 0.0;
    if (!parse_finite(lat_s, lat) || !parse_finite(lon_s, lon)) return std::nullopt;
    e.pos = LatLon{lat, lon};
  }
  e.t_open = e.t;
  if (e.kind == EventKind::Close) {
    std::string tok, opened_date, opened_time, value;
    if (!next_token(line, pos, tok) || !strip_prefix(tok, "opened=", opened_date)) return std::nullopt;
    if (!next_token(line, pos, opened_time) || !parse_utc_date_time(opened_date, opened_time, e.t_open)) {
      return std::nullopt;
    }
    if (!next_token(line, pos, tok) || !strip_prefix(tok, "duration_s=", value) || !parse_finite(value, e.duration_s)) {
      return std::nullopt;
    }
    if (!next_token(line, pos, tok) || !strip_prefix(tok, "reason=", e.reason) ||
        (e.reason != "recovered" && e.reason != "shutdown")) {
      return std::nullopt;
    }
    if (!next_token(line, pos, tok) || !strip_prefix(tok, "peak=", value) || !parse_peak(value, e.peak)) {
      return std::nullopt;
    }
  }
  // format_event_line 用一个空格把结论接在最后;结论可以为空(行尾只剩这个空格或什么都没有)
  if (pos < line.size() && line[pos] == ' ') ++pos;
  e.message = line.substr(pos);
  return e;
}

std::optional<std::pair<double, Ecef>> parse_base_history_line(const std::string& line) {
  if (line.empty() || line[0] == '%') return std::nullopt;
  size_t pos = 0;
  std::string date, time, xs, ys, zs, extra;
  double t = 0.0;
  Ecef p;
  if (!next_token(line, pos, date) || !next_token(line, pos, time) || !parse_utc_date_time(date, time, t)) {
    return std::nullopt;
  }
  if (!next_token(line, pos, xs) || !next_token(line, pos, ys) || !next_token(line, pos, zs) ||
      !parse_finite(xs, p.x) || !parse_finite(ys, p.y) || !parse_finite(zs, p.z)) {
    return std::nullopt;
  }
  if (next_token(line, pos, extra)) return std::nullopt;   // 多余字段视为损坏
  return std::make_pair(t, p);
}
```

注意：`parse_utc_date_time` 用 `sscanf("%d/%d/%d")`，`"not-a-date"` 读不出三个整数会失败。若实现时发现某个拒绝用例因 `sscanf` 过于宽松而通过，**不要改测试**，在报告里说明并改用更严格的检查（比如先确认日期词形如 `dddd/dd/dd`）。

`gnss_bringup/include/gnss_bringup/diag_node_support.hpp`：`parse_base_history_line` 的函数体改为

```cpp
  if (const auto r = gnss_core::parse_base_history_line(line)) return r->second;
  return std::nullopt;
```

上方注释改为 `// base.pos 数据行 → ECEF;解析规则见 gnss_core::parse_base_history_line(注释行、半行、非有限坐标返回空)。`。如果 `<sstream>` 在该头文件里已经没有别的用处，一并删掉这个 include。

- [ ] **Step 4: GREEN + 变异 + 全量**

Run: `colcon build --symlink-install --packages-up-to gnss_bringup && ./build/gnss_core/test_diag_io && ./build/gnss_core/test_pos_io && ./build/gnss_bringup/test_diag_node_support`，全部 PASS。

变异（逐个做、逐个恢复，记录结果）：
1. 删掉 `parse_event_line` 里 `|| !known_level(e.level)` → `RejectsCommentsHalfLinesAndUnknownWords` 必须 FAIL。
2. 删掉 `parse_base_history_line` 里"多余字段"那一行 → `BaseHistoryParsing.RoundTripsAndRejectsBrokenLines` 必须 FAIL。
3. 把 `parse_event_line` 末尾的 `if (pos < line.size() && line[pos] == ' ') ++pos;` 删掉 → 两个 RoundTrips 用例必须 FAIL（结论会多一个前导空格）。

全量测试 0 failures。

- [ ] **Step 5: 提交**

```bash
cd /home/steve/glim_ws/src/glim_ext
git add gnss_core/include/gnss_core/pos_io.hpp gnss_core/src/pos_io.cpp \
  gnss_core/include/gnss_core/diag_io.hpp gnss_core/src/diag_io.cpp gnss_core/test/test_diag_io.cpp \
  gnss_bringup/include/gnss_bringup/diag_node_support.hpp
git commit -m "feat(gnss_core): parse events.log and base.pos lines back

<RED(桩)与三个变异的结果>

Co-Authored-By: <实际模型>"
```

---
### Task 2: 按时间窗装载 `.pos`、事件与基站坐标史

落实设计决定 4、9、10。

**Files:**
- Create: `glim_ext/gnss_core/include/gnss_core/report_inputs.hpp`
- Create: `glim_ext/gnss_core/src/report_inputs.cpp`
- Create: `glim_ext/gnss_core/test/test_report_inputs.cpp`
- Modify: `glim_ext/gnss_core/CMakeLists.txt`

**Interfaces:**
- Consumes: Task 1 的 `parse_event_line`、`parse_base_history_line`；已有的 `read_pos`、`PosReadOptions`、`parse_day_dir_date`、`utc_yyyymmdd`
- Produces（`namespace gnss_core`）:
  - `struct ReportWindow { double t0; double t1; }`（半开区间，UTC unix 秒）
  - `struct ReportEvent { std::string code, level, message; double t_open; std::optional<double> t_close; std::string close_reason; std::optional<LatLon> pos; std::map<std::string, double> peak; }`
  - `struct BaseSample { double t; Ecef p; }`
  - `struct ReportInputs { std::map<std::string, std::vector<PosRecord>> sources; std::vector<ReportEvent> events; std::vector<BaseSample> base_history; std::vector<std::string> warnings; }`
  - `ReportInputs load_report_inputs(const std::string& root, const ReportWindow& window, const PosReadOptions& pos_options = {})`（root 不是目录时抛 `std::runtime_error`）

- [ ] **Step 1: 写失败测试**

Create `test/test_report_inputs.cpp`：

```cpp
#include <gtest/gtest.h>
#include <sys/stat.h>
#include <unistd.h>

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "gnss_core/diag_io.hpp"
#include "gnss_core/report_inputs.hpp"
using namespace gnss_core;
namespace fs = std::filesystem;

namespace {
const double T = 1789430400.0;   // 2026-09-15 00:00:00 UTC
const ReportWindow kDay{T, T + 86400.0};

class TempDir {
public:
  TempDir() {
    const char* base = std::getenv("TMPDIR");
    std::string tmpl = std::string(base ? base : "/tmp") + "/report_inputs_XXXXXX";
    std::vector<char> buf(tmpl.begin(), tmpl.end());
    buf.push_back('\0');
    if (::mkdtemp(buf.data()) != nullptr) path_ = buf.data();
  }
  ~TempDir() {
    if (path_.empty()) return;
    std::error_code ec;
    fs::permissions(path_, fs::perms::owner_all, fs::perm_options::add, ec);
    for (auto it = fs::recursive_directory_iterator(path_, ec); !ec && it != fs::recursive_directory_iterator(); it.increment(ec)) {
      fs::permissions(it->path(), fs::perms::owner_all, fs::perm_options::add, ec);
    }
    fs::remove_all(path_, ec);
  }
  const std::string& path() const { return path_; }

private:
  std::string path_;
};

PosRecord rec(double t, int q = 1, double lat = 44.5, double lon = 90.28) {
  PosRecord r;
  r.stamp = t;
  r.q = q;
  r.lat = lat;
  r.lon = lon;
  r.height = 600.0;
  r.ns = 20;
  return r;
}

void write_text(const std::string& path, const std::string& text) {
  fs::create_directories(fs::path(path).parent_path());
  std::ofstream(path, std::ios::binary) << text;
}

EventTransition ev(EventKind kind, double t, double t_open, const std::string& code,
                   std::optional<LatLon> pos = std::nullopt) {
  EventTransition e;
  e.kind = kind;
  e.t = t;
  e.t_open = t_open;
  e.code = code;
  e.level = Level::Serious;
  e.message = code + " 的结论";
  e.pos = pos;
  e.reason = CloseReason::Recovered;
  return e;
}

std::string events_file(const std::vector<EventTransition>& es) {
  std::string s = events_log_header();
  for (const auto& e : es) s += format_event_line(e) + "\n";
  return s;
}
}  // namespace

TEST(ReportInputs, LoadsPosSourcesInsideTheWindowSortedAndDropsEmptySources) {
  TempDir root;
  ASSERT_FALSE(root.path().empty());
  fs::create_directories(root.path() + "/20260915");
  write_pos(root.path() + "/20260915/can.pos", {rec(T + 10.0), rec(T + 5.0, 2)});   // 乱序写入,GPST
  write_pos(root.path() + "/20260915/rtkrcv.pos", {rec(T + 90000.0)});               // 次日,在窗口外
  write_pos(root.path() + "/20260914/gpchc.pos", {rec(T - 10.0)});                   // 前一天目录不读 .pos
  write_text(root.path() + "/20260915/base.pos", base_pos_header());                 // base.pos 不是数据源

  const auto in = load_report_inputs(root.path(), kDay);
  ASSERT_EQ(in.sources.size(), 1u) << "窗口内没有记录的源不出现";
  const auto& can = in.sources.at("can");
  ASSERT_EQ(can.size(), 2u);
  EXPECT_NEAR(can[0].stamp, T + 5.0, 1e-3) << "按时间升序";
  EXPECT_EQ(can[0].q, 2);
  EXPECT_NEAR(can[1].stamp, T + 10.0, 1e-3);
  EXPECT_TRUE(in.warnings.empty());
}

TEST(ReportInputs, PairsOpenAndCloseAcrossDaysAndKeepsTheOpenPosition) {
  TempDir root;
  ASSERT_FALSE(root.path().empty());
  const LatLon at_open{44.5, 90.28}, at_close{44.6, 90.29};
  write_text(root.path() + "/20260914/events.log",
             events_file({ev(EventKind::Open, T - 50.0, T - 50.0, "corr_outage", at_open)}));
  EventTransition close = ev(EventKind::Close, T + 30.0, T - 50.0, "corr_outage", at_close);
  close.peak = {{"corr_gap_s", 80.0}};
  write_text(root.path() + "/20260915/events.log",
             events_file({close, ev(EventKind::Open, T + 100.0, T + 100.0, "low_sats", at_close)}));

  const auto in = load_report_inputs(root.path(), kDay);
  ASSERT_EQ(in.events.size(), 2u);
  const auto& a = in.events[0];
  EXPECT_EQ(a.code, "corr_outage");
  EXPECT_NEAR(a.t_open, T - 50.0, 1e-3);
  ASSERT_TRUE(a.t_close.has_value());
  EXPECT_NEAR(*a.t_close, T + 30.0, 1e-3);
  EXPECT_EQ(a.close_reason, "recovered");
  ASSERT_TRUE(a.pos.has_value());
  EXPECT_NEAR(a.pos->lat, 44.5, 1e-9) << "问题路段取开启位置";
  EXPECT_NEAR(a.peak.at("corr_gap_s"), 80.0, 1e-9);
  EXPECT_EQ(a.level, "serious");
  const auto& b = in.events[1];
  EXPECT_EQ(b.code, "low_sats");
  EXPECT_FALSE(b.t_close.has_value()) << "没有关闭行:仍在进行或进程中断";
  EXPECT_TRUE(b.close_reason.empty());
}

TEST(ReportInputs, UnmatchedCloseStandsAloneAndASecondOpenLeavesTheFirstUnclosed) {
  TempDir root;
  ASSERT_FALSE(root.path().empty());
  const LatLon p{44.5, 90.28};
  const ReportWindow w{T + 100.0, T + 200.0};
  write_text(root.path() + "/20260915/events.log",
             events_file({
                 ev(EventKind::Open, T + 10.0, T + 10.0, "x", p),
                 ev(EventKind::Close, T + 20.0, T + 10.0, "x", p),             // 窗口前就关了
                 ev(EventKind::Open, T + 110.0, T + 110.0, "y", p),
                 ev(EventKind::Open, T + 150.0, T + 150.0, "y", p),            // 进程重启后又开
                 ev(EventKind::Close, T + 160.0, T + 5.0, "z", std::nullopt),  // OPEN 行找不到
                 ev(EventKind::Open, T + 300.0, T + 300.0, "w", p),            // 窗口后才开
             }));
  const auto in = load_report_inputs(root.path(), w);
  ASSERT_EQ(in.events.size(), 3u);
  EXPECT_EQ(in.events[0].code, "z");
  EXPECT_NEAR(in.events[0].t_open, T + 5.0, 1e-3);
  ASSERT_TRUE(in.events[0].t_close.has_value());
  EXPECT_FALSE(in.events[0].pos.has_value());
  EXPECT_EQ(in.events[1].code, "y");
  EXPECT_NEAR(in.events[1].t_open, T + 110.0, 1e-3);
  EXPECT_FALSE(in.events[1].t_close.has_value()) << "被第二条 OPEN 顶替,视为未关闭";
  EXPECT_EQ(in.events[2].code, "y");
  EXPECT_NEAR(in.events[2].t_open, T + 150.0, 1e-3);
}

TEST(ReportInputs, BaseHistoryIsWindowedAndBrokenInputsBecomeWarnings) {
  TempDir root;
  ASSERT_FALSE(root.path().empty());
  write_text(root.path() + "/20260915/base.pos",
             base_pos_header() + format_base_history_line(T + 1.0, Ecef{1.0, 2.0, 3.0}) + "\n" +
                 format_base_history_line(T + 90000.0, Ecef{4.0, 5.0, 6.0}) + "\n" + "2026/09/15 00:00:02.000  1.0\n");
  write_text(root.path() + "/20260915/events.log", events_log_header() + "hello world\n");
  const auto in = load_report_inputs(root.path(), kDay);
  ASSERT_EQ(in.base_history.size(), 1u);
  EXPECT_NEAR(in.base_history[0].t, T + 1.0, 1e-3);
  EXPECT_DOUBLE_EQ(in.base_history[0].p.z, 3.0);
  bool events_warned = false, base_warned = false;
  for (const auto& w : in.warnings) {
    if (w.find("events.log") != std::string::npos && w.find("1 行") != std::string::npos) events_warned = true;
    if (w.find("base.pos") != std::string::npos && w.find("1 行") != std::string::npos) base_warned = true;
  }
  EXPECT_TRUE(events_warned);
  EXPECT_TRUE(base_warned);
}

TEST(ReportInputs, UnreadablePosFileIsAWarningNotAFailure) {
  if (::geteuid() == 0) GTEST_SKIP() << "root 无视文件权限,无法构造读失败";
  TempDir root;
  ASSERT_FALSE(root.path().empty());
  write_pos(root.path() + "/20260915/can.pos", {rec(T + 1.0)});
  write_pos(root.path() + "/20260915/rtkrcv.pos", {rec(T + 1.0)});
  ASSERT_EQ(::chmod((root.path() + "/20260915/can.pos").c_str(), 0), 0);
  const auto in = load_report_inputs(root.path(), kDay);
  EXPECT_EQ(in.sources.count("can"), 0u);
  EXPECT_EQ(in.sources.count("rtkrcv"), 1u);
  ASSERT_EQ(in.warnings.size(), 1u);
  EXPECT_NE(in.warnings[0].find("can.pos"), std::string::npos) << in.warnings[0];
}

TEST(ReportInputs, MissingRootThrowsButAnEmptyRootIsFine) {
  TempDir root;
  ASSERT_FALSE(root.path().empty());
  EXPECT_THROW(load_report_inputs(root.path() + "/absent", kDay), std::runtime_error);
  const auto in = load_report_inputs(root.path(), kDay);
  EXPECT_TRUE(in.sources.empty());
  EXPECT_TRUE(in.events.empty());
  EXPECT_TRUE(in.base_history.empty());
  EXPECT_TRUE(in.warnings.empty());
}
```

`CMakeLists.txt`：`add_library(gnss_core SHARED ...)` 列表末尾（`src/diagnosis_engine.cpp` 之后）加 `src/report_inputs.cpp`；`BUILD_TESTING` 块里 `gnss_core_add_test(test_diagnosis_engine)` 之后加 `gnss_core_add_test(test_report_inputs)`。

- [ ] **Step 2: 确认 RED（对桩）**

桩：头文件按 Interfaces 写全；`load_report_inputs` 只做"root 不是目录就抛 `std::runtime_error`"，其余返回空的 `ReportInputs`。

Run: `cd /home/steve/glim_ws && source /opt/ros/humble/setup.bash && colcon build --symlink-install --packages-select gnss_core && ./build/gnss_core/test_report_inputs`
Expected: 前 5 个用例 FAIL；`MissingRootThrowsButAnEmptyRootIsFine` 对桩 PASS（Step 4 用变异验证）。

- [ ] **Step 3: 实现**

Create `include/gnss_core/report_inputs.hpp`：

```cpp
#pragma once
// 报告工具的输入装载(spec §3 F2,轮 4a):从 <root>/YYYYMMDD/ 读取时间窗内的 .pos、events.log、base.pos。
// 只读文件、配对事件,不做统计(统计在 report_stats.hpp)。
#include <map>
#include <optional>
#include <string>
#include <vector>

#include "gnss_core/base_station_monitor.hpp"
#include "gnss_core/event_book.hpp"
#include "gnss_core/pos_io.hpp"

namespace gnss_core {

// 报告时间窗:UTC unix 秒,半开区间 [t0, t1)
struct ReportWindow {
  double t0 = 0.0;
  double t1 = 0.0;
};

// 一次诊断事件:events.log 里 OPEN 行与 CLOSE 行配对后的结果
struct ReportEvent {
  std::string code;
  std::string level;                    // ok/info/warning/serious/critical
  std::string message;
  double t_open = 0.0;
  std::optional<double> t_close;        // 空:没找到关闭行(事件仍在进行,或进程在关闭前中断)
  std::string close_reason;             // recovered / shutdown;未关闭为空
  std::optional<LatLon> pos;            // 开启时位置;找不到 OPEN 行时取 CLOSE 行的位置
  std::map<std::string, double> peak;   // 仅已关闭事件
};

struct BaseSample {
  double t = 0.0;
  Ecef p;
};

struct ReportInputs {
  std::map<std::string, std::vector<PosRecord>> sources;   // 源名(.pos 文件名去扩展名) → 窗口内记录,时间升序
  std::vector<ReportEvent> events;                          // 与窗口有交集的事件,按开启时刻升序
  std::vector<BaseSample> base_history;                     // 窗口内基站坐标史,时间升序
  std::vector<std::string> warnings;                        // 读不了的文件、解析不了的行等,不中断装载
};

// root 不是目录时抛 std::runtime_error。
// .pos 读 [t0 所在日, t1 所在日] 的目录;events.log 与 base.pos 多读 t0 前一天,
// 以便找到窗口开始前开启、窗口内关闭的事件的 OPEN 行(设计决定 9、10)。
ReportInputs load_report_inputs(const std::string& root, const ReportWindow& window,
                                const PosReadOptions& pos_options = {});

}  // namespace gnss_core
```

Create `src/report_inputs.cpp`：

```cpp
#include "gnss_core/report_inputs.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <tuple>
#include <utility>

#include "gnss_core/diag_io.hpp"
#include "gnss_core/retention.hpp"

namespace gnss_core {

namespace {
namespace fs = std::filesystem;

bool in_window(double t, const ReportWindow& w) { return t >= w.t0 && t < w.t1; }

// root 下日期落在 [first, last] 的 YYYYMMDD 目录,按日期升序
std::vector<std::pair<int, fs::path>> day_dirs(const std::string& root, int first, int last) {
  std::error_code ec;
  if (!fs::is_directory(root, ec)) throw std::runtime_error("报告根目录不存在或不是目录: " + root);
  std::vector<std::pair<int, fs::path>> out;
  fs::directory_iterator it(root, ec);
  if (ec) throw std::runtime_error("无法遍历 " + root + ": " + ec.message());
  for (; !ec && it != fs::directory_iterator(); it.increment(ec)) {
    std::error_code type_ec;
    if (!it->is_directory(type_ec) || type_ec) continue;
    const auto date = parse_day_dir_date(it->path().filename().string());
    if (date && *date >= first && *date <= last) out.emplace_back(*date, it->path());
  }
  std::sort(out.begin(), out.end());
  return out;
}

bool read_lines(const fs::path& path, std::vector<std::string>& lines) {
  std::ifstream in(path, std::ios::binary);
  if (!in) return false;
  for (std::string line; std::getline(in, line);) {
    if (!line.empty() && line.back() == '\r') line.pop_back();
    lines.push_back(std::move(line));
  }
  return !in.bad();
}

bool is_content_line(const std::string& line) { return !line.empty() && line[0] != '%'; }

// 设计决定 9:按文件顺序配对 OPEN/CLOSE
std::vector<ReportEvent> pair_events(const std::vector<EventLogLine>& lines) {
  std::map<std::string, ReportEvent> pending;
  std::vector<ReportEvent> all;
  for (const auto& l : lines) {
    if (l.kind == EventKind::Open) {
      auto it = pending.find(l.code);
      if (it != pending.end()) {   // 同一规则码又开了:上一条没等到关闭行(进程中断后重启)
        all.push_back(it->second);
        pending.erase(it);
      }
      ReportEvent e;
      e.code = l.code;
      e.level = l.level;
      e.message = l.message;
      e.t_open = l.t;
      e.pos = l.pos;
      pending.emplace(l.code, std::move(e));
      continue;
    }
    ReportEvent e;
    auto it = pending.find(l.code);
    if (it != pending.end() && std::abs(it->second.t_open - l.t_open) < 0.002) {
      e = std::move(it->second);
      pending.erase(it);
      if (!e.pos) e.pos = l.pos;
    } else {
      e.code = l.code;
      e.level = l.level;
      e.message = l.message;
      e.t_open = l.t_open;
      e.pos = l.pos;
    }
    e.t_close = l.t;
    e.close_reason = l.reason;
    e.peak = l.peak;
    all.push_back(std::move(e));
  }
  for (auto& [code, e] : pending) all.push_back(std::move(e));
  return all;
}
}  // namespace

ReportInputs load_report_inputs(const std::string& root, const ReportWindow& window,
                                const PosReadOptions& pos_options) {
  ReportInputs in;
  const int pos_first = utc_yyyymmdd(window.t0);
  const int first = utc_yyyymmdd(window.t0 - 86400.0);
  const int last = utc_yyyymmdd(window.t1 - 1e-3);
  std::vector<EventLogLine> event_lines;

  for (const auto& [date, dir] : day_dirs(root, first, last)) {
    if (date >= pos_first) {
      std::vector<fs::path> pos_files;
      std::error_code ec;
      for (fs::directory_iterator it(dir, ec); !ec && it != fs::directory_iterator(); it.increment(ec)) {
        std::error_code type_ec;
        if (!it->is_regular_file(type_ec) || type_ec) continue;
        const fs::path& p = it->path();
        if (p.extension() == ".pos" && p.filename() != "base.pos" && !p.stem().empty()) pos_files.push_back(p);
      }
      std::sort(pos_files.begin(), pos_files.end());
      for (const auto& p : pos_files) {
        try {
          auto records = read_pos(p.string(), pos_options);
          auto& dst = in.sources[p.stem().string()];
          for (auto& r : records) {
            if (in_window(r.stamp, window)) dst.push_back(std::move(r));
          }
        } catch (const std::exception& e) {
          in.warnings.push_back("读取 " + p.string() + " 失败: " + e.what());
        }
      }
    }

    const fs::path events_path = dir / "events.log";
    std::error_code ec;
    if (fs::exists(events_path, ec)) {
      std::vector<std::string> lines;
      if (!read_lines(events_path, lines)) {
        in.warnings.push_back("读取 " + events_path.string() + " 失败");
      } else {
        int bad = 0;
        for (const auto& line : lines) {
          if (auto parsed = parse_event_line(line)) {
            event_lines.push_back(std::move(*parsed));
          } else if (is_content_line(line)) {
            ++bad;
          }
        }
        if (bad > 0) {
          in.warnings.push_back(events_path.string() + " 中 " + std::to_string(bad) + " 行无法解析(可能是掉电留下的半行)");
        }
      }
    }

    const fs::path base_path = dir / "base.pos";
    if (fs::exists(base_path, ec)) {
      std::vector<std::string> lines;
      if (!read_lines(base_path, lines)) {
        in.warnings.push_back("读取 " + base_path.string() + " 失败");
      } else {
        int bad = 0;
        for (const auto& line : lines) {
          if (const auto parsed = parse_base_history_line(line)) {
            if (in_window(parsed->first, window)) in.base_history.push_back(BaseSample{parsed->first, parsed->second});
          } else if (is_content_line(line)) {
            ++bad;
          }
        }
        if (bad > 0) {
          in.warnings.push_back(base_path.string() + " 中 " + std::to_string(bad) + " 行无法解析(可能是掉电留下的半行)");
        }
      }
    }
  }

  for (auto it = in.sources.begin(); it != in.sources.end();) {
    if (it->second.empty()) {
      it = in.sources.erase(it);
    } else {
      std::stable_sort(it->second.begin(), it->second.end(),
                       [](const PosRecord& a, const PosRecord& b) { return a.stamp < b.stamp; });
      ++it;
    }
  }
  std::stable_sort(in.base_history.begin(), in.base_history.end(),
                   [](const BaseSample& a, const BaseSample& b) { return a.t < b.t; });

  for (auto& e : pair_events(event_lines)) {
    if (e.t_open < window.t1 && (!e.t_close || *e.t_close >= window.t0)) in.events.push_back(std::move(e));
  }
  std::stable_sort(in.events.begin(), in.events.end(), [](const ReportEvent& a, const ReportEvent& b) {
    return std::tie(a.t_open, a.code) < std::tie(b.t_open, b.code);
  });
  return in;
}

}  // namespace gnss_core
```

- [ ] **Step 4: GREEN + 变异 + 全量**

Run: `./build/gnss_core/test_report_inputs` 全部 PASS（`UnreadablePosFileIsAWarningNotAFailure` 在 root 用户下 SKIP 属正常，记录实际结果）。

变异（逐个做、逐个恢复）：
1. `day_dirs` 里两处 `throw` 都换成 `return out;` → `MissingRootThrowsButAnEmptyRootIsFine` 必须 FAIL。
2. `pair_events` 里"同一规则码又开了"那段删掉（直接覆盖 `pending`）→ `UnmatchedCloseStandsAlone...` 必须 FAIL。
3. 删掉 `.pos` 记录的 `in_window(r.stamp, window)` 过滤（全部 push）→ `LoadsPosSourcesInsideTheWindowSortedAndDropsEmptySources` 必须 FAIL（次日的 rtkrcv 记录会进来）。

全量测试 0 failures。

- [ ] **Step 5: 提交**

```bash
cd /home/steve/glim_ws/src/glim_ext
git add gnss_core/include/gnss_core/report_inputs.hpp gnss_core/src/report_inputs.cpp \
  gnss_core/test/test_report_inputs.cpp gnss_core/CMakeLists.txt
git commit -m "feat(gnss_core): load .pos, events and base history for a report window

<RED(桩)与变异结果>

Co-Authored-By: <实际模型>"
```

---
### Task 3: 报告统计

落实设计决定 5–9、11，口径对照 rtk-monitor `report.py` 与 `tests/test_report.py`。

**Files:**
- Create: `glim_ext/gnss_core/include/gnss_core/report_stats.hpp`
- Create: `glim_ext/gnss_core/src/report_stats.cpp`
- Create: `glim_ext/gnss_core/test/test_report_stats.cpp`
- Modify: `glim_ext/gnss_core/CMakeLists.txt`

**Interfaces:**
- Consumes: Task 2 的 `ReportInputs`、`ReportWindow`、`ReportEvent`、`BaseSample`；已有 `ControlPoint`（diagnosis.hpp）、`geodesic_distance_m`、`LlaToEnu`（geodetic.hpp）
- Produces（`namespace gnss_core`）：下面头文件里的全部类型，以及
  - `std::vector<std::string> ordered_source_names(const std::map<std::string, std::vector<PosRecord>>& sources)`
  - `ReportStats compute_report(const ReportInputs& inputs, const ReportParams& params)`

- [ ] **Step 1: 写失败测试**

Create `test/test_report_stats.cpp`：

```cpp
#include <gtest/gtest.h>

#include <cmath>
#include <string>
#include <vector>

#include "gnss_core/report_stats.hpp"
using namespace gnss_core;

namespace {
const double kBase = 3600.0 * 100;   // 整点
const double kLat = 44.5, kLon = 90.28;

double north(double metres) { return kLat + metres / 111132.0; }

PosRecord rec(double t, int q = 1, double lat = kLat, double lon = kLon) {
  PosRecord r;
  r.stamp = t;
  r.q = q;
  r.lat = lat;
  r.lon = lon;
  r.height = 600.0;
  return r;
}

ReportParams params(double t0, double t1) {
  ReportParams p;
  p.window = ReportWindow{t0, t1};
  return p;
}

ReportEvent event(const std::string& code, double t_open, std::optional<double> t_close,
                  std::optional<LatLon> pos = std::nullopt) {
  ReportEvent e;
  e.code = code;
  e.level = "serious";
  e.message = code;
  e.t_open = t_open;
  e.t_close = t_close;
  if (t_close) e.close_reason = "recovered";
  e.pos = pos;
  return e;
}
}  // namespace

// 移植 rtk-monitor test_report_stats:固定率 9/11、分小时 0.8
TEST(ReportStats, FixRatioQualityCountsAndHourlyPerSource) {
  ReportInputs in;
  for (int i = 0; i < 10; ++i) in.sources["rtkrcv"].push_back(rec(kBase + i, i < 8 ? 1 : 2));
  in.sources["rtkrcv"].push_back(rec(kBase + 3700.0, 1));
  in.sources["can"] = {rec(kBase, 1), rec(kBase + 1, 1), rec(kBase + 2, 2), rec(kBase + 3, 5)};

  const auto s = compute_report(in, params(kBase, kBase + 7200.0));
  ASSERT_EQ(s.sources.size(), 2u);
  EXPECT_EQ(s.sources[0].name, "can");
  EXPECT_EQ(s.sources[1].name, "rtkrcv");

  const auto& rtk = s.sources[1];
  EXPECT_EQ(rtk.epochs, 11);
  ASSERT_TRUE(rtk.fix_ratio.has_value());
  EXPECT_NEAR(*rtk.fix_ratio, 9.0 / 11.0, 1e-12);
  ASSERT_EQ(rtk.hourly.size(), 2u);
  EXPECT_DOUBLE_EQ(rtk.hourly[0].t_start, kBase);
  EXPECT_EQ(rtk.hourly[0].epochs, 10);
  EXPECT_NEAR(*rtk.hourly[0].fix_ratio, 0.8, 1e-12);
  EXPECT_EQ(rtk.hourly[1].epochs, 1);
  EXPECT_NEAR(*rtk.hourly[1].fix_ratio, 1.0, 1e-12);

  const auto& can = s.sources[0];
  EXPECT_NEAR(*can.fix_ratio, 0.5, 1e-12) << "固定解一律按 RTKLIB Q==1,不沿用 rtk-monitor 的 can q==4";
  EXPECT_EQ(can.counts.fixed, 2);
  EXPECT_EQ(can.counts.floating, 1);
  EXPECT_EQ(can.counts.dgps, 0);
  EXPECT_EQ(can.counts.single, 1);
  EXPECT_EQ(can.counts.other, 0);
}

TEST(ReportStats, HourlyBucketsCoverTheWholeWindowAndEmptyHoursHaveNoRatio) {
  ReportInputs in;
  in.sources["rtkrcv"] = {rec(kBase + 10.0)};
  auto s = compute_report(in, params(kBase, kBase + 3 * 3600.0));
  ASSERT_EQ(s.sources[0].hourly.size(), 3u);
  EXPECT_EQ(s.sources[0].hourly[1].epochs, 0);
  EXPECT_FALSE(s.sources[0].hourly[1].fix_ratio.has_value());

  s = compute_report(in, params(kBase + 1800.0, kBase + 5400.0));   // 不在整点开始的窗口
  ASSERT_EQ(s.sources[0].hourly.size(), 2u);
  EXPECT_DOUBLE_EQ(s.sources[0].hourly[0].t_start, kBase);
  EXPECT_DOUBLE_EQ(s.sources[0].hourly[1].t_start, kBase + 3600.0);
}

TEST(ReportStats, SourcesAreOrderedCanGpchcRtkrcvRefThenOthersByName) {
  ReportInputs in;
  for (const char* n : {"zeta", "ref", "can", "rtkrcv", "alpha"}) in.sources[n] = {rec(kBase)};
  const std::vector<std::string> want{"can", "rtkrcv", "ref", "alpha", "zeta"};
  EXPECT_EQ(ordered_source_names(in.sources), want);
  const auto s = compute_report(in, params(kBase, kBase + 10.0));
  ASSERT_EQ(s.sources.size(), want.size());
  for (size_t i = 0; i < want.size(); ++i) EXPECT_EQ(s.sources[i].name, want[i]);
}

// 移植 rtk-monitor test_report_can_rtk_deviation
TEST(ReportStats, DeviceVsRtkrcvPairsTheNearestEpochWithinTolerance) {
  ReportInputs in;
  in.sources["rtkrcv"] = {rec(kBase + 1.0), rec(kBase + 2.0)};
  in.sources["can"] = {rec(kBase + 1.2, 1, north(1.0))};
  const auto s = compute_report(in, params(kBase, kBase + 10.0));
  ASSERT_EQ(s.divergence.size(), 1u);
  EXPECT_EQ(s.divergence[0].device, "can");
  EXPECT_EQ(s.divergence[0].reference, "rtkrcv");
  EXPECT_EQ(s.divergence[0].n, 1) << "kBase+2 的独立解离 can 历元 0.8 s,不配对";
  EXPECT_NEAR(*s.divergence[0].max_m, 1.0, 0.01);
  EXPECT_NEAR(*s.divergence[0].mean_m, 1.0, 0.01);
}

// 移植 rtk-monitor test_report_can_rtk_deviation_skips_far_apart_timestamps,并覆盖 gpchc 与容差边界
TEST(ReportStats, FarApartEpochsAreNotPairedAndEachDeviceIsReported) {
  ReportInputs in;
  in.sources["rtkrcv"] = {rec(kBase + 1.0), rec(kBase + 5.0)};
  in.sources["can"] = {rec(kBase + 1.9, 1, north(5.0))};
  in.sources["gpchc"] = {rec(kBase + 5.5, 1, north(2.0))};   // 恰好 0.5 s,容差含边界
  auto s = compute_report(in, params(kBase, kBase + 10.0));
  ASSERT_EQ(s.divergence.size(), 2u);
  EXPECT_EQ(s.divergence[0].device, "can");
  EXPECT_EQ(s.divergence[0].n, 0);
  EXPECT_FALSE(s.divergence[0].max_m.has_value());
  EXPECT_EQ(s.divergence[1].device, "gpchc");
  EXPECT_EQ(s.divergence[1].n, 1);
  EXPECT_NEAR(*s.divergence[1].max_m, 2.0, 0.01);

  ReportInputs no_ref;
  no_ref.sources["can"] = {rec(kBase + 1.0)};
  EXPECT_TRUE(compute_report(no_ref, params(kBase, kBase + 10.0)).divergence.empty()) << "没有 rtkrcv 就无法比较";
}

// 移植 test_report_abs_ref_control_point_deviation / ignores_non_fixed_epochs,改为只取最近控制点
TEST(ReportStats, AbsRefUsesFixedEpochsOfEverySourceAndTheNearestControlPoint) {
  ReportInputs in;
  in.sources["rtkrcv"] = {rec(kBase + 1.0, 1, north(0.5)), rec(kBase + 2.0, 2, north(0.3)), rec(kBase + 3.0, 1, 44.6)};
  in.sources["can"] = {rec(kBase + 4.0, 1, north(1.8))};
  auto p = params(kBase, kBase + 10.0);
  p.control_points = {ControlPoint{"CP1", kLat, kLon}, ControlPoint{"CP2", north(2.0), kLon}};

  auto s = compute_report(in, p);
  ASSERT_EQ(s.abs_ref.samples.size(), 2u);
  EXPECT_EQ(s.abs_ref.samples[0].source, "rtkrcv");
  EXPECT_EQ(s.abs_ref.samples[0].control_point, "CP1") << "离 CP1 0.5 m、CP2 1.5 m,只算最近的";
  EXPECT_NEAR(s.abs_ref.samples[0].dev_m, 0.5, 0.01);
  EXPECT_EQ(s.abs_ref.samples[1].source, "can");
  EXPECT_EQ(s.abs_ref.samples[1].control_point, "CP2");
  EXPECT_NEAR(s.abs_ref.samples[1].dev_m, 0.2, 0.01);
  EXPECT_NEAR(*s.abs_ref.max_m, 0.5, 0.01);
  EXPECT_TRUE(s.abs_ref.exceeded);

  p.abs_ref_max_m = 0.6;
  EXPECT_FALSE(compute_report(in, p).abs_ref.exceeded);
  p.control_points.clear();
  s = compute_report(in, p);
  EXPECT_TRUE(s.abs_ref.samples.empty());
  EXPECT_FALSE(s.abs_ref.max_m.has_value());
  EXPECT_FALSE(s.abs_ref.exceeded);
}

// 移植 test_report_base_series_curve
TEST(ReportStats, BaseSeriesIsTheOffsetFromTheFirstSample) {
  ReportInputs in;
  in.base_history = {BaseSample{kBase, Ecef{-2148744.0, 4426641.0, 4044655.0}},
                     BaseSample{kBase + 10.0, Ecef{-2148744.3, 4426641.0, 4044655.0}}};
  auto p = params(kBase, kBase + 100.0);
  auto s = compute_report(in, p);
  ASSERT_EQ(s.base.series.size(), 2u);
  EXPECT_DOUBLE_EQ(s.base.series[0].offset_m, 0.0);
  EXPECT_NEAR(s.base.series[1].offset_m, 0.3, 1e-6);
  EXPECT_NEAR(*s.base.max_m, 0.3, 1e-6);
  EXPECT_TRUE(s.base.exceeded) << "超过 base_shift_m 0.1";
  p.base_shift_m = 0.5;
  EXPECT_FALSE(compute_report(in, p).base.exceeded);
  EXPECT_FALSE(compute_report(ReportInputs{}, p).base.max_m.has_value());
}

TEST(ReportStats, EventsAreCopiedAndSummarisedByCode) {
  ReportInputs in;
  in.events = {event("corr_outage", kBase, kBase + 28.0), event("low_sats", kBase + 50.0, std::nullopt),
               event("corr_outage", kBase + 100.0, kBase + 112.0)};
  in.warnings = {"某个警告"};
  const auto s = compute_report(in, params(kBase, kBase + 200.0));
  EXPECT_EQ(s.events.size(), 3u);
  ASSERT_EQ(s.event_summary.size(), 2u);
  EXPECT_EQ(s.event_summary[0].code, "corr_outage");
  EXPECT_EQ(s.event_summary[0].count, 2);
  EXPECT_NEAR(s.event_summary[0].closed_duration_s, 40.0, 1e-9);
  EXPECT_EQ(s.event_summary[0].unclosed, 0);
  EXPECT_EQ(s.event_summary[1].code, "low_sats");
  EXPECT_EQ(s.event_summary[1].count, 1);
  EXPECT_EQ(s.event_summary[1].unclosed, 1);
  EXPECT_EQ(s.warnings, in.warnings) << "装载阶段的警告要带进报告";
}

TEST(ReportStats, TracksShareOneOriginAndSplitOnTimeGaps) {
  ReportInputs in;
  for (int i = 0; i < 10; ++i) in.sources["can"].push_back(rec(kBase + i, 1, north(i)));
  for (int i = 0; i < 5; ++i) in.sources["can"].push_back(rec(kBase + 30 + i, 2, north(20 + i)));
  in.sources["rtkrcv"] = {rec(kBase + 5.0, 1, north(3.0))};
  const auto s = compute_report(in, params(kBase, kBase + 100.0));
  ASSERT_TRUE(s.track_origin.has_value());
  EXPECT_EQ(*s.track_origin, "can");
  ASSERT_EQ(s.tracks.size(), 2u);
  EXPECT_EQ(s.tracks[0].source, "can");
  ASSERT_EQ(s.tracks[0].segments.size(), 2u) << "中间断了 21 s > 5 s";
  ASSERT_EQ(s.tracks[0].segments[0].size(), 10u);
  EXPECT_NEAR(s.tracks[0].segments[0][0].e, 0.0, 1e-6);
  EXPECT_NEAR(s.tracks[0].segments[0][0].n, 0.0, 1e-6);
  EXPECT_NEAR(s.tracks[0].segments[0][9].n, 9.0, 0.02);
  EXPECT_EQ(s.tracks[0].segments[1].size(), 5u);
  EXPECT_EQ(s.tracks[0].segments[1][0].q, 2);
  ASSERT_EQ(s.tracks[1].segments.size(), 1u);
  EXPECT_NEAR(s.tracks[1].segments[0][0].n, 3.0, 0.02) << "所有源用同一个原点";
}

TEST(ReportStats, LongTracksAreDownsampledKeepingSegmentEnds) {
  ReportInputs in;
  for (int i = 0; i < 10000; ++i) in.sources["can"].push_back(rec(kBase + i, 1, north(i * 0.1)));
  auto p = params(kBase, kBase + 20000.0);
  p.max_track_points = 4000;
  const auto s = compute_report(in, p);
  ASSERT_EQ(s.tracks.size(), 1u);
  ASSERT_EQ(s.tracks[0].segments.size(), 1u);
  const auto& seg = s.tracks[0].segments[0];
  EXPECT_LE(seg.size(), 4002u);
  EXPECT_GE(seg.size(), 2500u);
  EXPECT_DOUBLE_EQ(seg.front().t, kBase);
  EXPECT_DOUBLE_EQ(seg.back().t, kBase + 9999.0) << "段尾必须保留";
}

TEST(ReportStats, EventMarkersUseTheEventIndexAndSkipEventsWithoutPosition) {
  ReportInputs in;
  in.events = {event("a", kBase, kBase + 1.0, LatLon{kLat, kLon}), event("b", kBase + 2.0, std::nullopt),
               event("c", kBase + 3.0, std::nullopt, LatLon{north(10.0), kLon})};
  const auto s = compute_report(in, params(kBase, kBase + 10.0));
  ASSERT_TRUE(s.track_origin.has_value());
  EXPECT_EQ(*s.track_origin, "事件") << "没有 .pos 时用第一个带位置的事件作原点";
  ASSERT_EQ(s.event_markers.size(), 2u);
  EXPECT_EQ(s.event_markers[0].index, 1);
  EXPECT_NEAR(s.event_markers[0].n, 0.0, 1e-6);
  EXPECT_EQ(s.event_markers[1].index, 3);
  EXPECT_NEAR(s.event_markers[1].n, 10.0, 0.05);
  EXPECT_TRUE(s.tracks.empty());
}

TEST(ReportStats, EmptyInputsProduceEmptyStats) {
  const auto s = compute_report(ReportInputs{}, params(kBase, kBase + 100.0));
  EXPECT_TRUE(s.sources.empty());
  EXPECT_TRUE(s.divergence.empty());
  EXPECT_TRUE(s.events.empty());
  EXPECT_TRUE(s.tracks.empty());
  EXPECT_FALSE(s.track_origin.has_value());
  EXPECT_DOUBLE_EQ(s.params.window.t1, kBase + 100.0);
}
```

`CMakeLists.txt`：库源文件列表加 `src/report_stats.cpp`；测试注册加 `gnss_core_add_test(test_report_stats)`。

- [ ] **Step 2: 确认 RED（对桩）**

桩：头文件写全；`ordered_source_names` 返回空 vector；`compute_report` 只返回 `ReportStats{}` 并把 `params` 抄进去。

Run: `colcon build --symlink-install --packages-select gnss_core && ./build/gnss_core/test_report_stats`
Expected: 除 `EmptyInputsProduceEmptyStats` 外全部 FAIL；`EmptyInputsProduceEmptyStats` 对桩 PASS（它钉的是空输入不崩、不造数据，Step 4 用变异验证）。

- [ ] **Step 3: 实现**

Create `include/gnss_core/report_stats.hpp`：

```cpp
#pragma once
// 报告统计(spec §3 F2,轮 4a)。口径移植自 rtk-monitor report.py,
// 差异见 docs/gnss/plans/2026-09-16-round4a-report-tool.md「设计决定」5–11。纯函数,不读文件。
#include <cstddef>
#include <map>
#include <optional>
#include <string>
#include <vector>

#include "gnss_core/diagnosis.hpp"
#include "gnss_core/report_inputs.hpp"

namespace gnss_core {

struct ReportParams {
  ReportWindow window;
  std::vector<ControlPoint> control_points;
  double abs_ref_radius_m = 3.0;         // 经过控制点的判定半径
  double abs_ref_max_m = 0.2;            // 绝对基准告警阈值
  double base_shift_m = 0.1;             // 基站坐标变动告警阈值
  double pair_tol_s = 0.5;               // 610 与独立解按时间配对的容差(含边界)
  std::size_t max_track_points = 4000;   // 轨迹图每源最多点数
  double track_gap_s = 5.0;              // 相邻记录间隔超过此值时断开折线
};

// 按 RTKLIB Q 计数:1 fixed、2 float、4 dgps、5 single,其余计 other
struct QualityCounts {
  int fixed = 0, floating = 0, dgps = 0, single = 0, other = 0;
};

struct HourBucket {
  double t_start = 0.0;              // UTC 整点
  int epochs = 0;
  std::optional<double> fix_ratio;   // 该小时没有记录时为空
};

struct SourceStats {
  std::string name;
  int epochs = 0;
  std::optional<double> fix_ratio;
  QualityCounts counts;
  std::vector<HourBucket> hourly;    // 覆盖整个时间窗的每个 UTC 小时
};

struct DivergenceStats {
  std::string device;                // can / gpchc
  std::string reference = "rtkrcv";
  int n = 0;
  std::optional<double> max_m, mean_m;
};

struct AbsRefSample {
  double t = 0.0;
  std::string source;
  std::string control_point;
  double dev_m = 0.0;
};

struct AbsRefStats {
  std::vector<AbsRefSample> samples;   // 按时间升序
  std::optional<double> max_m;
  bool exceeded = false;               // max_m > abs_ref_max_m
};

struct BaseOffsetSample {
  double t = 0.0;
  double offset_m = 0.0;
};

struct BaseStats {
  std::vector<BaseOffsetSample> series;   // 相对时间窗内第一条记录
  std::optional<double> max_m;
  bool exceeded = false;                  // max_m > base_shift_m
};

struct TrackPoint {
  double t = 0.0;
  double e = 0.0, n = 0.0;   // 局部 ENU 东/北(m)
  int q = 0;                 // RTKLIB Q
};

struct Track {
  std::string source;
  std::vector<std::vector<TrackPoint>> segments;   // 按时间断开的折线段
};

struct EventMarker {
  int index = 0;             // 事件表里的编号(从 1 起)
  double e = 0.0, n = 0.0;
};

struct EventCodeSummary {
  std::string code;
  int count = 0;
  double closed_duration_s = 0.0;   // 已关闭事件的时长之和
  int unclosed = 0;
};

struct ReportStats {
  ReportParams params;
  std::vector<SourceStats> sources;              // 顺序同 ordered_source_names
  std::vector<DivergenceStats> divergence;       // 有 rtkrcv 时,can、gpchc 中存在的各一条
  AbsRefStats abs_ref;
  BaseStats base;
  std::vector<ReportEvent> events;
  std::vector<EventCodeSummary> event_summary;   // 次数降序,次数相同按代码
  std::vector<Track> tracks;                     // 顺序同 sources
  std::vector<EventMarker> event_markers;
  std::optional<std::string> track_origin;       // 局部坐标原点取自哪个源("事件" 表示取自事件);无任何位置时为空
  std::vector<std::string> warnings;             // 装载阶段带来的警告
};

// can、gpchc、rtkrcv、ref 在前(存在才列),其余按名字
std::vector<std::string> ordered_source_names(const std::map<std::string, std::vector<PosRecord>>& sources);

ReportStats compute_report(const ReportInputs& inputs, const ReportParams& params);

}  // namespace gnss_core
```

Create `src/report_stats.cpp`：

```cpp
#include "gnss_core/report_stats.hpp"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>
#include <map>
#include <memory>
#include <utility>

#include "gnss_core/geodetic.hpp"

namespace gnss_core {

namespace {
constexpr int kFixedQ = 1;   // RTKLIB Q:固定解(设计决定 5)

std::optional<double> ratio(int num, int den) {
  if (den <= 0) return std::nullopt;
  return static_cast<double>(num) / static_cast<double>(den);
}

SourceStats source_stats(const std::string& name, const std::vector<PosRecord>& recs, const ReportWindow& w) {
  SourceStats s;
  s.name = name;
  s.epochs = static_cast<int>(recs.size());
  for (const auto& r : recs) {
    switch (r.q) {
      case 1: ++s.counts.fixed; break;
      case 2: ++s.counts.floating; break;
      case 4: ++s.counts.dgps; break;
      case 5: ++s.counts.single; break;
      default: ++s.counts.other; break;
    }
  }
  s.fix_ratio = ratio(s.counts.fixed, s.epochs);

  const long long h0 = static_cast<long long>(std::floor(w.t0 / 3600.0));
  const long long h1 = static_cast<long long>(std::ceil(w.t1 / 3600.0));
  if (h1 <= h0) return s;
  std::vector<int> n(static_cast<size_t>(h1 - h0), 0), fixed(n.size(), 0);
  for (const auto& r : recs) {
    const long long idx = static_cast<long long>(std::floor(r.stamp / 3600.0)) - h0;
    if (idx < 0 || idx >= static_cast<long long>(n.size())) continue;
    ++n[static_cast<size_t>(idx)];
    if (r.q == kFixedQ) ++fixed[static_cast<size_t>(idx)];
  }
  for (size_t i = 0; i < n.size(); ++i) {
    s.hourly.push_back(HourBucket{static_cast<double>(h0 + static_cast<long long>(i)) * 3600.0, n[i],
                                  ratio(fixed[i], n[i])});
  }
  return s;
}

// 设计决定 6:对每个独立解历元取时间最近的设备历元,|Δt| <= tol 才配对
DivergenceStats divergence_stats(const std::string& device, const std::vector<PosRecord>& dev,
                                 const std::vector<PosRecord>& ref, double tol) {
  DivergenceStats d;
  d.device = device;
  double sum = 0.0;
  for (const auto& r : ref) {
    const auto it = std::lower_bound(dev.begin(), dev.end(), r.stamp,
                                     [](const PosRecord& a, double t) { return a.stamp < t; });
    const PosRecord* best = nullptr;
    double best_dt = std::numeric_limits<double>::infinity();
    if (it != dev.end()) {
      best = &*it;
      best_dt = std::abs(it->stamp - r.stamp);
    }
    if (it != dev.begin()) {
      const auto prev = std::prev(it);
      if (std::abs(prev->stamp - r.stamp) < best_dt) {
        best = &*prev;
        best_dt = std::abs(prev->stamp - r.stamp);
      }
    }
    if (!best || best_dt > tol) continue;
    const double dist = geodesic_distance_m(r.lat, r.lon, best->lat, best->lon);
    ++d.n;
    sum += dist;
    d.max_m = d.max_m ? std::max(*d.max_m, dist) : dist;
  }
  if (d.n > 0) d.mean_m = sum / d.n;
  return d;
}
}  // namespace

std::vector<std::string> ordered_source_names(const std::map<std::string, std::vector<PosRecord>>& sources) {
  static const char* const kPreferred[] = {"can", "gpchc", "rtkrcv", "ref"};
  std::vector<std::string> out;
  for (const char* n : kPreferred) {
    if (sources.count(n)) out.emplace_back(n);
  }
  for (const auto& [name, recs] : sources) {
    if (std::find(out.begin(), out.end(), name) == out.end()) out.push_back(name);
  }
  return out;
}

ReportStats compute_report(const ReportInputs& in, const ReportParams& p) {
  ReportStats s;
  s.params = p;
  s.warnings = in.warnings;
  s.events = in.events;
  const auto names = ordered_source_names(in.sources);

  for (const auto& name : names) s.sources.push_back(source_stats(name, in.sources.at(name), p.window));

  const auto ref = in.sources.find("rtkrcv");
  if (ref != in.sources.end()) {
    for (const char* device : {"can", "gpchc"}) {
      const auto dev = in.sources.find(device);
      if (dev != in.sources.end()) s.divergence.push_back(divergence_stats(device, dev->second, ref->second, p.pair_tol_s));
    }
  }

  // 设计决定 7:每个源的固定解,只比最近的控制点
  if (!p.control_points.empty()) {
    for (const auto& name : names) {
      for (const auto& r : in.sources.at(name)) {
        if (r.q != kFixedQ) continue;
        const ControlPoint* nearest = nullptr;
        double nearest_m = std::numeric_limits<double>::infinity();
        for (const auto& cp : p.control_points) {
          const double d = geodesic_distance_m(r.lat, r.lon, cp.lat, cp.lon);
          if (d < nearest_m) {
            nearest_m = d;
            nearest = &cp;
          }
        }
        if (nearest && nearest_m <= p.abs_ref_radius_m) {
          s.abs_ref.samples.push_back(AbsRefSample{r.stamp, name, nearest->name, nearest_m});
        }
      }
    }
    std::stable_sort(s.abs_ref.samples.begin(), s.abs_ref.samples.end(),
                     [](const AbsRefSample& a, const AbsRefSample& b) { return a.t < b.t; });
    for (const auto& x : s.abs_ref.samples) s.abs_ref.max_m = s.abs_ref.max_m ? std::max(*s.abs_ref.max_m, x.dev_m) : x.dev_m;
    s.abs_ref.exceeded = s.abs_ref.max_m && *s.abs_ref.max_m > p.abs_ref_max_m;
  }

  // 设计决定 8:相对窗口内第一条基站坐标
  if (!in.base_history.empty()) {
    const Ecef& p0 = in.base_history.front().p;
    for (const auto& b : in.base_history) {
      const double off = std::sqrt((b.p.x - p0.x) * (b.p.x - p0.x) + (b.p.y - p0.y) * (b.p.y - p0.y) +
                                   (b.p.z - p0.z) * (b.p.z - p0.z));
      s.base.series.push_back(BaseOffsetSample{b.t, off});
      s.base.max_m = s.base.max_m ? std::max(*s.base.max_m, off) : off;
    }
    s.base.exceeded = *s.base.max_m > p.base_shift_m;
  }

  // 事件按规则码汇总
  std::map<std::string, EventCodeSummary> by_code;
  for (const auto& e : in.events) {
    auto& x = by_code[e.code];
    x.code = e.code;
    ++x.count;
    if (e.t_close) {
      x.closed_duration_s += *e.t_close - e.t_open;
    } else {
      ++x.unclosed;
    }
  }
  for (auto& [code, x] : by_code) s.event_summary.push_back(x);
  std::stable_sort(s.event_summary.begin(), s.event_summary.end(),
                   [](const EventCodeSummary& a, const EventCodeSummary& b) { return a.count > b.count; });

  // 设计决定 11:共用一个局部 ENU 原点
  std::unique_ptr<LlaToEnu> origin;
  for (const auto& name : names) {
    const auto& recs = in.sources.at(name);
    origin = std::make_unique<LlaToEnu>(recs.front().lat, recs.front().lon, recs.front().height);
    s.track_origin = name;
    break;
  }
  if (!origin) {
    for (const auto& e : in.events) {
      if (!e.pos) continue;
      origin = std::make_unique<LlaToEnu>(e.pos->lat, e.pos->lon, 0.0);
      s.track_origin = "事件";
      break;
    }
  }
  if (!origin) return s;

  const size_t max_points = std::max<size_t>(p.max_track_points, 2);
  for (const auto& name : names) {
    const auto& recs = in.sources.at(name);
    const size_t stride = std::max<size_t>(1, (recs.size() + max_points - 1) / max_points);
    Track track;
    track.source = name;
    size_t a = 0;
    while (a < recs.size()) {
      size_t b = a;
      while (b + 1 < recs.size() && recs[b + 1].stamp - recs[b].stamp <= p.track_gap_s) ++b;
      std::vector<TrackPoint> seg;
      for (size_t i = a; i <= b; ++i) {
        if (i != a && i != b && (i - a) % stride != 0) continue;
        const Eigen::Vector3d enu = origin->forward(recs[i].lat, recs[i].lon, recs[i].height);
        seg.push_back(TrackPoint{recs[i].stamp, enu.x(), enu.y(), recs[i].q});
      }
      track.segments.push_back(std::move(seg));
      a = b + 1;
    }
    s.tracks.push_back(std::move(track));
  }
  for (size_t i = 0; i < in.events.size(); ++i) {
    const auto& e = in.events[i];
    if (!e.pos) continue;
    const Eigen::Vector3d enu = origin->forward(e.pos->lat, e.pos->lon, 0.0);
    s.event_markers.push_back(EventMarker{static_cast<int>(i + 1), enu.x(), enu.y()});
  }
  return s;
}

}  // namespace gnss_core
```

说明：事件标记用 `alt=0` 投影、轨迹用各自高度投影，对 E/N 的影响在毫米级，可忽略；如果 `LlaToEnu::forward` 的高度差带来可测偏差（测试 `EventMarkers...` 只在"事件原点"场景下比较），在报告里说明即可，不改测试。

- [ ] **Step 4: GREEN + 变异 + 全量**

Run: `./build/gnss_core/test_report_stats` 全部 PASS。

变异（逐个做、逐个恢复）：
1. `divergence_stats` 里 `best_dt > tol` 改成 `best_dt >= tol` → `FarApartEpochsAreNotPaired...` 必须 FAIL（边界 0.5 s）。
2. 绝对基准改成"半径内每个控制点都算一次"（去掉 nearest，逐个 push）→ `AbsRefUsesFixedEpochs...` 必须 FAIL。
3. 轨迹抽稀里去掉 `i != b` 条件 → `LongTracksAreDownsampledKeepingSegmentEnds` 必须 FAIL。
4. `compute_report` 开头 `s.events = in.events;` 删掉 → `EmptyInputsProduceEmptyStats` 不会 FAIL，而 `EventsAreCopiedAndSummarisedByCode` 必须 FAIL。`EmptyInputs...` 另做变异：在 `origin` 为空时把 `s.track_origin = "事件"` 赋值挪到循环外（总是赋值）→ 它必须 FAIL。

全量测试 0 failures。

- [ ] **Step 5: 提交**

```bash
cd /home/steve/glim_ws/src/glim_ext
git add gnss_core/include/gnss_core/report_stats.hpp gnss_core/src/report_stats.cpp \
  gnss_core/test/test_report_stats.cpp gnss_core/CMakeLists.txt
git commit -m "feat(gnss_core): report statistics ported from rtk-monitor, per source

<RED(桩)与变异结果>

Co-Authored-By: <实际模型>"
```

---
### Task 4: 内嵌 SVG 图表

落实设计决定 3、11 的绘图部分。只生成字符串，不依赖任何图表库；SVG 内联在 HTML5 里，**不写 `xmlns`**（HTML5 不需要；写了会引入 `http://`，违反自包含约束）。

**Files:**
- Create: `glim_ext/gnss_core/include/gnss_core/report_svg.hpp`
- Create: `glim_ext/gnss_core/src/report_svg.cpp`
- Create: `glim_ext/gnss_core/test/test_report_svg.cpp`
- Modify: `glim_ext/gnss_core/CMakeLists.txt`

**Interfaces:**
- Produces（`namespace gnss_core`）：下面头文件里的全部声明。Task 5 使用 `html_escape`、`format_utc_short`、`svg_line_chart`、`svg_ratio_bar_chart`、`svg_track_map` 及其参数结构。

- [ ] **Step 1: 写失败测试**

Create `test/test_report_svg.cpp`：

```cpp
#include <gtest/gtest.h>

#include <limits>
#include <string>
#include <vector>

#include "gnss_core/report_svg.hpp"
using namespace gnss_core;

namespace {
const double T = 1789430400.0;   // 2026-09-15 00:00:00 UTC
const double kNaN = std::numeric_limits<double>::quiet_NaN();

size_t count(const std::string& hay, const std::string& needle) {
  size_t n = 0;
  for (size_t p = hay.find(needle); p != std::string::npos; p = hay.find(needle, p + needle.size())) ++n;
  return n;
}

// 输出里不能出现 printf 打出来的 nan / inf
void expect_clean(const std::string& svg) {
  EXPECT_EQ(svg.rfind("<svg", 0), 0u) << svg.substr(0, 80);
  EXPECT_EQ(svg.substr(svg.size() - 6), "</svg>");
  EXPECT_EQ(svg.find("nan"), std::string::npos) << svg;
  EXPECT_EQ(svg.find("inf"), std::string::npos) << svg;
  EXPECT_EQ(svg.find("http"), std::string::npos) << "不写 xmlns,报告必须自包含";
}

SvgLineChartOptions window_opts() {
  SvgLineChartOptions o;
  o.t0 = T;
  o.t1 = T + 3600.0;
  o.y_label = "偏差(m)";
  return o;
}
}  // namespace

TEST(ReportSvg, HtmlEscapeCoversMarkupAmpersandAndQuotes) {
  EXPECT_EQ(html_escape("<a href=\"x\">&'</a>"), "&lt;a href=&quot;x&quot;&gt;&amp;&#39;&lt;/a&gt;");
  EXPECT_EQ(html_escape("中文 plain"), "中文 plain");
}

TEST(ReportSvg, FormatUtcShortWithAndWithoutDate) {
  EXPECT_EQ(format_utc_short(T + 3723.9, false), "01:02");
  EXPECT_EQ(format_utc_short(T + 3723.9, true), "09/15 01:02");
}

TEST(ReportSvg, LineChartDrawsSeriesThresholdAndEscapedLabels) {
  auto o = window_opts();
  o.threshold = 0.2;
  o.threshold_label = "阈值 0.2 m";
  const auto svg = svg_line_chart({{"a<b", "#1f77b4", {{T + 10.0, 0.05}, {T + 20.0, 0.1}, {T + 30.0, 0.3}}}}, o);
  expect_clean(svg);
  EXPECT_EQ(count(svg, "<polyline"), 1u);
  EXPECT_NE(svg.find("stroke-dasharray"), std::string::npos) << "阈值画虚线";
  EXPECT_NE(svg.find("阈值 0.2 m"), std::string::npos);
  EXPECT_NE(svg.find("a&lt;b"), std::string::npos);
  EXPECT_EQ(svg.find("a<b"), std::string::npos);
  EXPECT_NE(svg.find("偏差(m)"), std::string::npos);
}

TEST(ReportSvg, LineChartSkipsNonFiniteAndOutOfWindowPointsAndCopesWithFlatData) {
  const auto o = window_opts();
  auto svg = svg_line_chart({{"s", "#000", {{T + 1.0, 0.0}, {T + 2.0, kNaN}, {T + 3.0, 0.0}}}}, o);
  expect_clean(svg);
  EXPECT_EQ(count(svg, "<polyline"), 1u) << "全为 0 的平线也要能画";

  svg = svg_line_chart({{"s", "#000", {{T + 1.0, 0.1}}}}, o);
  expect_clean(svg);
  EXPECT_EQ(count(svg, "<polyline"), 0u);
  EXPECT_EQ(count(svg, "<circle"), 1u) << "单点画圆点";

  EXPECT_NE(svg_line_chart({{"s", "#000", {{T - 100.0, 0.1}, {T + 7200.0, 0.2}}}}, o).find("无数据"), std::string::npos)
      << "窗口外的点不画";
  EXPECT_NE(svg_line_chart({{"s", "#000", {{T + 1.0, kNaN}}}}, o).find("无数据"), std::string::npos);
  EXPECT_NE(svg_line_chart({}, o).find("无数据"), std::string::npos);
  auto bad = o;
  bad.t1 = bad.t0;
  EXPECT_NE(svg_line_chart({{"s", "#000", {{T, 0.1}}}}, bad).find("无数据"), std::string::npos);
}

TEST(ReportSvg, RatioBarChartSkipsMissingValues) {
  const std::vector<SvgBarGroup> groups{{"00:00", {0.5, std::nullopt}}, {"01:00", {1.0, 0.25}}};
  auto svg = svg_ratio_bar_chart(groups, {"can", "rtkrcv"}, {"#1f77b4", "#ff7f0e"});
  expect_clean(svg);
  EXPECT_EQ(count(svg, "class=\"bar\""), 3u);
  EXPECT_NE(svg.find("01:00"), std::string::npos);
  EXPECT_NE(svg.find("rtkrcv"), std::string::npos);
  svg = svg_ratio_bar_chart({{"00:00", {std::nullopt}}}, {"can"}, {"#1f77b4"});
  EXPECT_NE(svg.find("无数据"), std::string::npos);
}

TEST(ReportSvg, TrackMapDrawsRunsAndMarkersAndCopesWithDegenerateInput) {
  const std::vector<SvgTrackLayer> layers{
      {"can", 3.0, "", {{"#3fb96c", {{0.0, 0.0}, {10.0, 5.0}, {20.0, 5.0}}}, {"#e0b23c", {{20.0, 5.0}, {30.0, 0.0}}}}}};
  auto svg = svg_track_map(layers, {{"1", 10.0, 5.0}, {"<x>", 30.0, 0.0}});
  expect_clean(svg);
  EXPECT_EQ(count(svg, "<polyline"), 2u);
  EXPECT_NE(svg.find(">1</text>"), std::string::npos);
  EXPECT_NE(svg.find("&lt;x&gt;"), std::string::npos);
  EXPECT_NE(svg.find(" m</text>"), std::string::npos) << "比例尺";

  svg = svg_track_map({{"rtkrcv", 2.0, "4 3", {{"#3fb96c", {{5.0, 5.0}}}}}}, {});
  expect_clean(svg);
  EXPECT_EQ(count(svg, "<circle"), 1u) << "单点轨迹画圆点";

  EXPECT_NE(svg_track_map({}, {}).find("无数据"), std::string::npos);
  EXPECT_NE(svg_track_map({{"x", 2.0, "", {{"#000", {{kNaN, 1.0}}}}}}, {}).find("无数据"), std::string::npos);
}
```

`CMakeLists.txt`：库源文件加 `src/report_svg.cpp`；测试注册加 `gnss_core_add_test(test_report_svg)`。

- [ ] **Step 2: 确认 RED（对桩）**

桩：头文件写全；`html_escape` 原样返回；`format_utc_short` 返回空串；三个图表函数返回 `"<svg></svg>"`。

Run: `colcon build --symlink-install --packages-select gnss_core && ./build/gnss_core/test_report_svg`
Expected: 6 个用例全部 FAIL。

- [ ] **Step 3: 实现**

Create `include/gnss_core/report_svg.hpp`：

```cpp
#pragma once
// 报告里的内嵌 SVG 图表(spec §3 F2,轮 4a)。纯字符串生成,不依赖图表库;
// 非有限数值与时间窗外的点一律跳过,输出里不会出现 nan / inf;不写 xmlns(HTML5 内联不需要)。
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace gnss_core {

// & < > " ' 转义,文本与属性值通用
std::string html_escape(const std::string& s);

// UTC 时刻短格式:"HH:MM";with_date 时 "MM/DD HH:MM"
std::string format_utc_short(double unix_s, bool with_date);

struct SvgLineSeries {
  std::string label;
  std::string color;                               // CSS 颜色
  std::vector<std::pair<double, double>> points;   // (UTC unix 秒, 值),时间升序
};

struct SvgLineChartOptions {
  std::string y_label;
  double t0 = 0.0, t1 = 0.0;          // 横轴 = 报告时间窗
  std::optional<double> threshold;    // 画一条红色虚线
  std::string threshold_label;
  int width = 760, height = 260;
};

std::string svg_line_chart(const std::vector<SvgLineSeries>& series, const SvgLineChartOptions& opt);

struct SvgBarGroup {
  std::string label;                            // 横轴分组标签
  std::vector<std::optional<double>> values;    // 与 series_labels 一一对应,取值 [0,1];空 = 无数据,不画
};

std::string svg_ratio_bar_chart(const std::vector<SvgBarGroup>& groups, const std::vector<std::string>& series_labels,
                                const std::vector<std::string>& colors, int width = 760, int height = 260);

struct SvgTrackRun {                             // 同一解质量、颜色相同的一段折线
  std::string color;
  std::vector<std::pair<double, double>> en;    // (东, 北) m
};

struct SvgTrackLayer {                           // 一个数据源
  std::string label;
  double stroke_width = 2.0;
  std::string dash;                              // stroke-dasharray;空 = 实线
  std::vector<SvgTrackRun> runs;
};

struct SvgMarker {                               // 事件位置标注
  std::string text;
  double e = 0.0, n = 0.0;
};

// 等比例的东/北平面图,北朝上,带比例尺与北向标;无底图
std::string svg_track_map(const std::vector<SvgTrackLayer>& layers, const std::vector<SvgMarker>& markers,
                          int width = 760, int height = 560);

}  // namespace gnss_core
```

Create `src/report_svg.cpp`：

```cpp
#include "gnss_core/report_svg.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <ctime>
#include <limits>

namespace gnss_core {

namespace {
std::string fmt(const char* f, double v) {
  char buf[64];
  std::snprintf(buf, sizeof(buf), f, v);
  return buf;
}

std::string px(double v) { return fmt("%.1f", v); }

std::string svg_open(int w, int h) {
  const std::string ws = std::to_string(w), hs = std::to_string(h);
  return "<svg class=\"chart\" viewBox=\"0 0 " + ws + " " + hs + "\" width=\"" + ws + "\" height=\"" + hs +
         "\" role=\"img\" font-family=\"sans-serif\" font-size=\"11\">";
}

std::string no_data(int w, int h) {
  return svg_open(w, h) + "<text x=\"" + px(w / 2.0) + "\" y=\"" + px(h / 2.0) +
         "\" text-anchor=\"middle\" fill=\"#888\">无数据</text></svg>";
}

// 1/2/5 × 10^k 中不小于 raw 的最小值
double nice_step(double raw) {
  if (!(raw > 0.0) || !std::isfinite(raw)) return 1.0;
  const double mag = std::pow(10.0, std::floor(std::log10(raw)));
  for (double m : {1.0, 2.0, 5.0, 10.0}) {
    if (m * mag >= raw) return m * mag;
  }
  return 10.0 * mag;
}

std::string legend(const std::vector<std::pair<std::string, std::string>>& items, double x, double y) {
  std::string o;
  for (size_t i = 0; i < items.size(); ++i) {
    const double yy = y + 14.0 * static_cast<double>(i);
    o += "<rect x=\"" + px(x) + "\" y=\"" + px(yy - 9.0) + "\" width=\"10\" height=\"10\" fill=\"" +
         html_escape(items[i].second) + "\"/>";
    o += "<text x=\"" + px(x + 14.0) + "\" y=\"" + px(yy) + "\">" + html_escape(items[i].first) + "</text>";
  }
  return o;
}
}  // namespace

std::string html_escape(const std::string& s) {
  std::string out;
  out.reserve(s.size());
  for (char c : s) {
    switch (c) {
      case '&': out += "&amp;"; break;
      case '<': out += "&lt;"; break;
      case '>': out += "&gt;"; break;
      case '"': out += "&quot;"; break;
      case '\'': out += "&#39;"; break;
      default: out += c; break;
    }
  }
  return out;
}

std::string format_utc_short(double unix_s, bool with_date) {
  const std::time_t tt = static_cast<std::time_t>(std::floor(unix_s));
  std::tm tm{};
  gmtime_r(&tt, &tm);
  char buf[32];
  if (with_date) {
    std::snprintf(buf, sizeof(buf), "%02d/%02d %02d:%02d", tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min);
  } else {
    std::snprintf(buf, sizeof(buf), "%02d:%02d", tm.tm_hour, tm.tm_min);
  }
  return buf;
}

std::string svg_line_chart(const std::vector<SvgLineSeries>& series, const SvgLineChartOptions& opt) {
  const int W = opt.width, H = opt.height;
  if (!(opt.t1 > opt.t0) || !std::isfinite(opt.t0) || !std::isfinite(opt.t1)) return no_data(W, H);
  const auto usable = [&](double t, double v) {
    return std::isfinite(t) && std::isfinite(v) && t >= opt.t0 && t <= opt.t1;
  };
  double ymin = 0.0, ymax = 0.0;
  bool any = false;
  for (const auto& s : series) {
    for (const auto& [t, v] : s.points) {
      if (!usable(t, v)) continue;
      any = true;
      ymin = std::min(ymin, v);
      ymax = std::max(ymax, v);
    }
  }
  if (!any) return no_data(W, H);
  const bool has_threshold = opt.threshold && std::isfinite(*opt.threshold);
  if (has_threshold) ymax = std::max(ymax, *opt.threshold);
  const double step = nice_step(ymax - ymin > 0.0 ? (ymax - ymin) / 5.0 : 0.02);
  ymax = std::ceil(ymax / step) * step;
  ymin = std::floor(ymin / step) * step;
  if (ymax <= ymin) ymax = ymin + step;

  const double L = 64.0, R = 16.0, T = 24.0, B = 36.0;
  const auto X = [&](double t) { return L + (t - opt.t0) / (opt.t1 - opt.t0) * (W - L - R); };
  const auto Y = [&](double v) { return H - B - (v - ymin) / (ymax - ymin) * (H - T - B); };

  std::string o = svg_open(W, H);
  for (int i = 0; ymin + step * i <= ymax + step * 0.5; ++i) {
    const double v = ymin + step * i;
    o += "<line x1=\"" + px(L) + "\" y1=\"" + px(Y(v)) + "\" x2=\"" + px(W - R) + "\" y2=\"" + px(Y(v)) +
         "\" stroke=\"#e0e0e0\"/>";
    o += "<text x=\"" + px(L - 6.0) + "\" y=\"" + px(Y(v) + 4.0) + "\" text-anchor=\"end\">" + fmt("%.3f", v) + "</text>";
  }
  const bool with_date = opt.t1 - opt.t0 > 86400.0;
  for (int i = 0; i <= 5; ++i) {
    const double t = opt.t0 + (opt.t1 - opt.t0) * i / 5.0;
    o += "<line x1=\"" + px(X(t)) + "\" y1=\"" + px(H - B) + "\" x2=\"" + px(X(t)) + "\" y2=\"" + px(H - B + 4.0) +
         "\" stroke=\"#444\"/>";
    o += "<text x=\"" + px(X(t)) + "\" y=\"" + px(H - B + 16.0) + "\" text-anchor=\"middle\">" +
         html_escape(format_utc_short(t, with_date)) + "</text>";
  }
  o += "<line x1=\"" + px(L) + "\" y1=\"" + px(H - B) + "\" x2=\"" + px(W - R) + "\" y2=\"" + px(H - B) + "\" stroke=\"#444\"/>";
  o += "<line x1=\"" + px(L) + "\" y1=\"" + px(T) + "\" x2=\"" + px(L) + "\" y2=\"" + px(H - B) + "\" stroke=\"#444\"/>";
  o += "<text x=\"" + px(L) + "\" y=\"" + px(T - 8.0) + "\">" + html_escape(opt.y_label) + "</text>";
  if (has_threshold) {
    const double y = Y(*opt.threshold);
    o += "<line x1=\"" + px(L) + "\" y1=\"" + px(y) + "\" x2=\"" + px(W - R) + "\" y2=\"" + px(y) +
         "\" stroke=\"#d62728\" stroke-dasharray=\"6 4\"/>";
    o += "<text x=\"" + px(W - R) + "\" y=\"" + px(y - 4.0) + "\" text-anchor=\"end\" fill=\"#d62728\">" +
         html_escape(opt.threshold_label) + "</text>";
  }
  std::vector<std::pair<std::string, std::string>> items;
  for (const auto& s : series) {
    std::string pts;
    size_t n = 0;
    double last_x = 0.0, last_y = 0.0;
    for (const auto& [t, v] : s.points) {
      if (!usable(t, v)) continue;
      last_x = X(t);
      last_y = Y(v);
      pts += (n++ ? " " : "") + px(last_x) + "," + px(last_y);
    }
    if (n == 0) continue;
    items.emplace_back(s.label, s.color);
    if (n == 1) {
      o += "<circle cx=\"" + px(last_x) + "\" cy=\"" + px(last_y) + "\" r=\"3\" fill=\"" + html_escape(s.color) + "\"/>";
    } else {
      o += "<polyline fill=\"none\" stroke=\"" + html_escape(s.color) + "\" stroke-width=\"1.5\" points=\"" + pts + "\"/>";
    }
  }
  o += legend(items, W - R - 150.0, T + 4.0);
  o += "</svg>";
  return o;
}

std::string svg_ratio_bar_chart(const std::vector<SvgBarGroup>& groups, const std::vector<std::string>& series_labels,
                                const std::vector<std::string>& colors, int width, int height) {
  const int W = width, H = height;
  bool any = false;
  for (const auto& g : groups) {
    for (const auto& v : g.values) any = any || (v && std::isfinite(*v));
  }
  if (groups.empty() || series_labels.empty() || !any) return no_data(W, H);

  const double L = 48.0, R = 16.0, T = 24.0, B = 40.0;
  const double plot_w = W - L - R, plot_h = H - T - B;
  const double group_w = plot_w / static_cast<double>(groups.size());
  const double bar_w = std::max(1.0, group_w * 0.8 / static_cast<double>(series_labels.size()));
  const auto color = [&](size_t i) { return i < colors.size() ? colors[i] : std::string("#888"); };

  std::string o = svg_open(W, H);
  for (int pct = 0; pct <= 100; pct += 25) {
    const double y = T + plot_h * (1.0 - pct / 100.0);
    o += "<line x1=\"" + px(L) + "\" y1=\"" + px(y) + "\" x2=\"" + px(W - R) + "\" y2=\"" + px(y) + "\" stroke=\"#e0e0e0\"/>";
    o += "<text x=\"" + px(L - 6.0) + "\" y=\"" + px(y + 4.0) + "\" text-anchor=\"end\">" + std::to_string(pct) + "%</text>";
  }
  const size_t label_every = std::max<size_t>(1, (groups.size() + 11) / 12);
  for (size_t gi = 0; gi < groups.size(); ++gi) {
    const double x0 = L + group_w * static_cast<double>(gi) + group_w * 0.1;
    for (size_t si = 0; si < series_labels.size(); ++si) {
      if (si >= groups[gi].values.size()) break;
      const auto& v = groups[gi].values[si];
      if (!v || !std::isfinite(*v)) continue;
      const double r = std::clamp(*v, 0.0, 1.0);
      const double h = r * plot_h;
      o += "<rect class=\"bar\" x=\"" + px(x0 + bar_w * static_cast<double>(si)) + "\" y=\"" + px(T + plot_h - h) +
           "\" width=\"" + px(bar_w) + "\" height=\"" + px(h) + "\" fill=\"" + html_escape(color(si)) + "\"><title>" +
           html_escape(series_labels[si]) + " " + fmt("%.1f%%", r * 100.0) + "</title></rect>";
    }
    if (gi % label_every == 0) {
      o += "<text x=\"" + px(L + group_w * (static_cast<double>(gi) + 0.5)) + "\" y=\"" + px(H - B + 16.0) +
           "\" text-anchor=\"middle\">" + html_escape(groups[gi].label) + "</text>";
    }
  }
  o += "<line x1=\"" + px(L) + "\" y1=\"" + px(T + plot_h) + "\" x2=\"" + px(W - R) + "\" y2=\"" + px(T + plot_h) +
       "\" stroke=\"#444\"/>";
  std::vector<std::pair<std::string, std::string>> items;
  for (size_t i = 0; i < series_labels.size(); ++i) items.emplace_back(series_labels[i], color(i));
  o += legend(items, W - R - 150.0, T + 4.0);
  o += "</svg>";
  return o;
}

std::string svg_track_map(const std::vector<SvgTrackLayer>& layers, const std::vector<SvgMarker>& markers,
                          int width, int height) {
  const int W = width, H = height;
  double min_e = std::numeric_limits<double>::infinity(), max_e = -min_e;
  double min_n = min_e, max_n = -min_e;
  bool any = false;
  const auto grow = [&](double e, double n) {
    if (!std::isfinite(e) || !std::isfinite(n)) return;
    any = true;
    min_e = std::min(min_e, e);
    max_e = std::max(max_e, e);
    min_n = std::min(min_n, n);
    max_n = std::max(max_n, n);
  };
  for (const auto& layer : layers) {
    for (const auto& run : layer.runs) {
      for (const auto& [e, n] : run.en) grow(e, n);
    }
  }
  for (const auto& m : markers) grow(m.e, m.n);
  if (!any) return no_data(W, H);

  const double M = 40.0;
  const double dx = std::max(max_e - min_e, 10.0), dy = std::max(max_n - min_n, 10.0);   // 视野至少 10 m
  const double scale = std::min((W - 2.0 * M) / dx, (H - 2.0 * M) / dy);
  const double ce = (min_e + max_e) / 2.0, cn = (min_n + max_n) / 2.0;
  const auto X = [&](double e) { return W / 2.0 + (e - ce) * scale; };
  const auto Y = [&](double n) { return H / 2.0 - (n - cn) * scale; };

  std::string o = svg_open(W, H);
  o += "<rect x=\"0.5\" y=\"0.5\" width=\"" + px(W - 1.0) + "\" height=\"" + px(H - 1.0) +
       "\" fill=\"#fafafa\" stroke=\"#ccc\"/>";
  for (const auto& layer : layers) {
    const std::string dash = layer.dash.empty() ? "" : " stroke-dasharray=\"" + html_escape(layer.dash) + "\"";
    for (const auto& run : layer.runs) {
      std::string pts;
      size_t n = 0;
      double last_x = 0.0, last_y = 0.0;
      for (const auto& [e, nn] : run.en) {
        if (!std::isfinite(e) || !std::isfinite(nn)) continue;
        last_x = X(e);
        last_y = Y(nn);
        pts += (n++ ? " " : "") + px(last_x) + "," + px(last_y);
      }
      if (n == 0) continue;
      if (n == 1) {
        o += "<circle cx=\"" + px(last_x) + "\" cy=\"" + px(last_y) + "\" r=\"" + px(std::max(2.0, layer.stroke_width)) +
             "\" fill=\"" + html_escape(run.color) + "\"/>";
      } else {
        o += "<polyline fill=\"none\" stroke=\"" + html_escape(run.color) + "\" stroke-width=\"" + px(layer.stroke_width) +
             "\" stroke-linejoin=\"round\"" + dash + " points=\"" + pts + "\"/>";
      }
    }
  }
  for (const auto& m : markers) {
    if (!std::isfinite(m.e) || !std::isfinite(m.n)) continue;
    o += "<g><circle cx=\"" + px(X(m.e)) + "\" cy=\"" + px(Y(m.n)) +
         "\" r=\"8\" fill=\"#fff\" stroke=\"#b00020\" stroke-width=\"1.5\"/>";
    o += "<text x=\"" + px(X(m.e)) + "\" y=\"" + px(Y(m.n) + 3.5) +
         "\" text-anchor=\"middle\" font-size=\"9\" fill=\"#b00020\">" + html_escape(m.text) + "</text></g>";
  }
  const double bar_m = nice_step((W - 2.0 * M) / scale / 5.0);
  const double bar_px = bar_m * scale;
  o += "<line x1=\"" + px(M) + "\" y1=\"" + px(H - 16.0) + "\" x2=\"" + px(M + bar_px) + "\" y2=\"" + px(H - 16.0) +
       "\" stroke=\"#222\" stroke-width=\"2\"/>";
  o += "<text x=\"" + px(M + bar_px + 6.0) + "\" y=\"" + px(H - 12.0) + "\">" + fmt("%g", bar_m) + " m</text>";
  o += "<text x=\"" + px(W - 24.0) + "\" y=\"22\" text-anchor=\"middle\" font-weight=\"bold\">N</text>";
  o += "<line x1=\"" + px(W - 24.0) + "\" y1=\"44\" x2=\"" + px(W - 24.0) + "\" y2=\"26\" stroke=\"#222\" stroke-width=\"2\"/>";
  o += "</svg>";
  return o;
}

}  // namespace gnss_core
```

- [ ] **Step 4: GREEN + 变异 + 全量**

Run: `./build/gnss_core/test_report_svg` 全部 PASS。

变异（逐个做、逐个恢复）：
1. `svg_line_chart` 的 `usable` 去掉时间窗判断 → `LineChartSkipsNonFiniteAndOutOfWindowPoints...` 必须 FAIL。
2. `svg_ratio_bar_chart` 里 `if (!v || !std::isfinite(*v)) continue;` 改成 `if (!std::isfinite(v.value_or(0.0))) continue;` → `RatioBarChartSkipsMissingValues` 必须 FAIL（缺值会被画成 0 高度的柱）。
3. `svg_track_map` 的 `std::max(max_e - min_e, 10.0)` 改成 `max_e - min_e` → 单点用例必须 FAIL（除零出 inf/nan）。

全量测试 0 failures。

- [ ] **Step 5: 提交**

```bash
cd /home/steve/glim_ws/src/glim_ext
git add gnss_core/include/gnss_core/report_svg.hpp gnss_core/src/report_svg.cpp \
  gnss_core/test/test_report_svg.cpp gnss_core/CMakeLists.txt
git commit -m "feat(gnss_core): inline SVG charts for the GNSS report

<RED(桩)与变异结果>

Co-Authored-By: <实际模型>"
```

---
### Task 5: HTML 报告渲染

落实设计决定 2、3、9（问题路段标注）、11（轨迹图样式）。

**Files:**
- Create: `glim_ext/gnss_core/include/gnss_core/report_html.hpp`
- Create: `glim_ext/gnss_core/src/report_html.cpp`
- Create: `glim_ext/gnss_core/test/test_report_html.cpp`
- Modify: `glim_ext/gnss_core/CMakeLists.txt`

**Interfaces:**
- Consumes: Task 3 的 `ReportStats` 及其成员类型、`compute_report`（测试里用）；Task 4 的 `html_escape`、`format_utc_short`、`svg_line_chart`、`svg_ratio_bar_chart`、`svg_track_map`；已有 `format_utc_timestamp`（diag_io.hpp）
- Produces（`namespace gnss_core`）:
  - `struct ReportMeta { std::string root; double generated_at; }`
  - `std::string quality_color(int q)`
  - `std::string render_report_html(const ReportStats& stats, const ReportMeta& meta)`
  - 报告里各节的锚点：`id="summary"`、`"fix"`、`"hourly"`、`"track"`、`"absref"`、`"divergence"`、`"base"`、`"events"`（Task 6 的测试会用）

- [ ] **Step 1: 写失败测试**

Create `test/test_report_html.cpp`：

```cpp
#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "gnss_core/report_html.hpp"
using namespace gnss_core;

namespace {
const double T = 1789430400.0;   // 2026-09-15 00:00:00 UTC
const double kLat = 44.5, kLon = 90.28;

double north(double m) { return kLat + m / 111132.0; }

PosRecord rec(double t, int q, double lat = kLat) {
  PosRecord r;
  r.stamp = t;
  r.q = q;
  r.lat = lat;
  r.lon = kLon;
  r.height = 600.0;
  return r;
}

ReportEvent event(const std::string& code, double t_open, std::optional<double> t_close, const std::string& reason,
                  const std::string& message = "结论") {
  ReportEvent e;
  e.code = code;
  e.level = "serious";
  e.message = message;
  e.t_open = t_open;
  e.t_close = t_close;
  e.close_reason = reason;
  e.pos = LatLon{kLat, kLon};
  return e;
}

ReportParams day_params() {
  ReportParams p;
  p.window = ReportWindow{T, T + 86400.0};
  return p;
}

ReportInputs rich_inputs() {
  ReportInputs in;
  for (int i = 0; i < 20; ++i) in.sources["can"].push_back(rec(T + i, i < 10 ? 1 : 2, north(i)));
  for (int i = 0; i < 20; ++i) in.sources["rtkrcv"].push_back(rec(T + i, 1, north(i + 0.3)));
  in.events = {event("corr_outage", T + 1.0, T + 11.0, "recovered"), event("low_sats", T + 5.0, T + 6.0, "shutdown"),
               event("multipath", T + 8.0, std::nullopt, "")};
  in.base_history = {BaseSample{T, Ecef{-2148744.0, 4426641.0, 4044655.0}},
                     BaseSample{T + 5.0, Ecef{-2148744.02, 4426641.0, 4044655.0}}};
  return in;
}

std::string section(const std::string& html, const std::string& id, const std::string& next_id) {
  const size_t a = html.find("id=\"" + id + "\"");
  const size_t b = html.find("id=\"" + next_id + "\"");
  if (a == std::string::npos || b == std::string::npos || b < a) return "";
  return html.substr(a, b - a);
}

size_t count(const std::string& hay, const std::string& needle) {
  size_t n = 0;
  for (size_t p = hay.find(needle); p != std::string::npos; p = hay.find(needle, p + needle.size())) ++n;
  return n;
}
}  // namespace

TEST(ReportHtml, QualityColorsMatchRtkMonitor) {
  EXPECT_EQ(quality_color(1), "#3fb96c");
  EXPECT_EQ(quality_color(2), "#e0b23c");
  EXPECT_EQ(quality_color(4), "#e05c4f");
  EXPECT_EQ(quality_color(5), "#e05c4f");
  EXPECT_EQ(quality_color(0), "#5a6472");
}

TEST(ReportHtml, ContainsEverySectionInOrderAndIsSelfContained) {
  auto p = day_params();
  p.control_points = {ControlPoint{"K1", kLat, kLon}};
  const auto html = render_report_html(compute_report(rich_inputs(), p), ReportMeta{"/data/gnss/pos", T + 90000.0});
  EXPECT_EQ(html.rfind("<!DOCTYPE html>", 0), 0u);
  size_t last = 0;
  for (const char* id : {"summary", "fix", "hourly", "track", "absref", "divergence", "base", "events"}) {
    const size_t at = html.find(std::string("id=\"") + id + "\"");
    ASSERT_NE(at, std::string::npos) << id;
    EXPECT_GT(at, last) << id << " 顺序不对";
    last = at;
  }
  EXPECT_NE(html.find("@page"), std::string::npos);
  EXPECT_NE(html.find("@media print"), std::string::npos);
  EXPECT_EQ(html.find("<script"), std::string::npos);
  EXPECT_EQ(html.find("<link"), std::string::npos);
  EXPECT_EQ(html.find("http://"), std::string::npos);
  EXPECT_EQ(html.find("https://"), std::string::npos);
  EXPECT_EQ(html.find("nan"), std::string::npos);
  EXPECT_GE(count(html, "<svg"), 4u) << "分小时柱状图、轨迹图、绝对基准曲线、基站曲线";
  EXPECT_NE(html.find("2026/09/15 00:00:00.000"), std::string::npos) << "时间窗按 UTC 显示";
  EXPECT_NE(section(html, "fix", "hourly").find("50.0%"), std::string::npos) << "can 固定率 10/20";
  EXPECT_NE(section(html, "fix", "hourly").find("100.0%"), std::string::npos) << "rtkrcv 固定率 20/20";
}

TEST(ReportHtml, EscapesEverythingThatComesFromFilesOrTheCommandLine) {
  ReportInputs in = rich_inputs();
  in.sources["evil<i>"] = {rec(T + 1.0, 1)};
  in.events.push_back(event("<b>code</b>", T + 2.0, T + 3.0, "recovered", "<script>alert(1)</script>"));
  auto p = day_params();
  p.control_points = {ControlPoint{"K&1", kLat, kLon}};
  const auto html = render_report_html(compute_report(in, p), ReportMeta{"/data/a&b", T});
  EXPECT_EQ(html.find("<script>alert"), std::string::npos);
  EXPECT_NE(html.find("&lt;script&gt;alert(1)&lt;/script&gt;"), std::string::npos);
  EXPECT_EQ(html.find("<b>code</b>"), std::string::npos);
  EXPECT_NE(html.find("&lt;b&gt;code&lt;/b&gt;"), std::string::npos);
  EXPECT_EQ(html.find("evil<i>"), std::string::npos);
  EXPECT_NE(html.find("evil&lt;i&gt;"), std::string::npos);
  EXPECT_NE(html.find("K&amp;1"), std::string::npos);
  EXPECT_NE(html.find("/data/a&amp;b"), std::string::npos);
}

TEST(ReportHtml, StatusLinesFollowTheThresholds) {
  ReportInputs in = rich_inputs();
  auto p = day_params();
  p.control_points = {ControlPoint{"K1", north(5.0), kLon}};   // rtkrcv 在 T+5 附近 0.3 m 处经过
  p.abs_ref_max_m = 0.2;
  p.base_shift_m = 0.01;
  auto html = render_report_html(compute_report(in, p), ReportMeta{"/r", T});
  EXPECT_NE(section(html, "absref", "divergence").find("全矿整体平移嫌疑"), std::string::npos);
  EXPECT_NE(section(html, "base", "events").find("基站坐标可能变动"), std::string::npos);

  p.abs_ref_max_m = 5.0;
  p.base_shift_m = 1.0;
  html = render_report_html(compute_report(in, p), ReportMeta{"/r", T});
  EXPECT_NE(section(html, "absref", "divergence").find("正常"), std::string::npos);
  EXPECT_EQ(section(html, "absref", "divergence").find("全矿整体平移嫌疑"), std::string::npos);
  EXPECT_NE(section(html, "base", "events").find("正常"), std::string::npos);
}

TEST(ReportHtml, EventTableShowsNumbersCloseReasonsAndUnclosedEvents) {
  const auto html = render_report_html(compute_report(rich_inputs(), day_params()), ReportMeta{"/r", T});
  const std::string ev = html.substr(html.find("id=\"events\""));
  EXPECT_NE(ev.find("恢复"), std::string::npos);
  EXPECT_NE(ev.find("停机关闭"), std::string::npos);
  EXPECT_NE(ev.find("未关闭"), std::string::npos);
  EXPECT_NE(ev.find("<td>3</td>"), std::string::npos) << "事件编号与轨迹图标注一致";
  EXPECT_NE(ev.find("44.500000, 90.280000"), std::string::npos) << "问题路段的经纬度";
  EXPECT_NE(ev.find("10.0"), std::string::npos) << "corr_outage 时长";
  const std::string track = section(html, "track", "absref");
  EXPECT_NE(track.find(">3</text>"), std::string::npos) << "轨迹图上标出事件编号";
}

TEST(ReportHtml, TrackRunsAreColouredByQuality) {
  const auto html = render_report_html(compute_report(rich_inputs(), day_params()), ReportMeta{"/r", T});
  const std::string track = section(html, "track", "absref");
  EXPECT_NE(track.find("stroke=\"#3fb96c\""), std::string::npos) << "固定解段";
  EXPECT_NE(track.find("stroke=\"#e0b23c\""), std::string::npos) << "can 后半段是浮点解";
}

TEST(ReportHtml, EmptyInputsRenderPlaceholdersInsteadOfFailing) {
  const auto html = render_report_html(compute_report(ReportInputs{}, day_params()), ReportMeta{"/r", T});
  for (const char* text : {"时间窗内没有任何 .pos 记录", "没有可绘制的位置", "未配置控制点", "无法比较",
                           "没有基站坐标记录", "时间窗内没有诊断事件"}) {
    EXPECT_NE(html.find(text), std::string::npos) << text;
  }
  EXPECT_EQ(html.find("nan"), std::string::npos);
}
```

说明：`StatusLinesFollowTheThresholds` 里 `rtkrcv` 第 5 条记录在 `north(5.3)`，离 `K1`（`north(5.0)`）约 0.3 m，在 3 m 半径内；`can` 与 `rtkrcv` 在 `K1` 附近的其他固定解也在半径内（最大约 3 m），所以最大偏差会超过 0.2 m 而不超过 5 m。

`CMakeLists.txt`：库源文件加 `src/report_html.cpp`；测试注册加 `gnss_core_add_test(test_report_html)`。

- [ ] **Step 2: 确认 RED（对桩）**

桩：头文件写全；`quality_color` 返回 `"#000"`；`render_report_html` 返回 `"<!DOCTYPE html><html></html>"`。

Run: `colcon build --symlink-install --packages-select gnss_core && ./build/gnss_core/test_report_html`
Expected: 6 个用例全部 FAIL。

- [ ] **Step 3: 实现**

Create `include/gnss_core/report_html.hpp`：

```cpp
#pragma once
// GNSS 报告的 HTML 渲染(spec §3 F2/F3,轮 4a):单个自包含文件——内联 CSS 与 SVG,不含脚本与外部资源;
// 浏览器打开后"打印 → 另存为 PDF"。所有来自文件或命令行的文本都经 html_escape。
#include <string>

#include "gnss_core/report_stats.hpp"

namespace gnss_core {

struct ReportMeta {
  std::string root;            // 数据根目录(显示用)
  double generated_at = 0.0;   // 生成时刻,UTC unix 秒
};

// RTKLIB Q → 轨迹颜色,与 rtk-monitor mapview.js 一致:固定绿、浮点黄、DGPS/单点红、其他灰
std::string quality_color(int q);

std::string render_report_html(const ReportStats& stats, const ReportMeta& meta);

}  // namespace gnss_core
```

Create `src/report_html.cpp`：

```cpp
#include "gnss_core/report_html.hpp"

#include <cstdio>
#include <map>
#include <optional>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "gnss_core/diag_io.hpp"
#include "gnss_core/report_svg.hpp"

namespace gnss_core {

namespace {
const char* const kCss = R"CSS(
body{font-family:"Noto Sans CJK SC","Source Han Sans SC","Microsoft YaHei","PingFang SC",sans-serif;color:#222;
  max-width:820px;margin:24px auto;padding:0 16px;font-size:14px;line-height:1.5}
h1{font-size:22px;margin:0 0 12px}
h2{font-size:17px;border-bottom:2px solid #444;padding-bottom:4px;margin-top:30px}
table{border-collapse:collapse;width:100%;margin:8px 0}
th,td{border:1px solid #bbb;padding:3px 6px;text-align:left;vertical-align:top}
th{background:#eee}
td.num{text-align:right;font-variant-numeric:tabular-nums}
tr.bad td{background:#fdecea}
.warn{color:#b00020;font-weight:bold}
.ok{color:#1b7a3a;font-weight:bold}
.note{color:#666;font-size:12px}
.legend span{display:inline-block;margin-right:14px;font-size:12px}
.swatch{display:inline-block;width:12px;height:12px;vertical-align:middle;margin-right:4px}
svg{max-width:100%;height:auto;display:block;margin:8px 0}
@page{size:A4;margin:14mm}
@media print{body{margin:0;max-width:none;font-size:11px}h2{break-after:avoid}svg,tr{break-inside:avoid}thead{display:table-header-group}}
)CSS";

const char* const kSourceColors[] = {"#1f77b4", "#ff7f0e", "#2ca02c", "#9467bd",
                                     "#8c564b", "#e377c2", "#7f7f7f", "#17becf"};

struct TrackStyle {
  double width;
  const char* dash;
  const char* desc;
};
const TrackStyle kTrackStyles[] = {{4.0, "", "粗实线"}, {3.0, "8 4", "虚线"}, {2.0, "", "细实线"}, {1.5, "2 3", "点线"}};

std::string fmt(const char* f, double v) {
  char buf[64];
  std::snprintf(buf, sizeof(buf), f, v);
  return buf;
}

std::string pct(const std::optional<double>& v) { return v ? fmt("%.1f%%", *v * 100.0) : "-"; }

std::string pct_of(int n, int total) { return total > 0 ? fmt("%.1f%%", 100.0 * n / total) : "-"; }

std::string metres(const std::optional<double>& v) { return v ? fmt("%.3f", *v) : "-"; }

std::string note(const std::string& text) { return "<p class=\"note\">" + text + "</p>"; }

// cells 必须已经转义
std::string table(const std::vector<std::string>& headers, const std::vector<std::vector<std::string>>& rows,
                  const std::vector<std::string>& row_classes = {}) {
  std::string o = "<table><thead><tr>";
  for (const auto& h : headers) o += "<th>" + h + "</th>";
  o += "</tr></thead><tbody>";
  for (size_t i = 0; i < rows.size(); ++i) {
    const std::string cls = i < row_classes.size() && !row_classes[i].empty() ? " class=\"" + row_classes[i] + "\"" : "";
    o += "<tr" + cls + ">";
    for (const auto& c : rows[i]) o += "<td>" + c + "</td>";
    o += "</tr>";
  }
  return o + "</tbody></table>";
}

std::string status(bool bad, const std::string& bad_text) {
  return bad ? "<span class=\"warn\">" + bad_text + "</span>" : "<span class=\"ok\">正常</span>";
}

bool multi_day(const ReportWindow& w) { return w.t1 - w.t0 > 86400.0; }

std::string section_summary(const ReportStats& s, const ReportMeta& m) {
  std::string sources;
  for (const auto& src : s.sources) {
    sources += (sources.empty() ? "" : "、") + html_escape(src.name) + "（" + std::to_string(src.epochs) + " 历元）";
  }
  std::string o = "<section id=\"summary\"><h1>GNSS 定位报告</h1>";
  o += table({"项目", "内容"},
             {{"时间窗", format_utc_timestamp(s.params.window.t0) + " – " + format_utc_timestamp(s.params.window.t1) + "（UTC，含起点不含终点）"},
              {"数据目录", html_escape(m.root)},
              {"生成时间", format_utc_timestamp(m.generated_at) + "（UTC）"},
              {"数据源", sources.empty() ? "无" : sources}});
  if (!s.warnings.empty()) {
    o += "<p class=\"warn\">读取数据时遇到的问题：</p><ul>";
    for (const auto& w : s.warnings) o += "<li>" + html_escape(w) + "</li>";
    o += "</ul>";
  }
  return o + "</section>";
}

std::string section_fix(const ReportStats& s) {
  std::string o = "<section id=\"fix\"><h2>1. 固定解可用率</h2>";
  if (s.sources.empty()) return o + note("时间窗内没有任何 .pos 记录。") + "</section>";
  std::vector<std::vector<std::string>> rows;
  for (const auto& src : s.sources) {
    rows.push_back({html_escape(src.name), std::to_string(src.epochs), "<b>" + pct(src.fix_ratio) + "</b>",
                    pct_of(src.counts.floating, src.epochs), pct_of(src.counts.dgps, src.epochs),
                    pct_of(src.counts.single, src.epochs), pct_of(src.counts.other, src.epochs)});
  }
  o += table({"数据源", "历元数", "固定率", "FLOAT", "DGPS", "SINGLE", "其他"}, rows);
  o += note("固定解按 RTKLIB Q=1 统计；百分比的分母是该源在时间窗内的全部历元。");
  return o + "</section>";
}

std::string section_hourly(const ReportStats& s) {
  std::string o = "<section id=\"hourly\"><h2>2. 分小时固定率</h2>";
  if (s.sources.empty()) return o + note("时间窗内没有任何 .pos 记录。") + "</section>";
  const bool with_date = multi_day(s.params.window);
  std::vector<std::string> labels, colors, headers{"小时（UTC）"};
  for (size_t i = 0; i < s.sources.size(); ++i) {
    labels.push_back(s.sources[i].name);
    colors.push_back(kSourceColors[i % 8]);
    headers.push_back(html_escape(s.sources[i].name) + " 固定率（历元）");
  }
  const size_t hours = s.sources.front().hourly.size();
  std::vector<SvgBarGroup> groups;
  std::vector<std::vector<std::string>> rows;
  for (size_t h = 0; h < hours; ++h) {
    const double t_start = s.sources.front().hourly[h].t_start;
    SvgBarGroup g;
    g.label = format_utc_short(t_start, with_date);
    std::vector<std::string> row{html_escape(g.label)};
    for (const auto& src : s.sources) {
      const HourBucket& b = src.hourly[h];
      g.values.push_back(b.fix_ratio);
      row.push_back(b.epochs > 0 ? pct(b.fix_ratio) + "（" + std::to_string(b.epochs) + "）" : "-");
    }
    groups.push_back(std::move(g));
    rows.push_back(std::move(row));
  }
  o += svg_ratio_bar_chart(groups, labels, colors);
  o += table(headers, rows);
  return o + "</section>";
}

std::string section_track(const ReportStats& s) {
  std::string o = "<section id=\"track\"><h2>3. 轨迹与问题路段</h2>";
  if (!s.track_origin) return o + note("时间窗内没有可绘制的位置。") + "</section>";
  std::vector<SvgTrackLayer> layers;
  std::string source_legend;
  for (size_t i = 0; i < s.tracks.size(); ++i) {
    const auto& track = s.tracks[i];
    const TrackStyle& style = kTrackStyles[i % 4];
    SvgTrackLayer layer;
    layer.label = track.source;
    layer.stroke_width = style.width;
    layer.dash = style.dash;
    for (const auto& seg : track.segments) {
      // 同一段内按解状态切成等色子段;切换点同时属于前后两段,折线不断开
      SvgTrackRun run;
      int run_q = seg.empty() ? 0 : seg.front().q;
      for (const auto& pt : seg) {
        if (pt.q != run_q && !run.en.empty()) {
          run.color = quality_color(run_q);
          run.en.emplace_back(pt.e, pt.n);
          layer.runs.push_back(std::move(run));
          run = SvgTrackRun{};
          run_q = pt.q;
        }
        run.en.emplace_back(pt.e, pt.n);
      }
      if (!run.en.empty()) {
        run.color = quality_color(run_q);
        layer.runs.push_back(std::move(run));
      }
    }
    layers.push_back(std::move(layer));
    source_legend += "<span>" + html_escape(track.source) + "：" + style.desc + "</span>";
  }
  std::vector<SvgMarker> markers;
  for (const auto& m : s.event_markers) markers.push_back(SvgMarker{std::to_string(m.index), m.e, m.n});
  o += svg_track_map(layers, markers);
  o += "<div class=\"legend\"><span><i class=\"swatch\" style=\"background:" + quality_color(1) + "\"></i>固定</span>"
       "<span><i class=\"swatch\" style=\"background:" + quality_color(2) + "\"></i>浮点</span>"
       "<span><i class=\"swatch\" style=\"background:" + quality_color(5) + "\"></i>DGPS / 单点</span>"
       "<span><i class=\"swatch\" style=\"background:" + quality_color(0) + "\"></i>其他</span>" +
       source_legend + "</div>";
  const std::string origin = *s.track_origin == "事件" ? "第一个带位置的事件"
                                                        : html_escape(*s.track_origin) + " 的第一条记录";
  o += note("无底图；坐标为以" + origin + "为原点的局部东/北（米），北朝上。红圈数字对应第 7 节事件表的编号，即问题路段位置。");
  return o + "</section>";
}

std::string section_absref(const ReportStats& s) {
  const auto& p = s.params;
  std::string o = "<section id=\"absref\"><h2>4. 绝对基准校验（控制点比对）</h2>";
  o += "<p>判定：固定解经过控制点 " + fmt("%.1f", p.abs_ref_radius_m) + " m 范围内时，与最近控制点的水平偏差；超过 " +
       fmt("%.3f", p.abs_ref_max_m) + " m 告警。</p>";
  if (p.control_points.empty()) {
    return o + note("未配置控制点（用 --control-point NAME,LAT,LON 指定），本节不做校验。") + "</section>";
  }
  std::string cps;
  for (const auto& cp : p.control_points) {
    cps += (cps.empty() ? "" : "、") + html_escape(cp.name) + "（" + fmt("%.8f", cp.lat) + ", " + fmt("%.8f", cp.lon) + "）";
  }
  o += note("控制点：" + cps);
  if (s.abs_ref.samples.empty()) {
    return o + note("时间窗内没有固定解经过任何控制点 " + fmt("%.1f", p.abs_ref_radius_m) + " m 范围。") + "</section>";
  }
  o += "<p>最大偏差 <b>" + metres(s.abs_ref.max_m) + " m</b>　状态：" +
       status(s.abs_ref.exceeded, "⚠ 超阈值（&gt; " + fmt("%.3f", p.abs_ref_max_m) + " m）——全矿整体平移嫌疑") + "</p>";

  struct Agg {
    int n = 0;
    double max = 0.0, sum = 0.0;
  };
  std::map<std::pair<std::string, std::string>, Agg> agg;
  std::map<std::string, SvgLineSeries> series;
  for (const auto& x : s.abs_ref.samples) {
    auto& a = agg[{x.source, x.control_point}];
    ++a.n;
    a.max = std::max(a.max, x.dev_m);
    a.sum += x.dev_m;
    series[x.source].points.emplace_back(x.t, x.dev_m);
  }
  std::vector<std::vector<std::string>> rows;
  std::vector<SvgLineSeries> chart;
  for (size_t i = 0; i < s.sources.size(); ++i) {
    const std::string& name = s.sources[i].name;
    for (const auto& [key, a] : agg) {
      if (key.first != name) continue;
      rows.push_back({html_escape(name), html_escape(key.second), std::to_string(a.n), fmt("%.3f", a.max),
                      fmt("%.3f", a.sum / a.n)});
    }
    auto it = series.find(name);
    if (it != series.end()) {
      it->second.label = name;
      it->second.color = kSourceColors[i % 8];
      chart.push_back(std::move(it->second));
    }
  }
  o += table({"数据源", "控制点", "经过历元数", "最大偏差（m）", "平均偏差（m）"}, rows);
  SvgLineChartOptions opt;
  opt.y_label = "与控制点的水平偏差（m）";
  opt.t0 = p.window.t0;
  opt.t1 = p.window.t1;
  opt.threshold = p.abs_ref_max_m;
  opt.threshold_label = "告警阈值 " + fmt("%.3f", p.abs_ref_max_m) + " m";
  o += svg_line_chart(chart, opt);
  return o + "</section>";
}

std::string section_divergence(const ReportStats& s) {
  std::string o = "<section id=\"divergence\"><h2>5. 610 与独立解偏差</h2>";
  o += "<p>对每个 rtkrcv 历元取时间最近的 610 历元，时间差 ≤ " + fmt("%.2f", s.params.pair_tol_s) +
       " s 才配对，统计两者的水平距离。</p>";
  if (s.divergence.empty()) {
    return o + note("缺少 rtkrcv.pos，或 can.pos 与 gpchc.pos 都不存在，无法比较。") + "</section>";
  }
  std::vector<std::vector<std::string>> rows;
  for (const auto& d : s.divergence) {
    if (d.n == 0) {
      rows.push_back({html_escape(d.device), html_escape(d.reference), "0", "无配对样本", "-"});
    } else {
      rows.push_back({html_escape(d.device), html_escape(d.reference), std::to_string(d.n), metres(d.max_m), metres(d.mean_m)});
    }
  }
  o += table({"610 数据源", "基准", "配对数", "最大（m）", "均值（m）"}, rows);
  return o + "</section>";
}

std::string section_base(const ReportStats& s) {
  std::string o = "<section id=\"base\"><h2>6. 基站坐标稳定性</h2>";
  if (s.base.series.empty()) return o + note("时间窗内没有基站坐标记录（base.pos）。") + "</section>";
  o += "<p>相对时间窗内第一条记录的最大偏移 <b>" + metres(s.base.max_m) + " m</b>　状态：" +
       status(s.base.exceeded, "⚠ 超过 " + fmt("%.3f", s.params.base_shift_m) + " m——基站坐标可能变动") + "</p>";
  SvgLineSeries series;
  series.label = "基站偏移";
  series.color = "#1f77b4";
  for (const auto& b : s.base.series) series.points.emplace_back(b.t, b.offset_m);
  SvgLineChartOptions opt;
  opt.y_label = "ECEF 偏移（m）";
  opt.t0 = s.params.window.t0;
  opt.t1 = s.params.window.t1;
  opt.threshold = s.params.base_shift_m;
  opt.threshold_label = "告警阈值 " + fmt("%.3f", s.params.base_shift_m) + " m";
  o += svg_line_chart({series}, opt);
  o += note("共 " + std::to_string(s.base.series.size()) + " 条记录（基站坐标变化超过 1 mm 才记一条）。");
  return o + "</section>";
}

std::string section_events(const ReportStats& s) {
  std::string o = "<section id=\"events\"><h2>7. 事件（" + std::to_string(s.events.size()) + " 条）</h2>";
  if (s.events.empty()) return o + note("时间窗内没有诊断事件。") + "</section>";
  std::vector<std::vector<std::string>> summary;
  for (const auto& x : s.event_summary) {
    summary.push_back({html_escape(x.code), std::to_string(x.count), fmt("%.1f", x.closed_duration_s), std::to_string(x.unclosed)});
  }
  o += table({"代码", "次数", "已关闭事件总时长（s）", "未关闭"}, summary);

  std::vector<std::vector<std::string>> rows;
  std::vector<std::string> classes;
  for (size_t i = 0; i < s.events.size(); ++i) {
    const auto& e = s.events[i];
    std::string peak;
    for (const auto& [k, v] : e.peak) peak += (peak.empty() ? "" : "; ") + html_escape(k) + "=" + fmt("%.3f", v);
    const std::string how = e.close_reason == "recovered" ? "恢复" : e.close_reason == "shutdown" ? "停机关闭" : "—";
    rows.push_back({std::to_string(i + 1), format_utc_timestamp(e.t_open),
                    e.t_close ? format_utc_timestamp(*e.t_close) : "未关闭",
                    e.t_close ? fmt("%.1f", *e.t_close - e.t_open) : "-", html_escape(e.level), html_escape(e.code),
                    e.pos ? fmt("%.6f", e.pos->lat) + ", " + fmt("%.6f", e.pos->lon) : "-", html_escape(e.message), how,
                    peak.empty() ? "-" : peak});
    classes.push_back(e.level == "serious" || e.level == "critical" ? "bad" : "");
  }
  o += table({"#", "开始（UTC）", "结束（UTC）", "时长（s）", "级别", "代码", "位置（纬, 经）", "结论", "关闭方式", "峰值"},
             rows, classes);
  o += note("“未关闭”表示数据里没有它的关闭记录：事件在时间窗结束时仍在进行，或诊断节点在关闭前中断。");
  return o + "</section>";
}
}  // namespace

std::string quality_color(int q) {
  switch (q) {
    case 1: return "#3fb96c";
    case 2: return "#e0b23c";
    case 4:
    case 5: return "#e05c4f";
    default: return "#5a6472";
  }
}

std::string render_report_html(const ReportStats& stats, const ReportMeta& meta) {
  std::string o = "<!DOCTYPE html>\n<html lang=\"zh-CN\"><head><meta charset=\"utf-8\"><title>GNSS 定位报告 " +
                  format_utc_timestamp(stats.params.window.t0) + "</title><style>" + kCss + "</style></head><body>";
  o += section_summary(stats, meta);
  o += section_fix(stats);
  o += section_hourly(stats);
  o += section_track(stats);
  o += section_absref(stats);
  o += section_divergence(stats);
  o += section_base(stats);
  o += section_events(stats);
  o += "<footer class=\"note\">由 gnss_core 的 gnss_report 生成。在浏览器中“打印 → 另存为 PDF”即得 PDF 报告。</footer>";
  o += "</body></html>\n";
  return o;
}

}  // namespace gnss_core
```

`report_html.cpp` 用到 `std::max`，include 区补 `<algorithm>`。

- [ ] **Step 4: GREEN + 变异 + 全量**

Run: `./build/gnss_core/test_report_html` 全部 PASS。

变异（逐个做、逐个恢复）：
1. `section_events` 里把结论 `html_escape(e.message)` 改成 `e.message` → `EscapesEverythingThatComesFromFiles...` 必须 FAIL。
2. `section_track` 里 `run.color = quality_color(run_q);` 两处都改成 `quality_color(1)` → `TrackRunsAreColouredByQuality` 必须 FAIL。
3. `status()` 的判断取反 → `StatusLinesFollowTheThresholds` 必须 FAIL。

全量测试 0 failures。

- [ ] **Step 5: 提交**

```bash
cd /home/steve/glim_ws/src/glim_ext
git add gnss_core/include/gnss_core/report_html.hpp gnss_core/src/report_html.cpp \
  gnss_core/test/test_report_html.cpp gnss_core/CMakeLists.txt
git commit -m "feat(gnss_core): render the GNSS report as a self-contained printable HTML

<RED(桩)与变异结果>

Co-Authored-By: <实际模型>"
```

---
### Task 6: 命令行 `gnss_report`

落实设计决定 7（控制点从命令行给）、10（时间窗）。

**Files:**
- Create: `glim_ext/gnss_core/include/gnss_core/report_cli.hpp`
- Create: `glim_ext/gnss_core/src/report_cli.cpp`
- Create: `glim_ext/gnss_core/test/test_report_cli.cpp`
- Create: `glim_ext/gnss_core/tools/gnss_report.cpp`
- Modify: `glim_ext/gnss_core/CMakeLists.txt`、`glim_ext/gnss_core/tools/README.md`

**Interfaces:**
- Consumes: Task 1 `parse_utc_date_time`；Task 2 `load_report_inputs`；Task 3 `ReportParams`、`compute_report`；Task 5 `ReportMeta`、`render_report_html`；已有 `parse_day_dir_date`
- Produces:
  - `int gnss_core::run_gnss_report(const std::vector<std::string>& args, std::ostream& out, std::ostream& err)`，`args` 不含程序名。返回 0 成功；1 参数错误（`err` 里是原因加用法）；2 根目录读不了或报告写不出。
  - 可执行文件 `gnss_report`，装到 `lib/gnss_core/`

- [ ] **Step 1: 写失败测试**

Create `test/test_report_cli.cpp`：

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
#include "gnss_core/report_cli.hpp"
using namespace gnss_core;
namespace fs = std::filesystem;

namespace {
const double T = 1789430400.0;   // 2026-09-15 00:00:00 UTC

class TempDir {
public:
  TempDir() {
    const char* base = std::getenv("TMPDIR");
    std::string tmpl = std::string(base ? base : "/tmp") + "/report_cli_XXXXXX";
    std::vector<char> buf(tmpl.begin(), tmpl.end());
    buf.push_back('\0');
    if (::mkdtemp(buf.data()) != nullptr) path_ = buf.data();
  }
  ~TempDir() {
    std::error_code ec;
    if (!path_.empty()) fs::remove_all(path_, ec);
  }
  const std::string& path() const { return path_; }

private:
  std::string path_;
};

// 测试里切换工作目录,析构时切回(同一进程内的其他用例依赖原工作目录)
class CwdGuard {
public:
  explicit CwdGuard(const std::string& dir) : old_(fs::current_path()) { fs::current_path(dir); }
  ~CwdGuard() {
    std::error_code ec;
    fs::current_path(old_, ec);
  }

private:
  fs::path old_;
};

struct Run {
  int code = -1;
  std::string out, err;
};

Run run(const std::vector<std::string>& args) {
  std::ostringstream out, err;
  Run r;
  r.code = run_gnss_report(args, out, err);
  r.out = out.str();
  r.err = err.str();
  return r;
}

std::string slurp(const std::string& path) {
  std::ifstream in(path, std::ios::binary);
  std::ostringstream ss;
  ss << in.rdbuf();
  return ss.str();
}

void make_day(const std::string& root) {
  fs::create_directories(root + "/20260915");
  std::vector<PosRecord> recs;
  for (int i = 0; i < 10; ++i) {
    PosRecord r;
    r.stamp = T + 3600.0 + i;
    r.q = i < 8 ? 1 : 2;
    r.lat = 44.5;
    r.lon = 90.28;
    r.height = 600.0;
    recs.push_back(r);
  }
  write_pos(root + "/20260915/can.pos", recs);
  EventTransition e;
  e.kind = EventKind::Open;
  e.t = e.t_open = T + 3601.0;
  e.code = "corr_outage";
  e.level = Level::Serious;
  e.message = "差分中断";
  std::ofstream(root + "/20260915/events.log") << events_log_header() << format_event_line(e) << "\n";
  std::ofstream(root + "/20260915/base.pos") << base_pos_header() << "2026/09/15 01:00:00.000  1.0\n";   // 半行
}
}  // namespace

TEST(ReportCli, HelpPrintsUsageAndExitsZero) {
  const auto r = run({"--help"});
  EXPECT_EQ(r.code, 0);
  EXPECT_NE(r.out.find("用法"), std::string::npos);
  EXPECT_NE(r.out.find("--control-point"), std::string::npos);
  EXPECT_TRUE(r.err.empty());
}

TEST(ReportCli, UsageErrorsExitOneWithAReasonAndTheUsage) {
  const std::vector<std::pair<std::vector<std::string>, std::string>> cases{
      {{}, "缺少 --root"},
      {{"--root"}, "缺少参数值"},
      {{"--root", "/x"}, "--day"},
      {{"--root", "/x", "--day", "20260915", "--from", "2026/09/15 00:00:00", "--to", "2026/09/15 01:00:00"}, "二选一"},
      {{"--root", "/x", "--day", "2026091"}, "YYYYMMDD"},
      {{"--root", "/x", "--from", "2026-09-15", "--to", "2026/09/15 00:00:00"}, "格式应为"},
      {{"--root", "/x", "--from", "2026/09/15 01:00:00", "--to", "2026/09/15 00:00:00"}, "晚于"},
      {{"--root", "/x", "--from", "2026/08/01 00:00:00", "--to", "2026/09/15 00:00:00"}, "31 天"},
      {{"--root", "/x", "--day", "20260915", "--control-point", "K1,91,90"}, "--control-point"},
      {{"--root", "/x", "--day", "20260915", "--control-point", "K1,44.5"}, "--control-point"},
      {{"--root", "/x", "--bogus"}, "不认识"},
      {{"--root", "/x", "--day", "20260915", "--abs-ref-max-m", "abc"}, "需要一个数字"},
      {{"--root", "/x", "--day", "20260915", "--pair-tol-s", "0"}, "大于 0"},
      {{"--root", "/x", "--day", "20260915", "--leap-seconds", "1.5"}, "--leap-seconds"},
  };
  for (const auto& [args, reason] : cases) {
    std::string joined;
    for (const auto& a : args) joined += a + " ";
    const auto r = run(args);
    EXPECT_EQ(r.code, 1) << joined;
    EXPECT_NE(r.err.find(reason), std::string::npos) << joined << "\n" << r.err;
    EXPECT_NE(r.err.find("用法"), std::string::npos) << joined;
    EXPECT_TRUE(r.out.empty()) << joined;
  }
}

TEST(ReportCli, WritesTheReportForADayAndPrintsASummaryAndWarnings) {
  TempDir dir;
  ASSERT_FALSE(dir.path().empty());
  make_day(dir.path() + "/root");
  const std::string out_path = dir.path() + "/r.html";
  const auto r = run({"--root", dir.path() + "/root", "--day", "20260915", "--out", out_path, "--control-point",
                      "K1,44.5,90.28"});
  ASSERT_EQ(r.code, 0) << r.err;
  EXPECT_NE(r.out.find("已生成 " + out_path), std::string::npos) << r.out;
  EXPECT_NE(r.out.find("can: 10 历元, 固定率 80.0%"), std::string::npos) << r.out;
  EXPECT_NE(r.out.find("事件 1 条"), std::string::npos) << r.out;
  EXPECT_NE(r.err.find("警告"), std::string::npos) << "base.pos 的半行要报出来\n" << r.err;
  const std::string html = slurp(out_path);
  EXPECT_EQ(html.rfind("<!DOCTYPE html>", 0), 0u);
  EXPECT_NE(html.find("id=\"events\""), std::string::npos);
  EXPECT_NE(html.find("corr_outage"), std::string::npos);
  EXPECT_NE(html.find("K1"), std::string::npos);
  EXPECT_NE(html.find("2026/09/16 00:00:00.000"), std::string::npos) << "--day 覆盖整个 UTC 自然日";
}

TEST(ReportCli, DefaultOutputNameEncodesTheWindowInTheCurrentDirectory) {
  TempDir dir;
  ASSERT_FALSE(dir.path().empty());
  make_day(dir.path() + "/root");
  {
    CwdGuard cwd(dir.path());
    const auto r = run({"--root", dir.path() + "/root", "--from", "2026/09/15 01:00:00", "--to", "2026/09/15 02:30:00"});
    ASSERT_EQ(r.code, 0) << r.err;
  }
  EXPECT_TRUE(fs::exists(dir.path() + "/gnss_report_20260915T010000_20260915T023000.html"));
}

TEST(ReportCli, UnreadableRootOrUnwritableOutputExitsTwo) {
  TempDir dir;
  ASSERT_FALSE(dir.path().empty());
  auto r = run({"--root", dir.path() + "/absent", "--day", "20260915", "--out", dir.path() + "/r.html"});
  EXPECT_EQ(r.code, 2);
  EXPECT_NE(r.err.find("absent"), std::string::npos) << r.err;
  EXPECT_FALSE(fs::exists(dir.path() + "/r.html"));

  make_day(dir.path() + "/root");
  r = run({"--root", dir.path() + "/root", "--day", "20260915", "--out", dir.path() + "/no/such/dir/r.html"});
  EXPECT_EQ(r.code, 2);
  EXPECT_NE(r.err.find("无法写出"), std::string::npos) << r.err;
}
```

`CMakeLists.txt`：
- 库源文件加 `src/report_cli.cpp`。
- `estimate_lever_arm` 的 `install` 之后加：

```cmake
# --- 报告(轮 4a,spec §3 F2/F3):按天或时间窗生成自包含 HTML,浏览器打印为 PDF ---
add_executable(gnss_report tools/gnss_report.cpp)
target_link_libraries(gnss_report gnss_core)
install(TARGETS gnss_report DESTINATION lib/${PROJECT_NAME})
```

- 测试注册加 `gnss_core_add_test(test_report_cli)`；紧接着加一条可执行文件级的冒烟测试：

```cmake
  # 可执行文件本身能起来(参数解析与报告生成由 test_report_cli 覆盖)
  add_test(NAME gnss_report_help COMMAND gnss_report --help)
```

- [ ] **Step 2: 确认 RED（对桩）**

桩：`report_cli.hpp` 写全；`run_gnss_report` 直接 `return 0;`；`tools/gnss_report.cpp` 按下面 Step 3 写好（它只转发）。

Run: `colcon build --symlink-install --packages-select gnss_core && ./build/gnss_core/test_report_cli`
Expected: 5 个用例全部 FAIL（`--help` 用例因 `out` 为空而 FAIL）。

- [ ] **Step 3: 实现**

Create `include/gnss_core/report_cli.hpp`：

```cpp
#pragma once
// gnss_report 命令行(spec §3 F2/F3,轮 4a)。参数解析与执行放在库里以便单测,tools/gnss_report.cpp 只是 main。
#include <ostream>
#include <string>
#include <vector>

namespace gnss_core {

// args 不含程序名。返回退出码:0 成功;1 参数错误(err 里是原因与用法);2 根目录读不了或报告写不出。
int run_gnss_report(const std::vector<std::string>& args, std::ostream& out, std::ostream& err);

}  // namespace gnss_core
```

Create `tools/gnss_report.cpp`：

```cpp
// gnss_report:按 UTC 自然日或时间窗,从 <root>/YYYYMMDD/ 的 .pos、events.log、base.pos 生成
// GNSS 定位报告(自包含 HTML,浏览器打印为 PDF)。用法见 gnss_report --help。spec §3 F2/F3。
#include <iostream>
#include <string>
#include <vector>

#include "gnss_core/report_cli.hpp"

int main(int argc, char** argv) {
  const std::vector<std::string> args(argv + 1, argv + argc);
  return gnss_core::run_gnss_report(args, std::cout, std::cerr);
}
```

Create `src/report_cli.cpp`：

```cpp
#include "gnss_core/report_cli.hpp"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <fstream>
#include <optional>
#include <stdexcept>

#include "gnss_core/pos_io.hpp"
#include "gnss_core/report_html.hpp"
#include "gnss_core/report_inputs.hpp"
#include "gnss_core/report_stats.hpp"
#include "gnss_core/retention.hpp"

namespace gnss_core {

namespace {
const char* const kUsage =
    "用法: gnss_report --root <目录> (--day YYYYMMDD | --from \"YYYY/MM/DD HH:MM:SS\" --to \"YYYY/MM/DD HH:MM:SS\") [选项]\n"
    "  --root DIR                  数据根目录,其下为 YYYYMMDD/ 日期目录(.pos、events.log、base.pos)\n"
    "  --day YYYYMMDD              报告一个 UTC 自然日\n"
    "  --from T --to T             报告 [from, to) 时间窗,UTC,最长 31 天\n"
    "  --out FILE                  输出 HTML 路径(默认 ./gnss_report_<开始>_<结束>.html)\n"
    "  --control-point N,LAT,LON   控制点(十进制度,可重复);不给则不做绝对基准校验\n"
    "  --abs-ref-radius-m M        经过控制点的判定半径,默认 3.0\n"
    "  --abs-ref-max-m M           绝对基准告警阈值,默认 0.2\n"
    "  --base-shift-m M            基站坐标变动告警阈值,默认 0.1\n"
    "  --pair-tol-s S              610 与独立解按时间配对的容差,默认 0.5\n"
    "  --leap-seconds N            .pos 为 GPST 时换算 UTC 的闰秒,默认 18\n"
    "  -h, --help                  显示本说明\n"
    "生成的 HTML 用浏览器打开,\"打印 → 另存为 PDF\" 即得 PDF 报告。\n";

class UsageError : public std::runtime_error {
public:
  using std::runtime_error::runtime_error;
};

bool full_double(const std::string& s, double& out) {
  if (s.empty()) return false;
  char* end = nullptr;
  out = std::strtod(s.c_str(), &end);
  return end == s.c_str() + s.size() && std::isfinite(out);
}

double parse_number(const std::string& flag, const std::string& v) {
  double x = 0.0;
  if (!full_double(v, x)) throw UsageError(flag + " 需要一个数字,收到 \"" + v + "\"");
  return x;
}

double parse_positive(const std::string& flag, const std::string& v) {
  const double x = parse_number(flag, v);
  if (!(x > 0.0)) throw UsageError(flag + " 必须大于 0,收到 \"" + v + "\"");
  return x;
}

double parse_time(const std::string& flag, const std::string& v) {
  const size_t sp = v.find(' ');
  double t = 0.0;
  if (sp == std::string::npos || !parse_utc_date_time(v.substr(0, sp), v.substr(sp + 1), t)) {
    throw UsageError(flag + " 的格式应为 \"YYYY/MM/DD HH:MM:SS\"(UTC),收到 \"" + v + "\"");
  }
  return t;
}

ControlPoint parse_control_point(const std::string& v) {
  const UsageError bad("--control-point 的格式应为 NAME,LAT,LON(十进制度),收到 \"" + v + "\"");
  const size_t a = v.find(',');
  const size_t b = a == std::string::npos ? std::string::npos : v.find(',', a + 1);
  if (a == std::string::npos || a == 0 || b == std::string::npos || v.find(',', b + 1) != std::string::npos) throw bad;
  ControlPoint cp;
  cp.name = v.substr(0, a);
  if (!full_double(v.substr(a + 1, b - a - 1), cp.lat) || !full_double(v.substr(b + 1), cp.lon) ||
      std::abs(cp.lat) > 90.0 || std::abs(cp.lon) > 180.0) {
    throw bad;
  }
  return cp;
}

std::string compact_utc(double t) {
  const std::time_t tt = static_cast<std::time_t>(std::floor(t));
  std::tm tm{};
  gmtime_r(&tt, &tm);
  char buf[32];
  std::snprintf(buf, sizeof(buf), "%04d%02d%02dT%02d%02d%02d", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
                tm.tm_hour, tm.tm_min, tm.tm_sec);
  return buf;
}

std::string pct(const std::optional<double>& v) {
  if (!v) return "-";
  char buf[32];
  std::snprintf(buf, sizeof(buf), "%.1f%%", *v * 100.0);
  return buf;
}
}  // namespace

int run_gnss_report(const std::vector<std::string>& args, std::ostream& out, std::ostream& err) {
  try {
    std::string root, day, from, to, out_path;
    bool have_from = false, have_to = false;
    ReportParams params;
    PosReadOptions pos_options;
    for (size_t i = 0; i < args.size(); ++i) {
      const std::string& a = args[i];
      const auto value = [&]() -> const std::string& {
        if (i + 1 >= args.size()) throw UsageError(a + " 缺少参数值");
        return args[++i];
      };
      if (a == "-h" || a == "--help") {
        out << kUsage;
        return 0;
      } else if (a == "--root") {
        root = value();
      } else if (a == "--day") {
        day = value();
      } else if (a == "--from") {
        from = value();
        have_from = true;
      } else if (a == "--to") {
        to = value();
        have_to = true;
      } else if (a == "--out") {
        out_path = value();
      } else if (a == "--control-point") {
        params.control_points.push_back(parse_control_point(value()));
      } else if (a == "--abs-ref-radius-m") {
        params.abs_ref_radius_m = parse_positive(a, value());
      } else if (a == "--abs-ref-max-m") {
        params.abs_ref_max_m = parse_positive(a, value());
      } else if (a == "--base-shift-m") {
        params.base_shift_m = parse_positive(a, value());
      } else if (a == "--pair-tol-s") {
        params.pair_tol_s = parse_positive(a, value());
      } else if (a == "--leap-seconds") {
        const std::string& v = value();
        const double x = parse_number(a, v);
        if (x != std::floor(x) || x < 0.0 || x > 100.0) throw UsageError("--leap-seconds 必须是 0 到 100 的整数,收到 \"" + v + "\"");
        pos_options.leap_seconds = static_cast<int>(x);
      } else {
        throw UsageError("不认识的参数: " + a);
      }
    }
    if (root.empty()) throw UsageError("缺少 --root");
    if (!day.empty() && (have_from || have_to)) throw UsageError("--day 与 --from/--to 只能二选一");
    if (day.empty() && !(have_from && have_to)) throw UsageError("需要 --day,或者同时给 --from 与 --to");
    if (!day.empty()) {
      if (!parse_day_dir_date(day) ||
          !parse_utc_date_time(day.substr(0, 4) + "/" + day.substr(4, 2) + "/" + day.substr(6, 2), "00:00:00",
                               params.window.t0)) {
        throw UsageError("--day 的格式应为 YYYYMMDD,收到 \"" + day + "\"");
      }
      params.window.t1 = params.window.t0 + 86400.0;
    } else {
      params.window.t0 = parse_time("--from", from);
      params.window.t1 = parse_time("--to", to);
      if (!(params.window.t1 > params.window.t0)) throw UsageError("--to 必须晚于 --from");
      if (params.window.t1 - params.window.t0 > 31.0 * 86400.0) throw UsageError("时间窗最长 31 天");
    }
    if (out_path.empty()) {
      out_path = "gnss_report_" + compact_utc(params.window.t0) + "_" + compact_utc(params.window.t1) + ".html";
    }

    ReportInputs inputs;
    try {
      inputs = load_report_inputs(root, params.window, pos_options);
    } catch (const std::exception& e) {
      err << "gnss_report: " << e.what() << "\n";
      return 2;
    }
    const ReportStats stats = compute_report(inputs, params);
    ReportMeta meta;
    meta.root = root;
    meta.generated_at = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
    const std::string html = render_report_html(stats, meta);

    std::ofstream file(out_path, std::ios::binary | std::ios::trunc);
    if (file) {
      file << html;
      file.flush();
    }
    if (!file) {
      err << "gnss_report: 无法写出 " << out_path << "\n";
      return 2;
    }
    for (const auto& w : stats.warnings) err << "gnss_report: 警告: " << w << "\n";
    out << "已生成 " << out_path << "\n";
    for (const auto& s : stats.sources) {
      out << "  " << s.name << ": " << s.epochs << " 历元, 固定率 " << pct(s.fix_ratio) << "\n";
    }
    out << "  事件 " << stats.events.size() << " 条\n";
    return 0;
  } catch (const UsageError& e) {
    err << "gnss_report: " << e.what() << "\n\n" << kUsage;
    return 1;
  }
}

}  // namespace gnss_core
```

说明：用法文本里的标点用半角，避免终端宽度计算问题；报告 HTML 里仍用中文全角标点。

`tools/README.md`：顶部表格末尾加一行

```markdown
| `gnss_report.cpp` | `gnss_report --root <目录> --day YYYYMMDD [--control-point N,LAT,LON]…`:按天或时间窗生成 GNSS 定位报告(自包含 HTML,浏览器打印为 PDF);spec §3 F2/F3,轮 4a |
```

并在文件末尾加一节：

~~~markdown
## 5. 定位报告(`gnss_report`,spec §3 F2/F3)

读取 `pos_writer` / `gnss_diag_node` 写出的 `<root>/YYYYMMDD/{*.pos,events.log,base.pos}`,生成一个自包含 HTML
(内联 CSS 与 SVG,离线可看),浏览器打开后"打印 → 另存为 PDF"。

```bash
gnss_report --root /data/gnss/pos --day 20260915 --control-point K1,44.50123456,90.28765432
gnss_report --root /data/gnss/pos --from "2026/09/15 08:00:00" --to "2026/09/15 18:00:00" --out shift.html
```

报告内容:固定解可用率(每个数据源一行)、分小时固定率、轨迹与问题路段(按解状态着色,标出事件位置)、
绝对基准校验(需要 `--control-point`)、610(can/gpchc)与 rtkrcv 的偏差、基站坐标稳定性、事件汇总与明细。
口径移植自 rtk-monitor 的 `report.py`,差异见 `glim_underground/docs/gnss/plans/2026-09-16-round4a-report-tool.md`「设计决定」。
读不了的文件与解析不了的行会在终端打"警告",并列在报告开头,不会中断生成。
~~~

（上面用 `~~~` 包住只是为了在本计划里显示；写进 README 时去掉外层 `~~~markdown` / `~~~` 两行。）

- [ ] **Step 4: GREEN + 变异 + 全量 + 真实数据冒烟**

Run: `./build/gnss_core/test_report_cli` 全部 PASS；`./build/gnss_core/gnss_report --help` 退出码 0。

变异（逐个做、逐个恢复）：
1. 去掉 `--day` 与 `--from/--to` 的互斥检查 → `UsageErrorsExitOne...` 必须 FAIL。
2. 写文件后不检查 `!file`（直接往下走）→ `UnreadableRootOrUnwritableOutputExitsTwo` 必须 FAIL。
3. `params.window.t1 = params.window.t0 + 86400.0;` 改成 `+ 3600.0` → `WritesTheReportForADay...` 必须 FAIL。

全量测试 0 failures（`colcon test-result --all` 里应多出 `gnss_report_help`）。

冒烟：若开发机上有轮 3b 冒烟或实际运行留下的 `.pos`/`events.log`（例如 `/data/gnss/pos` 可读），对最近有数据的一天跑一次 `gnss_report --root <那个目录> --day <YYYYMMDD> --out $TMPDIR/real.html`，记录终端输出。没有可用数据时，用 `make_day` 同样的方式在临时目录造一天数据跑一次，并记录"无真实数据"。**不要往 `/data` 写任何东西。**

- [ ] **Step 5: 提交**

```bash
cd /home/steve/glim_ws/src/glim_ext
git add gnss_core/include/gnss_core/report_cli.hpp gnss_core/src/report_cli.cpp gnss_core/test/test_report_cli.cpp \
  gnss_core/tools/gnss_report.cpp gnss_core/tools/README.md gnss_core/CMakeLists.txt
git commit -m "feat(gnss_core): gnss_report command-line tool

<RED(桩)、变异、冒烟结果>

Co-Authored-By: <实际模型>"
```

---

## 完成标准

- `glim_ext` 分支 `feat/round4a-report-tool` 上 6 个 Task 的提交齐全，未 push。
- `colcon test-result --all`：0 errors、0 failures；新增 `test_diag_io` 用例、`test_report_inputs`、`test_report_stats`、`test_report_svg`、`test_report_html`、`test_report_cli` 与 `gnss_report_help`。
- 每个 Task 的 commit message 里有 RED（桩或旧代码）与变异证据。
- 用 `gnss_report` 实际生成过一份 HTML，浏览器打开各节齐全、打印预览为 A4。
