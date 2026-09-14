# 轮 2 加固(RTKLIB-EX 2.5.1 实测之后)Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 修掉 2026-09-14 用真实 RTKLIB-EX 2.5.1 联调时发现的"节点在车上一条解都出不来"的两个缺陷,补上让它们不再静默的日志与测试,并清掉 `.pos` 数据链路上残留的静默部分读取。

**Architecture:** 进程监管在父进程里按 `PATH` 解析可执行文件,并通过回调把每一次派生/退出(含退出码、信号、存活时长、下次等待)交给节点打日志;conf 渲染新增基准站坐标来源与北斗/GLONASS 模糊度固定三项,并对所有枚举取值做 RTKLIB-EX 2.5.1 白名单校验;节点级行为用"gtest 起子进程"的方式第一次被测试覆盖,另加一条用真实 rtkrcv + libfaketime 回放双站 RTCM3 的端到端回归(环境不具备时跳过)。`gnss_core` 侧让 `PosWriter::open()` 的去重扫描校验字节数、`read_glim_traj` 检查 `badbit`。

**Tech Stack:** C++17、ROS 2 Humble(rclcpp)、ament_cmake + gtest、RTKLIB-EX 2.5.1(`/usr/local/bin/rtkrcv`)、libfaketime。

**Spec:**
- `docs/gnss/specs/2026-09-03-gnss-glim-modules-design.md`(§5 `rtkrcv_node`/`.pos`,§13 现场待确认)
- `docs/gnss/specs/2026-09-14-round2-rest-and-orphan-followups.md`(本计划消化其中 A.1、A.2、B.1、B.3 的一部分、D 节)
- 下面「背景:2026-09-14 实测结论」一节是本计划的直接依据,执行者必须先读

## Global Constraints

- 代码仓库:`/home/steve/glim_ws/src/glim_ext`(独立 git)。开工前执行 `git ls-remote origin`:若 `feat/round2-rest-and-orphan` 已合入 `origin/master`,从更新后的 `master` 开 `feat/round2-hardening`;否则从 `feat/round2-rest-and-orphan`(`8661bf5`)开 `feat/round2-hardening`。以 `git ls-remote` 为准,不信本地 ref(仓库有外部 git 自动同步)。
- **绝不 `git push`**(本环境没有凭据,推送与建 PR 由维护者执行)。
- 每个 commit message 以 `Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>` 结尾。
- `glim_underground` 仓库里 `include/glim/util/time_keeper.hpp`、`src/glim/util/time_keeper.cpp` 是维护者未提交的工作,**绝不触碰、绝不暂存**;在该仓库提交文档时只 `git add` 明确的文档路径。
- `launch/gnss_bringup.launch.py` 的 `enable_rtkrcv` 默认值保持 `"false"`(维护者的决定)。
- `gnss_core` 保持不依赖 ROS。
- 注释用中文、与周边代码的注释密度与语气一致;标识符用英文。
- ROS 包的测试一律用 gtest。**不要用 `ament_add_pytest_test` / `launch_testing`**:开发机 `~/.local` 里的 pytest 9.1.1 与 Humble 的 launch_testing 插件不兼容,必然 `PluginValidationError`。
- **诚实测试**:每个新测试在实现之前必须先亲眼看到失败(RED)——对旧代码失败,或者对一个写明的变异失败;把 RED 的证据写进 commit message。以前三次出现过"错误分支被执行、但错误本身从没发生"的假测试,评审会专门查这一点。
- 测试产生的文件一律放在 `mkdtemp` 建出的临时目录(基于 `$TMPDIR`,缺省 `/tmp`),测试结束删除;起 ROS 节点的测试必须给子进程设置独立的 `ROS_DOMAIN_ID` 与指向临时目录的 `ROS_LOG_DIR`。
- 构建:`cd /home/steve/glim_ws && colcon build --symlink-install --packages-select gnss_core gnss_bringup`
- 全量测试:`cd /home/steve/glim_ws && colcon test --packages-select gnss_core gnss_bringup && colcon test-result --all`
- 单个 gtest 快速迭代:`cd /home/steve/glim_ws && source install/setup.bash && ./build/gnss_bringup/<test_binary> --gtest_filter='<Suite>.<Name>'`(`gnss_core` 的在 `./build/gnss_core/`)
- 基线:`colcon test-result --all` = **340 tests, 0 errors, 0 failures, 1 skipped**。每个 Task 结束时 0 failures。
- 目标 RTKLIB:**RTKLIB-EX 2.5.1**,已装在 `/usr/local/bin/rtkrcv`(`rtkrcv --version` 输出 `rtkrcv RTKLIB EX 2.5.1`);libfaketime:`/usr/lib/x86_64-linux-gnu/faketime/libfaketimeMT.so.1`;RTKLIB 源码树:`/home/steve/Documents/GitHub/gnss-alg/RTKLIB-2.5.1`。

## 背景:2026-09-14 实测结论

用 RTKLIB-EX 2.5.1 + 真实节点联调(测试脚本与数据不在仓库里,结论如下):

1. **`binary: "rtkrcv"` 永远起不来。** `ProcessSupervisor` 用 `execv()`,不查 `PATH`;子进程已 `chdir(run_dir)`,于是在 `run_dir` 下找 `rtkrcv`,失败后 `_exit(127)`,监管线程按崩溃循环无限重启。节点只在第一次打一行 `rtkrcv spawned pid`,之后完全沉默(实测 20 s 内重启 6 次,节点日志 0 行)。
2. **conf 缺 `ant2-postype`,RTK 一条解都没有。** 2.5.1 默认 `ant2-postype=llh`、坐标 `0,0,0`。真实双站回放:现状 conf → 0 条 `RtkFix`、0 字节 `.stat`;追加 `ant2-postype =rtcm` → 101 条 `RtkFix`、151 KB `.stat`,`.pos` 正常写出。
3. **非法 conf 值不报错。** rtkrcv 对未知键静默忽略;对非法值只打一行 `invalid option value`,**回落到默认值继续运行**(例:`pos2-armode =continuouss` → `fix-and-hold`)。现有单测里用的 `obs_format = "novatel"` 在 RTKLIB 里就不是合法取值(应为 `oem4`)。
4. 2.5.1 其余默认值:`pos2-bdsarmode=off`、`pos2-gloarmode=fix-and-hold`、`pos2-armode=fix-and-hold`(我们 conf 显式写了 `continuous`)。
5. 验证通过:13 个 conf 键全部生效;`-s -nc -r 2 -o <conf>` 可无终端常驻、SIGTERM rc=0;两路上行按字节送达;`.stat` 命名 `rtkrcv_%Y%m%d%h%M.stat`;节点 SIGINT 约 160 ms 退出无残留;101 条 `RtkFix` 与 rtkrcv 原始解算行逐字段一致(经纬高、Q→quality、NEU→ENU sigma、卫星数、龄期)。
6. 回放数据只得到浮点解(转换 RTCM 时丢了锁定信息;原始 RINEX 后处理可固定),**固定率不在本计划验证范围内**。
7. 无解算输出时 `sol stream` 每个 `sol_idle_timeout_s` 打一对 `disconnected idle timeout` / `connected` 的 INFO,隧道里会刷一整夜。
8. 老版 RTCM(2012 年 GMSD7)在 2026 年回放时,rtkrcv 用系统时间补 RTCM 周数,星历对不上、0 解——回放历史数据必须用 libfaketime 把 rtkrcv 的时钟拨到数据所在的那一周,并且用多线程版 `libfaketimeMT.so.1`,用 `env ... exec` 而不是 `faketime` 命令(后者 fork,SIGTERM 到不了 rtkrcv)。

## File Structure

`gnss_core`(Task 1)
- Modify `gnss_core/include/gnss_core/pos_io.hpp` — 新增 `read_pos(std::istream&, ...)` 重载
- Modify `gnss_core/src/pos_io.cpp` — 读循环抽成带字节计数的内部函数;`PosWriter::open()` 校验去重扫描字节数;新增注入点 `read_pos_premature_eof`
- Modify `gnss_core/include/gnss_core/pos_io_test_hooks.hpp` — 注释里登记新注入点
- Modify `gnss_core/include/gnss_core/synth.hpp`、`gnss_core/src/synth.cpp` — `read_glim_traj(std::istream&)` 重载 + `badbit` 检查
- Test `gnss_core/test/test_pos_io.cpp`、`gnss_core/test/test_synth.cpp`

`gnss_bringup` 进程监管(Task 2)
- Create `gnss_bringup/include/gnss_bringup/executable_lookup.hpp`、`gnss_bringup/src/executable_lookup.cpp` — `resolve_executable()` 纯函数
- Modify `gnss_bringup/include/gnss_bringup/process_supervisor.hpp`、`gnss_bringup/src/process_supervisor.cpp` — 父进程解析可执行文件;`on_spawn`/`on_exit` 回调;`start_failure_count()`
- Test `gnss_bringup/test/test_executable_lookup.cpp`(新)、`gnss_bringup/test/test_process_supervisor.cpp`

conf 渲染(Task 3)
- Modify `gnss_bringup/include/gnss_bringup/rtkrcv_conf.hpp`、`gnss_bringup/src/rtkrcv_conf.cpp`
- Test `gnss_bringup/test/test_rtkrcv_conf.cpp`

节点接线(Task 4)
- Create `gnss_bringup/include/gnss_bringup/sol_stream_log_gate.hpp` — sol 流状态日志去重闸门(纯逻辑)
- Modify `gnss_bringup/src/rtkrcv_node.cpp` — 新参数、启动前可执行文件检查、监管回调日志、删掉 pid 日志线程、日志闸门
- Modify `gnss_bringup/config/gnss_bringup.yaml`、`gnss_bringup/README.md`(新增「安装 RTKLIB-EX 2.5.1」、参数说明)
- Test `gnss_bringup/test/test_sol_stream_log_gate.cpp`(新)

节点级测试(Task 5)
- Create `gnss_bringup/test/node_process_harness.hpp` — 把节点当子进程起、抓日志、等退出(Task 6 复用)
- Create `gnss_bringup/test/test_rtkrcv_node_process.cpp`

真实 rtkrcv 回归(Task 6)
- Create `gnss_bringup/test/rtkrcv_faketime.sh`、`gnss_bringup/test/test_rtkrcv_real_binary.cpp`
- Create `gnss_bringup/test/data/rtcm_20050402_0759_rover.rtcm3`、`gnss_bringup/test/data/rtcm_20050402_3040_base.rtcm3`、`gnss_bringup/test/data/rnx2rtcm.c`(不参与构建)、`gnss_bringup/test/data/README.md`
- Modify `gnss_bringup/README.md`(「未验证项」改写为「已验证 / 未验证项」)

文档笔误与遗留清单(Task 7)
- Modify `gnss_bringup/scripts/record_gnss.sh`、`gnss_bringup/config/gnss_bringup.yaml`、`gnss_bringup/README.md`
- Modify `glim_underground/docs/gnss/specs/2026-09-14-round2-rest-and-orphan-followups.md`

依赖顺序:Task 1 独立;Task 2、3 → Task 4 → Task 5 → Task 6 → Task 7。

---

### Task 1: `gnss_core` 的静默部分读取 fail closed

**Files:**
- Modify: `gnss_core/include/gnss_core/pos_io.hpp`(`read_pos` 声明附近,约第 43 行)
- Modify: `gnss_core/src/pos_io.cpp`(注入点约第 117-124 行;`read_pos` 约第 284-322 行;`PosWriter::open` 约第 382-465 行)
- Modify: `gnss_core/include/gnss_core/pos_io_test_hooks.hpp`
- Modify: `gnss_core/include/gnss_core/synth.hpp:21`、`gnss_core/src/synth.cpp:15-34`
- Test: `gnss_core/test/test_pos_io.cpp`、`gnss_core/test/test_synth.cpp`

**Interfaces:**
- Consumes: 现有 `gnss_core::testing::set_trailing_line_read_failure_injector(bool(*)(const char* step))`
- Produces:
  - `std::vector<PosRecord> gnss_core::read_pos(std::istream& in, const PosReadOptions& opt = {});` —— 读错误(`badbit`)抛 `std::runtime_error`
  - `std::vector<TrajPose> gnss_core::read_glim_traj(std::istream& in);` —— 读错误抛 `std::runtime_error`
  - 注入点步骤名 `"read_pos_premature_eof"`:在读循环里模拟"`read()` 提前返回 0"——循环正常结束、不设 `badbit`

背景:`PosWriter::open()` 扫描已有文件建去重键。在 FUSE/NFS 或并发截断下 `read()` 可能在真实文件末尾之前返回 0,流状态与正常 EOF 无法区分,`read_pos` 返回部分结果、`open()` 返回 true、已写过的时间戳被再次追加(评审实测 500 行只读到 230 行,重复行照样落盘)。`open()` 自己知道截断之后的文件大小,比对"已消费字节数 == 文件大小"即可 fail closed。`read_glim_traj` 有与 round 3 修掉的 `read_pos` 同一个 bug:`getline` 的 sentry 吞掉 I/O 异常只置 `badbit`,循环结束后没人检查。

- [ ] **Step 1: 写 PosWriter 提前 EOF 的失败测试**

追加到 `gnss_core/test/test_pos_io.cpp` 末尾(复用文件里已有的 `tmp_path`、`sample_record`、`read_raw`):

```cpp
// 去重扫描遇到"提前 EOF"(FUSE/NFS 上 read() 在真实文件末尾之前返回 0、或文件
// 被并发截断):getline 循环正常结束,流上没有 badbit,read_pos 看起来成功返回了
// 一份不完整的结果。open() 必须靠"扫描消费的字节数 != 文件大小"识别出来并失败,
// 否则 existing_keys_ 不完整,后面本该被去重的记录会重复追加。
TEST(PosWriter, PrematureEofDuringDedupScanLeavesFileUntouchedAndOpenFails) {
  const std::string p = tmp_path("pw_inject_premature_eof.pos");
  ::remove(p.c_str());
  const PosRecord a = sample_record();
  PosRecord b = a; b.stamp += 1.0;
  PosRecord c = a; c.stamp += 2.0;
  {
    PosWriter w;
    ASSERT_TRUE(w.open(p));
    ASSERT_TRUE(w.write(a));
    ASSERT_TRUE(w.write(b));
    ASSERT_TRUE(w.write(c));
  }
  const std::string original = read_raw(p);

  gnss_core::testing::set_trailing_line_read_failure_injector(
      [](const char* step) { return std::string(step) == "read_pos_premature_eof"; });
  PosWriter w;
  const bool opened = w.open(p);
  gnss_core::testing::set_trailing_line_read_failure_injector(nullptr);

  EXPECT_FALSE(opened) << "去重扫描只读到一部分就\"正常\"结束时,open() 必须失败";
  EXPECT_EQ(read_raw(p), original) << "失败时文件必须一字节都没被动过";
  ::remove(p.c_str());
}

// 同一个文件、不注入任何故障时必须照常打开——证明字节数校验不会误伤健康文件
// (表头 + 记录每行都以 '\n' 结尾,消费字节数必须恰好等于文件大小)。
TEST(PosWriter, DedupScanByteCountMatchesAHealthyFileExactly) {
  const std::string p = tmp_path("pw_bytecount_healthy.pos");
  ::remove(p.c_str());
  const PosRecord a = sample_record();
  {
    PosWriter w;
    ASSERT_TRUE(w.open(p));
    ASSERT_TRUE(w.write(a));
  }
  PosWriter w;
  EXPECT_TRUE(w.open(p));
  EXPECT_TRUE(w.write(a));
  EXPECT_TRUE(w.last_write_was_suppressed()) << "健康文件重开后去重必须照常生效";
  w.close();
  ::remove(p.c_str());
}
```

- [ ] **Step 2: 写 `read_pos(std::istream&)` / `read_glim_traj(std::istream&)` 走真实 sentry 路径的失败测试**

在 `gnss_core/test/test_pos_io.cpp` 顶部 include 区加 `#include <algorithm>`、`#include <sstream>`、`#include <streambuf>`,并在匿名命名空间里加:

```cpp
// 前 fail_after 个字节正常供给(每次最多 16 字节,确保失败发生在扫描中途),
// 之后 underflow() 抛 std::ios_base::failure——这正是 basic_filebuf 在真实
// EIO 时的行为。std::getline 的 sentry 会吞掉这个异常、只置 badbit,不会
// 重新抛出;被测函数必须自己检查 badbit。不经过任何代码内注入点。
class FailingStreambuf : public std::streambuf {
public:
  FailingStreambuf(std::string data, std::size_t fail_after)
      : data_(std::move(data)), fail_after_(std::min(fail_after, data_.size())) {}

protected:
  int_type underflow() override {
    if (pos_ >= fail_after_) throw std::ios_base::failure("injected EIO");
    const std::size_t n = std::min<std::size_t>(sizeof(buf_), fail_after_ - pos_);
    std::copy(data_.data() + pos_, data_.data() + pos_ + n, buf_);
    pos_ += n;
    setg(buf_, buf_, buf_ + n);
    return traits_type::to_int_type(buf_[0]);
  }

private:
  std::string data_;
  std::size_t fail_after_;
  std::size_t pos_ = 0;
  char buf_[16];
};
```

测试(追加到文件末尾):

```cpp
TEST(PosIo, ReadPosFromStreamThrowsWhenTheStreambufFailsMidScan) {
  std::string content = pos_header(PosTimeSystem::GPST);
  PosRecord r = sample_record();
  for (int i = 0; i < 5; ++i) {
    content += format_pos_record(r, PosTimeSystem::GPST, 18);
    r.stamp += 1.0;
  }
  FailingStreambuf buf(content, content.size() / 2);
  std::istream in(&buf);
  EXPECT_THROW(read_pos(in), std::runtime_error);

  std::istringstream healthy(content);
  EXPECT_EQ(read_pos(healthy).size(), 5u) << "同样的内容不出错时必须完整读出";
}
```

追加到 `gnss_core/test/test_synth.cpp`(同样在该文件加上 `FailingStreambuf` 的定义与 `<algorithm>`、`<sstream>`、`<streambuf>`):

```cpp
TEST(Synth, ReadGlimTrajFromStreamThrowsWhenTheStreambufFailsMidScan) {
  std::string content = "# timestamp tx ty tz qx qy qz qw\n";
  for (int i = 0; i < 20; ++i) content += std::to_string(100 + i) + " 1 2 3 0 0 0 1\n";
  FailingStreambuf buf(content, content.size() / 2);
  std::istream in(&buf);
  EXPECT_THROW(read_glim_traj(in), std::runtime_error)
      << "读到一半 I/O 出错时不能把前一半轨迹当成全部返回——"
         "estimate_lever_arm 会拿半条轨迹算出一个看起来正常的杆臂";

  std::istringstream healthy(content);
  EXPECT_EQ(read_glim_traj(healthy).size(), 20u);
}
```

- [ ] **Step 3: 构建并确认 RED**

Run: `cd /home/steve/glim_ws && colcon build --symlink-install --packages-select gnss_core`
Expected: 编译失败——`read_pos(std::istream&)`、`read_glim_traj(std::istream&)` 未声明。先把 Step 2 的两个测试临时 `#if 0` 掉再构建,运行:
`source install/setup.bash && ./build/gnss_core/test_pos_io --gtest_filter='PosWriter.PrematureEofDuringDedupScanLeavesFileUntouchedAndOpenFails'`
Expected: FAIL(`opened` 为 true——旧代码没有这个注入点、也没有字节数校验)。记录下来,去掉 `#if 0`。

- [ ] **Step 4: 实现 `pos_io`**

`gnss_core/include/gnss_core/pos_io.hpp`,在现有 `read_pos(const std::string&, ...)` 声明下面加:

```cpp
// 从任意输入流读 .pos(语义与按路径读取相同)。读错误(流上出现 badbit)抛
// std::runtime_error,绝不把读到一半的结果当成全部返回。
std::vector<PosRecord> read_pos(std::istream& in, const PosReadOptions& opt = {});
```

(头文件若尚未包含 `<istream>`,加上 `#include <istream>`。)

`gnss_core/src/pos_io.cpp`:

1. 在 `#ifdef GNSS_CORE_WITH_TEST_HOOKS` 分支里、`injected_read_pos_scan_failure()` 之后加:

```cpp
// 模拟"read() 提前返回 0":读循环正常结束、流上没有任何错误位——FUSE/NFS
// 或文件被并发截断时真实发生的就是这种情况,只能靠字节数识别。
bool injected_read_pos_premature_eof() {
  return g_trailing_line_read_failure_injector != nullptr &&
         g_trailing_line_read_failure_injector("read_pos_premature_eof");
}
```

在 `#else` 分支里加 `inline bool injected_read_pos_premature_eof() { return false; }`。

2. 把现有 `read_pos(const std::string&, ...)` 的函数体改成一个匿名命名空间里的内部函数,并提供两个公开入口:

```cpp
namespace {
// bytes_consumed 非空时累加实际从流里取走的字节数(含每行被 getline 消费掉的
// '\n';最后一行没有 '\n' 时 getline 会置 eofbit,那一行不加 1)。
std::vector<PosRecord> read_pos_stream(std::istream& in, const std::string& what,
                                       const PosReadOptions& opt, std::uint64_t* bytes_consumed) {
  PosTimeSystem ts = opt.default_time_system;
  std::vector<PosRecord> out;
  std::string line;
  while (std::getline(in, line)) {
    if (bytes_consumed) *bytes_consumed += line.size() + (in.eof() ? 0u : 1u);
    if (injected_read_pos_scan_failure()) {
      in.setstate(std::ios::badbit);
      break;
    }
    if (injected_read_pos_premature_eof()) break;
    // ……以下保持现有循环体原样(空行、'%' 表头解析时间系统、parse_llh_solution)……
  }
  if (in.bad()) {
    throw std::runtime_error("read_pos: I/O error while reading " + what);
  }
  return out;
}
}  // namespace

std::vector<PosRecord> read_pos(const std::string& path, const PosReadOptions& opt) {
  std::ifstream in(path);
  if (!in) throw std::runtime_error("read_pos: cannot open " + path);
  return read_pos_stream(in, path, opt, nullptr);
}

std::vector<PosRecord> read_pos(std::istream& in, const PosReadOptions& opt) {
  return read_pos_stream(in, "<stream>", opt, nullptr);
}
```

保留原函数体上方那段关于 sentry 吞异常的长注释(移到 `read_pos_stream` 上方)。

3. `PosWriter::open()`:用一个变量记住"截断之后的文件大小",去重扫描改为自己开流并比对字节数。把现有两处 `file_size` 读取改成同时更新 `expected_size`,把 `if (!is_new) { try { ... read_pos(path, ...) ... } }` 那一段替换为:

```cpp
  // 截断之后文件应有的字节数;去重扫描必须恰好消费这么多字节。
  std::uintmax_t expected_size = 0;
```

(放在 `bool is_new = true;` 旁边;第一次 `file_size` 成功时 `expected_size = sz;`,`kTruncated` 分支里 `file_size` 成功时 `expected_size = sz2;`。)

```cpp
  if (!is_new) {
    try {
      std::ifstream in(path);
      if (!in) return false;
      std::uint64_t consumed = 0;
      const auto existing = read_pos_stream(in, path, PosReadOptions{leap_, ts_}, &consumed);
      // 读循环"正常"结束却没读完整个文件:read() 提前返回 0(FUSE/NFS)或文件
      // 被并发截断。键集合不完整,继续打开会让重复记录落盘——fail closed。
      if (consumed != expected_size) return false;
      for (const auto& rec : existing) existing_keys_.insert(record_time_key_ms(rec, ts_, leap_));
    } catch (const std::exception&) {
      return false;
    }
  }
```

4. `gnss_core/include/gnss_core/pos_io_test_hooks.hpp`:在 `set_trailing_line_read_failure_injector` 的注释里补一行已有步骤名清单,加上 `"read_pos_premature_eof"`(读循环正常结束、不置 badbit,模拟提前 EOF)。

- [ ] **Step 5: 实现 `read_glim_traj`**

`gnss_core/include/gnss_core/synth.hpp`,在现有声明下加:

```cpp
// 从输入流读 TUM 轨迹(规则同上)。读错误(badbit)抛 std::runtime_error。
std::vector<TrajPose> read_glim_traj(std::istream& in);
```

`gnss_core/src/synth.cpp`:

```cpp
std::vector<TrajPose> read_glim_traj(std::istream& in) {
  std::vector<TrajPose> out;
  std::string line;
  while (std::getline(in, line)) {
    // ……现有循环体原样……
  }
  // 与 read_pos 同一个坑:getline 的 sentry 吞掉 underflow() 抛出的 I/O 异常、
  // 只置 badbit;不检查就会把读到一半的轨迹当成全部返回。
  if (in.bad()) throw std::runtime_error("read_glim_traj: I/O error while reading");
  std::stable_sort(out.begin(), out.end(), [](const TrajPose& a, const TrajPose& b) { return a.stamp < b.stamp; });
  return out;
}

std::vector<TrajPose> read_glim_traj(const std::string& path) {
  std::ifstream in(path);
  if (!in) throw std::runtime_error("read_glim_traj: cannot open " + path);
  return read_glim_traj(in);
}
```

(`gnss_core/tools/estimate_lever_arm.cpp:75-78`、`tools/synth_rtk_fix.cpp:54` 已经把调用包在 `catch (const std::exception&)` 里;打开这两处确认一下,不需要改。)

- [ ] **Step 6: 跑测试确认 GREEN,并做变异检查**

Run: `cd /home/steve/glim_ws && colcon build --symlink-install --packages-select gnss_core && source install/setup.bash && ./build/gnss_core/test_pos_io && ./build/gnss_core/test_synth`
Expected: 全部 PASS。

变异检查(每个做完都恢复):
- 删掉 `if (consumed != expected_size) return false;` → `PrematureEofDuringDedupScanLeavesFileUntouchedAndOpenFails` 必须 FAIL
- 删掉 `read_pos_stream` 里的 `if (in.bad()) throw` → `ReadPosFromStreamThrowsWhenTheStreambufFailsMidScan` 必须 FAIL
- 删掉 `read_glim_traj(std::istream&)` 里的 `if (in.bad()) throw` → `ReadGlimTrajFromStreamThrowsWhenTheStreambufFailsMidScan` 必须 FAIL
- 把字节计数里的 `(in.eof() ? 0u : 1u)` 改成 `1u` → `DedupScanByteCountMatchesAHealthyFileExactly` 不一定失败(健康文件每行都有 '\n');不要求,但在 commit message 里如实写明

- [ ] **Step 7: 全量测试**

Run: `cd /home/steve/glim_ws && colcon test --packages-select gnss_core gnss_bringup && colcon test-result --all`
Expected: 0 failures;测试总数 = 340 + 本 Task 新增数。

- [ ] **Step 8: Commit**

```bash
cd /home/steve/glim_ws/src/glim_ext
git add gnss_core/include/gnss_core/pos_io.hpp gnss_core/include/gnss_core/pos_io_test_hooks.hpp gnss_core/src/pos_io.cpp gnss_core/include/gnss_core/synth.hpp gnss_core/src/synth.cpp gnss_core/test/test_pos_io.cpp gnss_core/test/test_synth.cpp
git commit -m "fix(gnss_core): fail closed on silent partial reads in PosWriter dedup scan and read_glim_traj

<写明 RED 证据与三个变异的结果>

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 2: 进程监管按 PATH 解析可执行文件,并回报每次派生与退出

**Files:**
- Create: `gnss_bringup/include/gnss_bringup/executable_lookup.hpp`
- Create: `gnss_bringup/src/executable_lookup.cpp`
- Modify: `gnss_bringup/include/gnss_bringup/process_supervisor.hpp`
- Modify: `gnss_bringup/src/process_supervisor.cpp`(`run()`:argv 构建之后的 `while` 循环,fork 前、fork 后父进程分支、waitpid 循环之后)
- Modify: `gnss_bringup/CMakeLists.txt`
- Test: `gnss_bringup/test/test_executable_lookup.cpp`(新)、`gnss_bringup/test/test_process_supervisor.cpp`

**Interfaces:**
- Produces:
  - `std::string gnss_bringup::resolve_executable(const std::string& name, const char* path_env);`
  - `struct gnss_bringup::ChildExitInfo`(字段见 Step 4)
  - `ProcessSupervisorConfig::on_spawn`:`std::function<void(int pid, const std::string& executable)>`
  - `ProcessSupervisorConfig::on_exit`:`std::function<void(const ChildExitInfo&)>`
  - `int ProcessSupervisor::start_failure_count() const;`

- [ ] **Step 1: 写 `resolve_executable` 的失败测试**

`gnss_bringup/test/test_executable_lookup.cpp`:

```cpp
#include <gtest/gtest.h>
#include <sys/stat.h>
#include <unistd.h>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>
#include "gnss_bringup/executable_lookup.hpp"
using gnss_bringup::resolve_executable;

namespace {
class ExecutableLookup : public ::testing::Test {
protected:
  void SetUp() override {
    const char* base = std::getenv("TMPDIR");
    std::string tmpl = std::string(base ? base : "/tmp") + "/exe_lookup_XXXXXX";
    std::vector<char> buf(tmpl.begin(), tmpl.end());
    buf.push_back('\0');
    ASSERT_NE(::mkdtemp(buf.data()), nullptr);
    dir_ = buf.data();
    std::filesystem::create_directories(dir_ + "/a");
    std::filesystem::create_directories(dir_ + "/b");
  }
  void TearDown() override { std::filesystem::remove_all(dir_); }

  std::string make_file(const std::string& rel, mode_t mode) {
    const std::string p = dir_ + "/" + rel;
    std::ofstream(p) << "#!/bin/sh\n";
    ::chmod(p.c_str(), mode);
    return p;
  }
  std::string dir_;
};
}  // namespace

TEST_F(ExecutableLookup, FindsTheFirstExecutableMatchInPathOrder) {
  make_file("a/tool", 0755);
  make_file("b/tool", 0755);
  const std::string path = dir_ + "/a:" + dir_ + "/b";
  EXPECT_EQ(resolve_executable("tool", path.c_str()), dir_ + "/a/tool");
}

TEST_F(ExecutableLookup, SkipsNonExecutableFilesAndDirectories) {
  make_file("a/tool", 0644);                               // 不可执行
  std::filesystem::create_directories(dir_ + "/b/tool");   // 同名目录
  const std::string path = dir_ + "/a:" + dir_ + "/b";
  EXPECT_EQ(resolve_executable("tool", path.c_str()), "");
}

TEST_F(ExecutableLookup, NameWithSlashIsCheckedDirectlyNotSearched) {
  const std::string exe = make_file("a/tool", 0755);
  EXPECT_EQ(resolve_executable(exe, "/nonexistent"), exe);
  EXPECT_EQ(resolve_executable(dir_ + "/b/tool", (dir_ + "/a").c_str()), "")
      << "带斜杠的名字不能再去 PATH 里找同名文件";
}

TEST_F(ExecutableLookup, RelativeNameWithSlashIsReturnedAsAbsolutePath) {
  // 子进程在 exec 之前会 chdir(run_dir),相对路径必须在父进程里就转成绝对路径
  make_file("a/tool", 0755);
  const auto old = std::filesystem::current_path();
  std::filesystem::current_path(dir_);
  const std::string got = resolve_executable("./a/tool", nullptr);
  std::filesystem::current_path(old);
  ASSERT_FALSE(got.empty());
  EXPECT_EQ(got.front(), '/');
  EXPECT_TRUE(std::filesystem::equivalent(got, dir_ + "/a/tool"));
}

TEST_F(ExecutableLookup, EmptyPathSegmentsDoNotMeanCurrentDirectory) {
  make_file("tool", 0755);
  const auto old = std::filesystem::current_path();
  std::filesystem::current_path(dir_);
  const std::string got = resolve_executable("tool", ":/nonexistent:");
  std::filesystem::current_path(old);
  EXPECT_EQ(got, "") << "空段在 POSIX 里表示当前目录;子进程的 cwd 是 run_dir,不能被当成查找目录";
}

TEST(ExecutableLookupInputs, NullOrEmptyInputsResolveToNothing) {
  EXPECT_EQ(resolve_executable("", "/usr/bin"), "");
  EXPECT_EQ(resolve_executable("sh", nullptr), "");
  EXPECT_EQ(resolve_executable("sh", ""), "");
}
```

`gnss_bringup/CMakeLists.txt`:`add_library(gnss_bringup_io SHARED ...)` 列表里加 `src/executable_lookup.cpp`;`if(BUILD_TESTING)` 里加

```cmake
  # resolve_executable:ProcessSupervisor 在父进程里按 PATH 解析 binary(execv
  # 不查 PATH,子进程又已 chdir 到 run_dir,裸名字 "rtkrcv" 以前永远起不来)。
  ament_add_gtest(test_executable_lookup test/test_executable_lookup.cpp)
  target_link_libraries(test_executable_lookup gnss_bringup_io)
```

- [ ] **Step 2: 写监管器的失败测试**

`gnss_bringup/test/test_process_supervisor.cpp`:include 区加 `#include <mutex>`、`#include <vector>`;匿名命名空间里加

```cpp
// 记录监管回调。用 shared_ptr 持有:stop_async() 超时的情况下监管对象会比
// 测试函数活得久,回调捕获裸 this 会变成悬空指针。
struct Recorder {
  std::mutex mu;
  std::vector<int> spawned;
  std::vector<ChildExitInfo> exits;
  std::size_t exit_count() {
    std::lock_guard<std::mutex> lk(mu);
    return exits.size();
  }
};

void attach_recorder(ProcessSupervisorConfig& c, const std::shared_ptr<Recorder>& rec) {
  c.on_spawn = [rec](int pid, const std::string&) {
    std::lock_guard<std::mutex> lk(rec->mu);
    rec->spawned.push_back(pid);
  };
  c.on_exit = [rec](const ChildExitInfo& e) {
    std::lock_guard<std::mutex> lk(rec->mu);
    rec->exits.push_back(e);
  };
}
```

**删除**现有 `TEST(ProcessSupervisor, MissingBinaryKeepsRetryingAndBacksOff)`(它断言"找不到二进制时会反复 fork",正是要改掉的行为),加入以下测试:

```cpp
TEST(ProcessSupervisor, BareBinaryNameIsFoundThroughPath) {
  // 回归:yaml 默认 binary: "rtkrcv" 是裸名字。execv() 不查 PATH,子进程又已经
  // chdir(cwd),以前会在 cwd 下找这个名字、_exit(127)、无限重启。
  const std::string full = fake();
  const std::string dir = full.substr(0, full.rfind('/'));
  const std::string name = full.substr(full.rfind('/') + 1);
  const char* old = std::getenv("PATH");
  const std::string saved = old ? old : "";
  ::setenv("PATH", (dir + ":" + saved).c_str(), 1);

  ProcessSupervisorConfig c = cfg_for("live", 0.05);
  c.binary = name;
  ProcessSupervisor s(c);
  s.start();
  for (int i = 0; i < 100 && s.spawn_count() < 1; ++i)
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  const int pid = s.last_child_pid();
  const bool alive = pid > 0 && ::kill(pid, 0) == 0;
  const int spawns = s.spawn_count();
  s.stop();
  ::setenv("PATH", saved.c_str(), 1);

  EXPECT_TRUE(alive) << "裸名字必须按 PATH 找到并真正跑起来";
  EXPECT_EQ(spawns, 1) << "跑起来的 live 子进程不该被反复重启";
}

TEST(ProcessSupervisor, OnExitReportsExitCodeLifetimeAndNextDelay) {
  auto rec = std::make_shared<Recorder>();
  ProcessSupervisorConfig c = cfg_for("die", 0.05);
  attach_recorder(c, rec);
  ProcessSupervisor s(c);
  s.start();
  for (int i = 0; i < 300 && rec->exit_count() < 2; ++i)
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  s.stop();

  std::lock_guard<std::mutex> lk(rec->mu);
  ASSERT_GE(rec->exits.size(), 2u);
  ASSERT_GE(rec->spawned.size(), 1u);
  const ChildExitInfo& e = rec->exits[0];
  EXPECT_FALSE(e.spawn_failed);
  EXPECT_TRUE(e.exited);
  EXPECT_EQ(e.exit_code, 1) << "fake_rtkrcv.sh die 以 1 退出";
  EXPECT_FALSE(e.signaled);
  EXPECT_EQ(e.pid, rec->spawned[0]);
  EXPECT_GE(e.lifetime_s, 0.0);
  EXPECT_LT(e.lifetime_s, 0.5);
  EXPECT_GT(e.next_delay_s, 0.0);
  EXPECT_TRUE(e.will_restart);
  EXPECT_GE(rec->exits[1].next_delay_s, e.next_delay_s) << "崩溃循环里下一次等待不应缩短";
}

TEST(ProcessSupervisor, OnExitReportsTheKillingSignal) {
  auto rec = std::make_shared<Recorder>();
  ProcessSupervisorConfig c = cfg_for("live", 0.05);
  c.restart_delay_s = 5.0;
  c.max_restart_delay_s = 5.0;   // 被杀之后的重启等待足够长,stop() 负责打断
  attach_recorder(c, rec);
  ProcessSupervisor s(c);
  s.start();
  for (int i = 0; i < 100 && s.spawn_count() < 1; ++i)
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  ASSERT_EQ(::kill(s.last_child_pid(), SIGKILL), 0);
  for (int i = 0; i < 300 && rec->exit_count() < 1; ++i)
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  s.stop();

  std::lock_guard<std::mutex> lk(rec->mu);
  ASSERT_GE(rec->exits.size(), 1u);
  EXPECT_TRUE(rec->exits[0].signaled);
  EXPECT_EQ(rec->exits[0].signal, SIGKILL);
  EXPECT_FALSE(rec->exits[0].exited);
  EXPECT_TRUE(rec->exits[0].will_restart);
}

TEST(ProcessSupervisor, MissingBinaryIsReportedWithoutForkingAndBacksOff) {
  auto rec = std::make_shared<Recorder>();
  ProcessSupervisorConfig c = cfg_for("live", 0.05);
  c.binary = "/nonexistent/rtkrcv";
  attach_recorder(c, rec);
  ProcessSupervisor s(c);
  s.start();
  for (int i = 0; i < 100 && (s.start_failure_count() < 2 || s.current_delay_s() <= 0.05); ++i)
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  const int failures = s.start_failure_count();
  const int spawns = s.spawn_count();
  const double delay = s.current_delay_s();
  s.stop();

  EXPECT_GE(failures, 2) << "必须持续重试";
  EXPECT_EQ(spawns, 0) << "解析不到可执行文件时不该 fork 一个注定 _exit(127) 的子进程";
  EXPECT_GT(delay, 0.05) << "必须退避";
  EXPECT_LE(delay, 0.4) << "但不得超过 max_restart_delay_s";
  std::lock_guard<std::mutex> lk(rec->mu);
  ASSERT_FALSE(rec->exits.empty());
  EXPECT_TRUE(rec->exits[0].spawn_failed);
  EXPECT_EQ(rec->exits[0].pid, -1);
  EXPECT_NE(rec->exits[0].detail.find("/nonexistent/rtkrcv"), std::string::npos) << rec->exits[0].detail;
}

TEST(ProcessSupervisor, ExitCausedByStopIsReportedAsNotRestarting) {
  auto rec = std::make_shared<Recorder>();
  ProcessSupervisorConfig c = cfg_for("live", 0.05);
  attach_recorder(c, rec);
  auto s = std::make_shared<ProcessSupervisor>(c);
  s->start();
  for (int i = 0; i < 100 && s->spawn_count() < 1; ++i)
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  auto done = stop_async(s);
  ASSERT_EQ(done.wait_for(std::chrono::seconds(15)), std::future_status::ready);

  std::lock_guard<std::mutex> lk(rec->mu);
  ASSERT_EQ(rec->exits.size(), 1u);
  EXPECT_FALSE(rec->exits[0].will_restart);
}
```

- [ ] **Step 3: 确认 RED**

Run: `cd /home/steve/glim_ws && colcon build --symlink-install --packages-select gnss_bringup`
Expected: 编译失败(`executable_lookup.hpp`、`ChildExitInfo`、`on_exit`、`start_failure_count` 不存在)。先只加空的 `executable_lookup.hpp` 声明 + 返回 `""` 的实现,以及 Step 4 的头文件字段(实现暂不调用回调),构建后运行:
`source install/setup.bash && ./build/gnss_bringup/test_process_supervisor --gtest_filter='ProcessSupervisor.BareBinaryNameIsFoundThroughPath:ProcessSupervisor.OnExit*:ProcessSupervisor.MissingBinary*:ProcessSupervisor.ExitCausedByStop*' && ./build/gnss_bringup/test_executable_lookup`
Expected: 这些测试全部 FAIL。记录输出。

- [ ] **Step 4: 实现**

`gnss_bringup/include/gnss_bringup/executable_lookup.hpp`:

```cpp
#pragma once
#include <string>

namespace gnss_bringup {

// 把 ProcessSupervisor 的 binary 解析成可以直接交给 execv() 的路径;找不到返回空串。
//   - name 含 '/':不查 PATH,只检查它是不是可执行的普通文件;相对路径转成绝对路径
//     (子进程 exec 之前会 chdir 到 run_dir,相对路径到那里就失效了)。
//   - 否则按 path_env(冒号分隔)依次查找第一个可执行的普通文件。空段(POSIX 里表示
//     当前目录)被忽略:子进程的 cwd 是 run_dir,那里不该被当作查找目录。
std::string resolve_executable(const std::string& name, const char* path_env);

}  // namespace gnss_bringup
```

`gnss_bringup/src/executable_lookup.cpp`:

```cpp
#include "gnss_bringup/executable_lookup.hpp"

#include <sys/stat.h>
#include <unistd.h>

#include <filesystem>

namespace gnss_bringup {

namespace {
bool is_executable_file(const std::string& p) {
  struct stat st {};
  return ::stat(p.c_str(), &st) == 0 && S_ISREG(st.st_mode) && ::access(p.c_str(), X_OK) == 0;
}
}  // namespace

std::string resolve_executable(const std::string& name, const char* path_env) {
  if (name.empty()) return {};
  if (name.find('/') != std::string::npos) {
    if (!is_executable_file(name)) return {};
    std::error_code ec;
    const auto abs = std::filesystem::absolute(name, ec);
    return ec ? std::string{} : abs.string();
  }
  if (path_env == nullptr) return {};
  const std::string path(path_env);
  std::size_t begin = 0;
  while (begin <= path.size()) {
    std::size_t end = path.find(':', begin);
    if (end == std::string::npos) end = path.size();
    if (end > begin) {
      std::string cand = path.substr(begin, end - begin);
      if (cand.back() != '/') cand += '/';
      cand += name;
      if (is_executable_file(cand)) return cand;
    }
    begin = end + 1;
  }
  return {};
}

}  // namespace gnss_bringup
```

`gnss_bringup/include/gnss_bringup/process_supervisor.hpp`:include 区加 `#include <functional>`;在 `ProcessSupervisorConfig` 之前加

```cpp
// 一次子进程生命周期结束(或一次没能派生)的报告。
struct ChildExitInfo {
  int pid = -1;                 // -1:本轮没有 fork(spawn_failed)
  bool spawn_failed = false;    // binary 解析不到可执行文件,本轮没有派生子进程
  std::string detail;           // spawn_failed 时的原因
  bool exited = false;          // WIFEXITED
  int exit_code = 0;
  bool signaled = false;        // WIFSIGNALED
  int signal = 0;
  double lifetime_s = 0.0;
  double next_delay_s = 0.0;    // 下一次尝试前要等的秒数
  bool will_restart = true;     // stop() 引起的退出为 false
};
```

`ProcessSupervisorConfig` 末尾加

```cpp
  // 两个回调都在监管线程上同步调用。回调里不得调用 stop() 或析构这个
  // ProcessSupervisor(stop() 会 join 监管线程,自己 join 自己会死锁),也不要
  // 在里面做耗时操作(会推迟对子进程的回收)。
  std::function<void(int pid, const std::string& executable)> on_spawn;
  std::function<void(const ChildExitInfo&)> on_exit;
```

类里加 `int start_failure_count() const { return start_failure_count_.load(); }` 与成员 `std::atomic<int> start_failure_count_{0};`,并更新类注释:`binary` 在每次派生前按 `PATH` 解析,找不到时不 fork、按崩溃循环退避。

`gnss_bringup/src/process_supervisor.cpp`:include 区加 `#include <cstdlib>` 与 `#include "gnss_bringup/executable_lookup.hpp"`。在 `run()` 的 `while (running_.load()) {` 循环体最前面(`sigset_t block_all` 之前)加:

```cpp
    // 每轮重新解析:binary 可能是在节点起来之后才装上的。解析放在父进程里做——
    // execv() 不查 PATH,而子进程在 exec 之前已经 chdir(cwd)。解析不到就不 fork
    // (以前会 fork 出一个注定 _exit(127) 的子进程,从外面看和"rtkrcv 自己崩了"
    // 一模一样),按崩溃循环退避并如实报告原因。
    const std::string exe = resolve_executable(cfg_.binary, ::getenv("PATH"));
    if (exe.empty()) {
      start_failure_count_.fetch_add(1);
      const double prev = current_delay_.load();
      current_delay_.store(prev <= 0.0 ? base_delay : std::min(prev * 2.0, max_delay));
      if (cfg_.on_exit) {
        ChildExitInfo info;
        info.spawn_failed = true;
        info.detail = "binary=" + cfg_.binary + " 在 PATH 中找不到或不可执行";
        info.next_delay_s = current_delay_.load();
        info.will_restart = running_.load();
        cfg_.on_exit(info);
      }
      if (!running_.load()) break;
      interruptible_wait(current_delay_.load());
      continue;
    }
```

子进程分支里把 `::execv(cfg_.binary.c_str(), argv.data());` 改成 `::execv(exe.c_str(), argv.data());`(`argv[0]` 保持 `cfg_.binary`,`ps` 里仍显示配置的名字)。

父进程分支里,在 `spawn_count_.fetch_add(1);` 之后加:

```cpp
    if (cfg_.on_spawn) cfg_.on_spawn(pid, exe);
```

waitpid 循环:在循环前声明 `bool reaped = false;`,把 `if (w == pid) break;` 改成 `if (w == pid) { reaped = true; break; }`。在计算完 `current_delay_`(`if (lifetime_s < cfg_.crash_loop_life_s) {...} else {...}`)之后、`if (!running_.load()) break;` 之前加:

```cpp
    if (cfg_.on_exit) {
      ChildExitInfo info;
      info.pid = pid;
      if (reaped) {
        if (WIFEXITED(status)) {
          info.exited = true;
          info.exit_code = WEXITSTATUS(status);
        } else if (WIFSIGNALED(status)) {
          info.signaled = true;
          info.signal = WTERMSIG(status);
        }
      }
      info.lifetime_s = lifetime_s;
      info.next_delay_s = current_delay_.load();
      info.will_restart = running_.load();
      cfg_.on_exit(info);
    }
```

- [ ] **Step 5: GREEN + 变异检查**

Run: `cd /home/steve/glim_ws && colcon build --symlink-install --packages-select gnss_bringup && source install/setup.bash && ./build/gnss_bringup/test_executable_lookup && ./build/gnss_bringup/test_process_supervisor`
Expected: 全部 PASS(包括原有的 `ChildDiesWhenTheForkingThreadGoesAway` 等)。

变异检查(做完恢复):
- 子进程分支改回 `execv(cfg_.binary.c_str(), ...)` → `BareBinaryNameIsFoundThroughPath` 必须 FAIL
- 把 `info.will_restart = running_.load();`(退出报告那一处)改成 `true` → `ExitCausedByStopIsReportedAsNotRestarting` 必须 FAIL
- `resolve_executable` 里去掉 `if (end > begin)` 的空段过滤(空段当 "." 用)→ `EmptyPathSegmentsDoNotMeanCurrentDirectory` 必须 FAIL

- [ ] **Step 6: 全量测试**

Run: `cd /home/steve/glim_ws && colcon test --packages-select gnss_core gnss_bringup && colcon test-result --all`
Expected: 0 failures。

- [ ] **Step 7: Commit**

```bash
cd /home/steve/glim_ws/src/glim_ext
git add gnss_bringup/include/gnss_bringup/executable_lookup.hpp gnss_bringup/src/executable_lookup.cpp gnss_bringup/include/gnss_bringup/process_supervisor.hpp gnss_bringup/src/process_supervisor.cpp gnss_bringup/CMakeLists.txt gnss_bringup/test/test_executable_lookup.cpp gnss_bringup/test/test_process_supervisor.cpp
git commit -m "fix(gnss_bringup): resolve the supervised binary through PATH and report every spawn and exit

<写明:yaml 默认 binary: \"rtkrcv\" 以前永远起不来的根因(execv 不查 PATH + 子进程已 chdir);RED 证据;三个变异结果>

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 3: conf 渲染补上基准站坐标来源与 AR 选项,枚举取值按 RTKLIB-EX 2.5.1 校验

**Files:**
- Modify: `gnss_bringup/include/gnss_bringup/rtkrcv_conf.hpp`
- Modify: `gnss_bringup/src/rtkrcv_conf.cpp`
- Test: `gnss_bringup/test/test_rtkrcv_conf.cpp`

**Interfaces:**
- Produces:`RtkrcvConfParams` 新字段 `std::string base_pos_type = "rtcm";`、`std::string bds_ar_mode = "off";`、`std::string glo_ar_mode = "fix-and-hold";`;`render_rtkrcv_conf()` 对非法取值抛 `std::invalid_argument`,消息以 `<字段名>="<值>"` 开头并列出可选值。

各字段允许的取值(照抄 RTKLIB-EX 2.5.1 `rtkrcv` 控制台 `option` 输出):

| 字段 | conf 键 | 允许值 |
|---|---|---|
| `obs_format` / `corr_format` | `inpstr1-format` / `inpstr2-format` | `rtcm2 rtcm3 oem4 ubx swift hemis skytraq javad nvs binex rt17 sbf unicore` |
| `pos_mode` | `pos1-posmode` | `single dgps kinematic static static-start movingbase fixed ppp-kine ppp-static ppp-fixed` |
| `ar_mode` | `pos2-armode` | `off continuous instantaneous fix-and-hold` |
| `glo_ar_mode` | `pos2-gloarmode` | `off on autocal fix-and-hold` |
| `bds_ar_mode` | `pos2-bdsarmode` | `off on` |
| `base_pos_type` | `ant2-postype` | `rtcm single`(`llh`/`xyz` 需要坐标参数,本轮不支持) |
| `navsys` | `pos1-navsys` | 整数 1..127 |

(输入流格式里 2.5.1 还列了 `rinex sp3 clk`,那是文件回放格式,不适用于我们的 TCP 流,故不放行。)

- [ ] **Step 1: 写失败测试**

在 `gnss_bringup/test/test_rtkrcv_conf.cpp` 中:把 `TEST(RtkrcvConf, StreamFormatsAreConfigurable)` 里的 `p.obs_format = "novatel";` 与期望 `"inpstr1-format =novatel"` 改为 `"oem4"`(`novatel` 在 RTKLIB 里不是合法取值,rtkrcv 会悄悄回落到默认值)。然后追加:

```cpp
TEST(RtkrcvConf, BasePositionComesFromRtcmByDefault) {
  // 回归(2026-09-14 实测):不写 ant2-postype 时 rtkrcv 默认 llh 0,0,0,
  // RTK 模式一条解都不输出。
  const auto c = render_rtkrcv_conf(RtkrcvConfParams{});
  EXPECT_TRUE(has_line(c, "ant2-postype =rtcm"));
}

TEST(RtkrcvConf, SinglePointBasePositionIsSelectable) {
  RtkrcvConfParams p;
  p.base_pos_type = "single";
  EXPECT_TRUE(has_line(render_rtkrcv_conf(p), "ant2-postype =single"));
}

TEST(RtkrcvConf, BasePositionTypesNeedingCoordinatesAreRejected) {
  RtkrcvConfParams p;
  p.base_pos_type = "llh";
  EXPECT_THROW(render_rtkrcv_conf(p), std::invalid_argument);
}

TEST(RtkrcvConf, AmbiguityResolutionOptionsArePinnedExplicitly) {
  // 显式写出 2.5.1 自身的默认值:换 RTKLIB 版本时默认值悄悄变化不会带进来
  const auto c = render_rtkrcv_conf(RtkrcvConfParams{});
  EXPECT_TRUE(has_line(c, "pos2-bdsarmode =off"));
  EXPECT_TRUE(has_line(c, "pos2-gloarmode =fix-and-hold"));

  RtkrcvConfParams p;
  p.bds_ar_mode = "on";
  p.glo_ar_mode = "autocal";
  const auto c2 = render_rtkrcv_conf(p);
  EXPECT_TRUE(has_line(c2, "pos2-bdsarmode =on"));
  EXPECT_TRUE(has_line(c2, "pos2-gloarmode =autocal"));
}

TEST(RtkrcvConf, UnknownEnumValuesAreRejectedInsteadOfSilentlyFallingBack) {
  // rtkrcv 遇到非法取值只打一行警告、回落到默认值继续跑(实测
  // pos2-armode =continuouss → fix-and-hold),所以必须在生成 conf 时拒绝
  const auto expect_rejected = [](RtkrcvConfParams p, const std::string& field) {
    try {
      render_rtkrcv_conf(p);
      ADD_FAILURE() << field << " 的非法取值没有被拒绝";
    } catch (const std::invalid_argument& e) {
      EXPECT_EQ(std::string(e.what()).rfind(field + "=", 0), 0u) << e.what();
    }
  };
  RtkrcvConfParams p;
  p.pos_mode = "kinematicc";   expect_rejected(p, "pos_mode");     p = {};
  p.ar_mode = "continuouss";   expect_rejected(p, "ar_mode");      p = {};
  p.obs_format = "novatel";    expect_rejected(p, "obs_format");   p = {};
  p.corr_format = "rtcm";      expect_rejected(p, "corr_format");  p = {};
  p.bds_ar_mode = "yes";       expect_rejected(p, "bds_ar_mode");  p = {};
  p.glo_ar_mode = "hold";      expect_rejected(p, "glo_ar_mode");  p = {};
  p.base_pos_type = "xyz";     expect_rejected(p, "base_pos_type");
}

TEST(RtkrcvConf, EveryRtklibEx251PositioningModeIsAccepted) {
  for (const char* m : {"single", "dgps", "kinematic", "static", "static-start", "movingbase",
                        "fixed", "ppp-kine", "ppp-static", "ppp-fixed"}) {
    RtkrcvConfParams p;
    p.pos_mode = m;
    EXPECT_NO_THROW(render_rtkrcv_conf(p)) << m;
  }
}

TEST(RtkrcvConf, NavsysOutsideTheSystemBitmaskIsRejected) {
  RtkrcvConfParams p;
  p.navsys = 0;
  EXPECT_THROW(render_rtkrcv_conf(p), std::invalid_argument);
  p.navsys = 128;
  EXPECT_THROW(render_rtkrcv_conf(p), std::invalid_argument);
  p.navsys = 127;
  EXPECT_NO_THROW(render_rtkrcv_conf(p));
}
```

- [ ] **Step 2: 确认 RED**

Run: `cd /home/steve/glim_ws && colcon build --symlink-install --packages-select gnss_bringup`
Expected: 编译失败(新字段不存在)。先只在头文件里加三个字段,再构建运行 `source install/setup.bash && ./build/gnss_bringup/test_rtkrcv_conf`
Expected: 新增的 7 个测试全部 FAIL,改过的 `StreamFormatsAreConfigurable` PASS。记录输出。

- [ ] **Step 3: 实现**

`gnss_bringup/include/gnss_bringup/rtkrcv_conf.hpp`:把注释"键名针对 RTKLIB demo5"改为"键名与取值针对 RTKLIB-EX 2.5.1(rtklibexplorer,原 demo5)",在 `ar_mode` 之后加

```cpp
  // 基准站坐标来源(ant2-postype)。"rtcm":取差分流里的 RTCM 1005/1006;"single":
  // 基准站观测的单点解。不写这个键时 rtkrcv 默认 llh 0,0,0,RTK 一条解都不输出。
  std::string base_pos_type = "rtcm";
  // 北斗 / GLONASS 模糊度固定。默认值与 RTKLIB-EX 2.5.1 自身默认一致,显式写进 conf,
  // 避免换版本后默认值悄悄变化。
  std::string bds_ar_mode = "off";
  std::string glo_ar_mode = "fix-and-hold";
```

并在 `render_rtkrcv_conf` 声明上方补一句:任何字段取值不在 RTKLIB-EX 2.5.1 允许范围内时抛 `std::invalid_argument`。

`gnss_bringup/src/rtkrcv_conf.cpp`:include 区加 `#include <initializer_list>`;匿名命名空间里加

```cpp
// rtkrcv 对非法取值只打一行 "invalid option value" 就回落到默认值继续运行,
// 进程照样活着——所以取值必须在生成 conf 时校验,不能指望 rtkrcv 报错。
void validate_one_of(const std::string& value, const char* field,
                     std::initializer_list<const char*> allowed) {
  for (const char* a : allowed) {
    if (value == a) return;
  }
  std::string list;
  for (const char* a : allowed) {
    if (!list.empty()) list += ", ";
    list += a;
  }
  throw std::invalid_argument(std::string(field) + "=\"" + value +
                              "\" 不是 RTKLIB-EX 2.5.1 认识的取值(可选: " + list + ")");
}
```

`render_rtkrcv_conf` 里,在现有 `validate_string_field` 调用之后、`elmask` 校验之前加

```cpp
  const std::initializer_list<const char*> kStreamFormats = {
      "rtcm2", "rtcm3", "oem4", "ubx", "swift", "hemis", "skytraq",
      "javad", "nvs", "binex", "rt17", "sbf", "unicore"};
  validate_one_of(p.obs_format, "obs_format", kStreamFormats);
  validate_one_of(p.corr_format, "corr_format", kStreamFormats);
  validate_one_of(p.pos_mode, "pos_mode",
                  {"single", "dgps", "kinematic", "static", "static-start", "movingbase",
                   "fixed", "ppp-kine", "ppp-static", "ppp-fixed"});
  validate_one_of(p.ar_mode, "ar_mode", {"off", "continuous", "instantaneous", "fix-and-hold"});
  validate_one_of(p.glo_ar_mode, "glo_ar_mode", {"off", "on", "autocal", "fix-and-hold"});
  validate_one_of(p.bds_ar_mode, "bds_ar_mode", {"off", "on"});
  // llh / xyz 需要另给基准站坐标参数,本轮不支持
  validate_one_of(p.base_pos_type, "base_pos_type", {"rtcm", "single"});
  if (p.navsys < 1 || p.navsys > 127) {
    throw std::invalid_argument("navsys=" + std::to_string(p.navsys) +
                                " 超出系统位掩码范围 [1,127](1:GPS 2:SBAS 4:GLO 8:GAL 16:QZS 32:BDS 64:NavIC)");
  }
```

(注意:`kStreamFormats` 是一个 `std::initializer_list` 局部变量,它引用的临时数组生命周期与该变量相同,在本函数内使用是安全的。)

在 `oss << "pos1-navsys =" ...` 之后加

```cpp
  oss << "pos2-gloarmode =" << p.glo_ar_mode << "\n";
  oss << "pos2-bdsarmode =" << p.bds_ar_mode << "\n";

  // 基准站坐标来源——不写时 rtkrcv 默认 llh 0,0,0,RTK 一条解都不输出
  oss << "ant2-postype =" << p.base_pos_type << "\n";
```

- [ ] **Step 4: GREEN + 变异检查**

Run: `cd /home/steve/glim_ws && colcon build --symlink-install --packages-select gnss_bringup && source install/setup.bash && ./build/gnss_bringup/test_rtkrcv_conf`
Expected: 全部 PASS。

变异检查(做完恢复):删掉 `ant2-postype` 那一行输出 → `BasePositionComesFromRtcmByDefault` 必须 FAIL;删掉 `validate_one_of(p.pos_mode, ...)` → `UnknownEnumValuesAreRejectedInsteadOfSilentlyFallingBack` 必须 FAIL。

- [ ] **Step 5: 全量测试**

Run: `cd /home/steve/glim_ws && colcon test --packages-select gnss_core gnss_bringup && colcon test-result --all`
Expected: 0 failures。

- [ ] **Step 6: Commit**

```bash
cd /home/steve/glim_ws/src/glim_ext
git add gnss_bringup/include/gnss_bringup/rtkrcv_conf.hpp gnss_bringup/src/rtkrcv_conf.cpp gnss_bringup/test/test_rtkrcv_conf.cpp
git commit -m "fix(gnss_bringup): set the base position source in rtkrcv.conf and validate every option value against RTKLIB-EX 2.5.1

<写明:缺 ant2-postype 时真实回放 0 条解、补上后 101 条的实测;rtkrcv 对非法值静默回落默认值;novatel 测试值更正;RED 与变异结果>

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 4: 节点接线:新参数、启动前检查可执行文件、子进程退出日志、sol 流日志去重

**Files:**
- Create: `gnss_bringup/include/gnss_bringup/sol_stream_log_gate.hpp`
- Create: `gnss_bringup/test/test_sol_stream_log_gate.cpp`
- Modify: `gnss_bringup/src/rtkrcv_node.cpp`
- Modify: `gnss_bringup/CMakeLists.txt`
- Modify: `gnss_bringup/config/gnss_bringup.yaml`(`rtkrcv_node:` 段)
- Modify: `gnss_bringup/README.md`(新增一节;「参数说明」)

**Interfaces:**
- Consumes: Task 2 的 `resolve_executable`、`ChildExitInfo`、`ProcessSupervisorConfig::on_spawn/on_exit`;Task 3 的 `RtkrcvConfParams::base_pos_type/bds_ar_mode/glo_ar_mode`
- Produces(Task 5、6 的测试按这些**原文子串**断言,不得改动措辞):
  - 启动前 binary 解析失败:进程退出码 1,日志含 `找不到或不可执行`
  - 每次派生:`RCLCPP_INFO` 含 `rtkrcv 已启动 pid=<pid>`
  - 子进程退出(会重启):`RCLCPP_WARN` 含 `rtkrcv 退出 pid=<pid>`
  - 解析失败(运行期):`RCLCPP_ERROR` 含 `rtkrcv 无法启动:`
  - sol 流第一次连上:`RCLCPP_INFO` 含 `sol stream: connected`
  - 类 `gnss_bringup::SolStreamLogGate`:`bool on_status(bool connected, const std::string& detail)`、`void on_solution_line()`、`bool quiet() const`

- [ ] **Step 1: 写 SolStreamLogGate 的失败测试**

`gnss_bringup/test/test_sol_stream_log_gate.cpp`:

```cpp
#include <gtest/gtest.h>
#include "gnss_bringup/sol_stream_log_gate.hpp"
using gnss_bringup::SolStreamLogGate;

TEST(SolStreamLogGate, FirstConnectIsPrinted) {
  SolStreamLogGate g;
  EXPECT_TRUE(g.on_status(true, "connected to 127.0.0.1:15020"));
}

TEST(SolStreamLogGate, OnlyTheFirstIdleCycleIsPrinted) {
  // 隧道里 rtkrcv 长时间没有解算输出:TcpStream 每 sol_idle_timeout_s 报一次
  // idle timeout 并立刻重连。只打第一次,后面整夜的重复都降级。
  SolStreamLogGate g;
  g.on_status(true, "connected to 127.0.0.1:15020");
  EXPECT_TRUE(g.on_status(false, "idle timeout"));
  for (int i = 0; i < 5; ++i) {
    EXPECT_FALSE(g.on_status(true, "connected to 127.0.0.1:15020"));
    EXPECT_FALSE(g.on_status(false, "idle timeout"));
  }
  EXPECT_TRUE(g.quiet());
}

TEST(SolStreamLogGate, ASolutionLineEndsTheQuietPeriod) {
  SolStreamLogGate g;
  g.on_status(false, "idle timeout");
  g.on_solution_line();
  EXPECT_FALSE(g.quiet());
  EXPECT_TRUE(g.on_status(false, "idle timeout")) << "解算恢复之后再次空闲,要重新提示一次";
}

TEST(SolStreamLogGate, PeerCloseIsAlwaysPrintedAndEndsTheQuietPeriod) {
  // 对端关闭说明 rtkrcv 本身退出/重启了,不能被空闲安静期吞掉
  SolStreamLogGate g;
  g.on_status(false, "idle timeout");
  EXPECT_TRUE(g.on_status(false, "peer closed connection"));
  EXPECT_FALSE(g.quiet());
  EXPECT_TRUE(g.on_status(true, "connected to 127.0.0.1:15020"));
}

TEST(SolStreamLogGate, ConnectFailuresAreAlwaysPrinted) {
  SolStreamLogGate g;
  g.on_status(false, "idle timeout");
  EXPECT_TRUE(g.on_status(false, "connect() failed: Connection refused"));
}
```

CMake(`if(BUILD_TESTING)` 内):

```cmake
  # sol 流状态日志去重闸门:纯逻辑,不碰 ROS
  ament_add_gtest(test_sol_stream_log_gate test/test_sol_stream_log_gate.cpp)
  target_include_directories(test_sol_stream_log_gate PRIVATE include)
```

Run: `cd /home/steve/glim_ws && colcon build --symlink-install --packages-select gnss_bringup`
Expected: 编译失败(头文件不存在)——这就是 RED。

- [ ] **Step 2: 实现 SolStreamLogGate**

`gnss_bringup/include/gnss_bringup/sol_stream_log_gate.hpp`:

```cpp
#pragma once
#include <string>

namespace gnss_bringup {

// rtkrcv_node 的 sol 流状态日志去重闸门(纯逻辑,调用方负责加锁)。
//
// 隧道里 rtkrcv 长时间没有解算输出时,TcpStream 每 sol_idle_timeout_s 报一次
// "idle timeout" 断开、紧接着一次 connected;不加控制会在 INFO 级别刷一整夜,
// 把真正要看的日志淹掉。规则:
//   - 第一次 idle timeout 照常打印,并进入安静期;
//   - 安静期内的 idle timeout 与随后的 connected 都降为 DEBUG;
//   - 收到一条有效解算行,或出现 idle timeout 以外的断开(对端关闭 / 连不上,
//     说明 rtkrcv 本身出了状况)时退出安静期,并且那条断开照常打印。
class SolStreamLogGate {
public:
  // 返回 true:按原级别打印;false:降为 DEBUG。
  bool on_status(bool connected, const std::string& detail) {
    if (connected) return !quiet_;
    if (detail == "idle timeout") {
      if (quiet_) return false;
      quiet_ = true;
      return true;
    }
    quiet_ = false;
    return true;
  }

  void on_solution_line() { quiet_ = false; }
  bool quiet() const { return quiet_; }

private:
  bool quiet_ = false;
};

}  // namespace gnss_bringup
```

Run: `cd /home/steve/glim_ws && colcon build --symlink-install --packages-select gnss_bringup && source install/setup.bash && ./build/gnss_bringup/test_sol_stream_log_gate`
Expected: PASS。

- [ ] **Step 3: 节点接线**

`gnss_bringup/src/rtkrcv_node.cpp`:

1. include 区加 `#include <cstdlib>`、`#include <cstring>`(若尚未包含)、`#include "gnss_bringup/executable_lookup.hpp"`、`#include "gnss_bringup/sol_stream_log_gate.hpp"`。

2. 构造函数:在 `check_sol_port_free();` 之后插入 `check_binary_resolvable();`;删除 `start_pid_logger();`,删除 `catch (...)` 里的 `stop_pid_logger();`。析构函数删除 `stop_pid_logger();`。删除 `start_pid_logger()`、`stop_pid_logger()`、`pid_log_loop()` 三个函数与成员 `pid_log_thread_`、`pid_log_running_`、`pid_log_mutex_`、`pid_log_cv_`(派生日志改由 `on_spawn` 回调负责)。同步修改文件头部注释里列出的启动步骤(删掉"spawned pid 日志"那一项,加上"检查 binary 可解析")。

3. `read_params()`,在 `conf_.ar_mode = ...` 之后加

```cpp
    conf_.base_pos_type = node_->declare_parameter<std::string>("base_pos_type", conf_.base_pos_type);
    conf_.bds_ar_mode = node_->declare_parameter<std::string>("bds_ar_mode", conf_.bds_ar_mode);
    conf_.glo_ar_mode = node_->declare_parameter<std::string>("glo_ar_mode", conf_.glo_ar_mode);
```

4. 新增成员函数(放在 `check_sol_port_free()` 之后):

```cpp
  // 启动前就确认 binary 能被解析:rtkrcv 没装、或写成了找不到的名字时,
  // 直接拒绝启动并说清楚,而不是起来之后在监管线程里无限退避重试。
  void check_binary_resolvable() {
    const char* path_env = std::getenv("PATH");
    if (gnss_bringup::resolve_executable(binary_, path_env).empty()) {
      throw std::invalid_argument(
          "binary=" + binary_ + " 找不到或不可执行(PATH=" +
          (path_env ? path_env : "<未设置>") +
          ")。rtkrcv 需要 RTKLIB-EX 2.5.1,安装方法见 gnss_bringup/README.md「安装 RTKLIB-EX 2.5.1」");
    }
  }

  void log_child_exit(const ChildExitInfo& e) {
    if (e.spawn_failed) {
      RCLCPP_ERROR(node_->get_logger(), "rtkrcv 无法启动:%s;%.1f s 后重试",
                   e.detail.c_str(), e.next_delay_s);
      return;
    }
    if (!e.will_restart) {
      RCLCPP_INFO(node_->get_logger(), "rtkrcv 已随节点停止 pid=%d", e.pid);
      return;
    }
    std::string how = "状态未知";
    if (e.exited) {
      how = "code=" + std::to_string(e.exit_code);
    } else if (e.signaled) {
      how = "signal=" + std::to_string(e.signal) + "(" + ::strsignal(e.signal) + ")";
    }
    const bool crash_loop = e.lifetime_s < crash_loop_life_s_;
    RCLCPP_WARN(node_->get_logger(), "rtkrcv 退出 pid=%d %s,存活 %.1f s,%.1f s 后重启%s",
                e.pid, how.c_str(), e.lifetime_s, e.next_delay_s,
                crash_loop ? "——疑似崩溃循环:检查 run_dir 下的 rtkrcv.conf 与 rtkrcv 版本"
                             "(需要 RTKLIB-EX 2.5.1;旧版不认 -nc,会打印用法后以 0 退出)"
                           : "");
  }
```

5. `start_supervisor()`:在 `supervisor_ = std::make_unique<ProcessSupervisor>(scfg);` 之前加

```cpp
    // 两个回调在监管线程上执行:只打日志,不碰 supervisor_ 本身
    scfg.on_spawn = [this](int pid, const std::string& executable) {
      RCLCPP_INFO(node_->get_logger(), "rtkrcv 已启动 pid=%d (%s)", pid, executable.c_str());
    };
    scfg.on_exit = [this](const ChildExitInfo& e) { log_child_exit(e); };
```

   (析构函数里 `supervisor_->stop()` 先于其它成员销毁执行,回调捕获的 `this` 在监管线程退出前一直有效。)

6. `connect_solution_stream()` 的状态回调改为:

```cpp
        [this](bool connected, const std::string& detail, bool terminal) {
          bool print = true;
          {
            std::lock_guard<std::mutex> lk(sol_log_mutex_);
            print = sol_log_gate_.on_status(connected, detail);
          }
          if (terminal) {
            RCLCPP_ERROR(node_->get_logger(),
                         "sol stream: %s %s(worker 线程已永久退出,不会再有任何"
                         "重连尝试,需要人工介入/重启节点)",
                         connected ? "connected" : "disconnected", detail.c_str());
          } else if (print) {
            const bool idle = !connected && detail == "idle timeout";
            RCLCPP_INFO(node_->get_logger(), "sol stream: %s %s%s",
                        connected ? "connected" : "disconnected", detail.c_str(),
                        idle ? "(rtkrcv 暂无解算输出,隧道内属正常;收到下一条解之前"
                               "不再重复打印空闲重连)"
                             : "");
          } else {
            RCLCPP_DEBUG(node_->get_logger(), "sol stream: %s %s",
                         connected ? "connected" : "disconnected", detail.c_str());
          }
          if (!connected) {
            sol_splitter_.reset();
          }
        });
```

7. `on_solution_bytes()`:在 `if (!gnss_core::parse_llh_solution(line, rec, pos_opts_)) continue;` 之后加

```cpp
      {
        std::lock_guard<std::mutex> lk(sol_log_mutex_);
        sol_log_gate_.on_solution_line();
      }
```

8. 成员区加 `SolStreamLogGate sol_log_gate_;` 与 `std::mutex sol_log_mutex_;`(`TcpStream` 的数据回调与状态回调是否同线程,不做假设,统一加锁)。

- [ ] **Step 4: yaml**

`gnss_bringup/config/gnss_bringup.yaml` 的 `rtkrcv_node:` 段:

把 `binary: "rtkrcv"            # RTKLIB demo5;开发机未安装,见 README 缺口登记` 改为

```yaml
    # RTKLIB-EX 2.5.1 的 rtkrcv。裸名字按 PATH 查找(sudo cmake --install 装在
    # /usr/local/bin);找不到时节点启动即失败并打印 PATH。安装方法见 README
    # 「安装 RTKLIB-EX 2.5.1」——不要用 apt 的 rtklib(2.4.3 b34 不认 -nc)。
    binary: "rtkrcv"
```

在 `ar_mode: "continuous"` 之后加

```yaml
    # 基准站坐标来源(conf 的 ant2-postype)。"rtcm":取平台差分流里的 RTCM
    # 1005/1006(默认——前提是差分流带这两条电文,现场待确认);"single":用基准站
    # 观测的单点解。不写这个键时 rtkrcv 默认 llh 0,0,0,RTK 一条解都不会输出
    # (2026-09-14 用 RTKLIB-EX 2.5.1 实测)。
    base_pos_type: "rtcm"
    # 北斗 / GLONASS 模糊度固定。取值与 RTKLIB-EX 2.5.1 自身默认一致,显式写出是
    # 为了换版本时默认值不会悄悄变化;现场按固定率再调(bds_ar_mode: off|on,
    # glo_ar_mode: off|on|autocal|fix-and-hold)。
    bds_ar_mode: "off"
    glo_ar_mode: "fix-and-hold"
```

- [ ] **Step 5: README**

`gnss_bringup/README.md`:在「## 快速开始」之前新增一节:

````markdown
## 安装 RTKLIB-EX 2.5.1(`rtkrcv`)

`rtkrcv_node` 需要 **RTKLIB-EX 2.5.1**(rtklibexplorer 维护,原 demo5)。
**不要用 `apt install rtklib`**:Ubuntu 22.04 源里是 Takasu 原版 2.4.3 b34,不认 `-nc`,
遇到就打印用法并以 0 退出,节点会陷入崩溃循环。

```bash
# 源码:https://github.com/rtklibexplorer/RTKLIB/releases/tag/v2.5.1
cd RTKLIB-2.5.1
# 只要命令行工具:关掉 Qt(系统 Qt6 缺 SerialPort 模块会让配置失败)。
# 需要 GUI 时去掉最后一个 -D,改传 -DCMAKE_PREFIX_PATH=<带 SerialPort 的 Qt6 目录>。
cmake --fresh -S . -B build -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
  -DCMAKE_DISABLE_FIND_PACKAGE_QT=TRUE
cmake --build build -j"$(nproc)"
sudo cmake --install build
sudo ldconfig        # 必须:librtklib.so 装在 /usr/local/lib,不刷新缓存 rtkrcv 起不来
rtkrcv --version     # 应输出:rtkrcv RTKLIB EX 2.5.1
```

`binary` 参数默认 `rtkrcv`,节点按 `PATH` 查找;找不到时节点启动即失败,日志里带着当时的 `PATH`。
````

在「## 参数说明」的列表里(`leap_seconds` 条目之前)加一条:

```markdown
- **`base_pos_type`(默认 `rtcm`)**——基准站坐标来源,对应 conf 的 `ant2-postype`。
  `rtcm` 要求平台差分流带 RTCM 1005/1006;没有时改成 `single`(基准站单点解,精度差)。
  **这一项不写时 rtkrcv 默认坐标 0,0,0,RTK 一条解都不输出**。`bds_ar_mode`/`glo_ar_mode`
  是北斗/GLONASS 模糊度固定开关,默认值与 RTKLIB-EX 2.5.1 一致,现场按固定率调整。
  所有枚举参数在节点启动时按 RTKLIB-EX 2.5.1 的取值表校验,写错直接拒绝启动——
  rtkrcv 自己遇到非法取值只会悄悄回落到默认值继续跑。
```

- [ ] **Step 6: 手工冒烟(不需要 GNSS 数据)**

```bash
cd /home/steve/glim_ws && colcon build --symlink-install --packages-select gnss_bringup && source install/setup.bash
T=$(mktemp -d) && mkdir -p $T/run
ROS_DOMAIN_ID=77 ROS_LOG_DIR=$T/log timeout -s INT 6 ros2 run gnss_bringup rtkrcv_node --ros-args \
  --params-file install/gnss_bringup/share/gnss_bringup/config/gnss_bringup.yaml \
  -p run_dir:=$T/run -p binary:=/nonexistent/rtkrcv 2>&1 | tail -3
```

Expected: 进程退出码 1(`ros2run` 报 failure),日志含 `找不到或不可执行`,`$T/run/rtkrcv.conf` 不存在。

```bash
ROS_DOMAIN_ID=77 ROS_LOG_DIR=$T/log timeout -s INT 8 ros2 run gnss_bringup rtkrcv_node --ros-args \
  --params-file install/gnss_bringup/share/gnss_bringup/config/gnss_bringup.yaml \
  -p run_dir:=$T/run > $T/node.log 2>&1; grep -E "已启动 pid=|退出 pid=|sol stream" $T/node.log; grep ant2-postype $T/run/rtkrcv.conf
pgrep -a -x rtkrcv || echo "no rtkrcv left"; rm -rf $T
```

Expected: 含 `rtkrcv 已启动 pid=<n> (/usr/local/bin/rtkrcv)` 与 `sol stream: connected`,没有 `rtkrcv 退出 pid=`;conf 含 `ant2-postype =rtcm`;结束后没有残留 rtkrcv。

- [ ] **Step 7: 全量测试**

Run: `cd /home/steve/glim_ws && colcon test --packages-select gnss_core gnss_bringup && colcon test-result --all`
Expected: 0 failures。

- [ ] **Step 8: Commit**

```bash
cd /home/steve/glim_ws/src/glim_ext
git add gnss_bringup/include/gnss_bringup/sol_stream_log_gate.hpp gnss_bringup/test/test_sol_stream_log_gate.cpp gnss_bringup/src/rtkrcv_node.cpp gnss_bringup/CMakeLists.txt gnss_bringup/config/gnss_bringup.yaml gnss_bringup/README.md
git commit -m "feat(gnss_bringup): refuse an unresolvable rtkrcv, log every rtkrcv exit, and stop idle sol-stream log spam

<写明手工冒烟结果>

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 5: 节点级测试(把 `rtkrcv_node` 当子进程起)

`rtkrcv_node.cpp` 以前没有任何测试链接或运行它(评审提过 5 次)。本 Task 用 gtest 把节点可执行文件当子进程起,配 `test/fake_rtkrcv.sh`,钉住启动顺序、PATH 解析、退出日志与 conf 内容。

**Files:**
- Create: `gnss_bringup/test/node_process_harness.hpp`
- Create: `gnss_bringup/test/test_rtkrcv_node_process.cpp`
- Modify: `gnss_bringup/CMakeLists.txt`

**Interfaces:**
- Consumes: Task 4 列出的日志子串与退出码约定
- Produces(Task 6 复用),命名空间 `gnss_bringup_test`:
  - `std::string make_temp_dir(const std::string& prefix);`
  - `int pick_free_port();`
  - `std::string read_file(const std::string& path);`
  - `std::size_t count_occurrences(const std::string& hay, const std::string& needle);`
  - `bool wait_until(const std::function<bool()>& pred, double timeout_s);`
  - `class ListeningSocket { public: ListeningSocket(); int port() const; };`
  - `class NodeProcess { public: NodeProcess(const std::string& exe, const std::vector<std::string>& args, const std::string& log_path, const std::vector<std::pair<std::string, std::string>>& env_overrides); int wait_exit(double timeout_s); void interrupt(); std::string log() const; pid_t pid() const; };`

- [ ] **Step 1: 写 harness**

`gnss_bringup/test/node_process_harness.hpp`:

```cpp
#pragma once
// 测试专用:把一个节点可执行文件当子进程起起来、把 stdout/stderr 抓进日志文件、
// 等它退出。fork 之后子进程里只调用 async-signal-safe 的 open/dup2/execve/_exit
// ——调用方(比如 Task 6 的测试)可能已经起了 rclcpp 的线程。
#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <signal.h>
#include <sys/socket.h>
#include <sys/wait.h>
#include <unistd.h>

#include <chrono>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <functional>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

extern char** environ;

namespace gnss_bringup_test {

inline std::string make_temp_dir(const std::string& prefix) {
  const char* base = std::getenv("TMPDIR");
  std::string tmpl = std::string(base ? base : "/tmp") + "/" + prefix + "XXXXXX";
  std::vector<char> buf(tmpl.begin(), tmpl.end());
  buf.push_back('\0');
  if (::mkdtemp(buf.data()) == nullptr) return {};
  return std::string(buf.data());
}

// 让内核挑一个空闲端口再关掉。有极小的竞态窗口,测试里可以接受。
inline int pick_free_port() {
  const int fd = ::socket(AF_INET, SOCK_STREAM, 0);
  if (fd < 0) return -1;
  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  addr.sin_port = 0;
  int port = -1;
  if (::bind(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) == 0) {
    socklen_t len = sizeof(addr);
    if (::getsockname(fd, reinterpret_cast<sockaddr*>(&addr), &len) == 0) port = ntohs(addr.sin_port);
  }
  ::close(fd);
  return port;
}

inline std::string read_file(const std::string& path) {
  std::ifstream in(path, std::ios::binary);
  std::ostringstream ss;
  ss << in.rdbuf();
  return ss.str();
}

inline std::size_t count_occurrences(const std::string& hay, const std::string& needle) {
  std::size_t n = 0;
  for (std::size_t pos = hay.find(needle); pos != std::string::npos; pos = hay.find(needle, pos + needle.size())) ++n;
  return n;
}

inline bool wait_until(const std::function<bool()>& pred, double timeout_s) {
  const auto end = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout_s);
  while (std::chrono::steady_clock::now() < end) {
    if (pred()) return true;
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  return pred();
}

// 在 127.0.0.1 上占住一个端口并 listen,析构时释放——模拟"孤儿 rtkrcv 占着 sol_port"。
class ListeningSocket {
public:
  ListeningSocket() {
    fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    addr.sin_port = 0;
    if (fd_ >= 0 && ::bind(fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) == 0 &&
        ::listen(fd_, 4) == 0) {
      socklen_t len = sizeof(addr);
      if (::getsockname(fd_, reinterpret_cast<sockaddr*>(&addr), &len) == 0) port_ = ntohs(addr.sin_port);
    }
  }
  ~ListeningSocket() { if (fd_ >= 0) ::close(fd_); }
  ListeningSocket(const ListeningSocket&) = delete;
  ListeningSocket& operator=(const ListeningSocket&) = delete;
  int port() const { return port_; }

private:
  int fd_ = -1;
  int port_ = -1;
};

class NodeProcess {
public:
  NodeProcess(const std::string& exe, const std::vector<std::string>& args, const std::string& log_path,
              const std::vector<std::pair<std::string, std::string>>& env_overrides)
      : log_path_(log_path) {
    // argv / envp 全部在 fork 之前建好
    std::vector<std::string> argv_s{exe};
    argv_s.insert(argv_s.end(), args.begin(), args.end());
    std::vector<std::string> env_s;
    for (char** e = environ; *e != nullptr; ++e) {
      const std::string kv(*e);
      bool overridden = false;
      for (const auto& [k, v] : env_overrides) {
        if (kv.rfind(k + "=", 0) == 0) overridden = true;
      }
      if (!overridden) env_s.push_back(kv);
    }
    for (const auto& [k, v] : env_overrides) env_s.push_back(k + "=" + v);
    std::vector<char*> argv, envp;
    for (auto& s : argv_s) argv.push_back(s.data());
    argv.push_back(nullptr);
    for (auto& s : env_s) envp.push_back(s.data());
    envp.push_back(nullptr);

    pid_ = ::fork();
    if (pid_ == 0) {
      const int fd = ::open(log_path_.c_str(), O_WRONLY | O_CREAT | O_TRUNC, 0644);
      if (fd >= 0) {
        ::dup2(fd, STDOUT_FILENO);
        ::dup2(fd, STDERR_FILENO);
      }
      ::execve(argv[0], argv.data(), envp.data());
      _exit(127);
    }
  }

  ~NodeProcess() {
    if (pid_ > 0) {
      ::kill(pid_, SIGKILL);
      ::waitpid(pid_, nullptr, 0);
    }
  }
  NodeProcess(const NodeProcess&) = delete;
  NodeProcess& operator=(const NodeProcess&) = delete;

  // 返回退出码;被信号终止返回 128+信号;超时则 SIGKILL 并返回 -1。
  int wait_exit(double timeout_s) {
    if (pid_ <= 0) return -1;
    const auto end = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout_s);
    for (;;) {
      int status = 0;
      const pid_t w = ::waitpid(pid_, &status, WNOHANG);
      if (w == pid_) {
        pid_ = -1;
        if (WIFEXITED(status)) return WEXITSTATUS(status);
        if (WIFSIGNALED(status)) return 128 + WTERMSIG(status);
        return -1;
      }
      if (std::chrono::steady_clock::now() >= end) {
        ::kill(pid_, SIGKILL);
        ::waitpid(pid_, nullptr, 0);
        pid_ = -1;
        return -1;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
  }

  void interrupt() { if (pid_ > 0) ::kill(pid_, SIGINT); }
  std::string log() const { return read_file(log_path_); }
  pid_t pid() const { return pid_; }

private:
  std::string log_path_;
  pid_t pid_ = -1;
};

}  // namespace gnss_bringup_test
```

- [ ] **Step 2: 写测试**

`gnss_bringup/test/test_rtkrcv_node_process.cpp`:

```cpp
#include <gtest/gtest.h>
#include <signal.h>
#include <unistd.h>
#include <cerrno>
#include <cstdlib>
#include <filesystem>
#include <string>
#include <vector>
#include "node_process_harness.hpp"
using namespace gnss_bringup_test;

namespace {
const char* node_exe() { return RTKRCV_NODE_PATH; }
const char* fake() { return FAKE_RTKRCV_PATH; }

std::vector<std::pair<std::string, std::string>> isolated_env(const std::string& dir) {
  return {{"ROS_DOMAIN_ID", std::to_string(40 + ::getpid() % 50)}, {"ROS_LOG_DIR", dir + "/roslog"}};
}

std::vector<std::string> node_args(const std::string& dir, const std::string& binary, int sol_port,
                                   const std::string& mode_arg) {
  return {"--ros-args",
          "-p", "binary:=" + binary,
          "-p", "run_dir:=" + dir + "/run",
          "-p", "sol_port:=" + std::to_string(sol_port),
          "-p", "corr_port:=" + std::to_string(pick_free_port()),
          "-p", "obs_port:=" + std::to_string(pick_free_port()),
          "-p", "restart_delay_s:=0.1",
          "-p", "crash_loop_life_s:=5.0",
          "-p", "max_restart_delay_s:=0.4",
          "-p", "args:=['" + mode_arg + "']"};
}

int pid_after(const std::string& log, const std::string& marker) {
  const auto pos = log.find(marker);
  if (pos == std::string::npos) return -1;
  return std::atoi(log.c_str() + pos + marker.size());
}
}  // namespace

TEST(RtkrcvNodeProcess, BusySolPortRefusesToStartAndWritesNoConf) {
  const std::string dir = make_temp_dir("rtkrcv_node_busy_");
  ASSERT_FALSE(dir.empty());
  std::filesystem::create_directories(dir + "/run");
  ListeningSocket orphan;
  ASSERT_GT(orphan.port(), 0);
  NodeProcess node(node_exe(), node_args(dir, fake(), orphan.port(), "live"), dir + "/node.log", isolated_env(dir));
  EXPECT_EQ(node.wait_exit(20.0), 1) << node.log();
  EXPECT_NE(node.log().find("孤儿"), std::string::npos) << node.log();
  EXPECT_FALSE(std::filesystem::exists(dir + "/run/rtkrcv.conf")) << "端口检查必须先于写 conf";
  std::filesystem::remove_all(dir);
}

TEST(RtkrcvNodeProcess, UnresolvableBinaryRefusesToStartAndWritesNoConf) {
  const std::string dir = make_temp_dir("rtkrcv_node_nobin_");
  ASSERT_FALSE(dir.empty());
  std::filesystem::create_directories(dir + "/run");
  NodeProcess node(node_exe(), node_args(dir, "/nonexistent/rtkrcv", pick_free_port(), "live"),
                   dir + "/node.log", isolated_env(dir));
  EXPECT_EQ(node.wait_exit(20.0), 1) << node.log();
  EXPECT_NE(node.log().find("找不到或不可执行"), std::string::npos) << node.log();
  EXPECT_FALSE(std::filesystem::exists(dir + "/run/rtkrcv.conf"));
  std::filesystem::remove_all(dir);
}

TEST(RtkrcvNodeProcess, BareBinaryNameIsResolvedThroughPathAndStaysUp) {
  // 回归:yaml 默认 binary: "rtkrcv" 是裸名字,以前永远起不来
  const std::string dir = make_temp_dir("rtkrcv_node_path_");
  ASSERT_FALSE(dir.empty());
  std::filesystem::create_directories(dir + "/run");
  const std::string full = fake();
  const std::string fake_dir = full.substr(0, full.rfind('/'));
  const std::string fake_name = full.substr(full.rfind('/') + 1);
  const char* old_path = std::getenv("PATH");
  auto env = isolated_env(dir);
  env.emplace_back("PATH", fake_dir + ":" + (old_path ? old_path : ""));

  NodeProcess node(node_exe(), node_args(dir, fake_name, pick_free_port(), "live"), dir + "/node.log", env);
  ASSERT_TRUE(wait_until([&] { return node.log().find("rtkrcv 已启动 pid=") != std::string::npos; }, 20.0))
      << node.log();
  const int child = pid_after(node.log(), "rtkrcv 已启动 pid=");
  ASSERT_GT(child, 0);
  std::this_thread::sleep_for(std::chrono::seconds(1));
  EXPECT_EQ(::kill(child, 0), 0) << "按 PATH 找到的子进程必须一直活着\n" << node.log();
  EXPECT_EQ(count_occurrences(node.log(), "rtkrcv 退出 pid="), 0u) << node.log();

  node.interrupt();
  EXPECT_EQ(node.wait_exit(20.0), 0) << node.log();
  EXPECT_TRUE(wait_until([&] { return ::kill(child, 0) != 0 && errno == ESRCH; }, 5.0))
      << "节点退出后子进程必须被收掉";
  std::filesystem::remove_all(dir);
}

TEST(RtkrcvNodeProcess, EveryChildExitIsLogged) {
  // 回归:以前崩溃循环完全静默(实测 20 s 重启 6 次,节点日志 0 行)
  const std::string dir = make_temp_dir("rtkrcv_node_die_");
  ASSERT_FALSE(dir.empty());
  std::filesystem::create_directories(dir + "/run");
  NodeProcess node(node_exe(), node_args(dir, fake(), pick_free_port(), "die"), dir + "/node.log", isolated_env(dir));
  EXPECT_TRUE(wait_until([&] { return count_occurrences(node.log(), "rtkrcv 退出 pid=") >= 3; }, 20.0))
      << node.log();
  EXPECT_NE(node.log().find("疑似崩溃循环"), std::string::npos) << node.log();
  node.interrupt();
  EXPECT_EQ(node.wait_exit(20.0), 0) << node.log();
  std::filesystem::remove_all(dir);
}

TEST(RtkrcvNodeProcess, WrittenConfTakesTheBasePositionFromRtcm) {
  const std::string dir = make_temp_dir("rtkrcv_node_conf_");
  ASSERT_FALSE(dir.empty());
  std::filesystem::create_directories(dir + "/run");
  NodeProcess node(node_exe(), node_args(dir, fake(), pick_free_port(), "live"), dir + "/node.log", isolated_env(dir));
  const std::string conf = dir + "/run/rtkrcv.conf";
  ASSERT_TRUE(wait_until([&] { return std::filesystem::exists(conf); }, 20.0)) << node.log();
  ASSERT_TRUE(wait_until([&] { return node.log().find("rtkrcv 已启动 pid=") != std::string::npos; }, 20.0));
  EXPECT_NE(read_file(conf).find("ant2-postype =rtcm\n"), std::string::npos) << read_file(conf);
  node.interrupt();
  EXPECT_EQ(node.wait_exit(20.0), 0) << node.log();
  std::filesystem::remove_all(dir);
}
```

CMake(`if(BUILD_TESTING)` 内):

```cmake
  # 节点级测试:把 rtkrcv_node 可执行文件当子进程起,配 fake_rtkrcv.sh。
  # 用 gtest 而不是 launch_testing:开发机的 pytest 9.1.1 与 Humble 的
  # launch_testing 插件不兼容。
  ament_add_gtest(test_rtkrcv_node_process test/test_rtkrcv_node_process.cpp TIMEOUT 180)
  target_include_directories(test_rtkrcv_node_process PRIVATE include test)
  add_dependencies(test_rtkrcv_node_process rtkrcv_node)
  target_compile_definitions(test_rtkrcv_node_process PRIVATE
    RTKRCV_NODE_PATH="$<TARGET_FILE:rtkrcv_node>"
    FAKE_RTKRCV_PATH="${CMAKE_CURRENT_SOURCE_DIR}/test/fake_rtkrcv.sh")
```

- [ ] **Step 3: 跑测试,并用变异确认每条都诚实**

Run: `cd /home/steve/glim_ws && colcon build --symlink-install --packages-select gnss_bringup && source install/setup.bash && ./build/gnss_bringup/test_rtkrcv_node_process`
Expected: 5 个测试全部 PASS。

本 Task 的实现已由 Task 2-4 完成,RED 用变异证明(每个做完都恢复、重新构建):
- 构造函数里把 `check_sol_port_free();` 挪到 `write_conf();` 之后 → `BusySolPortRefusesToStartAndWritesNoConf` 必须 FAIL(conf 存在)
- 删掉构造函数里的 `check_binary_resolvable();` → `UnresolvableBinaryRefusesToStartAndWritesNoConf` 必须 FAIL
- `process_supervisor.cpp` 子进程分支改回 `execv(cfg_.binary.c_str(), ...)` → `BareBinaryNameIsResolvedThroughPathAndStaysUp` 必须 FAIL
- 把 `scfg.on_exit = ...` 那一行删掉 → `EveryChildExitIsLogged` 必须 FAIL
- `rtkrcv_conf.cpp` 删掉 `ant2-postype` 输出 → `WrittenConfTakesTheBasePositionFromRtcm` 必须 FAIL

另外连续跑 5 遍 `./build/gnss_bringup/test_rtkrcv_node_process`,必须 5 遍全过(子进程测试容易有时序抖动;若出现偶发失败,修测试的等待条件,不要加 sleep 掩盖)。测完确认 `pgrep -a fake_rtkrcv` 与 `pgrep -a rtkrcv_node` 都为空,`ls /tmp | grep rtkrcv_node_` 为空。

- [ ] **Step 4: 全量测试**

Run: `cd /home/steve/glim_ws && colcon test --packages-select gnss_core gnss_bringup && colcon test-result --all`
Expected: 0 failures。

- [ ] **Step 5: Commit**

```bash
cd /home/steve/glim_ws/src/glim_ext
git add gnss_bringup/test/node_process_harness.hpp gnss_bringup/test/test_rtkrcv_node_process.cpp gnss_bringup/CMakeLists.txt
git commit -m "test(gnss_bringup): first node-level tests for rtkrcv_node, run as a child process

<写明五个变异各自的结果与 5 遍连续运行结果>

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 6: 真实 rtkrcv 端到端回归(回放双站 RTCM3)

**Files:**
- Create: `gnss_bringup/test/data/rnx2rtcm.c`、`gnss_bringup/test/data/README.md`
- Create: `gnss_bringup/test/data/rtcm_20050402_0759_rover.rtcm3`、`gnss_bringup/test/data/rtcm_20050402_3040_base.rtcm3`
- Create: `gnss_bringup/test/rtkrcv_faketime.sh`
- Create: `gnss_bringup/test/test_rtkrcv_real_binary.cpp`
- Modify: `gnss_bringup/CMakeLists.txt`、`gnss_bringup/README.md`(「## 未验证项」)

**Interfaces:**
- Consumes: Task 5 的 `node_process_harness.hpp` 全部接口;Task 2 的 `resolve_executable`;Task 4 的日志子串 `sol stream: connected`

- [ ] **Step 1: 生成测试数据**

`gnss_bringup/test/data/rnx2rtcm.c`(不参与本包构建,只用来重新生成数据):

```c
/* RINEX obs+nav -> RTCM3:每个历元一条 1004,每 10 个历元补发 1019(GPS 星历)+ 1005(基准站坐标)。
 * 只用于生成 gnss_bringup 的回放测试数据,编译方法见同目录 README.md。 */
#include <stdio.h>
#include <string.h>
#include "rtklib.h"

int main(int argc, char **argv) {
    if (argc < 4) { fprintf(stderr, "usage: %s obs nav out.rtcm3\n", argv[0]); return 1; }
    static obs_t obs; static nav_t nav; static sta_t sta; static obs_t dum; static rtcm_t rtcm;
    if (readrnx(argv[1], 1, "", &obs, &nav, &sta) < 0) { fprintf(stderr, "obs read failed\n"); return 1; }
    if (readrnx(argv[2], 1, "", &dum, &nav, NULL) < 0) { fprintf(stderr, "nav read failed\n"); return 1; }
    sortobs(&obs);
    if (!init_rtcm(&rtcm)) return 1;
    rtcm.sta = sta; rtcm.staid = 1;
    FILE *fp = fopen(argv[3], "wb"); if (!fp) return 1;
    int ep = 0, n1004 = 0, n1019 = 0, n1005 = 0;
    for (int i = 0; i < obs.n; ep++) {
        int m = 0; gtime_t t = obs.data[i].time;
        while (i + m < obs.n && fabs(timediff(obs.data[i + m].time, t)) < 1e-3) m++;
        if (ep % 10 == 0) {
            for (int s = 1; s <= MAXSAT; s++) {
                if (satsys(s, NULL) != SYS_GPS) continue;
                int best = -1; double bd = 1e9;
                for (int k = 0; k < nav.n; k++) {
                    if (nav.eph[k].sat != s) continue;
                    double d = fabs(timediff(nav.eph[k].toe, t));
                    if (d < bd) { bd = d; best = k; }
                }
                if (best < 0 || bd > 7200.0) continue;
                rtcm.nav.eph[s - 1] = nav.eph[best]; rtcm.ephsat = s; rtcm.ephset = 0;
                if (gen_rtcm3(&rtcm, 1019, 0, 0)) { fwrite(rtcm.buff, rtcm.nbyte, 1, fp); n1019++; }
            }
            if (gen_rtcm3(&rtcm, 1005, 0, 0)) { fwrite(rtcm.buff, rtcm.nbyte, 1, fp); n1005++; }
        }
        rtcm.time = t; rtcm.obs.n = m;
        memcpy(rtcm.obs.data, obs.data + i, sizeof(obsd_t) * m);
        if (gen_rtcm3(&rtcm, 1004, 0, 0)) { fwrite(rtcm.buff, rtcm.nbyte, 1, fp); n1004++; }
        i += m;
    }
    fclose(fp);
    printf("%s: epochs=%d 1004=%d 1019=%d 1005=%d\n", argv[3], ep, n1004, n1019, n1005);
    return 0;
}
```

生成:

```bash
RT=/home/steve/Documents/GitHub/gnss-alg/RTKLIB-2.5.1
D=/home/steve/glim_ws/src/glim_ext/gnss_bringup/test/data
B=$(mktemp -d)
gcc -O2 -DDLL -DENACMP -DENAGAL -DENAGLO -DENAIRN -DENAQZS -DNEXOBS=3 -DNFREQ=3 -DTRACE \
  -I$RT/src -o $B/rnx2rtcm $D/rnx2rtcm.c -L/usr/local/lib -lrtklib -lm -lpthread
$B/rnx2rtcm $RT/test/data/rinex/07590920.05o $RT/test/data/rinex/07590920.05n $D/rtcm_20050402_0759_rover.rtcm3
$B/rnx2rtcm $RT/test/data/rinex/30400920.05o $RT/test/data/rinex/07590920.05n $D/rtcm_20050402_3040_base.rtcm3
sha256sum $D/*.rtcm3
rm -rf $B
```

Expected:两行都输出 `epochs=120 1004=120 1019=192 1005=12`;

```
407839810c3b08e4a748cc8975cc8def3e998bf6bc41f2493db3d3bdcf8a5a4d  .../rtcm_20050402_0759_rover.rtcm3
4568755b6951d4acbe88256292d8911523c5a0df32cceea8b7927b1c241ecbd5  .../rtcm_20050402_3040_base.rtcm3
```

(`-D` 宏必须与 `librtklib.so` 构建时一致,否则 `obs_t`/`nav_t` 结构体布局对不上。)

`gnss_bringup/test/data/README.md`:

```markdown
# 回放测试数据

`test_rtkrcv_real_binary` 用的两路 RTCM3,由 RTKLIB-EX 2.5.1 源码树自带的 GSI 两站 RINEX
(`test/data/rinex/07590920.05o`、`30400920.05o`,2005-04-02 00:00–00:59:30 GPST,30 s 间隔,
基线约 3.3 km;RTKLIB 以 BSD-2-Clause 发布)经 `rnx2rtcm.c` 转换:

| 文件 | 测站 | 用途 |
|---|---|---|
| `rtcm_20050402_0759_rover.rtcm3` | 0759 | 流动站原始观测(喂 `/gnss/raw_obs`) |
| `rtcm_20050402_3040_base.rtcm3`  | 3040 | 差分(喂 `/gnss/rtcm_corrections`,含 1005 基准站坐标) |

每个历元一条 1004,每 10 个历元补发 GPS 星历 1019 与 1005。转换丢了锁定时间信息,
rtkrcv 只能得到浮点解(原始 RINEX 后处理可以固定)——这份数据验证的是链路与解析,不是固定率。

重新生成的命令与期望的 sha256 见 `docs/gnss/plans/2026-09-14-round2-hardening.md` Task 6 Step 1。

回放必须用 libfaketime 把 rtkrcv 的时钟拨到 2005-04-02 那一周:RTCM MSM/1004 只带周内秒,
rtkrcv 用系统时间补周数,补错了星历对不上、一条解都没有。
```

- [ ] **Step 2: 包装脚本**

`gnss_bringup/test/rtkrcv_faketime.sh`:

```bash
#!/usr/bin/env bash
# 测试专用:在 libfaketime 下 exec 真实 rtkrcv。
# - 用 exec,pid 不变,节点发的 SIGTERM 直接到 rtkrcv(faketime 命令会 fork,不能用)
# - 用多线程版 libfaketimeMT:rtkrcv 是多线程的
# - 回放数据是 2005-04-02 的 RTCM3,rtkrcv 用系统时间补 RTCM 周数,时钟必须在同一周
exec env LD_PRELOAD="${GNSS_TEST_FAKETIME_LIB:?}" FAKETIME="@2005-04-02 03:00:00" TZ=UTC rtkrcv "$@"
```

```bash
chmod +x /home/steve/glim_ws/src/glim_ext/gnss_bringup/test/rtkrcv_faketime.sh
```

- [ ] **Step 3: 写测试**

`gnss_bringup/test/test_rtkrcv_real_binary.cpp`:

```cpp
#include <gtest/gtest.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include "gnss_bringup/executable_lookup.hpp"
#include "gnss_msgs/msg/raw_stream.hpp"
#include "gnss_msgs/msg/rtk_fix.hpp"
#include "node_process_harness.hpp"

using namespace gnss_bringup_test;
using gnss_msgs::msg::RawStream;
using gnss_msgs::msg::RtkFix;

namespace {
// 按 RTCM3 帧切分,每遇到一条 1004(每个历元最后一条)切一段
std::vector<std::vector<uint8_t>> split_epochs_after_1004(const std::string& path) {
  std::ifstream f(path, std::ios::binary);
  const std::vector<uint8_t> d((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
  std::vector<std::vector<uint8_t>> out;
  std::vector<uint8_t> cur;
  std::size_t i = 0;
  while (i + 6 <= d.size()) {
    if (d[i] != 0xD3) { ++i; continue; }
    const std::size_t len = (static_cast<std::size_t>(d[i + 1] & 0x03) << 8) | d[i + 2];
    if (i + 6 + len > d.size()) break;
    const int type = (d[i + 3] << 4) | (d[i + 4] >> 4);
    cur.insert(cur.end(), d.begin() + static_cast<long>(i), d.begin() + static_cast<long>(i + 6 + len));
    i += 6 + len;
    if (type == 1004) {
      out.push_back(std::move(cur));
      cur.clear();
    }
  }
  return out;
}

std::string find_faketime_lib() {
  for (const char* p : {"/usr/lib/x86_64-linux-gnu/faketime/libfaketimeMT.so.1",
                        "/usr/lib/aarch64-linux-gnu/faketime/libfaketimeMT.so.1"}) {
    if (::access(p, R_OK) == 0) return p;
  }
  return {};
}

std::string run_capture(const std::string& cmd) {
  std::string out;
  if (FILE* p = ::popen(cmd.c_str(), "r")) {
    char buf[256];
    while (std::fgets(buf, sizeof(buf), p)) out += buf;
    ::pclose(p);
  }
  return out;
}
}  // namespace

TEST(RtkrcvRealBinary, ReplayedTwoStationRtcmYieldsRtkFixesThroughTheNode) {
  const std::string rtkrcv = gnss_bringup::resolve_executable("rtkrcv", std::getenv("PATH"));
  if (rtkrcv.empty()) GTEST_SKIP() << "PATH 里没有 rtkrcv(RTKLIB-EX 2.5.1 未安装),跳过真实二进制回归";
  const std::string faketime = find_faketime_lib();
  if (faketime.empty()) GTEST_SKIP() << "未安装 libfaketime(sudo apt install faketime),跳过";
  const std::string version = run_capture(rtkrcv + " --version 2>&1");
  ASSERT_NE(version.find("EX 2.5"), std::string::npos)
      << "PATH 里的 rtkrcv 不是 RTKLIB-EX 2.5.x,rtkrcv_node 在这台机器上跑不起来:\n" << version;

  const auto rover = split_epochs_after_1004(std::string(GNSS_TEST_DATA_DIR) + "/rtcm_20050402_0759_rover.rtcm3");
  const auto base = split_epochs_after_1004(std::string(GNSS_TEST_DATA_DIR) + "/rtcm_20050402_3040_base.rtcm3");
  ASSERT_EQ(rover.size(), 120u);
  ASSERT_EQ(base.size(), 120u);

  const std::string dir = make_temp_dir("rtkrcv_real_");
  ASSERT_FALSE(dir.empty());
  std::filesystem::create_directories(dir + "/run");
  const std::string domain = std::to_string(40 + ::getpid() % 50);
  ::setenv("ROS_DOMAIN_ID", domain.c_str(), 1);            // 本测试进程自己的 rclcpp 也在这个域
  ::setenv("ROS_LOG_DIR", (dir + "/roslog").c_str(), 1);

  NodeProcess node(RTKRCV_NODE_PATH,
                   {"--ros-args",
                    "-p", std::string("binary:=") + RTKRCV_FAKETIME_WRAPPER,
                    "-p", "run_dir:=" + dir + "/run",
                    "-p", "sol_port:=" + std::to_string(pick_free_port()),
                    "-p", "corr_port:=" + std::to_string(pick_free_port()),
                    "-p", "obs_port:=" + std::to_string(pick_free_port()),
                    "-p", "sol_idle_timeout_s:=5.0"},
                   dir + "/node.log",
                   {{"ROS_DOMAIN_ID", domain}, {"ROS_LOG_DIR", dir + "/roslog"}, {"GNSS_TEST_FAKETIME_LIB", faketime}});

  rclcpp::init(0, nullptr);
  auto n = std::make_shared<rclcpp::Node>("rtkrcv_real_binary_test");
  std::mutex mu;
  std::vector<RtkFix> fixes;
  std::atomic<std::size_t> stat_bytes{0};
  const auto qos = rclcpp::QoS(1000).reliable();
  auto fix_sub = n->create_subscription<RtkFix>("/rtkrcv_node/rtk_fix", qos, [&](RtkFix::SharedPtr m) {
    std::lock_guard<std::mutex> lk(mu);
    fixes.push_back(*m);
  });
  auto stat_sub = n->create_subscription<RawStream>("/rtkrcv_node/stat", qos,
                                                    [&](RawStream::SharedPtr m) { stat_bytes += m->data.size(); });
  auto pub_obs = n->create_publisher<RawStream>("/gnss/raw_obs", qos);
  auto pub_corr = n->create_publisher<RawStream>("/gnss/rtcm_corrections", qos);
  rclcpp::executors::SingleThreadedExecutor ex;
  ex.add_node(n);
  const auto spin_for = [&](double s) {
    const auto end = std::chrono::steady_clock::now() + std::chrono::duration<double>(s);
    while (std::chrono::steady_clock::now() < end) ex.spin_some(std::chrono::milliseconds(20));
  };

  const bool ready = wait_until(
      [&] {
        ex.spin_some(std::chrono::milliseconds(20));
        return pub_obs->get_subscription_count() > 0 && pub_corr->get_subscription_count() > 0 &&
               node.log().find("sol stream: connected") != std::string::npos;
      },
      30.0);
  if (!ready) {
    rclcpp::shutdown();
    FAIL() << "节点没有就绪\n" << node.log();
  }

  for (std::size_t k = 0; k < rover.size(); ++k) {
    RawStream mb;
    mb.data = base[k];
    pub_corr->publish(mb);
    RawStream mr;
    mr.data = rover[k];
    pub_obs->publish(mr);
    spin_for(0.1);
  }
  spin_for(5.0);
  node.interrupt();
  const int rc = node.wait_exit(20.0);
  spin_for(0.5);
  rclcpp::shutdown();

  EXPECT_EQ(rc, 0) << node.log();
  EXPECT_GT(stat_bytes.load(), 0u) << "rtkrcv 的 $SAT/.stat 输出必须被转发";
  std::lock_guard<std::mutex> lk(mu);
  EXPECT_GE(fixes.size(), 80u)
      << "120 个历元回放下来应有约 100 条解;conf 缺 ant2-postype 时这里是 0\n" << node.log();
  for (const auto& f : fixes) {
    EXPECT_TRUE(f.quality == RtkFix::QUALITY_FLOAT || f.quality == RtkFix::QUALITY_FIXED)
        << "quality=" << static_cast<int>(f.quality);
    EXPECT_NEAR(f.latitude, 35.16087, 1e-3);
    EXPECT_NEAR(f.longitude, 139.61384, 1e-3);
    EXPECT_NEAR(f.altitude, 70.0, 10.0);
    // 2005-04-02 00:00:00–00:59:30 GPST,节点按 leap_seconds=18 换成 UTC unix 秒
    EXPECT_GT(f.gnss_time, 1112399900.0);
    EXPECT_LT(f.gnss_time, 1112403700.0);
  }
  std::filesystem::remove_all(dir);
}
```

CMake(`if(BUILD_TESTING)` 内):

```cmake
  # 真实 rtkrcv(RTKLIB-EX 2.5.1)+ libfaketime 回放双站 RTCM3 的端到端回归。
  # 机器上没有 rtkrcv 或 libfaketime 时 GTEST_SKIP;rtkrcv 版本不对时 FAIL。
  ament_add_gtest(test_rtkrcv_real_binary test/test_rtkrcv_real_binary.cpp TIMEOUT 180)
  target_link_libraries(test_rtkrcv_real_binary gnss_bringup_io)
  target_include_directories(test_rtkrcv_real_binary PRIVATE include test)
  ament_target_dependencies(test_rtkrcv_real_binary rclcpp gnss_msgs)
  add_dependencies(test_rtkrcv_real_binary rtkrcv_node)
  target_compile_definitions(test_rtkrcv_real_binary PRIVATE
    RTKRCV_NODE_PATH="$<TARGET_FILE:rtkrcv_node>"
    RTKRCV_FAKETIME_WRAPPER="${CMAKE_CURRENT_SOURCE_DIR}/test/rtkrcv_faketime.sh"
    GNSS_TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/data")
```

- [ ] **Step 4: GREEN,再用变异确认 RED**

Run: `cd /home/steve/glim_ws && colcon build --symlink-install --packages-select gnss_bringup && source install/setup.bash && ./build/gnss_bringup/test_rtkrcv_real_binary`
Expected: PASS(不是 SKIPPED;若 SKIPPED,说明 PATH 或 libfaketime 找不到,先修环境)。

变异(做完恢复):`rtkrcv_conf.cpp` 删掉 `ant2-postype` 输出 → 本测试必须 FAIL,且 `fixes.size()` 为 0。把实际输出写进 commit message。

另外连续跑 3 遍,必须 3 遍全过;测完 `pgrep -a -x rtkrcv` 为空,`ls /tmp | grep rtkrcv_real_` 为空。

- [ ] **Step 5: README「未验证项」改写**

把 `gnss_bringup/README.md` 的 `## 未验证项` 整节(到 `## 已知问题` 之前)替换为:

```markdown
## 已验证 / 未验证项

2026-09-14 在开发机上用 **RTKLIB-EX 2.5.1** 验证过。回归用例是 `test/test_rtkrcv_real_binary.cpp`
(未装 rtkrcv 或 libfaketime 时自动跳过)与 `test/test_rtkrcv_node_process.cpp`:

- 生成的 conf 键全部被 2.5.1 识别并生效(用 rtkrcv 控制台 `option` 逐项核对过)
- `-s -nc -r 2 -o <conf>` 能无终端常驻,SIGTERM 下正常退出
- 两路上行按字节原样送达(corrections → `inpstr2`,raw_obs → `inpstr1`),节点连得上 `sol_port`,
  `.stat` 文件名为 `rtkrcv_%Y%m%d%h%M.stat`
- 真实双站 RTCM3 回放(RTKLIB 自带 2005 年 GSI 两站 RINEX 转换而来,基线约 3.3 km):
  `RtkFix` 与 rtkrcv 原始解算行逐字段一致(经纬高、质量、NEU→ENU 标准差、卫星数、龄期),
  `pos_writer` 按 UTC 日轮转写出 `.pos`

仍未验证:

- **固定率**:回放数据只得到浮点解(转换成 RTCM 时丢了锁定信息,原始 RINEX 后处理可以固定),
  固定率只能用现场数据判断
- 现场板卡的原始观测格式,以及平台差分流是否带 1005/1006(`base_pos_type: rtcm` 的前提)
- 现场端点与连接方向(见「现场待确认」)
- 长时间运行、真实丢星与 AR 状态切换下的解算行
```

- [ ] **Step 6: 全量测试**

Run: `cd /home/steve/glim_ws && colcon test --packages-select gnss_core gnss_bringup && colcon test-result --all`
Expected: 0 failures;`test_rtkrcv_real_binary` 计入且不是 skipped。

- [ ] **Step 7: Commit**

```bash
cd /home/steve/glim_ws/src/glim_ext
git add gnss_bringup/test/data/rnx2rtcm.c gnss_bringup/test/data/README.md gnss_bringup/test/data/rtcm_20050402_0759_rover.rtcm3 gnss_bringup/test/data/rtcm_20050402_3040_base.rtcm3 gnss_bringup/test/rtkrcv_faketime.sh gnss_bringup/test/test_rtkrcv_real_binary.cpp gnss_bringup/CMakeLists.txt gnss_bringup/README.md
git update-index --chmod=+x gnss_bringup/test/rtkrcv_faketime.sh
git commit -m "test(gnss_bringup): end-to-end regression with the real RTKLIB-EX 2.5.1 rtkrcv replaying two-station RTCM3

<写明 GREEN 的 fixes 数量、删掉 ant2-postype 后的 RED 输出、3 遍连续运行结果>

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 7: 文档笔误与遗留清单

**Files:**
- Modify: `gnss_bringup/scripts/record_gnss.sh`(头部注释,约第 14-25 行)
- Modify: `gnss_bringup/config/gnss_bringup.yaml`(`pos_writer` 段注释,约第 120-128 行)
- Modify: `gnss_bringup/README.md`(「rosbag2 录制」一节的默认话题说明,约第 303-305 行)
- Modify: `/home/steve/glim_ws/src/glim_underground/docs/gnss/specs/2026-09-14-round2-rest-and-orphan-followups.md`

- [ ] **Step 1: `record_gnss.sh` 头部注释**

把 `GNSS_BAG_TOPICS` 说明里"默认覆盖 spec §5.2 的四路:两路裸流(……)、三路 RtkFix(两路来自 gnss_cgi610,一路来自本包的 rtkrcv_node)、以及 rtkrcv_node 转发的 $SAT 状态行。"这几行改为:

```bash
#                         默认录制六个话题:两路裸流(rtcm_bridge 转发,类型都是
#                         gnss_msgs/RawStream,靠话题名区分,见 README)、
#                         三路 RtkFix(/gnss_cgi610/rtk_fix;/gnss_cgi610/rtk_fix_gpchc
#                         目前仓库里没有任何发布者,留在清单里是为了现场协议确认后
#                         不必改脚本;/rtkrcv_node/rtk_fix)、以及 rtkrcv_node 转发的
#                         $SAT 状态行。
```

(只改注释,默认话题列表本身不动。)

- [ ] **Step 2: yaml 前后矛盾**

`gnss_bringup/config/gnss_bringup.yaml` 的 `pos_writer` 段里,把

```yaml
    # gpchc 这一路(现场协议确认之后),再加上 "gpchc"。下面 can/gpchc/
    # rtkrcv 三行 topic 映射都保留在配置里,按需把对应的名字加进 sources
    # 即可,不需要另外声明。
```

改为

```yaml
    # gpchc 这一路(现场协议确认之后),再加上 "gpchc"。注意下面只有 can 的
    # topic 映射是生效的,gpchc/rtkrcv 两行是注释:把名字加进 sources 时必须
    # 同时取消对应那一行的注释——否则 <name>.topic 会回落到默认值
    # "/gnss/<name>",那个话题没有任何发布者,现象只是一条沉默告警。
```

(依据:`src/pos_writer_node.cpp:329` 的 `declare_parameter<std::string>(n + ".topic", "/gnss/" + n)`。)

- [ ] **Step 3: README 默认录制话题**

`gnss_bringup/README.md` 里"默认录制:`/gnss/rtcm_corrections`、……`/rtkrcv_node/stat` 六路话题"这一句之后补一句:

```markdown
其中 `/gnss_cgi610/rtk_fix_gpchc` 目前没有任何发布者(驱动是否发布 gpchc 这一路待现场协议确认),
录到的 bag 里这个话题为空是正常的。
```

- [ ] **Step 4: 构建确认没有破坏安装**

Run: `cd /home/steve/glim_ws && colcon build --symlink-install --packages-select gnss_bringup && bash -n src/glim_ext/gnss_bringup/scripts/record_gnss.sh && python3 -c "import yaml,sys; yaml.safe_load(open('src/glim_ext/gnss_bringup/config/gnss_bringup.yaml'))" && echo OK`
Expected: `OK`。

- [ ] **Step 5: Commit(glim_ext)**

```bash
cd /home/steve/glim_ws/src/glim_ext
git add gnss_bringup/scripts/record_gnss.sh gnss_bringup/config/gnss_bringup.yaml gnss_bringup/README.md
git commit -m "docs(gnss_bringup): fix the gpchc topic and pos_writer source-mapping slips

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

- [ ] **Step 6: 更新遗留清单(glim_underground)**

在 `docs/gnss/specs/2026-09-14-round2-rest-and-orphan-followups.md` 文件末尾追加一节。表中"提交"一列填 `feat/round2-hardening` 上对应 Task 的 commit 短哈希(`git -C /home/steve/glim_ws/src/glim_ext log --oneline` 查):

```markdown
## 处理状态(`feat/round2-hardening`,2026-09-14 计划)

2026-09-14 用真实 RTKLIB-EX 2.5.1 联调又发现两个在车上一条解都出不来的缺陷(yaml 默认 `binary`
永远起不来;conf 缺 `ant2-postype`),与下列遗留项一并在该分支处理。

| 条目 | 处理 | 提交 |
|---|---|---|
| A.1 去重扫描提前 EOF | `open()` 比对扫描消费字节数与文件大小 | <Task 1 短哈希> |
| A.2 `read_glim_traj` 未检查 badbit | 已检查,并加 `std::istream&` 重载 | <Task 1 短哈希> |
| B.1 `rtkrcv_node.cpp` 无测试 | 节点级子进程测试 5 条 + 真实 rtkrcv 回归 1 条 | <Task 5 短哈希>、<Task 6 短哈希> |
| B.3 测试 hook 门控 | 部分:新增的 `read_pos`/`read_glim_traj` 流重载测试走真实 sentry 路径,不依赖 hook;hook 门控本身未改 | <Task 1 短哈希> |
| C.4 `open()` 拒绝时无诊断 | 未处理 | — |
| D 文档笔误(record_gnss.sh / yaml / README gpchc) | 已修 | <Task 7 短哈希> |
| (新)binary 裸名字起不来、崩溃循环静默 | 父进程按 PATH 解析;每次派生/退出打日志;启动前检查 | <Task 2 短哈希>、<Task 4 短哈希> |
| (新)conf 缺 `ant2-postype`、非法值静默回落 | 新增 `base_pos_type`/`bds_ar_mode`/`glo_ar_mode`;全部枚举按 2.5.1 校验 | <Task 3 短哈希> |
```

把表里每个 `<Task N 短哈希>` 替换成真实哈希后再提交:

```bash
cd /home/steve/glim_ws/src/glim_underground
git add docs/gnss/specs/2026-09-14-round2-rest-and-orphan-followups.md
git status --short   # 确认暂存区里只有这一个文件,time_keeper 两个文件仍是未暂存的 M
git commit -m "docs(gnss): record which round-2 follow-ups feat/round2-hardening addressed

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

- [ ] **Step 7: 最终全量验证**

Run: `cd /home/steve/glim_ws && colcon test --packages-select gnss_core gnss_bringup && colcon test-result --all`
Expected: 0 failures;`test_rtkrcv_real_binary` 为 passed 而非 skipped。
确认:`git -C /home/steve/glim_ws/src/glim_ext status --short` 为空;`pgrep -a -x rtkrcv`、`pgrep -a rtkrcv_node`、`pgrep -a fake_rtkrcv` 为空;`/tmp` 下没有本计划测试留下的 `rtkrcv_node_*`、`rtkrcv_real_*`、`exe_lookup_*`、`pw_inject_premature_eof.pos`、`pw_bytecount_healthy.pos`。
