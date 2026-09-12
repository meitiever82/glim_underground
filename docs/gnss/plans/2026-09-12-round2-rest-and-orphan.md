# 轮 2 收尾:.pos 写出、rosbag2 录制、rtkrcv 孤儿进程

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**版本:** v1(2026-09-12)。前置:`feat/round2-gnss-bringup`(PR #2)已合入 master。

**Goal:** 补齐轮 2 剩余的 D1(`.pos` 1 Hz 写出)与 A5/F1(rosbag2 录制与回放接入),并关掉上一轮最终评审
留下的最大残余风险——`rtkrcv` 孤儿进程。

**Architecture:** 与上一轮同构:算法与纯逻辑下沉到 `gnss_core`(无 ROS/GLIM),ROS 壳只做订阅、
接线与文件 I/O 编排。`.pos` 写出需要 core 侧新增一个**可追加**的 `PosWriter`——现有 `write_pos`
是一次性全量写出(截断 + 写表头),不满足 spec §5.3 的"追加写、崩溃安全"。孤儿进程用
`prctl(PR_SET_PDEATHSIG)` 做预防 + `sol_port` 占用检查做兜底。

**Tech Stack:** C++17、ament_cmake、rclcpp、gnss_msgs、gnss_core、POSIX、GoogleTest、ROS2 Humble(开发机)/Jazzy(Orin)。

**Spec:** `../specs/2026-09-03-gnss-glim-modules-design.md`(§3 A5/D1/F1、§5.2、§5.3)
**遗留项来源:** `../2026-09-12-round2-followups.md` B 节

---

## Global Constraints

- `gnss_core` 严禁依赖 ROS 与 GLIM,也不得加入网络 I/O 与进程管理。`PosWriter` 是纯文件 I/O
  与格式化,属于 core;订阅、话题、参数属于壳。
- 算法只写一遍:`.pos` 的列格式与时间系统换算已在 `pos_io` 里,`PosWriter` 必须复用同一套
  格式化与 GPST/UTC 处理,不得另写一份。
- **σ 顺序**:`RtkFix.sigma_enu` 是 **E/N/U**,`PosRecord.sdne` 是 RTKLIB 的 `(sdn, sde, sdu)` = **N/E/U**。
  本轮需要的是 `RtkFix → PosRecord` 的**反向**映射,前两项要换回来。上一轮已因此踩过一次坑。
- 参数校验沿用已有模式:端口用 `is_valid_port_for_direction`,秒数用 `declare_positive_seconds`。
  新参数一律校验,不得让一个 yaml 值静默地把功能关掉。
- 本包已确立的并发规则(每个阻塞点都 poll 唤醒 fd、每个 fd 只有一个关闭者、`stop()` 无条件 join、
  errno 分类而非吞掉、不留"声称在跑但线程已死"的状态)对新代码同样适用。
- 注释与文档用中文。
- 每个任务 TDD:先写失败测试 → 跑失败 → 最小实现 → 跑通过 → 提交。

---

## 现状与本轮增量

上一轮(PR #2)已交付:`TcpStream`、`LocalReserver`、`ProcessSupervisor`、`render_rtkrcv_conf`、
`LineSplitter`、`rtcm_bridge` 与 `rtkrcv_node`,106 个测试。`gnss_core` 侧 `write_pos` /
`PosDecimator` / `read_pos` / `q_to_quality` 已就绪并全绿。

| 已有 | 位置 | 本轮谁用 |
|---|---|---|
| `PosRecord` / `PosReadOptions` / `PosTimeSystem` | `gnss_core/pos_io.hpp` | Task 2/3 |
| `write_pos`(一次性全量) | 同上 | **不直接用**,Task 2 复用其格式化逻辑 |
| `PosDecimator`(按 floor(stamp/period) 分桶) | 同上 | Task 4 |
| `q_to_quality` | 同上 | Task 3 的反向映射需要其逆 |
| `to_rtk_fix(PosRecord) → RtkFix` | `gnss_bringup/rtk_fix_mapping.hpp` | Task 3 写**反向** |
| `is_valid_port_for_direction` / `declare_positive_seconds` 模式 | `gnss_bringup` | Task 5/6 |

**本轮不做**(spec 明确属轮 3):A6 保留天数与磁盘水位清理、D2 `events.log`、D3 `base.pos`、
九条诊断规则与 `gnss_diag`。

---

## 文件结构

**修改 `gnss_core`**:
- `include/gnss_core/pos_io.hpp` + `src/pos_io.cpp` — 新增 `PosWriter`(可追加)与 `format_pos_record`
  (从 `write_pos` 抽出的单行格式化,两者共用)

**修改 `gnss_bringup`**:
- `include/gnss_bringup/rtk_fix_mapping.hpp` — 新增 `to_pos_record(const RtkFix&)`
- `include/gnss_bringup/pos_rotation.hpp`(新) — 按天轮转的纯决策函数
- `src/pos_writer_node.cpp`(新) — 订阅 N 路 `RtkFix`,抽稀后写各自的 `.pos`
- `src/process_supervisor.cpp` — `prctl(PR_SET_PDEATHSIG)`
- `src/rtkrcv_node.cpp` — `sol_port` 占用检查
- `test/test_pos_rotation.cpp`(新)、`test/test_rtk_fix_mapping.cpp`(增)、
  `test/test_process_supervisor.cpp`(增)
- `scripts/record_gnss.sh`(新) — rosbag2 录制
- `config/gnss_bringup.yaml`、`launch/gnss_bringup.launch.py`、`README.md` — 接入

**修改 `gnss_core/test/test_pos_io.cpp`** — `PosWriter` 用例

---

## 依赖顺序

```
Task 1 (孤儿:prctl + 端口检查)          ← 独立,先做,关掉最大风险
Task 2 (gnss_core::PosWriter)
  └→ Task 3 (RtkFix → PosRecord 反向映射)
       └→ Task 4 (pos_rotation 纯决策) → Task 5 (pos_writer_node)
                                            └→ Task 6 (rosbag2 脚本 + launch/yaml/README 接入)
```

---

### Task 1: `rtkrcv` 孤儿进程 —— `prctl` 预防 + `sol_port` 占用检查

**Files:**
- Modify: `gnss_bringup/src/process_supervisor.cpp`(子进程分支)
- Modify: `gnss_bringup/src/rtkrcv_node.cpp`(启动检查)
- Create: `gnss_bringup/include/gnss_bringup/port_probe.hpp` + `gnss_bringup/src/port_probe.cpp`
- Create: `gnss_bringup/test/test_port_probe.cpp`(Step 5 的用例放这里,不要塞进
  `test_process_supervisor.cpp`——它们用的是 `LocalReserver`,与进程监管无关)
- Modify: `gnss_bringup/test/test_process_supervisor.cpp`(只加 Step 1 的 PDEATHSIG 用例)
- Modify: `gnss_bringup/CMakeLists.txt`(`port_probe.cpp` 进 `gnss_bringup_io`;注册
  `test_port_probe` 并链接 `gnss_bringup_io`,与既有四个 gtest 同样式)

**Interfaces:**
- Consumes: 无新接口
- Produces: 子进程在父线程死亡时自杀;`rtkrcv_node` 在 `sol_port` 已被占用时启动即响亮失败

**背景(实现者必须理解,否则会改错地方):** `setsid()` 让子进程脱离父进程的进程组,这对
`stop()` 的组 kill 是必需的,但副作用是**父进程非 `stop()` 的死亡不会带走子进程**。最终评审实测:
对 `rtkrcv_node` 发 `kill -9`,子进程被 init 收养、继续占着 `sol_port`;节点重启后新的 `rtkrcv`
绑不上该端口,而节点自己的 `TcpStream` 会**连上那个孤儿**,把它的陈旧解当新鲜 `RtkFix` 发出去。

`PR_SET_PDEATHSIG` 的 man page 警告说 "parent" 指**创建该进程的线程**而非进程。对本设计而言
这不是缺点:`fork()` 发生在 supervisor 的 worker 线程上,该线程终止只有两种情况——`stop()`
(本来就会杀子进程,信号冗余无害)与整个进程死亡(正是我们要覆盖的)。不存在"worker 线程没了
但还想让子进程活着"的场景。该设置**跨 `execve` 保留**(非 setuid 二进制),所以对 `rtkrcv` 生效。

残余窗口:`fork()` 之后、子进程执行 `prctl()` 之前的几微秒内父进程被 `SIGKILL`,信号不会送达。
端口占用检查就是兜这个,以及兜旧版本遗留的孤儿和误开的第二个实例。

- [ ] **Step 1: 写失败测试(追加到 test_process_supervisor.cpp)**

```cpp
TEST(ProcessSupervisor, ChildDiesWhenTheForkingThreadGoesAway) {
  // 模拟"父进程异常死亡":在一个独立线程里起 supervisor,让该线程直接结束,
  // 不调用 stop()。设了 PR_SET_PDEATHSIG 的子进程应当随之死亡。
  pid_t child = -1;
  {
    ProcessSupervisorConfig c = cfg_for("live", 0.05);
    auto sup = std::make_unique<ProcessSupervisor>(c);
    sup->start();
    for (int i = 0; i < 200 && sup->last_child_pid() <= 0; ++i)
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    child = sup->last_child_pid();
    ASSERT_GT(child, 0);
    ASSERT_EQ(::kill(child, 0), 0) << "子进程应当活着";
    // 故意泄漏 supervisor:不析构、不 stop(),模拟进程被 SIGKILL 时
    // 谁都来不及清理的情形。worker 线程随之被回收。
    (void)sup.release();
  }
  // 注:本用例只能验证到"forking 线程消失后子进程被信号带走"。
  // 真正的 kill -9 场景无法在同一个进程里测,见 Step 4 的手工复现。
  bool gone = false;
  for (int i = 0; i < 500; ++i) {
    if (::kill(child, 0) != 0 && errno == ESRCH) { gone = true; break; }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  EXPECT_TRUE(gone) << "forking 线程消失后子进程仍在,PR_SET_PDEATHSIG 没生效";
}
```

> 实现者注意:上面这个用例**故意泄漏** supervisor 对象。这是为了让 worker 线程在没有 `stop()`
> 的情况下结束,从而触发 PDEATHSIG——如果改成正常析构,`stop()` 会先杀掉子进程,用例就测不到
> 想测的东西了。泄漏在测试进程里是可接受的;请在注释里写明原因,否则将来会被"顺手修好"。

- [ ] **Step 2: 跑测试确认失败**

```bash
cd ~/glim_ws && source /opt/ros/humble/setup.bash && source ~/driver_ws/install/setup.bash && \
colcon build --packages-select gnss_bringup --cmake-args -DBUILD_TESTING=ON && \
./build/gnss_bringup/test_process_supervisor --gtest_filter='*ChildDiesWhenTheForkingThreadGoesAway*'
```

Expected: FAIL —— 子进程在 forking 线程消失后仍然活着(当前没有 PDEATHSIG)。

- [ ] **Step 3: 写实现**

在 `process_supervisor.cpp` 的子进程分支里,`setsid()` 之后、`sigaction` 循环之前加:

```c
    // 父线程(即 supervisor 的 worker 线程)一旦终止,内核给本进程发 SIGTERM。
    // 覆盖父进程被 SIGKILL / 崩溃 / OOM 的情形——那些路径下 stop() 根本来不及跑,
    // 而 setsid() 已经让本进程脱离了父进程组,不会被任何组信号带走。
    // man 2 prctl 的 "parent 指创建本进程的线程" 警告在这里反而正合适:
    // 该线程的生命周期恰好等于 "supervisor 应当在运行"。
    // 设置跨 execve 保留(非 setuid 二进制),因此对 execv 进来的 rtkrcv 依然有效。
    ::prctl(PR_SET_PDEATHSIG, SIGTERM, 0, 0, 0);
```

`prctl` 是裸系统调用,异步信号安全,可以放在 `fork()` 与 `execv()` 之间。需要 `#include <sys/prctl.h>`。

- [ ] **Step 4: 跑测试确认通过,并手工复现真实场景**

```bash
./build/gnss_bringup/test_process_supervisor          # 全绿
# 手工:真正的 kill -9 无法在单测里造,必须实跑一次
ros2 run gnss_bringup rtkrcv_node --ros-args \
  -p binary:=$PWD/src/gnss_bringup/test/fake_rtkrcv.sh -p args:="['live']" \
  -p run_dir:=/tmp/orphan_check &
sleep 2; NODE=$(pgrep -x rtkrcv_node); kill -9 "$NODE"; sleep 2
pgrep -a fake_rtkrcv || echo "无孤儿 —— 符合预期"
```

Expected: `无孤儿 —— 符合预期`。修复前同样的操作会留下一个 `fake_rtkrcv.sh live`。

- [ ] **Step 5: 写 `sol_port` 占用检查的失败测试**(文件:`test/test_port_probe.cpp`)

占用检查是纯逻辑之外的 I/O;为了可测,抽成一个不依赖 ROS 的自由函数,放进 `gnss_bringup_io`:

```cpp
// gnss_bringup/include/gnss_bringup/port_probe.hpp
// 探测 127.0.0.1:port 是否已有人监听。用于启动时发现残留的 rtkrcv 孤儿,
// 或误开的第二个实例——两者都会让本节点连上别人的解流,把陈旧解当新鲜数据发出去。
bool is_local_port_listening(int port);
```

```cpp
TEST(PortProbe, DetectsAListeningPort) {
  // 自己起一个监听,确认探测得到
  LocalReserver r;
  ASSERT_TRUE(r.start(0));
  EXPECT_TRUE(is_local_port_listening(r.bound_port()));
  r.stop();
}

TEST(PortProbe, ReportsFreePortAsNotListening) {
  LocalReserver r;
  ASSERT_TRUE(r.start(0));
  const int p = r.bound_port();
  r.stop();                       // 释放后同一端口应当探测为空闲
  EXPECT_FALSE(is_local_port_listening(p));
}
```

- [ ] **Step 6: 实现探测并接进 `rtkrcv_node`**

实现用一次 `connect()` 尝试即可(连得上说明有人监听)。接线位置:在 `start_supervisor()` **之前**、
参数校验之后;占用则抛 `std::runtime_error`,消息要明确指出可能是残留的 `rtkrcv` 孤儿并给出
`pgrep -a rtkrcv` 的排查提示——README 部署章节已有同样的措辞,保持一致。

- [ ] **Step 7: 跑通过 + 提交**

```bash
cd ~/glim_ws/src/glim_ext && git add gnss_bringup && \
git commit -m "fix(gnss_bringup): kill rtkrcv with its parent and refuse a busy sol_port"
```

---

### Task 2: `gnss_core::PosWriter`(可追加、崩溃安全)

**Files:**
- Modify: `gnss_core/include/gnss_core/pos_io.hpp`、`gnss_core/src/pos_io.cpp`
- Test: `gnss_core/test/test_pos_io.cpp`(追加)

**Interfaces:**
- Consumes: 既有 `PosRecord` / `PosTimeSystem`
- Produces: `gnss_core::PosWriter`,接口 `open(path)` / `write(const PosRecord&)` / `close()` /
  `is_open()`;以及从 `write_pos` 抽出的 `std::string format_pos_record(const PosRecord&, PosTimeSystem, int leap_seconds)`
  与 `std::string pos_header(PosTimeSystem)`,供 `write_pos` 与 `PosWriter` 共用。

**为什么不能直接用 `write_pos`:** 它是 `std::ofstream out(path)`(截断)+ 写三行表头 + 写全部记录。
1 Hz 连续写出的节点每来一条调一次就会重写表头并截断文件。spec §5.3 要求的是"追加写、崩溃安全"。

- [ ] **Step 1: 写头文件**

```cpp
// 追加式 .pos 写出器(spec §5.3:追加写、崩溃安全)。
// 与 write_pos 共用同一套表头与单行格式化,格式只写一遍。
// 打开已存在且非空的文件时不再重写表头,直接续写——因此进程重启不会破坏文件。
class PosWriter {
public:
  PosWriter() = default;
  explicit PosWriter(PosTimeSystem ts, int leap_seconds = 18) : ts_(ts), leap_(leap_seconds) {}
  ~PosWriter();
  PosWriter(const PosWriter&) = delete;
  PosWriter& operator=(const PosWriter&) = delete;

  // 打开(追加模式)。父目录不存在时创建。失败返回 false,不抛。
  bool open(const std::string& path);
  // 追加一条并 flush —— 崩溃安全的代价是每条一次 flush,1 Hz 下可忽略。
  bool write(const PosRecord& r);
  void close();
  bool is_open() const { return out_.is_open(); }
  const std::string& path() const { return path_; }

private:
  std::ofstream out_;
  std::string path_;
  PosTimeSystem ts_ = PosTimeSystem::GPST;
  int leap_ = 18;
};
```

- [ ] **Step 2: 写失败测试**

```cpp
TEST(PosWriter, WritesHeaderOnceForANewFile) {
  const std::string p = tmp_path("pw_new.pos");
  ::remove(p.c_str());
  PosWriter w;
  ASSERT_TRUE(w.open(p));
  ASSERT_TRUE(w.write(sample_record()));
  ASSERT_TRUE(w.write(sample_record()));
  w.close();
  // 表头行(以 % 开头)只应出现在文件开头,且数量与 write_pos 一致
  std::ifstream in(p);
  std::string line; int header_lines = 0, data_lines = 0;
  while (std::getline(in, line)) { if (!line.empty() && line[0] == '%') ++header_lines; else if (!line.empty()) ++data_lines; }
  EXPECT_EQ(data_lines, 2);
  EXPECT_GT(header_lines, 0);
  EXPECT_EQ(header_lines, 3) << "表头应与 write_pos 相同,且只写一次";
}

TEST(PosWriter, ReopeningAnExistingFileAppendsWithoutRewritingTheHeader) {
  const std::string p = tmp_path("pw_append.pos");
  ::remove(p.c_str());
  { PosWriter w; ASSERT_TRUE(w.open(p)); ASSERT_TRUE(w.write(sample_record())); }
  { PosWriter w; ASSERT_TRUE(w.open(p)); ASSERT_TRUE(w.write(sample_record())); }
  std::ifstream in(p);
  std::string line; int header_lines = 0, data_lines = 0;
  while (std::getline(in, line)) { if (!line.empty() && line[0] == '%') ++header_lines; else if (!line.empty()) ++data_lines; }
  EXPECT_EQ(data_lines, 2);
  EXPECT_EQ(header_lines, 3) << "重开不得再写一遍表头";
}

TEST(PosWriter, OutputIsReadableByReadPos) {
  // 写出的东西必须能被既有的 read_pos 原样读回 —— 这是格式没写歪的真正证明
  const std::string p = tmp_path("pw_roundtrip.pos");
  ::remove(p.c_str());
  PosRecord a = sample_record();
  PosRecord b = sample_record(); b.stamp += 1.0; b.q = 2; b.ns = 20;
  { PosWriter w; ASSERT_TRUE(w.open(p)); ASSERT_TRUE(w.write(a)); ASSERT_TRUE(w.write(b)); }
  const auto back = read_pos(p);
  ASSERT_EQ(back.size(), 2u);
  EXPECT_NEAR(back[0].lat, a.lat, 1e-8);
  EXPECT_NEAR(back[0].stamp, a.stamp, 1e-3);
  EXPECT_EQ(back[1].q, 2);
  EXPECT_EQ(back[1].ns, 20);
}

TEST(PosWriter, MatchesWritePosByteForByte) {
  // PosWriter 与 write_pos 必须产出完全相同的内容,否则格式就有两份实现了
  const std::string p1 = tmp_path("pw_a.pos"), p2 = tmp_path("pw_b.pos");
  ::remove(p1.c_str()); ::remove(p2.c_str());
  std::vector<PosRecord> recs{sample_record(), sample_record()};
  recs[1].stamp += 1.0;
  write_pos(p1, recs);
  { PosWriter w; ASSERT_TRUE(w.open(p2)); for (const auto& r : recs) ASSERT_TRUE(w.write(r)); }
  std::ifstream f1(p1), f2(p2);
  const std::string s1((std::istreambuf_iterator<char>(f1)), std::istreambuf_iterator<char>());
  const std::string s2((std::istreambuf_iterator<char>(f2)), std::istreambuf_iterator<char>());
  EXPECT_EQ(s1, s2);
}

TEST(PosWriter, CreatesMissingParentDirectories) {
  const std::string dir = tmp_path("pw_deep/20260912");
  const std::string p = dir + "/can.pos";
  std::filesystem::remove_all(tmp_path("pw_deep"));
  PosWriter w;
  EXPECT_TRUE(w.open(p)) << "按天轮转会写到当天的新目录里,必须自动建目录";
  EXPECT_TRUE(w.write(sample_record()));
}

TEST(PosWriter, OpenFailureIsReportedNotThrown) {
  PosWriter w;
  EXPECT_FALSE(w.open("/proc/definitely-not-writable/x.pos"));
  EXPECT_FALSE(w.is_open());
  EXPECT_FALSE(w.write(sample_record())) << "未打开时写入应返回 false 而不是崩";
}

TEST(PosWriter, EachRecordIsFlushedSoACrashKeepsWhatWasWritten) {
  const std::string p = tmp_path("pw_flush.pos");
  ::remove(p.c_str());
  PosWriter w;
  ASSERT_TRUE(w.open(p));
  ASSERT_TRUE(w.write(sample_record()));
  // 不 close,直接从另一个句柄读 —— 没 flush 的话读不到数据行
  const auto back = read_pos(p);
  EXPECT_EQ(back.size(), 1u) << "每条写完必须 flush,否则崩溃会丢掉整个缓冲";
}
```

`sample_record()` 是本文件已有的辅助(若无则新增:`stamp` 用一个固定的 UTC unix 秒,
lat/lon/height/q/ns/sdne/age/ratio 取与既有 `PosIo` 用例一致的值)。

- [ ] **Step 3: 跑测试确认失败**

先写空桩(`open` 返回 false、`write` 返回 false)保证链接通过。Expected: 6 FAIL / 1 PASS
(`OpenFailureIsReportedNotThrown` 空桩即过,实现后仍须保持)。

- [ ] **Step 4: 写实现**

要点:
- 把 `write_pos` 里的表头三行抽成 `pos_header(ts)`,把单条记录的 `snprintf` 抽成
  `format_pos_record(r, ts, leap)`;`write_pos` 改为调用这两个,**行为不得变化**
  (`MatchesWritePosByteForByte` 与既有 6 条 `PosIo` 用例一起守住这点)。
- `open`:用 `std::filesystem::create_directories` 建父目录(忽略"已存在");
  以 `std::ios::app` 打开;若打开后文件大小为 0 则写表头。
- `write`:未打开返回 false;写一行后 `out_ << std::flush`。
- 析构调 `close()`。

- [ ] **Step 5: 跑测试确认通过**

Expected: 7 passed,且 `test_pos_io` 既有 13 条不变。

- [ ] **Step 6: Commit**

```bash
cd ~/glim_ws/src/glim_ext && git add gnss_core && \
git commit -m "feat(gnss_core): append-capable PosWriter sharing write_pos's formatting"
```

---

### Task 3: `RtkFix → PosRecord` 反向映射

**Files:**
- Modify: `gnss_bringup/include/gnss_bringup/rtk_fix_mapping.hpp`
- Test: `gnss_bringup/test/test_rtk_fix_mapping.cpp`(追加)

**Interfaces:**
- Consumes: `gnss_msgs::msg::RtkFix`、`gnss_core::PosRecord`
- Produces: `gnss_bringup::to_pos_record(const gnss_msgs::msg::RtkFix&) -> gnss_core::PosRecord`

**这是上一轮踩过的坑的镜像版。** `to_rtk_fix` 做的是 `sdne(N/E/U) → sigma_enu(E/N/U)`;
本任务要反着来。质量字段同理:`RtkFix.quality` 是归一化枚举(NONE/SINGLE/DGPS/FLOAT/FIXED),
`PosRecord.q` 是 RTKLIB 的 Q(1=fix 2=float 4=dgps 5=single),需要 `q_to_quality` 的逆。

- [ ] **Step 1: 写失败测试**

```cpp
namespace {
gnss_msgs::msg::RtkFix fix_sample() {
  gnss_msgs::msg::RtkFix m;
  m.gnss_time = 1789045801.0;
  m.quality = gnss_msgs::msg::RtkFix::QUALITY_FIXED;
  m.latitude = 44.50123456; m.longitude = 90.28765432; m.altitude = 617.123;
  m.sigma_enu[0] = 0.022;   // E
  m.sigma_enu[1] = 0.011;   // N
  m.sigma_enu[2] = 0.033;   // U
  m.diff_age = 0.8f; m.sats_used = 38;
  return m;
}
}  // namespace

TEST(ToPosRecord, SigmaIsReorderedFromEnuBackToNeu) {
  const auto r = to_pos_record(fix_sample());
  EXPECT_DOUBLE_EQ(r.sdne(0), 0.011) << "sdn 应取 sigma_enu[1](N)";
  EXPECT_DOUBLE_EQ(r.sdne(1), 0.022) << "sde 应取 sigma_enu[0](E)";
  EXPECT_DOUBLE_EQ(r.sdne(2), 0.033) << "sdu 应取 sigma_enu[2](U)";
}

TEST(ToPosRecord, NormalizedQualityMapsBackToRtklibQ) {
  auto m = fix_sample();
  m.quality = gnss_msgs::msg::RtkFix::QUALITY_FIXED;  EXPECT_EQ(to_pos_record(m).q, 1);
  m.quality = gnss_msgs::msg::RtkFix::QUALITY_FLOAT;  EXPECT_EQ(to_pos_record(m).q, 2);
  m.quality = gnss_msgs::msg::RtkFix::QUALITY_DGPS;   EXPECT_EQ(to_pos_record(m).q, 4);
  m.quality = gnss_msgs::msg::RtkFix::QUALITY_SINGLE; EXPECT_EQ(to_pos_record(m).q, 5);
  m.quality = gnss_msgs::msg::RtkFix::QUALITY_NONE;   EXPECT_EQ(to_pos_record(m).q, 0);
}

TEST(ToPosRecord, RoundTripsThroughToRtkFix) {
  // 两个方向必须互逆 —— 这是防止某一侧悄悄改了顺序的最强约束
  const auto original = fix_sample();
  const auto back = to_rtk_fix(to_pos_record(original));
  EXPECT_NEAR(back.latitude, original.latitude, 1e-9);
  EXPECT_NEAR(back.longitude, original.longitude, 1e-9);
  EXPECT_NEAR(back.altitude, original.altitude, 1e-9);
  EXPECT_EQ(back.quality, original.quality);
  EXPECT_EQ(back.sats_used, original.sats_used);
  EXPECT_DOUBLE_EQ(back.sigma_enu[0], original.sigma_enu[0]);
  EXPECT_DOUBLE_EQ(back.sigma_enu[1], original.sigma_enu[1]);
  EXPECT_DOUBLE_EQ(back.sigma_enu[2], original.sigma_enu[2]);
  EXPECT_NEAR(back.diff_age, original.diff_age, 1e-6);
}

TEST(ToPosRecord, UsesGnssTimeAsTheEpochWhenPresent) {
  const auto r = to_pos_record(fix_sample());
  EXPECT_DOUBLE_EQ(r.stamp, 1789045801.0);
}

TEST(ToPosRecord, PositionAndAgeAndSatsAreCopied) {
  const auto r = to_pos_record(fix_sample());
  EXPECT_NEAR(r.lat, 44.50123456, 1e-9);
  EXPECT_NEAR(r.lon, 90.28765432, 1e-9);
  EXPECT_NEAR(r.height, 617.123, 1e-9);
  EXPECT_NEAR(r.age, 0.8, 1e-6);
  EXPECT_EQ(r.ns, 38);
}
```

- [ ] **Step 2: 跑测试确认失败 → 写实现 → 跑通过**

实现先写成 `return gnss_core::PosRecord{};` 看红,再填。`ratio` 在 `RtkFix` 里没有对应字段,
填 0 并在注释里写明(rtkrcv 的 `.pos` 有 ratio,但经 `RtkFix` 中转会丢失——这是消息定义的
既有取舍,不在本轮修改范围)。Expected: 5 passed。

- [ ] **Step 3: Commit**

```bash
cd ~/glim_ws/src/glim_ext && git add gnss_bringup && \
git commit -m "feat(gnss_bringup): RtkFix -> PosRecord mapping (ENU->NEU sigma swap)"
```

---

### Task 4: 按天轮转的纯决策函数

**Files:**
- Create: `gnss_bringup/include/gnss_bringup/pos_rotation.hpp`
- Create: `gnss_bringup/test/test_pos_rotation.cpp`
- Modify: `gnss_bringup/CMakeLists.txt`

**Interfaces:**
- Produces: `gnss_bringup::pos_path_for(const std::string& root, const std::string& source, double utc_stamp) -> std::string`
  与 `bool should_rotate(const std::string& current_path, const std::string& next_path)`

spec §5.3 的目录布局是 `<root>/YYYYMMDD/<source>.pos`,按天轮转。把"该写哪个文件、要不要换文件"
做成纯函数,节点只负责在换文件时关旧开新——与上一轮 `plan_stat_tail` 同样的处理方式,
理由也一样:时间与路径的决策是最容易出错又最容易测的部分。

- [ ] **Step 1: 写失败测试**

```cpp
TEST(PosRotation, PathIsRootSlashYyyymmddSlashSource) {
  // 2026-09-12 10:23:45 UTC
  EXPECT_EQ(pos_path_for("/data/gnss", "can", 1789208625.0), "/data/gnss/20260912/can.pos");
}

TEST(PosRotation, UsesUtcNotLocalTime) {
  // 2026-09-12 23:30:00 UTC —— 在 UTC+8 的本地时区已是 13 日,但目录必须按 UTC 走,
  // 否则同一份数据在不同时区的机器上会落进不同目录
  EXPECT_EQ(pos_path_for("/data/gnss", "can", 1789255800.0), "/data/gnss/20260912/can.pos");
}

TEST(PosRotation, RollsOverAtUtcMidnight) {
  const double before = 1789257599.0;   // 2026-09-12 23:59:59 UTC
  const double after  = 1789257600.0;   // 2026-09-13 00:00:00 UTC
  EXPECT_EQ(pos_path_for("/d", "rtkrcv", before), "/d/20260912/rtkrcv.pos");
  EXPECT_EQ(pos_path_for("/d", "rtkrcv", after),  "/d/20260913/rtkrcv.pos");
}

TEST(PosRotation, ShouldRotateOnlyWhenThePathChanges) {
  EXPECT_FALSE(should_rotate("/d/20260912/can.pos", "/d/20260912/can.pos"));
  EXPECT_TRUE(should_rotate("/d/20260912/can.pos", "/d/20260913/can.pos"));
  EXPECT_TRUE(should_rotate("", "/d/20260912/can.pos")) << "首次打开也算换文件";
}

TEST(PosRotation, DifferentSourcesGetDifferentFiles) {
  EXPECT_NE(pos_path_for("/d", "can", 1789208625.0), pos_path_for("/d", "gpchc", 1789208625.0));
}

TEST(PosRotation, TrailingSlashInRootIsHandled) {
  EXPECT_EQ(pos_path_for("/data/gnss/", "can", 1789208625.0), "/data/gnss/20260912/can.pos");
}
```

> 实现者注意:上面三个时间戳是按 UTC 算好的,**不要按本机时区重算**。实现必须用 `gmtime_r`
> 而不是 `localtime_r`——`pos_io.cpp` 里既有的时间处理也是这么做的,原因相同。

- [ ] **Step 2: 跑测试确认失败 → 写实现 → 跑通过**

Expected: 6 passed。

- [ ] **Step 3: Commit**

```bash
cd ~/glim_ws/src/glim_ext && git add gnss_bringup && \
git commit -m "feat(gnss_bringup): daily .pos rotation as a pure decision"
```

---

### Task 5: `pos_writer_node`

**Files:**
- Create: `gnss_bringup/src/pos_writer_node.cpp`
- Modify: `gnss_bringup/CMakeLists.txt`

**Interfaces:**
- Consumes: `PosWriter`(Task 2)、`to_pos_record`(Task 3)、`pos_path_for`/`should_rotate`(Task 4)、
  `gnss_core::PosDecimator`
- Produces: 可执行 `pos_writer`,订阅 N 路 `gnss_msgs/RtkFix`,每路按 1 Hz 抽稀写各自的 `.pos`

参数用与 `rtcm_bridge` 相同的扁平前缀风格:

```yaml
pos_writer:
  ros__parameters:
    root: "/data/gnss"
    sources: ["can", "gpchc", "rtkrcv"]
    period_s: 1.0              # 抽稀周期,spec §5.3 是 1 Hz
    time_system: "GPST"        # 写出的时间列;GPST | UTC
    leap_seconds: 18
    can:    { topic: "/gnss_cgi610/rtk_fix" }
    gpchc:  { topic: "/gnss_cgi610/rtk_fix_gpchc" }
    rtkrcv: { topic: "/rtkrcv_node/rtk_fix" }
```

- [ ] **Step 1: 写节点**

每路一个 `WrittenSource`,持有 `PosDecimator` + `PosWriter` + 当前路径。收到 `RtkFix`:
`to_pos_record` → `PosDecimator::accept` 不过则丢弃 → `pos_path_for(root, name, r.stamp)` →
与当前路径不同则 `close()` 旧的、`open()` 新的 → `write(r)`。

要点(每一条都是上一轮评审反复抓过的类型):
- `root` 为空、`sources` 重名或类型不对 → 清晰报错并 `exit(1)`,不得抛到 `main` 之外;
  沿用 `rtcm_bridge` 已有的 `find_duplicate_stream_name` 与 try/catch 模式。
- `period_s`、`leap_seconds` 校验;`time_system` 只接受 `"GPST"`/`"UTC"`,其它值报错。
- `PosWriter::open` 失败(磁盘满、只读挂载)必须 **ERROR** 而不是 INFO,并且不能每条记录刷屏——
  同一路径失败只报一次,路径变化时重置。
- `r.stamp` 为 0(板卡没给 `gnss_time`)时用 `header.stamp` 兜底,否则会写进 1970 年的目录。
  这一条要有测试或明确注释。

- [ ] **Step 2: 加进 CMakeLists、构建、冒烟**

```bash
ros2 run gnss_bringup pos_writer --ros-args -p root:=/tmp/poswriter -p sources:="['can']" \
  -p can.topic:=/test/rtk_fix &
ros2 topic pub -r 10 /test/rtk_fix gnss_msgs/msg/RtkFix \
  "{gnss_time: 1789208625.0, quality: 4, latitude: 44.5, longitude: 90.2, altitude: 617.0, \
    sigma_enu: [0.022, 0.011, 0.033], diff_age: 0.8, sats_used: 38}"
sleep 5; cat /tmp/poswriter/20260912/can.pos
```

Expected: 文件存在,三行表头 + 约 5 条数据行(10 Hz 输入被抽成 1 Hz),`Q` 列为 1。

- [ ] **Step 3: Commit**

---

### Task 6: rosbag2 录制脚本与接入

**Files:**
- Create: `gnss_bringup/scripts/record_gnss.sh`
- Modify: `gnss_bringup/CMakeLists.txt`(安装 scripts)、`config/gnss_bringup.yaml`、
  `launch/gnss_bringup.launch.py`、`README.md`

**Interfaces:** 无 C++ 接口。

spec §5.2:四路数据全部在总线上后,rosbag2 直接承担原始流记录与回放。**缺口**:rosbag2 没有
按保留天数或磁盘水位自动删除——那属 A6,轮 3 做,本轮只在 README 里写明这个缺口。

- [ ] **Step 1: 写脚本**

录制 `/gnss/rtcm_corrections`、`/gnss/raw_obs`、各路 `rtk_fix` 与 `~/stat`;
`--max-bag-duration` 按天分卷;输出目录带时间戳。脚本要:
- `set -euo pipefail`
- 可用环境变量覆盖输出根目录与话题清单,默认值写在脚本顶部注释里
- 启动前检查话题是否存在,不存在则**警告但继续**(录制空话题是合法的,链路可能稍后才起来)
- 在注释里写明 rosbag2 无保留策略这一缺口,并指向 README

- [ ] **Step 2: 接入 launch / yaml / README**

launch 加 `enable_pos_writer` 开关(默认 true);yaml 加 `pos_writer` 段与全部参数及注释;
README 增加一节:`.pos` 的目录布局与轮转、录制脚本用法、以及 **rosbag2 无保留策略**的缺口
(属轮 3 的 A6)。

**同时修掉 README 的构建说明(2026-09-12 更新)**:构建章节只写了
`source install/setup.bash`,没说清楚 `gnss_msgs` 从哪来。现状已变更——`gnss_msgs` 现在由
`setup_workspace.sh` 链接进 `glim_ws/src/`,**构建 glim_ws 不再需要 source driver_ws**。
README 要写明:

1. 新机器 / 新克隆在 `colcon build` 之前必须先跑一次 `bash src/glim_ext/setup_workspace.sh`
   (它建 `gnss_core` / `gnss_bringup` / `gnss_msgs` 三个符号链接);
2. **`gnss_msgs` 因此会被编两遍**——`driver_ws` 为了 `gnss_chcnav` 也在编它。
   改过 `.msg` 之后**两个 workspace 都要重建**,否则一边用新结构发、另一边用旧结构订,
   ROS2 Humble 下的表现是 **DDS 静默不匹配**:不报错、不警告,话题就是收不到。
   轮 1 往 `RtkFix` 里加 `gnss_time` 时正好是这个场景。这条要写得显眼,它是个静默陷阱。

顺带在 README 里写明**两路裸流共用 `RawStream` 类型但各有独立话题**:
`/gnss/rtcm_corrections`(平台差分)与 `/gnss/raw_obs`(板卡原始观测)。区分它们的是话题不是类型,
因为桥不解析任何东西;格式差异由 `rtkrcv_node` 的 `corr_format`/`obs_format` 两个参数承担。
并写明**只有原始观测、没有差分时**的用法:`streams: ["raw_obs"]` 只起那一路(已实测),
但 `rtkrcv` 只能出单点解(Q=5 → `QUALITY_SINGLE`),会被约束模块 `min_quality` 默认值全部拒掉——
链路通、不产约束,这是设计意图而非故障。这两点都实际被问到过,说明现有文档没讲清楚。

- [ ] **Step 3: 全量验证 + Commit**

```bash
cd ~/glim_ws && colcon build --symlink-install && \
colcon test --packages-select gnss_core gnss_bringup && colcon test-result --all
ros2 launch gnss_bringup gnss_bringup.launch.py enable_rtkrcv:=false
```

---

## 验收

- `colcon build --symlink-install` 全 workspace 通过
- `gnss_core` 与 `gnss_bringup` 全部测试通过;新增约 24 条(PosWriter 7、to_pos_record 5、
  pos_rotation 6、PortProbe 2、PDEATHSIG 1,其余为接线)
- `kill -9 rtkrcv_node` 后无孤儿 `rtkrcv`(手工复现)
- `sol_port` 被占用时 `rtkrcv_node` 启动即响亮失败,消息指向孤儿排查
- `pos_writer` 冒烟:10 Hz 输入 → 1 Hz 落盘,文件可被 `read_pos` 读回,跨 UTC 零点换文件
- README 写明 `.pos` 布局、录制脚本用法、rosbag2 无保留策略的缺口

## 遗留(不在本轮)

A6 保留天数与磁盘水位清理、旧 `.pos` 的 gzip、D2 `events.log`、D3 `base.pos`、轮 3 的诊断规则。
`followups.md` 其余各节(测试完整性、重复谓词、静默放弃路径等)仍然有效。
