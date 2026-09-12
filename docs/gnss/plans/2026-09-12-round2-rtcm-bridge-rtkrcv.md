# 轮 2:gnss_bringup(rtcm_bridge + rtkrcv_node)实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**版本:** v1(2026-09-12)。轮 2 的 `gnss_core` 侧(RTCM 分帧与 1005/1006、`$SAT` 与 llh 解析、`.pos` 写出与 1 Hz 抽稀)已于 2026-09-12 完成并全绿,本计划只做剩下的两个 ROS 壳。

**Goal:** 交付 `gnss_bringup` 包,内含 `rtcm_bridge`(TCP↔`RawStream` 双向桥,不解析)与 `rtkrcv_node`(RTKLIB 独立解算的进程管理 + 流转发 + 解流发布),补齐 spec §3 的 A1/A2/A7/B1–B3。

**Architecture:** 两个节点都是薄壳,算法全部复用轮 2 已完成的 `gnss_core`(`RtcmFramer` / `parse_llh_solution` / `StatEpochAccumulator`)。壳内新增的只有 I/O 与进程管理,且这两块也抽成不依赖 ROS 的类(`TcpStream` / `LocalReserver` / `ProcessSupervisor`),以便对 loopback 与假二进制做完整单测——ROS 节点本身只剩参数读取与话题接线。`rtkrcv_node` 内不含任何对外 TCP:它订阅 topic、把字节喂给本机 TCP 服务,rtkrcv 作为 tcpcli 连进来(spec §5.1 的"单一路径"原则)。

**Tech Stack:** C++17、ament_cmake、rclcpp、gnss_msgs、gnss_core、POSIX socket/process API、GoogleTest、ROS2 Humble(开发机)/ Jazzy(Orin)。

**Spec:** `../specs/2026-09-03-gnss-glim-modules-design.md`(§3 A1/A2/A5/A7、§3 B1–B3、§4.2、§5.1)

**参考实现:** `~/Documents/GitHub/gnss-alg/rtk-monitor/src/rtk_monitor/`——`collectors/tcp.py`(连/听两模式、指数退避、静默超时)、`collectors/reserve.py`(本机 TCP 扇出)、`solver/rtkrcv.py`(conf 模板、`-s -nc -r 2 -o`、崩溃退避、进程组信号)。**已交付并现场验证过**,本计划的行为语义以它为准,不重新设计。

---

## Global Constraints

- `gnss_core` 严禁依赖 ROS 与 GLIM(spec §6)。**本计划不得往 `gnss_core` 里加任何网络 I/O 或进程管理代码**——它是算法核心,`TcpStream`/`LocalReserver`/`ProcessSupervisor` 一律放 `gnss_bringup`。
- 算法只写一遍:RTCM 分帧、`$SAT`/llh 解析一律调用 `gnss_core`,壳内不得重写(spec §2.1)。
- `rtcm_bridge` **不做任何解析**,收到多少发多少(spec §5.1)。解析是 `rtkrcv_node` 与轮 3 `gnss_diag` 各自的事。
- `rtkrcv_node` 内不得出现对平台/板卡的对外 TCP 连接;它只订阅 topic(spec §3 A7:"取消本地转发——`rtkrcv_node` 订阅 topic")。
- `RtkFix` 用 **reliable** QoS 发布(spec §4.1 v2:GLIM 的 `TopicSubscription` 固定以默认 reliable 订阅,best_effort 会静默不匹配)。
- `RtkFix.sigma_enu` 是 **E/N/U** 顺序;RTKLIB 的 `sdn/sde/sdu` 是 **N/E/U**。转换时必须交换前两项(spec §4.1 v2 的 σ 顺序注意项)。
- 时间:`header.stamp` = 主机接收时刻;`gnss_time` = 解算历元时刻(由 `parse_llh_solution` 输出的 UTC unix 秒)。两者都要填(spec §4.1 v2)。
- 工作区布局:`gnss_bringup` 源码放 **`~/glim_ws/src/glim_ext/gnss_bringup/`**(与 `gnss_core` 同一 git 仓),由 `glim_ext/setup_workspace.sh` 幂等建 `src/gnss_bringup` 符号链接供 colcon 发现。理由:复用已有机制、单一 git 归属、不新建仓库。**这是一个可复议的决定**——若日后要把 GNSS 相关包从 glim_ext fork 里分出去,改动只是移动目录 + 改 `setup_workspace.sh`。
- 构建 `glim_ws` 前先 `source ~/driver_ws/install/setup.bash`(拿 `gnss_msgs`)。
- 每个任务 TDD:先写失败测试 → 跑失败 → 最小实现 → 跑通过 → 提交。

---

## 现状与本轮范围

**轮 2 的 `gnss_core` 侧已完成(2026-09-12,14 可执行 122 gtest + 10 Python 用例全绿)**,本计划直接消费这些接口:

| 已有接口 | 头文件 | 本轮谁用 |
|---|---|---|
| `crc24q` / `RtcmFramer` / `parse_base_station` | `gnss_core/rtcm.hpp` | 轮 3 `gnss_diag`;本轮不用(`rtcm_bridge` 不解析) |
| `parse_sat_line` / `StatEpochAccumulator` / `SlipWindow` | `gnss_core/rtkstat.hpp` | 轮 3;本轮 `rtkrcv_node` 只把 `.stat` 原样发出 |
| `parse_llh_solution(line, PosRecord&, PosReadOptions)` | `gnss_core/rtkstat.hpp` | **Task 7** |
| `q_to_quality(int)` | `gnss_core/pos_io.hpp` | **Task 7** |
| `write_pos` / `PosDecimator` | `gnss_core/pos_io.hpp` | 轮 2 的 `.pos` 写出节点(D1,**不在本计划内**,见下) |

**本计划不包含**(轮 2 剩余项,另行安排):
- `.pos` 1 Hz 写出节点(D1)——core 侧 `write_pos` + `PosDecimator` 已就绪,只差一个订阅 `RtkFix` 的薄节点。
- rosbag2 录制脚本(A5/F1)。

**已知缺口(不阻塞本计划的编码,但阻塞现场联调):**
1. **`inpstr1-format` / `inpstr2-format` 的取值(spec §13 P0)。** rtk-monitor 两者都假定 `rtcm3`。本计划把它们做成 ROS 参数,默认沿用 `rtcm3`;现场确认后改默认值即可,**不需要改代码**。
2. **RTKLIB 未安装。** 开发机 `which rtkrcv` 为空、apt 无包、无源码树。因此 Task 5–7 只能用**假二进制**验证 conf 生成、进程监管与流转发;`rtkrcv` 真实键名的验证是集成步骤(rtk-monitor 自己的注释也这么写:"verifying exact key names against the real binary is an integration step, not a unit test")。Task 8 登记这一条。
3. **平台 TCP 端点与连接方向。** `host` / `port` / `listen` 三个参数,现场填。协议本身已确认是**裸 TCP 无解析**(rtk-monitor 全仓无 NTRIP:无 mountpoint、无 GGA 上传、无 Basic auth)。

> **spec 缺口(本计划一并修正)**:spec §5.1 写的是"`rtcm_bridge` 保持'笨':**TCP 客户端**",而参考实现是 connect-or-listen 两种模式。平台若主动推流到车上就必须监听。Task 2 实现两种模式,Task 8 把 §5.1 改掉。

---

## 文件结构

**新建包 `gnss_bringup`**(源码在 `glim_ext/gnss_bringup/`,经符号链接被 colcon 识别为 `src/gnss_bringup`):

```
gnss_bringup/
├── package.xml                              ament_cmake;depend rclcpp / gnss_msgs / gnss_core
├── CMakeLists.txt                           两个可执行 + 一个内部库 gnss_bringup_io + 测试
├── include/gnss_bringup/
│   ├── tcp_stream.hpp                       连/听两模式 + 指数退避 + 静默超时(Task 1/2)
│   ├── local_reserver.hpp                   本机 TCP 扇出,喂 rtkrcv(Task 4)
│   ├── rtkrcv_conf.hpp                      conf 文本生成(纯函数,Task 5)
│   └── process_supervisor.hpp               子进程起停 + 崩溃退避(Task 6)
├── src/
│   ├── tcp_stream.cpp
│   ├── local_reserver.cpp
│   ├── rtkrcv_conf.cpp
│   ├── process_supervisor.cpp
│   ├── rtcm_bridge_node.cpp                 main() + 参数 + 接线(Task 3)
│   └── rtkrcv_node.cpp                      main() + 参数 + 接线(Task 7)
├── launch/
│   └── gnss_bringup.launch.py               两个节点一起起(Task 8)
├── config/
│   └── gnss_bringup.yaml                    参数默认值,P0 未定项在此加注释(Task 8)
├── test/
│   ├── test_tcp_stream.cpp
│   ├── test_local_reserver.cpp
│   ├── test_rtkrcv_conf.cpp
│   ├── test_process_supervisor.cpp
│   ├── test_rtk_fix_mapping.cpp             PosRecord → RtkFix 的纯映射(Task 7)
│   └── fake_rtkrcv.sh                       假二进制:按参数决定立即退出 / 长活(Task 6)
└── README.md                                含 P0 缺口登记(Task 8)
```

**修改 `glim_ext/setup_workspace.sh`**:把要链接的目录从写死的 `gnss_core` 改成列表 `gnss_core gnss_bringup`(Task 1)。

**修改 spec §5.1**:TCP 客户端 → connect-or-listen 两模式(Task 8)。

---

## 依赖顺序

```
Task 1 (包骨架 + TcpStream 客户端模式)
  ├→ Task 2 (TcpStream 监听模式 + 退避)  →  Task 3 (rtcm_bridge_node)
  ├→ Task 4 (LocalReserver) ─────────────┐
  ├→ Task 5 (rtkrcv_conf) ───────────────┤→ Task 7 (rtkrcv_node)
  └→ Task 6 (ProcessSupervisor) ─────────┘
                                          → Task 8 (launch + 配置 + 文档 + spec 修正)
```

Task 2、4、5、6 彼此独立,可并行。

---

### Task 1: `gnss_bringup` 包骨架 + `TcpStream` 客户端模式

**Files:**
- Create: `glim_ext/gnss_bringup/package.xml`
- Create: `glim_ext/gnss_bringup/CMakeLists.txt`
- Create: `glim_ext/gnss_bringup/include/gnss_bringup/tcp_stream.hpp`
- Create: `glim_ext/gnss_bringup/src/tcp_stream.cpp`
- Test: `glim_ext/gnss_bringup/test/test_tcp_stream.cpp`
- Modify: `glim_ext/setup_workspace.sh`

**Interfaces:**
- Consumes: 无(本包第一个任务)
- Produces: `gnss_bringup::TcpStream`、`gnss_bringup::TcpStreamConfig`、回调类型 `OnData = std::function<void(const uint8_t*, size_t)>` 与 `OnState = std::function<void(bool connected, const std::string& detail)>`。Task 2 扩展它的监听模式,Task 3 用它。

- [ ] **Step 1: 写 `setup_workspace.sh` 的多目录版本**

把脚本里写死的 `gnss_core` 改成列表,其余逻辑(幂等、拒绝覆盖非符号链接、链接指向不符时报错)不变:

```bash
for name in gnss_core gnss_bringup; do
  link="${src_dir}/${name}"
  target_rel="$(basename "${glim_ext_dir}")/${name}"
  if [[ ! -d "${glim_ext_dir}/${name}" ]]; then
    echo "error: ${glim_ext_dir}/${name} 不存在" >&2; exit 1
  fi
  if [[ -L "${link}" ]]; then
    current="$(readlink "${link}")"
    if [[ "${current}" == "${target_rel}" ]]; then echo "ok: ${link} -> ${current} (已存在)"; continue; fi
    echo "error: ${link} 已是指向 ${current} 的符号链接,与预期的 ${target_rel} 不符" >&2; exit 1
  fi
  if [[ -e "${link}" ]]; then echo "error: ${link} 已存在且不是符号链接,拒绝覆盖" >&2; exit 1; fi
  ln -s "${target_rel}" "${link}"
  echo "created: ${link} -> ${target_rel}"
done
```

- [ ] **Step 2: 写 `package.xml`**

```xml
<?xml version="1.0"?>
<package format="3">
  <name>gnss_bringup</name>
  <version>0.1.0</version>
  <description>GNSS data-plane nodes: raw TCP stream bridge and RTKLIB rtkrcv supervisor</description>
  <maintainer email="dev@example.com">dev</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <depend>rclcpp</depend>
  <depend>gnss_msgs</depend>
  <depend>gnss_core</depend>
  <test_depend>ament_cmake_gtest</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

- [ ] **Step 3: 写 `CMakeLists.txt`**

I/O 与进程管理放进内部库 `gnss_bringup_io`,两个节点各自链接它。这样测试只链库,不必把 `main()` 拉进来。

```cmake
cmake_minimum_required(VERSION 3.16)
project(gnss_bringup)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gnss_msgs REQUIRED)
find_package(gnss_core REQUIRED)

# 不依赖 ROS 的 I/O 与进程管理,便于单测
add_library(gnss_bringup_io SHARED
  src/tcp_stream.cpp
)
target_include_directories(gnss_bringup_io PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(gnss_bringup_io pthread)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS gnss_bringup_io EXPORT gnss_bringup-targets
  LIBRARY DESTINATION lib ARCHIVE DESTINATION lib RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_tcp_stream test/test_tcp_stream.cpp)
  target_link_libraries(test_tcp_stream gnss_bringup_io)
endif()

ament_export_targets(gnss_bringup-targets HAS_LIBRARY_TARGET)
ament_package()
```

- [ ] **Step 4: 写头文件 `tcp_stream.hpp`**

```cpp
#pragma once
#include <atomic>
#include <cstdint>
#include <functional>
#include <string>
#include <thread>

namespace gnss_bringup {

using OnData = std::function<void(const uint8_t* data, size_t len)>;
// connected=true 进入连通态,false 断开;detail 供日志
using OnState = std::function<void(bool connected, const std::string& detail)>;

struct TcpStreamConfig {
  std::string host = "127.0.0.1";
  int port = 0;                     // listen 模式下 0 = 让内核选端口(测试用)
  bool listen = false;              // true: 本机监听等对端连进来;false: 主动连对端
  double initial_backoff_s = 1.0;
  double max_backoff_s = 30.0;
  double idle_timeout_s = 30.0;     // 静默视为断开:链路差时对端常不发 RST 就消失
};

// 一条裸字节流。不做任何解析,收到多少回调多少。
// 断开后按指数退避重连(客户端模式)或继续等待下一个连接(监听模式),永不放弃。
class TcpStream {
public:
  TcpStream(TcpStreamConfig cfg, OnData on_data, OnState on_state = {});
  ~TcpStream();
  TcpStream(const TcpStream&) = delete;
  TcpStream& operator=(const TcpStream&) = delete;

  void start();
  void stop();
  int bound_port() const { return bound_port_.load(); }   // 监听模式实际绑定端口

private:
  void run_client();

  TcpStreamConfig cfg_;
  OnData on_data_;
  OnState on_state_;
  std::thread thread_;
  std::atomic<bool> running_{false};
  std::atomic<int> bound_port_{-1};
  std::atomic<int> wake_fd_{-1};    // stop() 用来打断阻塞中的 recv/accept
};

}  // namespace gnss_bringup
```

- [ ] **Step 5: 写失败测试 `test_tcp_stream.cpp`**

```cpp
#include <gtest/gtest.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#include "gnss_bringup/tcp_stream.hpp"
using namespace gnss_bringup;

namespace {

// 收集回调数据的小工具,带"等到收够 n 字节"的超时等待
class Sink {
public:
  void push(const uint8_t* d, size_t n) {
    std::lock_guard<std::mutex> lk(m_);
    bytes_.insert(bytes_.end(), d, d + n);
    cv_.notify_all();
  }
  bool wait_for_bytes(size_t n, std::chrono::milliseconds to) {
    std::unique_lock<std::mutex> lk(m_);
    return cv_.wait_for(lk, to, [&] { return bytes_.size() >= n; });
  }
  std::string str() {
    std::lock_guard<std::mutex> lk(m_);
    return std::string(bytes_.begin(), bytes_.end());
  }
private:
  std::mutex m_;
  std::condition_variable cv_;
  std::vector<uint8_t> bytes_;
};

// 一个只接受一个连接、发一段字节然后按需关闭的最小测试服务端
class TestServer {
public:
  int start() {
    fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
    int one = 1;
    ::setsockopt(fd_, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
    sockaddr_in a{};
    a.sin_family = AF_INET;
    a.sin_addr.s_addr = ::htonl(INADDR_LOOPBACK);
    a.sin_port = 0;
    ::bind(fd_, reinterpret_cast<sockaddr*>(&a), sizeof(a));
    ::listen(fd_, 1);
    socklen_t len = sizeof(a);
    ::getsockname(fd_, reinterpret_cast<sockaddr*>(&a), &len);
    return ::ntohs(a.sin_port);
  }
  void accept_and_send(const std::string& payload) {
    conn_ = ::accept(fd_, nullptr, nullptr);
    ::send(conn_, payload.data(), payload.size(), 0);
  }
  void close_conn() { if (conn_ >= 0) { ::close(conn_); conn_ = -1; } }
  ~TestServer() { close_conn(); if (fd_ >= 0) ::close(fd_); }
private:
  int fd_ = -1, conn_ = -1;
};

}  // namespace

TEST(TcpStream, ClientModeDeliversBytesFromServer) {
  TestServer srv;
  const int port = srv.start();

  Sink sink;
  TcpStreamConfig cfg;
  cfg.host = "127.0.0.1";
  cfg.port = port;
  TcpStream s(cfg, [&](const uint8_t* d, size_t n) { sink.push(d, n); });
  s.start();

  srv.accept_and_send("HELLO-RTCM");
  EXPECT_TRUE(sink.wait_for_bytes(10, std::chrono::seconds(3)));
  EXPECT_EQ(sink.str(), "HELLO-RTCM");
  s.stop();
}

TEST(TcpStream, ReportsConnectedThenDisconnected) {
  TestServer srv;
  const int port = srv.start();

  std::mutex m;
  std::condition_variable cv;
  std::vector<bool> states;
  TcpStreamConfig cfg;
  cfg.port = port;
  cfg.initial_backoff_s = 0.05;
  TcpStream s(cfg, [](const uint8_t*, size_t) {},
              [&](bool connected, const std::string&) {
                std::lock_guard<std::mutex> lk(m);
                states.push_back(connected);
                cv.notify_all();
              });
  s.start();
  srv.accept_and_send("x");
  srv.close_conn();

  std::unique_lock<std::mutex> lk(m);
  ASSERT_TRUE(cv.wait_for(lk, std::chrono::seconds(3),
                          [&] { return states.size() >= 2; }));
  EXPECT_TRUE(states[0]) << "先报 connected";
  EXPECT_FALSE(states[1]) << "对端关闭后报 disconnected";
  lk.unlock();
  s.stop();
}

TEST(TcpStream, StopReturnsPromptlyWhenNeverConnected) {
  // 连一个没人监听的端口:start 后线程在退避循环里,stop 必须能及时打断
  Sink sink;
  TcpStreamConfig cfg;
  cfg.port = 1;                      // 特权端口,必然连不上
  cfg.initial_backoff_s = 10.0;      // 故意设很长,验证 stop 不是靠等退避结束
  TcpStream s(cfg, [&](const uint8_t* d, size_t n) { sink.push(d, n); });
  s.start();
  const auto t0 = std::chrono::steady_clock::now();
  s.stop();
  const auto dt = std::chrono::steady_clock::now() - t0;
  EXPECT_LT(std::chrono::duration_cast<std::chrono::milliseconds>(dt).count(), 1000);
}

TEST(TcpStream, DestructorReleasesTheListeningSocket) {
  // 析构必须真的把线程 join 掉、把 socket 关掉,而不只是"没崩"。
  // 可观测的后果:析构后那个端口不再接受连接。
  int port = -1;
  {
    Sink sink;
    TcpStreamConfig cfg;
    cfg.listen = true;
    cfg.port = 0;
    TcpStream s(cfg, [&](const uint8_t* d, size_t n) { sink.push(d, n); });
    s.start();
    for (int i = 0; i < 100 && port <= 0; ++i) {
      port = s.bound_port();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    ASSERT_GT(port, 0);
  }   // 无显式 stop(),只靠析构

  int c = ::socket(AF_INET, SOCK_STREAM, 0);
  sockaddr_in a{};
  a.sin_family = AF_INET;
  a.sin_addr.s_addr = ::htonl(INADDR_LOOPBACK);
  a.sin_port = ::htons(static_cast<uint16_t>(port));
  const int rc = ::connect(c, reinterpret_cast<sockaddr*>(&a), sizeof(a));
  ::close(c);
  EXPECT_NE(rc, 0) << "析构后端口仍在监听,说明 socket 没被关掉";
}
```

- [ ] **Step 6: 跑测试确认失败**

```bash
cd ~/glim_ws && bash src/glim_ext/setup_workspace.sh && \
source /opt/ros/humble/setup.bash && source ~/driver_ws/install/setup.bash && \
colcon build --packages-select gnss_bringup --cmake-args -DBUILD_TESTING=ON && \
./build/gnss_bringup/test_tcp_stream
```

Expected: 4 个用例全部 FAIL(实现为空桩)。先把 `src/tcp_stream.cpp` 写成空桩(构造/析构/start/stop 都为空体)以保证能编译链接——**必须看到断言失败,而不是编译错误**。

- [ ] **Step 7: 写实现 `tcp_stream.cpp`(客户端模式)**

要点:
- 用一对 `pipe()` 做唤醒 fd(`wake_fd_`),`stop()` 往里写一个字节;所有阻塞点用 `poll()` 同时等 socket 与唤醒 fd,因此 `stop()` 不必等退避睡完。
- 退避睡眠也用 `poll(wake_fd_, timeout)` 实现,而不是 `sleep`。
- 连上后 `backoff` 复位为 `initial_backoff_s`;每次失败 `backoff = min(backoff*2, max)`。
- `poll` 超过 `idle_timeout_s` 无数据 → 视为断开(对端消失未发 RST)。
- 状态回调只在**跳变**时发(参考实现的 `_last_state` 去重),否则连不上的端口会每个退避周期刷一条。
- 析构里调 `stop()` 并 `join()`。

- [ ] **Step 8: 跑测试确认通过**

Expected: 4 passed。

- [ ] **Step 9: Commit**

```bash
cd ~/glim_ws/src/glim_ext && git add gnss_bringup setup_workspace.sh && \
git commit -m "feat(gnss_bringup): package skeleton and TcpStream client mode"
```

---

### Task 2: `TcpStream` 监听模式与退避

**Files:**
- Modify: `glim_ext/gnss_bringup/src/tcp_stream.cpp`
- Modify: `glim_ext/gnss_bringup/include/gnss_bringup/tcp_stream.hpp`(加 `run_server()` 私有方法声明)
- Test: `glim_ext/gnss_bringup/test/test_tcp_stream.cpp`(追加)

**Interfaces:**
- Consumes: Task 1 的 `TcpStream` / `TcpStreamConfig`
- Produces: `cfg.listen = true` 时 `TcpStream` 监听 `host:port`;`port=0` 时由内核选端口,经 `bound_port()` 取回。Task 3 用它支持"平台主动推流到车上"。

- [ ] **Step 1: 写失败测试(追加到 test_tcp_stream.cpp)**

```cpp
TEST(TcpStream, ListenModeAcceptsPeerAndDeliversBytes) {
  Sink sink;
  TcpStreamConfig cfg;
  cfg.listen = true;
  cfg.port = 0;                       // 内核选端口
  TcpStream s(cfg, [&](const uint8_t* d, size_t n) { sink.push(d, n); });
  s.start();

  // 等绑定完成
  int port = -1;
  for (int i = 0; i < 100 && port <= 0; ++i) {
    port = s.bound_port();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  ASSERT_GT(port, 0) << "监听模式必须报出实际绑定端口";

  int c = ::socket(AF_INET, SOCK_STREAM, 0);
  sockaddr_in a{};
  a.sin_family = AF_INET;
  a.sin_addr.s_addr = ::htonl(INADDR_LOOPBACK);
  a.sin_port = ::htons(static_cast<uint16_t>(port));
  ASSERT_EQ(::connect(c, reinterpret_cast<sockaddr*>(&a), sizeof(a)), 0);
  const std::string payload = "PUSHED-RTCM";
  ::send(c, payload.data(), payload.size(), 0);

  EXPECT_TRUE(sink.wait_for_bytes(payload.size(), std::chrono::seconds(3)));
  EXPECT_EQ(sink.str(), payload);
  ::close(c);
  s.stop();
}

TEST(TcpStream, ListenModeAcceptsASecondPeerAfterFirstDisconnects) {
  Sink sink;
  TcpStreamConfig cfg;
  cfg.listen = true;
  cfg.port = 0;
  TcpStream s(cfg, [&](const uint8_t* d, size_t n) { sink.push(d, n); });
  s.start();
  int port = -1;
  for (int i = 0; i < 100 && port <= 0; ++i) {
    port = s.bound_port();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  ASSERT_GT(port, 0);

  auto connect_send_close = [&](const std::string& payload) {
    int c = ::socket(AF_INET, SOCK_STREAM, 0);
    sockaddr_in a{};
    a.sin_family = AF_INET;
    a.sin_addr.s_addr = ::htonl(INADDR_LOOPBACK);
    a.sin_port = ::htons(static_cast<uint16_t>(port));
    ASSERT_EQ(::connect(c, reinterpret_cast<sockaddr*>(&a), sizeof(a)), 0);
    ::send(c, payload.data(), payload.size(), 0);
    ::close(c);
  };
  connect_send_close("AAA");
  EXPECT_TRUE(sink.wait_for_bytes(3, std::chrono::seconds(3)));
  connect_send_close("BBB");
  EXPECT_TRUE(sink.wait_for_bytes(6, std::chrono::seconds(3)))
      << "第一个对端断开后必须继续接受新连接,而不是退出";
  EXPECT_EQ(sink.str(), "AAABBB");
  s.stop();
}

TEST(TcpStream, ClientBackoffGrowsButIsCappedByMax) {
  // 连不上的端口 + 很小的 max_backoff:在固定时间窗内断开回调的次数应受上限约束,
  // 既不会退化成忙等(次数爆炸),也不会一次就放弃(次数为 0)。
  std::atomic<int> disconnects{0};
  TcpStreamConfig cfg;
  cfg.port = 1;
  cfg.initial_backoff_s = 0.05;
  cfg.max_backoff_s = 0.1;
  TcpStream s(cfg, [](const uint8_t*, size_t) {},
              [&](bool connected, const std::string&) { if (!connected) ++disconnects; });
  s.start();
  std::this_thread::sleep_for(std::chrono::milliseconds(600));
  s.stop();
  EXPECT_GE(disconnects.load(), 1) << "必须持续重试";
  EXPECT_LE(disconnects.load(), 20) << "必须退避,不能忙等";
}
```

> 注:状态回调对连不上的情况**只在跳变时发**,所以上面第三个用例断言的是"至少 1 次";若实现改成每次重试都发,上限断言会兜住忙等。两条断言合起来既防"放弃"也防"忙等"。

- [ ] **Step 2: 跑测试确认失败**

```bash
cd ~/glim_ws && colcon build --packages-select gnss_bringup --cmake-args -DBUILD_TESTING=ON && \
./build/gnss_bringup/test_tcp_stream --gtest_filter='TcpStream.Listen*:TcpStream.ClientBackoff*'
```

Expected: 3 个新用例 FAIL(`listen` 分支未实现,`bound_port()` 恒为 -1)。

- [ ] **Step 3: 写实现(`run_server()` 分支)**

要点:
- `socket` + `SO_REUSEADDR` + `bind` + `listen`;`getsockname` 取回实际端口写入 `bound_port_`。
- `poll(listen_fd, wake_fd)` 等连接;接受后进入与客户端相同的 `pump` 循环(同一段代码抽成 `pump(int fd)` 复用)。
- 对端断开后回到 accept 循环,**不退出线程**。
- `start()` 里根据 `cfg_.listen` 选 `run_server()` 或 `run_client()`。

- [ ] **Step 4: 跑测试确认通过**

Expected: 7 passed(Task 1 的 4 条 + 本任务 3 条)。

- [ ] **Step 5: Commit**

```bash
cd ~/glim_ws/src/glim_ext && git add gnss_bringup && \
git commit -m "feat(gnss_bringup): TcpStream listen mode and capped backoff"
```

---

### Task 3: `rtcm_bridge` 节点

**Files:**
- Create: `glim_ext/gnss_bringup/src/rtcm_bridge_node.cpp`
- Modify: `glim_ext/gnss_bringup/CMakeLists.txt`

**Interfaces:**
- Consumes: `TcpStream`(Task 1/2)、`gnss_msgs::msg::RawStream`
- Produces: 可执行 `rtcm_bridge`;发布 `/gnss/rtcm_corrections` 与 `/gnss/raw_obs`(`gnss_msgs/RawStream`),话题名由参数决定。Task 7 的 `rtkrcv_node` 订阅这两个。

- [ ] **Step 1: 写节点**

节点本身无算法,不单独写单测(行为已由 Task 1/2 的 `TcpStream` 测试覆盖);验收靠 Task 8 的冒烟。参数用**扁平前缀**而非嵌套列表,便于 yaml 与命令行覆盖:

```cpp
#include <memory>
#include <string>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <gnss_msgs/msg/raw_stream.hpp>
#include "gnss_bringup/tcp_stream.hpp"

namespace {

// 一路流 = 一个 TcpStream + 一个 publisher。收到多少发多少,不解析(spec §5.1)。
class BridgedStream {
public:
  BridgedStream(rclcpp::Node* node, const std::string& name)
      : node_(node), name_(name) {
    const std::string p = name + ".";
    const auto host = node->declare_parameter<std::string>(p + "host", "127.0.0.1");
    const auto port = node->declare_parameter<int>(p + "port", 0);
    const auto listen = node->declare_parameter<bool>(p + "listen", false);
    const auto topic = node->declare_parameter<std::string>(p + "topic", "/gnss/" + name);
    const auto frame = node->declare_parameter<std::string>(p + "frame_id", name);

    frame_ = frame;
    pub_ = node->create_publisher<gnss_msgs::msg::RawStream>(topic, rclcpp::QoS(100).reliable());

    gnss_bringup::TcpStreamConfig cfg;
    cfg.host = host;
    cfg.port = port;
    cfg.listen = listen;
    cfg.initial_backoff_s = node->declare_parameter<double>(p + "initial_backoff_s", 1.0);
    cfg.max_backoff_s = node->declare_parameter<double>(p + "max_backoff_s", 30.0);
    cfg.idle_timeout_s = node->declare_parameter<double>(p + "idle_timeout_s", 30.0);

    RCLCPP_INFO(node->get_logger(), "%s: %s %s:%d -> %s", name.c_str(),
                listen ? "listen" : "connect", host.c_str(), port, topic.c_str());

    stream_ = std::make_unique<gnss_bringup::TcpStream>(
        cfg,
        [this](const uint8_t* d, size_t n) { publish(d, n); },
        [this](bool connected, const std::string& detail) {
          RCLCPP_INFO(node_->get_logger(), "%s: %s %s", name_.c_str(),
                      connected ? "connected" : "disconnected", detail.c_str());
        });
    stream_->start();
  }

  ~BridgedStream() { if (stream_) stream_->stop(); }

private:
  void publish(const uint8_t* d, size_t n) {
    gnss_msgs::msg::RawStream msg;
    msg.header.stamp = node_->now();
    msg.header.frame_id = frame_;
    msg.data.assign(d, d + n);
    pub_->publish(msg);
    bytes_ += n;
  }

  rclcpp::Node* node_;
  std::string name_, frame_;
  rclcpp::Publisher<gnss_msgs::msg::RawStream>::SharedPtr pub_;
  std::unique_ptr<gnss_bringup::TcpStream> stream_;
  size_t bytes_ = 0;
};

}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("rtcm_bridge");
  // 默认两路:平台差分(A1)与板卡原始观测(A2)。streams 参数可增减。
  const auto names = node->declare_parameter<std::vector<std::string>>(
      "streams", std::vector<std::string>{"rtcm_corrections", "raw_obs"});
  std::vector<std::unique_ptr<BridgedStream>> streams;
  streams.reserve(names.size());
  for (const auto& n : names) streams.push_back(std::make_unique<BridgedStream>(node.get(), n));

  rclcpp::spin(node);
  streams.clear();          // 先停 TcpStream 线程,再让 node 析构
  rclcpp::shutdown();
  return 0;
}
```

- [ ] **Step 2: 加进 CMakeLists**

```cmake
add_executable(rtcm_bridge src/rtcm_bridge_node.cpp)
target_link_libraries(rtcm_bridge gnss_bringup_io)
ament_target_dependencies(rtcm_bridge rclcpp gnss_msgs)
install(TARGETS rtcm_bridge DESTINATION lib/${PROJECT_NAME})
```

- [ ] **Step 3: 构建 + 冒烟**

```bash
cd ~/glim_ws && colcon build --packages-select gnss_bringup && source install/setup.bash
# 用监听模式起,自己给自己灌一段字节
ros2 run gnss_bringup rtcm_bridge --ros-args \
  -p streams:="['rtcm_corrections']" \
  -p rtcm_corrections.listen:=true -p rtcm_corrections.port:=15031 &
ros2 topic echo /gnss/rtcm_corrections --once &
printf 'HELLO' | nc 127.0.0.1 15031
```

Expected: `ros2 topic echo` 打印出 `data: [72, 69, 76, 76, 79]`。

- [ ] **Step 4: Commit**

```bash
cd ~/glim_ws/src/glim_ext && git add gnss_bringup && \
git commit -m "feat(gnss_bringup): rtcm_bridge node (TCP to RawStream, no parsing)"
```

---

### Task 4: `LocalReserver`(本机 TCP 扇出)

**Files:**
- Create: `glim_ext/gnss_bringup/include/gnss_bringup/local_reserver.hpp`
- Create: `glim_ext/gnss_bringup/src/local_reserver.cpp`
- Modify: `glim_ext/gnss_bringup/CMakeLists.txt`(源文件 + 测试)
- Test: `glim_ext/gnss_bringup/test/test_local_reserver.cpp`

**Interfaces:**
- Consumes: 无
- Produces: `gnss_bringup::LocalReserver`,接口 `start(port, host)` / `bound_port()` / `broadcast(const uint8_t*, size_t)` / `stop()` / `client_count()`。Task 7 用两个实例把订到的 `RawStream` 喂给 rtkrcv。

- [ ] **Step 1: 写头文件**

```cpp
#pragma once
#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace gnss_bringup {

// 把一条字节流在本机 TCP 上再服务出去,供 rtkrcv(inpstr*-type=tcpcli)连入。
// 写不动的客户端会被强制断开,避免内核发送缓冲无界增长拖垮节点。
class LocalReserver {
public:
  ~LocalReserver();
  // port=0 时由内核选端口,经 bound_port() 取回
  bool start(int port, const std::string& host = "127.0.0.1");
  void stop();
  int bound_port() const { return bound_port_.load(); }
  size_t client_count() const;
  void broadcast(const uint8_t* data, size_t len);

private:
  void accept_loop();

  std::thread thread_;
  std::atomic<bool> running_{false};
  std::atomic<int> bound_port_{-1};
  int listen_fd_ = -1, wake_fd_ = -1, wake_wr_ = -1;
  mutable std::mutex m_;
  std::vector<int> clients_;
};

}  // namespace gnss_bringup
```

- [ ] **Step 2: 写失败测试**

```cpp
#include <gtest/gtest.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#include <chrono>
#include <string>
#include <thread>
#include "gnss_bringup/local_reserver.hpp"
using namespace gnss_bringup;

namespace {
int connect_to(int port) {
  int c = ::socket(AF_INET, SOCK_STREAM, 0);
  sockaddr_in a{};
  a.sin_family = AF_INET;
  a.sin_addr.s_addr = ::htonl(INADDR_LOOPBACK);
  a.sin_port = ::htons(static_cast<uint16_t>(port));
  if (::connect(c, reinterpret_cast<sockaddr*>(&a), sizeof(a)) != 0) { ::close(c); return -1; }
  return c;
}
bool wait_clients(const LocalReserver& r, size_t n) {
  for (int i = 0; i < 200; ++i) {
    if (r.client_count() >= n) return true;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  return false;
}
std::string recv_n(int fd, size_t n) {
  std::string out;
  char buf[256];
  while (out.size() < n) {
    const ssize_t k = ::recv(fd, buf, sizeof(buf), 0);
    if (k <= 0) break;
    out.append(buf, static_cast<size_t>(k));
  }
  return out;
}
}  // namespace

TEST(LocalReserver, BindsAndReportsPort) {
  LocalReserver r;
  ASSERT_TRUE(r.start(0));
  EXPECT_GT(r.bound_port(), 0);
  r.stop();
}

TEST(LocalReserver, BroadcastsToASingleClient) {
  LocalReserver r;
  ASSERT_TRUE(r.start(0));
  const int c = connect_to(r.bound_port());
  ASSERT_GE(c, 0);
  ASSERT_TRUE(wait_clients(r, 1));

  const std::string payload = "RTCM-BYTES";
  r.broadcast(reinterpret_cast<const uint8_t*>(payload.data()), payload.size());
  EXPECT_EQ(recv_n(c, payload.size()), payload);
  ::close(c);
  r.stop();
}

TEST(LocalReserver, BroadcastsToEveryConnectedClient) {
  LocalReserver r;
  ASSERT_TRUE(r.start(0));
  const int c1 = connect_to(r.bound_port());
  const int c2 = connect_to(r.bound_port());
  ASSERT_GE(c1, 0);
  ASSERT_GE(c2, 0);
  ASSERT_TRUE(wait_clients(r, 2));

  const std::string payload = "XYZ";
  r.broadcast(reinterpret_cast<const uint8_t*>(payload.data()), payload.size());
  EXPECT_EQ(recv_n(c1, 3), payload);
  EXPECT_EQ(recv_n(c2, 3), payload);
  ::close(c1);
  ::close(c2);
  r.stop();
}

TEST(LocalReserver, DropsDisconnectedClientFromCount) {
  LocalReserver r;
  ASSERT_TRUE(r.start(0));
  const int c = connect_to(r.bound_port());
  ASSERT_GE(c, 0);
  ASSERT_TRUE(wait_clients(r, 1));
  ::close(c);

  const std::string payload = "Z";
  // 对端已关闭,broadcast 时写失败 → 客户端应被摘掉
  for (int i = 0; i < 50 && r.client_count() > 0; ++i) {
    r.broadcast(reinterpret_cast<const uint8_t*>(payload.data()), payload.size());
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  EXPECT_EQ(r.client_count(), 0u);
  r.stop();
}

TEST(LocalReserver, BroadcastWithNoClientsIsANoop) {
  LocalReserver r;
  ASSERT_TRUE(r.start(0));
  const std::string payload = "nobody-listening";
  r.broadcast(reinterpret_cast<const uint8_t*>(payload.data()), payload.size());
  EXPECT_EQ(r.client_count(), 0u);
  r.stop();
}

TEST(LocalReserver, StopReleasesThePortAndIsIdempotent) {
  int port = 0;
  {
    LocalReserver r;
    ASSERT_TRUE(r.start(0));
    port = r.bound_port();
    ASSERT_GT(port, 0);
    r.stop();
    r.stop();                       // 第二次必须无害
    EXPECT_EQ(r.client_count(), 0u);
  }
  // 可观测的后果:同一端口能被重新绑定,说明上一个实例确实释放了它
  LocalReserver again;
  EXPECT_TRUE(again.start(port)) << "stop() 没释放端口,新实例绑不上";
  again.stop();
}

TEST(LocalReserver, StopWithoutStartLeavesNoBoundPort) {
  LocalReserver r;
  r.stop();
  EXPECT_EQ(r.bound_port(), -1);
  EXPECT_EQ(r.client_count(), 0u);
}
```

- [ ] **Step 3: 跑测试确认失败**

先写空桩(`start` 返回 false,其余空体)保证编译通过。Expected: 5 FAIL(`StopIsIdempotent...` 空桩即过,实现后仍须保持)。

- [ ] **Step 4: 写实现**

要点:
- accept 线程 `poll(listen_fd, wake_fd)`;新连接 `setsockopt(SO_SNDBUF)` 保持默认即可,但对每个客户端设 `O_NONBLOCK`,`broadcast` 里 `send(..., MSG_NOSIGNAL)`。
- `send` 返回 `EAGAIN` 或 `<0` 且非 `EINTR` → 关闭并摘掉该客户端(参考实现的 `_MAX_WRITE_BUFFER` 强制断开等价物)。
- 客户端发来的数据一律 `recv` 丢弃(rtkrcv 不会发,但不读会让对端阻塞)。
- `stop()` 幂等:`running_` CAS,关闭所有 fd,join 线程。

- [ ] **Step 5: 跑测试确认通过**

Expected: 6 passed。

- [ ] **Step 6: Commit**

```bash
cd ~/glim_ws/src/glim_ext && git add gnss_bringup && \
git commit -m "feat(gnss_bringup): LocalReserver TCP fan-out for rtkrcv"
```

---

### Task 5: `rtkrcv.conf` 生成

**Files:**
- Create: `glim_ext/gnss_bringup/include/gnss_bringup/rtkrcv_conf.hpp`
- Create: `glim_ext/gnss_bringup/src/rtkrcv_conf.cpp`
- Modify: `glim_ext/gnss_bringup/CMakeLists.txt`
- Test: `glim_ext/gnss_bringup/test/test_rtkrcv_conf.cpp`

**Interfaces:**
- Consumes: 无
- Produces: `gnss_bringup::RtkrcvConfParams` 与 `std::string render_rtkrcv_conf(const RtkrcvConfParams&)`。Task 7 调用它并写盘。

- [ ] **Step 1: 写头文件**

```cpp
#pragma once
#include <string>

namespace gnss_bringup {

// rtkrcv.conf 的可变项。键名针对 RTKLIB demo5。
// obs_format / corr_format 是 spec §13 的 P0 未定项:板卡原始观测格式与平台差分格式
// 现场确认前沿用 rtk-monitor 的假定值 "rtcm3"。二者是参数,改默认值不需要改代码。
struct RtkrcvConfParams {
  int obs_port = 15032;        // 本机喂板卡原始观测的端口(rtkrcv 作为 tcpcli 连入)
  int corr_port = 15031;       // 本机喂差分改正的端口
  int sol_port = 15020;        // rtkrcv 吐解的端口(rtkrcv 作为 tcpsvr)
  std::string obs_format = "rtcm3";
  std::string corr_format = "rtcm3";
  std::string pos_mode = "kinematic";
  int navsys = 63;
  double elmask = 10.0;
  std::string ar_mode = "continuous";
};

std::string render_rtkrcv_conf(const RtkrcvConfParams& p);

}  // namespace gnss_bringup
```

- [ ] **Step 2: 写失败测试**

```cpp
#include <gtest/gtest.h>
#include <string>
#include "gnss_bringup/rtkrcv_conf.hpp"
using namespace gnss_bringup;

namespace {
bool has_line(const std::string& conf, const std::string& line) {
  return conf.find(line + "\n") != std::string::npos;
}
}  // namespace

TEST(RtkrcvConf, WiresBothInputsAsLocalTcpClients) {
  // rtkrcv 必须主动连本机的两个端口,而不是自己去连平台——
  // 对外 TCP 只在 rtcm_bridge 里存在(spec §5.1 单一路径)
  RtkrcvConfParams p;
  p.obs_port = 15032;
  p.corr_port = 15031;
  const auto c = render_rtkrcv_conf(p);
  EXPECT_TRUE(has_line(c, "inpstr1-type =tcpcli"));
  EXPECT_TRUE(has_line(c, "inpstr1-path =127.0.0.1:15032"));
  EXPECT_TRUE(has_line(c, "inpstr2-type =tcpcli"));
  EXPECT_TRUE(has_line(c, "inpstr2-path =127.0.0.1:15031"));
}

TEST(RtkrcvConf, PublishesSolutionAsLocalTcpServer) {
  RtkrcvConfParams p;
  p.sol_port = 15020;
  const auto c = render_rtkrcv_conf(p);
  EXPECT_TRUE(has_line(c, "outstr1-type =tcpsvr"));
  EXPECT_TRUE(has_line(c, "outstr1-path =:15020"));
  EXPECT_TRUE(has_line(c, "outstr1-format =llh"));
}

TEST(RtkrcvConf, SolutionFormatIsHeadlessLlhInGpst) {
  // parse_llh_solution 按无表头、GPST 的 llh 列序解析;这三行是它的前提
  const auto c = render_rtkrcv_conf(RtkrcvConfParams{});
  EXPECT_TRUE(has_line(c, "out-solformat =llh"));
  EXPECT_TRUE(has_line(c, "out-outhead =off"));
  EXPECT_TRUE(has_line(c, "out-timesys =gpst"));
}

TEST(RtkrcvConf, StreamFormatsAreConfigurable) {
  // P0 未定:板卡原始格式若不是 rtcm3(如 novatel / ublox),只改参数不改代码
  RtkrcvConfParams p;
  p.obs_format = "novatel";
  p.corr_format = "rtcm3";
  const auto c = render_rtkrcv_conf(p);
  EXPECT_TRUE(has_line(c, "inpstr1-format =novatel"));
  EXPECT_TRUE(has_line(c, "inpstr2-format =rtcm3"));
}

TEST(RtkrcvConf, DefaultFormatsMatchRtkMonitorAssumption) {
  const auto c = render_rtkrcv_conf(RtkrcvConfParams{});
  EXPECT_TRUE(has_line(c, "inpstr1-format =rtcm3"));
  EXPECT_TRUE(has_line(c, "inpstr2-format =rtcm3"));
}

TEST(RtkrcvConf, PositioningOptionsAreConfigurable) {
  RtkrcvConfParams p;
  p.pos_mode = "static";
  p.elmask = 15.0;
  p.ar_mode = "fix-and-hold";
  p.navsys = 5;
  const auto c = render_rtkrcv_conf(p);
  EXPECT_TRUE(has_line(c, "pos1-posmode =static"));
  EXPECT_TRUE(has_line(c, "pos1-elmask =15"));
  EXPECT_TRUE(has_line(c, "pos2-armode =fix-and-hold"));
  EXPECT_TRUE(has_line(c, "pos1-navsys =5"));
}
```

- [ ] **Step 3: 跑测试确认失败 → 写实现 → 跑通过**

实现就是按 rtk-monitor 的 `_CONF_TEMPLATE` 逐行拼字符串(注意 `elmask` 用 `%g` 之类避免输出 `10.000000`)。Expected: 6 passed。

- [ ] **Step 4: Commit**

```bash
cd ~/glim_ws/src/glim_ext && git add gnss_bringup && \
git commit -m "feat(gnss_bringup): rtkrcv.conf rendering with configurable stream formats"
```

---

### Task 6: `ProcessSupervisor`(起停 + 崩溃退避)

**Files:**
- Create: `glim_ext/gnss_bringup/include/gnss_bringup/process_supervisor.hpp`
- Create: `glim_ext/gnss_bringup/src/process_supervisor.cpp`
- Create: `glim_ext/gnss_bringup/test/fake_rtkrcv.sh`
- Modify: `glim_ext/gnss_bringup/CMakeLists.txt`
- Test: `glim_ext/gnss_bringup/test/test_process_supervisor.cpp`

**Interfaces:**
- Consumes: 无
- Produces: `gnss_bringup::ProcessSupervisor`,接口 `start()` / `stop()` / `spawn_count()` / `current_delay_s()` / 构造参数 `{binary, args, cwd, restart_delay_s}`。Task 7 用它管 rtkrcv。

- [ ] **Step 1: 写假二进制 `test/fake_rtkrcv.sh`**

```bash
#!/usr/bin/env bash
# 假 rtkrcv:用于在没有 RTKLIB 的机器上测试进程监管。
#   fake_rtkrcv.sh die      —— 立即退出(模拟 conf 错误导致的崩溃循环)
#   fake_rtkrcv.sh live     —— 长活直到被杀
# 其余参数(-s -nc -r 2 -o <conf>)一律忽略,只用于验证传参不会导致启动失败。
mode="die"
for a in "$@"; do case "$a" in die|live) mode="$a";; esac; done
if [[ "$mode" == "live" ]]; then
  trap 'exit 0' TERM INT
  while true; do sleep 0.1; done
fi
exit 1
```

> 写完立刻 `chmod +x test/fake_rtkrcv.sh` —— 不可执行时 `execv` 直接失败,
> Task 6 的用例会以"子进程立刻退出"的形式红掉,排查方向完全错。

- [ ] **Step 2: 写头文件**

```cpp
#pragma once
#include <atomic>
#include <string>
#include <thread>
#include <vector>

namespace gnss_bringup {

struct ProcessSupervisorConfig {
  std::string binary;
  std::vector<std::string> args;
  std::string cwd;
  double restart_delay_s = 5.0;
  // 活得比这还短就认为是崩溃循环(坏二进制 / 坏 conf),重启间隔翻倍直到上限,
  // 否则一天能刷出几万条 connected/disconnected
  double crash_loop_life_s = 30.0;
  double max_restart_delay_s = 60.0;
};

// 起一个子进程并在它退出后重启,永不放弃。子进程放进独立进程组,
// stop() 对整个进程组发 SIGTERM,超时再 SIGKILL —— rtkrcv 会派生孙进程。
class ProcessSupervisor {
public:
  explicit ProcessSupervisor(ProcessSupervisorConfig cfg);
  ~ProcessSupervisor();
  void start();
  void stop();
  int spawn_count() const { return spawn_count_.load(); }
  double current_delay_s() const { return current_delay_.load(); }

private:
  void run();

  ProcessSupervisorConfig cfg_;
  std::thread thread_;
  std::atomic<bool> running_{false};
  std::atomic<int> spawn_count_{0};
  std::atomic<double> current_delay_{0.0};
  std::atomic<int> child_pid_{-1};
  int wake_fd_ = -1, wake_wr_ = -1;
};

}  // namespace gnss_bringup
```

- [ ] **Step 3: 写失败测试**

```cpp
#include <gtest/gtest.h>
#include <chrono>
#include <string>
#include <thread>
#include "gnss_bringup/process_supervisor.hpp"
using namespace gnss_bringup;

namespace {
// CMake 通过 target_compile_definitions 传入假二进制的绝对路径
const char* fake() { return FAKE_RTKRCV_PATH; }

ProcessSupervisorConfig cfg_for(const std::string& mode, double delay) {
  ProcessSupervisorConfig c;
  c.binary = fake();
  c.args = {mode};
  c.cwd = "/tmp";
  c.restart_delay_s = delay;
  c.crash_loop_life_s = 0.5;
  c.max_restart_delay_s = 0.4;
  return c;
}
}  // namespace

TEST(ProcessSupervisor, SpawnsTheChildOnce) {
  ProcessSupervisor s(cfg_for("live", 0.05));
  s.start();
  for (int i = 0; i < 100 && s.spawn_count() < 1; ++i)
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  EXPECT_EQ(s.spawn_count(), 1);
  s.stop();
}

TEST(ProcessSupervisor, RestartsAfterTheChildExits) {
  ProcessSupervisor s(cfg_for("die", 0.05));
  s.start();
  for (int i = 0; i < 200 && s.spawn_count() < 3; ++i)
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  EXPECT_GE(s.spawn_count(), 3) << "子进程退出后必须重启";
  s.stop();
}

TEST(ProcessSupervisor, BacksOffWhenChildDiesImmediately) {
  ProcessSupervisor s(cfg_for("die", 0.05));
  s.start();
  for (int i = 0; i < 200 && s.spawn_count() < 4; ++i)
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  EXPECT_GT(s.current_delay_s(), 0.05) << "崩溃循环必须拉长重启间隔";
  EXPECT_LE(s.current_delay_s(), 0.4) << "但不得超过 max_restart_delay_s";
  s.stop();
}

TEST(ProcessSupervisor, StopTerminatesALongLivedChild) {
  ProcessSupervisor s(cfg_for("live", 0.05));
  s.start();
  for (int i = 0; i < 100 && s.spawn_count() < 1; ++i)
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  const auto t0 = std::chrono::steady_clock::now();
  s.stop();
  const auto dt = std::chrono::steady_clock::now() - t0;
  EXPECT_LT(std::chrono::duration_cast<std::chrono::milliseconds>(dt).count(), 6000)
      << "SIGTERM 后最多等 5 s 再 SIGKILL";
}

TEST(ProcessSupervisor, StopWithoutStartSpawnsNothing) {
  ProcessSupervisor s(cfg_for("live", 0.05));
  s.stop();
  EXPECT_EQ(s.spawn_count(), 0) << "没 start 过就不该派生任何子进程";
}

TEST(ProcessSupervisor, MissingBinaryKeepsRetryingAndBacksOff) {
  // 二进制不存在时,exec 在子进程里失败 → 子进程立刻退出 → 属于崩溃循环。
  // 要断言的是"确实在重试"且"确实退避了",而不只是"没崩"。
  ProcessSupervisorConfig c = cfg_for("live", 0.05);
  c.binary = "/nonexistent/rtkrcv";
  ProcessSupervisor s(c);
  s.start();
  for (int i = 0; i < 100 && s.spawn_count() < 2; ++i)
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  const int spawns = s.spawn_count();
  const double delay = s.current_delay_s();
  s.stop();
  EXPECT_GE(spawns, 2) << "必须持续重试";
  EXPECT_GT(delay, 0.05) << "崩溃循环必须退避";
  EXPECT_LE(delay, 0.4) << "但不得超过 max_restart_delay_s";
}
```

CMake 侧:

```cmake
ament_add_gtest(test_process_supervisor test/test_process_supervisor.cpp)
target_link_libraries(test_process_supervisor gnss_bringup_io)
target_compile_definitions(test_process_supervisor PRIVATE
  FAKE_RTKRCV_PATH="${CMAKE_CURRENT_SOURCE_DIR}/test/fake_rtkrcv.sh")
```

- [ ] **Step 4: 跑测试确认失败 → 写实现 → 跑通过**

实现要点:
- `fork()` + `setsid()`(独立进程组)+ `chdir(cwd)` + `execv`;`execv` 失败时子进程 `_exit(127)`。
- 父进程 `waitpid` 等退出;记录存活时长,`< crash_loop_life_s` → `delay = min(delay*2, max)`,否则复位为 `restart_delay_s`。
- 退避睡眠用 `poll(wake_fd_, timeout_ms)`,`stop()` 写唤醒管道打断。
- `stop()`:置 `running_=false`,对 `-child_pid_`(进程组)发 `SIGTERM`,`waitpid` 带超时轮询最多 5 s,超时发 `SIGKILL`,再 join。

Expected: 6 passed。

- [ ] **Step 5: Commit**

```bash
cd ~/glim_ws/src/glim_ext && git add gnss_bringup && \
git commit -m "feat(gnss_bringup): ProcessSupervisor with crash-loop backoff"
```

---

### Task 7: `rtkrcv_node` 接线与 `RtkFix` 映射

**Files:**
- Create: `glim_ext/gnss_bringup/src/rtkrcv_node.cpp`
- Create: `glim_ext/gnss_bringup/include/gnss_bringup/rtk_fix_mapping.hpp`
- Modify: `glim_ext/gnss_bringup/CMakeLists.txt`
- Test: `glim_ext/gnss_bringup/test/test_rtk_fix_mapping.cpp`

**Interfaces:**
- Consumes: `TcpStream`(Task 1/2,用于连 rtkrcv 的 `sol_port`)、`LocalReserver`(Task 4)、`render_rtkrcv_conf`(Task 5)、`ProcessSupervisor`(Task 6)、`gnss_core::parse_llh_solution` / `q_to_quality`
- Produces: 可执行 `rtkrcv_node`;发布 `~/rtk_fix`(`gnss_msgs/RtkFix`,reliable)与 `~/stat`(`gnss_msgs/RawStream`,原始 `$SAT` 行,供轮 3 `gnss_diag` 用 `StatEpochAccumulator` 解析)。

- [ ] **Step 1: 写纯映射头 `rtk_fix_mapping.hpp`**

映射单独成纯函数,不碰 ROS 节点即可单测(与驱动侧 Task 8 的做法一致)。

```cpp
#pragma once
#include <gnss_msgs/msg/rtk_fix.hpp>
#include "gnss_core/pos_io.hpp"

namespace gnss_bringup {

// PosRecord(rtkrcv llh 解)→ RtkFix。
// 注意 σ 顺序:PosRecord.sdne 是 RTKLIB 的 (sdn, sde, sdu) = N/E/U,
// 而 RtkFix.sigma_enu 是 E/N/U —— 前两项必须交换(spec §4.1 v2)。
inline gnss_msgs::msg::RtkFix to_rtk_fix(const gnss_core::PosRecord& r) {
  gnss_msgs::msg::RtkFix m;
  m.gnss_time = r.stamp;                 // parse_llh_solution 已换算为 UTC unix 秒
  m.quality = static_cast<uint8_t>(gnss_core::q_to_quality(r.q));
  m.raw_status = static_cast<uint8_t>(r.q);   // 保留 RTKLIB 原始 Q 供追溯
  m.latitude = r.lat;
  m.longitude = r.lon;
  m.altitude = r.height;
  m.sigma_enu[0] = r.sdne(1);            // sde → E
  m.sigma_enu[1] = r.sdne(0);            // sdn → N
  m.sigma_enu[2] = r.sdne(2);            // sdu → U
  m.diff_age = static_cast<float>(r.age);
  m.sats_used = static_cast<uint8_t>(r.ns);
  m.sats_main = 0;
  m.sats_aux = 0;
  m.heading = 0.0f;
  m.heading_sigma = 0.0f;
  m.heading_valid = false;               // rtkrcv 单天线解无双天线航向
  return m;
}

}  // namespace gnss_bringup
```

- [ ] **Step 2: 写失败测试**

```cpp
#include <gtest/gtest.h>
#include "gnss_bringup/rtk_fix_mapping.hpp"
using namespace gnss_bringup;
using gnss_core::PosRecord;

namespace {
PosRecord sample() {
  PosRecord r;
  r.stamp = 1789045801.0;
  r.lat = 44.50123456;
  r.lon = 90.28765432;
  r.height = 617.123;
  r.q = 1;
  r.ns = 38;
  r.sdne = Eigen::Vector3d(0.011, 0.022, 0.033);   // sdn, sde, sdu
  r.age = 0.8;
  r.ratio = 20.5;
  return r;
}
}  // namespace

TEST(RtkFixMapping, SigmaIsReorderedFromNeuToEnu) {
  // 这是最容易出的错:RTKLIB 给 N/E/U,RtkFix 要 E/N/U
  const auto m = to_rtk_fix(sample());
  EXPECT_DOUBLE_EQ(m.sigma_enu[0], 0.022) << "E 应取 sde";
  EXPECT_DOUBLE_EQ(m.sigma_enu[1], 0.011) << "N 应取 sdn";
  EXPECT_DOUBLE_EQ(m.sigma_enu[2], 0.033) << "U 应取 sdu";
}

TEST(RtkFixMapping, RtklibQMapsToNormalizedQuality) {
  PosRecord r = sample();
  r.q = 1; EXPECT_EQ(to_rtk_fix(r).quality, gnss_msgs::msg::RtkFix::QUALITY_FIXED);
  r.q = 2; EXPECT_EQ(to_rtk_fix(r).quality, gnss_msgs::msg::RtkFix::QUALITY_FLOAT);
  r.q = 4; EXPECT_EQ(to_rtk_fix(r).quality, gnss_msgs::msg::RtkFix::QUALITY_DGPS);
  r.q = 5; EXPECT_EQ(to_rtk_fix(r).quality, gnss_msgs::msg::RtkFix::QUALITY_SINGLE);
  r.q = 0; EXPECT_EQ(to_rtk_fix(r).quality, gnss_msgs::msg::RtkFix::QUALITY_NONE);
}

TEST(RtkFixMapping, KeepsRawRtklibQForTraceability) {
  PosRecord r = sample();
  r.q = 2;
  EXPECT_EQ(to_rtk_fix(r).raw_status, 2);
}

TEST(RtkFixMapping, GnssTimeComesFromTheSolutionEpoch) {
  const auto m = to_rtk_fix(sample());
  EXPECT_DOUBLE_EQ(m.gnss_time, 1789045801.0);
}

TEST(RtkFixMapping, PositionAndAgeAndSatsAreCopied) {
  const auto m = to_rtk_fix(sample());
  EXPECT_NEAR(m.latitude, 44.50123456, 1e-9);
  EXPECT_NEAR(m.longitude, 90.28765432, 1e-9);
  EXPECT_NEAR(m.altitude, 617.123, 1e-9);
  EXPECT_NEAR(m.diff_age, 0.8f, 1e-6);
  EXPECT_EQ(m.sats_used, 38);
}

TEST(RtkFixMapping, HeadingIsAlwaysInvalidForSingleAntennaSolution) {
  EXPECT_FALSE(to_rtk_fix(sample()).heading_valid);
}
```

- [ ] **Step 3: 跑测试确认失败 → 实现(头文件即实现)→ 跑通过**

先把 `to_rtk_fix` 的函数体写成 `return gnss_msgs::msg::RtkFix{};` 看红,再填入 Step 1 的内容。Expected: 6 passed。

- [ ] **Step 4: 写节点 `rtkrcv_node.cpp`**

接线顺序(必须先起本机服务再起 rtkrcv,否则 rtkrcv 首次连接必失败走重连):

```
1. 读参数 → RtkrcvConfParams
2. LocalReserver corr_.start(corr_port) / obs_.start(obs_port)
3. render_rtkrcv_conf() → 写 <run_dir>/rtkrcv.conf
4. ProcessSupervisor 起 rtkrcv:  <binary> -s -nc -r 2 -o <run_dir>/rtkrcv.conf,cwd=run_dir
5. TcpStream 连 127.0.0.1:sol_port  → 按行切分 → parse_llh_solution → to_rtk_fix → 发 ~/rtk_fix
6. 订阅 corrections/raw_obs 两个 RawStream → 分别 broadcast 进两个 LocalReserver
7. 轮询 run_dir 下最新的 rtkrcv_*.stat,tail 新增内容 → 原样发 ~/stat(RawStream)
```

要点:
- **按行切分**:`TcpStream` 给的是任意切分的字节块,必须自己攒 `\n`;半行留在缓冲里等下一块(与 `RtcmFramer` 同样的道理)。
- `header.stamp = node->now()`(接收时刻),`gnss_time` 由映射填(解算历元)。二者之差即接收延迟,轮 3 可做 `stamp_skew` 诊断。
- `~/rtk_fix` 用 `rclcpp::QoS(100).reliable()`。
- 析构顺序:先 `ProcessSupervisor::stop()`(杀 rtkrcv),再 `TcpStream::stop()`,最后 `LocalReserver::stop()`。

- [ ] **Step 5: 构建 + 用假二进制冒烟**

```bash
cd ~/glim_ws && colcon build --packages-select gnss_bringup && source install/setup.bash
ros2 run gnss_bringup rtkrcv_node --ros-args \
  -p binary:=$(pwd)/src/gnss_bringup/test/fake_rtkrcv.sh -p args:="['live']" \
  -p run_dir:=/tmp/rtkrcv_run
cat /tmp/rtkrcv_run/rtkrcv.conf     # 另开终端
```

Expected: 节点不崩;`rtkrcv.conf` 内容与 Task 5 的断言一致;日志出现 `rtkrcv spawned pid ...`。**真实解流验证需要 RTKLIB,见 Task 8 的缺口登记。**

- [ ] **Step 6: Commit**

```bash
cd ~/glim_ws/src/glim_ext && git add gnss_bringup && \
git commit -m "feat(gnss_bringup): rtkrcv_node with RtkFix mapping (NEU->ENU sigma swap)"
```

---

### Task 8: launch、配置、文档与 spec 修正

**Files:**
- Create: `glim_ext/gnss_bringup/launch/gnss_bringup.launch.py`
- Create: `glim_ext/gnss_bringup/config/gnss_bringup.yaml`
- Create: `glim_ext/gnss_bringup/README.md`
- Modify: `glim_ext/gnss_bringup/CMakeLists.txt`(安装 launch/config)
- Modify: `../specs/2026-09-03-gnss-glim-modules-design.md`(§5.1 与 §13)

**Interfaces:**
- Consumes: Task 3 与 Task 7 的两个可执行
- Produces: `ros2 launch gnss_bringup gnss_bringup.launch.py` 一把起两个节点

- [ ] **Step 1: 写 `config/gnss_bringup.yaml`**

所有 P0 未定项集中在此,并就地注明"现场确认"。

```yaml
rtcm_bridge:
  ros__parameters:
    streams: ["rtcm_corrections", "raw_obs"]
    rtcm_corrections:
      # TODO(现场确认, spec §13 P0):平台差分的 IP/端口,以及是我们连它(listen=false)
      # 还是它推给我们(listen=true)。协议已确认为裸 TCP 无解析(rtk-monitor 无 NTRIP)。
      host: "127.0.0.1"
      port: 15031
      listen: false
      topic: "/gnss/rtcm_corrections"
    raw_obs:
      # TODO(现场确认, spec §13 P0):板卡原始观测输出的 IP/端口或串口转发端点
      host: "127.0.0.1"
      port: 15032
      listen: false
      topic: "/gnss/raw_obs"

rtkrcv_node:
  ros__parameters:
    binary: "rtkrcv"            # RTKLIB demo5;开发机未安装,见 README 缺口登记
    run_dir: "/tmp/rtkrcv_run"
    corr_port: 15041
    obs_port: 15042
    sol_port: 15020
    # TODO(现场确认, spec §13 P0):板卡原始观测格式。rtk-monitor 假定 rtcm3。
    obs_format: "rtcm3"
    corr_format: "rtcm3"
    pos_mode: "kinematic"
    navsys: 63
    elmask: 10.0
    ar_mode: "continuous"
```

- [ ] **Step 2: 写 launch 文件**

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params = PathJoinSubstitution([FindPackageShare("gnss_bringup"), "config", "gnss_bringup.yaml"])
    return LaunchDescription([
        DeclareLaunchArgument("params_file", default_value=params),
        # RTKLIB 未装时(见 README 缺口登记)用 enable_rtkrcv:=false 只起桥
        DeclareLaunchArgument("enable_rtkrcv", default_value="true"),
        Node(package="gnss_bringup", executable="rtcm_bridge", name="rtcm_bridge",
             parameters=[LaunchConfiguration("params_file")], output="screen"),
        Node(package="gnss_bringup", executable="rtkrcv_node", name="rtkrcv_node",
             parameters=[LaunchConfiguration("params_file")], output="screen",
             condition=IfCondition(LaunchConfiguration("enable_rtkrcv"))),
    ])
```

- [ ] **Step 3: 安装 launch/config**

```cmake
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
```

- [ ] **Step 4: 写 README 的缺口登记**

必须逐条写清、可被现场核对:

```markdown
## 现场待确认(spec §13 P0)

| # | 待确认 | 影响 | 现在的取值 | 确认后怎么改 |
|---|---|---|---|---|
| 1 | 平台差分的 IP:Port,以及连接方向 | rtcm_bridge 连不上就没有差分 | 127.0.0.1:15031, listen=false | 只改 config/gnss_bringup.yaml |
| 2 | 板卡原始观测的端点 | rtkrcv 无观测无法解算 | 127.0.0.1:15032, listen=false | 同上 |
| 3 | 板卡原始观测格式(inpstr1-format) | 格式错 rtkrcv 解不出 | rtcm3(沿用 rtk-monitor 假定) | 只改 yaml 的 obs_format |
| 4 | 平台差分格式(inpstr2-format) | 同上 | rtcm3 | 只改 yaml 的 corr_format |

以上四项**都不需要改代码**。

## 未验证项

- **RTKLIB 未安装**:本包在开发机上用 `test/fake_rtkrcv.sh` 验证进程监管、conf 生成与流转发;
  rtkrcv 的**真实 conf 键名**与解流格式未对真实二进制验证过。rtk-monitor 的注释同样写明
  这是集成步骤而非单测。装上 demo5 版 RTKLIB 后须补:conf 被 rtkrcv 接受、`outstr1` 有 llh 输出、
  `-r 2` 生成 `rtkrcv_*.stat`。
- **端到端**:`rtcm_bridge` → `rtkrcv_node` → `~/rtk_fix` 的全链路需要真实差分与观测流。
```

- [ ] **Step 5: 修正 spec §5.1(connect-or-listen)**

把 "`rtcm_bridge` 保持"笨":TCP 客户端 → 收到多少发多少 → 断线重连" 改为:

> `rtcm_bridge` 保持"笨":**TCP 连接或监听**(`listen` 参数;平台主动推流时用监听模式)→ 收到多少发多少 → 断线重连/继续等待下一个连接。**不做任何解析**。

并在 §13 的 P0 行补一句:"协议已确认为裸 TCP(参考实现 rtk-monitor 全仓无 NTRIP);待确认的是端点、方向与两个 `inpstr*-format` 的取值,均为配置项,不影响代码结构。"

- [ ] **Step 6: 全 workspace 构建 + 全测试**

```bash
cd ~/glim_ws && bash src/glim_ext/setup_workspace.sh && \
source /opt/ros/humble/setup.bash && source ~/driver_ws/install/setup.bash && \
colcon build --symlink-install && \
colcon test --packages-select gnss_core gnss_bringup --event-handlers console_direct+ && \
colcon test-result --all
```

Expected: 全部包构建成功,`gnss_core` 与 `gnss_bringup` 测试全绿。

- [ ] **Step 7: Commit**

```bash
cd ~/glim_ws/src/glim_ext && git add gnss_bringup && \
git commit -m "feat(gnss_bringup): launch, params and field-gap registry"
cd ~/glim_ws/src/glim_underground && git add docs/gnss/specs && \
git commit -m "docs(spec): rtcm_bridge is connect-or-listen; narrow the round-2 P0 items"
```

---

## 验收(轮 2 两个壳完成标志)

- `colcon build --symlink-install` 在 `glim_ws` 全通过;`gnss_bringup` 出现在 `colcon list` 里
- `gnss_bringup` 全部 gtest 通过:`test_tcp_stream`(7)、`test_local_reserver`(7)、`test_rtkrcv_conf`(6)、`test_process_supervisor`(6)、`test_rtk_fix_mapping`(6)= **32 用例**
- `rtcm_bridge` 监听模式冒烟:`nc` 灌进去的字节出现在 `/gnss/rtcm_corrections`
- `rtkrcv_node` 用假二进制冒烟:生成的 `rtkrcv.conf` 与 Task 5 断言一致,进程被监管且能被干净停掉
- `README.md` 的四项现场待确认与两项未验证项已登记
- spec §5.1 已改为 connect-or-listen;§13 的 P0 行已收窄

## 遗留(不在本计划)

- `.pos` 1 Hz 写出节点(D1):core 的 `write_pos` + `PosDecimator` 已就绪
- rosbag2 录制脚本(A5/F1)
- 装 RTKLIB 后的真实 rtkrcv 集成验证
- 轮 3:九条规则 + 事件机 + `gnss_diag` 双壳(消费本轮发出的 `~/stat` 与 `/gnss/rtcm_corrections`)
