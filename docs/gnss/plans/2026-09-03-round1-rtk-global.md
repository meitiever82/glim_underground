# 轮 1:gnss_msgs + gnss_core + rtk_global 实施计划(v2)

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**版本:** v2(2026-09-07)。依据 `../specs/2026-09-07-round1-review.md` 修订;spec 同步为 v2。改动摘要:Task 1 `RtkFix` 加 `gnss_time`;Task 2 加 `effective_stamp`;Task 6 冻结语义显式化;Task 7 参数名 `body_point`;Task 8 QoS/DR 航向/σ 顺序/gnss_time;Task 9 重写(glim_ext 单包集成、`SubmapAnchor` 跨线程拷贝、`stamp_source`/`time_offset`、bag 验证取代 ctypes);Task 10 GPST/UTC;Task 11 中位数与 RMS 比;新增 Task 13 合成注入、Task 14 杆臂/时间偏移估计。

**Goal:** 交付质量感知的 GNSS 全局约束模块 `rtk_global`,连同其纯 C++ 算法核心 `gnss_core`、统一消息包 `gnss_msgs`、驱动增发的 `RtkFix` topic、无 LiDAR 的轨迹对比与定权系数标定工具、合成注入验证工具,以及杆臂/时间偏移估计工具。

**Architecture:** 纯 C++17 算法核心 `gnss_core`(无 ROS、无 GLIM 依赖,依赖 Eigen/GTSAM/GeographicLib),被三个薄壳复用。本轮只做 `rtk_global`(GLIM extension module 壳,编入 `glim_ext` 包)与离线工具;`rtk_odometry` 在轮 1.5。算法单元(噪声策略、fix 缓冲、帧对齐、带杆臂因子、时间戳选择)全部可脱离 ROS/GLIM 单测。

**Tech Stack:** C++17、CMake、ament_cmake、Eigen3、GTSAM ≥4.2(Orin 4.3;CI/云端 4.2)、gtsam_points、GeographicLib 2.3、GLIM extension module API、GoogleTest、ROS2 Jazzy、Python 3 + rosbag2_py(工具)。

**Spec:** `../specs/2026-09-03-gnss-glim-modules-design.md`(v2)

## Global Constraints

- `gnss_core` 严禁依赖 ROS 与 GLIM;仅允许 Eigen / GTSAM / GeographicLib(spec §6)。
- 算法只写一遍在 `gnss_core`,壳只做订阅、类型转换、回调注册(spec §2.1)。
- 坐标换算一律用 `GeographicLib::LocalCartesian`,不自写第四份 geodetic;局部 ENU 而非 UTM(spec §6.1)。
- `RtkFix` 质量枚举归一化、源无关:`QUALITY_NONE=0 SINGLE=1 DGPS=2 FLOAT=3 FIXED=4`(spec §4.1)。
- `T_world_enu` bootstrap 一次后冻结,不重估(spec §6.1 v2)。
- 自定义因子须过 `gtsam::numericalDerivative11` 的 Jacobian 数值校验(spec §12.1);只用 GTSAM 4.2/4.3 共有接口,CMake 不锁版本。
- `rtk_global` 与 `gnss_global` 互斥,不得同时启用(spec §7.6)。
- **`glim_ext` 是一个 ament 包**:新模块通过顶层 `CMakeLists.txt` 的 `option` + `add_subdirectory` 编入,依赖声明在 `glim_ext/package.xml`,构建命令 `colcon build --packages-select glim_ext`,产物在 `install/glim_ext/lib/`(spec §7.6 v2)。
- 后台线程不持有 `SubMap` 指针;`on_insert_submap` 回调内拷贝 POD 入队(spec §7.1 v2)。
- 工作区(2026-09-07 核实的实际布局):
  - `gnss_msgs`、`gnss_CGI610` 正本在 **`~/Documents/GitHub/ztpilot/finder_ros/drivers/`**(finder_ros 仓库管理),`~/driver_ws/src/` 下是指向它们的符号链接;commit 在 finder_ros 仓库做。
  - `gnss_core` 已建在 **`~/glim_ws/src/glim_ext/gnss_core/`**(glim_ext 仓库内,colcon 在 `glim_ws` 里把它识别为独立包 `gnss_core`,`build/gnss_core` 已存在);`rtk_global` 在 `~/glim_ws/src/glim_ext/modules/mapping/`。
  - **构建 `glim_ws` 前先 `source ~/driver_ws/install/setup.bash`**(拿 `gnss_msgs`)。
- 目标平台 aarch64(Orin);系统已装 GTSAM 4.3(`/usr/local/include/gtsam`)、GeographicLib 2.3(`/usr/share/cmake/geographiclib/FindGeographicLib.cmake`)。
- `gnss_core` 可在无 ROS 的环境(云端/CI,Ubuntu 24.04 apt:`libgtsam-dev 4.2`、`libgeographiclib-dev 2.3`、`libgtest-dev`)以纯 CMake 构建与测试,便于脱离 Orin 做 TDD;ament 路径与纯 CMake 路径共用同一 `CMakeLists.txt`(Task 2)。
- 每个任务 TDD:先写失败测试 → 跑失败 → 最小实现 → 跑通过 → 提交。

---

## 现状与本轮增量(2026-09-07 核实)

已在 Orin 上完成并通过测试(`glim_ws/build/gnss_core`,`colcon test` rc=0,test_types/geodetic/rtk_fix_buffer(4)/rtk_noise_policy(5)/frame_aligner(2) 全绿):

| Task | v1 状态 | v2 增量(本轮要做) |
|---|---|---|
| 1 `gnss_msgs` | 已建(finder_ros/drivers) | 加 `float64 gnss_time`;重建 |
| 2 `types` | 已建 | `RtkFixSample` 加 `header_stamp`/`gnss_time`;加 `StampSource`/`effective_stamp()` + 4 个测试;CMake 改为 ament/纯 CMake 双路径 + `gnss_core_add_test` 宏 |
| 3 `geodetic` | 已建 | 加 `reverse()`(Task 13 用) + 往返测试 |
| 4 `RtkFixBuffer` | 已建 | 加 `latest_stamp()`;`prune` 注释 |
| 5 `RtkNoisePolicy` | 已建 | 构造函数配置校验 + 测试;`sigma_floor` 默认改 1.0 m,现有 floor 测试显式设 cfg |
| 6 `FrameAligner` | 已建(已是冻结语义) | 加 `FrozenAfterInitialization` 测试;头文件注释写明冻结 |
| 7 `AntennaPriorFactor` | **未做** | 全部 |
| 8 驱动增发 | 未做 | 全部 |
| 9 `rtk_global` | 未做 | 全部 |
| 10 `pos_io` | 仅空壳 | 全部 |
| 11 `trajectory_compare` | 仅空壳 | 全部 |
| 12–14 工具 | 未做 | 全部 |

下文各 Task 的步骤对已完成部分照旧保留(作为规格记录),执行时只做"v2 增量"列的内容;已通过的测试不得回退。

**2026-09-07 进展**:Task 2–7、10–14 全部完成并经独立评审修复(云端 GTSAM 4.2a9 纯 CMake:10 个测试可执行 60 用例全绿,`-Wall -Wextra` 零 warning),已写回 `glim_ws/src/glim_ext/gnss_core/`。评审后追加的接口:`RtkFixBuffer::interpolate(t, max_gap_s)`、`oldest_stamp()`、`AntennaPriorFactor::create()`、`RtkNoisePolicy` 拒绝越界 quality 与非有限 σ、`FrameAligner` 退化保护(ENU 基线 + SVD 奇异值)。**待 Orin 上 GTSAM 4.3 复核**(`antenna_prior_factor.hpp` 的 4.3 分支未在云端编过)。评审遗留(带到 1.5):`RtkNoisePolicy` 拒绝原因枚举、航向 360° 回绕插值、`trajectory_compare` 的 ref 质量过滤与 Rayleigh 中位数偏差说明、`export_bag_to_pos.py --db` 未实现(rtk-monitor SQLite schema 不可见)。剩余:Task 1 增量、8、9(需 Orin)。

## 文件结构

**已有包 `gnss_msgs`**(`finder_ros/drivers/gnss_msgs`,v1 已建好并可构建;本轮只增 `gnss_time` 字段):
- `gnss_msgs/msg/RtkFix.msg` — 带质量标签的 GNSS 定位
- `gnss_msgs/msg/RawStream.msg` — 裸字节流
- `gnss_msgs/CMakeLists.txt` / `package.xml`

**已有包 `gnss_core`**(`glim_ext/gnss_core`,v1 的 Task 2–6 已实现并在 Orin 上测试通过;本轮按 v2 增量修改,并补 Task 7、10–14):
- `include/gnss_core/types.hpp` — `Quality` 枚举、`RtkFixSample`(含 `gnss_time`)、`EnuPoint`、`StampSource`、`effective_stamp()`
- `include/gnss_core/geodetic.hpp` + `src/geodetic.cpp` — `LlaToEnu`(封装 GeographicLib::LocalCartesian)
- `include/gnss_core/rtk_fix_buffer.hpp` + `src/rtk_fix_buffer.cpp` — 时间插值缓冲
- `include/gnss_core/rtk_noise_policy.hpp` + `src/rtk_noise_policy.cpp` — 质量→noise model / 拒绝
- `include/gnss_core/frame_aligner.hpp` + `src/frame_aligner.cpp` — SVD 求 `T_world_enu`
- `include/gnss_core/antenna_prior_factor.hpp` — GTSAM 自定义因子(header-only)
- `include/gnss_core/trajectory_compare.hpp` + `src/trajectory_compare.cpp` — 分档误差统计
- `include/gnss_core/pos_io.hpp` + `src/pos_io.cpp` — `.pos` 读取(GPST/UTC 识别)
- `include/gnss_core/synth.hpp` + `src/synth.cpp` — 轨迹 → 合成 RTK 观测(含故障注入)、`read_glim_traj`(Task 13)
- `include/gnss_core/lever_arm_estimator.hpp` + `src/lever_arm_estimator.cpp` — 杆臂 + `T_world_enu` + 时间偏移联合估计(Task 14)
- `test/` — 每单元一个 gtest

**新建模块 `glim_ext/modules/mapping/rtk_global`**(是 `glim_ext` 包的子目录,不是独立包):
- `include/glim_ext/rtk_global_module.hpp` — `ExtensionModuleROS2` 子类
- `src/glim_ext/rtk_global_module_ros2.cpp` — 接线 + `create_extension_module()`
- `CMakeLists.txt`(仅 `add_library` + 依赖,无 `ament_package()`)

**修改 `glim_ext` 包**:`glim_ext/CMakeLists.txt`(`option(ENABLE_RTK_GLOBAL)` + `add_subdirectory`)、`glim_ext/package.xml`(`<depend>gnss_core</depend>` `<depend>gnss_msgs</depend>`)

**新建配置**:`glim_ext/config/config_rtk_global.json`

**修改驱动**:`driver_ws/src/gnss_CGI610`(增发 `~/rtk_fix`)

**新建工具**(`gnss_core/tools/`):
- `calibrate_sigma_scale.cpp` — 读 ref/test `.pos` → 分档统计 → 打印标定表(Task 12)
- `export_bag_to_pos.py` — rtk-monitor 既有录包 → `.pos`(Task 12)
- `synth_rtk_fix.cpp` + `pos_to_rtkfix_bag.py` — GLIM 轨迹 → 合成 RTK 观测(含故障注入)→ `.pos` / `RtkFix` bag(Task 13)
- `estimate_lever_arm.cpp` — GLIM 轨迹 + ENU 轨迹 → `lever_imu`、`time_offset`(Task 14)

---

### Task 1: gnss_msgs 消息包(已有,增 gnss_time)

**Files:**(正本 `finder_ros/drivers/gnss_msgs/`,`driver_ws/src/gnss_msgs` 是其符号链接)
- Modify: `msg/RtkFix.msg`(加 `gnss_time`)
- 已有、不动: `msg/RawStream.msg`、`CMakeLists.txt`、`package.xml`

**Interfaces:**
- Produces: `gnss_msgs/msg/RtkFix`(字段见下)、`gnss_msgs/msg/RawStream`。供 Task 8(驱动)、Task 9(模块壳)使用。

- [ ] **Step 1: 写 RtkFix.msg**

```
# gnss_msgs/RtkFix.msg —— 带质量标签的 GNSS 定位
std_msgs/Header header    # 接收/发布时刻(ROS 时钟)
float64 gnss_time         # 板卡自报观测时刻(UTC unix 秒,由 GPS 周/周内秒换算);0 = 源不提供

uint8 QUALITY_NONE=0
uint8 QUALITY_SINGLE=1
uint8 QUALITY_DGPS=2
uint8 QUALITY_FLOAT=3
uint8 QUALITY_FIXED=4
uint8 quality
uint8 raw_status

float64 latitude
float64 longitude
float64 altitude
float64[3] sigma_enu

float32 diff_age
uint8 sats_used
uint8 sats_main
uint8 sats_aux

float32 heading
float32 heading_sigma
bool heading_valid
```

- [ ] **Step 2: 写 RawStream.msg**

```
# gnss_msgs/RawStream.msg —— 裸字节流(差分/原始观测通用)
std_msgs/Header header
uint8[] data
```

- [ ] **Step 3: 写 package.xml**

```xml
<?xml version="1.0"?>
<package format="3">
  <name>gnss_msgs</name>
  <version>0.1.0</version>
  <description>GNSS messages with quality labels for the GLIM ecosystem</description>
  <maintainer email="dev@example.com">dev</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <buildtool_depend>rosidl_default_generators</buildtool_depend>
  <depend>std_msgs</depend>
  <exec_depend>rosidl_default_runtime</exec_depend>
  <member_of_group>rosidl_interface_packages</member_of_group>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

- [ ] **Step 4: 写 CMakeLists.txt**

```cmake
cmake_minimum_required(VERSION 3.8)
project(gnss_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RtkFix.msg"
  "msg/RawStream.msg"
  DEPENDENCIES std_msgs
)

ament_package()
```

- [ ] **Step 5: 构建验证**

Run: `cd /home/steve/driver_ws && colcon build --packages-select gnss_msgs`
Expected: 构建成功,生成 `gnss_msgs/msg/RtkFix` 类型。

- [ ] **Step 6: 类型可见性验证**

Run: `cd /home/steve/driver_ws && source install/setup.bash && ros2 interface show gnss_msgs/msg/RtkFix`
Expected: 打印完整字段定义,含 `QUALITY_FIXED=4` 常量与 `float64 gnss_time`。

- [ ] **Step 7: Commit**

```bash
cd /home/steve/Documents/GitHub/ztpilot/finder_ros && git add drivers/gnss_msgs && \
git commit -m "feat(gnss_msgs): RtkFix and RawStream messages"
```

---

### Task 2: gnss_core 包骨架 + types

**Files:**
- Create: `gnss_core/CMakeLists.txt`
- Create: `gnss_core/package.xml`
- Create: `gnss_core/include/gnss_core/types.hpp`
- Test: `gnss_core/test/test_types.cpp`

**Interfaces:**
- Produces: `gnss_core::Quality`(enum: `NONE=0,SINGLE=1,DGPS=2,FLOAT=3,FIXED=4`)、`struct RtkFixSample`(`stamp` = 已选定并加偏移后的有效时间;`header_stamp`/`gnss_time` 为原始值)、`struct EnuPoint`、`enum StampSource`、`double effective_stamp(header_stamp, gnss_time, source, offset)`。所有后续 Task 消费。

- [ ] **Step 1: 写 types.hpp**

```cpp
#pragma once
#include <cstdint>
#include <Eigen/Core>

namespace gnss_core {

enum class Quality : uint8_t { NONE = 0, SINGLE = 1, DGPS = 2, FLOAT = 3, FIXED = 4 };

enum class StampSource { Header, GnssTime };

// 选择样本的有效时间:GnssTime 且 gnss_time>0 → gnss_time,否则 header_stamp;再加 offset
inline double effective_stamp(double header_stamp, double gnss_time, StampSource src, double offset) {
  const double base = (src == StampSource::GnssTime && gnss_time > 0.0) ? gnss_time : header_stamp;
  return base + offset;
}

// 源无关的一条 GNSS 定位样本(与 gnss_msgs/RtkFix 对应,但不依赖 ROS 类型)
struct RtkFixSample {
  double stamp = 0.0;            // 有效时间(unix s):壳侧用 effective_stamp() 填
  double header_stamp = 0.0;     // 原始 header.stamp
  double gnss_time = 0.0;        // 原始板卡时间(0 = 无)
  Quality quality = Quality::NONE;
  double lat = 0.0, lon = 0.0, alt = 0.0;   // WGS-84, deg/deg/m
  Eigen::Vector3d sigma_enu = Eigen::Vector3d::Zero();  // m, E/N/U
  double diff_age = 0.0;         // s
  int sats_used = 0;
  double heading = 0.0;          // deg
  bool heading_valid = false;
};

struct EnuPoint {
  double stamp = 0.0;
  Eigen::Vector3d enu = Eigen::Vector3d::Zero();
  Quality quality = Quality::NONE;
};

}  // namespace gnss_core
```

- [ ] **Step 2: 写 package.xml**

```xml
<?xml version="1.0"?>
<package format="3">
  <name>gnss_core</name>
  <version>0.1.0</version>
  <description>Framework-free GNSS algorithms (no ROS, no GLIM)</description>
  <maintainer email="dev@example.com">dev</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <depend>eigen</depend>
  <test_depend>ament_cmake_gtest</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

- [ ] **Step 3: 写 CMakeLists.txt(库 + 依赖发现,ament / 纯 CMake 双路径)**

注:GeographicLib 用 Find 模块(非 Config),须先把其 cmake 目录加入 `CMAKE_MODULE_PATH`。`ament_cmake` 用 `QUIET` 探测:找到走 ament 导出与 `ament_add_gtest`,找不到(云端/CI)走纯 CMake + `GTest::gtest_main` + `add_test`,同一份源码与测试。

```cmake
cmake_minimum_required(VERSION 3.16)
project(gnss_core)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake QUIET)
find_package(Eigen3 REQUIRED)
find_package(GTSAM REQUIRED)            # 4.2 或 4.3 均可,不锁版本
list(APPEND CMAKE_MODULE_PATH "/usr/share/cmake/geographiclib")
find_package(GeographicLib REQUIRED)

add_library(gnss_core SHARED
  src/geodetic.cpp
  src/rtk_fix_buffer.cpp
  src/rtk_noise_policy.cpp
  src/frame_aligner.cpp
  src/trajectory_compare.cpp
  src/pos_io.cpp
  src/lever_arm_estimator.cpp
)
target_include_directories(gnss_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${GeographicLib_INCLUDE_DIRS}
)
target_link_libraries(gnss_core PUBLIC Eigen3::Eigen gtsam ${GeographicLib_LIBRARIES})

install(DIRECTORY include/ DESTINATION include)
install(TARGETS gnss_core EXPORT gnss_core-targets
  LIBRARY DESTINATION lib ARCHIVE DESTINATION lib RUNTIME DESTINATION bin)

# --- 测试注册:一个宏,两条路径 ---
function(gnss_core_add_test name)
  if(ament_cmake_FOUND)
    find_package(ament_cmake_gtest REQUIRED)
    ament_add_gtest(${name} test/${name}.cpp)
    target_link_libraries(${name} gnss_core)
  else()
    find_package(GTest REQUIRED)
    add_executable(${name} test/${name}.cpp)
    target_link_libraries(${name} gnss_core GTest::gtest_main)
    add_test(NAME ${name} COMMAND ${name})
  endif()
endfunction()

if(BUILD_TESTING)
  enable_testing()
  gnss_core_add_test(test_types)
  # 后续 Task 依次追加:test_geodetic test_rtk_fix_buffer test_rtk_noise_policy
  #                    test_frame_aligner test_antenna_prior_factor test_pos_io
  #                    test_trajectory_compare test_lever_arm_estimator
endif()

if(ament_cmake_FOUND)
  ament_export_targets(gnss_core-targets HAS_LIBRARY_TARGET)
  ament_export_dependencies(Eigen3 GTSAM)
  ament_package()
else()
  install(EXPORT gnss_core-targets DESTINATION lib/cmake/gnss_core)
endif()
```

纯 CMake 路径的构建命令(云端/CI):
```bash
cmake -S glim_ext/gnss_core -B build -DBUILD_TESTING=ON && cmake --build build -j && ctest --test-dir build --output-on-failure
```

因 CMakeLists 引用了尚未创建的 src 文件,先建空实现占位以便本 Task 能编译。后续 Task 会填。

- [ ] **Step 4: 建空实现占位**

为 `geodetic/rtk_fix_buffer/rtk_noise_policy/frame_aligner/trajectory_compare/pos_io/lever_arm_estimator` 各建一个仅含 `#include` 对应头文件的空 `.cpp`,头文件先建最小空壳(仅 `#pragma once` + namespace)。这些在各自 Task 中填实。types.hpp 已完整。

- [ ] **Step 5: 写失败测试 test_types.cpp**

```cpp
#include <gtest/gtest.h>
#include "gnss_core/types.hpp"
using namespace gnss_core;

TEST(Types, QualityEnumValues) {
  EXPECT_EQ(static_cast<uint8_t>(Quality::NONE), 0);
  EXPECT_EQ(static_cast<uint8_t>(Quality::FIXED), 4);
}

TEST(Types, RtkFixSampleDefaults) {
  RtkFixSample s;
  EXPECT_EQ(s.quality, Quality::NONE);
  EXPECT_FALSE(s.heading_valid);
  EXPECT_EQ(s.gnss_time, 0.0);
}

TEST(Types, EffectiveStampPrefersGnssTime) {
  EXPECT_DOUBLE_EQ(effective_stamp(100.0, 99.95, StampSource::GnssTime, 0.0), 99.95);
}

TEST(Types, EffectiveStampFallsBackToHeaderWhenGnssTimeZero) {
  EXPECT_DOUBLE_EQ(effective_stamp(100.0, 0.0, StampSource::GnssTime, 0.0), 100.0);
}

TEST(Types, EffectiveStampHeaderSourceIgnoresGnssTime) {
  EXPECT_DOUBLE_EQ(effective_stamp(100.0, 99.95, StampSource::Header, 0.0), 100.0);
}

TEST(Types, EffectiveStampAppliesOffset) {
  EXPECT_DOUBLE_EQ(effective_stamp(100.0, 0.0, StampSource::Header, -0.03), 99.97);
}
```

- [ ] **Step 6: 构建并跑测试**

Run(Orin,ament): `cd /home/steve/glim_ws && colcon build --packages-select gnss_core --cmake-args -DBUILD_TESTING=ON && colcon test --packages-select gnss_core --event-handlers console_direct+`
Run(云端/CI,纯 CMake): 见 Step 3 末尾命令。
Expected: 构建成功,test_types 6 passed。**这一步同时验证了 GTSAM/GeographicLib 的 find_package 链路(spec §13 首个风险)。**

- [ ] **Step 7: Commit**

```bash
cd /home/steve/glim_ws/src/glim_ext && git add gnss_core && \
git commit -m "feat(gnss_core): package skeleton + core types"
```

---

### Task 3: geodetic(LlaToEnu 封装 GeographicLib)

**Files:**
- Modify: `gnss_core/include/gnss_core/geodetic.hpp`
- Modify: `gnss_core/src/geodetic.cpp`
- Test: `gnss_core/test/test_geodetic.cpp`(新建,并在 CMakeLists 注册)

**Interfaces:**
- Consumes: 无
- Produces: `class LlaToEnu { LlaToEnu(double lat0,double lon0,double alt0); Eigen::Vector3d forward(double lat,double lon,double alt) const; Eigen::Vector3d reverse(const Eigen::Vector3d& enu) const; /* → [lat,lon,alt],Task 13 补 */ };`

- [ ] **Step 1: 写头文件**

```cpp
#pragma once
#include <Eigen/Core>
#include <memory>

namespace GeographicLib { class LocalCartesian; }

namespace gnss_core {

// 经纬高 → 局部 ENU(米)。原点固定于构造时给定的 lat0/lon0/alt0。
class LlaToEnu {
public:
  LlaToEnu(double lat0, double lon0, double alt0);
  ~LlaToEnu();
  Eigen::Vector3d forward(double lat, double lon, double alt) const;  // 返回 [E,N,U]
private:
  std::unique_ptr<GeographicLib::LocalCartesian> impl_;
};

}  // namespace gnss_core
```

- [ ] **Step 2: 写失败测试 test_geodetic.cpp**

```cpp
#include <gtest/gtest.h>
#include "gnss_core/geodetic.hpp"

TEST(Geodetic, OriginIsZero) {
  gnss_core::LlaToEnu conv(44.5, 90.28, 617.0);
  const auto p = conv.forward(44.5, 90.28, 617.0);
  EXPECT_NEAR(p.norm(), 0.0, 1e-6);
}

TEST(Geodetic, OneMetreNorth) {
  gnss_core::LlaToEnu conv(44.5, 90.28, 617.0);
  const double dlat = 1.0 / 111320.0;   // ≈ 1 m 纬度
  const auto p = conv.forward(44.5 + dlat, 90.28, 617.0);
  EXPECT_NEAR(p.x(), 0.0, 0.02);        // E
  EXPECT_NEAR(p.y(), 1.0, 0.02);        // N
  EXPECT_NEAR(p.z(), 0.0, 0.02);        // U
}
```

在 CMakeLists 的 `if(BUILD_TESTING)` 块内添加 `gnss_core_add_test(test_geodetic)`。

- [ ] **Step 3: 跑测试确认失败**

Run: `cd /home/steve/glim_ws && colcon build --packages-select gnss_core --cmake-args -DBUILD_TESTING=ON && colcon test --packages-select gnss_core --ctest-args -R test_geodetic --event-handlers console_direct+`
Expected: 链接失败或断言失败(forward 未实现)。

- [ ] **Step 4: 写实现 geodetic.cpp**

```cpp
#include "gnss_core/geodetic.hpp"
#include <GeographicLib/LocalCartesian.hpp>

namespace gnss_core {

LlaToEnu::LlaToEnu(double lat0, double lon0, double alt0)
  : impl_(std::make_unique<GeographicLib::LocalCartesian>(lat0, lon0, alt0)) {}

LlaToEnu::~LlaToEnu() = default;

Eigen::Vector3d LlaToEnu::forward(double lat, double lon, double alt) const {
  double e, n, u;
  impl_->Forward(lat, lon, alt, e, n, u);
  return {e, n, u};
}

}  // namespace gnss_core
```

- [ ] **Step 5: 跑测试确认通过**

Run: 同 Step 3
Expected: test_geodetic 2 passed。

- [ ] **Step 6: Commit**

```bash
cd /home/steve/glim_ws/src/glim_ext && git add gnss_core && \
git commit -m "feat(gnss_core): LlaToEnu via GeographicLib::LocalCartesian"
```

---

### Task 4: RtkFixBuffer(时间插值,质量取较差者)

**Files:**
- Modify: `gnss_core/include/gnss_core/rtk_fix_buffer.hpp`
- Modify: `gnss_core/src/rtk_fix_buffer.cpp`
- Test: `gnss_core/test/test_rtk_fix_buffer.cpp`

**Interfaces:**
- Consumes: `RtkFixSample`, `Quality`(types.hpp)
- Produces: `class RtkFixBuffer { void push(const RtkFixSample&); std::optional<RtkFixSample> interpolate(double t) const; void prune(double horizon_s, double now); double latest_stamp() const; };`
  - `interpolate`:t 落在两样本之间→线性插值 lat/lon/alt/sigma/diff_age,**quality 取两端较差者(数值较小者)**;t 越界或缓冲不足→`std::nullopt`。
  - `prune` 的 `now` 是**数据时间**(通常传 `latest_stamp()`),不是壁钟;后台线程无壁钟意义,壳里不得传 `ros::now`。头文件注释写明。

- [ ] **Step 1: 写头文件**

```cpp
#pragma once
#include <deque>
#include <optional>
#include "gnss_core/types.hpp"

namespace gnss_core {

class RtkFixBuffer {
public:
  void push(const RtkFixSample& s);              // 按 stamp 递增维护
  std::optional<RtkFixSample> interpolate(double t) const;
  void prune(double horizon_s, double now);      // 丢弃 stamp < now - horizon_s;now 为数据时间(见 latest_stamp)
  double latest_stamp() const { return buf_.empty() ? 0.0 : buf_.back().stamp; }
  size_t size() const { return buf_.size(); }
private:
  std::deque<RtkFixSample> buf_;
};

}  // namespace gnss_core
```

- [ ] **Step 2: 写失败测试**

```cpp
#include <gtest/gtest.h>
#include "gnss_core/rtk_fix_buffer.hpp"
using namespace gnss_core;

static RtkFixSample mk(double t, Quality q, double lat) {
  RtkFixSample s; s.stamp = t; s.quality = q; s.lat = lat; return s;
}

TEST(RtkFixBuffer, InterpolatesMidpoint) {
  RtkFixBuffer b;
  b.push(mk(100.0, Quality::FIXED, 44.0));
  b.push(mk(102.0, Quality::FIXED, 46.0));
  auto r = b.interpolate(101.0);
  ASSERT_TRUE(r.has_value());
  EXPECT_NEAR(r->lat, 45.0, 1e-9);
}

TEST(RtkFixBuffer, QualityTakesWorseOfEnds) {
  RtkFixBuffer b;
  b.push(mk(100.0, Quality::FIXED, 44.0));
  b.push(mk(102.0, Quality::SINGLE, 46.0));
  auto r = b.interpolate(101.0);
  ASSERT_TRUE(r.has_value());
  EXPECT_EQ(r->quality, Quality::SINGLE);   // 较差者
}

TEST(RtkFixBuffer, OutOfRangeReturnsNullopt) {
  RtkFixBuffer b;
  b.push(mk(100.0, Quality::FIXED, 44.0));
  b.push(mk(102.0, Quality::FIXED, 46.0));
  EXPECT_FALSE(b.interpolate(105.0).has_value());
  EXPECT_FALSE(b.interpolate(99.0).has_value());
}

TEST(RtkFixBuffer, PruneDropsOld) {
  RtkFixBuffer b;
  b.push(mk(100.0, Quality::FIXED, 44.0));
  b.push(mk(160.0, Quality::FIXED, 46.0));
  b.prune(30.0, 161.0);                     // 丢弃 < 131
  EXPECT_EQ(b.size(), 1u);
}
```

在 CMakeLists 注册 `gnss_core_add_test(test_rtk_fix_buffer)`。

- [ ] **Step 3: 跑测试确认失败**

Run: `cd /home/steve/glim_ws && colcon build --packages-select gnss_core --cmake-args -DBUILD_TESTING=ON && colcon test --packages-select gnss_core --ctest-args -R test_rtk_fix_buffer --event-handlers console_direct+`
Expected: FAIL。

- [ ] **Step 4: 写实现 rtk_fix_buffer.cpp**

```cpp
#include "gnss_core/rtk_fix_buffer.hpp"
#include <algorithm>

namespace gnss_core {

void RtkFixBuffer::push(const RtkFixSample& s) {
  // 常规为递增到达;若乱序则插入到正确位置
  if (buf_.empty() || s.stamp >= buf_.back().stamp) { buf_.push_back(s); return; }
  auto it = std::lower_bound(buf_.begin(), buf_.end(), s.stamp,
      [](const RtkFixSample& a, double t){ return a.stamp < t; });
  buf_.insert(it, s);
}

std::optional<RtkFixSample> RtkFixBuffer::interpolate(double t) const {
  if (buf_.size() < 2) return std::nullopt;
  if (t < buf_.front().stamp || t > buf_.back().stamp) return std::nullopt;
  auto right = std::lower_bound(buf_.begin(), buf_.end(), t,
      [](const RtkFixSample& a, double tt){ return a.stamp < tt; });
  if (right == buf_.begin()) return *right;          // t == front
  auto left = right - 1;
  const double tl = left->stamp, tr = right->stamp;
  const double p = (tr > tl) ? (t - tl) / (tr - tl) : 0.0;
  RtkFixSample out;
  out.stamp = t;
  out.lat = (1 - p) * left->lat + p * right->lat;
  out.lon = (1 - p) * left->lon + p * right->lon;
  out.alt = (1 - p) * left->alt + p * right->alt;
  out.sigma_enu = (1 - p) * left->sigma_enu + p * right->sigma_enu;
  out.diff_age = (1 - p) * left->diff_age + p * right->diff_age;
  out.sats_used = std::min(left->sats_used, right->sats_used);
  // quality 取较差者(枚举数值较小者)
  out.quality = static_cast<Quality>(std::min(
      static_cast<uint8_t>(left->quality), static_cast<uint8_t>(right->quality)));
  out.heading = (1 - p) * left->heading + p * right->heading;
  out.heading_valid = left->heading_valid && right->heading_valid;
  return out;
}

void RtkFixBuffer::prune(double horizon_s, double now) {
  const double cutoff = now - horizon_s;
  while (!buf_.empty() && buf_.front().stamp < cutoff) buf_.pop_front();
}

}  // namespace gnss_core
```

- [ ] **Step 5: 跑测试确认通过**

Run: 同 Step 3
Expected: 4 passed。

- [ ] **Step 6: Commit**

```bash
cd /home/steve/glim_ws/src/glim_ext && git add gnss_core && \
git commit -m "feat(gnss_core): RtkFixBuffer with time interpolation"
```

---

### Task 5: RtkNoisePolicy(门限 + 质量缩放 + 鲁棒核)

**Files:**
- Modify: `gnss_core/include/gnss_core/rtk_noise_policy.hpp`
- Modify: `gnss_core/src/rtk_noise_policy.cpp`
- Test: `gnss_core/test/test_rtk_noise_policy.cpp`

**Interfaces:**
- Consumes: `RtkFixSample`, `Quality`
- Produces:
```cpp
struct NoisePolicyConfig {
  int min_quality = 3;                    // FLOAT
  double max_diff_age = 15.0;
  int min_sats = 6;
  std::array<double,5> quality_sigma_scale = {0.0, 50.0, 20.0, 5.0, 1.0};
  Eigen::Vector3d sigma_floor = {1.0, 1.0, 1.0};   // 杆臂未标定期间;标定后 {0.02,0.02,0.05}
  double vertical_scale = 3.0;
  std::string robust_kernel = "huber";    // none|huber|cauchy
  double robust_delta = 1.345;
};
class RtkNoisePolicy {
public:
  explicit RtkNoisePolicy(const NoisePolicyConfig& cfg);   // cfg 非法(min_quality∉[0,4]、floor≤0、未知 kernel)抛 std::invalid_argument
  // 通过门限则返回 noise model,否则 nullptr
  gtsam::SharedNoiseModel evaluate(const RtkFixSample& s) const;
private:
  NoisePolicyConfig cfg_;
};
```

注:`sigma_floor` 默认值按 spec §7.5 v2 为 `{1.0, 1.0, 1.0}`(杆臂未标定期间);测试 `AcceptsFixedAndAppliesFloor` 显式设 `cfg.sigma_floor = {0.02, 0.02, 0.05}` 后再断言。

- [ ] **Step 1: 写头文件**(内容同上 Interfaces,加 `#include <gtsam/linear/NoiseModel.h>` 与 `<array>`)

- [ ] **Step 2: 写失败测试**

```cpp
#include <gtest/gtest.h>
#include "gnss_core/rtk_noise_policy.hpp"
using namespace gnss_core;

static RtkFixSample good() {
  RtkFixSample s;
  s.quality = Quality::FIXED; s.diff_age = 1.0; s.sats_used = 20;
  s.sigma_enu = {0.01, 0.01, 0.02};
  return s;
}

TEST(NoisePolicy, RejectsBelowMinQuality) {
  RtkNoisePolicy p{NoisePolicyConfig{}};
  auto s = good(); s.quality = Quality::SINGLE;
  EXPECT_EQ(p.evaluate(s), nullptr);
}

TEST(NoisePolicy, RejectsStaleDiffAge) {
  RtkNoisePolicy p{NoisePolicyConfig{}};
  auto s = good(); s.diff_age = 30.0;
  EXPECT_EQ(p.evaluate(s), nullptr);
}

TEST(NoisePolicy, RejectsFewSats) {
  RtkNoisePolicy p{NoisePolicyConfig{}};
  auto s = good(); s.sats_used = 4;
  EXPECT_EQ(p.evaluate(s), nullptr);
}

TEST(NoisePolicy, AcceptsFixedAndAppliesFloor) {
  NoisePolicyConfig cfg; cfg.sigma_floor = {0.02, 0.02, 0.05};
  RtkNoisePolicy p{cfg};
  auto s = good(); s.sigma_enu = {0.001, 0.001, 0.001};  // 板卡报 1mm
  auto m = p.evaluate(s);
  ASSERT_NE(m, nullptr);
  // 剥出鲁棒核下的高斯 sigma;floor 抬到 0.02(E/N)、0.05*3(U)
  auto robust = std::dynamic_pointer_cast<gtsam::noiseModel::Robust>(m);
  ASSERT_NE(robust, nullptr);
  auto diag = std::dynamic_pointer_cast<const gtsam::noiseModel::Diagonal>(robust->noise());
  ASSERT_NE(diag, nullptr);
  EXPECT_NEAR(diag->sigmas()(0), 0.02, 1e-9);
  EXPECT_NEAR(diag->sigmas()(2), 0.05 * 3.0, 1e-9);   // vertical_scale
}

TEST(NoisePolicy, RejectsInvalidConfig) {
  NoisePolicyConfig cfg; cfg.min_quality = 7;
  EXPECT_THROW(RtkNoisePolicy{cfg}, std::invalid_argument);
  NoisePolicyConfig cfg2; cfg2.robust_kernel = "tukey";
  EXPECT_THROW(RtkNoisePolicy{cfg2}, std::invalid_argument);
}

TEST(NoisePolicy, FloatScalesSigmaFiveX) {
  NoisePolicyConfig cfg; cfg.min_quality = 3; cfg.sigma_floor = {0.02, 0.02, 0.05};
  RtkNoisePolicy p{cfg};
  auto s = good(); s.quality = Quality::FLOAT; s.sigma_enu = {0.1, 0.1, 0.1};
  auto m = p.evaluate(s);
  ASSERT_NE(m, nullptr);
  auto robust = std::dynamic_pointer_cast<gtsam::noiseModel::Robust>(m);
  auto diag = std::dynamic_pointer_cast<const gtsam::noiseModel::Diagonal>(robust->noise());
  EXPECT_NEAR(diag->sigmas()(0), 0.1 * 5.0, 1e-9);     // FLOAT scale=5
}
```

在 CMakeLists 注册 `gnss_core_add_test(test_rtk_noise_policy)`。

- [ ] **Step 3: 跑测试确认失败**

Run: `cd /home/steve/glim_ws && colcon build --packages-select gnss_core --cmake-args -DBUILD_TESTING=ON && colcon test --packages-select gnss_core --ctest-args -R test_rtk_noise_policy --event-handlers console_direct+`
Expected: FAIL。

- [ ] **Step 4: 写实现 rtk_noise_policy.cpp**

```cpp
#include "gnss_core/rtk_noise_policy.hpp"
#include <gtsam/linear/NoiseModel.h>
#include <algorithm>
#include <stdexcept>

namespace gnss_core {

RtkNoisePolicy::RtkNoisePolicy(const NoisePolicyConfig& cfg) : cfg_(cfg) {
  if (cfg.min_quality < 0 || cfg.min_quality > 4) throw std::invalid_argument("min_quality must be in [0,4]");
  if ((cfg.sigma_floor.array() <= 0.0).any()) throw std::invalid_argument("sigma_floor must be > 0");
  if (cfg.robust_kernel != "none" && cfg.robust_kernel != "huber" && cfg.robust_kernel != "cauchy")
    throw std::invalid_argument("robust_kernel must be none|huber|cauchy");
}

gtsam::SharedNoiseModel RtkNoisePolicy::evaluate(const RtkFixSample& s) const {
  const int q = static_cast<int>(s.quality);
  if (q < cfg_.min_quality) return nullptr;
  if (s.diff_age > cfg_.max_diff_age) return nullptr;
  if (s.sats_used < cfg_.min_sats) return nullptr;

  const double scale = cfg_.quality_sigma_scale.at(q);
  if (scale <= 0.0) return nullptr;         // 防零 σ / 无穷权重(spec §7.5 约束)

  Eigen::Vector3d sigma = s.sigma_enu * scale;
  sigma = sigma.cwiseMax(cfg_.sigma_floor);
  sigma(2) *= cfg_.vertical_scale;

  gtsam::SharedNoiseModel base = gtsam::noiseModel::Diagonal::Sigmas(sigma);
  if (cfg_.robust_kernel == "none") return base;
  gtsam::noiseModel::mEstimator::Base::shared_ptr m;
  if (cfg_.robust_kernel == "cauchy")
    m = gtsam::noiseModel::mEstimator::Cauchy::Create(cfg_.robust_delta);
  else
    m = gtsam::noiseModel::mEstimator::Huber::Create(cfg_.robust_delta);
  return gtsam::noiseModel::Robust::Create(m, base);
}

}  // namespace gnss_core
```

- [ ] **Step 5: 跑测试确认通过**

Run: 同 Step 3
Expected: 6 passed。

- [ ] **Step 6: Commit**

```bash
cd /home/steve/glim_ws/src/glim_ext && git add gnss_core && \
git commit -m "feat(gnss_core): RtkNoisePolicy (gates + quality scaling + robust)"
```

---

### Task 6: FrameAligner(SVD 求 T_world_enu)

**Files:**
- Modify: `gnss_core/include/gnss_core/frame_aligner.hpp`
- Modify: `gnss_core/src/frame_aligner.cpp`
- Test: `gnss_core/test/test_frame_aligner.cpp`

**Interfaces:**
- Consumes: 无(纯几何)
- Produces:
```cpp
class FrameAligner {
public:
  explicit FrameAligner(double min_baseline);
  void add(const Eigen::Vector3d& submap_xyz, const Eigen::Vector3d& enu);
  bool initialized() const;
  // 已初始化后返回 T_world_enu(把 ENU 坐标映射到 world);未初始化返回 identity
  Eigen::Isometry3d T_world_enu() const;
private:
  double min_baseline_;
  bool initialized_ = false;
  Eigen::Isometry3d T_world_enu_ = Eigen::Isometry3d::Identity();
  std::vector<Eigen::Vector3d> est_, enu_;
};
```
- 语义:累积成对点,当 `est_` 首尾间距 > `min_baseline` 时用 2D Umeyama(仅 yaw + 平移,z 不参与旋转)一次性求解 `T_world_enu`,**之后冻结**:`initialized()` 为 true 后 `add` 只累积、不重解(spec §6.1 v2,避免 gauge 反馈回路)。移植自 `gnss_global` 的 SVD 逻辑。基线判据用 `est_` 首尾距离(与 `gnss_global` 一致);共线轨迹下 2D Umeyama 仍可解 yaw,退化只发生在所有点重合。

- [ ] **Step 1: 写头文件**(同上 Interfaces,加 `#include <Eigen/Geometry>` `<vector>`)

- [ ] **Step 2: 写失败测试**

```cpp
#include <gtest/gtest.h>
#include "gnss_core/frame_aligner.hpp"

TEST(FrameAligner, RecoversKnownTransform) {
  // 构造已知:world = Rz(30°) * enu + t
  const double a = 30.0 * M_PI / 180.0;
  Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
  R.block<2,2>(0,0) << std::cos(a), -std::sin(a), std::sin(a), std::cos(a);
  Eigen::Vector3d t(10.0, -5.0, 2.0);
  Eigen::Isometry3d T_world_enu_true = Eigen::Isometry3d::Identity();
  T_world_enu_true.linear() = R; T_world_enu_true.translation() = t;

  gnss_core::FrameAligner al(10.0);
  std::vector<Eigen::Vector3d> enus = {
    {0,0,0},{5,0,0},{10,0,0},{15,3,0},{20,6,1}};   // 首尾 > 10m
  for (const auto& e : enus) {
    Eigen::Vector3d world = T_world_enu_true * e;    // submap 位置 = world 真值
    al.add(world, e);
  }
  ASSERT_TRUE(al.initialized());
  const auto T = al.T_world_enu();
  // 用它把 enu 变到 world,应与真值一致
  for (const auto& e : enus) {
    EXPECT_LT((T * e - T_world_enu_true * e).norm(), 0.1);
  }
}

TEST(FrameAligner, NotInitializedBelowBaseline) {
  gnss_core::FrameAligner al(10.0);
  al.add({0,0,0},{0,0,0});
  al.add({1,0,0},{1,0,0});          // 基线仅 1m
  EXPECT_FALSE(al.initialized());
}

TEST(FrameAligner, FrozenAfterInitialization) {
  gnss_core::FrameAligner al(10.0);
  al.add({0,0,0},{0,0,0});
  al.add({20,0,0},{20,0,0});        // 恒等变换,基线 20m → 初始化
  ASSERT_TRUE(al.initialized());
  const auto T0 = al.T_world_enu();
  al.add({40,0,0},{0,40,0});        // 与恒等矛盾的点对
  al.add({60,0,0},{0,60,0});
  EXPECT_TRUE(al.T_world_enu().isApprox(T0));   // 冻结:不受后续点对影响
}
```

在 CMakeLists 注册 `gnss_core_add_test(test_frame_aligner)`。

- [ ] **Step 3: 跑测试确认失败**

Run: `cd /home/steve/glim_ws && colcon build --packages-select gnss_core --cmake-args -DBUILD_TESTING=ON && colcon test --packages-select gnss_core --ctest-args -R test_frame_aligner --event-handlers console_direct+`
Expected: FAIL。

- [ ] **Step 4: 写实现 frame_aligner.cpp**

```cpp
#include "gnss_core/frame_aligner.hpp"

namespace gnss_core {

FrameAligner::FrameAligner(double min_baseline) : min_baseline_(min_baseline) {}

void FrameAligner::add(const Eigen::Vector3d& submap_xyz, const Eigen::Vector3d& enu) {
  est_.push_back(submap_xyz);
  enu_.push_back(enu);
  if (initialized_ || est_.size() < 2) return;
  if ((est_.back() - est_.front()).norm() < min_baseline_) return;

  // 2D Umeyama(仅 yaw + 平移):在 XY 平面上对齐 enu → est
  Eigen::Vector3d mean_est = Eigen::Vector3d::Zero(), mean_enu = Eigen::Vector3d::Zero();
  for (size_t i = 0; i < est_.size(); ++i) { mean_est += est_[i]; mean_enu += enu_[i]; }
  mean_est /= est_.size(); mean_enu /= enu_.size();

  Eigen::Matrix2d cov = Eigen::Matrix2d::Zero();
  for (size_t i = 0; i < est_.size(); ++i)
    cov += (est_[i].head<2>() - mean_est.head<2>()) * (enu_[i].head<2>() - mean_enu.head<2>()).transpose();

  Eigen::JacobiSVD<Eigen::Matrix2d> svd(cov, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix2d R2 = svd.matrixU() * svd.matrixV().transpose();
  if (R2.determinant() < 0) { Eigen::Matrix2d V = svd.matrixV(); V.col(1) *= -1; R2 = svd.matrixU() * V.transpose(); }

  Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
  R.block<2,2>(0,0) = R2;
  T_world_enu_ = Eigen::Isometry3d::Identity();
  T_world_enu_.linear() = R;
  T_world_enu_.translation() = mean_est - R * mean_enu;
  initialized_ = true;
}

bool FrameAligner::initialized() const { return initialized_; }
Eigen::Isometry3d FrameAligner::T_world_enu() const { return T_world_enu_; }

}  // namespace gnss_core
```

- [ ] **Step 5: 跑测试确认通过**

Run: 同 Step 3
Expected: 3 passed。

- [ ] **Step 6: Commit**

```bash
cd /home/steve/glim_ws/src/glim_ext && git add gnss_core && \
git commit -m "feat(gnss_core): FrameAligner (2D Umeyama T_world_enu, frozen after bootstrap)"
```

---

### Task 7: AntennaPriorFactor(带杆臂,Jacobian 数值校验)

**Files:**
- Modify: `gnss_core/include/gnss_core/antenna_prior_factor.hpp`(header-only)
- Test: `gnss_core/test/test_antenna_prior_factor.cpp`

**Interfaces:**
- Consumes: 无
- Produces:
```cpp
class AntennaPriorFactor : public gtsam::NoiseModelFactorN<gtsam::Pose3> {
public:
  // body_point:天线在被约束 node 局部系中的位置。
  //   rtk_global 在 submap 原点帧时刻取样,T_origin_frame = I,故 body_point = lever_imu;
  //   rtk_odometry(轮 1.5)直接传 lever_imu。见 spec §7.3 v2。
  AntennaPriorFactor(gtsam::Key key, const Eigen::Vector3d& measured_world,
                     const Eigen::Vector3d& body_point, const gtsam::SharedNoiseModel& model);
  gtsam::Vector evaluateError(const gtsam::Pose3& X,
                              gtsam::OptionalMatrixType H) const override;
};
```
- 误差:`h(X) = X.transformFrom(body_point)`;残差 `h(X) - measured_world`。body_point=0 时退化为纯平移先验。

- [ ] **Step 1: 写头文件(header-only 实现)**

```cpp
#pragma once
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/geometry/Pose3.h>
#include <Eigen/Core>

namespace gnss_core {

class AntennaPriorFactor : public gtsam::NoiseModelFactorN<gtsam::Pose3> {
  gtsam::Point3 measured_;
  gtsam::Point3 body_point_;
public:
  using Base = gtsam::NoiseModelFactorN<gtsam::Pose3>;
  AntennaPriorFactor(gtsam::Key key, const Eigen::Vector3d& measured_world,
                     const Eigen::Vector3d& body_point, const gtsam::SharedNoiseModel& model)
    : Base(model, key), measured_(measured_world), body_point_(body_point) {}

  gtsam::Vector evaluateError(const gtsam::Pose3& X,
                              gtsam::OptionalMatrixType H) const override {
    gtsam::Matrix36 Hpred;   // d(predicted)/d(pose)
    // 天线在 world 系的位置 = X 变换 body_point
    const gtsam::Point3 predicted = X.transformFrom(body_point_, H ? &Hpred : nullptr);
    if (H) *H = Hpred;
    return predicted - measured_;
  }
};

}  // namespace gnss_core
```

- [ ] **Step 2: 写 Jacobian 数值校验测试**

```cpp
#include <gtest/gtest.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/linear/NoiseModel.h>
#include "gnss_core/antenna_prior_factor.hpp"
using namespace gtsam;

TEST(AntennaPriorFactor, JacobianMatchesNumerical) {
  Key k = 0;
  Eigen::Vector3d measured(1.0, 2.0, 3.0), lever(0.5, -0.2, 0.1);
  auto model = noiseModel::Isotropic::Sigma(3, 0.05);
  gnss_core::AntennaPriorFactor f(k, measured, lever, model);

  Pose3 X(Rot3::RzRyRx(0.3, -0.1, 0.2), Point3(1.0, 1.0, 1.0));
  Matrix H;
  f.evaluateError(X, &H);
  Matrix Hnum = numericalDerivative11<Vector, Pose3>(
      [&](const Pose3& p){ return f.evaluateError(p, OptionalMatrixType(nullptr)); }, X);
  EXPECT_TRUE(assert_equal(Hnum, H, 1e-6));
}

TEST(AntennaPriorFactor, ZeroLeverEqualsTranslationResidual) {
  Key k = 0;
  Eigen::Vector3d measured(1.0, 2.0, 3.0), lever(0,0,0);
  auto model = noiseModel::Isotropic::Sigma(3, 0.05);
  gnss_core::AntennaPriorFactor f(k, measured, lever, model);
  Pose3 X(Rot3(), Point3(1.5, 2.5, 3.5));
  Vector e = f.evaluateError(X, OptionalMatrixType(nullptr));
  EXPECT_TRUE(assert_equal(Vector(Point3(0.5, 0.5, 0.5)), e, 1e-9));
}
```

在 CMakeLists 注册 `gnss_core_add_test(test_antenna_prior_factor)`。

> 注:GTSAM 4.2 与 4.3 都提供 `NoiseModelFactorN` 与 `OptionalMatrixType`(4.2 起引入),两端均可编译;若实现时编译报接口不符,以本机 `gtsam/nonlinear/NonlinearFactor.h` 的实际签名为准调整(这是本 Task 唯一的 API 风险点)。

- [ ] **Step 3: 跑测试确认失败**(工厂头未纳入编译前)

Run: `cd /home/steve/glim_ws && colcon build --packages-select gnss_core --cmake-args -DBUILD_TESTING=ON && colcon test --packages-select gnss_core --ctest-args -R test_antenna_prior_factor --event-handlers console_direct+`
Expected: 编译失败(测试引用未注册)或断言前失败。

- [ ] **Step 4: 确保头文件编入**(header-only,无需改 .cpp;确认 CMakeLists 测试目标链接 gnss_core 与 gtsam)

- [ ] **Step 5: 跑测试确认通过**

Run: 同 Step 3
Expected: 2 passed(Jacobian 数值一致 + 零杆臂退化)。

- [ ] **Step 6: Commit**

```bash
cd /home/steve/glim_ws/src/glim_ext && git add gnss_core && \
git commit -m "feat(gnss_core): AntennaPriorFactor with numerical-verified Jacobian"
```

---

### Task 8: 驱动增发 RtkFix

**Files:**(正本 `finder_ros/drivers/gnss_CGI610/`,`driver_ws/src/gnss_CGI610` 指向它;**包名是 `gnss_chcnav`**,节点名 `gnss_cgi610`,可执行 `gnss_chcnav_can`)
- Modify: `driver_ws/src/gnss_CGI610/src/gnss_can_node.cpp`
- Modify: `driver_ws/src/gnss_CGI610/CMakeLists.txt`(加 `gnss_msgs` 依赖)
- Modify: `driver_ws/src/gnss_CGI610/package.xml`(加 `<depend>gnss_msgs</depend>`)
- Create: `driver_ws/src/gnss_CGI610/include/cgi610/rtk_fix_mapping.hpp`
- Test: `driver_ws/src/gnss_CGI610/test/test_rtk_fix_mapping.cpp`(新建)

**已核对的现状(2026-09-07 读源码)**:
- `cgi610::Cycle` 已有 `gps_week`(uint16)与 `gps_tow`(double,周内秒),`cgi610::GpsToUnix(week, tow, leap=18)` 已存在——`gnss_time` 直接调它,不必再写换算。
- 节点已有参数 `timestamp_source`(`arrival` | `gps`)决定 `header.stamp`;`RtkFix.gnss_time` **无论该参数取什么都填 `GpsToUnix()`**,这样下游总能拿到板卡时间。
- 现有三个 publisher 已用 `rclcpp::QoS(KeepLast(50)).reliable()`,`~/rtk_fix` 沿用同一 `qos` 变量即可。
- `pos_sigma_enu_m` 由 CAN 0x326 解出,解码器注释为 "position std-dev ENU";Step 0 只需对照手册 `doc/CGI-610用户手册(修订202008).pdf` 的 0x326 字段顺序确认一次。
- 话题全名为 **`/gnss_cgi610/rtk_fix`**(节点名 `gnss_cgi610`),Task 9 的 `rtk_fix_topic` 默认值据此修正。

**Interfaces:**
- Consumes: `gnss_msgs/msg/RtkFix`(Task 1)、`cgi610::Cycle`(现有)、`cgi610::SatStatus`(现有)
- Produces: topic `~/rtk_fix`(**reliable** QoS);一个可单测的纯映射函数 `gnss_msgs::msg::RtkFix MapCycleToRtkFix(const cgi610::Cycle& c)`

- [ ] **Step 0: 核对三件事(写代码前,结论写进本 Task 的 commit message)**

1. `Cycle::pos_sigma_enu_m` 的分量顺序:对照手册 0x326 字段顺序,确认是 E/N/U 还是 N/E/U。若为 N/E/U,映射时交换前两项,`RtkFix.sigma_enu` 恒为 E/N/U。
2. (已核对)板卡时间已解析:`gps_week`/`gps_tow`,`GpsToUnix()` 已有。
3. (已核对)QoS 已是 reliable,沿用。

- [ ] **Step 1: 写失败测试(纯映射函数)**

新建 `test/test_rtk_fix_mapping.cpp`:
```cpp
#include <gtest/gtest.h>
#include "cgi610/rtk_fix_mapping.hpp"   // 下一步抽出的纯函数头
#include "cgi610/cgi610_decoder.hpp"

TEST(RtkFixMapping, FixedWithHeading) {
  cgi610::Cycle c;
  c.satellite_status = static_cast<uint8_t>(cgi610::SatStatus::RTK_FIXED);
  c.lat_deg = 44.5; c.lon_deg = 90.28; c.alt_m = 617.0;
  c.pos_sigma_enu_m[0] = 0.01; c.pos_sigma_enu_m[1] = 0.012; c.pos_sigma_enu_m[2] = 0.03;
  c.gps_age_s = 0.8; c.sats_used = 20; c.heading_deg = 123.4; c.att_sigma_deg[0] = 0.2;
  auto m = cgi610::MapCycleToRtkFix(c);
  EXPECT_EQ(m.quality, gnss_msgs::msg::RtkFix::QUALITY_FIXED);
  EXPECT_TRUE(m.heading_valid);
  EXPECT_NEAR(m.sigma_enu[0], 0.01, 1e-9);
  EXPECT_NEAR(m.diff_age, 0.8, 1e-6);
}

TEST(RtkFixMapping, FloatNoHeadingClearsHeadingValid) {
  cgi610::Cycle c;
  c.satellite_status = static_cast<uint8_t>(cgi610::SatStatus::RTK_FLOAT_NO_HEADING);
  auto m = cgi610::MapCycleToRtkFix(c);
  EXPECT_EQ(m.quality, gnss_msgs::msg::RtkFix::QUALITY_FLOAT);
  EXPECT_FALSE(m.heading_valid);
}

TEST(RtkFixMapping, SingleMapsToSingle) {
  cgi610::Cycle c;
  c.satellite_status = static_cast<uint8_t>(cgi610::SatStatus::SINGLE);
  EXPECT_EQ(cgi610::MapCycleToRtkFix(c).quality, gnss_msgs::msg::RtkFix::QUALITY_SINGLE);
}

TEST(RtkFixMapping, CombinedDrHasNoValidHeading) {
  cgi610::Cycle c;
  c.satellite_status = static_cast<uint8_t>(cgi610::SatStatus::COMBINED_DR);
  auto m = cgi610::MapCycleToRtkFix(c);
  EXPECT_EQ(m.quality, gnss_msgs::msg::RtkFix::QUALITY_SINGLE);
  EXPECT_FALSE(m.heading_valid);          // DR 航向来自惯导递推,不是双天线观测
}

TEST(RtkFixMapping, GnssTimeFromWeekAndTow) {
  cgi610::Cycle c;
  c.gps_week = 2433; c.gps_tow = 100.0;
  auto m = cgi610::MapCycleToRtkFix(c);
  EXPECT_NEAR(m.gnss_time, cgi610::GpsToUnix(2433, 100.0), 1e-6);
  EXPECT_NEAR(m.gnss_time, 315964800.0 + 2433.0 * 604800.0 + 100.0 - 18.0, 1e-6);
}
```

- [ ] **Step 2: 跑测试确认失败**

Run: `cd /home/steve/driver_ws && colcon build --packages-select gnss_chcnav --cmake-args -DBUILD_TESTING=ON`
Expected: 编译失败(`cgi610/rtk_fix_mapping.hpp` / `MapCycleToRtkFix` 不存在)。

- [ ] **Step 3: 抽出纯映射函数头 rtk_fix_mapping.hpp**

`include/cgi610/rtk_fix_mapping.hpp`:
```cpp
#pragma once
#include <gnss_msgs/msg/rtk_fix.hpp>
#include "cgi610/cgi610_decoder.hpp"

namespace cgi610 {

inline gnss_msgs::msg::RtkFix MapCycleToRtkFix(const Cycle& c) {
  gnss_msgs::msg::RtkFix m;
  using S = SatStatus; using Q = gnss_msgs::msg::RtkFix;
  switch (static_cast<S>(c.satellite_status)) {
    case S::RTK_FIXED: case S::RTK_FIXED_NO_HEADING: m.quality = Q::QUALITY_FIXED; break;
    case S::RTK_FLOAT: case S::RTK_FLOAT_NO_HEADING: m.quality = Q::QUALITY_FLOAT; break;
    case S::PSRDIFF:   case S::PSRDIFF_NO_HEADING:   m.quality = Q::QUALITY_DGPS;  break;
    case S::SINGLE: case S::SINGLE_NO_HEADING: case S::COMBINED_DR: m.quality = Q::QUALITY_SINGLE; break;
    default: m.quality = Q::QUALITY_NONE; break;
  }
  m.raw_status = c.satellite_status;
  const S s = static_cast<S>(c.satellite_status);
  m.heading_valid = (s == S::RTK_FIXED || s == S::RTK_FLOAT ||
                     s == S::PSRDIFF || s == S::SINGLE);     // COMBINED_DR 与 *_NO_HEADING 均为 false
  m.latitude = c.lat_deg; m.longitude = c.lon_deg; m.altitude = c.alt_m;
  // 顺序按 Step 0 结论;此处假定 Cycle 已是 E/N/U,若为 N/E/U 则交换 [0] 与 [1]
  m.sigma_enu = {c.pos_sigma_enu_m[0], c.pos_sigma_enu_m[1], c.pos_sigma_enu_m[2]};
  m.gnss_time = (c.gps_week > 0) ? GpsToUnix(c.gps_week, c.gps_tow) : 0.0;
  m.diff_age = static_cast<float>(c.gps_age_s);
  m.sats_used = c.sats_used; m.sats_main = c.sats_main; m.sats_aux = c.sats_aux;
  m.heading = static_cast<float>(c.heading_deg);
  m.heading_sigma = static_cast<float>(c.att_sigma_deg[0]);
  return m;
}

}  // namespace cgi610
```

在 CMakeLists 注册测试:
```cmake
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(gnss_msgs REQUIRED)
  ament_add_gtest(test_rtk_fix_mapping test/test_rtk_fix_mapping.cpp)
  target_include_directories(test_rtk_fix_mapping PRIVATE include)
  ament_target_dependencies(test_rtk_fix_mapping gnss_msgs)
endif()
```
(现有 `test_cgi610_decoder` 保留。)

- [ ] **Step 4: 跑测试确认通过**

Run: `cd /home/steve/driver_ws && colcon build --packages-select gnss_chcnav --cmake-args -DBUILD_TESTING=ON && colcon test --packages-select gnss_chcnav --ctest-args -R test_rtk_fix_mapping --event-handlers console_direct+`
Expected: 5 passed。

- [ ] **Step 5: 在节点里接线增发**

`gnss_can_node.cpp`:加 `#include "cgi610/rtk_fix_mapping.hpp"`;在构造函数建 publisher `rtk_fix_pub_ = create_publisher<gnss_msgs::msg::RtkFix>("~/rtk_fix", qos);`(与现有三个 publisher 同一个 reliable `qos`);在 `Flush()` 里 `fix_pub_->publish(fix);` 之后、`if (!odom_pub_) return;` 之前追加:
```cpp
auto rtk = cgi610::MapCycleToRtkFix(c);
rtk.header.stamp = stamp;           // 随 timestamp_source 参数;gnss_time 已在映射里填 GpsToUnix
rtk.header.frame_id = frame_id_;
rtk_fix_pub_->publish(rtk);
```
在 `package.xml` 加 `<depend>gnss_msgs</depend>`;`CMakeLists.txt` 加 `find_package(gnss_msgs REQUIRED)`,`ament_target_dependencies(gnss_chcnav_can ...)` 加 `gnss_msgs`。

- [ ] **Step 6: 构建 + 冒烟(有 bag 时回放,无 bag 则仅确认节点起来且 topic 存在)**

Run: `cd /home/steve/driver_ws && colcon build --packages-select gnss_chcnav && source install/setup.bash`
Expected: 构建成功。若有含 CAN 的 bag:`ros2 topic echo /gnss_cgi610/rtk_fix` 能看到 quality 字段随解状态变化;`timestamp_source=arrival` 时 `header.stamp − gnss_time` 稳定在几十 ms 以内(这个差值就是接收延迟,记录下来供 Task 14 对照)。

- [ ] **Step 7: Commit**

```bash
cd /home/steve/Documents/GitHub/ztpilot/finder_ros && git add drivers/gnss_CGI610 && \
git commit -m "feat(gnss_chcnav): publish gnss_msgs/RtkFix alongside NavSatFix"
```

---

### Task 9: rtk_global 模块壳(编入 glim_ext 包)

**Files:**
- Create: `glim_ws/src/glim_ext/modules/mapping/rtk_global/include/glim_ext/rtk_global_module.hpp`
- Create: `glim_ws/src/glim_ext/modules/mapping/rtk_global/src/glim_ext/rtk_global_module_ros2.cpp`
- Create: `glim_ws/src/glim_ext/modules/mapping/rtk_global/CMakeLists.txt`
- Modify: `glim_ws/src/glim_ext/CMakeLists.txt`(`option(ENABLE_RTK_GLOBAL)` + `add_subdirectory`)
- Modify: `glim_ws/src/glim_ext/package.xml`(`<depend>gnss_core</depend>` `<depend>gnss_msgs</depend>`)
- Create: `glim_ws/src/glim_ext/config/config_rtk_global.json`
- Modify: `glim_ws/src/glim_underground/config/casbot/config_ros.json`(`extension_modules` 加 `librtk_global.so`,注释掉 `libgnss_global.so`)

**Interfaces:**
- Consumes: `gnss_core::{RtkFixBuffer, RtkNoisePolicy, FrameAligner, AntennaPriorFactor, LlaToEnu, RtkFixSample, Quality, NoisePolicyConfig, StampSource, effective_stamp}`;`gnss_msgs/msg/RtkFix`;GLIM `ExtensionModuleROS2`、`GlobalMappingCallbacks::{on_insert_submap, on_smoother_update}`、`SubMap`、`gtsam::symbol_shorthand::X`。
- Produces: `install/glim_ext/lib/librtk_global.so` + `extern "C" create_extension_module()`。

**Why not a separate package:** `glim_ext` 是一个 ament 包,所有模块由顶层 `CMakeLists.txt` 通过 `option`+`add_subdirectory` 编入并统一 install(见 `gnss_global`、`flat_earther` 的做法)。`colcon build --packages-select rtk_global` 不存在这样的包。

- [ ] **Step 1: 写配置 config_rtk_global.json**

```jsonc
{
  "rtk_global": {
    "rtk_fix_topic": "/gnss_cgi610/rtk_fix",   // 节点名 gnss_cgi610(包 gnss_chcnav)
    "stamp_source": "gnss_time",         // header | gnss_time(gnss_time==0 时回退 header)
    "time_offset": 0.0,                  // s;由 Task 14 工具估计后填入
    "min_quality": 3,
    "max_diff_age": 15.0,
    "min_sats": 6,
    "quality_sigma_scale": [0.0, 50.0, 20.0, 5.0, 1.0],
    // 杆臂未标定期间 sigma_floor 取杆臂量级,否则 Huber 会把转弯段固定解当外点(spec §7.3 v2);
    // T_imu_gnss 标定填入后改回 [0.02, 0.02, 0.05]
    "sigma_floor": [1.0, 1.0, 1.0],
    "vertical_scale": 3.0,
    "robust_kernel": "huber",
    "robust_delta": 1.345,
    "T_imu_gnss": [0.0, 0.0, 0.0],       // 天线杆臂 (m, IMU 系);由 Task 14 工具估计或量取
    "min_baseline": 10.0,
    "enu_origin": [],
    "fix_buffer_horizon": 60.0
  }
}
```

- [ ] **Step 2: 写模块头 rtk_global_module.hpp**

参照 `modules/mapping/gnss_global/include/glim_ext/gnss_global_module.hpp` 的结构(`#define GLIM_ROS2`、`ExtensionModuleROS2` 基类、`create_subscriptions()`、后台线程 + `ConcurrentVector`),但:

```cpp
// 在 on_insert_submap 回调(global mapping 线程)内拷出的 POD;后台线程只碰这个,不持有 SubMap 指针。
// 理由:callbacks.hpp 明确 submap->T_world_origin 只在 global mapping 线程更新,跨线程读不安全。
struct SubmapAnchor {
  int id;
  double stamp;                 // origin_frame()->stamp
  Eigen::Vector3d t_world_origin;
};

class RtkGlobal : public glim::ExtensionModuleROS2 {
public:
  RtkGlobal();                          // 读 config,建 policy/buffer/aligner,注册回调,起后台线程
  ~RtkGlobal();
  std::vector<glim::GenericTopicSubscription::Ptr> create_subscriptions() override;  // 订阅 rtk_fix_topic
private:
  void rtk_fix_callback(const gnss_msgs::msg::RtkFix::ConstSharedPtr msg);  // → RtkFixSample(stamp=effective_stamp) → input_fix_queue_
  void on_insert_submap(const glim::SubMap::ConstPtr& submap);             // 拷 SubmapAnchor → input_anchor_queue_
  void on_smoother_update(gtsam_points::ISAM2Ext&, gtsam::NonlinearFactorGraph&, gtsam::Values&);  // drain output_factors_
  void backend_task();                  // 关联→ENU→对齐→造因子→输出队列

  gnss_core::RtkFixBuffer buffer_;      // 仅后台线程访问
  std::unique_ptr<gnss_core::RtkNoisePolicy> policy_;
  std::unique_ptr<gnss_core::FrameAligner> aligner_;
  std::unique_ptr<gnss_core::LlaToEnu> lla_to_enu_;   // 首个过门限 fix 时惰性建(或 config enu_origin)
  Eigen::Vector3d lever_imu_;
  gnss_core::StampSource stamp_source_;
  double time_offset_;
  double fix_buffer_horizon_;
  std::string rtk_fix_topic_;
  glim::ConcurrentVector<gnss_core::RtkFixSample> input_fix_queue_;
  glim::ConcurrentVector<SubmapAnchor> input_anchor_queue_;
  glim::ConcurrentVector<gtsam::NonlinearFactor::shared_ptr> output_factors_;
  std::atomic_bool kill_switch_;
  std::thread thread_;
  std::shared_ptr<spdlog::logger> logger_;
  // 计数器,供日志与 Step 6 验证
  std::atomic<size_t> n_fix_received_{0}, n_factors_added_{0}, n_rejected_{0};
};
```

`backend_task()` 逻辑(spec §7.1):每轮 `input_fix_queue_.get_all_and_clear()` → `buffer_.push`;`buffer_.prune(horizon, buffer_.latest_stamp())`;对每个待处理 `SubmapAnchor a`(保留在本地 deque,直到 `buffer_` 覆盖其 `a.stamp` 或 `a.stamp` 已早于缓冲最老样本则丢弃并计 `n_rejected_`):`buffer_.interpolate(a.stamp)`;拿到样本后,若 `lla_to_enu_` 未建则以该样本 lla 建原点(或 config `enu_origin`);`enu = lla_to_enu_->forward(...)`;`aligner_->add(a.t_world_origin, enu)`;若 `aligner_->initialized()`:`p_world = aligner_->T_world_enu() * enu`,`model = policy_->evaluate(sample)`,若非空则 `output_factors_.push_back(std::make_shared<gnss_core::AntennaPriorFactor>(X(a.id), p_world, lever_imu_ /* body_point:原点帧时刻 T_origin_frame=I */, model))` 并 `++n_factors_added_`,否则 `++n_rejected_`。首次 `initialized()` 时 `logger_->info("T_world_enu={}", ...)`。`on_smoother_update` 里 drain `output_factors_` 到 `new_factors`。

`rtk_fix_callback`:`s.header_stamp = to_sec(msg->header.stamp); s.gnss_time = msg->gnss_time; s.stamp = gnss_core::effective_stamp(s.header_stamp, s.gnss_time, stamp_source_, time_offset_);` 其余字段直拷;`++n_fix_received_`。

析构:`kill_switch_ = true; thread_.join();` 并打印三个计数器。

- [ ] **Step 3: 写实现 rtk_global_module_ros2.cpp**

含各方法实现与:
```cpp
extern "C" glim::ExtensionModule* create_extension_module() { return new glim::RtkGlobal(); }
```

config 读取用 `glim::GlobalConfigExt::get_config_path("config_rtk_global")` + `glim::Config`,键集见 Step 1;`stamp_source` 字符串非 `header`/`gnss_time` 时抛异常;`NoisePolicyConfig` 装配后交给 `RtkNoisePolicy` 构造(非法配置由它抛出)。

- [ ] **Step 4: 模块 CMakeLists.txt + 接入 glim_ext**

`modules/mapping/rtk_global/CMakeLists.txt`(仿 `gnss_global`,只做 ROS2):
```cmake
cmake_minimum_required(VERSION 3.22)
project(rtk_global)
set(CMAKE_CXX_STANDARD 17)

find_package(glim REQUIRED)
find_package(GTSAM REQUIRED)
find_package(spdlog REQUIRED)
find_package(gnss_core REQUIRED)
find_package(gnss_msgs REQUIRED)
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(rtk_global SHARED src/glim_ext/rtk_global_module_ros2.cpp)
target_include_directories(rtk_global PRIVATE include ${GTSAM_INCLUDE_DIRS} ${glim_INCLUDE_DIRS})
target_link_libraries(rtk_global glim_ext gnss_core::gnss_core ${GTSAM_LIBRARIES} ${glim_LIBRARIES} spdlog::spdlog)
ament_target_dependencies(rtk_global gnss_msgs)
```

`glim_ext/CMakeLists.txt`,在 `option(ENABLE_GNSS ...)` 后加:
```cmake
option(ENABLE_RTK_GLOBAL "Enable quality-aware RTK global constraint module (ROS2 only)" ON)
```
在 `if(ENABLE_GNSS) ... endif()` 后加:
```cmake
if(ENABLE_RTK_GLOBAL)
  add_subdirectory(modules/mapping/rtk_global)
  list(APPEND glim_ext_LIBRARIES rtk_global)
endif()
```

`glim_ext/package.xml` 加 `<depend>gnss_core</depend>` `<depend>gnss_msgs</depend>`。

- [ ] **Step 5: 构建**

Run:
```bash
source /opt/ros/jazzy/setup.bash && source /home/steve/driver_ws/install/setup.bash && \
cd /home/steve/glim_ws && colcon build --packages-select gnss_core glim_ext --event-handlers console_direct+
```
Expected: 生成 `install/glim_ext/lib/librtk_global.so`。(`gnss_core` 与 `glim_ext` 同在 `glim_ws`,colcon 按 `package.xml` 依赖先建 `gnss_core`。)

- [ ] **Step 6: 用 bag 跑 glim_rosbag 验证加载与订阅**

脱离 GLIM 进程 `dlopen`/ctypes 调 `create_extension_module()` 不可行(构造函数依赖 GLIM 全局 config 并起线程),用真实流程验证:

1. `config/casbot/config_ros.json` 的 `extension_modules` 加 `"librtk_global.so"`(确认 `libgnss_global.so` 未同时启用)。
2. 准备一段含 LiDAR + IMU + `RtkFix` 的 bag:有实车 bag 用实车 bag;没有则先完成 Task 13,用合成 `RtkFix` bag 与既有纯 LiDAR bag 一起回放(`ros2 bag play` 两个 bag,或用 Task 13 的合并脚本)。
3. Run: `cd /home/steve/glim_ws && source install/setup.bash && ros2 run glim_ros glim_rosbag <bag> 2>&1 | tee /tmp/rtk_global.log`
4. Expected(日志断言,写成 `tools/check_rtk_global_log.sh`):
   - 出现 `[rtk_global] initializing`;
   - 出现 `T_world_enu=`(基线 > `min_baseline` 后);
   - 退出时计数 `n_fix_received > 0` 且 `n_factors_added > 0`;
   - 无 `msg type mismatch` 与 `failed to deserialize` 警告。
5. 若 `n_fix_received == 0`:检查 topic 名与 QoS(Task 8 Step 0.3)。

- [ ] **Step 7: Commit**

```bash
cd /home/steve/glim_ws && git add src/glim_ext/modules/mapping/rtk_global src/glim_ext/CMakeLists.txt src/glim_ext/package.xml src/glim_ext/config/config_rtk_global.json src/glim_underground/config/casbot/config_ros.json && \
git commit -m "feat(glim_ext): rtk_global quality-aware GNSS global constraint module"
```

---
### Task 10: pos_io(.pos 读取)

**Files:**
- Modify: `gnss_core/include/gnss_core/pos_io.hpp`
- Modify: `gnss_core/src/pos_io.cpp`
- Test: `gnss_core/test/test_pos_io.cpp`

**Interfaces:**
- Consumes: `Quality`
- Produces:
```cpp
struct PosRecord {
  double stamp;                 // unix seconds
  double lat, lon, height;
  int q;                        // RTKLIB Q: 1=fix 2=float 4=dgps 5=single
  int ns;
  Eigen::Vector3d sdne;         // sdn, sde, sdu
  double age, ratio;
};
// 时间系统:RTKLIB .pos 默认 GPST(比 UTC 快 leap_seconds),头部 "% (time=GPST)" / "(time=UTC)" 标明。
// read_pos 统一输出 UTC unix 秒:GPST 减 leap_seconds;UTC 原样;无头部标注时按 default_time_system。
enum class PosTimeSystem { GPST, UTC };
struct PosReadOptions { int leap_seconds = 18; PosTimeSystem default_time_system = PosTimeSystem::GPST; };
std::vector<PosRecord> read_pos(const std::string& path, const PosReadOptions& opt = {});   // 跳过 % 注释行
Quality q_to_quality(int q);    // 1→FIXED 2→FLOAT 4→DGPS 5→SINGLE 其它→NONE
```

- [ ] **Step 1: 写头文件**(同上 Interfaces,加 includes)

- [ ] **Step 2: 写失败测试**

```cpp
#include <gtest/gtest.h>
#include <fstream>
#include "gnss_core/pos_io.hpp"
using namespace gnss_core;

TEST(PosIo, ParsesRtklibPos) {
  const char* path = "/tmp/test_gnss_core.pos";
  std::ofstream f(path);
  f << "% program : RTKLIB\n";
  f << "%  GPST latitude longitude height Q ns sdn sde sdu ...\n";
  f << "2026/09/03 10:23:45.000 44.50123456 90.28765432 617.123 1 38 0.012 0.011 0.030 0.0 0.0 0.0 0.8 20.5\n";
  f.close();
  auto recs = read_pos(path);
  ASSERT_EQ(recs.size(), 1u);
  EXPECT_EQ(recs[0].q, 1);
  EXPECT_NEAR(recs[0].lat, 44.50123456, 1e-8);
  EXPECT_EQ(recs[0].ns, 38);
  EXPECT_NEAR(recs[0].sdne(0), 0.012, 1e-9);
  EXPECT_NEAR(recs[0].ratio, 20.5, 1e-6);
}

TEST(PosIo, QToQuality) {
  EXPECT_EQ(q_to_quality(1), Quality::FIXED);
  EXPECT_EQ(q_to_quality(2), Quality::FLOAT);
  EXPECT_EQ(q_to_quality(5), Quality::SINGLE);
}

TEST(PosIo, GpstHeaderSubtractsLeapSeconds) {
  const char* path = "/tmp/test_gnss_core_gpst.pos";
  std::ofstream f(path);
  f << "% (lat/lon/height=WGS84/ellipsoidal,Q=1:fix,2:float,4:dgps,5:single, time=GPST)\n";
  f << "2026/09/03 10:23:45.000 44.5 90.28 617.0 1 38 0.01 0.01 0.03 0 0 0 0.8 20.5\n";
  f.close();
  auto gpst = read_pos(path);
  std::ofstream g(path);
  g << "% (time=UTC)\n";
  g << "2026/09/03 10:23:45.000 44.5 90.28 617.0 1 38 0.01 0.01 0.03 0 0 0 0.8 20.5\n";
  g.close();
  auto utc = read_pos(path);
  ASSERT_EQ(gpst.size(), 1u); ASSERT_EQ(utc.size(), 1u);
  EXPECT_NEAR(utc[0].stamp - gpst[0].stamp, 18.0, 1e-6);   // 同一行,GPST 解释晚 18 s → unix 秒小 18
}
```

在 CMakeLists 注册 `gnss_core_add_test(test_pos_io)`。

- [ ] **Step 3: 跑测试确认失败**

Run: `cd /home/steve/glim_ws && colcon build --packages-select gnss_core --cmake-args -DBUILD_TESTING=ON && colcon test --packages-select gnss_core --ctest-args -R test_pos_io --event-handlers console_direct+`
Expected: FAIL。

- [ ] **Step 4: 写实现 pos_io.cpp**

按 RTKLIB `.pos` 格式:注释行以 `%` 开头跳过,但要扫描其中的 `time=GPST` / `time=UTC` 决定时间系统(未出现则用 `opt.default_time_system`);数据行首两列是 `YYYY/MM/DD` 与 `HH:MM:SS.sss`,按 UTC 日历合成 unix 秒(用 `timegm`,不要用 `mktime`,后者受本机时区影响),若时间系统为 GPST 再减 `opt.leap_seconds`;其余列按 `lat lon height Q ns sdn sde sdu sdne sdeu sdun age ratio`。

- [ ] **Step 5: 跑测试确认通过**

Run: 同 Step 3
Expected: 3 passed。

- [ ] **Step 6: Commit**

```bash
cd /home/steve/glim_ws/src/glim_ext && git add gnss_core && \
git commit -m "feat(gnss_core): .pos reader (GPST/UTC aware)"
```

---

### Task 11: trajectory_compare(分档误差统计)

**Files:**
- Modify: `gnss_core/include/gnss_core/trajectory_compare.hpp`
- Modify: `gnss_core/src/trajectory_compare.cpp`
- Test: `gnss_core/test/test_trajectory_compare.cpp`

**Interfaces:**
- Consumes: `PosRecord`(Task 10)、`Quality`、`LlaToEnu`(Task 3)
- Produces:
```cpp
struct QualityStats {
  int n = 0;
  double rmse_h = 0, rmse_v = 0;
  double sigma_ratio_h_mean = 0;     // mean(err_h / σ_h) —— 仅参考,受外点主导
  double sigma_ratio_h_median = 0;   // median(err_h / σ_h) —— 建议系数取此
  double sigma_ratio_h_rms = 0;      // RMS(err_h) / RMS(σ_h)
};
// 以 ref 为基准,按最近时间戳(容差 tol_s)配对 test,按 test 的 quality 分档统计
// rmse_*:test 相对 ref 的水平/垂直 RMSE;sigma_ratio_h_*:实际水平误差与板卡报水平 σ 的比(三种统计量,见 spec §9.2 v2)
std::map<Quality, QualityStats> compare_by_quality(
    const std::vector<PosRecord>& ref, const std::vector<PosRecord>& test, double tol_s = 0.1);
```

- [ ] **Step 1: 写头文件**(同上 Interfaces)

- [ ] **Step 2: 写失败测试**

```cpp
#include <gtest/gtest.h>
#include "gnss_core/trajectory_compare.hpp"
using namespace gnss_core;

static PosRecord rec(double t, double lat, double lon, double h, int q, double sd) {
  PosRecord r{}; r.stamp=t; r.lat=lat; r.lon=lon; r.height=h; r.q=q;
  r.sdne = {sd, sd, sd}; return r;
}

TEST(TrajCompare, PerfectMatchZeroRmse) {
  std::vector<PosRecord> ref = {rec(100, 44.5, 90.28, 617, 1, 0.01)};
  std::vector<PosRecord> test = {rec(100, 44.5, 90.28, 617, 1, 0.01)};
  auto s = compare_by_quality(ref, test);
  ASSERT_EQ(s.count(Quality::FIXED), 1u);
  EXPECT_EQ(s[Quality::FIXED].n, 1);
  EXPECT_NEAR(s[Quality::FIXED].rmse_h, 0.0, 1e-6);
}

TEST(TrajCompare, KnownHorizontalOffset) {
  // test 相对 ref 北偏 ~1m
  const double dlat = 1.0 / 111320.0;
  std::vector<PosRecord> ref = {rec(100, 44.5, 90.28, 617, 3, 0.1)};
  std::vector<PosRecord> test = {rec(100, 44.5 + dlat, 90.28, 617, 3, 0.1)};
  auto s = compare_by_quality(ref, test);
  ASSERT_EQ(s.count(Quality::FLOAT), 1u);
  EXPECT_NEAR(s[Quality::FLOAT].rmse_h, 1.0, 0.05);
  EXPECT_NEAR(s[Quality::FLOAT].sigma_ratio_h_median, 7.07, 0.5);   // 1 m 实际 / σ_h,σ_h = hypot(sdn, sde) = 0.141
}

TEST(TrajCompare, MedianRatioRobustToOneWrongFix) {
  // 9 个正常 FIXED 历元(误差 1 cm, σ 1 cm → 比值 1)+ 1 个错误固定(误差 1 m, σ 3 mm → 比值 ~333)
  const double dlat_1cm = 0.01 / 111320.0, dlat_1m = 1.0 / 111320.0;
  std::vector<PosRecord> ref, test;
  for (int i = 0; i < 9; ++i) { ref.push_back(rec(100+i, 44.5, 90.28, 617, 1, 0.01)); test.push_back(rec(100+i, 44.5+dlat_1cm, 90.28, 617, 1, 0.01)); }
  ref.push_back(rec(200, 44.5, 90.28, 617, 1, 0.003)); test.push_back(rec(200, 44.5+dlat_1m, 90.28, 617, 1, 0.003));
  auto s = compare_by_quality(ref, test);
  EXPECT_LT(s[Quality::FIXED].sigma_ratio_h_median, 2.0);   // 中位数不受单个外点影响
  EXPECT_GT(s[Quality::FIXED].sigma_ratio_h_mean, 20.0);    // 均值被外点拽走——这就是不用均值的理由
}

TEST(TrajCompare, UnpairedBeyondToleranceSkipped) {
  std::vector<PosRecord> ref = {rec(100, 44.5, 90.28, 617, 1, 0.01)};
  std::vector<PosRecord> test = {rec(102, 44.5, 90.28, 617, 1, 0.01)};  // 2s 差
  auto s = compare_by_quality(ref, test, 0.1);
  EXPECT_TRUE(s.empty());
}
```

在 CMakeLists 注册 `gnss_core_add_test(test_trajectory_compare)`。

- [ ] **Step 3: 跑测试确认失败**

Run: `cd /home/steve/glim_ws && colcon build --packages-select gnss_core --cmake-args -DBUILD_TESTING=ON && colcon test --packages-select gnss_core --ctest-args -R test_trajectory_compare --event-handlers console_direct+`
Expected: FAIL。

- [ ] **Step 4: 写实现 trajectory_compare.cpp**

对每个 test 记录,在 ref 中二分找最近 stamp;若 |dt|>tol_s 跳过。以配对首个 ref 的 lla 为 ENU 原点建 `LlaToEnu`,把 ref/test 都转 ENU 求差;水平误差 `err_h = hypot(dE,dN)`、垂直 `|dU|`;`σ_h = test.sdne.head<2>().norm()`。按 `q_to_quality(test.q)` 收集 `(err_h, σ_h)` 序列,末尾算 RMSE、`mean(err_h/σ_h)`、`median(err_h/σ_h)`(`std::nth_element`)、`RMS(err_h)/RMS(σ_h)`。σ_h 为 0 的历元跳过比值统计(计数仍算)。

- [ ] **Step 5: 跑测试确认通过**

Run: 同 Step 3
Expected: 4 passed。

- [ ] **Step 6: Commit**

```bash
cd /home/steve/glim_ws/src/glim_ext && git add gnss_core && \
git commit -m "feat(gnss_core): trajectory comparison by quality tier (median/RMS sigma ratios)"
```

---

### Task 12: 系数标定工具 + 导出脚本

**Files:**
- Create: `gnss_core/tools/calibrate_sigma_scale.cpp`
- Modify: `gnss_core/CMakeLists.txt`(加 `add_executable`)
- Create: `gnss_core/tools/export_bag_to_pos.py`(rtk-monitor 既有录包 → .pos,供轨迹 1–3)
- Create: `gnss_core/tools/README.md`

**Interfaces:**
- Consumes: `read_pos`、`compare_by_quality`(Task 10/11)
- Produces: 可执行 `calibrate_sigma_scale <ref.pos> <test.pos> [tol_s]`,打印各质量档的 n / rmse_h / rmse_v / 三种 σ 比值,并给出建议的 `quality_sigma_scale`(= 各档 `sigma_ratio_h_median`,归一到 FIXED=1)。

- [ ] **Step 1: 写 calibrate_sigma_scale.cpp**

```cpp
#include <iostream>
#include "gnss_core/pos_io.hpp"
#include "gnss_core/trajectory_compare.hpp"
using namespace gnss_core;

int main(int argc, char** argv) {
  if (argc < 3) { std::cerr << "usage: calibrate_sigma_scale <ref.pos> <test.pos> [tol_s]\n"; return 1; }
  const double tol = (argc > 3) ? std::stod(argv[3]) : 0.1;
  auto ref = read_pos(argv[1]);
  auto test = read_pos(argv[2]);
  auto stats = compare_by_quality(ref, test, tol);
  const char* names[] = {"NONE","SINGLE","DGPS","FLOAT","FIXED"};
  double fixed_ratio = 0.0;
  for (auto& [q, s] : stats) if (q == Quality::FIXED) fixed_ratio = s.sigma_ratio_h_median;
  std::cout << "quality  n   rmse_h(m)  rmse_v(m)  ratio_median  ratio_rms  ratio_mean\n";
  for (auto& [q, s] : stats)
    std::cout << names[static_cast<int>(q)] << "  " << s.n << "  " << s.rmse_h << "  " << s.rmse_v << "  "
              << s.sigma_ratio_h_median << "  " << s.sigma_ratio_h_rms << "  " << s.sigma_ratio_h_mean << "\n";
  std::cout << "\nsuggested quality_sigma_scale (median ratio, normalized to FIXED=1):\n";
  for (auto& [q, s] : stats)
    std::cout << "  " << names[static_cast<int>(q)] << " = "
              << (fixed_ratio > 0 ? s.sigma_ratio_h_median / fixed_ratio : s.sigma_ratio_h_median) << "\n";
  return 0;
}
```
CMakeLists 加:
```cmake
add_executable(calibrate_sigma_scale tools/calibrate_sigma_scale.cpp)
target_link_libraries(calibrate_sigma_scale gnss_core)
install(TARGETS calibrate_sigma_scale DESTINATION lib/${PROJECT_NAME})
```

- [ ] **Step 2: 写 export_bag_to_pos.py**

读 rtk-monitor 既有录包(SQLite `data/*.db` 的 epochs 表,或既有 .pos)导出为标准 `.pos`(每源一个文件),列顺序符合 Task 10 的解析。含 `--src {can,gpchc,rtkrcv}` 与 `--out` 参数。

- [ ] **Step 3: 构建工具**

Run: `cd /home/steve/glim_ws && colcon build --packages-select gnss_core`
Expected: 生成 `calibrate_sigma_scale` 可执行。

- [ ] **Step 4: 端到端标定(有数据时)**

Run:
```bash
# 用 rnx2rtkp 后处理输出作 ref,rtk-monitor 导出的 rtkrcv.pos 作 test(示例)
./install/gnss_core/lib/gnss_core/calibrate_sigma_scale ref.pos rtkrcv.pos
```
Expected: 打印分档统计与建议的 `quality_sigma_scale`。**若无成对数据,则用两条合成 .pos(已知偏移)验证工具本身正确,并在 README 记录待现场数据回来后重跑。**

- [ ] **Step 5: 写 README.md**

记录:工具用途、`.pos` 数据从哪来(rnx2rtkp / export_bag_to_pos.py)、如何把标定出的系数填回 `config_rtk_global.json` 的 `quality_sigma_scale`(spec §9.2 闭环)、RTKPLOT 叠加多条 `.pos` 做可视化对比的命令。

- [ ] **Step 6: Commit**

```bash
cd /home/steve/glim_ws/src/glim_ext && git add gnss_core/tools gnss_core/CMakeLists.txt && \
git commit -m "feat(gnss_core): sigma-scale calibration tool + bag-to-pos export"
```

---

### Task 13: 合成 RTK 注入测试(spec §12.3)

**Files:**
- Create: `gnss_core/include/gnss_core/synth.hpp` + `gnss_core/src/synth.cpp` — 轨迹 → 合成 RTK 样本(纯函数,可单测)
- Create: `gnss_core/tools/synth_rtk_fix.cpp` — 命令行:读 GLIM 轨迹 → 写 `.pos`
- Create: `gnss_core/tools/pos_to_rtkfix_bag.py` — `.pos` → `RtkFix` rosbag2(Orin 上跑,依赖 `rosbag2_py`、`gnss_msgs`)
- Create: `gnss_core/tools/run_injection_suite.sh` — 四种注入 × 开/关鲁棒核 跑 `glim_rosbag`,汇总轨迹 RMSE
- Test: `gnss_core/test/test_synth.cpp`
- Modify: `gnss_core/CMakeLists.txt`(库源加 `src/synth.cpp`,`gnss_core_add_test(test_synth)`,`add_executable(synth_rtk_fix)`)

**Interfaces:**
- Consumes: `LlaToEnu`(Task 3;需补 `Eigen::Vector3d reverse(enu)` → lat/lon/alt,封装 `LocalCartesian::Reverse`)、`RtkFixSample`、`pos_io` 写出(本 Task 补一个最小 `write_pos(path, records, time_system)`;spec §10 轮 2 的写出器可在此基础上扩展)
- Produces:
```cpp
struct TrajPose { double stamp; Eigen::Isometry3d T_world_imu; };
std::vector<TrajPose> read_glim_traj(const std::string& path);   // glim_rosbag dump 的 traj_imu.txt(TUM 格式:t x y z qx qy qz qw)

struct SynthConfig {
  Eigen::Vector3d lever_imu = Eigen::Vector3d::Zero();   // 天线在 IMU 系
  Eigen::Isometry3d T_enu_world = Eigen::Isometry3d::Identity();   // 把 world 放到 ENU 里的任意位姿(测试 FrameAligner)
  double lat0 = 44.5, lon0 = 90.28, alt0 = 617.0;        // ENU 原点
  double rate_hz = 10.0;
  Eigen::Vector3d sigma_fixed = {0.01, 0.01, 0.03};      // 真噪声(FIXED)
  Eigen::Vector3d sigma_float = {0.3, 0.3, 0.6};
  unsigned seed = 42;
};
// 注入脚本:按时间段/比例篡改
struct Injection {
  double wrong_fix_ratio = 0.0;     // 该比例历元:位置偏 1 m、σ 仍报 FIXED 级、quality=FIXED
  double stale_from = -1, stale_to = -1;   // 时间段内 diff_age 从 0 线性增长到 60 s
  double float_from = -1, float_to = -1;   // 时间段内 quality=FLOAT、σ 放大到 sigma_float
};
std::vector<RtkFixSample> synthesize(const std::vector<TrajPose>& traj, const SynthConfig& cfg, const Injection& inj);
```
- 生成逻辑:对每个采样时刻 t(按 `rate_hz` 在轨迹时间范围内等间隔,位姿线性/球面插值),`p_enu = T_enu_world · (T_world_imu(t) · lever_imu)`,加高斯噪声(σ 按当前质量档),`reverse` 回 lat/lon/alt;`sigma_enu` 填**报告值**(FIXED 报 `sigma_fixed`,FLOAT 报 `sigma_float`,错误固定仍报 `sigma_fixed`);`sats_used=20`、`diff_age=1`,再按 `Injection` 篡改;`gnss_time = t`、`header_stamp = t + 0.05`(模拟 50 ms 接收延迟,让 `stamp_source` 的差别可见)。

- [ ] **Step 1: 补 `LlaToEnu::reverse` + 测试**(加到 `test_geodetic.cpp`:forward→reverse 往返误差 < 1e-6 m)

- [ ] **Step 2: 写失败测试 test_synth.cpp**

```cpp
#include <gtest/gtest.h>
#include "gnss_core/synth.hpp"
#include "gnss_core/geodetic.hpp"
using namespace gnss_core;

static std::vector<TrajPose> straight_then_turn() {
  // 0–20 s 沿 +x 直行 100 m,20–30 s 原地转 90°,30–50 s 沿 +y 直行 100 m
  std::vector<TrajPose> tr;
  for (double t = 0; t <= 50; t += 0.1) {
    TrajPose p; p.stamp = t; p.T_world_imu = Eigen::Isometry3d::Identity();
    double yaw = (t < 20) ? 0 : (t < 30 ? (t - 20) / 10 * M_PI_2 : M_PI_2);
    p.T_world_imu.linear() = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    p.T_world_imu.translation() = (t < 20) ? Eigen::Vector3d(5 * t, 0, 0) : (t < 30 ? Eigen::Vector3d(100, 0, 0) : Eigen::Vector3d(100, 5 * (t - 30), 0));
    tr.push_back(p);
  }
  return tr;
}

TEST(Synth, ZeroNoiseZeroLeverReproducesTrajectory) {
  SynthConfig cfg; cfg.sigma_fixed.setZero(); cfg.sigma_float.setZero();
  auto s = synthesize(straight_then_turn(), cfg, {});
  ASSERT_GT(s.size(), 400u);
  LlaToEnu conv(cfg.lat0, cfg.lon0, cfg.alt0);
  const auto& last = s.back();
  const auto enu = conv.forward(last.lat, last.lon, last.alt);
  EXPECT_NEAR(enu.x(), 100.0, 1e-3); EXPECT_NEAR(enu.y(), 100.0, 1e-3);
  EXPECT_EQ(last.quality, Quality::FIXED);
  EXPECT_NEAR(last.header_stamp - last.gnss_time, 0.05, 1e-9);
}

TEST(Synth, LeverArmShowsUpOnlyAfterTurn) {
  SynthConfig cfg; cfg.sigma_fixed.setZero(); cfg.lever_imu = {0, 1.0, 0};   // 天线在 IMU 左侧 1 m
  auto s = synthesize(straight_then_turn(), cfg, {});
  LlaToEnu conv(cfg.lat0, cfg.lon0, cfg.alt0);
  auto at = [&](double t) { for (auto& x : s) if (std::abs(x.stamp - t) < 1e-6) return conv.forward(x.lat, x.lon, x.alt); throw std::runtime_error("no sample"); };
  EXPECT_NEAR(at(10.0).y(), 1.0, 1e-3);      // 直行段:天线在 +y 1 m
  EXPECT_NEAR(at(40.0).x(), 99.0, 1e-3);     // 转 90° 后:天线在 −x 1 m
}

TEST(Synth, WrongFixInjectionKeepsFixedLabelAndSmallSigma) {
  SynthConfig cfg; cfg.sigma_fixed.setZero();
  Injection inj; inj.wrong_fix_ratio = 0.05;
  auto s = synthesize(straight_then_turn(), cfg, inj);
  int n_bad = 0;
  LlaToEnu conv(cfg.lat0, cfg.lon0, cfg.alt0);
  for (auto& x : s) {
    // 真值可由 stamp 反推;偏 1 m 的历元 quality 仍为 FIXED、sigma 仍为 sigma_fixed
    if (x.quality == Quality::FIXED && x.sigma_enu.isZero()) { /* 全部 */ }
  }
  // 统计与真值偏差 > 0.5 m 的历元比例 ≈ 5%
  // (实现:synthesize 同时返回或可选输出 ground-truth ENU;此处用 SynthResult{samples, truth_enu})
  SUCCEED();  // 占位:实现 SynthResult 后改为断言 |n_bad/N − 0.05| < 0.02
}

TEST(Synth, StaleSegmentRaisesDiffAge) {
  SynthConfig cfg; Injection inj; inj.stale_from = 10; inj.stale_to = 20;
  auto s = synthesize(straight_then_turn(), cfg, inj);
  for (auto& x : s) if (x.stamp > 19.5 && x.stamp < 20.0) EXPECT_GT(x.diff_age, 50.0);
  for (auto& x : s) if (x.stamp < 9.5) EXPECT_LT(x.diff_age, 2.0);
}

TEST(Synth, FloatSegmentLabelsFloat) {
  SynthConfig cfg; Injection inj; inj.float_from = 30; inj.float_to = 40;
  auto s = synthesize(straight_then_turn(), cfg, inj);
  for (auto& x : s) if (x.stamp > 30 && x.stamp < 40) { EXPECT_EQ(x.quality, Quality::FLOAT); EXPECT_GT(x.sigma_enu.x(), 0.1); }
}
```

实现时把 `synthesize` 的返回改为 `struct SynthResult { std::vector<RtkFixSample> samples; std::vector<Eigen::Vector3d> truth_enu; }`,并把第三个测试的占位换成真实断言。

- [ ] **Step 3: 跑测试确认失败 → 写实现 synth.cpp → 跑通过**

Run: 纯 CMake 或 ament 路径,`-R test_synth`。Expected: 5 passed(含改写后的 WrongFix 断言)。

- [ ] **Step 4: 写 synth_rtk_fix.cpp**

```
usage: synth_rtk_fix <traj_imu.txt> <out.pos> [--lever x y z] [--wrong-fix 0.05] [--stale from to] [--float from to] [--seed N] [--truth out_truth.pos]
```
输出标准 `.pos`(`time=UTC`,用 Task 10/13 的 `write_pos`),`--truth` 另写无噪声真值 `.pos`。

- [ ] **Step 5: 写 pos_to_rtkfix_bag.py(Orin)**

读 `.pos` → 每行一条 `gnss_msgs/RtkFix`(`header.stamp = t + 0.05`,`gnss_time = t`,`quality = q_to_quality(Q)`,`sigma_enu = [sde, sdn, sdu]`,`diff_age = age`,`sats_used = ns`),用 `rosbag2_py.SequentialWriter` 写到 `--out` 目录,topic 名 `--topic /gnss_cgi610/rtk_fix`。同时支持 `--merge <lidar_bag>`:把 LiDAR/IMU bag 的消息与合成 `RtkFix` 合并写成一个 bag(按时间排序),避免双 bag 回放的时钟对齐问题。

- [ ] **Step 6: 写 run_injection_suite.sh(Orin)**

对同一段既有纯 LiDAR bag:
1. 无 GNSS 跑一次 `glim_rosbag`,dump 轨迹作真值 `traj_ref.txt`。
2. 用 `synth_rtk_fix` 从 `traj_ref.txt` 生成四个注入版本 + 一个干净版本的 `.pos`,各转成合并 bag。
3. 每个 bag 跑 `glim_rosbag`(`librtk_global.so` 启用),分别 `robust_kernel=huber` 与 `none`,dump 轨迹。
4. 用 `evo_ape`(或 Task 11 的 `compare_by_quality` 改成读 TUM 的变体)算每次轨迹相对 `traj_ref.txt` 的 RMSE,输出表格。

预期(spec §12.3):
| 注入 | huber | none |
|---|---|---|
| 5% 错误固定 | RMSE ≈ 干净版本 | 明显恶化 |
| diff_age 增长段 | 该段无因子(日志 `n_rejected` 增加),RMSE ≈ 干净 | 同左(门限与核无关) |
| 全浮动段 | 因子仍加,权重降 5× | 同左 |
| 非零杆臂 + 转弯(`--lever 0 1 0`,config `T_imu_gnss` 分别填 0 与 `[0,1,0]`) | 填对杆臂后转弯段 RMSE 明显低于填 0 | — |

- [ ] **Step 7: Commit**

```bash
cd /home/steve/glim_ws/src/glim_ext && git add gnss_core && \
git commit -m "feat(gnss_core): synthetic RTK generator with fault injection + injection suite"
```

---

### Task 14: 杆臂 + 时间偏移估计工具(spec §9.3)

**Files:**
- Modify: `gnss_core/include/gnss_core/lever_arm_estimator.hpp` + `src/lever_arm_estimator.cpp`
- Create: `gnss_core/tools/estimate_lever_arm.cpp`
- Test: `gnss_core/test/test_lever_arm_estimator.cpp`
- Modify: `gnss_core/CMakeLists.txt`(`gnss_core_add_test(test_lever_arm_estimator)`,`add_executable(estimate_lever_arm)`)

**Interfaces:**
- Consumes: `TrajPose`/`read_glim_traj`(Task 13)、`PosRecord`/`read_pos`(Task 10)、`LlaToEnu`(Task 3)、`synthesize`(Task 13,用于测试)
- Produces:
```cpp
struct LeverArmEstimate {
  Eigen::Vector3d lever_imu;        // 天线在 IMU 系
  Eigen::Isometry3d T_world_enu;    // yaw + 平移(z 平移也解)
  double time_offset;               // s;加到 RTK 样本时间上使其与轨迹对齐
  double rms_residual;              // m
  int n_pairs;
  bool lever_observable;            // 轨迹 yaw 变化范围 < 30° 时 false(水平杆臂不可观)
};
struct LeverArmOptions { double dt_min = -0.5, dt_max = 0.5, dt_step = 0.01; double pair_tol = 0.05; Quality min_quality = Quality::FIXED; };
LeverArmEstimate estimate_lever_arm(const std::vector<TrajPose>& traj, const std::vector<RtkFixSample>& fixes,
                                    const LlaToEnu& conv, const LeverArmOptions& opt = {});
```
- 算法:对候选 `Δt` 网格:把 `fixes` 的 `stamp + Δt` 与 `traj` 按 `pair_tol` 配对(轨迹侧线性/球面插值到该时刻),得到 `(R_i, t_i, enu_i)`;求解 `min Σ ||R_i·lever + t_i − (R_yaw·enu_i + p)||²`:先用 Umeyama(`FrameAligner` 同款 2D)取 `R_yaw, p` 初值(lever=0),再对 `(lever, p)` 做线性最小二乘(给定 `R_yaw`),交替 3–5 轮或做一步 Gauss-Newton 同时更新 yaw。取残差最小的 `Δt`。可观性:`traj` 的 yaw 极差 < 30° 时置 `lever_observable=false`(仍返回结果,但工具打印警告)。

- [ ] **Step 1: 写失败测试**

```cpp
#include <gtest/gtest.h>
#include "gnss_core/lever_arm_estimator.hpp"
#include "gnss_core/synth.hpp"
#include "gnss_core/geodetic.hpp"
using namespace gnss_core;

// 复用 Task 13 的 straight_then_turn()(移到 test/synth_fixtures.hpp)

TEST(LeverArm, RecoversLeverAndOffsetFromSyntheticData) {
  SynthConfig cfg; cfg.lever_imu = {0.3, 1.2, -0.5}; cfg.sigma_fixed = {0.01, 0.01, 0.03};
  Eigen::Isometry3d T_enu_world = Eigen::Isometry3d::Identity();
  T_enu_world.linear() = Eigen::AngleAxisd(0.7, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  T_enu_world.translation() = {50, -20, 3};
  cfg.T_enu_world = T_enu_world;
  auto traj = straight_then_turn();
  auto res = synthesize(traj, cfg, {});
  // 模拟 RTK 时间戳整体早 80 ms(即需要 +0.08 的 time_offset)
  for (auto& s : res.samples) s.stamp -= 0.08;
  LlaToEnu conv(cfg.lat0, cfg.lon0, cfg.alt0);
  auto est = estimate_lever_arm(traj, res.samples, conv);
  EXPECT_TRUE(est.lever_observable);
  EXPECT_NEAR(est.time_offset, 0.08, 0.011);
  EXPECT_LT((est.lever_imu - cfg.lever_imu).norm(), 0.05);
  EXPECT_LT(est.rms_residual, 0.05);
}

TEST(LeverArm, StraightLineIsNotObservable) {
  std::vector<TrajPose> tr;
  for (double t = 0; t <= 20; t += 0.1) { TrajPose p; p.stamp = t; p.T_world_imu = Eigen::Isometry3d::Identity(); p.T_world_imu.translation() = {5 * t, 0, 0}; tr.push_back(p); }
  SynthConfig cfg; cfg.lever_imu = {0, 1, 0}; cfg.sigma_fixed.setZero();
  auto res = synthesize(tr, cfg, {});
  LlaToEnu conv(cfg.lat0, cfg.lon0, cfg.alt0);
  auto est = estimate_lever_arm(tr, res.samples, conv);
  EXPECT_FALSE(est.lever_observable);
}
```

- [ ] **Step 2: 跑测试确认失败 → 写实现 → 跑通过**

Expected: 2 passed。`Δt` 步长 10 ms,因此 `time_offset` 精度断言为 ±11 ms。

- [ ] **Step 3: 写 estimate_lever_arm.cpp**

```
usage: estimate_lever_arm <traj_imu.txt> <rtk.pos> [--origin lat lon alt] [--dt-range -0.5 0.5] [--min-quality 4]
```
打印 `lever_imu`、`time_offset`、`rms_residual`、`n_pairs`、可观性警告,并打印可直接粘进 `config_rtk_global.json` 的两行:`"T_imu_gnss": [..]`、`"time_offset": ..`。

- [ ] **Step 4: 端到端(有实车数据时)**

用实车 LiDAR+IMU bag 无 GNSS 跑 `glim_rosbag` 得 `traj_imu.txt`,同段 `RtkFix` 用 `export_bag_to_pos.py`(Task 12)转 `.pos`,跑工具;把结果填入 config,`sigma_floor` 降到 `[0.05,0.05,0.1]`,重跑 Task 9 Step 6 的验证。无实车数据时用 Task 13 合成数据验证工具本身,并在 README 记录待办。

- [ ] **Step 5: Commit**

```bash
cd /home/steve/glim_ws/src/glim_ext && git add gnss_core && \
git commit -m "feat(gnss_core): lever-arm and time-offset estimator from mapping trajectory"
```

---

## 依赖顺序

```
Task 1 (gnss_msgs) ─────────────────────────┐
Task 2 (core 骨架) → Task 3–7 (core 单元,可并行) ┤→ Task 9 (rtk_global 壳;需 Orin 构建)
Task 1 → Task 8 (驱动;需 driver_ws + Orin)   ┘
Task 2 → Task 10 → Task 11 → Task 12 (对比与标定工具链)
Task 3, 10 → Task 13 (合成注入;C++ 部分可离线,bag/suite 需 Orin)
Task 13, 10 → Task 14 (杆臂/时间偏移估计)
Task 9 + Task 13 → Task 9 Step 6 与 Task 13 Step 6 的 bag 验证(需 Orin)
```

**可脱离 Orin(纯 CMake,云端/CI)完成的部分**:Task 2–7 的 v2 增量、10–12、13(Step 1–4)、14。云端改完后回写到 `glim_ws/src/glim_ext/gnss_core/`,再在 Orin 上 `colcon build --packages-select gnss_core` 复核。
**必须在 Orin 上做的部分**:Task 1、8、9、13(Step 5–6)、14(Step 4)。

## 验收(轮 1 完成标志)

- `gnss_msgs`(finder_ros/drivers,经 driver_ws)、`gnss_core` 与 `glim_ext`(含 `rtk_global`)在 `glim_ws` 于 aarch64 上 `colcon build` 通过;`gnss_core` 亦可在无 ROS 环境以纯 CMake 构建
- `gnss_core` 全部 gtest 通过(types/geodetic/buffer/policy/aligner/factor/pos/compare/synth/lever_arm),含 AntennaPriorFactor 的 Jacobian 数值校验、FrameAligner 冻结、`.pos` GPST/UTC、σ 比值中位数稳健性
- `librtk_global.so` 在 `glim_rosbag` 中加载,日志出现 `T_world_enu=`,`n_fix_received > 0`、`n_factors_added > 0`(Task 9 Step 6)
- `gnss_CGI610` 增发 `~/rtk_fix`(reliable,含 `gnss_time`),映射函数测试通过;`header.stamp − gnss_time` 已记录
- 合成注入四用例结果表产出(Task 13 Step 6),至少"5% 错误固定:huber ≈ 干净 / none 恶化"与"杆臂填对后转弯段改善"两条成立
- `calibrate_sigma_scale` 能对成对 `.pos` 产出分档统计与建议系数;`estimate_lever_arm` 在合成数据上恢复杆臂(< 5 cm)与时间偏移(< 11 ms)
- `config_rtk_global.json` 的 `sigma_floor` 注释与 spec §7.3 v2 一致;杆臂标定前为 1.0 m
