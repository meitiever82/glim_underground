# GNSS 模块化设计（GLIM 生态）

**状态**：已评审通过，作为后续各实施计划的约束权威
**修订**：v2（2026-09-07）——依据 [2026-09-07-round1-review.md](2026-09-07-round1-review.md) 修订：轮 1 只做 `rtk_global`（`rtk_odometry` 移至轮 1.5）；`T_world_enu` 改为 bootstrap 后冻结；`RtkFix` 增加 `gnss_time`；配置增加 `stamp_source` / `time_offset`；杆臂未标定期间的 `sigma_floor` 约束；submap 跨线程访问规则；补合成注入测试与杆臂/时间偏移估计工具。修订处以 **[v2]** 标注。
**前置文档**：[2026-08-31-rtk-monitor-design.md](2026-08-31-rtk-monitor-design.md)（rtk-monitor 单体应用设计，本文为其架构续篇）

---

## 1. 背景与目标

### 1.1 出发点

rtk-monitor 作为独立 Python 应用已交付并验证（四路接入、rtkrcv 独立解算、九条诊断规则、Web 界面、回放与报告）。但它是一个 `App` God class 手工接线的单体：635 行的 `main.py` 占全项目 23%，52 个实例成员，27 处测试伸进私有方法才能测。工程完整度达不到 GLIM 的水准。

更重要的是定位问题：**GNSS 相关算法最终应成为 GLIM 生态的模块**，按算法命名、配置启用，提供 odometry 或 mapping 约束——而不是一个体外的 Python 进程。

### 1.2 上游的缺口（本设计要填的洞）

`glim_ext/modules/mapping/gnss_global` 是 GLIM 现有的 GNSS 约束模块，其源码注释明确写着：

> `@note This implementation is very naive and ignores the IMU-GNSS transformation and GNSS observation covariance. If you use a precise GNSS (e.g., RTK), consider asking for a closed-source extension module with better GNSS handling.`

即：**用 RTK 的话，上游只提供闭源模块**。踏歌矿区正是 RTK 场景。具体 naive 在：

| 现有 `gnss_global` | 后果 |
|---|---|
| 收 `PoseWithCovarianceStamped` 却丢弃协方差，用固定 `prior_inf_scale` | 所有历元同权 |
| 不区分固定/浮动/单点解 | 2 m 单点解与 1 cm 固定解同权 |
| 无异常门限 | 差分中断期间照样加约束 |
| 忽略 IMU-GNSS 杆臂 | 转弯时系统性偏移 |
| 不用双天线航向 | 丢失强约束（本轮不做，见 §11） |
| 无鲁棒核 | 单个错误固定解可拽歪整张图 |

### 1.3 目标

1. 用 C++ 实现质量感知的 GNSS 约束模块，纳入 `glim_ext`，按算法命名、配置启用
2. 算法核心（`gnss_core`）不依赖 ROS、不依赖 GLIM，可单测、可被多个壳复用
3. rtk-monitor 现有功能**全部有明确归宿**，不因迁移而丢失（见 §3 归宿总表）
4. 无 LiDAR 时也能完成轨迹对比与定权系数标定（见 §9）

### 1.4 非目标（本轮）

- 双天线航向约束（后续在 `rtk_odometry` 上扩展姿态先验；本轮只做位置约束）
- 界面迁移（已确定机制，见 §11，但不在本轮实施）
- 改造 `gnss_global`（保持原样，作为 A/B 对比基线）

---

## 2. 总体架构

```
┌──── 生产者层（ROS2 节点，彼此平级，都发 gnss_msgs/RtkFix）─────────┐
│     gnss_CGI610      610 板卡组合解 / 卫导解（已有，增发 rtk_fix） │
│     rtkrcv_node      RTKLIB 独立解算（新）                      │
│     <未来算法>        PPP / 其他                                │
└───────────────────────────┬───────────────────────────────────┘
                            │ gnss_msgs/RtkFix
  ┌─────────────────────────┼──────────────────────┐
  │                         │                      │
  ↓                         ↓                      ↓
┌──── gnss_core（纯 C++ 库，无 ROS / 无 GLIM 依赖，可单测）──────────┐
│     类型      RtkFixSample / EpochRecord / Verdict             │
│     诊断      九条规则链 + 事件状态机 + 轨迹对比统计                 │
│     约束      RtkNoisePolicy / RtkFixBuffer / FrameAligner     │
│              AntennaPriorFactor（统一 body_point 形式）          │
│     解析      RTCM 分帧与 1005 解析 / rtkrcv $SAT 与 llh 解析    │
│     落盘      .pos 写出器                                       │
└──┬────────────┬───────────────┬────────────────┬──────────────┘
   │            │               │                │
   ↓            ↓               ↓                ↓
odometry     mapping         glim_ext         独立 ROS2 节点
module       module          module           gnss_diag_node
rtk_odometry rtk_global      gnss_diag        （无 GLIM 时诊断/对比）
挂 X(frame)  挂 X(submap)    （诊断 + UI）
```

**RTK 约束挂两处（odometry + global 都要，缺一不可）：**
- `rtk_odometry`（`modules/odometry/`）挂 `OdometryEstimationCallbacks::on_smoother_update`，
  往 fixed-lag 图里的**每帧节点 `X(frame.id) = T_world_imu`** 加因子——RTK 观测本就对应某一时刻的
  帧状态，这是最细粒度、最正确的落点。
- `rtk_global`（`modules/mapping/`）挂 `GlobalMappingCallbacks::{on_insert_submap, on_smoother_update}`，
  往**submap 节点 `X(s)`** 加因子——负责大范围/长程的全局一致性（回环尺度），
  防止 global 优化把配准地图整体拽离 ENU。
- 二者共用 `gnss_core`；odometry 保证局部轨迹全局参考，global 保证整图 georeference 不漂。
- **[v2] 交付顺序**：轮 1 只交付 `rtk_global`，用实车/合成数据验证定权与杆臂建模后，轮 1.5 再做 `rtk_odometry`（复用同一套 core，只加壳）。"缺一不可"指最终形态，不指同一轮。

### 2.1 分层原则

- **算法只写一遍**，在 `gnss_core`；所有壳（两个约束模块 + 诊断模块 + 独立节点）都薄，只做订阅、类型转换、回调注册
- **生产者与消费者靠 `RtkFix` 消息解耦**：约束模块可订阅任意一个 fix 源（610 的或 rtkrcv 的），支持整体替换与并行对比，这正是"不同 GNSS 算法 = 不同模块"的落地方式
- **`gnss_core` 单独成包**，不放进 `glim_ext`——否则独立节点会被迫依赖 glim_ext
- **约束分两个模块**：`rtk_odometry`（帧级）与 `rtk_global`（submap 级）职责不同、可各自 config 启停，但共用 `gnss_core` 的 `FrameAligner` / `AntennaPriorFactor` / `RtkNoisePolicy` / `RtkFixBuffer`（这些是纯 Eigen/GTSAM，无 GLIM 依赖，故放共享 core 而非某个模块内）

### 2.2 为什么诊断也是 glim_ext module

先前一版设计认为"诊断不产约束、不消费 submap，不符合 extension module 定位"。该判断错误：`glim_ext/modules/odometry/imu_validator` 正是一个不产因子的状态检测器——继承 `ExtensionModule`，订阅 `OdometryEstimationCallbacks`，计算加速度与重力偏差、LiDAR 与 IMU 角速度差，并通过

```cpp
guik::LightViewer::instance()->register_ui_callback("imu_calibration_validation", [this]{ ui_callback(); });
```

把自己的 ImGui/ImPlot 界面挂进 GLIM 的 viewer。诊断类模块有明确先例。

但存在一个真实约束：**glim_ext module 只在 GLIM 运行时才活**，而无 LiDAR 场景下 GLIM 不运行。故采用 core + 双壳：module 壳提供 GLIM 内的集成体验，独立节点壳保证无 LiDAR 时可用。

---

## 3. 功能归宿总表

**约束：rtk-monitor 现有功能不得有任何缺失。** 下表逐项定死归宿；"轮次"列见 §10。

| # | 功能 | 归宿 | 轮次 |
|---|---|---|---|
| **A 接入与落盘** ||||
| A1 | 差分 RTCM 接入 | `rtcm_bridge` 节点 → `/gnss/rtcm_corrections` | 2 |
| A2 | 板卡原始观测接入 | `rtcm_bridge` 节点 → `/gnss/raw_obs` | 2 |
| A3 | GPCHC 卫导解接入 | `gnss_CGI610`（已有） | — |
| A4 | CAN 融合解接入 | `gnss_CGI610`（已有） | — |
| A5 | 裸流落盘 | rosbag2 录制全部 topic | 2 |
| A6 | 保留天数 / 磁盘水位清理 | 清理节点或 systemd timer（管 rosbag 与 .pos 两处） | 3 |
| A7 | 本地转发（喂 rtkrcv） | 取消——`rtkrcv_node` 订阅 topic，无需转发 | 2 |
| **B 解算** ||||
| B1 | rtkrcv 进程管理（conf 生成、`-r 2`、崩溃重启） | `rtkrcv_node` | 2 |
| B2 | rtkrcv llh 解流解析 | `gnss_core::parse_llh_solution` | 2 |
| B3 | rtkrcv `$SAT` 状态流解析 | `gnss_core::StatEpochAccumulator` | 2 |
| **C 诊断（九规则 + 事件机）** ||||
| C1 | `corr_outage` 差分中断 / 龄期 | `gnss_core` 规则链；`rtk_global` 另有 `max_diff_age` 门限 | 3 |
| C2 | `base_shift` 基站坐标平移 | `gnss_core` 规则链（订阅 `/gnss/rtcm_corrections` 解 1005） | 3 |
| C3 | `abs_ref_shift` 控制点绝对基准校验 | `gnss_core` 规则链 | 3 |
| C4 | `low_sats` 卫星数不足 | `gnss_core` 规则链；`rtk_global` 另有 `min_sats` 门限 | 3 |
| C5 | `multipath` 残差异常星 | `gnss_core` 规则链（需 `$SAT` 流） | 3 |
| C6 | `ambiguity` ratio 不足 | `gnss_core` 规则链 | 3 |
| C7 | `cycle_slip` 周跳频繁 | `gnss_core` 规则链（需 `$SAT` 流） | 3 |
| C8 | `device_divergence` 610 与独立解发散 | `gnss_core` 规则链（订阅两路 `RtkFix`） | 3 |
| C9 | `no_data` / `no_solution` / `not_fixed` / `rtk_fixed` 状态 | `gnss_core` 规则链 | 3 |
| C10 | 事件状态机（迟滞 open/close） | `gnss_core::EventMachine` | 3 |
| **D 存储** ||||
| D1 | 历元表（1 Hz 抽稀，四源） | `.pos` 文本文件，每源一个（§6.3）；SQLite 历元库取消 | 2 |
| D2 | 事件表 | `events.log` 文本 | 3 |
| D3 | 基站坐标史 | `.pos` 同目录的 `base.pos` | 3 |
| D4 | DB 保留清理 | 见 A6 | 3 |
| **E 界面（8 项）** ||||
| E1-E8 | 地图/三轨迹开关/状态条/天空图/时间线/事件列表/回放条/瓦片 | `register_ui_callback` 挂进 GLIM viewer（机制见 §11） | 4 |
| **F 回放与报告** ||||
| F1 | 按时间轴重推 | rosbag2 原生回放 | 2 |
| F2 | 报告（固定率/分小时/事件/基站稳定性/绝对基准/610 偏差/问题路段） | 离线工具，读 `.pos` + `events.log` | 4 |
| F3 | 打印为 PDF | 同上 | 4 |
| **G 对外** ||||
| G1 | UDP JSON Lines | ROS2 topic 取代 | — |

---

## 4. 消息定义

新建独立消息包 **`gnss_msgs`**（不放进驱动包，否则 glim_ext 要拖驱动的全部依赖；workspace 内 `rslidar_msg` 已是此惯例）。

### 4.1 `gnss_msgs/RtkFix.msg`

**为什么不能用 `sensor_msgs/NavSatFix`**：其 `status.status` 枚举只有四档，`gnss_CGI610` 的 `MapStatus()` 把 `RTK_FIXED` / `RTK_FIXED_NO_HEADING` / `RTK_FLOAT` / `RTK_FLOAT_NO_HEADING` **全部压成同一个 `STATUS_GBAS_FIX`**。下游因此无法区分 1 cm 固定解与 30 cm 浮动解——而这正是定权最关键的一维。同时 `diff_age`、卫星数、航向有效性也无处安放。

**质量字段归一化，不绑 610**：不同 GNSS 算法（610 板卡、rtkrcv、未来的 PPP）都应能发同一种 fix，故质量做成源无关的枚举，同时保留原始值备查。

```
# gnss_msgs/RtkFix.msg —— 带质量标签的 GNSS 定位
std_msgs/Header header    # 接收/发布时刻（ROS 时钟）
float64 gnss_time         # [v2] 板卡自报的观测时刻（UTC unix 秒；由 GPS 周/周内秒换算）。0 = 源不提供

# 归一化解质量（源无关）
uint8 QUALITY_NONE=0
uint8 QUALITY_SINGLE=1
uint8 QUALITY_DGPS=2
uint8 QUALITY_FLOAT=3
uint8 QUALITY_FIXED=4
uint8 quality
uint8 raw_status          # 板卡原始状态字（610: SatStatus 0-9），仅供追溯

float64 latitude          # WGS-84
float64 longitude
float64 altitude          # 椭球高
float64[3] sigma_enu      # 各轴标准差 (m)，板卡直出，非推算

float32 diff_age          # 差分龄期 (s)
uint8 sats_used
uint8 sats_main
uint8 sats_aux

float32 heading           # 双天线航向 (deg, 北零顺时针)
float32 heading_sigma
bool heading_valid
```

**610 状态映射**（`gnss_CGI610` 内）：

| `cgi610::SatStatus` | `quality` | `heading_valid` |
|---|---|---|
| `RTK_FIXED` (4) | `QUALITY_FIXED` | true |
| `RTK_FIXED_NO_HEADING` (8) | `QUALITY_FIXED` | false |
| `RTK_FLOAT` (5) | `QUALITY_FLOAT` | true |
| `RTK_FLOAT_NO_HEADING` (9) | `QUALITY_FLOAT` | false |
| `PSRDIFF` (2) / `PSRDIFF_NO_HEADING` (7) | `QUALITY_DGPS` | true / false |
| `SINGLE` (1) / `SINGLE_NO_HEADING` (6) / `COMBINED_DR` (3) | `QUALITY_SINGLE` | true / false |
| `NO_FIX` (0) | `QUALITY_NONE` | false |

字段来源均为 `cgi610::Cycle` 已解析的成员：`lat_deg`/`lon_deg`/`alt_m`、`pos_sigma_enu_m[3]`、`gps_age_s`、`sats_used`/`sats_main`/`sats_aux`、`heading_deg`、`att_sigma_deg[0]`。

**[v2] 关于时间戳**：项目现场的传感器时间同步尚不可靠。`header.stamp` 若是主机接收时刻，串口/CAN 延迟几十 ms 在 5 m/s 车速下即 10–25 cm，与建图分层指标（0.25 m）同量级。因此：
- `gnss_time` 取板卡自报的观测时刻（GPCHC/CAN 帧内的 GPS 周与周内秒 → UTC unix 秒，闰秒当前 18 s）；`Cycle` 若尚未解析该字段，驱动侧需补解析。
- `header.stamp − gnss_time` 即接收延迟，可在线诊断（轮 3 规则链可加 `stamp_skew`）。
- `COMBINED_DR` 状态下 `heading_valid = false`（航向来自惯导递推，不是双天线观测）。
- `pos_sigma_enu_m` 的分量顺序须在驱动侧核对板卡协议（板卡常按 N/E/U 给出），保证进 `RtkFix.sigma_enu` 时为 E/N/U。
- `~/rtk_fix` 用 **reliable** QoS 发布：GLIM 的 `TopicSubscription` 固定以默认（reliable）QoS 订阅，best_effort 发布会静默不匹配。

**故意不含速度字段**：`rtk_global` 用不上，驱动已发 `nav_msgs/Odometry`；待真正实现 odometry 约束模块时再议（YAGNI）。

### 4.2 `gnss_msgs/RawStream.msg`

```
# 裸字节流（差分 / 原始观测通用）
std_msgs/Header header
uint8[] data
```

### 4.3 驱动改动

`gnss_CGI610` **增发** `~/rtk_fix`，现有 `~/fix`（NavSatFix）、`~/imu`、`~/odom` 原样保留，不影响任何现有订阅者。

---

## 5. 数据层

### 5.1 `rtcm_bridge` 节点

差分 RTCM 与板卡原始观测是裸字节流，需进入 ROS 总线才能被录制、回放、并供多个消费者订阅。

**设计决定：`rtkrcv_node` 内不含任何 TCP 代码**，它是纯订阅者——喂一个 bag 就能完整测试解算逻辑，无需平台和板卡在线。这比"解算直连 TCP、另外转发一份"的双路径方案更干净（一套代码、一条维护路线）。

`rtcm_bridge` 保持"笨"：TCP 客户端 → 收到多少发多少 → 断线重连。**不做任何解析**。RTCM 分帧与 1005 解析放在 `gnss_core`（`rtkrcv_node` 与 `gnss_diag` 共用，只写一遍）。

| 话题 | 类型 | 内容 |
|---|---|---|
| `/gnss/rtcm_corrections` | `gnss_msgs/RawStream` | 平台 5G 广播差分流 |
| `/gnss/raw_obs` | `gnss_msgs/RawStream` | 板卡原始观测流 |

### 5.2 落盘：rosbag2 存全量

四路数据全部在总线上后，rosbag2 直接承担原始流记录与回放，取代 rtk-monitor 的自建裸流落盘。

**缺口**：rosbag2 只有 `--max-bag-size` / `--max-bag-duration` 分卷，**没有按保留天数或磁盘水位自动删除**。需一个清理节点或 systemd timer（A6），同时管 rosbag 目录与 `.pos` 目录。

### 5.3 摘要：`.pos` 文本

`metadata.yaml` 是包级元数据（时长、起始时间、各 topic 消息数、QoS、文件清单），**无任何逐历元内容**，不能充当摘要。而"过去 14 天的分小时固定率"这类统计从 bag 顺序流里查非常痛苦。

故保留一份 1 Hz 抽稀摘要，**格式采用 RTKLIB `.pos`**：

```
%  GPST              latitude(deg)  longitude(deg)  height(m)  Q  ns  sdn  sde  sdu  sdne  sdeu  sdun  age  ratio
2026/09/03 10:23:45.000  44.50123456  90.28765432  617.123   1  38  0.012 0.011 0.030  ...   0.8  20.5
```

`.pos` 的标准列恰好覆盖全部需存字段：`Q`=解质量、`ns`=卫星数、`sdn/sde/sdu`=各轴 σ、`age`=差分龄期、`ratio`。

**选 `.pos` 而非 CSV / SQLite 的理由**：

- **RTKPLOT 可直接打开多个 `.pos` 对比轨迹**——轨迹对比的可视化白赚，不必自写（直接服务 §9）
- `rnx2rtkp` 后处理输出天然同格式，参考真值零转换即可加入对比
- 无依赖、追加写、崩溃安全；GNSS 工程师无需学习新格式

**目录布局**（按天轮转，旧文件 gzip）：

```
/data/gnss/20260903/
  ├── can.pos          610 融合解
  ├── gpchc.pos        610 卫导解
  ├── rtkrcv.pos       独立解算
  ├── ref.pos          后处理基准（可选）
  ├── base.pos         基站坐标史
  └── events.log       诊断事件（时间 级别 代码 消息）
```

体积：1 Hz × ~110 字节 ≈ 38 MB/天/源，gzip 后个位数 MB，保留 14 天无压力。

**结论**：rosbag2 存全量原始流（可回放整条链路），`.pos` 存 1 Hz 摘要（报告、统计、轨迹对比），**SQLite 历元库取消**。

---

## 6. `gnss_core` 库

纯 C++17，**不依赖 ROS、不依赖 GLIM**，全部可单测。单独成包。

```
gnss_core/
├── include/gnss_core/
│   ├── types.hpp              RtkFixSample / EpochRecord / Verdict / Quality
│   ├── rtk_noise_policy.hpp   质量 → GTSAM noise model（§7.2）
│   ├── rtk_fix_buffer.hpp     缓存 + 时间插值
│   ├── frame_aligner.hpp      T_world_enu 估计（SVD，可重估精化，不冻结）
│   ├── antenna_prior_factor.hpp  GTSAM 因子（统一 body_point 形式，odometry/global 共用）
│   ├── rules.hpp              九条诊断规则链
│   ├── event_machine.hpp      事件迟滞状态机
│   ├── trajectory_compare.hpp 轨迹对比统计（§9）
│   ├── rtcm.hpp               RTCM3 分帧 + 1005/1006 解析
│   ├── rtkstat.hpp            rtkrcv $SAT / llh 解析
│   └── pos_io.hpp             .pos 读取（轮 1，§9 对比需要）与写出（轮 2，§5.3）
├── src/…
└── test/…                     gtest，不需要 ROS / GLIM 运行
```

> **注**：`rtk_noise_policy` 与 `antenna_prior_factor` 依赖 GTSAM 类型。GTSAM 不是"框架"（无运行时/无消息总线），且 GLIM 与独立节点两侧都已链接它，故允许 `gnss_core` 依赖 GTSAM 与 Eigen。ROS 与 GLIM 依赖则严格禁止。

### 6.1 坐标换算：用 GeographicLib

生态内同一套公式已有三份实现：`gnss_CGI610` 的手写 `EnuConverter`、其 ROS1 遗留文件用的 `GeographicLib::LocalCartesian`、`gnss_global` 私有的 `geodetic.cpp`。**不再自写第四份。**

采用 `GeographicLib::LocalCartesian`（系统已装 `libgeographiclib-dev 2.3`；CMake 经 `/usr/share/cmake/geographiclib/FindGeographicLib.cmake`，亦有 `geographiclib.pc`）。

**用局部 ENU 而非 UTM**：

| | UTM | LocalCartesian |
|---|---|---|
| 分带 | 跨带需处理 | 无 |
| 尺度畸变 | 有投影尺度因子 | 局部严格无畸变 |
| 适用 | 大范围制图 | **几公里的矿区** |

故 `gnss_global` 中的 `T_world_utm` 在本设计中对应 **`T_world_enu`**。ENU 原点由配置指定，默认取首个通过门限的 fix，**一经设定即固定，永不作为优化变量**（见 §7 甲方案）。

**不需要 `Geoid`**：GNSS 出椭球高，矿区范围内 geoid 差为常数，会被 `T_world_enu` 的平移吸收。

**FrameAligner 为何在 core**：它只是点集配准（Umeyama），纯 Eigen、无 GLIM 类型依赖；且 `rtk_odometry` 与 `rtk_global` 两个模块都要用（各自 world 系不同，各持一个 FrameAligner 实例），放共享 core 而非某模块内以避免重复。

**[v2] 一次 bootstrap 后冻结，不持续重估。** v1 设想"随优化推进重算 T"，评审指出这是反馈回路：RTK 因子进图后，node 位置已被 `T_world_enu × ENU` 拉过去，再用它们反推 `T_world_enu` 是用结果验证前提；更实际的问题是旧因子按旧 gauge 算 `p_world`、新因子按新 gauge，图里同时存在两套基准。轮 1 与 `gnss_global` 一样在基线足够时解一次并冻结；`FrameAligner::add` 在 `initialized()` 后只累积不重解。真正的联合估计走 §7.乙。

---

## 7. 约束模块（rtk_odometry + rtk_global）

RTK 约束挂两处（缺一不可，§2 已述）。两模块共用 `gnss_core`（RtkFixBuffer / RtkNoisePolicy / FrameAligner / AntennaPriorFactor），差异只在：挂哪套 callback、约束哪个 node、`body_point` 怎么算。各自可 config 启停。

### 7.0 挂载点对照

| | `rtk_odometry`（modules/odometry/） | `rtk_global`（modules/mapping/） |
|---|---|---|
| callback | `OdometryEstimationCallbacks::{on_new_frame, on_smoother_update}` | `GlobalMappingCallbacks::{on_insert_submap, on_smoother_update}` |
| 约束 node | `X(frame.id)` = `T_world_imu`（该帧时刻的 IMU 位姿，本就是图变量） | `X(submap.id)` = submap 原点位姿 |
| body_point | `lever_imu`（天线在 IMU 系） | `T_origin_frame(t) · lever_imu`（把杆臂搬到 submap 原点系，约束 t 时刻真实位置而非原点） |
| 作用 | 局部轨迹全局参考（滑窗内每帧被 RTK 钉住） | 大范围/长程一致性，防 global 优化把整图拽离 ENU |
| gauge | 自己的 `T_world_enu`（odometry world） | 自己的 `T_world_enu`（global map world；与 odometry world 可能不同，故各持一个） |

### 7.1 主流程（两模块同构，只差挂载点）

```
RtkFix topic ──→ [1] 入 RtkFixBuffer（按时间排序）
                       │
GLIM 回调（新帧 / 新 submap）─→ [2] 目标 node 入队（带 id 与时间戳 t）
                       │
             后台线程循环：
                       ↓
       [3] 关联：对目标 node 的时间戳 t，在缓冲中取左右两个 fix 线性插值
           └─ 质量取两端较差者（不得插出假 FIXED）
                       ↓
       [4] 经纬高 → ENU（GeographicLib::LocalCartesian，原点固定）
                       ↓
       [5] FrameAligner.add(node 位置, ENU)；基线足够时估一次 T_world_enu
           └─ 甲 [v2]：bootstrap 立 gauge 后冻结（理由见 §6.1）
                       ↓
       [6] 造因子：p_world = T_world_enu × ENU
           └─ RtkNoisePolicy 决定 noise model 或拒绝（不过门限则跳过该历元）
           └─ AntennaPriorFactor(X(node_id), p_world, body_point, model)
               odometry: body_point = lever_imu
               global:   body_point = T_origin_frame(t) · lever_imu
                       ↓
       [7] 入输出队列 ──→ 对应的 on_smoother_update 中 drain 进因子图
```

[3][5] 自 `gnss_global` 移植其数学（已验证），但抽成 `gnss_core` 可测单元、且 `T_world_enu` 改为可重估。**改进集中在 [6]**（定权）与挂载点（帧/submap 而非只 submap 原点）。

线程模型沿用 `gnss_global`：后台线程 + `ConcurrentVector` 队列；`on_smoother_update` 运行在优化器线程上，只做 drain + add，必须快。

**[v2] 跨线程访问规则**：`callbacks.hpp` 明确 `submap->T_world_origin` 只在 global mapping 线程更新，跨线程读不安全（`gnss_global` 在后台线程读它，是已知缺陷）。`on_insert_submap` 回调本身运行在 global mapping 线程，因此在回调内把 `id`、`origin_frame()->stamp`、`T_world_origin.translation()` 拷进一个 POD（`SubmapAnchor`）再入队，后台线程**不持有 `SubMap` 指针**。odometry 侧同理（`on_new_frame` 内拷 `id`/`stamp`/`T_world_imu`）。

**[v2] 时间戳选择**：入 `RtkFixBuffer` 前，样本时间 `stamp = (stamp_source == "gnss_time" && gnss_time > 0 ? gnss_time : header.stamp) + time_offset`。`gnss_core::effective_stamp()` 实现，可单测。

### 7.甲乙 T_world_enu 的估计（本轮=甲）

- **甲（本轮，[v2] 冻结版）**：前若干帧/submap 攒够基线后 bootstrap 出 `T_world_enu` 立 gauge，之后冻结。建图结束时用该 gauge 给全部激光点赋 WGS-84 坐标。ENU 原点**始终固定**，不进图当变量。
- **乙（记待办，本轮不做）**：把 world↔enu gauge 作为因子图变量联合优化，RTK 因子同时做配准 + 纠漂移；ENU 原点仍固定。最准但需自定义 gauge 因子、改动大。

### 7.2 `RtkNoisePolicy`（核心）

纯函数：`(sample, config) → noise model 或 拒绝`。

**三道门限**（任一不过则该历元不加因子）：

| 门限 | 默认 | 理由 |
|---|---|---|
| `quality < min_quality` | 拒绝 DGPS / SINGLE | 米级先验在矿区会与 LiDAR 打架 |
| `diff_age > max_diff_age` | 15 s | **差分中断由此兜住**，无需额外诊断通道 |
| `sats_used < min_sats` | 6 | 与 rtk-monitor 同阈值 |

**σ 构建**：

```
σ = sample.sigma_enu × quality_sigma_scale[quality]
σ = max(σ, sigma_floor)
σ[2] *= vertical_scale
model = Diagonal::Sigmas(σ)
model = Robust(Huber(robust_delta), model)
```

**为什么不能只用板卡 σ**：板卡报的是**形式精度**而非真实误差。浮动解 σ 系统性偏乐观；更危险的是**错误固定**（ambiguity 解错）——σ 只有几毫米，位置却偏几十厘米甚至数米。故必须叠加按质量的放大，再以**鲁棒核**兜底：错误固定的历元被 Huber 自动降权，不致单个坏历元拽歪整张图。三层（门限 + 质量缩放 + 鲁棒核）是常数 `prior_inf_scale` 完全没有的。

**浮动解收但放大**：矿区坑底遮挡多，固定率可能不高，全丢会导致长时间无全局约束而漂移；放大 σ 后仍可提供弱约束。

**垂直用板卡 σ × 3**：`gnss_global` 直接将 Z 权重设为 0（完全忽略高程）。矿区开阔天空下 RTK 高程质量并不差，直接丢弃可惜；乘一个放大系数是更合理的折中。

> `quality_sigma_scale` 的默认值为工程估计，**须经 §9 的实测标定后修订**。

### 7.3 `AntennaPriorFactor`（统一 body_point 形式）

补上上游明说忽略的 IMU-GNSS 杆臂。**一个因子服务两个模块**，差异全在调用方传入的 `body_point`（天线在被约束 node 局部系里的位置）：

```
AntennaPriorFactor(gtsam::Key key, Point3 measured_world, Point3 body_point, SharedNoiseModel);
h(X) = X.transformFrom(body_point)          // X 是被约束 node 的位姿
残差 = h(X) − measured_world
```

- `rtk_odometry`：X = `T_world_imu`，`body_point = lever_imu`（天线在 IMU 系）→ `h(X)=X·lever_imu`。
- `rtk_global`：X = submap 原点位姿，`body_point = T_origin_frame(t)·lever_imu`（把杆臂先搬到 submap 原点系）→ 约束的几何是 **t 时刻车的真实天线位置**，而非 submap 原点。

带解析 Jacobian（`X.transformFrom` 的 GTSAM 内建雅可比），须过 `numericalDerivative11` 数值校验。**`lever_imu = [0,0,0]` 且 `body_point` 为该 node 原点时退化为纯平移先验**；外参标定到位后填入即生效。

**[v2] 轮 1 的 `rtk_global` 在 `origin_frame()->stamp` 处取 RTK 样本，此时 `T_origin_frame = I`，`body_point = lever_imu`。** 因子构造参数命名为 `body_point`，壳侧注释说明这一等价关系；轮 1.5 的 odometry 壳直接传 `lever_imu`。

**[v2] "杆臂为零退化为纯平移先验"不等于"没有风险"**：矿卡天线到 IMU 通常 1–3 m。杆臂不填时转弯处残差可达米级，若 `sigma_floor` 仍是 2 cm，Huber（δ=1.345σ）会把转弯段的**好固定解**当外点系统性降权，只剩直线段起作用——与期望相反。故：**杆臂未标定期间 `sigma_floor` 必须设为杆臂量级（默认 1.0 m）**，用 §9.3 工具标定杆臂后再降到 cm 级。

> 相比原设计（`h(X)=X.t + X.R·lever`,只对 odometry 成立），`transformFrom(body_point)` 统一了两种挂载：odometry 传杆臂、global 传"帧偏移·杆臂",同一因子、同一 Jacobian。

### 7.4 模块结构

不照抄 `gnss_global` 的单体 header（其 200 行 `backend_task()` 把关联、对齐、造因子全糅在一起，无法单测）。算法单元在 `gnss_core`，两个模块壳都只做接线：

```
glim_ext/modules/odometry/rtk_odometry/
├── package.xml / CMakeLists.txt          # → librtk_odometry.so
├── include/glim_ext/rtk_odometry_module.hpp   # ExtensionModuleROS2，挂 OdometryEstimationCallbacks
└── src/glim_ext/rtk_odometry_module_ros2.cpp  # + create_extension_module()

glim_ext/modules/mapping/rtk_global/
├── package.xml / CMakeLists.txt          # → librtk_global.so
├── include/glim_ext/rtk_global_module.hpp     # ExtensionModuleROS2，挂 GlobalMappingCallbacks
└── src/glim_ext/rtk_global_module_ros2.cpp    # + create_extension_module()
```

两壳都 `find_package(gnss_core)` 并链接之；共用逻辑不在壳里重复。

### 7.5 配置

两模块各一份配置：`config_rtk_odometry.json` 与 `config_rtk_global.json`，键集相同（下例以 global 为例；odometry 同结构，section 名为 `rtk_odometry`）。分开是因为两者门限/权重可独立调（例如 odometry 侧可更保守）。

> GLIM 的 `Config` 只支持 section 内的**扁平**键值，类型限于 bool / int / double / string / `vector<...>` / `Eigen::Vector*d` / `Isometry3d`，故不使用嵌套对象。

```jsonc
{
  "rtk_global": {   // odometry 侧改为 "rtk_odometry"
    // --- 输入 ---
    "rtk_fix_topic": "/gnss_cgi610/rtk_fix",   // [v2] 驱动包 gnss_chcnav、节点名 gnss_cgi610
    "stamp_source": "gnss_time",         // [v2] header | gnss_time（gnss_time==0 时回退 header）
    "time_offset": 0.0,                  // [v2] s，加到样本时间上；由 §9.3 工具估计

    // --- 门限：任一不过 → 该历元不加因子 ---
    "min_quality": 3,          // 归一化 quality 下限，3=FLOAT（>4 为配置错误，启动即报错）
    "max_diff_age": 15.0,      // s
    "min_sats": 6,

    // --- 噪声模型 ---
    // 索引 0 (NONE) 恒被 min_quality 拦下，占位不参与计算；
    // 若将 min_quality 降至 0，实现须拒绝非正的 scale 而非产生零 σ（无穷权重）。
    //                       [NONE, SINGLE, DGPS, FLOAT, FIXED]
    "quality_sigma_scale": [   0.0,   50.0,  20.0,   5.0,   1.0],
    // [v2] 杆臂未标定期间 sigma_floor 取杆臂量级（1.0 m），否则 Huber 会把转弯段固定解当外点；
    //      T_imu_gnss 标定填入后改回 [0.02, 0.02, 0.05]
    "sigma_floor": [1.0, 1.0, 1.0],      // ENU 下限 (m)
    "vertical_scale": 3.0,
    "robust_kernel": "huber",            // none | huber | cauchy
    "robust_delta": 1.345,

    // --- 外参 ---
    "T_imu_gnss": [0.0, 0.0, 0.0],       // 天线杆臂 (m, IMU 系)；用 §9.3 工具估计或量取

    // --- 帧对齐 ---
    "min_baseline": 10.0,                // m
    "enu_origin": [],                    // 空=自动取首个过门限的 fix；或 [lat,lon,alt]

    // --- 缓冲 ---
    "fix_buffer_horizon": 60.0           // s，超期丢弃
  }
}
```

杆臂用 `Vector3d` 而非 `Isometry3d`：天线只有位置、无姿态，写成 6DoF 是假精确。

### 7.6 部署接线

`glim/config/casbot/config_ros.json`：

```jsonc
"extension_modules": [
  "libmemory_monitor.so",
  "libstandard_viewer.so",
  "libimu_validator.so",
  "librtk_odometry.so",     // ← 新增(帧级约束)
  "librtk_global.so"        // ← 新增(submap 级约束)
//"libgnss_global.so"       // ← 与 rtk_global 二选一(下述)
]
```

- `librtk_odometry.so` 与 `librtk_global.so` **可同时启用**——它们挂不同的图(odometry / global),不冲突,正是"两处都要"。
- `librtk_global.so` 与 `libgnss_global.so` **二选一**:两者都对 submap 加位置先验,同开即重复约束打架。A/B 对比通过注释切换。

**[v2] 构建形态**：`glim_ext` 是**一个** ament 包，各模块经顶层 `CMakeLists.txt` 的 `option(ENABLE_xxx)` + `add_subdirectory(modules/...)` 编入，产物统一装到 `install/glim_ext/lib/`。新模块因此不是独立 colcon 包，而是：`glim_ext/CMakeLists.txt` 加 `option(ENABLE_RTK_GLOBAL)` 与 `add_subdirectory`，`glim_ext/package.xml` 加 `<depend>gnss_core</depend>` `<depend>gnss_msgs</depend>`（模块 CMake 的 `ament_auto_find_build_dependencies()` 从这里取依赖）。构建命令是 `colcon build --packages-select glim_ext`。

实际布局（2026-09-07 核实）：`gnss_msgs` 与 `gnss_CGI610` 正本在 `finder_ros/drivers/`（`driver_ws/src` 下为符号链接）；`gnss_core` 位于 `glim_ext/gnss_core/`，由 colcon 在 `glim_ws` 内识别为独立包 `gnss_core`（仍满足 §2.1 "单独成包、不依赖 glim_ext"——它只是放在同一仓库目录下）。构建 `glim_ws` 前须 `source ~/driver_ws/install/setup.bash` 以获得 `gnss_msgs`。

---

## 8. `gnss_diag` 模块

承载 C1–C10 全部九条规则与事件状态机。规则逻辑在 `gnss_core::rules` / `EventMachine`，两个壳：

| 壳 | 用途 | 输入 |
|---|---|---|
| `glim_ext/modules/mapping/gnss_diag`（`libgnss_diag.so`） | GLIM 运行时，诊断集成在 GLIM 界面（`register_ui_callback`，同 `imu_validator`） | 订阅 `RtkFix` × N、`RawStream` |
| `gnss_diag_node`（独立 ROS2 节点） | 无 LiDAR / 不跑 GLIM 时诊断与轨迹对比 | 同上 |

九条规则的输入来源：

| 规则 | 所需输入 |
|---|---|
| `corr_outage` | `RtkFix.diff_age` 或 `/gnss/rtcm_corrections` 到达时刻 |
| `base_shift` | `/gnss/rtcm_corrections` → RTCM 1005/1006 |
| `abs_ref_shift` | `RtkFix` + 配置的控制点坐标 |
| `low_sats` / `ambiguity` / `not_fixed` | `RtkFix` |
| `multipath` / `cycle_slip` | rtkrcv `$SAT` 流（由 `rtkrcv_node` 发布） |
| `device_divergence` | 两路 `RtkFix`（610 与 rtkrcv） |

---

## 9. 无 LiDAR 轨迹对比与系数标定

`rtk_global` 是 mapping 约束模块，约束对象是 submap，而 submap 来自 LiDAR——**模块本身无 LiDAR 无法运行**。但轨迹对比不需要 LiDAR，且是本设计的必需能力，理由有二：

1. 现场常先有 GNSS 数据、后有同步的 LiDAR+RTK 包；对比能力使验证不被数据阻塞
2. **§7.2 的 `quality_sigma_scale` 默认值是工程估计**，直接决定模块表现，不应拍脑袋

### 9.1 可对比的轨迹（有几条比几条）

| # | 轨迹 | 来源 |
|---|---|---|
| 1 | 610 组合导航融合解 | `can.pos` |
| 2 | 610 卫导解 | `gpchc.pos` |
| 3 | rtkrcv 独立解算 | `rtkrcv.pos` |
| 4 | 后处理基准（可选，精度最高，可作参考真值） | `ref.pos`（`rnx2rtkp` 输出） |

四者同为 `.pos` 格式，**RTKPLOT 可直接叠加显示**，可视化对比零成本。

**轮 1 的数据来源**：`.pos` 写出属轮 2，故轮 1 的对比数据由以下两条既有途径提供——`rnx2rtkp` 的后处理输出本身即 `.pos`（轨迹 4）；rtk-monitor 既有录包中的 can / gpchc / rtkrcv 历元经一次性导出脚本转为 `.pos`（轨迹 1–3）。轮 2 起由 `pos_io` 写出器直接产出。

### 9.2 系数标定

`gnss_core::trajectory_compare` 以参考轨迹（优先 4，无则取质量最高者）为基准，**按 `quality` 分档统计实际误差分布**：

- 各档的实际水平/垂直 RMSE
- **板卡报的 σ 与实际误差的比值**——该比值即 `quality_sigma_scale`

产出一张标定表，写入 `config_rtk_global.json`。至此定权策略有实测支撑，而非估计值。

**[v2] 统计量**："实际误差 / 板卡 σ" 的**算术均值**会被少数外点主导（一次错误固定 σ=3 mm、误差 1 m → 比值 300）。同时输出**中位数**与 `RMS(误差)/RMS(σ)`，建议系数取中位数；均值仅作参考列。

**[v2] `.pos` 时间系统**：RTKLIB `.pos` 默认时间列为 **GPST**（比 UTC 快 18 s），头部 `% (time=GPST)` 或 `(time=UTC)` 标明。`pos_io` 须识别并统一到 UTC unix 秒（闰秒可配置），否则与 ROS 时间戳导出的轨迹按 0.1 s 容差配对会全部落空。

### 9.3 [v2] 杆臂与时间偏移估计（无需专用标定场）

项目现场的 IMU–GNSS 杆臂与 RTK–LiDAR 时间偏移都未标定，而二者直接决定 §7.2/§7.3 的有效性。用建图产物即可估计：

输入：GLIM 输出的 IMU 轨迹 `T_world_imu(t_i)`（`glim_rosbag` dump 的 traj）与同段 RTK 的 ENU 轨迹 `enu(t)`。
模型：对每个 `t_i`，`R_world_imu(t_i)·lever + t_world_imu(t_i) = T_world_enu · enu(t_i + Δt)`。
求解：外层对 `Δt` 网格搜索（±0.5 s、步长 10 ms），内层给定 `Δt` 时 `(T_world_enu 的 yaw/平移, lever)` 是线性最小二乘（`R_world_imu` 已知，对 `lever` 线性；yaw 用 Umeyama 初值后一步 Gauss-Newton）。取残差最小的 `Δt`。
输出：`lever_imu`、`time_offset`、各自的残差 RMS 与可观性提示（轨迹无转弯则 `lever` 水平分量不可观，工具须报告）。
产出直接填入 `config_rtk_global.json` 的 `T_imu_gnss` 与 `time_offset`。

这个工具与 §9.2 用同一批数据，不阻塞于专用标定。

---

## 10. 实施轮次

| 轮 | 内容 | 依赖 |
|---|---|---|
| **1**（本轮实施，[v2]） | `gnss_msgs`（RtkFix〔含 gnss_time〕/ RawStream）、`gnss_core` 骨架与约束单元（NoisePolicy / FixBuffer / FrameAligner〔冻结〕/ AntennaPriorFactor〔body_point〕/ effective_stamp）、**`rtk_global`（submap 级）**、驱动增发 `~/rtk_fix`、`.pos` **读取**（GPST/UTC）+ 轨迹对比与系数标定（§9.2）、**合成注入测试（§12.3）**、**杆臂/时间偏移估计工具（§9.3）** | — |
| **1.5** [v2] | `rtk_odometry`（帧级壳，复用 core），在 `rtk_global` 实车验证通过后 | 轮 1 |
| **2** | `rtcm_bridge`、`rtkrcv_node`（B1–B3）、`.pos` **写出**（D1）、rosbag2 录制回放接入（A5/F1） | 轮 1 的消息与 core |
| **3** | `gnss_core` 九条规则 + 事件机（C1–C10）、`gnss_diag` 双壳、`events.log`/`base.pos`（D2/D3）、清理逻辑（A6/D4） | 轮 2 的 `$SAT` 与 RTCM 流 |
| **4** | 界面迁移（E1–E8，`register_ui_callback`）、报告离线工具（F2/F3） | 轮 3 的诊断输出 |

每轮结束时，上表 §3 中该轮次的功能项须全部可用。

---

## 11. 界面迁移机制（轮 4，此处仅定机制）

`imu_validator` 的先例表明：**任何 glim_ext module 都可通过 `guik::LightViewer::instance()->register_ui_callback(name, fn)` 把自己的 ImGui/ImPlot 界面挂进 GLIM 的 standard viewer**，析构时注销。故 rtk-monitor 的八项界面功能不需要独立的 viewer module，由 `gnss_diag` 模块自带界面即可，与 GLIM 现有诊断类模块保持一致。

界面能力的具体映射（地图底图、瓦片、回放条等在 ImGui 环境下的等价形态）留待轮 4 设计。

---

## 12. 测试策略

### 12.1 纯单元测试（gtest，不启 ROS / 不启 GLIM）

`gnss_core` 全部单元可独立测试：

| 单元 | 关键用例 |
|---|---|
| `RtkNoisePolicy` | 各质量档缩放正确；三道门限各自触发拒绝；`sigma_floor` 生效（板卡报 1 mm → 抬至 2 cm）；`vertical_scale` 仅作用于 Z；阈值边界行为 |
| `RtkFixBuffer` | 线性插值正确；**质量取两端较差者**（FIXED 与 SINGLE 之间插出 SINGLE）；时间戳越界返回空；超期样本清理 |
| `FrameAligner` | 构造已知 `T_world_enu` → 合成 (submap, ENU) 对 → 验证 SVD 可恢复；基线不足不初始化；**初始化后再 add 不改变 T（冻结）[v2]**；共线退化行为 |
| `effective_stamp` [v2] | `gnss_time` 优先、为 0 回退 `header`、`time_offset` 叠加 |
| `pos_io` [v2] | GPST 头 → 减闰秒；UTC 头 → 原样 |
| `AntennaPriorFactor` | **Jacobian 数值验证**（`gtsam::numericalDerivative11` 对比解析式）；杆臂为零时与 `PoseTranslationPrior` 结果一致 |
| 规则链 / 事件机 | 沿用 rtk-monitor 既有用例（构造指标序列断言结论） |

### 12.2 模块加载与订阅

`librtk_global.so`（轮 1.5 起含 `librtk_odometry.so`）可被 GLIM 扩展机制加载。**[v2] 验证方式**：用一段短 bag 跑 `glim_rosbag`，`config_ros.json` 启用该模块，断言日志出现模块初始化与 `T_world_enu=` 行、收到的 `RtkFix` 计数 > 0。不用脱离 GLIM 进程的 `dlopen`/ctypes 调 `create_extension_module()`——构造函数依赖 GLIM 全局 config 并起线程，脱离进程既跑不起来也验证不了任何东西。

### 12.3 合成注入（无实车即可验证核心行为）

取一段既有纯 LiDAR 包，先让 GLIM 无 GNSS 跑出轨迹作基准，再据该轨迹**合成"理想 RTK 观测"**（加已知噪声与质量标签），然后注入故障：

| 注入 | 期望行为 | 验证了什么 |
|---|---|---|
| 5% 错误固定历元（σ 仅 cm，位置偏 1 m） | 开鲁棒核：轨迹 RMSE 基本不变<br>关鲁棒核：明显恶化 | 鲁棒核确实兜底 |
| 一段 `diff_age` 持续增长 | 该段不产生因子 | 差分中断门限有效 |
| 一段全浮动解 | 因子仍加但权重降 5 倍 | 质量分档有效 |
| 非零杆臂 + 车辆转弯 | 不设杆臂时先验有系统性偏移，设置后消失 | 杆臂建模有效 |

**该层的价值**：验证的恰是 `gnss_global` 做不到的部分，且结论不依赖真实 RTK 数据——注入的是已知真值。

**[v2] 实现载体**：`gnss_core/tools/synth_rtk_fix`——读 GLIM 轨迹 → `LocalCartesian::Reverse` 反算 lat/lon/alt → 加噪声与质量标签、按注入脚本篡改 → 写出 `.pos` 与 `RtkFix` bag（bag 写出用 Python `rosbag2_py`，在 Orin 上跑）。四种注入各一个用例，验收指标为轨迹 RMSE 对比。这是轮 1 必做项，不再只是"策略"。

### 12.4 实车对比（需现场数据，此处仅定验收标准）

录一段 LiDAR + IMU + RTK 的包后，同一包分别挂 `libgnss_global.so` 与 `librtk_global.so`，对比：轨迹相对固定解历元的 RMSE、重复路段点云重合度、优化器残差分布。

**验收标准**：在含浮动解或差分中断的路段，`rtk_global` 的轨迹 RMSE 应优于 `gnss_global`，且不出现被单个坏历元拽偏的情形。

---

## 13. 风险与待确认项

| 项 | 影响 | 处理 |
|---|---|---|
| `quality_sigma_scale` 默认值为估计值 | 直接决定定权效果 | §9 实测标定后修订；轮 1 必做 |
| IMU-GNSS 杆臂未标定 [v2] | 转弯时米级残差；配 cm 级 `sigma_floor` 时 Huber 会把转弯段固定解当外点 | 未标定期间 `sigma_floor`=1.0 m；§9.3 工具用建图数据估计杆臂后填入并降 floor |
| RTK 与 LiDAR 时间戳不同源 [v2] | 几十 ms 延迟 = 10–25 cm 位置误差，与分层指标同量级 | `RtkFix.gnss_time` + `stamp_source`/`time_offset`；§9.3 工具估计 `time_offset` |
| `.pos` 时间系统 GPST≠UTC [v2] | 轨迹配对全部落空 | `pos_io` 识别头部时间系统并统一到 UTC |
| 云端/CI 的 GTSAM 为 4.2，Orin 为 4.3 [v2] | 接口差异 | `gnss_core` 只用两版共有接口（`NoiseModelFactorN`、`OptionalMatrixType`），CMake 不锁 4.3 |
| GeographicLib 的 rosdep key 未验证 | 构建依赖声明 | 轮 1 首个任务确认（预期为 `geographiclib`） |
| `gnss_diag` 在 `glim_ext` 中的目录归类 | 仅影响路径 | 暂置于 `modules/mapping/`（GNSS 约束属 mapping 侧） |
| rosbag2 无保留天数/水位清理 | 长期无人值守磁盘占满 | 轮 3 的清理逻辑（A6） |
| 平台差分协议、板卡原始格式未确认（P0） | `rtcm_bridge` 与 `rtkrcv_node` 的输入解析 | 沿用前置文档的现场核对清单；轮 2 前必须定死 |
| 踏歌现场无同步 LiDAR+RTK 录包 | 阻塞 §12.4 | §9 与 §12.3 使实车前的验证不被阻塞 |
