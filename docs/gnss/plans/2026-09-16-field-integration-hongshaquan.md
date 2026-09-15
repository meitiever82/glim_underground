# 真数据联调：红沙泉 2026-09-15 seg_164931_165748 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 用维护者 2026-09-15 在红沙泉现场录的一段约 8 分钟数据，按接近实车的方式回放，第一次端到端检验 GNSS 数据面。回放的输入与链路如下：
- **输入**：平台差分 RTCM3 走 TCP、610 原始观测走 TCP、610 CAN 帧走 vcan。
- **链路**：`rtcm_bridge` → `rtkrcv_node` → `gnss_chcnav` 驱动 → `pos_writer` / `gnss_diag_node` / rosbag2 录制。

**比对与产出：** 把输出与现场后处理基准比对，产出一份结果文档：每项做了什么、数值结果、发现的问题与处理。

**Architecture:**
- **数据集只读**，所有产物放 `<seg>/integration_20260916/`。
- **回放工具进 `glim_ext` 分支，以后可复用**：新写的工具放在 `gnss_bringup/tools/field_replay/`。
  - Python 编排器负责三件事：按数据时间把两路字节流通过 TCP 喂给 `rtcm_bridge`（`rtcm_bridge` 按 yaml 默认主动连 127.0.0.1:15031/15032）；按同一时间基准启动 `canplayer`；发布 `/clock`。
  - 另有一个 C 小工具，把 RINEX 星历编码成 RTCM3 星历电文。
- **跑两遍**：A 按现场原样；B 在差分流里补星历。
- **问题处理**：联调中发现的产品代码问题，在同一分支上带测试修复。

**Tech Stack:** ROS 2 Humble、Python 3.10（rclpy，unittest）、C（RTKLIB-EX 2.5.1 `librtklib.so`）、can-utils（`canplayer`）、vcan、RTKLIB `rnx2rtkp`/`convbin`。

**Spec / 依据:**
- `docs/gnss/specs/2026-09-03-gnss-glim-modules-design.md` §5（数据层）、§8、§12.4（实车对比）、§13 P0（平台差分与板卡格式待确认项）
- 已合并实现：`glim_ext/gnss_bringup`（rtcm_bridge / rtkrcv_node / pos_writer / gnss_diag_node / gnss_cleanup_node / record_gnss.sh）、`finder_ros/drivers/gnss_CGI610`（包名 `gnss_chcnav`，节点 `gnss_chcnav_can`，发布 `/gnss_cgi610/rtk_fix`）
- 数据段：`/home/steve/Documents/Datasets/tage/hongshaquan/20260915/seg_164931_165748/`（下称 `<seg>`）

## 数据段清单（写作时核对）

| 文件 | 内容 |
|---|---|
| `<seg>/gnss/base.rtcm3` | 接收机实际收到的平台差分转发：1006 ×50、MSM4 1074/1084/1094/1114/1124 各 ~498 |
| `<seg>/raw/cgi610.dat` | 610 NovAtel 二进制（CRC 通过）：RAWIMUSB 100 Hz、INSPVAXB 10 Hz、RANGECMPB 1 Hz、GALEPHEMERISB 13、BDSEPHEMERISB 1；**没有 GPS 星历** |
| `<seg>/raw/can7.candump.log` | candump 格式（带 unix 时间戳），接口名 `can7`，ID 0x320–0x32E |
| `<seg>/gnss/rover.obs`、`base.obs`、`rover.nav` | RINEX；`rover.nav` 含 G 34 / E 214 / C 59 条星历，来自当天更早的 dat |
| `<seg>/gnss/rtk_check.pos` | `rnx2rtkp` EX 2.5.1 后处理：498 历元，Q1 486 / Q2 12；基准站参考坐标 44.519819020 90.259104320 615.0870；时间 GPST 2026/09/15 08:49:49–08:58:06 |
| `<seg>/bag/` | 35 GB rosbag2：两个激光雷达、IMU、`/cgi610/odom`、`/cgi610/fix`（NavSatFix，10 Hz）；**没有 gnss_msgs 话题** |
| `<seg>/gt/` | TUM 轨迹与 ENU 原点（原点 = 段内第一条 INSPVAXB，unix 1789462171.000） |

## Global Constraints

- **数据集只读**：不改、不删、不移动 `<seg>` 下任何已有文件；新产物只写 `<seg>/integration_20260916/`。
- **代码仓库** `/home/steve/glim_ws/src/glim_ext`：
  - 从最新 `master`（`git ls-remote origin master`，写作时 `dbd2986`）开 `feat/field-replay-20260915`。
  - **绝不 `git push`**；以 `git ls-remote` 为准（外部自动同步）；提交后 `git log --oneline -3` 确认。
- `glim_underground` 只提交本计划与结果文档两个明确路径；维护者未提交的 `time_keeper.*` 绝不暂存。
- `finder_ros` 不改源码（CAN 驱动按现状用）。如确有驱动缺陷，只写进结果文档，不在本计划内修。
- `driver_ws`：只允许 `colcon build --symlink-install --packages-select gnss_msgs gnss_chcnav`（维护者 2026-09-16 同意），不改源码、不动其他包。
- **不做需要 sudo 的操作**。vcan0 与 can-utils 已由维护者准备好（`ip -br link show vcan0` 为 UP）。
- ROS 进程一律 `ROS_DOMAIN_ID=66`、`ROS_LOG_DIR=<run>/roslog`，避免干扰机器上其他会话；跑完确认进程全部退出（`pgrep -f` 无残留 rtkrcv / 节点）。
- 产品代码修复带 gtest 与诚实 RED 证据（沿用前几轮规则）。回放工具的纯函数用 Python `unittest`，经 CMake `add_test` 注册；开发机 pytest 与 ROS 插件冲突，不用 pytest。
- 每个 commit message 以 `Co-Authored-By:` 行结尾，写实际执行提交的模型。
- 注释中文；标识符英文。

## 设计决定（控制者，2026-09-16，维护者已同意方向）

1. **610 CAN 走 vcan + 真实驱动**（维护者选择）：`canplayer -I <seg>/raw/can7.candump.log vcan0=can7`，驱动 `gnss_chcnav_can` 设 `can_device:=vcan0`、`timestamp_source:=gps`。
2. **两遍回放**（维护者选择）：
   - **A 原样**：差分流 = `base.rtcm3`，观测流 = `cgi610.dat`，`rtkrcv_node obs_format:=oem4`。
   - **B 补星历**：在 A 的差分流开头注入由 `rover.nav` 编码的 RTCM3 星历电文（GPS 1019、GLONASS 1020、BDS 1042、Galileo 1046），之后每 30 s 重发一次。
3. **时间基准**：
   - 各流按数据自带时间调度，1× 实时，共用一个 `t0`（三路里最早的数据时刻，UTC unix 秒）。
   - RTCM MSM 历元时间：GPS/Galileo 用 GPST 周内秒，BDS 为 BDT（= GPST − 14 s），GLONASS 为莫斯科时间日内秒。1006 等不带时间的帧沿用前一帧的时间。
   - NovAtel 取头部 GPS 周 + 毫秒。
   - CAN 用 candump 时间戳（unix）。
   - GPST → UTC 固定减 18 s。
   - 编排器以 50 Hz 发布 `/clock = t0 + (wall − W0)`。
4. **rtkrcv 不用 libfaketime**：数据（09-15）与回放日（09-16）同一 GPS 周（2436），rtkrcv 用系统时间补周数不会补错。若实测解不出且日志指向周数问题，再加 faketime，并在结果文档记录。
5. **诊断节点用 sim time**（`use_sim_time:=true`），`events.log` 落在数据日期 20260915；`rtkrcv_node`、`pos_writer`、驱动用墙钟（`.pos` 时间取 `gnss_time`，不受影响）。
6. **比对基准**：
   - `rtk_check.pos`（后处理，EX 2.5.1）作为 `ref`。
   - 位置差用已有的 `calibrate_sigma_scale` 与新写的评估脚本：水平差按 610 质量分档，给出中位数、95% 分位与最大值。
   - 610 融合解与天线解之间可能存在杆臂偏移，要**按航向分析**（spec 轮 3b 留下的实车确认项）。
7. **录包**：用 `record_gnss.sh`，`GNSS_BAG_ROOT=<run>/bags`。话题在默认六个之外加 `/gnss/diagnostics`、`/rtkrcv_node/diagnostics`、`/clock`。
8. **执行顺序**：先各部件单独打通（Task 1–3），再整链路跑 A、B（Task 4），最后评估写文档（Task 5）。任何一步失败先定位原因，能修则修（Task 6 汇总修复），修不了的写进结果文档，接着往下做，不因单点卡住整夜。

## File Structure

`glim_ext`（分支 `feat/field-replay-20260915`）
- Create `gnss_bringup/tools/field_replay/stream_timing.py` — RTCM3 / NovAtel 二进制分帧与帧时间（纯函数）
- Create `gnss_bringup/tools/field_replay/candump_log.py` — candump 行解析（纯函数）
- Create `gnss_bringup/tools/field_replay/field_replay.py` — 编排器：TCP 服务端 × 2、canplayer 调度、`/clock` 发布、可选星历注入
- Create `gnss_bringup/tools/field_replay/rnx_nav_to_rtcm.c` — RINEX 星历 → RTCM3 星历电文
- Create `gnss_bringup/tools/field_replay/field_eval.py` — 结果评估（读 `.pos` / `events.log` / `base.pos` / 录包，输出 Markdown 表）
- Create `gnss_bringup/tools/field_replay/run_field_integration.sh` — 一遍整链路运行脚本（起节点、回放、收尾）
- Create `gnss_bringup/tools/field_replay/README.md`
- Create `gnss_bringup/tools/field_replay/tests/test_stream_timing.py`、`tests/test_candump_log.py`、`tests/test_field_eval.py`
- Modify `gnss_bringup/CMakeLists.txt` — 注册 Python unittest（`add_test`）并安装脚本
- 产品代码修复（如有）：按问题所在文件，带 gtest

`glim_underground`
- Create `docs/gnss/field/2026-09-16-hongshaquan-seg164931-integration.md` — 结果文档（Task 5）

数据段产物（不进 git）：`<seg>/integration_20260916/{inventory.md, run_A/, run_B/, eval/, logs/}`

---

### Task 1: 基线核查与部件单独打通

**目标：** 在整链路之前把每个输入和部件单独验证，结果写进 `<seg>/integration_20260916/inventory.md`。

- [ ] **Step 1: 建目录与分支**

```bash
SEG=/home/steve/Documents/Datasets/tage/hongshaquan/20260915/seg_164931_165748
mkdir -p $SEG/integration_20260916/{run_A,run_B,eval,logs}
cd /home/steve/glim_ws/src/glim_ext && git ls-remote origin master && git status --short && git checkout -b feat/field-replay-20260915
```

- [ ] **Step 2: 重编 driver_ws 的 gnss_msgs 与 gnss_chcnav**

```bash
cd /home/steve/driver_ws && source /opt/ros/humble/setup.bash && colcon build --symlink-install --packages-select gnss_msgs gnss_chcnav 2>&1 | tail -5
```

记录是否成功。失败时记录报错，并退回离线方案（设计决定 1 的备选：用驱动的 `cgi610_decoder.hpp` 写一个离线解码发布器），在结果文档注明。

- [ ] **Step 3: CAN 驱动单独打通**

在隔离域里起驱动，用 `canplayer` 回放前 60 s，统计 `/gnss_cgi610/rtk_fix`：
- 消息数、平均频率；
- `quality` 分布；
- `gnss_time` 与 `header.stamp` 是否有效（`timestamp_source=gps` 时二者应同源）；
- 经纬度是否在 44.470 / 90.2946 附近。

`canplayer` 用法：`canplayer -I <log> vcan0=can7`（`-l i` 无限循环不要用）。只放 60 s 的办法：用 `head` 截取 candump 行到 `logs/can_60s.log`（按时间戳截），不改原文件。

- [ ] **Step 4: 离线核对 610 原始观测与差分**

- `convbin -r oem4 <seg>/raw/cgi610.dat -o $SEG/integration_20260916/logs/cgi610_convbin.obs -n .../cgi610_convbin.nav`：历元数应 ≈ 498，与 `rover.obs` 一致；导出的 nav 里 GPS 星历应为 0（证实"没有 GPS 星历"）。
- 用 `rnx2rtkp` 以 `cgi610_convbin.obs` + `base.obs` + `rover.nav` 复算一次，确认与 `rtk_check.pos` 固定率同量级。这一步证明 RTKLIB 能正确读 610 的 RANGECMPB。
- 用 `gnss_core` 的 RTCM 解析（`RtcmFramer` + `parse_base_station`，可写一个一次性 C++ 片段或 Python 按 DF 位解析）取 `base.rtcm3` 里 1006 的 ECEF，换算经纬高，与 `rtk_check.pos` 头部 `ref pos` 比较，记录差值（期望 < 0.01 m）。

- [ ] **Step 5: 写 inventory.md**

每步一行结论加关键数值；有异常就写异常与判断。不提交（数据段目录不进 git）。

---

### Task 2: 回放工具的纯函数（分帧、时间、candump 解析、星历编码）

**Files:** `gnss_bringup/tools/field_replay/{stream_timing.py, candump_log.py, rnx_nav_to_rtcm.c, tests/test_stream_timing.py, tests/test_candump_log.py}`、`gnss_bringup/CMakeLists.txt`

**Interfaces（Python）：**
- `stream_timing.iter_rtcm3_frames(data: bytes) -> Iterator[Rtcm3Frame]`，其中 `Rtcm3Frame(offset:int, raw:bytes, msg_type:int, crc_ok:bool)`。
  - 按前导 0xD3 与 10 位长度分帧，校验 CRC-24Q；CRC 不过时跳过 1 字节继续找同步。
- `stream_timing.rtcm3_epoch_gpst_tow(frame) -> Optional[float]`：MSM1–7（1071–1127）返回 GPST 周内秒（s）。
  - GPS/Galileo/QZSS 直接取 30 位 tow（ms）。
  - BDS 为 BDT tow + 14 s。
  - GLONASS 取 3 位星期 + 27 位日内毫秒，换成 GPST 周内秒：莫斯科时间 − 3 h + 18 s 闰秒。
  - 非 MSM 帧返回 `None`。
- `stream_timing.iter_novatel_frames(data: bytes) -> Iterator[NovatelFrame]`，其中 `NovatelFrame(offset, raw, msg_id, week, ms, crc_ok)`。
  - 按 `AA 44 12` 同步。头长取第 4 字节，消息长取 8–9 字节（小端），`msg_id` 取 4–5 字节，周取 14–15 字节，毫秒取 16–19 字节；帧尾 CRC-32 为 NovAtel 多项式 0xEDB88320。
  - 同时支持 short header 同步 `AA 44 13`（RAWIMUSB 等用 short header：头长固定 12，第 3 字节为消息长度，周取 4–5 字节，毫秒取 6–9 字节）。
- `stream_timing.gpst_to_unix_utc(week:int, tow_s:float, leap_s:int=18) -> float`
- `stream_timing.schedule(frames_with_time) -> list[(unix_utc:float, bytes)]`：无时间的帧沿用前一个有时间的帧；文件开头的无时间帧沿用第一个有时间的帧。
- `candump_log.parse_line(line:str) -> Optional[CanRecord]`，其中 `CanRecord(t:float, iface:str, can_id:int, data:bytes)`，解析 `(1789462171.002222) can7 320#8409DCC7310C0000`；`first_timestamp(path) -> float`。

**`rnx_nav_to_rtcm.c`：** `rnx_nav_to_rtcm <rover.nav> <out.rtcm3>`。
- 用 `readrnx` 读星历；对每个卫星的每条星历设置 `rtcm.ephsat`（GLONASS 用 `geph`）后调用 `gen_rtcm3`：GPS→1019，GLONASS→1020，BDS→1042，Galileo→1046。
- 打印各类型条数。
- 构建参考 `test/data/rnx2rtcm.c` 与 round2 hardening 计划 Task 6 的编译命令：`-I` 指向 RTKLIB 源码树 `src/`，链接 `/usr/local/lib/librtklib.so`。**不进 CMake**，由 `run_field_integration.sh` 在运行时编译到 `<seg>/integration_20260916/logs/`，README 写明。

- [ ] **Step 1: 写失败测试**（unittest）
  - `test_stream_timing.py`：
    - 构造一个合法 RTCM3 帧（1074 负载，tow = 204589000 ms），断言分帧、`msg_type`、`crc_ok`、epoch = 204589.0。
    - 在帧前插垃圾字节，仍能找到帧；改一个字节，`crc_ok` 为 False 且不产出该帧。
    - BDS 1124 负载 tow = 204575000 → 204589.0。
    - GLONASS 1084 构造一例：星期 2（周二）、日内 ms = (08:49:49 − 18 s + 3 h) × 1000 → 期望 GPST tow 204589.0。
    - NovAtel 长头帧与短头帧各构造一例，断言周、毫秒、`msg_id`、`crc_ok`。
    - `gpst_to_unix_utc(2436, 204589.0)` = 1789462171.0（= 2026-09-15 08:49:31 UTC）。
    - `schedule` 对无时间帧的继承规则。
    - 真实文件计数（文件存在时才跑，否则 `skipTest`）：`base.rtcm3` 里 1006 = 50、1074 = 498；`cgi610.dat` 里 RANGECMPB（id 140）= 498、INSPVAXB（id 1465）= 4974，CRC 全部通过。
  - `test_candump_log.py`：正常行、空行、坏行（返回 None）、扩展帧 ID（8 位十六进制）、真实文件首行时间戳。
- [ ] **Step 2: 确认 RED**：模块只写函数签名、抛 `NotImplementedError`，跑 `python3 -m unittest discover -s gnss_bringup/tools/field_replay/tests -v`，全部失败。
- [ ] **Step 3: 实现**，GREEN。
- [ ] **Step 4: 注册测试**：`gnss_bringup/CMakeLists.txt` 的 `BUILD_TESTING` 块加

```cmake
  # 现场回放工具的纯函数(unittest,不用 pytest:开发机 pytest 与 launch_testing 插件冲突)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_test(NAME test_field_replay_py
    COMMAND ${Python3_EXECUTABLE} -m unittest discover -s ${CMAKE_CURRENT_SOURCE_DIR}/tools/field_replay/tests -v)
```

  `install(PROGRAMS tools/field_replay/field_replay.py tools/field_replay/run_field_integration.sh DESTINATION lib/${PROJECT_NAME})` 在 Task 3 加。
- [ ] **Step 5: 编译 `rnx_nav_to_rtcm.c`，对 `rover.nav` 跑一次**。记录各类型条数，并用 `stream_timing.iter_rtcm3_frames` 回读确认 CRC 全过、类型计数一致。
- [ ] **Step 6: 全量测试 0 failures，提交**（`tools/field_replay/*.py`、`tests/*`、`.c`、`CMakeLists.txt`）。

---

### Task 3: 回放编排器与运行脚本

**Files:** `gnss_bringup/tools/field_replay/{field_replay.py, run_field_integration.sh, README.md}`、`gnss_bringup/CMakeLists.txt`（install）

**`field_replay.py` 行为：**
- **参数**：`--rtcm <path> --obs <path> --can-log <path> [--nav-rtcm <path>] [--corr-port 15031] [--obs-port 15032] [--can-iface vcan0] [--can-log-iface can7] [--speed 1.0] [--connect-timeout 60] [--leap 18]`。
- **准备**：读两个二进制文件，用 Task 2 的纯函数生成调度表；`t0` = min（差分首帧时刻，观测首帧时刻，CAN 首帧时刻）。
- **等待连接**：在 127.0.0.1 两个端口各 `listen`，等 `rtcm_bridge` 连上。超时退出码 3，打印"rtcm_bridge 未连接"。
- **开始**：两个端口都连上后，记 `W0 = monotonic()`。
  - 起一个 rclpy 节点 `field_replay`，以 50 Hz 发布 `/clock`（`rosgraph_msgs/Clock`，`ClockQoS`）。
  - 在 `W0 + (can_t0 − t0)/speed` 时刻启动 `canplayer -I <can-log> <can-iface>=<can-log-iface>`。
  - 两路 TCP 按 `W0 + (frame_t − t0)/speed` 发送。`--nav-rtcm` 给出时，在差分流的第一帧之前先发一遍全部星历电文，之后每 30 s 数据时间重发一次。
- **结束**：全部发完、canplayer 退出后，再保持 `/clock` 走 10 s（让诊断节点关事件），然后退出码 0。
- **日志**：每 10 s 打印一行进度（数据时刻、已发帧数、两个连接状态）。连接断开时打印并尝试等待重连 30 s；等不到就退出码 4。
- **信号**：SIGINT 时杀掉 canplayer、关闭套接字，退出码 130。

**`run_field_integration.sh <A|B> <seg>`：**
1. **准备运行目录**：`RUN=$SEG/integration_20260916/run_$1`，清空其中上次产物（只清这个目录）；环境设 `ROS_DOMAIN_ID=66`、`ROS_LOG_DIR=$RUN/roslog`；source `/opt/ros/humble`、`glim_ws/install`、`driver_ws/install`（后者提供 `gnss_chcnav`）。
2. **按需编译星历工具**：B 需要且缺少时编译 `rnx_nav_to_rtcm` 并生成 `$RUN/nav.rtcm3`。
3. **写 params 覆盖**：写 `$RUN/params.yaml`（由 `gnss_bringup.yaml` 复制后改）：
   - `rtkrcv_node.obs_format: "oem4"`，`run_dir: $RUN/rtkrcv`；
   - `pos_writer.root: $RUN/pos`，`sources: ["can", "rtkrcv"]`，取消 `rtkrcv.topic` 注释；
   - `gnss_diag.root: $RUN/diag`、`startup_grace_s: 60.0`；
   - `gnss_cleanup` 不起。
4. **后台起全部节点**：
   - `ros2 launch gnss_bringup gnss_bringup.launch.py params_file:=$RUN/params.yaml enable_rtkrcv:=true enable_cleanup:=false`；
   - 诊断节点需要 sim time：launch 不转发，改为 `enable_diag:=false` 另起 `ros2 run gnss_bringup gnss_diag_node --ros-args --params-file $RUN/params.yaml -p use_sim_time:=true -p solver_enabled:=true`；
   - `ros2 run gnss_chcnav gnss_chcnav_can --ros-args -r __node:=gnss_cgi610 -p can_device:=vcan0 -p timestamp_source:=gps`；
   - `GNSS_BAG_ROOT=$RUN/bags GNSS_BAG_TOPICS="<默认六个> /gnss/diagnostics /rtkrcv_node/diagnostics /clock" record_gnss.sh`。
   - 各自 stdout/stderr 进 `$RUN/logs/*.log`。
5. **回放**：等 5 s 后前台运行 `field_replay.py`（B 带 `--nav-rtcm`）。
6. **收尾**：回放结束后依次 SIGINT 录包、诊断节点、驱动、launch，每个最多等 20 s，超时 SIGKILL 并记录；最后 `pgrep -af "rtkrcv|gnss_bringup|gnss_chcnav|ros2 bag"`，确认无残留。
7. **汇总**：打印 `$RUN` 下产物清单与各日志尾部。

- [ ] **Step 1: 实现 `field_replay.py`**。**试跑**：只起 `rtcm_bridge` 和 `ros2 topic hz /gnss/rtcm_corrections`，加 `--speed 10` 放 30 s，确认两个话题有数据、`/clock` 在走，进度日志正常。
- [ ] **Step 2: 实现 `run_field_integration.sh` 与 README**。README 写清前置条件（vcan0、can-utils、driver_ws 已编译）、A/B 含义、产物目录结构。
- [ ] **Step 3: CMake 安装脚本，构建，全量测试 0 failures，提交**。

---

### Task 4: 整链路运行 A 与 B

- [ ] **Step 1: 跑 A**：`run_field_integration.sh A <seg>`，完整 1× 约 9 分钟。结束后检查：
  - `run_A/pos/20260915/{can.pos,rtkrcv.pos}` 行数；
  - `run_A/diag/20260915/{events.log,base.pos}` 与 `diag/base_baseline`；
  - `run_A/bags/` 下有 bag，且 `ros2 bag info` 话题计数非零；
  - `rtkrcv` 日志里有无解算输出、有无崩溃循环。
- [ ] **Step 2: A 若 rtkrcv 一条解都没有**，按顺序排查并记录：
  1. `rtkrcv_node` 健康话题的消息（上行无数据 / 有数据无解）；
  2. `run_A/rtkrcv/rtkrcv_*.stat` 与 trace，看观测是否解码出来、有无星历；
  3. 与 Task 1 Step 4 的离线结论对照。

  预期原因是缺 GPS 星历、只剩 GAL/BDS。确认后作为发现写入，不算失败。
- [ ] **Step 3: 跑 B**，检查同 Step 1。
- [ ] **Step 4: 链路问题的定位与修复**：任何一遍出现节点崩溃、话题无数据、文件没写出等链路问题，先按日志定位，再判断：
  - **本仓库产品代码的缺陷**：进 Task 6 修复后重跑该遍；
  - **配置或数据问题**：记录，调整 `params.yaml` 后重跑，并在结果文档写明做了什么调整。
- [ ] **Step 5**：每遍都把 `ros2 bag info`、各节点日志的错误与警告摘要写进 `$RUN/summary.md`。

---

### Task 5: 评估与结果文档

**Files:** `gnss_bringup/tools/field_replay/{field_eval.py, tests/test_field_eval.py}`、`glim_underground/docs/gnss/field/2026-09-16-hongshaquan-seg164931-integration.md`

**`field_eval.py --run <run_dir> --ref <rtk_check.pos> [--bag <run_dir>/bags/...] --out <md>`，输出 Markdown：**
1. **各源概况**：每源（`can`、`rtkrcv`、`ref`）的历元数、时间覆盖、Q 分布与固定率。
2. **`rtkrcv` 对 `ref`**：按时间配对（容差 0.1 s）；对双方都 FIXED 的历元，统计水平差与高程差的中位数、95% 分位、最大值；另统计 rtkrcv 首次固定用时（相对第一条解）。
3. **`can` 对 `ref`**：配对口径同第 2 项，水平差按 can 质量分档。
   - 杆臂分析：从录包 `/gnss_cgi610/odom`（或 `rtk_fix.heading`）取航向，把水平差分解到车体前/右方向，报告均值与标准差；前/右均值远离 0、且标准差小，就提示存在固定杆臂偏移。
4. **基站坐标**：`diag/base_baseline` 与 `base.pos` 各条对 `ref pos`（`rtk_check.pos` 头部）的 ECEF 距离。
5. **事件**：`events.log` 按代码汇总次数与时长，列出全部事件行；对每个事件用 ref 与 rtkrcv 的实际情况给一句"合理 / 可疑"的判断依据（例如 `device_divergence` 开启时 can 与 ref 的实际偏差）。
6. **诊断话题**：`/gnss/diagnostics` 的状态码分布（来自录包）；`/rtkrcv_node/diagnostics` 的级别分布。

- [ ] **Step 1**：为 `field_eval.py` 里的配对、分位数、车体系分解写 unittest（合成数据，先 RED 再 GREEN），注册进同一个 `add_test`（在同一 tests 目录，自动发现）。
- [ ] **Step 2**：对 A、B 各跑一次，输出到 `<seg>/integration_20260916/eval/{A,B}.md`。
- [ ] **Step 3: 写结果文档**（中文）。结构：
  1. 结论摘要（5–8 条，先说最重要的）；
  2. 数据段与回放方式；
  3. 部件打通情况（Task 1）；
  4. A/B 结果对比表；
  5. 610 融合解与 RTK 的偏差及杆臂判断；
  6. 诊断事件是否可信；
  7. 发现的问题与处理（已修复：提交号；未修复：原因与建议）；
  8. 对 spec §13 P0 待确认项的新认识（差分格式、板卡输出配置缺 GPS 星历等）；
  9. 下一步建议；
  10. 复现方法（命令）。

  文档里所有数值都要可追溯到 `eval/*.md` 或日志。
- [ ] **Step 4**：`field_eval.py` 与测试提交到 `glim_ext` 分支。结果文档提交到 `glim_underground`，只 `git add` 这一个路径。

---

### Task 6: 修复联调中发现的产品代码问题（按需）

每个问题一个提交，要求如下：
- 先写能复现问题的 gtest（或回放工具级 unittest），看到 FAIL；
- 再修，看到 PASS；
- 跑全量测试；
- 提交信息写清现象、原因、证据。

修完重跑受影响的那一遍（A 或 B），结果文档引用重跑后的数据，并注明重跑原因。

不在本计划内修的范围：
- `finder_ros` 驱动缺陷；
- 需要改 spec 设计决定的问题（例如杆臂导致 `device_divergence` 常开）。

这两类只在结果文档里给出分析与建议。

---

## 完成标准

- `<seg>/integration_20260916/` 下有 `inventory.md`、`run_A/`、`run_B/`（含 `.pos`、`diag/`、`bags/`、`logs/`、`summary.md`）、`eval/A.md`、`eval/B.md`。
- `glim_ext` 分支 `feat/field-replay-20260915` 上的工具与修复提交齐全，全量测试 0 failures，未 push。
- `glim_underground` 上结果文档已提交。
- 机器上无残留回放相关进程；数据集原文件未变（`find <seg> -newer <plan 提交时间> -not -path '*integration_20260916*'` 为空）。
