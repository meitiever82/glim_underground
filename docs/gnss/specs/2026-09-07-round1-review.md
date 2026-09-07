# 轮 1 spec / plan 评审（2026-09-07）

评审对象：
- `docs/gnss/specs/2026-09-03-gnss-glim-modules-design.md`
- `docs/gnss/plans/2026-09-03-round1-rtk-global.md`

对照物：`glim_ext/modules/mapping/gnss_global`、`glim_ext/CMakeLists.txt`、`glim/mapping/callbacks.hpp`、`glim/odometry/callbacks.hpp`、`glim/util/extension_module_ros2.hpp`、`glim/mapping/sub_map.hpp`、`config/casbot/config_ros.json`。

结论先行：**架构方向是对的**（core 与壳分离、质量分档定权、鲁棒核、杆臂因子、`.pos` 做对比与标定），可以进入实施。但 spec 与 plan 之间有几处不一致，plan 里有 4 处会直接卡住实施的工程错误，以及 3 处与项目背景（时间同步、外参未标定、9 月分层指标）直接相关的缺口，建议在派工前改掉。

---

## A. 必须改（否则实施会卡住或做错）

### A1. glim_ext 不是"每个模块一个 colcon 包"
`glim_ext` 是**一个** ament 包，所有模块通过顶层 `CMakeLists.txt` 的 `option(ENABLE_xxx)` + `add_subdirectory(modules/...)` 编进来，装到 `install/glim_ext/lib/`。plan Task 9 的 `colcon build --packages-select rtk_global` 与 `install/rtk_global/lib/librtk_global.so` 都不成立。

改法：
- `glim_ext/CMakeLists.txt` 加 `option(ENABLE_RTK_GLOBAL ... ON)`，`add_subdirectory(modules/mapping/rtk_global)`，`list(APPEND glim_ext_LIBRARIES rtk_global)`。
- `glim_ext/package.xml` 加 `<depend>gnss_core</depend>` `<depend>gnss_msgs</depend>`（`gnss_global` 的 `ament_auto_find_build_dependencies()` 就是从这里找依赖）。
- 构建命令：`colcon build --packages-select glim_ext`；产物在 `install/glim_ext/lib/librtk_global.so`。

### A2. 包放哪个 workspace
plan 把 `gnss_msgs`/`gnss_core` 放 `driver_ws`，`rtk_global` 放 `glim_ws`，则 `glim_ws` 必须 overlay `driver_ws`，而 `gnss_CGI610` 又要用 `gnss_msgs`。两边互相 source 很容易变成循环。

建议：`gnss_msgs` 与 `gnss_core` 都放 **`glim_ws/src/`**（`rslidar_msg` 已在此，同惯例）。`driver_ws` 只需 source `glim_ws/install` 就能拿到 `gnss_msgs`；`gnss_core` 驱动侧不需要。

### A3. Task 9 的 ctypes 加载验证不可行
`create_extension_module()` 会构造模块：读 `GlobalConfigExt::get_config_path`（依赖 GLIM 已初始化的全局 config）、注册 callback、起后台线程。脱离 GLIM 进程用 ctypes 调它，大概率抛异常或挂起，而且验证不了任何东西。

改法：用一段短 bag 跑 `glim_rosbag`，`config_ros.json` 加 `librtk_global.so`，断言日志出现 `initializing rtk_global` 与 `T_world_enu=`；这就是 spec §12.2 真正要的。

### A4. `.pos` 的时间系统是 GPST，不是 UTC
RTKLIB `.pos` 默认时间列是 **GPST**（比 UTC 快 18 s）。Task 10 按 UTC 解析、Task 11 用 0.1 s 容差配对，与 ROS 时间戳（UTC）导出的另一条轨迹一配就全部落空。

改法：`read_pos` 读头部 `% (time=GPST)` / `(time=UTC)`，GPST 减闰秒（可配置，当前 18）；测试加一条"GPST 头 → 减 18 s"用例。

---

## B. spec 与 plan 不一致（二选一定死）

| 项 | spec | plan | 建议 |
|---|---|---|---|
| 模块数 | §7/§10：`rtk_odometry` + `rtk_global` 轮 1 都做，"缺一不可" | 只做 `rtk_global` | **plan 对**。先把 submap 级跑通并用真数据验证，再做帧级；spec §10 改成"轮 1 = rtk_global，轮 1.5 = rtk_odometry" |
| `T_world_enu` | §6.1/§7.甲：bootstrap 后**持续重估**、不冻结 | Task 6：解一次后固定 | **plan 对，且 spec 的"持续重估"有逻辑问题**（见 C1） |
| `AntennaPriorFactor` 参数 | §7.3：`body_point`，global 侧传 `T_origin_frame(t)·lever_imu` | Task 7：`lever_imu` | plan 在 `origin_frame()->stamp` 处取 fix，`T_origin_frame = I`，两者此时等价。把参数名改成 `body_point`，注释说明 global 侧因取原点帧时刻所以等于 `lever_imu` 即可 |
| 合成注入测试 | §12.3 四种故障注入 | 无对应 Task | **必须补**（见 C4），这是没有实车数据时唯一能证明"比 gnss_global 好"的手段 |

---

## C. 设计层面的问题与建议

### C1. `T_world_enu` 持续重估是一个反馈回路
一旦 RTK 因子进图，优化后的 node 位置已经被 `T_world_enu × ENU` 拉过去了，再用这些位置反推 `T_world_enu`，等于用结果验证前提。更实际的问题：旧因子的 `p_world` 是用旧 gauge 算的，新因子用新 gauge，图里同时存在两套基准。轮 1 冻结（同 `gnss_global`）；真要联合优化走 §7.乙。

### C2. 时间戳来源必须写进 spec（与项目背景直接相关）
项目说明里已经指出现有采集数据时间同步有大问题。`RtkFix.header.stamp` 若是 ROS 接收时刻（串口/CAN + 非 PTP 主机），几十 ms 的延迟在 5 m/s 下就是 10–25 cm，与 9 月 0.25 m 的分层指标同量级。

建议：
- `RtkFix` 加 `float64 gnss_time`（板卡自带的 GPS/UTC 时间，GPCHC 里有周/周内秒），`header.stamp` 保持接收时刻；两者之差即可在线诊断偏移。
- 模块 config 加 `stamp_source: "header" | "gnss_time"` 与 `time_offset`（秒）。
- `RtkFixBuffer` 测试加一条"给定 time_offset 后插值点随之平移"。

### C3. 杆臂为零 + `sigma_floor = 2 cm` 会让 Huber 在转弯时把好的固定解当外点
项目背景是 RTK–雷达外参未标定。矿卡天线到 IMU 通常 1–3 m，杆臂不填时转弯处残差可达米级，远超 2 cm σ 与 Huber 阈值（1.345σ），于是**转弯段的固定解被系统性降权**，直线段才起作用——恰好和期望相反。

建议：
- spec §13 风险项改为"杆臂未标定期间，`sigma_floor` 应设为杆臂量级（例如 1.0 m），标定后再降到 cm 级"，并把这一句写进 `config_rtk_global.json` 注释。
- 轮 1 增加一个离线小工具 `estimate_lever_arm`：输入 GLIM 轨迹（`T_world_imu(t)`）与 ENU 轨迹，联合最小二乘解 `T_world_enu` + `lever_imu`（对每个 t：`R_world_imu(t)·lever + t_world_imu(t) = T_world_enu·enu(t)`，对 lever 是线性的）。这正好补上"RTK–雷达外参"这个缺口，而且所需数据与 §9 一样。

### C4. 补合成注入测试（spec §12.3）作为 Task 13
工具 `synth_rtk_fix`：读 GLIM 输出轨迹 → 反算 lat/lon/alt（`LocalCartesian::Reverse`）+ 已知噪声 + 质量标签 → 写成 bag 里的 `RtkFix` 话题（或 `.pos` 后经脚本转 bag）。四种注入（错误固定 5%、diff_age 增长段、全浮动段、非零杆臂转弯）各一个用例，验收比较开/关鲁棒核、有/无杆臂的轨迹 RMSE。没有这个，轮 1 的"比 gnss_global 好"只是断言。

### C5. `on_insert_submap` 里不要保存 `SubMap` 指针到后台线程
`callbacks.hpp` 明确写了 `submap->T_world_origin` 在 global mapping 线程更新、跨线程读不安全。`gnss_global` 也犯了这个错。改为在 callback 里（该线程内）把 `id`、`origin_frame()->stamp`、`T_world_origin.translation()` 拷进一个 POD 结构再入队。

### C6. 订阅 QoS
`TopicSubscription` 固定 `create_subscription<Msg>(topic, 100, ...)`（默认 reliable）。若 `gnss_CGI610` 的 `~/rtk_fix` 用 `sensor_data`（best_effort）发布，订阅匹配不上、静默收不到。Task 8 里把 publisher QoS 定为 reliable，或在 Task 9 的 bag 验证里断言收到消息数 > 0。

### C7. `trajectory_compare` 的 `sigma_ratio_h` 用均值不稳
"实际误差 / 板卡 σ" 的均值会被少数外点主导（一次错误固定 σ=3 mm、误差 1 m → 比值 300）。改为 **中位数**，或 `RMS(误差)/RMS(σ)`，两者都输出。这个数直接进 `quality_sigma_scale`，稳健性很重要。

### C8. `RtkFixBuffer::prune` 的 `now`
后台线程没有壁钟意义上的 now，用"最新 fix 的 stamp"做 now，horizon 60 s 是相对于数据流的。写进接口注释，避免壳里传 `ros::now`。

### C9. 小问题
- `pos_sigma_enu_m` 在 610 驱动里到底是 ENU 顺序还是 NED 顺序（板卡协议常给 σN σE σU），Task 8 要核一下再映射。
- `heading_valid` 的判定与 §4.1 表一致，但 `COMBINED_DR` 归 SINGLE 且 heading_valid=true 不妥（DR 时航向由惯导给出，可信度另论）；建议 DR 时 `heading_valid=false`。
- `NoisePolicyConfig::quality_sigma_scale.at(q)`：`q` 来自 `uint8`，`min_quality` 是 int，OK；但 config 里 `min_quality` 若被写成 5 以上会全部拒绝且无日志，加一条启动时校验。
- 云端/CI 可用的 GTSAM 是 4.2（apt），Orin 上是 4.3；`NoiseModelFactorN` 与 `OptionalMatrixType` 两个版本都有，`gnss_core` 可以在 4.2 上单测、4.3 上部署，但 CMake 里别写死 4.3。

---

## D. 与 9 月目标的关系（提醒，不阻塞本轮）

9 月目标是"采集车建图，点云分层 80% < 0.25 m"。本 spec 解决的是"地图整体钉在 ENU 上不漂"，对分层指标的贡献是间接的（长程一致性、回环处不错位）。分层本身更多取决于：时间同步（C2）、LiDAR–IMU 外参、去畸变、多雷达外参。建议把 9 月目标至少拆成 4 个 feature 并行推进：① 本 spec（RTK 全局约束）；② 时间同步与采集（PTP/PPS、`gnss_time`、录包规范）；③ 分层指标工具（定义"分层"的可计算口径——例如按重复路段地面点的高度分布 / 平面拟合残差，80% 分位 < 0.25 m——并与客户对齐）；④ 外参标定（LiDAR–IMU、杆臂 C3、多雷达）。③ 应尽早做，否则没法量化①②④各自的收益。

---

## E. 执行方式的现实约束

- 云端沙箱没有 GLIM/ROS2，但可以装 Eigen、GTSAM 4.2、GeographicLib、gtest。**`gnss_core` 的 Task 2–7、10–12 以及 C3/C4 的离线工具可以在云端完整 TDD**，再回写到 `glim_ws/src/gnss_core`（同时给 ament 与纯 CMake 两套构建入口）。
- `gnss_msgs`、`rtk_global` 壳、`glim_ext` 集成、驱动改动，agent 只能写代码，**构建与 bag 验证要在 Orin 上跑**（本会话没有在你机器上执行命令的通道）。
- `driver_ws` 未连接到本会话；Task 8（`gnss_CGI610` 增发）需要把它加进来，或者先跳过、由你本地完成。
