# GNSS Web HMI 开发任务拆分（P0–P2）Implementation Plan

> **For agentic workers:** 按任务顺序执行，每个任务以“验收命令全绿 + 独立 commit”为完成标准；不要跳任务，不要顺带重构无关代码。

**Goal:** 把 `rtk-monitor` 改造成 `gnss_bringup/hmi/` 的只读 Web HMI，消费 `gnss_bringup` 的 ROS 话题，支持远程浏览器查看、双解常显、离线底图与回放。

**Architecture:** 保留 `rtk-monitor` 的 FastAPI + WebSocket + 静态前端；移除自采集（TCP/CAN/solver/`_stat_reader`）与本地诊断规则，新增 `rtkfix/stat/diagnostic/rtcm_rate` 四类 ROS 订阅和双源状态聚合。HMI 只读，不发布、不调用 ROS 服务。

**Tech Stack:** Python 3.10+、FastAPI、uvicorn、rclpy、SQLite；前端 Vue3/Leaflet/uPlot（已 vendored，无 npm 构建）；测试 pytest + node `--test`。

**Spec:** `docs/gnss/specs/2026-11-02-gnss-web-hmi.md`

---

## Global Constraints

- 代码落点：`glim_ext/gnss_bringup/hmi/`（`glim_ws/src/gnss_bringup` 为 symlink）。该目录**不注册 `package.xml`**，不进 colcon。
- 基线：`/home/boonraygrn/Documents/GitHub/gnss-alg/rtk-monitor`。复制后只做改名与功能改造，不改前端视觉体系。
- **只读**：HMI 不创建 publisher、不调用 service/action、不启动/停止任何 ROS 节点或 `rtkrcv`。
- **单一数据源**：诊断以 `/gnss/diagnostics` 为准，本地规则默认关闭；不得双跑 `rtkrcv`/TCP/CAN。
- 不触碰 `glim_underground` 中维护者未提交的 `include/glim/util/time_keeper.hpp`、`src/glim/util/time_keeper.cpp`。
- 不擅自 `git push`；每个任务单独提交，`git add` 只加本任务文件。
- 现有 pytest 与 `tests_js/*.mjs` 必须保持全绿；新测试先看到失败再实现。
- 回放测试一律 `use_sim_time` + 临时数据目录，不写生产 `root`。

## 前置检查（开工前一次）

- [ ] `ros2 topic list` 确认：`/rtkrcv_node/rtk_fix`、`/gnss_cgi610/rtk_fix`、`/rtkrcv_node/stat`、`/gnss/diagnostics`、`/rtkrcv_node/diagnostics`、`/gnss/rtcm_corrections`、`/gnss/raw_obs` 存在。
- [ ] `ros2 topic info -v <topic>` 记录每个 `RtkFix` 发布端的 QoS（尤其 610 是否 `best_effort`）。
- [ ] `ros2 topic hz /rtkrcv_node/stat` 确认 `$SAT` 流真实存在；不存在则天空图在 P0 只能显示“等待数据”。
- [ ] HMI 运行环境 source ROS overlay 后能 `python -c "import rclpy, gnss_msgs"`。
- [ ] 8080 端口空闲；旧 `rtk-monitor` 已停（避免双诊断、双解算、端口冲突）。
- [ ] Esri 在线底图的归属标注与使用条款已确认。

---

## P0：页面跑通（只读、无存储、无自采集）

### Task 0：基线落位与改名

- [ ] 复制 `rtk-monitor` 到 `gnss_bringup/hmi/`，保留 `web/`、`tests/`、`tests_js/`、`docs/`。
- [ ] Python 包 `rtk_monitor` → `gnss_hmi`，更新 imports、入口、`pyproject.toml`/脚本与测试引用；**不改任何行为**。
- [ ] 删掉仅属于旧部署的产物（旧 `run.log`、缓存、SQLite 样例），保留代码与测试。
- [ ] `hmi/README.md` 写明基线来源、只读约束、运行前置（source ROS overlay）。

**验收:** `pytest -q` 与 `python test_js.py` 全绿；`python -m gnss_hmi.main --help` 可执行。

### Task 1：配置开关与运行模式

- [ ] `config.py`：`RosTopicCfg` 增加 `kind`（gga/navsat/bestpos/rtkfix/stat/diagnostic/rtcm_rate）、`qos`（auto/reliable/best_effort）、`source`（app/fused）；`StreamCfg`/CAN 增加 `enabled`；新增 `storage.enabled` 与 `diagnosis.local_fallback`。
- [ ] `main.py`：按 `enabled` 跳过 TCP/CAN、`solver`、`_stat_reader`、SQLite 写入；启动日志明确打印“只读模式 / 已禁用采集”。
- [ ] 新增 `config/gnss_hmi.yaml`：只启用 ROS 订阅；端口 8080；底图源/缓存目录；日志路径。
- [ ] 旧配置兼容：缺省 `enabled=false`，必须显式开启才起 TCP/CAN。

**验收:** 用新配置启动，`ss -tlnp` 无 TCP 客户端连接，无 CAN 线程，无 `rtkrcv` 子进程；配置解析单测通过。

### Task 2：`RtkFix` 双源订阅与映射

- [ ] `ros_collector.py` 新增 `rtkfix`：订阅 `/rtkrcv_node/rtk_fix`（source=app）与 `/gnss_cgi610/rtk_fix`（source=fused）。
- [ ] 映射到 `Epoch`：`quality` 0–4、`sigma_enu[0,1,2]`（E/N/U）、`diff_age`、`sats_used`、`ratio`（0→None）、`gnss_time`（0→None）、`raw_status`、`heading` 有效性。
- [ ] QoS 按配置创建；订阅后 N 秒无消息记录 WARN，并提示 `ros2 topic info -v`。
- [ ] 单测：五档 quality、E/N/U 顺序、ratio/gnss_time 空值、NaN/Inf 拒绝、双源互不覆盖。

**验收:** 合成消息驱动后，内存中同时存在 app/fused 两路最新快照，字段值正确。

### Task 3：`$SAT` 流接入天空图

- [ ] `ros_collector.py` 新增 `stat`：订阅 `/rtkrcv_node/stat` 的 `RawStream`，用行重组器跨块拼接。
- [ ] 复用 `parsers/rtkstat.py` 的 `StatEpochAccumulator`，按 `tow` 分历元，生成 `sats_json`。
- [ ] 禁用旧的 `_stat_reader`，避免同一份 `$SAT` 被解析两次。
- [ ] 单测：块边界断行、同星多频、空块、`tow` 换历元；无数据时页面显示“等待 `$SAT` 数据”。

**验收:** 回放含 `$SAT` 的 bag 时，天空图卫星数与 `RtkFix.sats_used` 基本一致；无 `$SAT` 时页面有明确空态而非空白。

### Task 4：诊断消费

- [ ] 订阅 `/gnss/diagnostics` 与 `/rtkrcv_node/diagnostics`。
- [ ] 解析 `status_code`、`open_events`、`verdict.<code>`、`divergence_m`、`message`、`level`；生成事件列表与告警横幅。
- [ ] 默认关闭 `diagnosis.local_fallback`；仅话题缺失且显式开启时才跑本地规则。
- [ ] 单测：级别映射、`open_events` 拆分、宽限期文案不误报、双诊断源取最严重级。

**验收:** 人为触发差分中断，页面横幅与 `events.log` 同时出现同一条事件；HMI 本地不产生重复事件。

### Task 5：状态聚合与 WS 契约

- [ ] `main.py` 把 ROS 两路快照与 `$SAT` 接入 `last_status`（当前 ROS 数据只入库、不进 status，必须补上）。
- [ ] 新 status 结构：`sources.app` / `sources.fused`（各自 quality、sats、age、sigma、ratio、heading、latency）；顶层 `verdict`、`corr_rate`、`replay`、`stale`。
- [ ] 更新 `docs/ws-contract.md` 与后端 WS 推送；保持 `position`/`event`/`replay_end` 消息兼容。
- [ ] 单测：状态构造、双源缺一路、延迟计算、超 3 s 标 stale。

**验收:** `/api/status` 与 WS 首帧包含两路状态；停一路后该路变“无数据”，另一路不受影响。

### Task 6：前端双源常显

- [ ] `protocol.js` 的 `fixClass` 改为 0–4 映射；删除 rtkrcv 1/2、610 4/5 的旧假设。
- [ ] `statusbar.js` 改为两个固定徽标（APP 自解 / 610 融合），各自显示 quality/sats/age/sigma/ratio；无数据灰化标“无数据”。
- [ ] `mapview.js` 双轨迹图层默认都开；`heading` 无效时不画箭头；保留 σ 圆与离线底图。
- [ ] 单测：五档徽标、双源渲染、无数据状态、轨迹分段颜色。

**验收:** 回放双源 bag，页面同时出现两条轨迹与两个徽标；拔掉一路后对应徽标灰化，另一路正常。

### Task 7：曲线、链路卡片与日志窗

- [ ] `timeline.js` 增加接收延迟、差分字节/秒两条曲线；保持 1 Hz 抽稀与 1 h 窗口。
- [ ] 新增链路卡片：差分/观测速率、中断时长、`/rtkrcv_node/diagnostics` 健康、最严重诊断级别。
- [ ] 日志窗默认只显示事件与链路变化；原始 `$SAT` 不进主日志，支持暂停/过滤/复制。
- [ ] 单测：曲线数据裁剪、字节速率滑动窗口、断流/恢复。

**验收:** 断网 3 s 内链路卡片显示“差分中断”，恢复后自动清除；主日志不刷屏。

### Task 8：底图基础（在线 XYZ + 本地缓存）

- [ ] 新增 `tiles/manager.py`：按 bbox/zoom 拉取 XYZ（默认 Esri World Imagery），写入本地 MBTiles 缓存。
- [ ] 原子切换 `current.mbtiles`，保留最近 2 个版本与 SHA-256；地图显示归属。
- [ ] 无网/源不可用时读缓存，缓存缺失退网格；页面显示底图版本。
- [ ] 单测：mock HTTP 拉取、缺失瓦片补齐、原子切换、坏文件回退；不依赖真实外网。

**验收:** 首次联网预热后断网重启，底图仍可用；断网且无缓存时正常显示网格，不阻塞数据面。

### Task 9：只读校验与 P0 验收

- [ ] `ros2 node info /gnss_hmi` 确认无 publisher、无 service server；API 仅 GET + 静态资源（回放控制除外，P0 关闭）。
- [ ] 回放真实 bag（`--clock` + 临时 `root`），页面 1 Hz 状态与同段 `.pos`、`events.log` 对齐。
- [ ] 真车冒烟：双源 + `$SAT` 在线；拔网线；HMI 重启；旧 `rtk-monitor` 停止状态下无端口冲突。
- [ ] 验收记录写入 `hmi/docs/p0-acceptance.md`。

**验收:** 上述四项全部通过；所有测试全绿；`git status` 只含本任务文件。

---

## P1：工程化交付与底图自动维护

### Task 10：部署、systemd 与自检

- [ ] `deploy/deploy_and_start_gnss_hmi.sh`：rsync → 远端 venv → systemd 重启 → `curl /api/status` 自检，失败打印车端日志。
- [ ] systemd unit：`Restart=always`、`KillMode=control-group`、source ROS overlay 的包装脚本、`ROS_DOMAIN_ID` 配置。
- [ ] `run.log` 轮转、`status.json` 快照、配置持久化；服务重启不丢配置。
- [ ] 验收：换一台车/重装后一条命令可用；重启服务后 5 s 内页面恢复。

### Task 11：`tile_manager` 自动维护

- [ ] 每周定时检查 bbox/zoom 配置，补齐缺失瓦片；地形变化手动触发。
- [ ] 原子生成新版本、保留 2 版、SHA-256 校验、失败自动回滚到上一版。
- [ ] 归属标注与 API key 配置化；源不可用退回缓存/网格。
- [ ] 单测：定时调度、版本回滚、校验失败、无网。

### Task 12：矿区正射 GeoTIFF 导入

- [ ] CLI：GeoTIFF → MBTiles（可选 GDAL）；校验 bbox/zoom/EPSG:3857；生成元数据与版本。
- [ ] 拒绝 GCJ-02/BD-09 坐标；无法判定时要求显式配置。
- [ ] 验收：导入后页面版本切换成功；控制点叠加 ≤0.3 m 或压路 ≤0.5–1 m。

### Task 13：远程访问与 UI 打磨

- [ ] 绑定 `0.0.0.0:8080` 的部署说明；VPN/反向代理示例；不做内置账号。
- [ ] 窄屏可用；暗色卡片/大状态/日志窗对齐 `bynav_firmware_updater` 风格；事件导出。
- [ ] 验收：办公网经 VPN 连续观察 30 min 无断连；窄屏不遮挡关键指标。

### Task 14：P1 现场演练

- [ ] 断流、恢复、服务重启、时钟回跳、底图版本回滚五项演练。
- [ ] 记录到 `hmi/docs/p1-acceptance.md`；更新 `hmi/README.md` 故障排查。

---

## P2：存储、回放与分析

### Task 15：SQLite 1 Hz 存储与保留

- [ ] 启用 `storage.enabled`，按天分区写历元与事件；保留策略沿用 `gnss_cleanup_node` 配置。
- [ ] 验收：24 h 写入量、磁盘水位、清理后页面历史窗口正确。

### Task 16：回放与 live 切换

- [ ] WS `cmd replay/live`；回放从 SQLite 逐秒重建 status/position/event；游标同步曲线与地图。
- [ ] 验收：回放与 `.pos`/`events.log` 对齐；回放不影响 live 实例。

### Task 17：报告、`.pos` 下载与控制点偏差

- [ ] HTML 报告复用现有 `report.py`；`.pos` 下载；控制点绝对偏差与告警。
- [ ] 验收：报告数据与页面/`.pos` 三方一致。

### Task 18（可选）：Foxglove 旁路

- [ ] `foxglove_bridge` 启动说明与推荐布局；不替代主 HMI。
- [ ] 验收：浏览器可看 Plot/原始消息；不影响 HMI 进程。

---

## 依赖与顺序

```
T0 → T1 → T2 ─┬→ T5 → T6 → T7 → T9
              ├→ T3 ─┘
              ├→ T4 ─┘
              └→ T8 ────→ T11 → T12 → T14
T9 → T10 → T13
T9 → T15 → T16 → T17
```

- T2/T3/T4 可并行；T5 必须等三者落地。
- T6/T7 只依赖 T5 的 WS 契约；T8 可在 T1 后并行。
- P1/P2 必须以 T9 的 P0 验收为入口。

## Definition of Done（每个任务）

- 新测试先失败后通过；`pytest -q` 与 `python test_js.py` 全绿。
- 只读约束未被破坏：无 publisher/service，无自采集，无本地诊断重复。
- 文档同步：`hmi/README.md`、`docs/ws-contract.md`、验收记录按任务更新。
- `git status` 仅包含本任务文件，单独 commit，不 push。

## 不在本计划内

- 接收机参数下发、固件升级、节点重启按钮。
- 多车云端汇聚、账号体系、TLS 终结（部署层）。
- 点云/相机/SLAM 叠加、RTKLIB 后处理分析。
- 旧 `rtk-monitor` 的代码清理（先禁用；P1 结束后另开任务删除死路径）。
