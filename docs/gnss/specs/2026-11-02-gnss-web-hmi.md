# SPEC：GNSS Web HMI（矿区现场实时分析）

> 来源：2026-11-02 现场 HMI 需求讨论。现场与远程都需要浏览器直接看，不能依赖 GLIM 的 ImGui viewer（必须接显示器）。
> 状态：已定稿（2026-11-02 决策）
> 参考实现：`/home/boonraygrn/Documents/GitHub/gnss-alg/rtk-monitor`（Web 前端、诊断规则、回放与报告的既有实现）

---

## Problem Statement

现有 GNSS 观测手段在两个极端：

- `gnss_bringup` 是纯无头 ROS2 节点，现场只能 `ros2 topic echo`、看日志、事后读 `.pos` 和 `events.log`。定位是否可用、为什么不可用、链路哪一段断了，需要人同时看多个终端。
- GLIM 的 ImGui viewer 能看，但必须接显示器、坐在车上，无法满足矿区远程分析；且界面迁移原计划在轮 4，至今没有落地。
- 旧 `rtk-monitor` 已有完整 Web HMI（地图、天空图、事件、曲线、回放、报告），但它自己直连 TCP/CAN、自己监管 `rtkrcv`，与 `gnss_bringup` 的职责重叠；继续用会出现两套解算、两套诊断、两套落盘。

结果是：**数据都在总线上，但没有一个适合产品交付、可远程访问、且与 `gnss_bringup` 单一数据源一致的 HMI。**

## Goals

1. 浏览器打开即用：车端跑一个 Web 服务，车内和远程（VPN/端口映射）看同一页面，不需要 VNC、不需要显示器。
2. 实时回答四个问题：**现在能不能用**（固定/浮点/单点/无数据）、**精度如何**（σ、ratio、龄期、卫星数）、**为什么不可用**（诊断事件与告警）、**轨迹是否可信**（地图漂移/跳变/双解偏差）。
3. 与 `gnss_bringup` 严格单一数据源：只订阅 ROS 话题与只读文件，不再起第二套 `rtkrcv`、TCP/CAN 采集或诊断规则。
4. 只读：HMI 不下发接收机参数、不重启节点、不触发服务，避免现场误操作。
5. 复用 `rtk-monitor` 的 Web 前端与既有测试，不重写地图、天空图、uPlot 曲线和回放。

## Non-Goals

- 接收机配置、固件升级（由 `bynav_firmware_updater` 独立承担）。
- 云端多车汇聚、账号体系、权限管理（v1 由部署层 VPN/反向代理解决）。
- 3D LiDAR 点云、SLAM 地图、相机画面。
- RTKPLOT 级别的后处理分析（残差、多径、观测值质量、RINEX）。
- 替代 rosbag2 录制、`.pos` 落盘、`gnss_diag` 的 `events.log`。
- 手机原生 App；P1 只保证窄屏可用。

## User Stories

**现场运维**

1. 作为现场运维，我要打开浏览器 3 秒内看到固定状态、经纬高、卫星数、龄期、σ、ratio，以便判断当前定位能不能用于作业。
2. 作为现场运维，我要在矿区离线影像上看到分色轨迹和车辆朝向，以便一眼发现漂移、跳变、绕圈。
3. 作为现场运维，我要看到天空图与每颗卫星的 C/N0、高度角，以便判断是遮挡、多径还是卫星几何差。
4. 作为现场运维，我要看到当前未关闭的诊断事件和告警横幅，以便知道“为什么不可用”，而不是只看一个灰点。
5. 作为现场运维，我要看到差分流速率、接收延迟、上行丢弃计数，以便区分“链路问题”和“定位问题”。
6. 作为现场运维，我要在 APP 独立解与 610 融合解之间对比偏差，以便发现其中一路解算异常。

**远程专家 / 调度**

7. 作为远程专家，我要通过车端 Web 端口看到与现场完全相同的页面，以便不做 VNC、不装客户端。
8. 作为远程专家，我要能回放当天已落盘的历元，以便复现现场描述的现象。
9. 作为远程专家，我要能下载 `.pos` 与诊断报告，以便离线分析。

**测试 / 项目**

10. 作为测试工程师，我要用 rosbag2 回放真实数据并在页面上核对 `.pos`、`events.log`，以便证明 HMI 显示与既有链路一致。
11. 作为项目，我要 HMI 与 `gnss_bringup` 的接口只有 ROS 话题和只读文件，以便不引入第二套数据源。
12. 作为项目，我要一条脚本完成车端部署与自检，以便换车/重装不依赖人工步骤。

## Solution

```
gnss_bringup（ROS2，无头）
  /rtkrcv_node/rtk_fix        (gnss_msgs/RtkFix, reliable)
  /gnss_cgi610/rtk_fix        (gnss_msgs/RtkFix)
  /rtkrcv_node/stat           (gnss_msgs/RawStream，内容是 $SAT 行块)
  /gnss/diagnostics           (diagnostic_msgs/DiagnosticArray)
  /rtkrcv_node/diagnostics    (diagnostic_msgs/DiagnosticArray)
  /gnss/rtcm_corrections      (gnss_msgs/RawStream，仅统计速率)
  /gnss/raw_obs               (gnss_msgs/RawStream，仅统计速率)
        │ DDS
        ▼
gnss_bringup/hmi（独立 Python 应用，随 gnss_bringup 仓库分发）
  rclpy 订阅线程 → 最新状态缓存（RtkFix/$SAT/诊断/字节计数）
                 → 可选 SQLite 1 Hz 历元 + 事件（P2）
                 → FastAPI + WebSocket + 静态页
        │ HTTP / WS :8080
        ▼
浏览器（车内 / 远程 VPN 或端口映射）
```

核心决策：**HMI 是 `rtk-monitor` 的“去采集化”改造**——保留 `web/`、`api.py`、`diagnosis` 展示与回放/报告，替换掉 `collectors/tcp.py`、`collectors/can.py`、`solver/rtkrcv.py` 三条自采集路径。

## Implementation Decisions

### 边界与部署形态

- 应用放在 `gnss_bringup/hmi/`，与 `gnss_bringup` 同仓库、同版本分发（`src/gnss_bringup` 是 symlink，实际路径为 `glim_ext/gnss_bringup/hmi/`）。Python 3.10+，FastAPI + uvicorn + rclpy；该目录不注册 `package.xml`，**不编进 `glim_ws` 的 colcon 构建**，避免把 Web 依赖引入 ROS 工作区。运行时 source ROS overlay 取得 `rclpy` 与 `gnss_msgs`。
- 代码以 `rtk-monitor` 为基线 fork；`web/` 静态资源原样保留（Vue3/Leaflet/uPlot 均已 vendored，无 npm 构建）。
- 车端由 systemd 常驻，脚本一键部署；端口默认 `8080`，可配置。
- HMI 进程**只读**：不创建 publisher，不调用 service/action，不启动/停止任何 ROS 节点或 `rtkrcv`。

### 订阅接口（唯一数据源）

| 话题 | 类型 | QoS | HMI 用途 |
| --- | --- | --- | --- |
| `/rtkrcv_node/rtk_fix` | `gnss_msgs/RtkFix` | reliable（发布端 `QoS(100).reliable`） | APP 独立解：状态徽标、地图轨迹、曲线 |
| `/gnss_cgi610/rtk_fix` | `gnss_msgs/RtkFix` | 按发布端配置 | 610 融合解：第二条轨迹、双解偏差 |
| `/rtkrcv_node/stat` | `gnss_msgs/RawStream` | reliable（发布端 `QoS(100).reliable`） | 内容为 `.stat` 文件块，按行重组后解析 `$SAT` → 天空图、C/N0、星座统计 |
| `/gnss/diagnostics` | `diagnostic_msgs/DiagnosticArray` | 默认 | 事件列表、告警横幅、`status_code`、`open_events`、`divergence_m` |
| `/rtkrcv_node/diagnostics` | `diagnostic_msgs/DiagnosticArray` | 默认 | rtkrcv 子进程/无解健康卡片 |
| `/gnss/rtcm_corrections` | `gnss_msgs/RawStream` | 默认 | 只统计字节/秒与中断时长，不解码 |
| `/gnss/raw_obs` | `gnss_msgs/RawStream` | 默认 | 只统计字节/秒与中断时长，不解码 |

- 每条订阅的 `qos` 可配置为 `auto` / `reliable` / `best_effort`；默认按上表。启动后 N 秒未收到消息时，在页面提示“检查 `ros2 topic info -v` 的 QoS 是否匹配”，不静默。
- `/gnss/diagnostics` 的 `status.name` 为 `gnss_diag`；`values` 里已有 `status_code`、`open_events`、`divergence_m`、`verdict.<code>`。HMI 直接消费，**不再本地重跑规则**。仅当该话题不存在且配置显式开启 `diagnosis.local_fallback` 时，才使用 `rtk-monitor` 旧规则兜底。

### 数据映射

| 来源字段 | HMI 字段 | 规则 |
| --- | --- | --- |
| `RtkFix.quality` | 徽标/轨迹颜色 | 0 NONE → 灰；1 SINGLE → 红；2 DGPS → 红；3 FLOAT → 黄；4 FIXED → 绿 |
| `RtkFix.latitude/longitude/altitude` | 地图、状态栏 | 直接使用，WGS-84 |
| `RtkFix.sigma_enu[0,1,2]` | σ（水平） | **E/N/U 顺序**，水平 σ = `hypot(E,N)`；不要沿用 rtk-monitor 的 `sdn/sde` 命名 |
| `RtkFix.diff_age` | 龄期 | 直接显示；无解时保留上一次并标灰 |
| `RtkFix.sats_used` | 卫星数 | 以 `RtkFix` 为准；`$SAT` 的 used 计数用于天空图填色 |
| `RtkFix.ratio` | 固定质量 | `0` = 源不提供，显示 `—`，不参与“差/好”判断 |
| `RtkFix.gnss_time` | 曲线横轴、延迟 | `0` = 源不提供；延迟 = `header.stamp - gnss_time`，为负或 > 5 s 标黄 |
| `RtkFix.raw_status` | 详情展开 | 仅追溯展示，不参与徽标判定 |
| `$SAT` 行 | 天空图 | 复用 `rtkstat.parse_sat_line` + `StatEpochAccumulator`；按 `tow` 分历元，每星取第一频率 |
| `DiagnosticStatus.level` | 告警横幅 | 取全部 status 的最严重级；`ERROR` 红、`WARN` 黄、`OK` 绿 |
| `DiagnosticStatus.message` | 横幅文案 | 直接显示，保留中文原文 |
| `open_events` | 事件列表 | 逗号分隔的规则码 → 事件条目 |
| `verdict.<code>` | 规则详情 | 每行一个规则码的级别与文案 |
| `rtcm_corrections` 字节 | 链路卡片 | 滑动窗口字节/秒；> 3 s 无字节 → “差分中断”提示 |

### UI 规格

1. **顶部状态栏**：**两个固定状态徽标常显**——“APP 自解”和“610 融合”，各自显示质量、卫星数、龄期、水平 σ、ratio；另有速度、接收延迟、页面时间、连接状态。任一来源无数据时该徽标灰化并标“无数据”，不隐藏。
2. **地图**：Leaflet；P0 用在线 XYZ（默认 Esri World Imagery，显示归属）并由 `tile_manager` 本地缓存瓦片，无网时回退缓存/网格；P1 支持矿区正射 GeoTIFF 导入与自动更新；按质量分色轨迹；车辆箭头按 `heading`（无效时不显示）；σ 圆；基站位置与控制点；APP 自解/610 融合两个轨迹图层默认都开，可独立关闭；“跟随车辆”开关。
3. **天空图**：方位/高度角极坐标；C/N0 着色（>40 绿、35–40 黄、<35 红），参与解算填实心、未参与描边；无 `$SAT` 时显示“等待 $SAT 数据（需 rtkrcv stat 流）”。
4. **事件区**：未关闭事件置顶；级别色；显示规则码、文案、开始时间、持续时长、峰值指标；最近 24 h 可折叠；支持导出文本。
5. **曲线**：卫星数、龄期、水平 σ、ratio、接收延迟、差分字节/秒；1 Hz 抽稀，默认 1 h 窗口，可缩放；回放时有同步游标。
6. **链路卡片**：差分/观测字节速率、上行丢弃计数、`/rtkrcv_node/diagnostics` 状态、最严重诊断级别。
7. **日志窗**：默认只显示事件与链路变化；原始 `$SAT` 不进主日志（完整内容留在车端 `run.log`）；支持暂停/过滤/复制。
8. **回放条（P2）**：时间范围、速度、播放/暂停；回放数据来自 HMI 自己的 SQLite 1 Hz 历元，不替代 rosbag2。

### 状态与异常行为

- **无数据**：超过 3 s 没有新的可用解 → 横幅“数据中断”，徽标灰化，地图保留最后位置但整体降透明度；恢复后自动清除。绝不把旧解伪装成新解。
- **单路故障**：只有 610 融合解或只有 APP 自解时，正常显示可用那一路，另一路显示“无数据”，不阻塞页面。
- **启动宽限**：`gnss_diag` 的宽限状态（`OK` + “启动宽限期”）原样显示；不误报“无解”。
- **时间回跳**：检测 ROS 时间回退 > 1 s → 清空曲线窗口、保留地图最后轨迹并提示“时间回跳，已重置窗口”。
- **服务重启**：浏览器 WebSocket 断开后指数退避重连；服务重启不影响 `gnss_bringup`；配置持久化，重启后恢复话题/QoS/底图路径。
- **只读保护**：页面无任何写操作按钮；`/api` 只提供 GET 与静态资源（回放控制除外，仅影响 HMI 自身）。

### 非功能要求

- 端到端显示延迟：从消息到页面 ≤ 2 s（1 Hz 诊断节拍下）。
- 24/7 运行：内存用环形缓冲，默认保留 1 h 曲线；P2 SQLite 按天分区并纳入 `gnss_cleanup_node` 同级保留策略。
- 带宽：WebSocket 默认 1 Hz status + 事件增量；地图轨迹按增量追加，不整段重发。
- 离线：P0 在线 XYZ 首次拉取后本地缓存，无网读缓存、缓存缺失退网格；P1 使用矿区正射 MBTiles；页面自身不依赖在线 CDN。
- 浏览器：Chrome/Edge 最新版；不强制 TLS（由部署层 VPN/反代提供）。

### 部署

- `deploy_and_start_gnss_hmi.sh`（沿用 bynav 模式）：rsync → 远端创建 venv/安装依赖 → systemd 重启 → `curl /api/status` 自检；失败时直接打印车端日志。
- systemd：`Restart=always`、`KillMode=control-group`、`Environment=ROS_DOMAIN_ID=<x>`，`ExecStart` 用包装脚本先 source `/opt/ros/humble/setup.bash` 与 `install/setup.bash`。
- 配置：`config.yaml`（话题、QoS、端口、底图源/API key/缓存目录/更新周期、数据目录、保留天数），页面可改的仅限显示偏好，服务端配置由文件管理。
- 远程访问：默认绑定 `0.0.0.0:8080`；远程统一经 VPN 或反向代理访问，认证与 TLS 全部在 VPN/反代层完成，**HMI 自身不做账号体系**。
- 运行期文件：`run.log`（完整）、`status.json`（最近状态快照，便于 curl 自检）、SQLite（P2）。所有路径可配。

### 底图（MBTiles）交付标准

- **覆盖范围**：作业区（采坑、排土场、装卸点、主干道路）+ 基站 + 10% 缓冲；bbox 用 WGS-84，并向外对齐到整瓦片。全域 z13–16 概览，重点区 z17–19 细节；单文件目标 ≤ 1–2 GB，超出则拆成“概览/细节”两个 MBTiles，按缩放切换。
- **坐标系**：瓦片必须是 EPSG:3857（Web Mercator）；源影像/正射必须是 WGS-84 或 CGCS2000（两者厘米级差异可接受）。**不接受 GCJ-02 / BD-09 来源**（高德/腾讯/百度在线影像），否则会出现几十到几百米的恒定偏移。
- **格式**：卫星影像用 JPEG（质量 80–85），纯线划/注记用 PNG；接入时校验 TMS/XYZ 翻转，避免轨迹上下颠倒或错位。
- **源与缓存**：P0 默认在线 XYZ（Esri World Imagery，显示归属），`tile_manager` 首次按 bbox/zoom 拉取并写成本地 MBTiles 缓存；无网或源不可用时读缓存，缓存缺失退网格。P1 支持矿区正射 GeoTIFF 导入（转 MBTiles，可选 GDAL）；天地图作为需 API key 的备选源。
- **版本与命名**：`mine_<source>_<YYYYMMDD>_z<min>-<max>_<crs>.mbtiles`，例如 `mine_ortho_20261102_z13-19_wgs84.mbtiles`；配置指向 `current.mbtiles` 软链，保留最近 2 个版本并记录 SHA-256。
- **责任与自动维护**：瓦片下载、缓存、MBTiles 生成/更新、版本与校验由 HMI 内的 `tile_manager` 负责，开发者维护；默认按配置的源（XYZ/TMS URL 或本地 GeoTIFF/MBTiles）自动补齐缺失瓦片、原子切换 `current.mbtiles`、保留最近 2 个版本并记录 SHA-256。在线源需要 API key/授权时写在配置里，地图显示归属标注；源不可用时自动退回当前缓存或网格。定时任务默认每周检查一次，台阶/道路重大变化或发现 > 0.5 m 偏移时可手动触发重做。测量组只需提供正射/影像（如用在线源则不需要）和验收用控制点；运维只反馈偏移。
- **验收**：页面显示底图版本；有 ≥3 个已知 WGS-84 控制点（不共线）时，叠加偏差 ≤ 0.3 m（最低 ≤ 0.5 m）；没有控制点时，沿已知道路行驶的轨迹在 z19 下偏差 ≤ 0.5–1 m，且不出现恒定偏移、旋转或缩放错误。

## Testing Decisions

**单元测试（Python）**

- `RtkFix → Epoch/status`：quality 五档映射、σ 的 E/N/U 顺序、`ratio=0`/`gnss_time=0` 映射为空、延迟计算、NaN/Inf 拒绝。
- `RawStream → $SAT`：跨块边界的行重组；`tow` 换历元；同星多频去重；无 `$SAT` 时不产生空天空图抖动。
- `/gnss/diagnostics → 事件/横幅`：级别映射、`open_events` 解析、`verdict.*` 拆分、宽限期文案不误报。
- 字节速率：滑动窗口、断流、恢复；只统计不解码。

**JavaScript 测试（node --test，沿用 rtk-monitor 既有用例）**

- `fixClass` 五档、`badge` 文案、`segmentTrail` 分色、事件去重键、天空图坐标变换。

**集成与现场**

- 回放真实 bag（`ros2 bag play --clock` + HMI `use_sim_time` + 隔离数据目录）：页面 API 的 1 Hz 状态与同段 `.pos`、`events.log` 对齐。
- 真车：APP 自解 + 610 融合 + `$SAT` 同时在线；拔网线 3 s 内出现“数据中断”；恢复后自动清除；HMI 重启后 5 s 内恢复页面。
- 远程：办公网经映射端口连续观察 30 min，无断连、无明显延迟漂移。
- 安全：确认 HMI 进程无 publisher/service 调用；页面无写操作入口。

**阶段验收**

| 阶段 | 验收门槛 |
| --- | --- |
| P0 | 单页可看：状态徽标、地图、天空图、事件、四条曲线；数据全部来自 ROS 话题；无自采集进程 |
| P1 | 一键部署 + systemd；链路卡片、延迟、双解对比、离线底图；断流/恢复/重启演练通过 |
| P2 | SQLite 回放、HTML 报告、`.pos` 下载、控制点偏差；回放与落盘数据一致 |

## 分期

| 期 | 内容 | 依赖 |
| --- | --- | --- |
| P0 | ROS 适配层（RtkFix/$SAT/diagnostics）+ 现有页面直出；在线 XYZ 底图 + 本地瓦片缓存；只读；无存储 | `rtk-monitor` 基线；`gnss_bringup` 话题 |
| P1 | 工程化交付：systemd/部署脚本/配置持久化/链路卡片/延迟/双解对比常显/`tile_manager` 自动更新与正射 GeoTIFF 导入/窄屏 | P0 |
| P2 | SQLite 1 Hz 历元 + 回放 + HTML 报告 + `.pos` 下载 + 控制点绝对偏差 | P1；`gnss_diag` 落盘 |
| 可选 | `foxglove_bridge` 作为旁路调试（Plot/原始消息/MCAP），不替代主 HMI | 独立 |

## Out of Scope

- 内置登录/账号/HTTPS/TLS（部署层解决）。
- 多车云端汇聚与车-云上传协议。
- 接收机参数下发、基线重置服务调用、节点重启按钮。
- 云台/相机/点云联动、SLAM 地图叠加。
- 替代 RTKLIB 后处理与 RTKPLOT 的观测值分析。

## 决策记录（2026-11-02）

| 事项 | 决策 |
| --- | --- |
| 代码位置 | `gnss_bringup/hmi/`，与 `gnss_bringup` 同仓同版本；不注册 `package.xml`，不进 colcon |
| 远程访问 | 统一 VPN/反向代理，HMI 自身不做账号体系 |
| 双解对比 | APP 自解 + 610 融合常显；任一路无数据时灰化标“无数据”，不隐藏 |
| 底图源 | P0 在线 XYZ（Esri World Imagery）+ 本地缓存；P1 矿区正射 GeoTIFF；天地图为需 key 的备选 |
| 底图维护 | `tile_manager` 负责下载/缓存/切片/版本/校验，开发者维护；每周自检 + 地形变化手动触发 |
| 底图验收 | ≥3 个已知 WGS-84 控制点：≤ 0.3 m；无控制点：压已知道路 ≤ 0.5–1 m |
| 数据保留 | P2 SQLite 沿用 `gnss_cleanup_node` 的保留策略 |
| 界面形态 | Web 为主、ImGui 可选 |

## Risks

1. **QoS 静默不匹配**：610 驱动的 `RtkFix` QoS 若为 `best_effort`，reliable 订阅收不到且无报错。必须做“订阅后 N 秒无消息 + `ros2 topic info -v` 提示”，并允许按话题配置 QoS。
2. **两套诊断漂移**：HMI 本地规则只在 `/gnss/diagnostics` 缺失时启用；默认关闭，避免与 `gnss_core` 阈值不一致。
3. **`$SAT` 可用性**：只有 `rtkrcv` 运行且 stat tail 正常时天空图才有数据；页面必须明确区分“没数据”与“没卫星”。
4. **`gnss_msgs` 跨仓同步**：HMI 依赖 `RtkFix` 定义；字段变更后 `glim_ws` 与 `driver_ws` 都要重编，否则反序列化失败。
5. **回放隔离**：回放必须 `use_sim_time` + 独立数据目录，不能写入生产 `root`（沿用 `gnss_bringup` README 纪律）。
6. **时钟跳变**：向前跳不处理、向后跳重置窗口；与 `gnss_diag` 的规则保持一致。
7. **610 融合解的可用性**：双解对比定为常显；若现场长期只有一路，另一路会一直显示“无数据”，需要现场确认这是预期的降级状态，而不是采集缺失。
8. **在线底图授权与可用性**：Esri 需按条款显示归属；若现场网络不稳定，`tile_manager` 必须提前预热缓存；天地图需 key。

## Further Notes

- 与 `2026-09-03-gnss-glim-modules-design.md` §11 的关系：ImGui viewer 保留为**车上有显示器时的本地视图**；Web HMI 是**产品交付与远程分析**的主界面。两者消费同一批 ROS 话题，不产生第二套数据源。轮 4 的界面迁移计划应改成“Web HMI 为主、ImGui 可选”，避免重复实现地图/天空图。
- 与 `bynav_firmware_updater` 的关系：只复用交付与交互骨架——暗色卡片、大状态、节流日志窗、配置持久化、一键部署/远端自检、独立子进程。HMI 是只读监控，不沿用“测试连接→升级”的写操作门控。
- 过渡期：旧 `rtk-monitor`（`gnss_ws`）可继续运行；新 HMI 上线并完成双跑对比后，再停旧实例，避免两套诊断同时告警。
- 旧 `rtk-monitor` 的 `ros_collector.py` 已支持 `gga/navsat/bestpos`，改造时新增 `rtkfix/stat/diagnostic/rtcm_rate` 四类即可，保留其“rclpy 线程 + `call_soon_threadsafe` 回主循环”的写法。
