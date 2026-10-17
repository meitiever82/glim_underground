# geacx 真车 gnss_bringup 部署设计（北云接收机数据源）

- 日期：2026-10-14
- 目标车：geacx（192.168.7.11，nvidia-desktop，aarch64，Ubuntu 22.04，ROS2 Humble）
- 状态：待评审。**先不动车上任何东西。**

## 1. 真车勘察结论（2026-10-14 实测）

| 项 | 现状 | 结论 |
| --- | --- | --- |
| 平台差分 | `/soft_dtu/rtcm_data`（robominer_msgs/RtcmData），~2 Hz 在流；北云驱动内置 NTRIP 客户端从 caster 117.159.171.77:2101 挂载点 SYDNEY 取流，同步回写接收机 | **可用**。首帧解码为 RTCM 1006（基准站坐标），`base_pos_type: rtcm` 前提成立 |
| 板卡原始观测 | `/bynav/oemraw`（novatel_oem7_msgs/Oem7RawMsg）话题存在但**无任何数据**：`oem_raw_msgs` 参数为空，且接收机 `receiver_init_commands` 未 LOG 任何原始观测（无 RANGEB/星历） | **不可用，必须改接收机输出** |
| 设备融合解 | `/gnss_cgi610/rtk_fix` 发布者数 0（该车无 CGI610；`device_divergence` 无输入） | 本车没有 device 路，diag 只跑 solver 侧规则 |
| rtk_monitor | 在跑（gnss_ws），其 config 的 raw_obs `192.168.200.1:9901` 标注"直连 610 前暂不可用"——与本设计无关，保持不动 | — |
| rtkrcv | 未安装 | 需装 RTKLIB-EX 2.5.1 |
| glim_ws | 车上没有 | 需部署 gnss_msgs + gnss_core + gnss_bringup |
| 系统时间 | NTP active，与开发机一致 | ✓（gnss_diag 前置条件满足） |
| 磁盘 | `/` 88% 已用，余 6.3 G | 超 cleanup 水位 85%，会一直 WARN；.pos/bag 量小可接受，但建议顺手清旧数据 |

## 2. 与 gnss_bringup 原设计的差异

原设计假设两路输入都是**裸 TCP 端点**（rtcm_bridge 去连或监听）。本车实际情况是两路（潜在的）都在 **ROS 话题**上，且类型不是 `gnss_msgs/RawStream`：

- 差分：`robominer_msgs/RtcmData`（`uint8[] rtcm_data`）
- 观测：`novatel_oem7_msgs/Oem7RawMsg`（`uint8[] message_data`，完整 OEM7 二进制帧含头与 CRC——已读驱动源码确认，RTKLIB `nov` 格式可直接解）

因此 `rtcm_bridge` 在本车**用不上**，代之以一个适配节点；`rtkrcv_node`/`pos_writer`/`gnss_diag`/`gnss_cleanup` 原样复用，只改 yaml。

## 3. 待决策点

### 3.1 原始观测怎么出来（必做，接收机当前不输出观测）

- **方案 A（ROS 内，推荐）**：改车上 bynav 驱动配置
  `oem_raw_msgs: [RANGE, GPSEPHEM, GLOEPHEMERIS, GALEPHEMERIS, BDSEPHEMERIS, QZSSEPHEMERIS]`，
  并在 `receiver_init_commands` 追加 `LOG RANGEB ONTIME 1` + 各系统星历 `ONCHANGED`
  （写进配置而非只发 Oem7Cmd 临时命令——驱动重启时会 `UNLOGALL THISPORT`，临时命令会静默丢）。
  代价：**重启一次 oem7_driver_node**（`/gps/fix` 等定位话题中断数秒）。
  带宽：RANGEB 1 Hz 约 1–2 KB/s，对现有 100 Hz INSPVAXB 通道可忽略。
- **方案 B（旁路 TCP）**：不动驱动，给接收机加一路 ICOM 拨出（如 → 车:15032），
  rtcm_bridge 以 `listen=true` 收。要改接收机网络配置（ICOM 拨出），
  且多一条带外链路，与"数据已在 ROS 发布"的方向相悖。

### 3.2 话题类型适配（必做，唯一的新代码）

新节点 `ros_stream_adapter`（gnss_bringup 内，约 80 行）：
订阅 `RtcmData`/`Oem7RawMsg`，原样把字节数组搬进 `gnss_msgs/RawStream`
（header.stamp 取到达时刻），发布 `/gnss/rtcm_corrections`、`/gnss/raw_obs`。
不做任何解析，与 rtcm_bridge 同一"哑管道"语义，rtkrcv_node 零改动。

### 3.3 rtkrcv_node 关键参数

- `obs_format: "nov"`（OEM7 二进制）、`corr_format: "rtcm3"`
- `corrections_topic`/`raw_obs_topic` 默认值不变（适配节点对齐到这两个话题）
- `base_pos_type: "rtcm"`（1006 已确认在流）
- `navsys: 63` 不变；北云是双天线但 rtkrcv 只用主天线观测

### 3.4 部署与目录

- 车上新建独立工作区 `~/gnss_bringup_ws`（不与 rtk_monitor 的 gnss_ws 混）：
  放 gnss_msgs（从 finder_ros 拷，注意与 driver_ws 那份同源）、gnss_core、gnss_bringup 三包。
  依赖：libgeographic-dev（apt）、RTKLIB-EX 2.5.1 源码编译（车上 gcc 11/cmake 3.22 够，
  源码包从开发机 scp 过去，避免车上访问 GitHub）。
- yaml 本地化：`root` 改 `/home/nvidia/gnss/pos`（车上无 /data）；
  `pos_writer.sources: ["rtkrcv"]`（本车无 can 路）；
  `gnss_diag` 的 `device_topic` 无发布者——device_divergence 静默不触发，其余规则正常。
-  systemd 单元（`KillMode=control-group`）在链路验证通过后再加上，先手动跑。

## 4. 验证步骤（部署后）

1. `ros2 topic hz /gnss/raw_obs /gnss/rtcm_corrections`——两路有流
2. `/rtkrcv_node/rtk_fix` 出解，quality 收敛到 FIXED；与 `/bynav/bestpos` 比对位置一致性
3. `.pos` 落盘、`/gnss/diagnostics`、`/rtkrcv_node/diagnostics` 正常
4. `record_gnss.sh` 录包 + cleanup 一轮

## 5. 风险

- 重启 oem7_driver_node 的几秒内整车定位话题断流——若车当时在用定位，需要挑停车窗口。
- VRS（SYDNEY 挂载点）是网络 RTK：rtkrcv 解与北云内部解用的是同一份差分，
  两者不一致时不能简单归因于"一边错了"（同源相关）。
- 星历消息名以接收机实际支持为准（BDSEPHEMERIS 等名要在车上用 Oem7Cmd 试通）。
