# geacx 真车 gnss_bringup 部署：设计与执行记录

- 日期：2026-10-14 设计，2026-10-17 执行完成
- 目标车：geacx（192.168.7.11，nvidia-desktop，aarch64，Ubuntu 22.04，ROS2 Humble）
- 状态：**已上线运行**（待室外天空视野做 RTK 固定验证）
- 设计初稿（含 ASCII→binary 转换设想，已被推翻）留档在
  `2026-10-14-geacx-bringup-deployment-design-v1.md`

## 1. 最终架构（与设计稿的差异）

设计稿假设"接收机只能输出 ASCII，驱动做 ASCII→binary 转换"。**执行时推翻**：
北云协议 UG016 §4.6.3 明确"星历及观测数据输出采用二进制格式"——rangecmpb/
gpsephemb 等根本没有 ASCII 变体，ASCII 转换路线没有输入可转。最终方案：

```
北云接收机(192.168.7.110) ──TCP:2222(ASCII 日志 + 新增二进制 rangecmpb/星历)──▶ bynav_ros_driver
  │                                                                                │
  │  NTRIP 客户端(驱动内置) ◀── caster 117.159.171.77:2101/SYDNEY                  │ oem_raw_msgs 原样转发
  ▼                                                                                ▼
/soft_dtu/rtcm_data (RtcmData, RTCM3 含 1006)                          /bynav/oemraw (Oem7RawMsg, 完整二进制帧)
  └──────────────┬─────────────────────────────────────────────────────┘
                 ▼  ros_stream_adapter(新节点,纯字节搬运)
        /gnss/rtcm_corrections + /gnss/raw_obs (gnss_msgs/RawStream)
                 ▼
        rtkrcv_node(监管 rtkrcv EX 2.5.1) → /rtkrcv_node/rtk_fix、~/stat、~/diagnostics
                 ▼
        pos_writer(.pos 落盘) / gnss_diag / gnss_cleanup
```

rtcm_bridge 不起（`enable_bridge:=false`）；rtkrcv_node/pos_writer/gnss_diag/
gnss_cleanup 零代码改动。

## 2. 改动清单（已全部落地并提交）

**finder_ros（ros2 分支，commit 389028e9，本地未 push）：**
- `oem_message_ids.h` / `oem_message_util.cpp`：补 RANGECMP(140)/GPSEPHEM(7)/
  GLOEPHEMERIS(723)/GALEPHEMERIS(1122)/BDSEPHEMERIS(1696)/QZSSEPHEMERIS(1336)/
  IONUTC(8)/RANGE(43) 的名字→ID 映射（`oem_raw_msgs` 按名字查表，缺了会被
  静默丢弃）
- `config/bynav_net.params.yaml`：`oem_raw_msgs` 启用上述消息；
  `receiver_init_commands` 追加 `LOG RANGECMPB ONTIME 1` + 5 星历 `ONCHANGED`
  + `IONUTCB` + `SAVECONFIG`（整车 launch 不起 oem7_config_node，SAVECONFIG
  保证接收机重启后日志不丢）

**glim_ws/glim_ext（master，commit 691786f，本地未 push）：**
- 新节点 `ros_stream_adapter`（RtcmData/Oem7RawMsg → RawStream，空消息丢弃，
  10s 节流统计日志）；仅在找到 robominer_msgs + novatel_oem7_msgs 时编译
- launch 增加 `enable_bridge`（默认 true）/ `enable_adapter`（默认 false）
- `config/gnss_bringup_geacx.yaml`：车上覆盖配置
- 测试：进程级用例 ×2（字节原样搬运/空消息丢弃/缺配置拒启），全包 268 项全绿

**geacx 车上：**
- `~/gnss_bringup_ws`：gnss_msgs（符号链接→finder_ros）、robominer_msgs
  （符号链接→driver_ws/src）、gnss_core、gnss_bringup（rsync 自开发机）
- RTKLIB-EX 2.5.1：源码 rsync 后车上编译，`~/.local` 用户态安装（rpath 指向
  `~/.local/lib`，免 sudo）
- GTSAM 4.3a1：车上 `/usr/local/lib` 的 libgtsam.so.4.2.0 是裸拷库（无头文件、
  无 cmake 配置、依赖车上没有的 boost 1.71）——用 finder_deps 的 gtsam-4.3a1.zip
  编到 `~/gtsam-local`；**启动脚本必须 `export LD_LIBRARY_PATH=$HOME/gtsam-local/lib`**
  否则 ldconfig 缓存里的 4.2.0 优先加载，节点 127 退出
- 启停脚本：`/tmp/start_gnss_bringup.sh`、`/tmp/restart_gnss_bringup.sh`
  （幂等重启；/tmp 重启即失，正式化需转 systemd + 固化路径）

## 3. 启动方式（现状）

```bash
# 驱动(带 config node 发 LOG 命令):
ros2 launch bynav_ros_driver connect_net.launch.py   # 当前以 setsid 常驻
# gnss_bringup:
/tmp/start_gnss_bringup.sh   # = launch params_file:=gnss_bringup_geacx.yaml \
                             #   enable_bridge:=false enable_adapter:=true enable_rtkrcv:=true
```

**与整车栈的关系（重要）**：`boonray_smallcar.launch.py` 也会直起
bynav_ros_driver_node（同一 yaml，会自动带上 oem_raw_msgs，但不发 init 命令、
靠 SAVECONFIG 兜底）。两拨驱动不能同时跑（:2222 只能一家绑定）。当前是我单独
起的 connect_net.launch 在服务；现场重启整车栈前，应先停掉我的 connect_net
launch 与 gnss_bringup launch，避免端口冲突。

## 4. 验证结果（2026-10-17，室内）

| 项 | 结果 |
| --- | --- |
| oemraw 转发 | ✓ RANGECMP 1 Hz、IONUTC、星历（启动时 dump）都在 |
| 适配节点 | ✓ 两路 RawStream 按字节原样转发，~7.5 KB/10s obs、~9.2 KB/10s rtcm |
| rtkrcv 监管 | ✓ EX 2.5.1 起停正常，sol 流连接，conf 校验拦截了错误格式名（oem4 之外的 nov/tersus 在 2.5.1 不存在） |
| RTK 解算 | ⏸ 室内 nobs=0（接收机零卫星），无解属预期；`/rtkrcv_node/diagnostics` 正确报"上行有数据但无解" |
| 星历 | ⏸ 接收机无星历（室内）；GPS/QZSS 星历在 RTKLIB-EX 2.5.1 的 oem4 解码器里**不认** gpsephemb(7)/qzssephemerisb(1336)，需靠差分流 RTCM 1019 或后处理注入——**室外验证时必须确认差分流是否带 1019**（不带则 navsys 改 36 或注入星历） |
| .pos/diag/cleanup | ✓ 进程正常；pos_writer 的沉默告警与 diag 的 no_solution 事件按设计触发 |

## 5. 室外验证清单（下次出车）

1. 差分流抓包确认 RTCM 消息类型：有没有 1019（GPS 星历）/1020/1042/1045。
2. `/rtkrcv_node/rtk_fix` 是否收敛到 FIXED；与 `/bynav/bestpos` 比对。
3. `oemraw` 里五个星历消息是否齐（ID 7/723/1122/1696/1336）。
4. 若 GPS 不可用（无 1019）：navsys 改 36（GLO+BDS）或评估星历注入。

## 6. 已知遗留

- `rtcm_bridge` 无健康话题（worker 终态只有一行 ERROR）——评审遗留项，未动。
- 车上 finder_ros 的改动是 rsync 落位的**未提交**状态；正本在开发机
  commit 389028e9。
- 车上磁盘 88%，cleanup 水位已在 geacx yaml 抬到 92%。
- 接收机 SAVECONFIG 已含新日志；若平台团队重刷接收机配置，需重发 LOG 命令。
