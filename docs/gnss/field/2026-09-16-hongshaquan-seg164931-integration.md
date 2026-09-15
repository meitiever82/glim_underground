# 红沙泉 2026-09-15 数据段 seg_164931_165748:GNSS 整链路首次实车数据联调结果

- 日期:2026-09-16(夜间无人值守执行)。本文由 Claude Opus 5(Task 5)汇总,依据 Task 1–6 的报告与产物;文中数值均来自下面列出的文件,已按文件重新核对。
- 计划:`docs/gnss/plans/2026-09-16-field-integration-hongshaquan.md`
- 代码:`glim_ext` 分支 `feat/field-replay-20260915`,从 master `dbd2986` 开出,**未 push**。
  - 回放工具:`b4dba99`、`38a33ea`、`adba6fc`、`26cb18f`
  - 产品修复:`a911a91`、`e7c7895`、`d2d97c3`、`4ccf694`
  - 评估脚本:`763ca56`、`6362e74`(fix round 1)
- 数据段:`<seg>` = `/home/steve/Documents/Datasets/tage/hongshaquan/20260915/seg_164931_165748`。数据只读,产物都在 `<seg>/integration_20260916/`。
- 主要追溯文件:
  - 部件核查:`<seg>/integration_20260916/inventory.md`
  - rtkrcv 不固定的离线调查:`investigation_float/REPORT.md`
  - 各遍运行摘要:`run_{A,B,A_before_conf_fix,B_before_conf_fix}/summary.md`
  - 评估输出:`eval/{A,B,A_before_conf_fix,B_before_conf_fix}.md`,由 `field_eval.py` 生成;`eval/A0_nobag.md` 是不读录包的对照运行
  - 车辆外参:`/home/steve/Documents/Datasets/tage/hongshaquan/extrinsics_tage_truck.yaml`、`<seg>/../README_LIO.md` §4

名词约定:
- **ref**:`<seg>/gnss/rtk_check.pos`。RTKLIB-EX 2.5.1 `rnx2rtkp` 后处理解,498 历元,Q1 486 / Q2 12。
- **rtkrcv**:本链路 `rtkrcv_node` 的实时解。
- **can**:610 组合导航经 CAN 驱动输出的融合解。
- 修复前的两遍记为 **A0 / B0**(目录 `run_*_before_conf_fix`);修复后的两遍记为 **A / B**。
- 时间:`.pos` 是 GPST,`events.log` 是 UTC,GPST = UTC + 18 s。本文除非另注,一律写 UTC。

---

## 1. 结论摘要

1. **实车上 CAN 这条路走不通,这是最要紧的问题。** 这台车的 610 不发 3 帧 σ 报文:0x326 位置 σ、0x328 速度 σ、0x32B 姿态 σ。`gnss_chcnav` 驱动要求 14 帧齐全才发布,于是 `/gnss_cgi610/rtk_fix` 一条也没有,实车上同样如此。
   - 同一天另两次抓包也缺这三帧。
   - 本次联调在 CAN 日志副本里补了全零 σ 帧才跑通,所以 can 的 σ 没有意义。
   - 按约束没有改 finder_ros,待维护者定方案(§7 问题 1)。
2. **配置修复之后,补上星历(B)的实时 RTK 与后处理结果一致。**
   - rtkrcv 489/497 条固定(98.4%)。首次固定在首解后 +8 s,即数据起点 +9 s;ref 是 +12 s。
   - 双方都固定的 486 对:水平差中位数 0.002 m、p95 0.005 m、最大 0.009 m;|高程差| 中位数 0.004 m、p95 0.014 m、最大 0.072 m。
   - 修复前 B0 是 497 条全部浮点。
   - 注意:rtkrcv 和 ref 用的是同一套引擎、同一份观测和差分,所以这只说明实时链路复现了后处理结果,**不说明绝对精度**。
3. **修复前实时解一次都没固定,根因是配置。**
   - 当时是 `pos1-elmask=10`,没设模糊度固定的高度角门限;10–15° 的低卫星多路径大,把 ratio 压在 1.1–1.6。
   - 修复:新增 `ar_elmask`(`pos2-arelmask=15`),提交 `a911a91`。
   - 同批还修了 rtkrcv 输入流"空闲 10 s 断开、10 s 后才重连"导致丢数据的问题:`e7c7895`、`d2d97c3`,回放工具配套改动 `4ccf694`。
4. **610 原始输出里没有 GPS 星历,平台差分流里也没有星历电文。** 所以原样回放(A)全程只有 3–5 颗 Galileo:
   - 401 条解(Q2 360 / Q4 41),从未固定;
   - 相对 ref(Q1),水平差中位数 3.94 m,高程差均值 −12.33 m。
   - 修复前后位置、Q、ns 逐条相同,修复对 A 的定位不起作用;只有 ratio 列不同(A0 为 0.0–3.0,A 全为 0.0),这是修复带来的副作用,见第 6 条。
   - 现场要么让 610 板卡输出星历,要么让平台转发星历电文(§8)。
5. **610 融合解与 RTK 天线的偏差就是车辆外参里已知的杆臂,实测与外参吻合到 1–2 cm;`device_divergence` 因为没按同一点比较,结构性地一直是 serious。**
   - 610 的 INSPVAXB / CAN 位置输出在 base_link,即后轴中心正下方地面;GNSS 天线在 base_link 的(前 8.45, 左 1.85, 上 5.16)m。见 `extrinsics_tage_truck.yaml` 的 `T_base_link_gnss_antenna`。
   - 本次实测 can − ref 的车体系均值(标准差):前 −8.448(0.051)、右 +1.860(0.070)、天 −5.139(0.091)m。换成天线在 base_link 中的位置是(8.448, 1.860, 5.139)m,与外参(8.45, 1.85, 5.16)相差(−0.002, +0.010, −0.021)m;与外参文件记录的 16:49 段事后 RTK 实测(8.439, 1.842, 5.153)相差(+0.009, +0.018, −0.014)m。
   - 扣除后水平残差中位 0.056 m;前向差对速度回归斜率 −0.002 s,没有时间偏差。
   - 所以"疑似 610 融合问题"的归因不成立。`gnss_diag` 需要先把 610 的 base_link 位置按杆臂和航向换到天线点(或反过来)再比较(§5、§9)。
6. **修复 `a911a91`(`pos2-arelmask=15`)带来一个回退:rtkrcv 在没尝试模糊度固定的浮点历元上把 ratio 写成 0。**
   - A 的 401 条、B 的 8 条浮点历元 ratio 全为 0。A0 的 401 条里 300 条 ratio 在 1.0–3.0,其余 101 条为 0。
   - `diag_node_support.hpp` 把 ratio ≤ 0 当作"源不提供",`ambiguity` 规则因此永远不会触发:A0 有 2 次 ambiguity 事件,A 为 0。今晚不改诊断代码,只记录(§7 问题 16)。
7. **诊断事件里的"现象"大多属实,"原因"多半说不通。**
   - A 的 `low_sats` 归因为"疑似遮挡",但同期 ref 有 20 颗卫星。
   - B 的 `cycle_slip` / `multipath` 从启动宽限期一结束就一直开到退出:截至数据终点约 436 s,事件记录时长 446.7 s,含数据结束后的尾部。同期 rtkrcv 100% 固定,与 ref 只差 2 mm。
   - 每遍末尾的 `corr_outage` / `no_solution` 是回放在数据结束后多走 10 s `/clock` 造成的,不是数据里的现象。
   - `diag/base_baseline` 四遍都没生成:`base_warmup_s` 600 s 比数据长 497.3 s 还长(§6)。
8. **还有几条信息性的发现:**
   - 610 的 CAN 质量标签全程是 FIXED,而天线处前 12 个历元其实是浮点,可见 610 的融合标签不等于当历元的 RTK 状态。
   - 驱动不填 `RtkFix.ratio`。
   - `cgi610.dat` 里夹着 RTCM 帧,而且 RANGECMPB 的时间会往回跳,最多 190 ms。
   - convbin 读 NovAtel 格式要用 `-r nov`。
   - `gnss_core` 的 `read_pos`(`calibrate_sigma_scale` 也在用)不认 rnx2rtkp 默认的"周 + 周内秒"时间列:直接读 `rtk_check.pos` 得 0 条记录,退出码 3。

---

## 2. 数据段与回放方式

### 2.1 数据段

来源:`inventory.md` 与 plan context。

| 项 | 内容 |
|---|---|
| 时段 | 2026-09-15 08:49:31.000–08:57:48.300 UTC(GPST 08:49:49–08:58:06),497.3 s;GPS 周 2436 |
| `gnss/base.rtcm3` | 接收机实际收到的平台差分转发,375833 B:1006 ×50(每 10 s 一条,站号 1024);MSM4 1074 ×498、1084/1094/1114/1124 各 ×497;**没有星历电文** |
| `raw/cgi610.dat` | 610 NovAtel 二进制,5467909 B:RAWIMUSB 100 Hz、INSPVAXB 10 Hz、RANGECMPB 1 Hz、GALEPHEMERISB 13 条、BDSEPHEMERISB 1 条;**没有 GPS / GLONASS 星历**;夹带约 378 kB RTCM3 帧 |
| `raw/can7.candump.log` | 273514 行,只有 11 个 ID(0x320–0x32E 中缺 0x326/0x328/0x32B),约 50 Hz |
| `gnss/rtk_check.pos`(ref) | `rnx2rtkp -p 2 -f 2 -sys G,E,C`,rover.obs + base.obs + rover.nav;498 历元,Q1 486 / Q2 12;首次固定 GPST 204601(数据起点 +12 s) |
| 车速 | 最大 5.54 m/s,均值 2.67 m/s(CAN 离线解码) |

三路数据的起点:t0 = 1789462171.000(UTC unix)。RTCM 与 NovAtel 同为这一刻,CAN 抓包晚 2.2 ms。

### 2.2 回放链路

```
base.rtcm3 (+B: rover.nav 编码的 1019/1020/1042/1045/1046,开头一次,之后每 30 数据秒一次)
   ──TCP 15031──┐
cgi610.dat ──TCP 15032──┴─ rtcm_bridge ─ rtkrcv_node(rtkrcv,obs 格式 oem4)─ /rtkrcv_node/rtk_fix
can_with_sigma.log ─ canplayer ─ vcan0 ─ gnss_chcnav_can(timestamp_source=gps)─ /gnss_cgi610/rtk_fix
field_replay 50 Hz /clock ─ gnss_diag_node(use_sim_time)
pos_writer → pos/20260915/{can,rtkrcv}.pos;gnss_diag → diag/;record_gnss.sh → bags/
```

- 回放速度 1×,各路按数据自带时间调度。
  - 发送的是文件原始字节,按文件顺序发。
  - 块时刻取帧时间的累计最大值,以吸收 RANGECMPB 的时间倒退。
  - 开始前等 rtkrcv 连上 rtkrcv_node 的本机端口(`--wait-connected-ports`)。
- 隔离:`ROS_DOMAIN_ID=66`。每遍墙钟约 8 分 50 秒。四遍都没有进程残留,字节核对全部通过(各遍 `summary.md`)。
  - A / A0:差分 375833 B、观测 5467909 B,都与文件大小相等。
  - B / B0:差分 734023 B = 375833 B 数据 + 358190 B 注入星历(17 次 × 21070 B);观测 5467909 B。
- CAN 用的是 `can_with_sigma.log`:每个周期补 0x326/0x328/0x32B 三帧全零,共 24865 周期、补 74595 帧。
- 诊断节点用 sim time,所以 `events.log` 的时间就是数据时间。`startup_grace_s` 为 60 s,判定从数据起点 +60 s(08:50:31 UTC)开始。
- 修复前 A0/B0 的 conf:`pos1-elmask =10`,没有 `pos2-arelmask`、`misc-timeout`、`misc-reconnect`。
- 修复后 A/B 的 conf 多出三行:`misc-timeout =0`、`misc-reconnect =1000`、`pos2-arelmask =15`。见 `eval/*.md` 第 0 节。

---

## 3. 部件打通情况(Task 1–3)

| 部件 | 结论 | 依据 |
|---|---|---|
| driver_ws 编译 `gnss_msgs gnss_chcnav` | OK。首次失败是旧的非 symlink 构建目录冲突,把冲突目录移到 scratchpad 后编过,没有删任何东西 | inventory Step 2 |
| CAN 驱动 + vcan 回放原始日志 | **阻断**:60 s 回放 `rtk_fix` 0 条,日志 `incomplete CGI-610 cycle (mask 0x3B5F), dropped`。0x3B5F 正好缺 bit5/7/10,对应 0x326/0x328/0x32B | inventory 3.2 |
| CAN 离线解码(直接用驱动的 decoder,不要求 14 帧齐全) | 24865 周期,49.998 Hz;quality 100% RTK_FIXED;system_state=2 | inventory 3.3 |
| 补零 σ 帧后跑驱动 | 60 s 发 2999 条,50.000 Hz;`header.stamp == gnss_time`;`ratio` 恒为 0 | inventory 3.4 |
| convbin | `-r oem4` 不识别,须用 `-r nov`。498 历元;导出星历 E 13 / C 1 / **G 0** | inventory 4a |
| rnx2rtkp 复算 | 用 rover.obs 和 610 dat convbin 出来的观测分别复算,都是 486/498 固定;与 ref 逐历元同 Q,水平差 0.0000 m | inventory 4b |
| 1006 基站坐标 | 与 ref 头部 `ref pos` 的 ECEF 距离 0.0000 m。但 ref pos 本来就取自由这条 1006 写出的 base.obs 头,**不是独立校验** | inventory 4c |
| 回放工具(Task 2/3) | RTCM3/NovAtel 分帧与时间、candump 解析、星历编码、编排器、运行脚本 | 提交 `b4dba99`..`26cb18f`,review clean |
| 试跑 dry A 120 s(修复前) | rtkrcv 只有 23 条,Galileo-only,+46 s 起才有解 | Task 3 报告 |
| 试跑 dryB 120 s | 修复前 119 条全浮点(ratio ≤ 1.6);修复后 111/119 固定,首次固定 +8 s | Task 3 / Task 6 报告 |

---

## 4. A/B 结果对比(修复前后)

来源:`eval/{A0,A,B0,B}.md`,文件名分别是 `A_before_conf_fix.md`、`A.md`、`B_before_conf_fix.md`、`B.md`。

### 4.1 rtkrcv 解

| 指标 | A0(修复前) | A(修复后) | B0(修复前) | **B(修复后)** | ref |
|---|---|---|---|---|---|
| 解条数 | 401 | 401 | 497 | 497 | 498 |
| Q 分布 | Q2 360 / Q4 41 | Q2 360 / Q4 41 | Q2 497 | **Q1 489 / Q2 8** | Q1 486 / Q2 12 |
| 固定率 | 0% | 0% | 0% | **98.4%** | 97.6% |
| ns 范围 | 3–5 | 3–5 | 20–27 | 20–27 | 18–21 |
| 首解相对数据起点 | +97 s(08:51:08) | +97 s | +1 s(08:49:32) | +1 s | 0 |
| 首次固定 | 无 | 无 | 无 | **首解后 +8 s,即数据起点 +9 s(08:49:40)** | +12 s(08:49:43) |
| 双方都 FIXED 的配对数 | 0 | 0 | 0 | 486 | — |
| 水平差 中位 / p95 / max(m) | —(Q2/Q4 对 ref FIXED 全部:3.936 / 6.712 / 7.120) | 同 A0 | —(Q2 对 ref FIXED:0.299 / 0.445 / 0.477) | **0.002 / 0.005 / 0.009** | — |
| \|高程差\| 中位 / p95 / max(m) | —(全部:13.482 / 16.155 / 16.422;均值 −12.332) | 同 A0 | —(Q2 对 ref FIXED:0.690 / 3.291 / 4.348;均值 −1.048) | **0.004 / 0.014 / 0.072** | — |
| 配对 (rtkrcv Q, ref Q) | (2,1) 360、(4,1) 41 | 同 A0 | (2,1) 486、(2,2) 11 | (1,1) 486、(1,2) 3、(2,2) 8 | — |

- 配对规则:按时间一对一取最近,容差 0.1 s。差值 = rtkrcv − ref,在 ref 点的 ENU 下计算。
- B 有 3 个历元是 rtkrcv 已经固定、ref 还是浮点,所以首次固定比 ref 早 3 s。这 3 个历元不计入"双方都 FIXED"。
- A0 与 A 的 rtkrcv.pos 前 14 列(时间、位置、Q、ns、σ、age)逐条相同,`diff` 为 0 行,说明 A 的瓶颈是星历,不在配置。
  - 第 15 列 ratio 不同:A0 有 300 条在 1.0–3.0、101 条为 0;A 的 401 条全为 0。
  - 按行 diff 有 300 行不同,都只差 ratio。原因见 §7 问题 16。

### 4.2 诊断

| 指标 | A0 | A | B0 | B |
|---|---|---|---|---|
| events.log 事件数(OPEN) | 15 | 13 | 6 | 5 |
| 按代码 | ambiguity 2、corr_outage 1、cycle_slip 4、device_divergence 1、low_sats 1、multipath 4、no_solution 2 | 同 A0,但无 ambiguity(ratio 恒 0 使规则不触发,§7 问题 16) | ambiguity 1、corr_outage 1、cycle_slip 1、device_divergence 1、multipath 1、no_solution 1 | corr_outage 1、cycle_slip 1、device_divergence 1、multipath 1、no_solution 1 |
| 其中数据结束后才打开 | 2(corr_outage、no_solution) | 2 | 2 | 2 |
| `/gnss/diagnostics` 级别 | ERROR 410 / OK 60 / WARN 37 | 同 A0 | ERROR 442 / OK 60 / WARN 6 | ERROR 436 / OK 60 / WARN 11 |
| `/gnss/diagnostics` 主要 status_code | low_sats(ERROR)403 | low_sats(ERROR)403 | multipath(ERROR)388、ambiguity(ERROR)46 | multipath(ERROR)398、cycle_slip(ERROR)31 |
| `/rtkrcv_node/diagnostics` 级别 | OK 438 / WARN 88 | OK 436 / WARN 83 | OK 528 | OK 522 |
| `diag/base_baseline` | 不存在 | 不存在 | 不存在 | 不存在 |

- `/gnss/diagnostics` 里 OK 的 60 条全部是启动宽限期。
- 级别与 status_code 取法不同:级别取所有结论里最严重的一条,message 和 status_code 取优先级最高的一条。这是设计行为,见 `gnss_bringup/include/gnss_bringup/diag_node_support.hpp` 的 `make_diagnostic_status`,并有测试 `LevelIsTheWorstVerdictAndMessageIsTheStatusVerdict`。
  - 所以 B 里"multipath 是 ERROR",其实是 multipath(warning)与同时打开的 `device_divergence`(serious)叠在一起。
  - 在 B 的录包里抽了一条核实:`open_events=cycle_slip,device_divergence,multipath`。

---

## 5. 610 融合解与 RTK 的偏差及杆臂判断

来源:`eval/B.md` §3 与 §3.1。can.pos 四遍 md5 相同,A/A0/B0 的这一节数值也完全一样。
车辆外参来源:
- `/home/steve/Documents/Datasets/tage/hongshaquan/extrinsics_tage_truck.yaml`
- `/home/steve/Documents/Datasets/tage/hongshaquan/20260915/README_LIO.md` §4

### 5.1 数据

can 对 ref(ref FIXED,486 对):

| 项 | 中位数 | p95 | max |
|---|---|---|---|
| 水平差 m | 8.642 | 8.734 | 8.857 |
| \|高程差\| m | 5.153 | 5.270 | 5.437 |

ENU 分量(can − ref)的均值与标准差:

| 分量 | 均值 m | 标准差 m |
|---|---|---|
| 东 | −2.059 | 6.285 |
| 北 | −1.561 | 5.366 |
| 天 | −5.139 | 0.091 |

车体系分解。航向取录包 `/gnss_cgi610/rtk_fix.heading`,即 610 航向,24864 条全部 `heading_valid=true`;这里不判断它是否来自双天线。速度由同一话题 50 Hz 位置在 t±0.5 s 的位移求得。本段航向圆周标准差 88.6°,转向充分,足以区分车体系偏移与地理系偏移。

| 子集 | n | 前 均值 m | 前 标准差 m | 右 均值 m | 右 标准差 m | 东 标准差 m | 北 标准差 m |
|---|---|---|---|---|---|---|---|
| 全部 | 486 | −8.448 | 0.051 | 1.860 | 0.070 | 6.285 | 5.366 |
| 速度 > 1 m/s | 348 | −8.451 | 0.060 | 1.876 | 0.077 | 6.403 | 5.839 |
| 速度 < 0.2 m/s | 133 | −8.441 | 0.012 | 1.819 | 0.012 | 0.104 | 0.131 |

- **前向差对速度回归**(n=485):前 = −8.442 m + (−0.0022 s)× 速度。斜率只有 −2 ms,不是时间偏差。
- **航向与航迹向之差**(速度 > 1 m/s,n=348):中位数 0.21°,|差| 的 p95 为 4.82°。610 航向与行驶方向一致,航向约定(北零顺时针)正确,车体系分解可信。
- **扣除车体系固定偏移后的水平残差**(n=486):中位数 0.056 m、p95 0.170 m、最大 0.298 m。偏移是从同一批数据估出来的,这只是自洽检查。
- **不用录包也能复现。** 退回用 can.pos 航迹向(只取速度 > 1 m/s,n=347),得到前 −8.445 m、右 +1.876 m,标准差 0.098 / 0.329 m,航向圆周标准差 153.9°。
  - 来源:`eval/A0_nobag.md` §3.1,即不 source ROS 对 `run_A_before_conf_fix` 运行 `field_eval.py`,命令见 §10。

### 5.2 判断

- **是固定在车体上的偏移,不是融合发散,也不是基准差。**
  - 如果偏移固定在地理坐标系,ENU 标准差会小。实际 ENU 标准差达 5–6 m,因为车一转弯偏移就跟着转。
  - 换到车体系后标准差只有 5–7 cm;静止时 1 cm。
  - 高程偏移 −5.14 m 全程稳定(标准差 9 cm)。
- **这就是已知的外参杆臂,实测确认了它。**
  - `extrinsics_tage_truck.yaml`:base_link 原点 = CGI-610 输出参考点"目标点" = 后轴中心正下方地面,INSPVAXB / CAN 输出都是这个点。`T_base_link_gnss_antenna` = (8.45, 1.85, 5.16)m(前左上),由接收机网页杆臂取反得到。文件注明已用 2026-09-15 16:49 段事后 RTK 固定解验证,实测(前 8.439, 左 1.842, 上 5.153),差 1 cm 以内。`README_LIO.md` §4 的说法相同。
  - 本次的差值定义是 can − ref,即 base_link 减天线。所以天线在 base_link 中的位置 = −(前 −8.448, 右 +1.860, 天 −5.139)= **(前 8.448, 左 1.860, 上 5.139)m**。

    | 来源 | 前 m | 左 m | 上 m |
    |---|---|---|---|
    | 外参 `T_base_link_gnss_antenna`(网页杆臂) | 8.45 | 1.85 | 5.16 |
    | 外参文件记录的 16:49 段事后 RTK 实测 | 8.439 | 1.842 | 5.153 |
    | **本次实测**(`eval/B.md` §3.1,n=486) | **8.448** | **1.860** | **5.139** |
    | 本次 − 网页杆臂 | −0.002 | +0.010 | −0.021 |
    | 本次 − 16:49 段实测 | +0.009 | +0.018 | −0.014 |

  - "上"取的是 ENU 天向均值,没有按俯仰/横滚旋到车体 z 轴;车身基本水平,差别在厘米以下。
  - 复核组另外解码了 INSPVAXB,核实了三点:CAN 高程是椭球高(MSL 668.673 m + 大地水准面差距 −58.034 m = 610.639 m),与 ref 同为椭球高;航向约定正确;静止历元偏移相同,没有时间误差。
- **影响:**
  1. `gnss_diag` 的 `device_divergence` 直接比较 610 的 base_link 位置与 rtkrcv 的天线位置,不是同一个点,所以在这台车上**结构性地一直是 serious**。"疑似 610 融合问题"的结论是错的。
     - B 在宽限期结束约 6 s 后(08:50:37.780)打开,一直开到退出;事件行记的是打开时 8.65 m、峰值 8.858 m。
     - A 要等 rtkrcv 出首解后才打开(08:51:16.760),打开时 7.73 m、峰值 14.2 m;A 的峰值里还叠加了 rtkrcv 自己几米的误差。
     - 修法:比较前用已知杆臂和 610 航向,把 610 的 base_link 位置换到天线(或把 rtkrcv 换到 base_link)。见 §9。
  2. 610 输出点(base_link)是 GT(`gt/gt_base_link.tum`)、外参和 LIO 数据共同依赖的参考点,**不应改动 610 的输出点**。用天线级 RTK(rtkrcv、rtk_check)与 GT 或 610 对比时,要加上这个杆臂。

---

## 6. 诊断事件是否可信

来源:`eval/*.md` §5.1 逐事件核对与 §5.2 原文。判断依据是同期 ref / rtkrcv / can 的实际情况。"合理 / 可疑"只针对事件描述的现象与原因,不是对诊断算法的整体结论。

### 6.1 B(修复后)

| UTC 打开–关闭 | 代码 | 同期数据 | 判断 |
|---|---|---|---|
| 08:50:31.780–08:57:58.500(shutdown) | cycle_slip | rtkrcv 437 条全 Q1,与 ref 水平中位 0.002 m;ref 固定率 100% | **原因可疑,解算没受影响。** 宽限期一结束就打开,开满全段:截至数据终点 436 s,事件记录时长 446.7 s;pos 层面否定不了失锁本身 |
| 08:50:32.780–08:57:58.500(shutdown) | multipath(G29、E30) | 同上 | **现象大概率属实,不影响解算。** 离线调查查到 G29、E07、E10、E12、E30 高度角 10–15°,伪距多路径 2–20 m;事件点的正是其中两颗(录包里还见到 E07、C01) |
| 08:50:37.780–08:57:58.500(shutdown) | device_divergence(serious,8.65 m) | can − ref 水平中位 8.64 m,rtkrcv − ref 0.00 m | **偏差属实,原因可疑。** 扣除车体系固定偏移后残差中位 0.06 m;偏移就是已知外参杆臂(§5),诊断没有换到同一点比较 |
| 08:57:51.760–08:57:58.500 | corr_outage(serious) | 数据已于 08:57:48 结束 | **可疑。** 这是回放 `--tail-clock-s 10` 在数据结束后多走 `/clock` 造成的;实车上差分真断了会同样报 |
| 08:57:53.760–08:57:58.500 | no_solution | 同上 | **可疑**,原因同上 |

### 6.2 A(修复后;A0 基本相同)

| UTC 打开–关闭 | 代码 | 同期数据 | 判断 |
|---|---|---|---|
| 08:50:31.760–08:51:18.760 | no_solution | 打开前 5 s 内确实没有 rtkrcv 解;首条解在 08:51:08 | **合理。** 但首条解出现后 10.8 s 才关闭,关闭有滞后 |
| 08:51:08.760–shutdown | low_sats(serious,3 颗) | rtkrcv ns 中位 4;同期 ref ns 中位 20、固定率 100% | **现象属实,原因可疑。** 卫星少是因为缺 GPS 星历,不是"疑似遮挡" |
| 08:51:16.760–shutdown | device_divergence(serious) | can − ref 8.64 m;rtkrcv − ref 4.00 m | **偏差属实,原因可疑**,同 B |
| 4 次 multipath、4 次 cycle_slip(recovered) | — | rtkrcv 只有 3–5 颗 Galileo,与 ref 水平中位 1.2–7.0 m | **无法判定**,pos 层面验证不了 |
| 数据结束后的 corr_outage、no_solution | — | — | **可疑**,回放尾部产物 |

A0 另有两次 `ambiguity`(ratio 1.1、1.0)。rtkrcv 确实没固定,但同期 ref 固定率 100%,所以"遮挡过渡区常见"这个原因不成立。B0 的 `ambiguity`(ratio=1.3,开满全段)同理,根因是 elmask 配置。
A 没有 `ambiguity` 事件,不是因为情况变好,而是 ratio 恒为 0 让规则失效(§7 问题 16)。

### 6.3 rtkrcv_node 诊断

- A 有 83 条 WARN,A0 有 88 条,都是"N s 没有解算输出,上行有数据但无解——检查 base_pos_type 所需的 RTCM 1005/1006 与 obs_format"。
- 但本段 1006 与 obs_format 都正常,真正原因是缺星历。**提示文案会把人引向错误方向**,建议加上"星历"(§9)。
- B 全部 OK。

### 6.4 基站坐标

- `diag/20260915/base.pos` 四遍都只有 1 条记录,时间 08:49:32 UTC,ECEF (−20601.3842, 4555556.3139, 4449890.0097),与 ref pos 距离 0.0000 m。
- 这只证明 1006 解析正确:ref pos 本来就来自同一条 1006,不是独立校验。
- `diag/base_baseline` 四遍都不存在。`base_warmup_s` 为 600 s,数据只有 497.3 s,基线学习从未完成,**这个功能本次没有验证到**。

---

## 7. 发现的问题与处理

| # | 问题 | 证据 | 状态 |
|---|---|---|---|
| 1 | **610 CAN 不发 σ 帧 0x326/0x328/0x32B,`gnss_chcnav` 要求 14 帧齐全(`COMPLETE_MASK 0x3FFF`),整周期丢弃,实车零输出** | inventory 3.1/3.2;同日 `mdc-20260915-153000`、`mdc-20260915-165000` 两次抓包相同 | **未修复**(约束:不改 finder_ros)。可选:① 在 610 上打开 CAN 806/808/811 输出(根本解决);② 驱动放宽判定,缺 σ 时照常发布,σ 标为未知;③ 回放侧补帧(本次用的办法,只适合打通链路);④ 用 decoder 另写发布器。**现场阻断,请维护者定方案** |
| 2 | 610 原始输出缺 GPS(和 GLONASS)星历;平台差分流也没有星历电文 → 原样回放(A)从未固定 | inventory 4a(convbin 导出 G 0);eval/A.md:3–5 颗,0 固定 | **未修复**(属板卡输出配置)。建议 610 串口加星历日志,或平台转发 1019/1020/1042/1046。B 用当天 rover.nav 注入,证明补上星历就能固定 |
| 3 | elmask 10、不设 AR 门限 → 全程浮点(ratio 1.1–1.6) | investigation_float/REPORT.md;B0 0/497 | **已修复**:`a911a91` 新增 `ar_elmask` → `pos2-arelmask`,默认 15。B 489/497 |
| 4 | rtkrcv tcpcli 输入空闲 10 s 断开、10 s 后重连;断开期间 LocalReserver 静默丢字节 | Task 3 报告;Task 6 独立实验(旧 conf 10.02 s timeout、20.03 s 才重连) | **已修复**:`e7c7895`(`misc-timeout=0`、`misc-reconnect=1000`);`d2d97c3`(没有 rtkrcv 接收时节流 WARN,并带累计丢弃字节数);`4ccf694`(回放改为等"已连上") |
| 5 | 驱动从不填 `RtkFix.ratio`(恒 0);CAN 质量 100% FIXED,而天线处 ref 前 12 历元是浮点 | inventory 3.3/3.4;eval §1 | **未修复**,信息性。610 融合标签不能当作当历元 RTK 状态;`ambiguity` 规则只能用 rtkrcv 的 ratio |
| 6 | `cgi610.dat` 夹带约 378 kB RTCM3 帧和 `\n`;RANGECMPB 头部时间最多倒退 190 ms(508 处) | inventory 4a、"三路数据时间" | **已在回放工具中处理**(按文件顺序发原始字节,块时刻取累计最大值)。任何按时间解析 dat 的工具都要注意 |
| 7 | `base_warmup_s` 600 s > 数据 497.3 s,基线从未学到 | 四遍都没有 `diag/base_baseline` | **未验证**。需要更长的数据段,或回放时临时调小 `base_warmup_s` |
| 8 | convbin 的 NovAtel 格式名是 `nov`,不是 `oem4`(rtkrcv 仍是 `oem4`) | inventory 4a | 已记录;计划与工具里的命令已按 `-r nov` 写 |
| 9 | `device_divergence` 拿 610 的 base_link 位置(后轴中心正下方地面)直接比 rtkrcv 的天线位置,相差已知杆臂 (8.45, 1.85, 5.16) m(前左上),结构性地常驻 serious。实测杆臂 (8.448, 1.860, 5.139) 与外参吻合到 1–2 cm | §5;eval/B.md §3.1;`extrinsics_tage_truck.yaml`;`README_LIO.md` §4 | **未修复**。`gnss_diag` 在比较前需用杆臂 + 610 航向把 610 位置换到天线点(或把 rtkrcv 换到 base_link);610 输出点不动 |
| 10 | 诊断归因不准:low_sats 归为遮挡;ambiguity 归为遮挡过渡区;B 的 cycle_slip/multipath 开满全段而解算完好;no_solution 关闭滞后约 11 s | §6 | **未修复**,需要更多数据评估阈值 |
| 11 | `gnss_core` 的 `read_pos` 与 `export_bag_to_pos.load_epochs_from_pos` 只认"日期 时间"列;`calibrate_sigma_scale rtk_check.pos …` 输出 `ref … (0 records)`,退出码 3 | 本次实测 | **未修复**(本计划外)。`field_eval.py` 自己解析两种格式并带单测;建议 `read_pos` 支持"周 周内秒",或文档要求 rnx2rtkp 加 `-t` |
| 12 | rtkrcv_node 无解 WARN 只提示 1005/1006 与 obs_format,不提星历 | §6.3 | **未修复**,建议补充文案 |
| 13 | 回放 `--tail-clock-s 10` 让诊断节点在数据结束后报 corr_outage / no_solution | §6 | 属回放产物,不是缺陷;`field_eval.py` 会标出"数据结束后才打开" |
| 14 | 驱动在 `timestamp_source=gps` 时 `header.stamp` 是数据日时间,墙钟下早约 8.25 h;CAN 抓包时间比板卡 GPS 时间早约 9.5 ms(中位数) | inventory 3.4、"三路数据时间" | 信息性。整链路必须用 sim time 或 `gnss_time`;50 Hz 级时间对齐分析要注意 |
| 15 | Task 4 两份报告与 run 摘要把"首解 +19 s(B)、+115 s(A)"写成相对数据起点,实际是拿 GPST 时刻减了 UTC 起点 | 本文 §4.1 按 eval 更正 | **已更正**:实际是 +1 s(B)、+97 s(A) |
| 16 | **`a911a91` 引入的回退**:`pos2-arelmask=15` 后,rtkrcv 在没尝试模糊度固定的浮点历元上把 ratio 写成 0(A 401/401、B 8/8)。`diag_node_support.hpp` 把 ratio ≤ 0 映射为"源不提供",`ambiguity` 规则(`diagnosis.cpp`,要求 FLOAT 且 ratio 存在且 < min_ratio)永远不触发:A0 有 2 次,A 为 0 次 | 对 `run_A_before_conf_fix` 与 `run_A` 的 rtkrcv.pos 做 diff:前 14 列 0 行不同,第 15 列 ratio A0 为 300 条 1.0–3.0 + 101 条 0,A 全为 0;B0 浮点 497 条中 489 条 ratio ≥ 1.0,B 的 8 条浮点全为 0 | **未修复**(控制者裁定今晚不改诊断代码)。可选:① ratio 缺失时,ambiguity 规则按"持续 FLOAT 超过 N s"触发;② rtkrcv_node 区分"AR 未尝试"与"源不提供 ratio"(如单独字段或诊断值),诊断据此判定;③ 取 rtkrcv stat 里的 AR 信息 |

---

## 8. 对 spec §13 P0 待确认项的新认识

spec §13 "平台差分协议、板卡原始格式未确认(P0)"这一项,本段数据给出了以下事实。都是这一辆车、这一天的情况,换车或换配置后要复核。

- **差分格式**
  - 裸 TCP 转发的 RTCM3 MSM4:1074 / 1084 / 1094 / 1114 / 1124,每秒一组。
  - 1006 基站坐标每 10 s 一条,站号 1024,天线高 0;流开头第 2 个历元之后才出现第一条 1006。
  - **没有任何星历电文**,也没有 1005 / 1033。`inpstr2-format = rtcm3` 可用。
- **板卡原始格式**
  - NovAtel OEM 二进制。rtkrcv 用 `inpstr1-format = oem4`,convbin 用 `-r nov`。
  - 观测来自 RANGECMPB,1 Hz。北斗只有 B1I 单频(C2I / L2I);GPS 有 L1/L2/L5,Galileo 有 E1/E5b/E5a,GLONASS 有 G1/G2。
- **板卡输出配置缺星历**
  - dat 里只有 GALEPHEMERISB(13 条)和 BDSEPHEMERISB(1 条),没有 GPS / GLONASS 星历。
  - 差分流里也没有,所以按现在的配置,实时 RTK 几乎只能用 Galileo(A:3–5 颗,从未固定)。**实车部署前必须补一个星历来源。**
- **CAN 输出配置缺 σ 帧**(806/808/811),驱动零输出。见 §7 问题 1。
- **610 融合解的参考点是 base_link**(后轴中心正下方地面),不是 GNSS 天线。天线在 base_link 的(8.45, 1.85, 5.16)m(前左上),本次实测(8.448, 1.860, 5.139)m,确认到 1–2 cm(§5)。
  - base_link → 天线这段杆臂**已验证**。用 610 融合解做对照或 `device_divergence` 时,必须先把两路换到同一点。
  - spec §13 "IMU-GNSS 杆臂未标定"这一项**仍然开着**:yaml 里的 `T_base_link_cgi610_imu` = (6.85, 1.00, 2.28)m 是由接收机网页杆臂推算的,标注"未验证"。所以 IMU → 天线杆臂(base_link 轴向)= (1.60, 0.85, 2.88)m 也未验证,仍需 §9.3 工具用建图数据估计。
- **rtkrcv ratio 语义**:开 `pos2-arelmask` 后,没有尝试 AR 的浮点历元 ratio 为 0。下游不能把 0 一律当"源不提供"(§7 问题 16)。
- **质量标签**:610 的 `satellite_status` 是融合后的标签,开头天线处还是浮点时它已报 FIXED,也不提供 ratio。`quality_sigma_scale` 标定时不能把 610 的 FIXED 当作天线级固定。
- **rtkrcv 配置**:在这类有低高度角多路径的环境里,要么 `pos1-elmask` 设到 15°,要么保持 10° 并加 `pos2-arelmask = 15`(已设为默认)。rnx2rtkp 默认的 elmask 是 15°,两边不能想当然视为一致。
- **基站坐标**:ref pos 与 1006 一致,但 1006 是平台下发的坐标,绝对精度没有独立核验。

---

## 9. 下一步建议

按优先级:

1. **CAN σ 帧(阻断)。** 请维护者二选一:车上 610 打开 CAN 806/808/811,或授权修改 `gnss_chcnav`,让它在缺 σ 帧时照常发布。前者改完后抓 1 分钟 candump,确认 14 个 ID 都在。
2. **星历来源。** 在 610 原始输出口加 GPS/GLONASS/BDS/Galileo 星历日志,或请平台在差分流里转发星历电文。改完抓一段 dat,用 `convbin -r nov` 核对导出星历里 G > 0。
3. **`gnss_diag` 的 `device_divergence` 改为同点比较。** 用已知杆臂 `T_base_link_gnss_antenna` = (8.45, 1.85, 5.16)m 和 610 航向,把 610 的 base_link 位置换到天线点(或把 rtkrcv 换到 base_link)后再算偏差;杆臂做成参数。
   - 本段扣除杆臂后残差中位 0.056 m、p95 0.170 m,可用作阈值参考。
   - 在改之前,这台车上 `device_divergence` 结构性地常驻 serious,没有参考价值。
   - 610 输出点不要改:GT、外参和 LIO 都依赖 base_link。
4. **修 `a911a91` 带来的 ambiguity 规则失效**(§7 问题 16):在"ratio 缺失时按持续 FLOAT 触发"和"rtkrcv_node 报告 AR 未尝试"之间选一个,并补测试;修完用 A 重跑,确认 ambiguity 事件恢复。
5. **验证 IMU → 天线杆臂**:`T_base_link_cgi610_imu` 是网页杆臂推算的,未验证,用 §9.3 工具估计。
6. **更长的数据段复测**:600 s 以上,最好含遮挡、出入隧道。验证 `base_baseline` 学习,以及 `cycle_slip` / `multipath` / `no_solution` 的阈值和关闭滞后。建议用同一天另一段(如 `mdc-20260915-153000`)做独立复测。
7. `gnss_core` 的 `read_pos` 支持 rnx2rtkp 默认的"周 + 周内秒"时间列(或在工具文档里写明要加 `-t`),避免 `calibrate_sigma_scale` 读到 0 条。
8. rtkrcv_node 的无解 WARN 文案加上"检查星历"。
9. 诊断归因:`low_sats` 在"输入卫星少、天空不遮挡"时不应报"疑似遮挡";考虑用 rtkrcv 的星历可用卫星数区分。
10. 可选:把 `field_eval.py` 的航向/杆臂分析沉淀进 `gnss_core` 工具,用于实车杆臂标定前的快速检查。

---

## 10. 复现方法

```bash
SEG=/home/steve/Documents/Datasets/tage/hongshaquan/20260915/seg_164931_165748
T=/home/steve/glim_ws/src/glim_ext/gnss_bringup/tools/field_replay

# 0. 编译(glim_ext 分支 feat/field-replay-20260915)
cd /home/steve/glim_ws && source /opt/ros/humble/setup.bash && colcon build --symlink-install --packages-up-to gnss_bringup
cd /home/steve/driver_ws && colcon build --symlink-install --packages-select gnss_msgs gnss_chcnav
# 前置:vcan0 已建好且 UP(需要 sudo,由维护者操作,见 $T/README.md);can-utils;RTKLIB-EX 2.5.1 rtkrcv

# 1. 整链路两遍(各约 9 min;脚本会先清空 run_A / run_B,修复前的存档在 run_*_before_conf_fix)
ROS_DOMAIN_ID=66 $T/run_field_integration.sh A $SEG
ROS_DOMAIN_ID=66 $T/run_field_integration.sh B $SEG
#    试跑:ROS_DOMAIN_ID=66 $T/run_field_integration.sh dryB $SEG 120

# 2. 评估(读录包要先 source 三层环境)
source /opt/ros/humble/setup.bash; source /home/steve/glim_ws/install/setup.bash; source /home/steve/driver_ws/install/setup.bash
for r in A B A_before_conf_fix B_before_conf_fix; do
  python3 $T/field_eval.py --run $SEG/integration_20260916/run_$r --ref $SEG/gnss/rtk_check.pos \
    --out $SEG/integration_20260916/eval/$r.md --label $r
done
#    不读录包的对照(航向退回 can.pos 航迹向):不 source ROS
env -i HOME=$HOME PATH=/usr/bin:/bin python3 $T/field_eval.py --run $SEG/integration_20260916/run_A_before_conf_fix \
  --ref $SEG/gnss/rtk_check.pos --out $SEG/integration_20260916/eval/A0_nobag.md --label "A_before_conf_fix(无 ROS 环境,航向退回 can.pos 航迹向)"
#    ratio 列对比(§7 问题 16)
diff <(grep -v '^%' $SEG/integration_20260916/run_A_before_conf_fix/pos/20260915/rtkrcv.pos | awk '{NF=14; print}') \
     <(grep -v '^%' $SEG/integration_20260916/run_A/pos/20260915/rtkrcv.pos | awk '{NF=14; print}') | wc -l   # 0
grep -v '^%' $SEG/integration_20260916/run_A/pos/20260915/rtkrcv.pos | awk '{print $15}' | sort | uniq -c           # 401 0.0

# 3. 离线参照(inventory 4a/4b)
convbin -r nov $SEG/raw/cgi610.dat -o cgi610.obs -n cgi610.nav
REF=$(grep "APPROX POSITION" $SEG/gnss/base.obs | awk '{print $1,$2,$3}')
rnx2rtkp -p 2 -f 2 -sys G,E,C -r $REF -o cgi610.pos cgi610.obs $SEG/gnss/base.obs $SEG/gnss/rover.nav

# 4. 测试(不要与 domain 66 的回放同时跑)
python3 -m unittest discover -s $T/tests
cd /home/steve/glim_ws && colcon test --packages-select gnss_core gnss_bringup && colcon test-result --all
```

测试结果(2026-09-16,提交 `6362e74` 之后):
- `colcon test-result --all`:**544 tests,0 errors,0 failures,1 skipped**。skip 是既有的 `test_local_reserver`。
- Python 套件在 ctest 里算 1 条(`test_field_replay_py`),其中 `Ran 116 tests`,含 `test_field_eval.py` 的 41 条。
