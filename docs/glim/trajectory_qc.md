# Trajectory QC — 建图质量自检

在任何下游处理（上色、缺陷投影、配准）之前，先确认这份地图能不能用。

## 为什么需要

长直涵洞、隧道、走廊这类**几何退化**场景，横截面处处相同，沿轴向没有约束，
LiDAR 里程计会沿退化轴"滑动"。表现为轨迹在十几秒内瞬移几十米，而**每帧残差
依旧正常**——常规指标看不出来。

实测：某 163 m 涵洞数据在 t=317~334 s 的 16.5 秒内沿轴向瞬移 **39.6 m**，
峰值速度 4.12 m/s。四足机器人不可能跑这么快。该段之后的地图全局位置全部偏移。

## 判据

**主判据是运动学**：速度或加速度超出平台物理上限 → `pose_jump`。

> ⚠️ `EstimationFrame::v_world_imu` **不是独立的 IMU 观测**，而是与位姿联合优化的
> 状态量。位姿跳变时速度状态跟着一起跳，两者始终自洽——实测 39 m 跳飞段
> `|v_pose - v_imu|` 峰值 0.27 m/s，正常段 0.25 m/s，无区分度。
> 因此 IMU 失配**不能否决**运动学违例，仅作为"位姿与速度状态不一致"这类
> 少见故障的辅助信号。

| 事件类型 | 含义 |
|---|---|
| `pose_jump` | 速度/加速度超限 → 定位失效 |
| `attitude_jump` | 角速度超限 |
| `data_gap` | 帧间隔过长 |
| `fast_motion` | 触发阈值但仍在运动学范围内 |

## GUI：offline viewer

```
Tools → Trajectory QC
```

- 顶部彩色结论（PASS/FAIL）
- 阈值滑块，改完实时重算
- 事件表格，点击某行相机飞到现场
- 3D 轨迹按严重度着色：绿=正常 / 橙=可疑 / 红=跳飞
- `Export report` 导出报告

## CLI：headless

```bash
# 单个地图
ros2 run glim_ros trajectory_qc /path/to/map

# 批量筛查 + 导出报告
ros2 run glim_ros trajectory_qc /maps/*/ --quiet --report_dir /tmp/qc

# 按平台调阈值（默认按四足机器人）
ros2 run glim_ros trajectory_qc /path/to/map --max_speed 3.0 --max_accel 10.0
```

退出码：`0` 全部通过 / `1` 存在失败 / `2` 参数或 IO 错误 —— 可直接用于 CI 与批处理脚本。

输出 `<name>_qc.txt`（摘要 + 事件清单 + 事件起止坐标）与 `<name>_qc.txt.csv`（逐帧
速度/加速度/角速度，便于自行绘图）。

## 阈值参考

| 平台 | max_speed | max_accel |
|---|---:|---:|
| 四足机器人（默认） | 1.5 m/s | 6 m/s² |
| 轮式 AGV | 2.5 m/s | 8 m/s² |
| 车载 | 20 m/s | 10 m/s² |
| 手持 | 2.0 m/s | 10 m/s² |
