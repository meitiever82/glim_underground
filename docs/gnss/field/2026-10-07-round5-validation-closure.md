# 轮 5 验证闭环结果:环境重建与 LiDAR-only 基线(Task 1)

- 日期:2026-10-07。本文由 Claude Sonnet 5(Task 1)撰写,依据下面列出的实际命令与输出整理,未凭空补数。
- 计划:`.superpowers/sdd/2026-10-07-round5-validation-closure/{task-1-brief,global-constraints}.md`
- 代码:`glim_underground` 分支 `feat/round5-validation-closure`,从 `master` `c20b111` 开出,**未 push**。本 Task 的配置集提交:`5bc1f7e`(`config: add tage_at128 config set (AT128P + CGI-610, bag_lio)`)。
- 数据段 A:`<segA>` = `/home/steve/Documents/Datasets/tage/hongshaquan/20260915/seg_164931_165748`。数据只读,本 Task 的产物都在 `<segA>/validation_20261007/`(`run_C/`、`eval/`)。
- 本文只写 §1(环境与构建)与 §2(C 组基线)两节,对应 Task 1 Step 1–13;§9.2/§9.3/§12.2/§12.3/§12.4 的其余部分由后续 Task 续写。

---

## 1. 环境与构建

### 1.1 工作区状态(重建前)

重建前 `/home/steve/glim_ws/build`、`/home/steve/glim_ws/install` 均不存在(与计划预期一致)。`src/` 下各仓库状态:

- `glim_underground`:`master`,HEAD `c20b111`,工作区有两处与本轮无关的未提交改动 `include/glim/util/time_keeper.hpp`、`src/glim/util/time_keeper.cpp`(按约束全程未 `git add` 它们)。
- `glim_ext`:`master`,HEAD `fb21d3f`,工作区干净。

### 1.2 Step 1:构建

实际使用的命令(**未触发 CUDA/nvcc 编译错误,没有用到 `-DBUILD_WITH_CUDA=OFF` 的退路**,工作区仍按 `BUILD_WITH_CUDA=ON` 默认构建):

```bash
cd /home/steve/glim_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to glim_ros glim_ext gnss_bringup
```

实际输出(尾部):

```
--- stderr: glim_ros
In this package, headers install destination is set to `include` by ament_auto_package. ...
---
[Processing: glim_ros]
Finished <<< glim_ros [1min 24s]

Summary: 6 packages finished [3min 17s]
  6 packages had stderr output: glim glim_ext glim_ros gnss_bringup gnss_core gnss_msgs
```

6 个包(`gnss_msgs`、`gnss_core`、`gnss_bringup`、`glim`、`glim_ext`、`glim_ros`)全部构建成功,0 失败。6 个包的 stderr 均为 CMake deprecation / dev 警告(`CMP0167`、`ament_auto_package` 头文件安装位置建议等)与一个 gtest 的 `-Wdangling-else` 编译警告,均非错误,不影响产物。总耗时 3 分 17 秒(远低于计划预估的 20–40 分钟,RTX 4060 + 本机已装好的 GTSAM/gtsam_points 使增量构建很快)。

### 1.3 Step 2:构建产物检查

```bash
cd /home/steve/glim_ws && source install/setup.bash
ls install/glim_ros/lib/glim_ros/glim_rosbag
ls install/glim_ext/lib/librtk_global.so install/glim_ext/lib/libgnss_global.so install/glim_ext/lib/librtk_odometry.so
ls install/gnss_core/lib/gnss_core/{calibrate_sigma_scale,estimate_lever_arm,synth_rtk_fix,gnss_report}
ls install/gnss_core/lib/gnss_core/{pos_to_rtkfix_bag.py,run_injection_suite.sh,traj_rmse.py}
```

十一个路径全部存在(`librtk_global.so` 也在,说明 `ENABLE_RTK_GLOBAL` 默认 ON 生效,未需要重新带选项构建)。`gnss_core` 实为 `src/gnss_core -> glim_ext/gnss_core` 的符号链接,与 `glim_ext` 共用同一份源。

### 1.4 Step 3:既有测试基线

```bash
cd /home/steve/glim_ws && source /opt/ros/humble/setup.bash
which python3   # /home/steve/.pyenv/shims/python3 —— pyenv shim 确实存在
colcon test --packages-select gnss_core gnss_bringup --event-handlers console_direct+
colcon test-result --verbose
```

实际输出:

```
100% tests passed, 0 tests failed out of 21
...
Finished <<< gnss_bringup [1min 6s]

Summary: 2 packages finished [1min 14s]
```

```
Summary: 607 tests, 0 errors, 0 failures, 1 skipped
```

**607 tests / 0 failures,与 PR #7 + #8 合并后的基线数字完全一致。** 本次没有遇到计划提到的 `ModuleNotFoundError: No module named 'ament_cmake_test'`(pyenv python3 shim 问题)——尽管 `which python3` 确认 shim 确实在生效,但 `gnss_core`/`gnss_bringup` 的测试全部通过,未触发该问题;因此未使用 `PATH=/usr/bin:$PATH` 的退路。这与计划给出的提示不矛盾,只是本次环境下没有触发条件。

---

## 2. LiDAR-only 基线(C 组)

### 2.1 配置集

新建 `src/glim_underground/config/tage_at128/`,共 17 个文件(10 个从 `mid360/` 原样拷贝 + 7 个新写),已提交(`5bc1f7e`)。关键数值(逐字对照 `global-constraints.md`):

- `T_lidar_imu = [-1.886806, 1.006244, 0.651553, 0,0,0,1]`(旋转单位阵),与 `README_LIO.md` 给 FAST-LIO 的 `extrinsic_T = [1.886806, -1.006244, -0.651553]` 符号相反,互为校验,核对无误。
- 话题 `/at128/points` + `/cgi610/imu`,`acc_scale=1.0`,`ring_field="ring"`,`perpoint_relative_time=true`。
- `extension_modules: []`(C 组本来就不需要扩展模块,空列表天然满足)。
- `config.json` 显式列出 `config_rtk_global`/`config_rtk_odometry`/`config_gnss_global` 三个 glim_ext 模块配置路径,本 Task C 组运行未加载任何扩展模块(日志里只 `load libodometry_estimation_cpu.so`、`libsub_mapping.so`、`libglobal_mapping.so`,没有扩展模块),三个文件暂未被使用,留给后续 Task 4/7。

### 2.2 Step 11:跑 C 组

```bash
SEGA=/home/steve/Documents/Datasets/tage/hongshaquan/20260915/seg_164931_165748
OUT=$SEGA/validation_20261007
mkdir -p $OUT/run_C $OUT/eval
cd /home/steve/glim_ws && source install/setup.bash
ros2 run glim_ros glim_rosbag $SEGA/bag_lio \
  --ros-args \
  -p config_path:=/home/steve/glim_ws/src/glim_underground/config/tage_at128 \
  -p auto_quit:=true \
  -p dump_path:=$OUT/run_C \
  2>&1 | tee $OUT/run_C/glim.log
```

**一次跑通,没有为了跑通做任何配置改动**(`random_downsample_target` 维持 15000,没有降;CUDA 没有关;`config_sensors.json`/`config_preprocess.json` 与提交时一致)。日志里逐条确认:

```
[info] config_path: /home/steve/glim_ws/src/glim_underground/config/tage_at128
[info] topics:
[info] - /cgi610/imu
[info] - /at128/points
[info] - /image
[info] detected storage_id=sqlite3 from metadata.yaml
...
[glim] [info] playback speed: 3.1x–4.8x 区间波动(离线批跑,无回放节流)
[global] [warning] previous submap has only a small overlap with the current submap (0.05–0.24 之间多次出现)
[global] [warning] create a between factor to prevent the submap from being isolated
...
[global] [info] saving to .../run_C...
[global] [info] saved
```

`small overlap` 告警是正常现象(停车场绕圈行驶,相邻 submap 之间视场重叠本来就小,GLIM 自动插入 between 因子防止该 submap 被图孤立),不是回归。进程以 exit code 0 正常退出,未出现 `intensity`/`ring` 字段警告,也没有反复 `waiting for odometry estimation` 的节流(只在结尾、流程正常收尾时各出现一次 `waiting for ...`)。

产物行数:

```
wc -l run_C/traj_imu.txt   → 4934
wc -l run_C/traj_lidar.txt → 4934
```

均 ≥ 4000(AT128 原始帧数 4966,经 submap 抽稀后略少,符合预期),且不为 0。

### 2.3 Step 12:C 组 RMSE(验收断言)

```bash
SEGA=/home/steve/Documents/Datasets/tage/hongshaquan/20260915/seg_164931_165748
OUT=$SEGA/validation_20261007
cd /home/steve/glim_ws && source install/setup.bash
python3 install/gnss_core/lib/gnss_core/traj_rmse.py \
  $SEGA/gt/gt_cgi610_imu_flu.tum $OUT/run_C/traj_imu.txt --align --tol 0.05 \
  | tee $OUT/eval/C.md
```

实际输出(`eval/C.md` 全文):

```
n=4934 rmse=1.8341 mean=1.4961 median=1.1244 max=5.7882
```

**验收通过:`n`(脚本输出字段名是 `n`,不是计划文字里的 `n_pairs`,但含义相同——配对成功的样本数)= 4934 > 1000;`rmse` = 1.8341 m < 2.0 m。** 排查过程:断言第一次就过,未发现 `T_lidar_imu` 符号或 `acc_scale` 问题,§2.3 无需修复记录。

注:`rmse=1.8341` 距 2.0 m 的阈值不算宽裕——这是纯 LiDAR 里程计 + 子图/全局建图在 1329 m、497 s、无 GNSS 修正、无显式回环检测(C 组本来就不挂任何扩展模块)下的累积漂移,量级合理,但不应误读为"高精度基线"。它是 §12.4 A/B 两组的分母,A/B(加了 RTK/GNSS 约束后)理应明显优于这个数字,否则说明扩展模块没有产生实际效果。

### 2.4 与 FAST-LIO 的横向对比

`<segA>/fastlio_eval/fastlio_cgi_tum.txt`(cgi610_imu_flu 系,已有的 FAST-LIO 结果,4098 行)对同一份真值跑同一脚本:

```bash
python3 install/gnss_core/lib/gnss_core/traj_rmse.py \
  $SEGA/gt/gt_cgi610_imu_flu.tum $SEGA/fastlio_eval/fastlio_cgi_tum.txt --align --tol 0.05
```

输出:

```
n=4098 rmse=10.3169 mean=7.7790 median=5.2055 max=23.8845
```

| | n | rmse (m) | mean (m) | median (m) | max (m) |
|---|---|---|---|---|---|
| GLIM C 组(本 Task,`tage_at128` + CPU 里程计/建图) | 4934 | 1.8341 | 1.4961 | 1.1244 | 5.7882 |
| FAST-LIO(`fastlio_eval/fastlio_cgi_tum.txt`,既有结果) | 4098 | 10.3169 | 7.7790 | 5.2055 | 23.8845 |

GLIM 的 C 组(带子图/全局因子图优化)RMSE 比 FAST-LIO(纯滤波式 LIO,无后端优化)低约 5.6 倍。两者不是严格的同条件对比(FAST-LIO 跑的是另一次独立流程、不同的点云抽稀/关联参数),这里只作为"同一份真值下两套算法量级是否合理"的交叉检验,不作为算法优劣的正式结论。

### 2.5 本 Task 未做、留给后续 Task 的事

- `config_rtk_global.json`/`config_rtk_odometry.json`/`config_gnss_global.json` 三个文件已写入配置集但本次 C 组运行未加载,数值未经实车验证(`T_imu_gnss` 仍是占位 `[0,0,0]`,等 Task 5 的 `estimate_lever_arm` 实测填回)。
- A/B 组(`rtk_global` vs `gnss_global`)、注入套件、杆臂实测、`rtk_odometry` 开门均不在本 Task 范围内。
