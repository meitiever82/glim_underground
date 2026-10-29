# 设计：RTK gauge 调查——F1(共享 gauge)已证伪回滚 / F2(FIXED-only bootstrap)保留

- 日期：2026-10-19（当日修订：初版"共享 gauge"设计被实车数据证伪）
- 状态：终稿
- 代码范围：`glim_ext/gnss_core`（anchor_pipeline / frame_aligner / gauge_registry）、两个 rtk 模块

## 0. 结论速览

| 项 | 初版设计 | 终版结论 |
|---|---|---|
| "双 gauge 矛盾 9.5m" | 判定为 bug，需共享 | **证伪**：两模块 ENU 原点不同（各自取首个 anchor 时刻的 fix 为原点），gauge 平移差 = R·原点差，**各自自洽，从未矛盾** |
| F1 共享 gauge | 主推 | **已回滚**（`shared_gauge` 默认 false，代码保留为实验开关）。共享数值而不统一原点会引入 R·Δorigin 系统偏差（seg B 9.4m、seg A ~2m） |
| F2 FIXED-only bootstrap | 次要 | **保留**（`aligner_min_quality` 默认 4）。FLOAT 游走不进冻结基准，属正确的鲁棒性改进 |

## 1. 调查全过程（证据链）

1. **起点**：seg B 日志里两模块打印的 T_world_enu 平移差 9.5m → 初判"gauge 矛盾"。
2. **初版修复（F1 共享 + F2 FIXED-only）上线后**：V1-V4 实测——结构目标达成（两模块打印同一值）但 ATE 拧巴：seg B f4 改善（1.228→0.956）、seg B f1 变差（0.892→1.123）、seg A f4 回归（0.324→0.484）。
3. **复算推翻初判**：用 rtk_global 的实际 anchor 数据（调试日志打印的 submap origin+stamp，与 odom 轨迹逐点核对无误）做两点 gauge 拟合：**yaw 与模块打印完全一致（-14.88° vs -14.89°），平移差 = R·(8.76, 4.39)**——正好是 rtk_global 的 ENU 原点（其首个 anchor 在 t≈15.6s 的 fix 位置）与 rtk_odometry 的 ENU 原点（t≈0s 的首个 fix）之差。
4. **机理确认**：`AnchorPipeline` 每个实例以"本模块首个过门限 anchor 时刻的插值 fix"为 ENU 原点。两模块原点不同 → 各自的 gauge 平移天然不同 → 各自因子在各自原点下自洽。**所谓矛盾是不存在的。**
5. **F1 的实际效果**：`adopt()` 只共享 gauge 数值、不统一原点 → rtk_global 的 37 个因子背上 R·Δorigin ≈ 9.4m 系统偏差。ATE 变化完全由这个偏差解释：f4 下 9.4m 超 Huber δ=5.4m → 因子被线性截断近似打废（等效只剩 odom 模块，0.956）；f1 下残差活跃地错拉（1.123）；seg A 的 Δorigin≈2m 落在二次区内 → 回归（0.484）。

## 2. F2（保留项）的设计与依据

`FrameAligner` 的 gauge 求解是无 robust 的 2D 最小二乘，且**冻结后永不重估**——bootstrap 窗口内的 FLOAT 游走（实车 seg B 前 14 历元 σ 0.5~1.8m）会直接烙进全程基准。

实现：`AnchorPipeline::process_one` 中仅当 `sample.quality >= aligner_min_quality`（默认 4=FIXED，可配置）才 `aligner_.add()`；低质量样本的因子照常按 spec 降权注入（deferred/输出路径不变），只是不参与定基准。后果是 bootstrap 推迟到首个 FIXED+基线达标，deferred 窗口变长（seg B 约 14s），可接受。

## 3. 单元测试（`gnss_core/test/test_shared_gauge.cpp`，4 例全绿）

- `GaugeRegistry.FirstPublishWinsAndReset`：registry 语义（含 NaN 拒绝）
- `AnchorPipeline.FloatSamplesNeverBootstrap`：前半程 FLOAT（带 3m 偏置）不定基准、后半程 FIXED 正常 bootstrap；FIXED 段约束 <5cm、FLOAT 段 deferred 因子自带其偏置测量（因子照用、基准未污染）
- `AnchorPipeline.SecondPipelineAdoptsSharedGauge`：显式开启 shared_gauge 时 adopt 路径正确
- `AnchorPipeline.SharedGaugeDisabledKeepsLocalOnly`：默认关闭时不受 registry 污染

## 4. 实车验证结论

修复后代码（shared=false + F2）复跑三段，预期回到修复前水位且 gauge 数据更干净；若 seg B f1 ≈ 0.892、seg A f4 ≈ 0.324 复现，则 F2 无回归，结案。

## 5. 遗留问题（与本次无关，记录在案）

- seg B 的 floor 敏感性（f1 0.892 / f4 1.228 / f05 1.278）在"双 gauge 自洽"的新认知下不再有 bug 解释，归为**定权-动力学问题**（转弯段 anchor 滞后、固定滞后平滑器被先验钉死等），归二期架构议题。
- 两个模块原点不同这一设计本身值得斟酌（统一原点可让 gauge 打印值直接可比，便于现场诊断）——非 bug，列为改进候选。
- `estimate_lever_arm` 估计的杆臂（[-0.406, 1.885]）两段验证残差 0.2m，沿用；z 分量手量、图内在线估计（V7）仍在二期。
