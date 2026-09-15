# 轮 3a `feat/round3a-diagnosis-core` 遗留项（2026-09-15）

计划：`docs/gnss/plans/2026-09-15-round3a-diagnosis-core.md`。7 个 Task 各自经过独立评审，之后做了整分支最终评审和一轮修复复审；最终结论是没有未关闭的 Critical 或 Important。全量测试 477 个，0 失败。

## A. 待维护者决定（影响 `device_divergence` 的可信度）

1. **持续存在的 610 偏差会被学习成正常值**（最终评审 I2，本轮没有修）。经验 σ 取的是非负距离的 RMS，分不清恒定偏差和噪声。启动时就存在的偏差，或者配对中断 600 s 以上（即每条长隧道）之后存在的偏差，都会在 60 个样本内并入基线；已经打开的事件会以 `recovered` 关闭。可选修法（可以组合）：
   - 给学习到的 σ 设上限，新增 `divergence_sigma_max_m`；
   - 隧道之类的配对中断之后不重新学习，保留原基线；
   - 只用 FIXED 样本学习。
2. **rtkrcv 进入浮点解时，窗口会被阈值拉宽**。修复波让阈值随 rtkrcv 当前 σ 抬高，这样浮点解期间不会误报；但这段时间内低于抬高后阈值的样本会进入窗口。模拟结果：90 s 的 FLOAT 过后，学习到的阈值在接下来最多 600 s 内停在 0.32–0.46 m（此前为 0.15 m），这期间真实的 0.3 m 故障不会被报出。修法和第 1 条一起定，比如只接纳 `current_sigma <= base` 的样本，或只接纳 FIXED 样本。

## B. 轮 3b 必须处理的接口事项

- **ratio 传不到诊断**：`gnss_msgs/RtkFix` 没有 ratio 字段，所以 `ambiguity` 规则在节点里拿不到数据。需要给消息加字段，或者由 `rtkrcv_node` 另发一个话题。
- **`SolutionSample::epoch_t` 需要填写**：3b 要用 `RtkFix.gnss_time`（> 0 时）填入。两路的历元始终没有落在 0.1 s 以内时，配对会静默失败，建议加诊断输出。
- **时间约定**：`DiagnosisEngine` 要求 `t`/`now` 单调不减，但这个 `t` 同时也是写进日志的 UTC 时间。可以用 steady clock 驱动引擎、写日志时换算成 UTC；也可以在检测到时间跳变时重置引擎。回放 bag 时要统一用 sim time。
- **线程**：引擎不是线程安全的，回调和 `tick` 需要串行执行（单线程 executor 或加锁）。
- **停机顺序**：`shutdown(now)` → 写入关闭行 → 关闭 `LineAppender`。
- **基站**：`baseline_learned` 时持久化 `engine.baseline()`；`last_history` 需要能跨日期目录读取 `base.pos` 末行（或者传空，接受每次重启多写一行）；把 `reset_base_baseline` 暴露成运维服务。
- **清理**：录包根目录和 `.pos` 根目录共用同一个磁盘水位，建议先扫录包目录；某次扫描结束后仍高于水位时打日志。
- **启动**：考虑给 `no_data` / `no_solution` 设一段启动宽限期（rtkrcv 收敛前 `no_solution` 必开，模拟中约 51 s）。

## C. 代码与测试的小问题（不阻塞）

**规则链**
- `format()` 用固定 256 字节缓冲，控制点名过长时会被静默截断，可能截在 UTF-8 字符中间。
- `DiagnosisInput::divergence_threshold_m` 默认为 0。
- 对空结果调用 `status()` 是未定义行为。
- `close_hysteresis_s <= divergence_hold_s + 1` 时事件会抖动，校验里没有拦。
- 覆盖缺口：默认值测试只查了 6/15 项，校验测试只覆盖 6/20 条；缺少边界用例。

**偏差监测**
- 注释：rule 5 说非有限值"不影响 since"，但代码会清零 since。
- 缺少测试钉住"held 基线从不存当前 σ"。
- 测试里仍有"回退模式"等旧措辞。
- 配对很稀疏时（每个窗口不足 60 个样本）会停在预热期。

**事件机**
- `InfoAndOkNeverOpen` 没有记录变异验证。
- 缺少测试：关闭时报告的是开启时的级别和消息；同一 tick 内多个关闭的顺序。
- 所有已开事件共用同一个 tick 的指标。
- `recovered` 也涵盖"规则输入过期"的情况（注释已写明）。

**基站**：warmup 0、覆盖已有基线文件、残留 `.tmp` 文件、NaN 坐标，这些情况都没有测试。

**落盘**
- `LineAppender::open` 在 `file_size` 报出非 ENOENT 错误时把文件当成新文件。
- `snprintf` 固定缓冲会截断；peak 键没有做清洗。
- `level_name` 靠间接 include 引入。
- 64 位 `time_t` 的假设没有写明。

**清理**：`fs::space` 失败时静默退化为只按天数删除；`DatedEntry` 直接构造时日期不做校验。

**引擎**
- 时间回退（未来时间戳被当作新鲜）没有防护，只在头文件里写明了前提。
- 覆盖缺口：`corr_age` 优先级、事件位置的来源、`solver_enabled` / `control_points` 的透传、构造参数顺序。
- 测试夹具 `set_bits` 会静默截断。
- `$SAT` 行被解析了两次。

## 轮 3b 处理状态(2026-09-15,计划 `docs/gnss/plans/2026-09-15-round3b-diag-node.md`)

- A.1、A.2:已处理——只用两路都 FIXED 的样本学习,学到的 σ 上限 `divergence_sigma_max_m = 0.10`(Task 2)。
- B 全部已处理:ratio 加进 `RtkFix`(Task 1);`epoch_t` 由 `gnss_time > 0` 填入、配对持续失败打 WARN;
  ROS 时间 + 回跳检测重建引擎;单线程 executor;停机顺序;基线持久化、`last_history` 跨日期读取、
  `reset_base_baseline` 服务;清理先录包后 `.pos`、清完仍超水位打 WARN;启动宽限期 60 s(Task 3–6)。
- C 已顺带处理:偏差监测规则 5 注释、held 基线不存当前 σ 的测试(Task 2);`fs::space` 失败不再静默
  ——`disk_used_pct` 返回空(Task 6),这一轮仍只按保留天数删除,由 `gnss_cleanup_node` 打 WARN 说明
  水位不起作用(整分支评审修复 M3)。其余 C 项仍未处理。
