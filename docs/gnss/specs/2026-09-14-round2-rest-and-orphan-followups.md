# 轮 2 收尾 `feat/round2-rest-and-orphan` 遗留项（2026-09-14）

本文件记录 `glim_ext` 分支 `feat/round2-rest-and-orphan`（计划 `plans/2026-09-12-round2-rest-and-orphan.md`，
6 个 Task：rtkrcv 孤儿修复、`PosWriter`、`RtkFix→PosRecord` 映射、按 UTC 日轮转、`pos_writer_node`、
rosbag2 脚本与 launch/yaml/README）执行期间各轮评审提出、经裁定**不阻塞合并**的遗留项。

分支经过了整分支最终评审、一轮修复波，以及维护者要求的合并前数据正确性修复（4 轮修复，每轮都有独立评审）。
最后一轮评审结论：**不阻塞合并**，`colcon test-result --all` 为 340 个测试，0 失败，1 跳过。
每轮修复的实测证据在 git 历史的 commit message 里。

上一份遗留清单 `2026-09-12-round2-followups.md` 的 **B 节（rtkrcv 孤儿进程）已由本分支处理**：
子进程 `setsid()` 之后调用 `prctl(PR_SET_PDEATHSIG, SIGTERM)`，节点启动前先探测 `sol_port`，端口被占用就拒绝启动。
实测 `kill -9` 节点后不留孤儿进程；第二个实例遇到端口被占时退出码为 1，且不写 `rtkrcv.conf`。
下面 C.4、C.5 两条是这个修法本身的剩余边角。

---

## A. 数据正确性：残留的静默部分读取（优先处理）

`.pos` 是 GNSS 约束权重标定的输入，重复行或缺行会静默扭曲建图结果，所以这一节排在最前面。

1. **提前 EOF 仍会造成去重失效**。在 FUSE/NFS 上，或文件被并发截断时，`read()` 可能在真实文件末尾之前就返回 0。
   流状态无法把这种情况和正常 EOF 区分开，于是 `read_pos` 静默返回部分结果，`open()` 返回 true，已写过的时间戳被再次追加。
   评审实测：500 行文件只读到 230 行，重复行照样落盘。本地文件系统上不会发生，所以不阻塞合并。
   修法：`open()` 已经拿到了 `file_size`，截断路径也已经在做同样的比较，
   在去重扫描里比对"已消费字节数 == file_size"，不相等就 fail closed。
2. **`read_glim_traj`（`gnss_core/src/synth.cpp:20`，`estimate_lever_arm` 在用）有同一类 bug**：
   `getline` 循环结束后不检查 `in.bad()`，读错误时返回部分轨迹，工具照常输出杠杆臂结果。
   应当顺手排查 `gnss_core` 里所有 `getline` 读取器。
3. **表头时间系统不一致没有检查**。已有文件的表头是 GPST，而 writer 配置成 UTC（或反之）时，去重键按 writer 的时间系统解析，
   与文件内容对不上，这时去重和轮转都会悄悄出错。`open()` 应当校验表头，不一致就拒绝打开。
4. **"新文件"只在 `open()` 时判定一次**：持有期间文件被别的进程截断，后续追加的内容就没有表头。
   当前是单写者设计，至少应写一句注释。

## B. 测试完整性

1. **`rtkrcv_node.cpp` 仍然没有任何测试链接它**（这是第 5 次被提到）。两处改动完全靠手动复现支撑：
   探测调用在 `start_local_reservers()`/`write_conf()` 之前的顺序，以及 `kProbeFailed` 时拒绝启动的策略。
   回退其中任何一处，全部测试仍然是绿的。
   **建议作为下一轮第一个 Task**：写一个针对 `fake_rtkrcv.sh` 的 `launch_testing` 用例，成本很低。
2. **`pos_writer` 沉默告警的两种区分没有测试钉住**：一种是"收到了消息但 `open()` 失败"，另一种是"完全没收到消息"。
   代码是对的，但评审做的 3 个把这两种情况混为一谈的变异全部通过了测试。
3. **测试 hook 的门控不可靠**：`GNSS_CORE_WITH_TEST_HOOKS` 挂在 `BUILD_TESTING` 上，而 ament 默认 `BUILD_TESTING=ON`，
   所以普通的 `colcon build` 产物照样导出 hook 符号（生产代码不调用它们）。
   另外，只在注入分支里抛异常的变异体能通过测试套件，真实 EIO 路径目前只有评审手工做的 `LD_PRELOAD` 实验覆盖。
   更好的修法是增加 `read_pos(std::istream&)` 重载，测试里传入会抛异常的 streambuf，走真实的 sentry 路径，
   这样可以完全删掉代码里的 hook。
4. `RatioHasNoCounterpartInRtkFixSoItIsZero` 是空过测试：`PosRecord` 的默认成员初始值本来就是 0.0，
   任何不碰 `ratio` 的实现都能通过。
5. 没有真正带小数秒、跨 UTC 午夜的轮转用例。评审手算确认过 `std::floor` 的行为正确，这是覆盖缺口，不是 bug。
6. 测试工装：`HelperCleanup` 先杀孙进程再杀 helper，中间有重生窗口；`fork()` 之后调用 `freopen()` 不是 async-signal-safe
   （目前由 3 s 轮询兜住）。
7. `test_pos_io` 的 `PosWriter` 用例只在开头 `remove` 旧文件、结束时不清理，每次运行在 `/tmp` 留下约 27 个 `pw_*.pos`。
   应改用每个用例独立的临时目录并在 TearDown 里删除。

## C. 行为与健壮性

1. **`PosDecimator` 的 K=1 抖动容忍有缺口**：真实的 1 个 bin 回退会让输出停顿最多 2 个数据秒（有上界，不会永久卡住）。
   在非默认 `period_s=0.1` 配合 0.25 s 抖动时，过滤会再次失效，实测 30 条里接受了 27 条。
2. **沉默告警的冷却在源恢复时不重置**：一个源恢复后又在上次告警 5 s 内再次失效，检测延迟最多多出约 5 s
   （实测盲区 2.8 s）。至少写一句注释。
3. **bad-stamp 的 `WARN_THROTTLE`、bad-path 的 `ERROR_THROTTLE` 和去重抑制告警仍然按文件/行共享一个静态状态**，
   多个源之间会互相吞告警。这和 Task 5 第 2 轮修掉的沉默告警是同一类问题。
4. **`open()` 拒绝打开时没有诊断日志说明原因**，比如尾部不完整行超过 `kMaxPlausibleIncompleteLineBytes`（4096 字节）、
   读错误、截断失败。节点侧只能看到笼统的失败。
   `create_directories` 的错误也被丢弃，调用方分不清是"建不了目录"还是"打不开文件"。
5. **`prctl(PR_SET_PDEATHSIG)` 之后缺少 `getppid()` 复查**：父进程如果在 `fork()` 与 `prctl()` 之间死掉，信号永远不会到达。
   标准做法是在 `prctl` 之后比对 `getppid()`，不一致就立即 `_exit`。
6. **端口探测的边角**：`bind()+SO_REUSEADDR` 会把"已绑定但尚未 listen"的套接字报告为 `kFree`。
   `sol_port < 1024` 且以非 root 运行时得到 `EACCES`，这时 `kProbeFailed` 的报错提示去查 `ulimit -n`，诊断方向是错的。
7. 兄弟类套接字未设置 `SOCK_CLOEXEC`（见上一份清单 D 节），本分支没有处理。
8. **重新打开时整文件解析**：一天大小的 `.pos` 大约耗时 225 ms，而且在 executor 线程上执行。
9. `gnss_time==0` 且 `header.stamp` 也为 0 时（默认构造或回放的消息），`to_pos_record` 的回退得到 epoch 时间戳。
   节点侧有 bad-stamp 守卫，但映射函数本身不拒绝这种输入。
10. `pos_path_for` 的 root 规范化只去掉尾部斜杠，中间的 `//` 或 `.` 原样保留。它目前是唯一的路径生成者，所以无害。
11. `estimate_lever_arm` 的诊断输出用 `front()`/`back()` 打印时间范围，默认输入已排序。
    `.pos` 现在允许回填（不保证时间有序），范围可能打印错误。结果本身不受影响，因为 `trajectory_compare` 会自行排序。

## D. 文档笔误

- `gnss_bringup/scripts/record_gnss.sh` 头部注释写"默认覆盖四路"却列了六个话题；
  还写着 `gnss_cgi610` 发布两路 RtkFix，但仓库里没有任何 `rtk_fix_gpchc` 的发布者。
- `gnss_bringup/config/gnss_bringup.yaml:126-128` 说"can/gpchc/rtkrcv 三行 topic 映射都保留在配置里，不需要另外声明"，
  但 `:153-157` 实际把 gpchc/rtkrcv 两行注释掉了，前后矛盾。
- `gnss_bringup/README.md:304` 的录制默认话题清单与脚本一致，也列出了 `/gnss_cgi610/rtk_fix_gpchc`，
  需要按"gpchc 待现场协议确认"统一口径。
- Task 1 的 commit body 与报告写的是 103 个用例，实际是 104 个（仅作记录，不改历史）。

## 处理状态(`feat/round2-hardening`,2026-09-14 计划)

2026-09-14 用真实 RTKLIB-EX 2.5.1 联调又发现两个在车上一条解都出不来的缺陷(yaml 默认 `binary`
永远起不来;conf 缺 `ant2-postype`),与下列遗留项一并在该分支处理。

| 条目 | 处理 | 提交 |
|---|---|---|
| A.1 去重扫描提前 EOF | `open()` 比对扫描消费字节数与文件大小 | 52b3326、2952605 |
| A.2 `read_glim_traj` 未检查 badbit | 已检查,并加 `std::istream&` 重载 | 52b3326、2952605 |
| B.1 `rtkrcv_node.cpp` 无测试 | 节点级子进程测试 5 条 + 真实 rtkrcv 回归 1 条 | bb3268a、b76fbd8 |
| B.3 测试 hook 门控 | 部分:新增的 `read_pos`/`read_glim_traj` 流重载测试走真实 sentry 路径,不依赖 hook;hook 门控本身未改 | 52b3326、2952605 |
| C.4 `open()` 拒绝时无诊断 | 未处理 | — |
| D 文档笔误(record_gnss.sh / yaml / README gpchc) | 已修 | f4124fb |
| (新)binary 裸名字起不来、崩溃循环静默 | 父进程按 PATH 解析;每次派生/退出打日志;启动前检查 | 7a705a5、874ee4e |
| (新)conf 缺 `ant2-postype`、非法值静默回落 | 新增 `base_pos_type`/`bds_ar_mode`/`glo_ar_mode`;全部枚举按 2.5.1 校验 | fec906d |

## 加固轮最终评审之后仍留下的项(2026-09-14)

`feat/round2-hardening` 分支(计划 `.superpowers/sdd/2026-09-14-round2-hardening/`)经过 7 个 Task 加最终整分支评审
后合入,评审结论「有修复项即可合入」,7 个修复项(F1-F7)已在本轮的 `glim_ext` 修复提交中处理完(其中 F3 就是下面
「rtkrcv_node.cpp / TcpStream / sol stream」一节里原先记录的构造函数 catch 路径 use-after-destroy,已修复,不再重复
记录)。以下是各 Task 过程评审中判定「不阻塞合并」的次要遗留项(取自 `progress.md` 里所有 `minor (deferred)` 记录),
以及最终评审新提出的两条,按区域归类,供轮 3 参考。

### pos_io.hpp / test_pos_io.cpp(Task 1)

- `pos_io.hpp` 的注释仍写着 `open()` 通过 `read_pos()` 收集已有 key,实际已经改名为 `read_pos_stream`。
- `test_pos_io.cpp` 里有两段紧挨着的匿名 namespace,可以合并。
- eof 特殊情况(读取在最后一个 `'\n'` 之前停止)没有测试覆盖。
- `size` 通过 `stat` 拿到之后,如果文件在 stat 和扫描之间被截断或变大导致大小对不上,现在的 fail-closed 行为没有写文档说明。
- 按简报要求新增的 `pos_io` 测试用固定的 `tmp_path` 文件名而不是 `mkdtemp`;`ASSERT` 失败时文件会留在 `/tmp`。
- (最终评审新增)`test_pos_io` 里既有的 `PosWriter` 用例在 `/tmp` 留下固定名字的文件 `pw_*.pos`,违反了本轮的 `mkdtemp` 约定。

### ProcessSupervisor(Task 2)

- 报告里缺少 `SkipsNonExecutableFilesAndDirectories`/`NullOrEmptyInputsResolveToNothing` 两个用例的 RED 证据
  (评审用变异测试验证过两者确实可杀);去掉 `if (name.empty()) return {};` 这一行不会让任何测试失败
  (简报规定的断言测不出这一处)。
- `on_spawn` 回调的 `executable` 参数从未被断言过,即使传成 `on_spawn(pid, cfg_.binary)` 也能通过测试——Task 4 用到了这个参数。
- `waitpid` 出错(比如 `ECHILD`)时的 `break` 分支会报告 `exited=false signaled=false` 且 `detail` 为空,errno 丢失。
- `running_` 在 spawn 失败分支(:264/:267)和退出分支(:530/:534)里各被读了两次:如果在两次读取之间插入 `stop()`,
  会先读到 `will_restart=true` 再 `break`;应该只读一次存到局部变量。
- 退避时间的计算公式在 spawn 失败分支和崩溃循环分支里各写了一遍(简报要求如此)。
- supervisor 线程里 `getenv("PATH")` 和并发的 `setenv` 之间不是线程安全的(已写注释说明);
  测试在 `PATH` 原本未设置时用 `setenv("")` 恢复,应该用 `unsetenv`。
- `on_spawn` 在记录 `t0` 之前就被调用,导致回调本身耗时的部分没有算进 `lifetime_s`。

### navsys 配置校验(Task 3)

- `navsys` 的报错信息格式是 `navsys=128` 而不是 `navsys="128"`(简报代码原文如此)。

### rtkrcv_node.cpp / TcpStream / sol stream(Task 4)

- 新增成员的注释说"不假设同一线程",但相邻的已有注释说"同一个 TcpStream 线程"
  (`tcp_stream.cpp` 证实确实是同一个 worker 线程),两处注释互相矛盾。
- "非空闲断开会结束安静期"这个分支只被联调时新写的测试覆盖到;`FirstConnectIsPrinted` 这个用例即使
  `on_status` 恒为 `true` 也能通过;节点里 `on_solution_line` 的接线本身没有测试。
- (最终评审新增)隧道与开阔天空目前无法区分:应在安静期内加一条 `WARN_THROTTLE`(10–30 分钟)报告距上一条解的
  时长;轮 3 再增加节点级健康信号(距上一条解时长、上行字节计数)——目前单靠日志闸门分不清"隧道"和"配置错误"。

### 测试工装 harness(Task 5)

- commit `bb3268a` 的说明把 mutation 3 的 RED 证据描述错了(真正会失败的断言是 test:84-85 的
  `kill(child,0)`/退出计数,不是 20 秒等待);这里做更正,不改历史。
- `test_rtkrcv_node_process.cpp` 里 `ASSERT_*` 提前返回时会漏清理 `mkdtemp` 出来的临时目录(没有 RAII)。
- busy-port 用例匹配的是"孤儿"这个词,但 `kProbeFailed` 的报错信息里也有这个词,应该改成匹配"已经有人在监听"。
- `count_occurrences`(harness 公共 API)在 needle 为空字符串时会死循环。
- harness 里的 socket 没有设置 `SOCK_CLOEXEC`,子进程 `execve` 前也没有关闭多余的 fd,节点会继承测试的 fd。
- `NodeProcess` 里 `fork` 失败是静默的(`wait_exit` 返回 -1 和超时是同一种表现);`pick_free_port` 三次调用之间
  有竞争窗口;test:115 的断言缺少日志辅助诊断。

### 真实二进制测试与构建(Task 6)

- `test_rtkrcv_real_binary` 失败路径下临时目录没有清理(没有 scope guard,简报要求如此)。
- `ROS_DOMAIN_ID` 的计算公式 `40+getpid()%50` 和 `test_rtkrcv_node_process` 共用,并行跑 ctest 时大约有 2% 概率撞车。
- 构建警告 `CMP0167`/`gtest_vendor` 的 cmake 最低版本告警是历史遗留;这台机器上 colcon 需要先
  `source /opt/ros/humble/setup.bash`(pyenv shim 的问题)。

### 文档(Task 7)

- README 里插入 gpchc 免责声明段落后留下一行较短的原始行(渲染没问题)。
