# 轮 2 `gnss_bringup` 遗留项（2026-09-12）

本文件固化 `feat/round2-gnss-bringup` 分支执行期间由各轮评审提出、经裁定**不阻塞合并**的遗留项。
分支已通过整分支最终评审（结论：可合并）。每条都标注了根因与裁定理由，便于后续按优先级消化。

来源：8 个 Task 的逐任务评审 + 整分支最终评审。执行记录（含每轮修复的实测证据）在 git 历史里。

---

## A. 必须在真实硬件上补验（不是缺陷，是未验证）

RTKLIB **未安装**在开发机上，以下从未对真实 `rtkrcv` 二进制验证过：

1. 生成的 `rtkrcv.conf` 键名是否被目标版本接受
2. 真实 llh 解流的内容与节奏是否如预期解析
3. 真实 `$SAT` / `.stat` 的格式与文件命名
4. `rtkrcv` 对 `SIGTERM` 的真实响应（能否在 5 s 内 flush）
5. 崩溃循环退避在真实 conf 错误下的表现
6. 多客户端 / 大流量下 `LocalReserver` 与 `rtkrcv` 的真实交互
7. 端到端：`rtcm_bridge` → `rtkrcv_node` → `~/rtk_fix` 全链路

另有四项现场待确认值（详见 `gnss_bringup/README.md` 的缺口表）：平台差分端点与连接方向、
板卡原始观测端点、`inpstr1-format`、`inpstr2-format`。**四项都只改 yaml,不改代码。**

## B. 最大的残余风险：`rtkrcv` 孤儿进程

`ProcessSupervisor` 的 `setsid()` 对进程组 kill 是正确的，但子进程会**存活于任何非 `stop()` 的父进程死亡**。
最终评审实测：对 `rtkrcv_node` 发 `kill -9`，子进程被 init 收养并继续占着 `sol_port`；
节点重启后新的 `rtkrcv` 绑不上该端口，而节点自己的 `TcpStream` 会**连上那个孤儿**、
把它的陈旧解当作新鲜的 `RtkFix` 发出去——正是本分支一直在防的"看起来正常的错误数据"。

本轮只做了文档（README 要求以 systemd 的 cgroup `KillMode` 运行）。代码修法需要决策：
`prctl(PR_SET_PDEATHSIG)` 是线程死亡语义，`run_dir` 里放 pidfile 更简单。**建议优先处理这一条。**

## C. 测试完整性（每条都在守一个真实回归）

- `test_local_reserver` 的 `dlsym`/`RTLD_NEXT` close 拦截**没有正向对照**：拦截一旦静默失效
  （静态链接、`-Bsymbolic`、绕过 PLT 的 libc），断言会空过，测试变成永远绿的空壳。
  加一段自检（`dup` 后连关两次，期望计数为 1）即可。`dlsym` 返回值也未判空。
- 换行注入的三个用例载荷同时含 `=`，只删换行守卫时全绿——守卫有效但无测试隔离它。
- `ImmediateCloseServer` 忽略 bind/listen/getsockname 返回值，且缺 `ASSERT_GT(port,0)`。
- `ClientBackoffGrowsButIsCappedByMax` 的"封顶"那一半仍是空的（每次成功连接都会重置退避）。
- Task 6 回归测试的循环内 `ASSERT_EQ` 会在恢复 SIGTERM 处理器前返回，失败路径上会泄漏处理器。
- 缺失覆盖：`stop()` 后无僵尸/孤儿的断言、`start()`-after-`stop()`、双重 `start()`。
- 最终修复波新加的双关闭不变量测试仍是两个真实线程竞争（40 次试验），不是 seam 强制的确定性测试；
  要硬保证需在 `LocalReserver` 里留测试 seam。
- **完全没有节点级测试**：`rtkrcv_node.cpp` 五百多行里只有纯函数被测，接线、启停顺序、析构顺序
  全靠手动冒烟。一个针对 `fake_rtkrcv.sh` 的 `launch_testing` 用例很便宜，且本可提前发现 `sol_port=0`。

## D. 代码重复与一致性

- **秒数校验谓词重复**：`rtk_fix_mapping.hpp` 与 `rtcm_bridge_params.hpp` 各有一份函数体逐字相同的
  正数有限秒校验，`rtkrcv_node.cpp` 同时包含两者，曾导致真实的重定义编译错误，现靠改名
  (`is_positive_finite_backoff_seconds`) 规避。**改名只是掩盖**：将来有人改动其中一份的边界行为，
  两份会静默分叉。正解是抽到一个共享头文件。
- `is_valid_port()` 现在只被 `is_valid_port_for_direction()` 使用，可合并或明确保留。
- 兄弟类的套接字未设 `SOCK_CLOEXEC`（`local_reserver.cpp`、`tcp_stream.cpp`）。目前由子进程侧的
  `close_range` 兜住，但兜底不该是唯一防线。

## E. 静默放弃路径（组件死了但没人知道）

- `ProcessSupervisor::start()` 的 `pipe2` 失败无回调无日志；`setsid()` 返回值被丢弃。
- `stop()` 的 10 s 放弃期限到期时不报告任何东西。
- 无健康话题 / 心跳：两个节点都不会重启自行退出的 worker，运维唯一的信号就是日志。
  轮 3 的 `gnss_diag` 需要一个可订阅的健康状态。

## F. 未文档化的单调用者约定

并发 `stop()`、回调内调 `stop()`（自 join 会 EDEADLK）、`start`/`stop` 对 `wake_fd_` 的竞争、
重启路径上不加锁关 `wake_wr_`。两个节点目前都不会触发。**诚实的修法是写进头文件注释，而不是加锁。**

## G. 行为变更与小问题

- `rtcm_bridge` 的 `port` 默认仍是 0 而 `listen` 默认 false，因此一个不写 port 的流声明现在会在
  启动时硬失败。这是正确的，但属行为变更（随包发布的 yaml/launch 不受影响，它们显式写了端口）。
- `args: []` 空 YAML 序列会让 `rtkrcv_node` 启动崩溃（ROS2 无法推断空序列元素类型）。
  现已在 yaml 里注释掉并记入 README 已知问题；正解是用 `ParameterDescriptor` 钉死类型，
  或改成分隔符字符串参数。
- `plan_stat_tail` 用严格 `>` 比较 mtime，相同时间戳时退化为目录序；同名重开且尺寸 ≥ 旧 offset 时
  检测不到（无 inode 检查）。
- `%.6f` 仍受 `LC_NUMERIC` 影响（本机无逗号小数点 locale，无法复现）。
- `CMakeLists.txt` 硬编码 `/usr/share/cmake/geographiclib`；失败是响亮的，但在 aarch64 上会咬人。
  正解是让 `gnss_core` 通过 ament extras 导出自己的 module path。
- 每块数据一次堆分配、`pump()` 里的 `std::function` 临时对象、200 ms 的重启检测节拍——
  均为性能/整洁项，裁定为不值得修。

## H. 裁定为不值得修（记录以免重复讨论）

PID 复用窗口（亚微秒且需目标恰为进程组首）、`pump()` 的 listen-fd `POLLERR` 槽位、
`memcpy` 用 `sizeof(sockaddr_in)` 而非 `ai_addrlen`（`AF_INET` 已钉死）、
`sysconf==-1` 回退的 65536 上限（回退的回退）、`g_alive` 在动态初始化前被触及。
