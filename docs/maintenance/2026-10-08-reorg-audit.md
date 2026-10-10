# 2026-10-08 结构、命名与文档整理审计

- 分支：`chore/reorg-naming-docs`（从 `cursor/ros2toladds` 的 `21f0804` 新建）
- 性质：**只读审计**。除本文件外，没有删除、移动、重命名、覆盖或格式化任何文件。
- 审计在 2026-10-08 完成，本文件在 2026-10-10 写入；写入时 HEAD 仍是 `21f0804`，工作区干净，证据没有失效。
- 本轮没有运行任何测试，也没有调用 WSL / ROS / Gazebo。文中凡写"未核实"的内容都没有实际验证。
- 使用方式：用户回复"批准 NN"后才执行对应编号；每执行完一项，在第 2 节总表的"状态"列写上 commit hash 与验证结果。

---

## 0. 第 0 步：安全检查结果

| 项目 | 结果 |
| --- | --- |
| 审计开始时的分支 | `cursor/ros2toladds`，工作区干净（无未提交、无未跟踪改动） |
| 上游 | `origin/cursor/ros2toladds` |
| 领先 / 落后 | 领先 6、落后 1。这是按上次 fetch（`.git/FETCH_HEAD` 时间 2026-09-21 14:49）计算的，本次没有 fetch，远端实际状态未核实 |
| 本地独有的 6 个提交 | `c7a55d6`、`cfe8081`、`e651cc2`、`83ba3c2`、`d37a775`、`21f0804`（2026-09-21 至 2026-10-08） |
| 远端独有的 1 个提交 | `1859419`（2026-09-21，Zirui）"Replace costmap 0141 adapter with LaserScan obstacle conversion"：删掉 `costmap_0141*`，新增 `scan_0141.py`/`scan_node.py`。本地这条线改成了 `radar_0141.py`/`radar_node.py`，两边对 0141 的实现方向不同，见第 31 项 |
| 新建分支 | `chore/reorg-naming-docs`，后续整理全部在此分支进行 |

---

## 1. 结论摘要

1. **最需要先拍板的是 LaDDS 原点**：提交 `21f0804`（2026-10-08）把 `route.yaml` 原点改成了 `39.9450901 / 116.3342309`，而代码默认值、`bridge.yaml`、`single_leg.yaml`、运行时 world 副本、文档和全部示例报文仍是 `30.5453 / 114.392931`，部分测试还用着上一版的 `30.5472 / 114.3891`。现在仓库里同时存在四组原点（见 D1）。
2. **默认控制器实际是 ADRC（±361 N）**，测试也锁定了这一点（`src/sim_env/test/test_launch_modules.py:106`），但 README 和三模块文档仍写默认 PID（见 D2）。
3. **结构上最乱的是 `docs/` 顶层和 `scripts/` 顶层**：`docs/` 顶层 25 份文档里，约一半是迁移期或调试期的历史记录，与现行说明混放；LaDDS 的说明分散在 6 份文档里，内容互相重叠，部分数值已过时（航点半径、原点、`/tmp` 安装区）。
4. **当前文档里的断链很少**：扫描 701 个相对链接（不含 `deliverables/`），只有 3 处断链在现行文档里，另外 4 处在 `docs/validation/` 历史证据里（按规则不改）。
5. **有一个测试实际上从不执行**：`scripts/validation/test_ladds_startup_ros.py:16-19` 的默认安装区是已不存在的 `/tmp/usv_ladds_single_leg_20260915/install`，因此该 ROS 启动测试会一直被跳过。按规则，跳过不能算通过。
6. **`scripts/build.sh` 和 `scripts/build_ladds_route.bash` 的构建方式冲突**：前者没有 `--base-paths src`，还用 `--symlink-install`；后者注释写明仓库 `install/` 是 copy install，不能改成 symlink（`scripts/build_ladds_route.bash:62-63`）。两者往同一个 `install/` 写。
7. Python 代码的命名本身是规范的（没有驼峰函数，类名都是 PascalCase，ROS 测试统一以 `_ros.py` 结尾）。命名问题主要集中在：脚本后缀 `.sh`/`.bash` 混用、`single_leg` 这个名字实际承载的是多航点航路、"值守"一词有两个不同含义、文档文件名风格不统一。

共 31 项建议，按风险从低到高排列：低风险 13 项（01–13）、中风险 10 项（14–23）、高风险 8 项（24–31）。高风险项大多建议"只记录、不改"。

---

## 2. 建议清单总表

| 编号 | 类别 | 标题 | 风险 | 依赖的决定 | 状态 |
| --- | --- | --- | --- | --- | --- |
| 01 | A 根目录 | `.gitignore` 补充现代条目（只增不删） | 低 | — | 待批准 |
| 02 | C 文档 | 新建文档总索引 `docs/README.md` | 低 | — | 待批准 |
| 03 | C 文档 | 新建 `docs/validation/README.md` 证据索引（不改证据本身） | 低 | — | 待批准 |
| 04 | C 文档 | 修复现行文档中的 3 处断链 | 低 | — | 待批准 |
| 05 | A 根目录 | 从仓库移除运行垃圾文件（`debug.log`、`图片/`、`preview.pid` 等） | 低 | — | 待批准 |
| 06 | A 根目录 | 处理一次性调试脚本 `_read_nav2.py` | 低 | — | 待批准 |
| 07 | C 文档 | 更正 README / CLAUDE.md / 三模块文档里无争议的过时陈述 | 低 | — | 待批准 |
| 08 | C 文档 | `docs/superpowers/`（AI 过程计划）移入历史区 | 低 | — | 待批准 |
| 09 | C 文档 | 迁移期历史文档移入 `docs/history/` | 低 | — | 待批准 |
| 10 | B 命名 | `README.python.md` 改名为 `README.md`（两个算法包） | 低 | — | 待批准 |
| 11 | C 文档 | 设计类文档归入 `docs/design/` | 低 | — | 待批准 |
| 12 | A 代码 | `docs/examples/trajectory_generation/generate.py` 旧路径失效 | 低 | — | 待批准 |
| 13 | B 命名 | 命名规范与术语定稿写入文档（含 CONTEXT.md 补术语） | 低 | Q7 | 待批准 |
| 14 | C 文档 | 值守类 5 份文档归并 | 中 | D3 | 待批准 |
| 15 | C 文档 | LaDDS 6 份文档归并 | 中 | D1、Q4 | 待批准 |
| 16 | C 文档 | 控制类 5 份文档整理（含 1 处代码引用路径） | 中 | D2 | 待批准 |
| 17 | A 脚本 | `scripts/build.sh` 缺 `--base-paths src`，且与 copy install 冲突 | 中 | Q5 | 待批准 |
| 18 | A 测试 | LaDDS 测试与验证脚本里的 `/tmp` 旧默认路径（导致测试恒跳过） | 中 | — | 待批准 |
| 19 | A 测试 | `pytest.ini` 的默认范围与全量回归范围不一致（漏 `scripts/control`） | 中 | — | 待批准 |
| 20 | A 脚本 | 14 个当前无人引用的验证/探针脚本 | 中 | — | 待批准 |
| 21 | B 命名 | LaDDS 仿真 profile 放在 `validation/config/`，核心脚本名叫 `single_leg` | 中 | — | 待批准 |
| 22 | A 根目录 | `scripts/colregs_demo/`（5.1 MB 独立原型）去留 | 中 | Q8 | 待批准 |
| 23 | A 根目录 | `rmp.code-workspace`、`.cursor/`、`docker/` 去留 | 中 | Q8 | 待批准 |
| 24 | B 命名 | `usv_ladds_bridge` 内的 `single_leg` 命名（模块/可执行/launch/YAML） | 高 | — | 建议只记录 |
| 25 | A 代码 | `usv_mission/configuration.py` 是 `usv_navigation/configuration.py` 的分叉副本 | 高 | D3 | 建议只记录 |
| 26 | A 根目录 | 根目录 `0924controller/`、`USV_control/` 控制参考源码 | 高 | — | 建议只记录 |
| 27 | A 代码 | `sim_env/config/` 下的原作者算法模板（会安装，但不加载） | 高 | — | 建议只记录 |
| 28 | B 命名 | 通用 ROS 包名（`common`、`controller`、`perception` 等） | 高 | — | 建议不改 |
| 29 | B 命名 | 节点名 `usv_simulation`、参数前缀、YAML 历史拼写 `control_frequncy` | 高 | — | 建议不改 |
| 30 | A 根目录 | `deliverables/` 是否移出仓库 | 高 | D3 | 待决定 |
| 31 | A 分支 | 本地与远端在 0141 实现上分叉 | 高 | Q6 | 待决定 |

---

## 3. 逐项说明

每一项都写了：现状（这是什么、为什么存在）、问题、依据、建议、风险、涉及文件、验证方法。

### 低风险（只动文档、忽略规则或无人引用的杂项，不影响构建和运行）

#### 01 ｜ A 根目录 ｜ `.gitignore` 补充现代条目

- **现状**：`.gitignore` 是原作者 ROS1/catkin 模板（`devel/`、`msg_gen/`、`*.cfgc` 等），后来在末尾追加了 colcon 条目（`install/`、`log/`、`.pytest_cache/`、`validation-results/`、`/validation/`、`/local/ladds/`）。它决定哪些文件不进 git。
- **问题**：
  1. `__pycache__` 只写了 `*/__pycache__`（`.gitignore:7`），只匹配第一层目录；更深层的靠 `*.pyc`（`:46`）兜住，目前碰巧有效。
  2. 没有忽略 `*.log`、`*.pid`、`desktop.ini`，所以 `debug.log`、`preview.pid`、`图片/desktop.ini` 被提交了（见 05）。
  3. `.claude/settings.local.json` 目前只被用户全局忽略文件 `C:\Users\卤子\.config\git\ignore:1` 忽略。换一台机器就可能被提交。
  4. `bin/`、`lib/`（`:12-13`）没有锚定，任何层级名为 `lib`/`bin` 的目录都会被忽略。目前没有被误伤的文件，但以后在包里新建 `lib/` 会被静默忽略。
- **依据**：`git check-ignore -v` 实测：`debug.log`、`foo.log` 不被忽略；`scripts/__pycache__` 由 `.gitignore:7` 忽略；`.claude/settings.local.json` 由全局 ignore 忽略。
- **建议**：只在末尾追加，不删除原有条目：`__pycache__/`、`/debug.log`、`*.pid`、`desktop.ini`、`.claude/settings.local.json`。`*.log` 不建议全局忽略，因为 `docs/validation/` 下有 465 个被跟踪的 `.log` 证据，以后新增证据日志会需要 `git add -f`。`lib/`、`bin/` 暂不改，只加注释说明。
- **风险**：低。只影响未跟踪文件是否显示，已跟踪文件不受影响。
- **涉及文件**：`.gitignore`
- **验证方法**：`git status --short` 前后一致；对上述样例路径运行 `git check-ignore -v`。

#### 02 ｜ C 文档 ｜ 新建 `docs/README.md` 总索引

- **现状**：没有文档总索引。README.md 末尾有一张"文档导航"表（`README.md:262-275`），只覆盖约一半文档，也不含 LaDDS 文档。
- **问题**：新接手的人不知道哪些文档是现行的，哪些是历史。
- **依据**：第 4 节文档清单。
- **建议**：新建 `docs/README.md`，按"现行指南 / 参考 / 设计 / 历史 / 验证证据"分组列出全部文档，每份写一句用途和"现行/历史"标记。第一版不移动任何文件，只做索引。后续 08、09、11、14–16 每移动一次就同步更新索引。
- **风险**：低（新文件）。
- **涉及文件**：`docs/README.md`（新）
- **验证方法**：用链接扫描脚本确认索引中的链接全部可打开。

#### 03 ｜ C 文档 ｜ 新建 `docs/validation/README.md`

- **现状**：`docs/validation/` 下有 108 个顶层条目（目录和文件），是各轮验证的原始证据，绑定具体源码版本。
- **问题**：没有索引，无法判断哪份证据对应哪个阶段。命名混用 `-001` 后缀和 `-20260915` 日期后缀。
- **依据**：`git ls-files docs/validation | cut -d/ -f3 | sort -u`。
- **建议**：只新增一个索引文件，按阶段分组（C++ 基线 / Python 迁移 / 三模块拆分 / 目录整理 / ADRC / LaDDS / 值守），不改任何证据的正文和数据，也不改目录名。
- **风险**：低。
- **涉及文件**：`docs/validation/README.md`（新）
- **验证方法**：链接扫描；`git diff --stat` 只显示这一个新文件。

#### 04 ｜ C 文档 ｜ 修复现行文档中的 3 处断链

- **现状 / 问题**：`usv_mission` 已从 `src/core/usv_mission` 移到 `src/core/mission_module/usv_mission`，两份值守文档仍指向旧路径。
- **依据**（链接扫描结果）：
  - `docs/station-guard-guide.md:11` → `../src/core/usv_mission/README.md`
  - `docs/station-guard-migration.md:80` → `../src/core/usv_mission/README.md`
  - `docs/station-guard-migration.md:81` → `../src/core/usv_mission/validation/README.md`
  - 另有 4 处断链位于历史证据中，按规则不改：`docs/validation/module-layout-001/README.md:17`（2 处）、`docs/validation/los-preview-speed-diagnostic/README.md:52`、`docs/validation/pid-course-damping-diagnostic/README.md:20`。
- **建议**：只改这 3 个链接的目标路径。如果先执行 14（值守文档归并），本项并入 14。
- **风险**：低。
- **涉及文件**：`docs/station-guard-guide.md`、`docs/station-guard-migration.md`
- **验证方法**：链接扫描，现行文档断链数为 0。

#### 05 ｜ A 根目录 ｜ 从仓库移除运行垃圾文件

- **现状 / 证据**：

  | 文件 | 是什么 | 被引用 | git 记录 |
  | --- | --- | --- | --- |
  | `debug.log`（10 KB） | Chromium/Electron 浏览器内核写出的 `[0914/125032...:WARNING:...]` 日志，与本项目代码无关 | 全仓库无引用 | `3797e05`（2026-09-17）随"定点微控0211和停船0331"一起提交，`5652eee` 再次快照 |
  | `图片/desktop.ini`、`图片/Camera Roll/desktop.ini` | Windows 资源管理器自动生成的文件夹配置 | 无 | `3797e05` 起 |
  | `图片/rosgraph.dot` | **0 字节空文件** | 无 | `cfe8081`（2026-09-23） |
  | `scripts/colregs_demo/validation/preview.pid`（内容 `62636`）、`preview.stderr.log`（476 B）、`preview.stdout.log`（0 B） | 本地预览服务器的进程号和输出 | 无 | `b9cd7fc` |

- **问题**：都是运行时或系统自动产生的文件，误提交进了仓库。
- **建议**：用 `git rm` 从仓库移除（git 历史里仍可找回），同时依赖 01 加忽略规则，避免再次提交。`图片/` 整个目录在移除后为空，会一并消失。
- **风险**：低。上述文件都没有被任何脚本、launch、测试引用。
- **涉及文件**：上表 6 个文件
- **验证方法**：`git grep` 文件名无结果；`python3 -m pytest scripts/validation -q`（确认 colregs 目录不在测试范围，不受影响）。

#### 06 ｜ A 根目录 ｜ 一次性调试脚本 `_read_nav2.py`

- **现状**：8 行脚本，打印 `/opt/ros/humble/include/nav2_costmap_2d/...obstacle_layer.hpp` 和 `observation_buffer.hpp` 的前 8000 个字符。
- **为什么存在**：在 `c7a55d6`（2026-09-21）里随 LaDDS 0141 障碍层的开发一起提交，推测是当时查看 Nav2 头文件用的（推测，未核实）。
- **问题**：全仓库无引用，也不属于任何构建或测试。
- **建议**：选项 A 直接 `git rm`（推荐，功能一行命令即可替代）；选项 B 移到 `scripts/tools/read_nav2_headers.py`。
- **风险**：低。
- **涉及文件**：`_read_nav2.py`
- **验证方法**：`git grep _read_nav2` 无结果。

#### 07 ｜ C 文档 ｜ 更正无争议的过时陈述

只改与代码事实明显不符、且不涉及待决事项的句子。涉及默认控制器、原点、航点半径的句子等 D1、D2、Q4 拍板后再改。

| 位置 | 现在写的 | 事实（依据） |
| --- | --- | --- |
| `README.md:5`、`docs/distributed-navigation/README.md:3` | 当前开发分支为 `codex/distributed-navigation` | 当前工作分支为 `cursor/ros2toladds`，默认远端分支为 `ziruijiang/ladds-adapter-integration` |
| `README.md:7` | LaDDS 只支持 `0211`/`0411`/`0141`，"另提供可选的仿真 `costmap_raw` 快照适配器" | 实际为 0211/0212/0331/0411/0141 收，0051/0052/0053/0054/0411/0141 发（`config/route.yaml:6-21`、`scripts/start_ladds_route_relay.ps1:6-19`）；`costmap_0141*` 已在 `c7a55d6` 删除，换成 `radar_0141.py` |
| `README.md:147` | `planner_name` 支持 astar/dijkstra/gbfs；`controller_name` 支持 pid/lqr | `launch_support.py:30` 还有 `hybrid_astar`，`:32` 还有 `adrc` |
| `README.md:258`、`CLAUDE.md:85` | 根目录 `adrc/` | 已在 `c7a55d6` 删除；根目录另有 `0924controller/` 未列入"不属于当前运行链" |
| `CLAUDE.md:77` | "用 `scripts/build_ladds_route.bash` 构建独立 overlay" | 该脚本现在构建进仓库自己的 `install/`（`scripts/build_ladds_route.bash:2,64-71`），与 `CLAUDE.md:10` 自相矛盾 |
| `docs/distributed-navigation/README.md:34` | 当前源码共 20 个 ROS 包 | 现为 21 个（含 `usv_ladds_bridge`）；`README.md:58` 已写 21 |
| `docs/distributed-navigation/README.md:22` | `controller/  # PID / LQR` | 还有 ADRC 与定点保持控制器 |
| `docs/distributed-navigation/README.md:47`、`README.md:186` | 本轮使用 `/home/usv/module-layout-20260914/install` | 现在 LaDDS 流程用 base + 仓库 `install/`；该句可改为"历史验证环境" |
| `scripts/control/README.md:5` | 运行时控制循环……仍由原工程 `sim_env` 负责 | 现在由 `usv_control` 负责 |

- **建议**：按上表逐句修改，不动其他段落。CLAUDE.md 和 README.md 的改动也可以留到第 3 步收尾时一起做。
- **风险**：低（只改文档）。
- **验证方法**：链接扫描；逐条 grep 新写的路径或名称确实存在。

#### 08 ｜ C 文档 ｜ `docs/superpowers/` 移入历史区

- **现状**：8 份 `plans/` 和 1 份 `specs/`，是 2026-09-13 至 09-16 AI 代理执行任务时写的分步计划（开头写着 "For agentic workers: REQUIRED SUB-SKILL..."）。
- **问题**：属于过程文件，不是使用说明；`README.md:256` 已建议交接时排除。放在 `docs/` 一级子目录里，容易被当成现行设计。
- **依据**：被引用的位置只有 `docs/distributed-navigation/requirements-clarification.md:77` 和 `superpowers` 内部互相引用。
- **建议**：`git mv docs/superpowers docs/history/process`，更新上述 1 处链接和 `README.md:256` 的路径。
- **风险**：低。
- **验证方法**：链接扫描；`git grep "docs/superpowers"` 只剩历史证据中的引用。

#### 09 ｜ C 文档 ｜ 迁移期历史文档移入 `docs/history/`

- **现状 / 依据**：

  | 文件 | 为什么是历史（核实依据） |
  | --- | --- |
  | `docs/python-migration-status.md`、`-validation.md`、`-differences.md` | 固定在 `dfb1fb7` 源码版本（status 第 3-4 行），描述的是 13 包时期 |
  | `docs/python-navigation-guide.md` | 6 处旧路径（`src/core/common`、`src/core/controller/controller` 等）已不存在；`README.md:270` 已注明"旧包路径……应按当前 README 更新理解" |
  | `docs/技术文档.md` | 原 C++ 版本技术文档 v1.0（2026-09-10，`ed4ed18`），含 32 处 C++/cpp 描述、28 处 `D:/` 旧路径；没有任何文档链接它 |
  | `docs/mmwave-simulation.md` | 文件开头自述"已移除毫米波雷达代码" |
  | `docs/runtime-topic-graph-20260918.{md,dot,png,svg}` | 2026-09-18 从运行实例采样的快照，之后新增了 0051/0053/0054、`usv_ladds_radar` 等；png 有 819 KB |

- **建议**：移到 `docs/history/python-migration/`、`docs/history/2026-09-10-cpp-baseline-technical-doc.md`（中文文件名改为 ASCII，避免 git 路径转义和跨平台问题）、`docs/history/2026-09-18-runtime-topic-graph/`。正文不改，只更新指向它们的链接（`README.md:270-271`；这几份文档之间的互链会随相对位置一起保持）。
- **风险**：低。文件名改了以后，外部（如聊天记录、别人的书签）里的旧路径会失效。
- **验证方法**：链接扫描；`git log --follow` 能追到改名前的历史。

#### 10 ｜ B 命名 ｜ `README.python.md` 改名为 `README.md`

- **现状**：`path_planner/` 和 `path_processor/` 包里只有 `README.python.md`，没有 `README.md`。这个后缀来自 Python 迁移期（当时 C++ 原 README 还在）。
- **问题**：GitHub 和多数工具只自动展示 `README.md`；后缀式命名与其他包不一致。
- **依据**：唯一引用是 `src/core/decision_module/path_processor/README.python.md:59`（链接到 path_planner 的 README.python.md）；`setup.py`/`CMakeLists.txt` 未引用。
- **建议**：两个文件 `git mv` 为 `README.md`，更新那 1 处链接。
- **风险**：低。
- **验证方法**：链接扫描；`git grep README.python` 只剩 `deliverables/` 内的旧副本。

#### 11 ｜ C 文档 ｜ 设计类文档归入 `docs/design/`

- **现状**：设计决策分散在 `docs/adr/`（3 份 ADR）、`docs/planning-control-interface.md`、`docs/controller-design-brief.md`、`docs/distributed-navigation/requirements-clarification.md`、`docs/distributed-navigation/module-package-organization.md`。
- **建议**：移到 `docs/design/adr/`、`docs/design/planning-control-interface.md` 等。被链接的位置：`README.md:252,268`、3 份 ADR 指向 requirements-clarification（`adr/0001:35`、`0002:21`、`0003:14`）、`docs/distributed-navigation/README.md:126`、`docs/station-guard-migration.md:7`、`docs/control-autopilot-interface.md:3`、`docs/controller-design-brief.md:5,134`。
- **风险**：低到中，链接较多，但都是文档。
- **验证方法**：链接扫描。

#### 12 ｜ A 代码 ｜ `docs/examples/trajectory_generation/generate.py` 旧路径失效

- **现状**：LOS 轨迹生成教学示例，配套 `trajectory.csv`、`example.json` 和一张 154 KB 的 PNG。
- **问题**：`generate.py:10` 把 `src/core/common`、`src/core/trajectory_planner` 加入导入路径，`:42` 读取 `src/core/trajectory_planner/trajectory_planner/los_trajectory_planner.py`。这两个目录在模块整理后已不存在。按路径判断脚本已无法运行（未实际运行）。
- **建议**：选项 A 把两处路径改成 `src/core/shared/common`、`src/core/decision_module/trajectory_planner`（2 行代码改动，推荐）；选项 B 整个示例目录移入 `docs/history/`。
- **风险**：低。不影响任何包和测试；但 `example.json` 里记录的源码哈希会与新运行结果不一致，重跑会覆盖输出。
- **验证方法**：`python3 -B docs/examples/trajectory_generation/generate.py` 能运行；先不提交重新生成的输出文件，交给用户决定。

#### 13 ｜ B 命名 ｜ 命名规范与术语定稿写入文档

- **现状**：没有成文的命名规范；`CONTEXT.md` 没有定义"刹停""终点值守""定点值守""航路"。
- **建议**：第 6 节的规范草案经用户修改定稿后，写入 `docs/reference/naming-conventions.md`（或 `CONTRIBUTING.md`），并在 `CONTEXT.md` 补充术语（措辞依 Q7 决定）。
- **风险**：低（只写文档，不改既有名称）。
- **验证方法**：人工审阅。

### 中风险（文档大合并、测试/脚本行为、带代码引用的路径）

#### 14 ｜ C 文档 ｜ 值守类 5 份文档归并

- **现状**：`station-guard-guide.md`（37 KB）、`-implementation.md`、`-validation.md`、`-debugging.md`、`-migration.md`，共约 135 KB，描述 `usv_mission` 的定点值守、绕圆、阵位保持。其中 implementation、validation、debugging 三份的开头都写明"迁移前历史记录"。
- **问题**：
  1. 5 份文档里有大段重复的数字（713 项测试、matrix006 21/21 等）。
  2. 有 3 处断链（见 04），6 处旧路径。
  3. 文件名 `station-guard` 与桥包里的 `README.stop-guard.md`（0331 刹停 + 0211/taskType=80 终点值守）读起来几乎一样，含义却不同。
- **建议**（取决于 D3 对 `usv_mission` 的决定）：
  - 若继续维护：`station-guard-guide.md` 改为 `docs/guides/mission-station-keeping.md`（修断链），其余 4 份移入 `docs/history/station-guard/`。
  - 若冻结：5 份全部移入 `docs/history/station-guard/`，`usv_mission/README.md` 加一句"保留、未维护"。
- **风险**：中（文件多、互链多：`README.md:269` 等 10 余处），但不涉及代码。
- **涉及文件**：5 份 `docs/station-guard-*.md`、`README.md`、`src/core/mission_module/usv_mission/README.md`
- **验证方法**：链接扫描。

#### 15 ｜ C 文档 ｜ LaDDS 6 份文档归并

- **现状 / 重叠关系**：

  | 文档 | 内容 | 与谁重叠 |
  | --- | --- | --- |
  | `docs/ladds-multi-waypoint-quickstart.md` | 0211 多航点：构建、启动、报文、0052/0053 语义（现行主流程） | 桥 README:28-33、README.single-leg |
  | `docs/ladds-0411-quickstart.md` | 0411 本船导航链路 | 桥 README:37-41、表格 |
  | `docs/ladds-0141-quickstart.md` | 0141 收障碍、栅格、双容器提示 | 桥 README:43-48 |
  | `src/.../usv_ladds_bridge/README.md` | 0054、0051、中继原理、早期纯桥测试、话题表 | 以上全部 |
  | `src/.../usv_ladds_bridge/README.single-leg.md` | 早期 0211 单段联调、travelId 约定 | multi-waypoint |
  | `src/.../usv_ladds_bridge/README.stop-guard.md` | 0331 刹停、taskType=80 终点值守、0212 靠离泊 | 桥 README:3-11 |

- **与代码/配置不一致处**（本步只列出）：
  - 航点半径：文档写 1.5 m（`ladds-multi-waypoint-quickstart.md:5,137`、桥 `README.md:30`），`route.yaml:72` 为 `3.0`（`83ba3c2` 改），代码默认 1.5（`route_node.py:38`）。见 Q4。
  - `ladds-0411-quickstart.md:62-71` 写"`config/route.yaml` 中已配置 origin 30.5453/114.392931"，实际 route.yaml 已是 39.9450901/116.3342309。见 D1。
  - `ladds-multi-waypoint-quickstart.md:89` 写 "`taskType` 必须为 16"，代码已接受 16 和 80（`route_task.py:106`）。
  - 桥 `README.md:116-126`、`README.single-leg.md:56-63,167` 仍使用 `/tmp/usv_ladds_*_20260915` 独立安装区；现流程是仓库 `install/`。
  - `start_ladds_all.bash` 没有任何文档提到；`start_ladds_split_sim.bash` 没有独立说明，只有 0141 文档第 67 行提到 split_nav。
- **建议**：建立 `docs/guides/ladds/`（快速联调主文档、双容器/一键启动、Windows 中继）和 `docs/reference/ladds-protocol.md`（主题号与 JSON 字段总表、0052/0053/0331/80/0212 语义）。桥包 README 只保留包级说明（节点、参数、代码结构）。`README.single-leg.md` 中"航线编号与执行约定"一节并入协议参考，其余移入 `docs/history/ladds/`。不改主题号、JSON 字段和话题名。
- **风险**：中。合并内容需要逐段判断取舍；被 `CLAUDE.md:77` 等引用。建议在 D1、Q4 决定后执行，避免改两遍。
- **涉及文件**：上表 6 份文档、`docs/examples/ladds-*.json`、`README.md`、`CLAUDE.md`
- **验证方法**：链接扫描；按新文档的命令清单逐条核对脚本参数（`--help` 输出），不启动 ROS。

#### 16 ｜ C 文档 ｜ 控制类 5 份文档整理

- **现状**：
  - `docs/control-autopilot-interface.md`：第 3 行自述"接口定义已更正……以 planning-control-interface 为准"，内容已被取代。
  - `docs/planning-control-interface.md`：规划—航控接口设计稿（2026-09-12），部分未落地（自述）。
  - `docs/controller-design-brief.md`：给新控制器开发者的任务书（2026-09-14）。
  - `docs/controller-development-guide.md`：新控制器接入步骤；文中引用的代码路径都存在。
  - `docs/0914-controller-integration.md`：ADRC 接入、调参与回退；`:62,69,94` 用的是 `D:\ROS2轻量仿真系统` 旧路径。
- **问题**：`control-autopilot-interface.md` 被代码引用：`scripts/control/export_control_workspace.py:150-152` 在导出控制工作区时，若该文件存在就复制它；`scripts/control/test_export_control_workspace.py:32,65,135-152` 用临时目录测试这一行为。直接移动不会让测试失败，但会让导出包悄悄少一份文档。
- **建议**：`controller-development-guide.md`、`0914-controller-integration.md` → `docs/guides/control/`（修 D: 路径）；`controller-design-brief.md`、`planning-control-interface.md` → `docs/design/`（见 11）；`control-autopilot-interface.md` 有两个选项：A 原地保留并在索引中标为"已取代"（推荐，不碰代码）；B 移入历史区，同时改 `export_control_workspace.py:150` 的路径并跑 `scripts/control/test_export_control_workspace.py`。
- **风险**：中（选项 B 涉及代码）。
- **涉及文件**：上述 5 份文档；选 B 时加 `scripts/control/export_control_workspace.py`
- **验证方法**：链接扫描；选 B 时运行 `python3 -m pytest scripts/control/test_export_control_workspace.py -q`。

#### 17 ｜ A 脚本 ｜ `scripts/build.sh` 的两个问题

- **现状**：`build.sh` 是原作者留下的全量构建脚本，会汇总 `compile_commands.json` 供 clangd 使用。`main.sh:20` 在找不到 install 时会提示用户运行它。
- **问题**：
  1. `scripts/build.sh:17-19` 没有 `--base-paths src`，会把 `deliverables/usv_navigation_handover_20260914/` 里的旧包副本一起收集。CLAUDE.md 已经点出这一点。
  2. 它用 `--symlink-install`，而 `scripts/build_ladds_route.bash:62-63` 注明"workspace install is a copy install, so do not switch it to --symlink-install"。两个脚本往同一个 `install/` 写，混用时安装区里会同时存在 symlink 和副本（实际后果未核实）。
- **建议**：加 `--base-paths src`（这一项无争议）；`--symlink-install` 是否保留由 Q5 决定。
- **风险**：中。会改变构建行为，但不改导航运行逻辑。
- **涉及文件**：`scripts/build.sh`
- **验证方法**：在 WSL 用 scratchpad 脚本执行 `bash scripts/build.sh`（会实际构建，需用户确认时机，避免与正在运行的仿真争用 install），然后运行 `run_distributed_source_tests.bash` 到新的输出目录。

#### 18 ｜ A 测试 ｜ `/tmp` 旧默认路径导致 LaDDS 测试恒跳过

- **现状 / 依据**：
  - `scripts/validation/test_ladds_startup_ros.py:16` 的默认安装区是 `/tmp/usv_ladds_single_leg_20260915/install`，`:18-19` 在该路径没有 `setup.bash` 时整体 `skipif`。现在的流程不再生成这个目录，所以这个测试**一直被跳过**。
  - `scripts/validation/validate_ladds_multi_gazebo.py:105` 写死 `LADDS_SINGLE_LEG_INSTALL='/tmp/usv_ladds_multi_waypoint_20260915/install'`。
- **问题**：跳过的测试在汇总里看起来像"没问题"，实际从未执行。
- **建议**：默认值改为仓库 `install/`（与 `start_ladds_route.bash:9` 一致），环境变量覆盖方式保持不变。
- **风险**：中。改完以后测试会真正运行，可能暴露出新的失败，需要如实记录。
- **涉及文件**：`scripts/validation/test_ladds_startup_ros.py`、`scripts/validation/validate_ladds_multi_gazebo.py`
- **验证方法**：在 WSL 隔离域（`ROS_DOMAIN_ID=192`、`ROS_LOCALHOST_ONLY=1`）运行该测试文件，记录通过/失败/跳过数。

#### 19 ｜ A 测试 ｜ `pytest.ini` 默认范围与全量回归范围不一致

- **现状**：有两种跑测试的方式，收集范围不同。
  - 裸跑 `python3 -m pytest`：按 `pytest.ini:3` 的 `testpaths`，包含 `scripts/validation`、`scripts/hybrid_tuner`，**不含** `scripts/control`。
  - 全量回归 `scripts/validation/run_distributed_source_tests.bash:31`：显式执行 `python3 -m pytest src scripts`，会收集 `scripts/` 下全部 `test_*.py`，**包含** `scripts/control/test_speed_tuner.py` 和 `test_export_control_workspace.py`。
- **问题**：同一个仓库有两个"全部测试"的定义。平时裸跑 pytest 的人会漏掉 2 个测试文件。
- **依据**：`git ls-files 'scripts/**/test_*.py'` 中不在 `testpaths` 下的只有上述 2 个文件。
- **建议**：先在 WSL 单独运行 `python3 -m pytest scripts/control -q`，确认是否通过（未核实）；通过后把 `scripts/control` 加入 `pytest.ini` 的 `testpaths`，让两种方式一致。
- **风险**：中（改变默认测试范围；若这两个测试本来就不过，会显现出来）。
- **涉及文件**：`pytest.ini`
- **验证方法**：改动前后各运行一次 `python3 -m pytest --collect-only -q`，比较收集到的用例数。

#### 20 ｜ A 脚本 ｜ 14 个当前无人引用的验证/探针脚本

- **证据**：以下脚本（都在 `scripts/validation/` 下）在当前代码、脚本、现行文档中都没有被引用（`git grep`，排除 `deliverables/` 与 `docs/validation/`），只在历史证据里出现过或完全没出现过：

  | 脚本 | 最后提交 | 历史证据引用数 |
  | --- | --- | --- |
  | `check_ladds_0211_plan.py` | `cfe8081` 2026-09-23 | 0 |
  | `gui_smoke.py` | `b9cd7fc` 2026-09-16 | 169 |
  | `hybrid_live_probe.py` | `b9cd7fc` | 1 |
  | `ladds_capture.py`、`ladds_live_trace.py`、`ladds_path_probe.py`、`ladds_ros_probe.py`、`ladds_roundtrip_peer.py` | `b9cd7fc` | 各 1 |
  | `ladds_sdk_probe.py` | `b9cd7fc` | 0 |
  | `plot_adrc_validation.py`、`plot_ladds_multi_waypoint.py` | `b9cd7fc` | 0 |
  | `plot_ladds_single_leg.py` | `b9cd7fc` | 1 |
  | `process_sampler.py` | `b9cd7fc` | 172 |
  | `validate_ladds_multi_gazebo.py` | `04d48c7` 2026-09-20 | 1 |

- **问题**："无人引用"不等于"没用"：它们是生成历史证据的工具，以后复现证据时可能用到。
- **建议**：不删除、不移动。在 `scripts/validation/README.md` 末尾加一节"一次性诊断工具"，逐个写明用途和对应的证据目录。
- **风险**：只加文档时为低；如果改为移动到 `scripts/validation/archive/`，历史证据里记录的命令路径会失效，风险为中。
- **涉及文件**：`scripts/validation/README.md`
- **验证方法**：链接扫描。

#### 21 ｜ B 命名 ｜ LaDDS 仿真 profile 的位置与名称

- **现状**：
  - `scripts/start_ladds_single_leg_sim.bash` 名字是"单航段"，实际是所有 LaDDS 仿真入口共用的核心 profile：`start_ladds_route_sim.bash:8`、`start_ladds_split_sim.bash:8` 都转发到它，`validate_ladds_multi_gazebo.py:107` 也调用它。
  - 它依赖的生产配置放在验证目录：`scripts/validation/config/ladds_single_leg_dynamics.yaml`、`ladds_single_leg_world.py`（`start_ladds_single_leg_sim.bash:9,64-65`；`start_ladds_split_nav.bash:9,109,125`；`.cursor/rules/workspace-install.mdc` 也写了这个路径）。
- **问题**：名字和位置都会误导人，以为它们只服务于验证或单航段。
- **建议**：把核心 profile 改名为 `scripts/ladds_sim_profile.bash`，配置移到 `scripts/ladds/profile/`；**旧路径全部保留转发壳**（`start_ladds_single_leg_sim.bash` 变成一行 `exec bash 新脚本 "$@"`，YAML 旧位置保留副本）。
- **风险**：中。用户的两个 Docker 容器里有这些脚本的副本（2026-09-28 记录），仓库改名后容器副本不会自动同步；环境变量名 `LADDS_SINGLE_LEG_INSTALL`、`LADDS_SINGLE_LEG_WORLD` 等是脚本接口，不建议改名。
- **建议优先级**：低。可以只在文档中说明，不实际改名。
- **涉及文件**：`scripts/start_ladds_*.bash`、`scripts/validation/config/ladds_single_leg_*`、`.cursor/rules/workspace-install.mdc`、LaDDS 文档
- **验证方法**：每个入口脚本运行 `--help`（不启动 ROS），比较前后输出；`bash -n` 语法检查。

#### 22 ｜ A 根目录 ｜ `scripts/colregs_demo/` 去留

- **现状**：COLREGs + A* + LOS 会遇实验台 V2，单文件 HTML/JS 原型，含 Node 测试和截图证据，共 5.1 MB；`b9cd7fc`（2026-09-16）后没有再改。
- **问题**：README.md 写明"当前版本未实现 COLREGs 决策"（`README.md:26`），主线文档也没有链接它，接手者很难知道它的定位。
- **依据**：在它自己的目录之外没有任何引用。
- **建议**：三选一（Q8）：保留并在 `docs/README.md` 标为"独立原型"（推荐，零风险）；移到 `tools/colregs_demo/`；移出仓库。
- **风险**：保留为零；移动或移出为中。
- **涉及文件**：`scripts/colregs_demo/**`
- **验证方法**：链接扫描。

#### 23 ｜ A 根目录 ｜ `rmp.code-workspace`、`.cursor/`、`docker/`

- **现状**：
  - `rmp.code-workspace`：原作者的 VS Code 工作区配置（clangd-19 路径、catkin `devel/`），`ed4ed18` 基线起就存在，无引用。
  - `.cursor/rules/workspace-install.mdc`：给 Cursor AI 的规则，内容与 CLAUDE.md 的运行环境一节重复（路径、base、动力学文件）。
  - `docker/humble/`：Humble 开发镜像；`docker/noetic/`：ROS1 历史镜像。`docs/技术文档.md:652` 记载 noetic Dockerfile 引用了当前不存在的 `3rd/conanfile.py`（未重新核实）。`README.md:91,254` 写明 Docker 未在当前版本验证。用户现在的 LaDDS 双容器是用自己提交的镜像，不是用这里的 Dockerfile 构建的（2026-09-28 记录，未重新核实）。
- **建议**：`rmp.code-workspace` 和 `.cursor/` 保留（个人工具配置，删除没有收益）；`docker/noetic/` 可移入 `docker/history/` 或删除，由用户决定（Q8）。
- **风险**：中（docker 路径可能被用户的外部流程使用，未核实）。
- **涉及文件**：`rmp.code-workspace`、`.cursor/`、`docker/`
- **验证方法**：`git grep docker/noetic`。

### 高风险（对外接口、代码内部结构、仓库历史；多数建议只记录不改）

#### 24 ｜ B 命名 ｜ `usv_ladds_bridge` 内的 `single_leg` 命名

- **现状**：
  - `usv_ladds_bridge/single_leg.py` 名字是单航段，实际被 `route_task.py:13`、`berth_task.py:13`、`route_node.py:20` 共用（`make_completion`、`make_progress`），属于航路协议的公共工具。
  - `single_leg_node.py` 只有一行别名：`from .route_node import RouteNode as SingleLegNode, main`。
  - 可执行文件 `ladds_single_leg_node`（`CMakeLists.txt:13`）、`launch/single_leg.launch.py`（"旧入口：转发到 route.launch.py"）、`config/single_leg.yaml` 是对外入口。
- **建议**：可执行文件名、launch 文件名、YAML 文件名都属于对外接口，**不改**。内部模块 `single_leg.py` 若要改名为 `route_protocol.py`，需要保留 `single_leg.py` 转发壳，并重新构建、跑全部桥测试。收益不大，**建议只记录**。
- **改名代价**：安装区里的可执行文件和 launch 名称会变，用户的启动命令、Docker 容器副本、基线安装区都会受影响。
- **验证方法**（若执行）：重新构建 `usv_ladds_bridge`，运行 `python3 -m pytest src/core/communication_module/usv_ladds_bridge/test -q` 和全量回归。

#### 25 ｜ A 代码 ｜ `usv_mission/configuration.py` 是分叉副本

- **现状**：`src/core/mission_module/usv_mission/usv_mission/configuration.py`（341 行）与 `src/core/shared/usv_navigation/usv_navigation/configuration.py`（357 行）结构相同，`diff` 有 92 行不同。mission 版多了 `mission.*` 键，ADRC 默认值也不同（mission：`k_i_speed 0.8`、`force ±8`）。
- **问题**：参数校验规则要在两处维护；两边已经开始分叉。
- **建议**：本次整理**不合并**（合并会改变 mission 的运行行为）。只在 `usv_mission/README.md` 记录这一事实；是否合并取决于 D3。

#### 26 ｜ A 根目录 ｜ `0924controller/`、`USV_control/`

- **现状**：
  - `0924controller/`（6 个文件，136 KB）：2026-09-24 交付的控制器原始源码。`thrust_allocator.py` 与 `controller/adrc_core/thrust_allocator.py` 哈希相同；`adrc_controller.py`、`pi_controller.py`、`heading_linear_regression_filter.py` 与 adrc_core 中的版本哈希不同（已移植并修改，`adrc_core/SOURCE.md:61` 有说明）；`lateral_controller.py` 尚未移植。
  - `USV_control/`（12 个文件）：更早的控制参考算法（NN 增益调度等），`b9cd7fc` 起没有变化。
- **建议**：保留，作为算法来源证据。只在 CLAUDE.md"不属于当前运行链"列表中加入 `0924controller/`、删除已不存在的 `adrc/`（属于 07）。目录本身不动。
- **风险**：移动或删除为高（`SOURCE.md` 依赖它做来源核对）。

#### 27 ｜ A 代码 ｜ `sim_env/config/` 下的原作者算法模板

- **现状**：`sim_env/config/controller/{apf,dwa,lqr,mpc,orca,pid,rpp}_controller_params.yaml`、`path_planner/{evolutionary,graph,sample}_planner_params.yaml`、`velocity_planner/`、`robots/otter/{ekf,gps,trajectory}_params_otter.yaml`。它们会随 `sim_env/CMakeLists.txt:33` 安装，但代码不加载（`usv_navigation/launch_support.py:77` 专门打印 "graph_planner_params.yaml is an inactive template and is not loaded"）。
- **建议**：保留不动（`README.md:26` 已说明"同名历史配置模板不代表算法已经接入"）。删除会改变安装区内容，`launch_support.py:77` 的提示也要跟着改。
- **风险**：高（改动安装内容和基线对照）。

#### 28 ｜ B 命名 ｜ 通用 ROS 包名

- **现状**：`common`、`controller`、`perception`、`path_planner`、`path_processor`、`trajectory_planner` 这 6 个包没有 `usv_` 前缀；节点包都带前缀（`usv_decision`、`usv_control` 等）。
- **问题**：名字过于通用，将来与其他 ROS 包同名的风险较大（例如 `common`）。
- **改名代价**：安装路径 `install/<包名>/`、Python 导入名（全部源码和测试中的 `from common.models import ...`）、`package.xml` 依赖、`setup.py`、`build_ladds_route.bash:10-11` 的包列表、`killpro.sh:26` 的匹配串、`conftest.py:7-13`、`pytest.ini:3`、`run_source_tests.bash` 的 PYTHONPATH，以及**基线安装区** `/home/usv/.local/share/usv_ros_motion_planning/base/install` 里的同名旧包（改名后新旧包会同时存在于 overlay 链中）。
- **建议**：**不改**。只写进命名规范：新包一律加 `usv_` 前缀。

#### 29 ｜ B 命名 ｜ 节点名、参数前缀、YAML 历史拼写

- **现状**：决策节点对外仍叫 `usv_simulation`，参数前缀 `usv_simulation.controller.` / `usv_simulation.trajectory_planner.`（代码常量 `C`、`T`）；YAML 键 `control_frequncy`（少一个 e）由 `configuration.normalize_parameters` 映射到 `control_frequency`（`controller_params_otter.yaml:4-5`）。
- **改名代价**：`ros2 param set/dump` 命令、所有项目 YAML 的展平键、航控对参数快照的一致性校验、验收脚本、历史证据里的参数快照都会受影响。
- **建议**：**不改**。在命名规范中注明这是保留的兼容名称。

#### 30 ｜ A 根目录 ｜ `deliverables/` 是否移出仓库

- 见 D3。工作区占 320 MB、2617 个被跟踪文件；`.git` 目录 599 MB。
- **关键事实**：只在当前分支 `git rm` 不会让仓库变小，历史里仍保留这些 zip；真正瘦身需要改写历史（`git filter-repo`），这会影响所有协作者和所有分支，风险很高。

#### 31 ｜ A 分支 ｜ 本地与远端在 0141 实现上分叉

- 见第 0 节。远端 `1859419` 用 LaserScan 实现 0141；本地 `c7a55d6` 起用 `radar_0141.py`，之后 `d37a775` 又把 0141 改成"只收不发"。两边都删除了 `costmap_0141*`，但新增的文件不同。
- **建议**：本次整理不处理。整理分支以后若要推送或合并，需要先决定以哪边为准（Q6）。

---

## 4. 文档清单（C 类全表）

"是否过时"一列都对照了现行代码或配置；没有逐段核对的写"未逐段核实"。

### 4.1 根目录与 `docs/`

| 文档 | 用途 / 读者 | 是否过时（依据） | 与谁重复 | 建议 |
| --- | --- | --- | --- | --- |
| `README.md` | 项目总入口 / 接手者 | 部分过时：见 07、D2 | 与三模块 README 的结构、启动段重复 | 保留，更新（07、第 3 步） |
| `CLAUDE.md` | AI 助手工作说明 | 部分过时：`:77`、`:85` | `.cursor/rules/…mdc` | 保留，更新（07） |
| `CONTEXT.md` | 术语表 / 全体 | 未过时，但缺"刹停/终点值守/定点值守/航路" | — | 保留，补术语（13、Q7） |
| `docs/distributed-navigation/README.md` | 三模块接口与话题全表 / 开发者 | 部分过时：见 07 | README.md | 保留为主参考 → `reference/`（11 之后） |
| `docs/distributed-navigation/requirements-clarification.md` | 拆分需求问答 / 设计 | 历史需求，被 3 份 ADR 链接 | — | `design/`（11） |
| `docs/distributed-navigation/module-package-organization.md` | 2026-09-14 目录整理需求 | 已落地 | `superpowers/plans/2026-09-14-module-package-organization.md` | `design/` 或 `history/`（11） |
| `docs/adr/0001–0003` | 架构决策记录 | 现行（0003 自述 proposed 未采纳） | — | `design/adr/`（11） |
| `docs/planning-control-interface.md` | 规划—航控接口设计稿 | 设计稿，部分未落地（自述） | control-autopilot-interface | `design/`（11、16） |
| `docs/control-autopilot-interface.md` | 早期接口稿 | 自述已被取代（`:3`） | planning-control-interface | 原地标注或归档（16） |
| `docs/controller-design-brief.md` | 新控制器任务书 | 未逐段核实；未提 0924 推力分配/0054 | development-guide | `design/`（16） |
| `docs/controller-development-guide.md` | 新控制器接入步骤 | 引用路径均存在；未逐段核实 | 0914 文档 | `guides/control/`（16） |
| `docs/0914-controller-integration.md` | ADRC 接入、调参、回退 | 部分过时：D: 路径 `:62,69,94` | development-guide | `guides/control/`（16） |
| `docs/ladds-multi-waypoint-quickstart.md` | 0211 多航点主流程 | 部分过时：半径、原点、taskType（15） | 桥 README、README.single-leg | `guides/ladds/`（15） |
| `docs/ladds-0411-quickstart.md` | 0411 本船导航 | 部分过时：原点（`:70-71`） | 桥 README | `guides/ladds/`（15） |
| `docs/ladds-0141-quickstart.md` | 0141 收障碍 | 当前（2026-09-29 更新） | 桥 README:43-48 | `guides/ladds/`（15） |
| `docs/examples/ladds-*.json`（6 份） | LaDDS 报文样例 | 坐标按 30.5453 原点计算，与 route.yaml 不一致（D1）；`single-leg.json` 与 `single-leg-54.json` 只差 `packageSeq`/`travelId` | 两份 single-leg | `reference/examples/`，D1 后复核 |
| `docs/examples/trajectory_generation/` | LOS 教学示例 | 脚本旧路径失效（12） | — | 修路径或归档（12） |
| `docs/explainers/*.html`（2） | Hybrid A* / LOS 可视化讲解 | 未逐段核实 | `scripts/hybrid_tuner/page.html:15` 链接它 | `guides/explainers/`，需同改 2 处链接 |
| `docs/python-migration-*.md`（3） | Python 迁移记录 | 历史（固定 `dfb1fb7`） | 互引 | `history/`（09） |
| `docs/python-navigation-guide.md` | 迁移期使用指南 | 过时：6 处旧路径 | README.md | `history/`（09） |
| `docs/技术文档.md` | 原 C++ 版技术文档 | 历史（C++、D: 路径） | README.md | `history/` 并改 ASCII 名（09） |
| `docs/mmwave-simulation.md` | 毫米波仿真 | 历史（自述） | — | `history/`（09） |
| `docs/runtime-topic-graph-20260918.*`（4） | 话题图快照 | 历史快照 | — | `history/`（09） |
| `docs/station-guard-*.md`（5） | 值守/阵位保持 | 3 份自述历史；guide/migration 有断链 | 互相重复 | 14 |
| `docs/superpowers/**`（9） | AI 过程计划 | 过程文件 | design 文档 | `history/process/`（08） |
| `docs/validation/**` | 历史证据 | 不判定，不改 | — | 加索引（03） |

### 4.2 包内与 `scripts/` 内的文档

| 文档 | 用途 | 是否过时 | 建议 |
| --- | --- | --- | --- |
| `usv_ladds_bridge/README.md` | 桥包说明 + 早期单点联调 | 部分过时：`/tmp` 安装区（`:116-126`）、半径 1.5（`:30`） | 精简为包级说明（15） |
| `usv_ladds_bridge/README.single-leg.md` | 早期 0211 单段联调 | 大部分历史：`/tmp` overlay（`:56-63,167`） | 拆分：约定并入协议参考，其余进历史区（15） |
| `usv_ladds_bridge/README.stop-guard.md` | 0331 刹停、类型 80 终点值守、0212 | 当前（2026-10-08 更新） | 并入协议参考或原地保留；名称见 Q7 |
| `controller/controller/adrc_core/SOURCE.md` | ADRC 移植来源 | 当前 | 原地保留 |
| `usv_control/config/README.md`、`usv_decision/config/README.md` | 指向共享配置 | 当前 | 原地保留 |
| `path_planner/README.python.md`、`path_processor/README.python.md` | 算法包说明 | 未逐段核实 | 改名 README.md（10） |
| `path_planner/test/fixtures/hybrid_long_leg/README.md` | 测试夹具说明 | — | 原地保留 |
| `usv_mission/README.md`、`config/mission/README.md`、`validation/README.md` | 任务模块 | 视 D3；validation/README 有 2 处旧本机路径 | 原地保留 |
| `perception/README.md`、`usv_perception/README.md` | 感知 | 未逐段核实 | 原地保留 |
| `src/sim_env/test/README.md` | 旧参照测试说明 | 当前；含 1 处旧本机路径 | 原地保留 |
| `scripts/validation/README.md` | 自动验收入口 | 当前；含 4 处旧本机路径（未逐条核实是否只是示例） | 原地保留，补"一次性工具"一节（20） |
| `scripts/control/README.md` | 控制数值工作区导出 | 部分过时（`:5` 写 sim_env） | 07 |
| `scripts/control/speed_tuner/README.md`、`DESIGN.md` | 速度调参台 | 当前 | 原地保留 |
| `scripts/hybrid_tuner/README.md` | Hybrid A* 观察台 | 当前 | 原地保留 |
| `scripts/colregs_demo/README.md` | COLREGs 原型 | 独立原型 | 22 |
| `scripts/validation/parity/**`、`diagnostics/**` | C++/Python 对照、诊断证据 | 历史证据 | 不改 |
| `deliverables/README.md` 等 | 交付物 | — | 不改（规则 4） |
| `图片/rosgraph.dot` | 0 字节空文件 | — | 移除（05） |

### 4.3 图片与图表类

| 文件 | 说明 | 建议 |
| --- | --- | --- |
| `docs/runtime-topic-graph-20260918.{png,svg,dot}` | 话题图快照（png 819 KB） | 随 09 进历史区 |
| `docs/examples/trajectory_generation/trajectory_generation.png` | 示例输出 | 随 12 |
| `scripts/colregs_demo/validation/**/*.png`（15 张） | 原型截图证据 | 随 22 |
| `scripts/validation/diagnostics/footprint_cost_semantics.{png,pdf}` | 诊断图 | 不改 |
| `src/sim_env/models/**/*.png` | Gazebo 模型贴图（不是文档） | 不改 |

---

## 5. `scripts/` 清单（A 类）

### 5.1 顶层入口脚本

| 脚本 | 做什么 | 被谁引用 | 结论 |
| --- | --- | --- | --- |
| `main.sh` | 普通导航主入口：加载 install、可选指向 VcXsrv、生成障碍、`ros2 launch sim_env main.launch.py` | README、CLAUDE、`start_sim_gui.ps1`、`test_main_launcher.py` | 在用，保留 |
| `build.sh` | colcon 全量构建并汇总 `compile_commands.json` | `main.sh:20` 提示、文档 | 在用，有问题（17） |
| `build_ladds_route.bash` | 以 base 为 underlay，增量构建 10 个包进仓库 `install/` | CLAUDE、quickstart、各 start 脚本的错误提示 | 在用，保留 |
| `killpro.sh` | 按命令行子串 SIGKILL 仿真进程 | `test_killpro_patterns.py` | 保留；规则禁止使用 |
| `start_sim_gui.ps1` | Windows 启动 VcXsrv 并调用 `main.sh` | README | 在用，保留 |
| `start_ladds_relay.ps1` | Windows LaDDS 中继（通用，可逐项指定话题） | 多份文档 | 在用；`:32` 旧 `D:\` SDK 路径和 `:54` Codex 运行时 Python 路径是本机兜底，可保留 |
| `start_ladds_route_relay.ps1` | 固定航路话题后调用通用中继 | quickstart | 在用，保留 |
| `start_ladds_bridge.bash` | 早期纯桥（只收发 `/ladds/*` 测试话题） | 桥 README、`test_ladds_startup_ros.py` | 早期测试入口，保留 |
| `start_ladds_route.bash` | 航路适配器 + 桥（tcp） | quickstart | 在用，保留 |
| `start_ladds_route_sim.bash` | 仿真入口（转发到 single_leg_sim） | quickstart | 在用，保留 |
| `start_ladds_single_leg.bash` | 旧名转发壳 → `start_ladds_route.bash` | quickstart 提及 | 兼容壳，保留 |
| `start_ladds_single_leg_sim.bash` | **所有 LaDDS 仿真共用的核心 profile** | route_sim、split_sim、split_nav、`validate_ladds_multi_gazebo.py` | 在用；名字误导（21） |
| `start_ladds_split_sim.bash` | 双容器的仿真端 | split_nav、all | 在用；无独立文档 |
| `start_ladds_split_nav.bash` + `ladds_split_nav.launch.py` | 双容器的算法端 | 0141 quickstart:67 | 在用 |
| `start_ladds_all.bash` | 单机一键：同时起 split 两半 | **无任何文档** | 是否在用未核实，需补文档（15） |

### 5.2 工具目录

| 目录 | 做什么 | 结论 |
| --- | --- | --- |
| `scripts/control/` | 控制算法数值工作区导出、速度 PI 调参台（README.md:11 推荐） | 在用；测试不在 `pytest.ini` 默认范围（19） |
| `scripts/hybrid_tuner/` | Hybrid A* 参数观察台，测试在 `pytest.ini` 中 | 在用 |
| `scripts/colregs_demo/` | COLREGs 原型 | 22 |
| `scripts/validation/` | 验收、回归、安装审计、诊断工具；`config/` 下混有 LaDDS 生产 profile（21） | 在用；14 个脚本无人引用（20） |
| `scripts/validation/parity/`、`diagnostics/` | 历史对照与诊断证据（parity 有 `COLCON_IGNORE`） | 不改 |

### 5.3 重复或相近的脚本

| 一组 | 关系 | 结论 |
| --- | --- | --- |
| `run_source_tests.bash`（14 行）/ `run_distributed_source_tests.bash`（35 行） | 前者是迁移期版本（默认域 174，无输出目录、无 JUnit）；后者是现行全量回归 | 前者仍被 `scripts/validation/README.md`、`build_clean.bash`、`usv_mission/validation/` 引用，保留 |
| `verify_install.py` / `verify_distributed_install.py` | 13 包时期 / 三模块时期的安装审计 | 前者被 `build_clean.bash`、`usv_mission/validation/build_missions.bash` 引用，保留 |
| `usv_mission/validation/run_source_tests.bash` | 与 `scripts/validation/run_source_tests.bash` 同名 | 同名不同目录，保留 |
| `start_ladds_relay.ps1` / `start_ladds_route_relay.ps1` | 通用 / 固定话题壳 | 合理分层，保留 |

---

## 6. 命名盘点与《命名规范草案》（B 类）

### 6.1 现状盘点

| 对象 | 现状 |
| --- | --- |
| ROS 包 / Python 包 | 全部 snake_case；包目录名与 `<name>` 一致（14 个 core 包已逐一核对）。6 个算法包无 `usv_` 前缀（28） |
| Python 模块 / 函数 / 类 | 全部 snake_case / PascalCase；`src/core`、`src/sim_env`、`scripts` 中没有驼峰函数定义 |
| 测试文件 | `test_*.py` 89 个，其中需要 ROS 的 18 个统一为 `test_*_ros.py` |
| Shell 脚本后缀 | `.bash` 25 个，`.sh` 7 个（`docker/` 4 个、`scripts/build.sh`、`main.sh`、`killpro.sh`）；`.gitattributes` 两种都设为 LF |
| 启动脚本前缀 | `start_ladds_<场景>[_<角色>]`：`route`、`route_sim`、`split_nav`、`split_sim`、`all`、`bridge`、`single_leg[_sim]`（旧） |
| `docs/` 文件名 | kebab-case 44 个、snake_case 1 个（`trajectory_generation/`）、中文名 1 个（`技术文档.md`）；日期写法三种：前缀 `2026-09-15-…`（superpowers）、后缀 `-20260918`（runtime-topic-graph）、数字前缀 `0914-…`（含义是"9 月 14 日的控制器"，不是序号） |
| `docs/validation/` 目录名 | 全部 kebab-case；编号后缀 `-001` 与日期后缀 `-20260915` 混用 |
| 包内 README | `README.md`、`README.python.md`（2 个）、`README.single-leg.md`、`README.stop-guard.md` |
| 配置文件 | `<用途>_params_<船型>.yaml`（`controller_params_otter.yaml`、`global_costmap_params_otter.yaml`）与 `<用途>_<船型>.yaml`（`dynamics_otter.yaml`）、`<用途>.yaml`（`mapping.yaml`、`route.yaml`）混用 |
| 中文术语（在 `*.md`/`*.py` 中出现的次数，不含历史证据与 superpowers） | 航控 170、航行控制 13、控制模块 6；决策规划 18、决策模块 6、规划模块 4；感知定位 11、感知模块 5；值守 54（定点值守 14、终点值守 5）；station-guard 61、stop-guard 4；停推 31、刹停 14、停船 10 |

### 6.2 规范草案（只约束新增和经批准的改名；已有对外名称不因此改动）

1. **目录与文件**
   - ROS 包、Python 包和模块、脚本、配置文件：snake_case。
   - `docs/` 下的文档和目录：kebab-case。
   - 文件名只用 ASCII；中文标题写在文档正文第一行。
2. **脚本**
   - 新的 Bash 脚本统一用 `.bash`（脚本都用了 `set -o pipefail` 等 Bash 特性）。现有 `main.sh`、`build.sh`、`killpro.sh` 已被文档、测试和用户习惯引用，**不改名**。
   - PowerShell 用 `.ps1`，供 Windows 双击的用 `.bat`。
   - 前缀：`start_<系统>_<场景>[_<角色>].bash` 用于启动，`build_*.bash` 用于构建，`run_*.bash` 用于运行测试，`check_*`/`verify_*` 用于只读检查，`plot_*` 用于画图。
   - 任何已有入口脚本改名时，旧名保留一行转发壳（规则 3）。
3. **文档**
   - 现行文档：无日期前缀，用名词短语，如 `ladds-route-quickstart.md`。
   - 过程或历史文档：日期前缀 `YYYY-MM-DD-主题.md`（与 superpowers 一致）。`runtime-topic-graph-20260918` 这种日期后缀在移动时改为前缀。
   - 包内 README：主说明叫 `README.md`；同一个包有多份专题说明时用 `README.<主题>.md`，主题用 kebab-case。
   - `docs/validation/` 保持现状，不改名。
4. **配置文件**：新文件用 `<用途>_params[_<船型>].yaml`。现有文件名被 launch 默认值、启动脚本和测试引用，属于接口，不改。
5. **Python**：模块 snake_case，类 PascalCase，测试 `test_<对象>.py`，需要真实 ROS 的测试为 `test_<对象>_ros.py`。新增 ROS 包加 `usv_` 前缀。
6. **保留的兼容名称**（不改，写进文档说明）：节点名 `usv_simulation`、参数前缀 `usv_simulation.controller.`/`usv_simulation.trajectory_planner.`、YAML 键 `control_frequncy`、`sim_env/sim_env/{configuration,runtime,simulation}.py`、`ladds_single_leg_node`、`single_leg.launch.py`、`single_leg.yaml`、`start_ladds_single_leg*.bash`、环境变量 `LADDS_SINGLE_LEG_*`。
7. **中文术语**（具体措辞待 Q7）：
   - 模块名称用"感知定位 / 决策规划 / 航控"；"航行控制"只指控制过程（与 CONTEXT.md 的定义一致）。
   - "停推"（零推力）、"刹停"（0331，主动制动到停稳，motion_mode `BRAKE`）、"停稳"三者区分，避免用含糊的"停船"。
   - "值守"有两种含义，必须带限定词：**任务值守**（`usv_mission` 的待机值守/绕圆值守，文件名 station-guard）和**终点值守**（0211 `taskType=80`，motion_mode `HOLD`，文件名 stop-guard）。

---

## 7. 目标文档结构草案

```text
docs/
├── README.md                     # 总索引（02）
├── guides/                       # 上手、启动、调参
│   ├── ladds/                    # 航路联调、双容器/一键启动、Windows 中继（15）
│   ├── control/                  # 新控制器接入、0914 ADRC 接入与调参（16）
│   └── explainers/               # Hybrid A*、LOS 讲解页
├── reference/                    # 接口、话题、参数、协议
│   ├── navigation-modules.md     # 现 distributed-navigation/README.md
│   ├── ladds-protocol.md         # 主题号、JSON 字段、0052/0053/0331/80/0212 语义（15）
│   ├── naming-conventions.md     # 13
│   └── examples/                 # LaDDS 报文、LOS 示例
├── design/                       # ADR 与设计稿（11）
│   ├── adr/
│   └── …
├── history/                      # 迁移记录、旧计划、历史话题图（08、09、14）
│   ├── python-migration/
│   ├── station-guard/
│   ├── ladds/
│   └── process/                  # 原 superpowers
├── maintenance/                  # 本类审计报告
└── validation/                   # 保持原样，只加索引（03）
```

说明：`docs/adr/` 也可以原地不动（ADR 习惯放在 `docs/adr/`），只要索引能找到即可。

---

## 8. D 类：需要拍板的冲突（当前值已重新核实，不替你选）

### D1 LaDDS WGS84 原点：仓库里现在有四组值

| 组 | 纬度 / 经度 | 出现位置 |
| --- | --- | --- |
| A（2026-10-08 新改） | 39.9450901 / 116.3342309 | `usv_ladds_bridge/config/route.yaml:42-43,51-52,61-62`（`21f0804`，从 C 组改来）。`route.launch.py:17`、`single_leg.launch.py:18` 默认加载它，所以 `start_ladds_route*.bash`、`start_ladds_split_nav.bash`、`start_ladds_all.bash` **实际运行的是 A 组** |
| B | 30.5453 / 114.392931 | `config.py:40-41,121-122`、`route_node.py:31`、`radar_node.py:36-37`（代码默认值）；`bridge.yaml:40-41,54-55`；`single_leg.yaml:25-26,33-34,43-44`；`scripts/validation/config/ladds_single_leg_world.py:10-11`（LaDDS 仿真运行时 world 副本的 `spherical_coordinates`）；`validate_ladds_multi_gazebo.py:44`；多数桥测试（如 `test_single_leg.py:11`、`test_route_task.py:13`）；文档 `ladds-0411-quickstart.md:70-71`（并声称这是 route.yaml 的值）、`ladds-multi-waypoint-quickstart.md:59`、`README.single-leg.md:24`；`docs/examples/*.json` 的坐标都是按 B 组算出来的 |
| C（route.yaml 上一版） | 30.5472 / 114.3891 | `test_obstacle_grid.py:20`、`test_inbound_obstacle_ros.py:22,68-69`、`check_ladds_0211_plan.py:97`；`test_berth_task.py:65-66` 把它当作靠泊点 |
| D | -33.7242 / 150.68 | 源 world 文件 `ocean.world:20-21`、`example_course.world`、`stationkeeping_vrx.world`（VRX 默认值，悉尼）。LaDDS 仿真时被替换成 B 组 |

- **影响（按经纬度差估算，未实际运行）**：用当前 route.yaml 运行时，把 `docs/examples/ladds-0211-three-waypoint.json`（按 B 组算的坐标）发进来，换算出的目标点与船相距约 1000 km（纬度相差约 9.4°），不可能正常导航。
- **测试为什么没发现**：`test_bridge_config.py:300-316` 只检查原点是 float、且 rx/route/radar 三处一致，不检查具体数值。
- **0411 的自洽性**：tx 编码和 rx 解码用的是同一份 route.yaml，所以船位换算本身是自洽的；不一致出现在外部系统发来的经纬度（0211/0212/0141）和这个原点之间。
- **可选方向**：
  - (a) A 组是现场或甲方给定的真实原点：route.yaml 保持 A；文档写明"现场原点"；示例报文按 A 重新计算；运行时 world 副本（`ladds_single_leg_world.py`）是否也改成 A，需另行决定（会改变仿真行为）。
  - (b) A 组是误改：route.yaml 改回 B（或 C），与代码默认值一致。
  - (c) 分开：现场联调用 A，纯仿真演示用 B，用两份参数文件区分（需要新增配置文件，偏离"最小改动"原则）。
- **需要用户提供的信息**：39.9450901 / 116.3342309 是从哪里来的？

### D2 默认控制器

| 层 | 当前值 | 依据 |
| --- | --- | --- |
| 代码兜底默认（YAML 不写时） | `pid` | `usv_navigation/configuration.py:39` |
| 主 YAML（main.launch 和所有 LaDDS 脚本都加载它） | `adrc`，`force_min/max ±361 N`、`speed_integral ±361` | `controller_params_otter.yaml:3,45-48`；`start_ladds_single_leg_sim.bash:120`、`start_ladds_split_nav.bash:126` |
| 专用 ADRC YAML | `adrc`，`force ±40 N`、`speed_integral ±30` | `adrc_controller_params_otter.yaml:3,37-40` |
| 测试 | 断言主入口默认是 `adrc` | `src/sim_env/test/test_launch_modules.py:106` |
| 文档 | 写默认 PID | `README.md:3`、`:147`（可选值缺 adrc）、`:193`（"PID / LQR"）；`docs/distributed-navigation/README.md:49` |

- **`adrc_controller_params_otter.yaml` 有没有被加载**：没有任何 launch 或启动脚本加载它。但它被这些工具和测试读取：`scripts/control/speed_tuner/engine.py:32`、`browser_smoke.py:16`、`scripts/control/control_demo.py:48`、`scripts/validation/test_adrc_numeric_probe.py:11`、`verify_distributed_install.py:59`（安装审计）、`src/sim_env/test/test_adrc_configuration.py:33-51`（断言它的值是 ±40/±30）。`docs/0914-controller-integration.md` 让用户通过 `controller_config:=` 手动指定它。所以 2026-09-24 那次审计的结论"没有任何脚本加载它"应修正为"没有任何启动入口加载它，但调参工具和测试在用"。
- **可选方向**：(a) 承认 ADRC（±361 N）是现行默认，只改文档（不改运行行为，符合本次范围）；(b) 把主 YAML 改回 PID（改变运行行为，还要改 `test_launch_modules.py:106`，超出本次"只整理"的范围）。另外还要决定：两份 ADRC 参数（±361 与 ±40）哪份是"推荐值"，文档如何表述。

### D3 `deliverables/` 和 `usv_mission`

- **`deliverables/`**：320 MB、2617 个文件，包括 5 个 zip（`usv_code_20260914.zip` 43 MB、`usv_code_zh_20260914.zip` 43 MB、`usv_deploy_20260924.zip` 49 MB（2026-09-29 在 `d37a775` 中加入）、`usv_navigation_handover_20260914.zip` 65 MB、`control_workspace.zip`）和解压目录 `usv_navigation_handover_20260914/`（125 MB，含 ROS 包副本，这正是必须加 `--base-paths src` 的原因）。被 `README.md:258`、`CLAUDE.md:85` 引用。
  - 选项：(a) 保留不动；(b) 只保留 `deliverables/README.md` 作说明，zip 移到网盘或 GitHub Release，在当前分支 `git rm`（仓库历史大小不变）；(c) 改写历史彻底瘦身（影响所有分支和协作者）。
- **`usv_mission`**：位于 `src/` 下，默认参与构建；测试在 `pytest.ini` 的 `testpaths` 中；普通导航主入口不启动它；带一份分叉的 `configuration.py`（25）；最近一次改动是 2026-09-24 的 0054 重构顺带修改。目前能否构建、测试是否通过：**未核实**（本轮没有运行）。终点值守（taskType=80）是在 `usv_control`/`usv_decision`/`controller/station_keeping_controller.py` 中实现的，并不依赖 `usv_mission`。
  - 选项：(a) 继续维护；(b) 冻结：代码不动，文档标为"保留、未维护"；(c) 移出构建（加 `COLCON_IGNORE`、从 `testpaths` 移除，这会改变构建和测试范围）。

---

## 9. 需要你决定的问题（含推荐和理由）

| 编号 | 问题 | 推荐 | 理由 |
| --- | --- | --- | --- |
| D1 | LaDDS 原点以哪组为准 | 先说明 A 组来源；若是现场原点，选 (a)，暂不改 world 副本 | 刚改的值最可能是现场需要；改文档和示例不影响运行 |
| D2 | 默认控制器的表述 | (a) 文档改为"默认 ADRC"，PID/LQR 为可选；说明两份 ADRC 参数的区别 | 符合"只整理不改行为"，测试已锁定 ADRC |
| D3a | `deliverables/` | (a) 暂时保留，等交付流程稳定后再考虑 (b) | (b)/(c) 收益有限（历史不变小），而 (c) 风险高 |
| D3b | `usv_mission` | (b) 冻结 | 不改代码，同时让接手者知道它不是主线 |
| Q4 | 航点到达半径以哪个为准：文档 1.5 m，还是 `route.yaml:72` 的 3.0 m | 以 `route.yaml` 3.0 m 为现行值，文档写明"代码默认 1.5、航路配置 3.0" | 3.0 是较新的改动（`83ba3c2`），且是实际加载的值 |
| Q5 | `build.sh` 是否保留 `--symlink-install` | 去掉，与 `build_ladds_route.bash` 统一为 copy install | 同一个 `install/` 不应混用两种安装方式 |
| Q6 | 远端 `1859419`（LaserScan 版 0141）与本地 radar 版分叉 | 本次整理不处理；推送前单独决定 | 涉及功能实现，不属于整理 |
| Q7 | "值守"的术语和文件名 | 采用"任务值守 / 终点值守"两个限定说法；暂不改文件名，只在 CONTEXT.md 和索引中解释 | 改说法即可消除歧义，改文件名会牵动很多链接 |
| Q8 | `colregs_demo/`、`docker/noetic/`、`rmp.code-workspace` 去留 | 全部保留，只在索引中标注定位 | 零风险；删除没有实际收益 |

---

## 10. 建议的执行顺序

1. **忽略规则和索引**（不碰任何现有内容）：01 → 02 → 03。
2. **小修**：04（断链）→ 05（运行垃圾）→ 06 → 10 → 12。
3. **你拍板 D1、D2、D3、Q4、Q7 之后**：07（无争议更正）→ 08 → 09 → 11 → 14 → 16 → 15（LaDDS 合并最后做，避免原点/半径改两遍）。
4. **脚本和测试**：19（先只运行不改配置）→ 18 → 17 → 20（只加文档）→ 21（可选）。
5. **代码内部命名**（24–29）：本轮建议只记录，不执行。
6. **收尾（第 3 步）**：README.md / CLAUDE.md / CONTEXT.md 的路径与术语、`docs/README.md` 定稿、命名规范落文档（13）、列出未处理项。

每一项单独提交，提交信息带编号，例如 `docs(reorg-02): add docs index`。

---

## 11. 未核实事项

- 基线安装区 `/home/usv/.local/share/usv_ros_motion_planning/base/install` 的实际内容（本轮未调用 WSL）。
- 远端分支在 2026-09-21 之后的状态（本轮未 fetch）。
- `usv_mission` 当前能否构建、测试是否通过。
- `scripts/control/test_*.py` 当前是否通过（19）。
- 用当前 route.yaml 原点发送示例报文时的实际运行表现（D1 中的距离只是按经纬度差估算）。
- `docs/explainers/*.html`、`controller-design-brief.md`、`perception/README.md` 等内容是否与当前算法逐段一致（只核对了引用路径）。
- `build.sh` 的 symlink 安装与 copy 安装混用时的实际后果（17）。
- 用户 Docker 容器内脚本副本与仓库的差异；`start_ladds_all.bash` 是否在实际使用。

---

## 12. 附：本轮使用的只读检查

- 链接扫描：遍历 `git ls-files '*.md'`（排除 `deliverables/`），解析 `[...](相对路径)`，检查目标是否存在。结果：检查 701 个链接，7 处断链（3 处在现行文档，4 处在 `docs/validation/`）。
- 引用检查：`git grep -l -F <文件名> -- . ':!deliverables' ':!docs/validation'`，并单独统计历史证据中的引用数。
- 忽略检查：`git check-ignore -v`、`git status --short --ignored`。
- 历史检查：`git log --format='%h %ad %an' -- <路径>`、`git show --stat <提交>`。

