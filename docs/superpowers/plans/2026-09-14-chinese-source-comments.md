# 中文源码注释 Implementation Plan

> **For agentic workers:** Use superpowers:dispatching-parallel-agents to annotate the independent owned modules; the primary agent reviews and verifies the combined result.

**Goal:** 给上次源码包对应的当前工程源码补充准确、有帮助的中文注释，并生成中文注释版源码包。

**Architecture:** 只插入独立注释行，不更改现有代码、字符串、docstring、配置值、许可证或原有注释。按目录分工，先阅读实际实现再写模块职责、输入输出、关键算法和边界条件说明。

**Tech Stack:** Python、C++、ROS 2 Humble、Bash/PowerShell、CMake、YAML、XML。

**Spec:** 用户要求“给所有代码加中文注释，方便阅读”；此前要求源码包简洁，不添加冗长交接材料。

## Global Constraints

- 范围为 `deliverables/_chinese_comments_20260914/inventory.json` 中的文件，原始字节保存在同目录 `before/`。
- 不修改图像、网格模型、CSV/JSON/NPZ 测试数据、历史文档及旧交付快照。
- 注释解释职责、数据流、物理量含义和选择原因；不把每个简单赋值重复描述一遍。
- 每个目标文件至少有适合其实际用途的中文说明；核心类、关键函数和不明显分支补局部解释。
- Python 只新增 `#` 注释，不新增或修改 docstring；装饰器、编码声明和 shebang 保持有效。
- C++ 使用独立 `//` 注释；XML 在声明之后使用合法注释；Bash 不在续行和 heredoc 内容中插入注释。
- 不提交、重构、覆盖他人修改，不新增功能测试；用结构比较和已有回归证明代码行为未变。

## Tasks

- [x] 保存目标文件及原始快照，确定分工。
- [x] 航控、决策与公共层：`src/core/control_module/`、`src/core/decision_module/`、`src/core/shared/`。
- [x] 任务与感知层：`src/core/mission_module/`、`src/core/perception_module/`。
- [x] 插件层：`src/plugins/` 下源码、构建及配置。
- [x] 主代理处理仿真、场景配置、脚本与根测试配置：`src/sim_env/`、`src/user_config/`、`scripts/`、`conftest.py`、`pytest.ini`。
- [x] 全量审计只增加注释：Python AST/词法、C++ 词法、YAML 解析对象、XML 结构及其他文件逐行恢复比较；审阅核心注释。
- [x] 在独立 Linux 副本上运行现有源码/ROS 测试，注明复用安装区、不声称重新编译。
- [x] 更新简洁源码包并校验归档内容；交付仅源码、原有配置资源和已有简短 README。

## 完成记录

- 最终清单包含 404 个文件、2,289 行新增中文注释，包含补充发现的 GLSL、Ogre 材质、ERB 和模型索引；JSON 格式的 perspective 与数据文件保持原样。
- 404/404 文件原始行可恢复；Python AST/token、C++/GLSL/Ogre/CMake/Bash token、XML/YAML/INI 解析及 ERB 的 Ruby 块比较均通过。PowerShell 的 263 个有效 token 相同，语法检查通过。
- 最终独立源码副本测试 935 passed、0 skipped，35.96 秒；Bash 语法检查通过。使用已有 ROS 安装，未重新编译 C++、运行 GUI/GPU 或 Gazebo 场景矩阵。
- 交付 `deliverables/usv_code_zh_20260914.zip`，保留原简洁源码包的 663 个成员，仅替换 404 个已审计注释文件。归档 CRC 与每文件哈希已校验。
- 内部审计、原始快照和一次性工具仅保存在 `deliverables/_chinese_comments_20260914/`，没有收入交付包。
