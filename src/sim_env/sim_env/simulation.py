# 保留旧可执行入口的导入兼容性；普通导航实际运行在独立决策与航控进程中。
"""Compatibility executable: ordinary navigation now runs in separate processes."""
from usv_decision.decision import DecisionNode as Simulator, main

if __name__ == "__main__":
    main()
