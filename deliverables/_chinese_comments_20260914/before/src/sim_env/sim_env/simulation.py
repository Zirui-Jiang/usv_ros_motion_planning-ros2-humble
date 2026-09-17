"""Compatibility executable: ordinary navigation now runs in separate processes."""
from usv_decision.decision import DecisionNode as Simulator, main

if __name__ == "__main__":
    main()
