# 验证工具路径定位：从自身或指定位置向上寻找包含 src 的源码根目录。
"""Find a source checkout without depending on validation directory depth."""
from pathlib import Path


# 逐级检查父目录，避免任务目录迁移后固定 parents 层数失效。
def find_source_root(start=None):
    location = Path(start or __file__).resolve()
    directory = location if location.is_dir() else location.parent
    for candidate in (directory, *directory.parents):
        if (candidate / "src").is_dir():
            return candidate
    raise ValueError(f"No source checkout containing src found above {location}")
