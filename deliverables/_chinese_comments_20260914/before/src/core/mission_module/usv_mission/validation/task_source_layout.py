"""Find a source checkout without depending on validation directory depth."""
from pathlib import Path


def find_source_root(start=None):
    location = Path(start or __file__).resolve()
    directory = location if location.is_dir() else location.parent
    for candidate in (directory, *directory.parents):
        if (candidate / "src").is_dir():
            return candidate
    raise ValueError(f"No source checkout containing src found above {location}")
