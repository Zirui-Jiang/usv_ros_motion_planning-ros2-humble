from pathlib import Path

from setuptools import find_packages, setup


PACKAGE = "usv_mission"
data_files = [
    ("share/ament_index/resource_index/packages", ["resource/" + PACKAGE]),
    ("share/" + PACKAGE, ["package.xml", "README.md"]),
]
for directory in ("launch", "config", "rviz"):
    for path in sorted(Path(directory).rglob("*")):
        if path.is_file() and "__pycache__" not in path.parts:
            data_files.append(("share/" + PACKAGE + "/" + path.parent.as_posix(), [str(path)]))

setup(
    name=PACKAGE, version="0.1.0", packages=find_packages(exclude=["test"]),
    data_files=data_files, install_requires=["setuptools", "numpy", "PyYAML"],
    zip_safe=True, maintainer="Winter", maintainer_email="913982779@qq.com",
    description="USV mission dispatch, lifecycle and navigation/control coordination",
    license="BSD-3-Clause", tests_require=["pytest"],
    entry_points={"console_scripts": [
        "mission_node = usv_mission.node:main",
        "mission_cli = usv_mission.mission_cli:main",
    ]},
)
