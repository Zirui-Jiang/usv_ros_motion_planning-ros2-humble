# 安装纯 Python perception 包及 ament 索引资源，测试代码不打包进运行模块。
from setuptools import find_packages, setup


setup(
    name="perception",
    version="0.0.0",
    packages=find_packages(exclude=("test",)),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/perception"]),
        ("share/perception", ["package.xml"]),
    ],
    install_requires=["setuptools", "numpy"],
    zip_safe=False,
    maintainer="winter",
    maintainer_email="913982779@qq.com",
    description="ROS-independent LiDAR integer-evidence occupancy mapping and scan queues",
    license="TODO",
    extras_require={"test": ["pytest"]},
)
