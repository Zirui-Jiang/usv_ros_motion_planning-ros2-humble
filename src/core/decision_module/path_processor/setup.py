# 安装 path_processor：打包RDP 路径简化与 SG 几何平滑，并注册 ament 资源索引和包元数据。
from setuptools import find_packages, setup

package_name = "path_processor"
setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=("test", "tests")),
    data_files=[("share/ament_index/resource_index/packages", ["resource/" + package_name]),
                ("share/" + package_name, ["package.xml"])],
    install_requires=["setuptools", "numpy"],
    zip_safe=True,
    maintainer="winter",
    maintainer_email="913982779@qq.com",
    description="Pure Python RDP and Savitzky-Golay path processing",
    license="GPL-3.0-only",
    extras_require={"test": ["pytest"]},
)
