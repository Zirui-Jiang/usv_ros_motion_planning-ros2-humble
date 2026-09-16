# 安装 controller：打包Otter PID、LQR 与定点保持纯 Python 控制算法，并注册 ament 资源索引和包元数据。
from setuptools import find_packages, setup

setup(
    name="controller", version="0.1.0", packages=find_packages(),
    data_files=[("share/ament_index/resource_index/packages", ["resource/controller"]),
                ("share/controller", ["package.xml"])],
    install_requires=["setuptools", "numpy"], zip_safe=True,
    maintainer="winter", maintainer_email="913982779@qq.com",
    description="Pure Python Otter PID and iterative discrete LQR controllers", license="GPL3",
)
