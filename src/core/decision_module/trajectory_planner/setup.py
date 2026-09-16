# 安装 trajectory_planner：打包LOS 轨迹生成、圆航线与八字段插值，并注册 ament 资源索引和包元数据。
from setuptools import find_packages, setup

setup(
    name="trajectory_planner", version="0.1.0", packages=find_packages(),
    data_files=[("share/ament_index/resource_index/packages", ["resource/trajectory_planner"]),
                ("share/trajectory_planner", ["package.xml"])],
    install_requires=["setuptools", "numpy"], zip_safe=True,
    maintainer="Winter", maintainer_email="913982779@qq.com",
    description="ROS-independent LOS trajectory generation", license="TODO",
)
