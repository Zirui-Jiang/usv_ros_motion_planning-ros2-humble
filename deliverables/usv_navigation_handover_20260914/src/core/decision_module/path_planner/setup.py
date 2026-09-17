from setuptools import find_packages, setup

package_name = "path_planner"
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
    description="ROS-independent Python grid planning algorithms",
    license="GPL-3.0-only",
    extras_require={"test": ["pytest"]},
)
