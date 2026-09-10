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
    description="LiDAR integer-evidence occupancy mapping for the USV simulation",
    license="TODO",
    extras_require={"test": ["pytest"]},
    scripts=["scripts/tf_process/usv_tf_expander.py"],
    entry_points={"console_scripts": [
        "perception_mapping_node = perception.mapping_node:main",
    ]},
)
