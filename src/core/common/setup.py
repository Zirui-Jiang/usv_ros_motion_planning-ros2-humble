from setuptools import find_packages, setup

setup(name="common", version="0.1.0", packages=find_packages(exclude=["test"]),
      data_files=[("share/ament_index/resource_index/packages", ["resource/common"]),
                  ("share/common", ["package.xml"])],
      install_requires=["setuptools", "numpy"], zip_safe=True,
      maintainer="Zirui", maintainer_email="1029302265@qq.com",
      description="Pure Python numerical contracts for USV navigation", license="GPL-3.0-only",
      tests_require=["pytest"])
