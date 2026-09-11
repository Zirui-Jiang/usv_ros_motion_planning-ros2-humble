from setuptools import find_packages, setup

setup(name="common", version="0.1.0", packages=find_packages(exclude=["test"]),
      data_files=[("share/ament_index/resource_index/packages", ["resource/common"]),
                  ("share/common", ["package.xml"])],
      install_requires=["setuptools", "numpy"], zip_safe=True,
      maintainer="winter", maintainer_email="913982779@qq.com",
      description="Pure Python numerical contracts for USV navigation", license="TODO",
      tests_require=["pytest"])
