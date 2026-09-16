# 安装 usv_navigation：打包共享导航参数、ROS 输入与会话协议支持，并注册 ament 资源索引和包元数据。
from pathlib import Path
from setuptools import setup

# 递归安装四类 YAML 并保留配置子目录，启动器据此定位实际算法配置。
config_files = [(str(Path('share/usv_navigation') / path.parent), [str(path)])
                for path in sorted(Path('config').rglob('*.yaml'))]

setup(
    name='usv_navigation', version='0.1.0', packages=['usv_navigation'],
    data_files=[('share/ament_index/resource_index/packages', ['resource/usv_navigation']),
                ('share/usv_navigation', ['package.xml']), *config_files],
    install_requires=['setuptools', 'numpy', 'PyYAML'], zip_safe=True,
    maintainer='Winter', maintainer_email='913982779@qq.com', license='BSD',
    description='Shared navigation configuration, input adapters and transport support',
)
