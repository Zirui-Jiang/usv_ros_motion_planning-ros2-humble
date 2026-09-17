from pathlib import Path
from setuptools import setup

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
