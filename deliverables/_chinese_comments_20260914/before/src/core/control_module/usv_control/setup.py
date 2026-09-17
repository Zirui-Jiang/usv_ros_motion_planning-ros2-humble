from glob import glob
from setuptools import setup

setup(
    name='usv_control', version='0.1.0', packages=['usv_control'],
    data_files=[('share/ament_index/resource_index/packages', ['resource/usv_control']),
                ('share/usv_control', ['package.xml']),
                ('share/usv_control/launch', glob('launch/*.launch.py')),
                ('share/usv_control/config', glob('config/*'))],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='Winter', maintainer_email='913982779@qq.com', license='BSD',
    description='Ordinary navigation control and sole propulsion output adapter',
    entry_points={'console_scripts': ['control_node = usv_control.control:main']},
)
