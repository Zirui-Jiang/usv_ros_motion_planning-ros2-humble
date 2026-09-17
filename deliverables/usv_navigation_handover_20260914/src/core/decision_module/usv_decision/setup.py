from glob import glob
from setuptools import setup

setup(
    name='usv_decision', version='0.1.0', packages=['usv_decision'],
    data_files=[('share/ament_index/resource_index/packages', ['resource/usv_decision']),
                ('share/usv_decision', ['package.xml']),
                ('share/usv_decision/launch', glob('launch/*.launch.py')),
                ('share/usv_decision/config', glob('config/*'))],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='Winter', maintainer_email='913982779@qq.com', license='BSD',
    description='Ordinary navigation task ownership and path/trajectory planning',
    entry_points={'console_scripts': ['decision_node = usv_decision.decision:main']},
)
