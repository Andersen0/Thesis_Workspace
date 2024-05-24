from setuptools import setup
import os
from glob import glob


package_name = 'imrt_teleop'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
 
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='henrik',
    maintainer_email='henrik.nordlie@nmbu.no',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'imrt_teleop = imrt_teleop.imrt_teleop:main',
            'teleop_triggers = imrt_teleop.teleop_triggers:main'
        ],
    },
)
