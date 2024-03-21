from setuptools import setup
import os
from glob import glob

package_name = 'srobot_realistic_output'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.py')),  # Include launch files
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Tage Andersen',
    maintainer_email='tage.andersen@nmbu.no',
    description='realistic output for srobot',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'publisher = srobot_realistic_output.publisher:main',
            'subscriber = srobot_realistic_output.subscriber:main',
        ],
    },
)
