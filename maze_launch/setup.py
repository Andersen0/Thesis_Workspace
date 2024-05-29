from setuptools import find_packages, setup
from glob import glob

package_name = 'maze_launch'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/worlds', glob('worlds/*')),  # Include all files in "worlds" folder
        ('share/' + package_name + '/launch', glob('launch/*')),  # Include all files in "launch" folder
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nbmurobotics',
    maintainer_email='tor.erik.aasestad@nmbu.no',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)

