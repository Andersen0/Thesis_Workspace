from setuptools import find_packages, setup

package_name = 'node_manager'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Tage Andersen',
    maintainer_email='tage.andersen@nmbu.no',
    description='Package containing a ROS2 node manager',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'node_manager = node_manager.node_manager:main',
        ],
    },
)