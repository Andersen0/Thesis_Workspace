from setuptools import setup, find_packages
import os
import glob

package_name = 'yolov6'
core = 'yolov6/core'
utils = 'yolov6/utils'
data = 'yolov6/data'
layers = 'yolov6/layers'
models = 'yolov6/models'
models_heads = 'yolov6/models/heads'
models_losses = 'yolov6/models/losses'
solver = 'yolov6/solver'
assigners = 'yolov6/assigners'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, core, utils, data, layers, models, models_heads, models_losses, solver, assigners],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            'inferer = yolov6.core.inferer:main',
            'depth_cam = yolov6.core.depth_cam:main'
        ],
    },
)
