from setuptools import find_packages, setup
import os
import glob
package_name = 'pyflask'
templates = 'pyflask/templates'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, templates],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (f'share/{package_name}/templates', ['pyflask/templates/index.html']),
        (f'share/{package_name}/static', ['pyflask/static/main.js']),
        (os.path.join('share', package_name, 'launch'), glob.glob(os.path.join('launch', '*.launch.py')))
    ],
    package_data={
    'pyflask': ['launch/*.launch.py'],},
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='eliash',
    maintainer_email='elias.hartmark@gmail.com',
    description='Runs a flask server and a ros2 node in the same process',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'server = pyflask.server:main',
            'talker = pyflask.publisher:main',
            'robot = pyflask.robot:main',
            'cycle_dist = pyflask.cycle_dist:main'
        ],
    },
)
