from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ros2_imitation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yassin',
    maintainer_email='yassin.frh@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'object_detection = ros2_imitation.object_detection:main',
            'spawn_boxes_service = ros2_imitation.spawn_boxes_service:main',
            'pick_place_action_server = ros2_imitation.pick_place_action_server:main',
            'environment = ros2_imitation.environment:main',
            'simplified_env = ros2_imitation.simplified_env:main',
        ],
    },
)
