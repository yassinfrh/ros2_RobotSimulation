from setuptools import find_packages, setup

package_name = 'test_node'

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
    maintainer='yassin',
    maintainer_email='yassin@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'test_node = test_node.test_node:main',
            'orientation_node = test_node.orientation:main',
            'webcam_node = test_node.webcam:main',
            'spawn_boxes = test_node.spawn_boxes:main',
        ],
    },
)
