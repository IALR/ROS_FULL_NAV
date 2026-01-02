from setuptools import setup
from glob import glob
from setuptools import find_packages

package_name = 'robotic_chair_nav'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install all launch files
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/maps', glob('maps/*')),
        # Install all YAML configs (nav2_params.yaml, amcl_params.yaml, etc.)
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='Obstacle stop behavior for the robotic chair.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'move_forward = robotic_chair_nav.move_forward:main',
            'keyboard_control = robotic_chair_nav.keyboard_control:main',
            'obstacle_stop_node = robotic_chair_nav.obstacle_stop_node:main',
        ],
    },
)