from setuptools import setup
from glob import glob
import os

package_name = 'robotic_chair_nav'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Add launch files
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.py'))),
        # Add config files
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
        # Add map files (both yaml and image files)
        (os.path.join('share', package_name, 'maps'), glob(os.path.join('maps', '*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Your Name',
    author_email='your.email@example.com',
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Navigation and control package for robotic chair',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'navigation_commander=robotic_chair_nav.navigation_commander:main',
        ],
    },
)