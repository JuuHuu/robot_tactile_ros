from setuptools import setup
import os
from glob import glob

package_name = 'gelsight_publisher'

setup(
    name=package_name,
    version='0.1.0',
    packages=[
        package_name,
        package_name + '.utilities',
    ],
    py_modules=[
        package_name + '.image_publisher',
        package_name + '.config',
    ],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your@email.com',
    description='ROS 2 node for GelSight tactile image publishing',
    license='MIT',
    entry_points={
        'console_scripts': [
            'gelsight_image_publisher = gelsight_publisher.image_publisher:main',
            'depth_reconstruction_node = gelsight_publisher.depth_reconstruction:main',
        ],
    },
)
