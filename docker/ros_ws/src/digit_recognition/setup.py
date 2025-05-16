from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'digit_recognition'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=[package_name, package_name + '.*']),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='team2',
    maintainer_email='team2@example.com',
    description='MNIST digit recognition node for ROS2',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'digit_recognition_node = digit_recognition.digit_recognition_node:main',
        ],
    },
    data_files=[
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'model'),
         glob('model/*.pth')),
    ],
)
