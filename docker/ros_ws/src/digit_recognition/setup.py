from setuptools import setup, find_packages
import os

package_name = 'digit_recognition'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(include=[package_name, package_name + '.*']),
    package_data={ package_name: ['model/*.pth'] },
    install_requires=[
        'setuptools',
        'torch',
        'torchvision',
        'opencv-python',
        'numpy',
        'cv_bridge',
        'rclpy',
        'sensor_msgs',
        'std_msgs',
    ],
    entry_points={
        'console_scripts': [
            'digit_recognition_node = digit_recognition.digit_recognition_node:main',
            'color_detection_node   = digit_recognition.color_detection_node:main',
            'train_mnist            = digit_recognition.train_mnist:main',
        ],
    },
)
