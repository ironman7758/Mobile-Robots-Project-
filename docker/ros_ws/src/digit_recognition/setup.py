from setuptools import setup, find_packages

package_name = 'digit_recognition'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(include=[package_name, package_name + '.*']),
    package_data={
        # include the trained model file
        package_name: ['model/*.pth'],
    },
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
    zip_safe=True,
    maintainer='team2',
    maintainer_email='team2@example.com',
    description='MNIST digit recognition node for ROS2',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Inference node
            'digit_recognition_node = digit_recognition.digit_recognition_node:main',
            # Training script
            'train_mnist = digit_recognition.train_mnist:main',
        ],
    },
)
