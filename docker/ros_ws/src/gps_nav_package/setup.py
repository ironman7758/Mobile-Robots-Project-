from setuptools import setup

package_name = 'gps_nav'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'pyproj'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='GPS waypoint navigation with Nav2',
    license='MIT',
    entry_points={
        'console_scripts': [
            'gps_waypoint_nav = gps_nav.gps_waypoint_nav:main'
        ],
    },
)
