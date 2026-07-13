from setuptools import setup
from glob import glob

package_name = 'n10p_fusion'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='N10P Developer',
    maintainer_email='user@example.com',
    description='N10P EKF odometry fusion — robot_localization wrapper',
    license='MIT',
    entry_points={
        'console_scripts': [
            'imu_filter_node = n10p_fusion.imu_filter_node:main',
        ],
    },
)
