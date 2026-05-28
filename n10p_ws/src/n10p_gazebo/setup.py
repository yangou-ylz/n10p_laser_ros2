from setuptools import setup
from glob import glob

package_name = 'n10p_gazebo'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/urdf', glob('urdf/*.urdf')),
        ('share/' + package_name + '/worlds', glob('worlds/*.world')),
        ('share/' + package_name + '/config', glob('config/*.yaml') + glob('config/*.rviz')),
    ],
    install_requires=['setuptools'],
    entry_points={
        'console_scripts': [
            'scan_relay = n10p_gazebo.scan_relay:main',
        ],
    },
    zip_safe=True,
    maintainer='N10P Developer',
    maintainer_email='user@example.com',
    description='N10P 无人机 Gazebo 仿真启动配置',
    license='MIT',
)
