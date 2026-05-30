from setuptools import setup
from glob import glob

package_name = 'n10p_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/params', glob('params/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='N10P Developer',
    maintainer_email='user@example.com',
    description='N10P LiDAR + 匿名凌霄飞控 启动配置',
    license='MIT',
    entry_points={
        'console_scripts': [
            'ano_bridge_node = n10p_bringup.ano_bridge_node:main',
            'dummy_odom_node = n10p_bringup.dummy_odom_node:main',
            'keyboard_odom_node = n10p_bringup.keyboard_odom_node:main',
        ],
    },
)
