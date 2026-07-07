from setuptools import setup
from glob import glob
import os

package_name = 'weedarm_h5u_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'pymodbus'],
    zip_safe=True,
    maintainer='HU CU',
    maintainer_email='1619675251@qq.com',
    description='ROS2 Modbus TCP bridge for H5U weed arm CSP Y/Z trajectory buffering.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'h5u_csp_bridge = weedarm_h5u_bridge.h5u_csp_bridge:main',
            'sync_traj_publisher = weedarm_h5u_bridge.sync_traj_publisher:main',
        ],
    },
)
