from setuptools import setup
from glob import glob
import os

package_name = 'weedarm_row_planner_test'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='HU CU',
    maintainer_email='1619675251@qq.com',
    description='Row seedling avoidance planner test node for weedarm Y/Z trajectory.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'row_seedling_planner_test = weedarm_row_planner_test.row_seedling_planner_test:main',
        ],
    },
)
