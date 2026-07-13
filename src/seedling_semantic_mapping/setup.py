from setuptools import setup
from glob import glob
import os

package_name = 'seedling_semantic_mapping'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'docs'), glob('docs/*.md')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='HU CU',
    maintainer_email='1619675251@qq.com',
    description='Seedling SEP 3D localization and landmark mapping using external image/cloud/odom synchronization.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yolo_sep_localizer = seedling_semantic_mapping.yolo_sep_localizer:main',
            'color_sep_localizer = seedling_semantic_mapping.color_sep_localizer:main',
            'seedling_mapper = seedling_semantic_mapping.seedling_mapper:main',
            'odom_tf_broadcaster = seedling_semantic_mapping.odom_tf_broadcaster:main',
            'rolling_submap_builder = seedling_semantic_mapping.rolling_submap_builder:main',
        ],
    },
)
