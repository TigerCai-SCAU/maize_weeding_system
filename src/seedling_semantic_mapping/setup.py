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
    maintainer='nvidia',
    maintainer_email='nvidia@todo.todo',
    description='Seedling SEP 3D localization and landmark mapping from FAST-LIVO2 synchronized outputs.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yolo_sep_localizer = seedling_semantic_mapping.yolo_sep_localizer:main',
            'seedling_mapper = seedling_semantic_mapping.seedling_mapper:main',
        ],
    },
)
