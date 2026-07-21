from glob import glob
import os

from setuptools import setup


package_name = "ground_mapper"


setup(
    name=package_name,
    version="0.4.0",
    packages=[package_name],
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml", "README.md"]),
        (
            os.path.join("share", package_name, "config"),
            glob("config/*.yaml"),
        ),
        (
            os.path.join("share", package_name, "launch"),
            glob("launch/*.launch.py"),
        ),
    ],
    install_requires=["setuptools", "numpy"],
    zip_safe=True,
    maintainer="HU CU",
    maintainer_email="1619675251@qq.com",
    description="Adaptive rolling ground map for the maize weeding robot.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "ground_mapper_node = ground_mapper.ground_mapper_node:main",
        ],
    },
)
