from glob import glob
import os

from setuptools import find_packages, setup


package_name = "seedling_path_planning"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=("test",)),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools", "numpy"],
    zip_safe=True,
    maintainer="HU CU",
    maintainer_email="1619675251@qq.com",
    description=(
        "Robust seedling-row analysis and terrain-aware safe coverage planning."
    ),
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            (
                "spatial_path_planner = "
                "seedling_path_planning.spatial_path_planner_node:main"
            ),
        ],
    },
)
