from setuptools import setup

package_name = "weed_calibration_tools"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools", "numpy"],
    zip_safe=True,
    maintainer="todo",
    maintainer_email="todo@example.com",
    description="Offline calibration utilities for Fast-LIVO2 and RTK trajectory alignment.",
    license="TODO",
    entry_points={
        "console_scripts": [
            "export_odom_tum = weed_calibration_tools.export_odom_tum:main",
            "align_trajectories = weed_calibration_tools.align_trajectories:main",
            "calib_axxb = weed_calibration_tools.calib_axxb:main",
            "odom_to_path = weed_calibration_tools.odom_to_path:main",
        ],
    },
)
