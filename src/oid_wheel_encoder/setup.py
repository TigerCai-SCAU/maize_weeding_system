from glob import glob
from setuptools import setup


package_name = "oid_wheel_encoder"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="HU CU",
    maintainer_email="1619675251@qq.com",
    description="SocketCAN OID measuring-wheel encoder driver.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "wheel_encoder_node = oid_wheel_encoder.wheel_encoder_node:main",
        ],
    },
)
