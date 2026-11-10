from glob import glob
from setuptools import setup

package_name = "rik_ros"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.py")),
        ("share/" + package_name + "/configs", glob("../../../configs/partners/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Vansh Gala",
    description="ROS 2 base driver for the robot integration kit",
    license="MIT",
    tests_require=["pytest"],
    entry_points={"console_scripts": ["base_driver = rik_ros.base_driver_node:main"]},
)
