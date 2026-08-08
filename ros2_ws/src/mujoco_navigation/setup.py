import os
from glob import glob

from setuptools import find_packages
from setuptools import setup


package_name = "mujoco_navigation"


setup(
    name=package_name,
    version="0.0.1",

    packages=find_packages(
        exclude=[
            "test",
        ]
    ),

    data_files=[

        # ROS2 package index
        (
            "share/ament_index/resource_index/packages",
            [
                "resource/" + package_name,
            ],
        ),

        # package.xml
        (
            "share/" + package_name,
            [
                "package.xml",
            ],
        ),

        # launch 文件
        (
            os.path.join(
                "share",
                package_name,
                "launch",
            ),
            glob(
                "launch/*.launch.py"
            ),
        ),

    ],

    install_requires=[
        "setuptools",
    ],

    zip_safe=True,

    maintainer="yangerhao",

    maintainer_email="your_email@example.com",

    description=(
        "ROS2 nodes for MuJoCo simulation, "
        "A* path planning and PD controller."
    ),

    license="MIT",

    tests_require=[
        "pytest",
    ],

    entry_points={
        "console_scripts": [

            (
                "simulation_node = "
                "mujoco_navigation.simulation_node:main"
            ),

            (
                "navigation_node = "
                "mujoco_navigation.navigation_node:main"
            ),

        ],
    },
)