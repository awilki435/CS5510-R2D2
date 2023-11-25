import os
from glob import glob
from setuptools import find_packages, setup

package_name = "ball_moving_robot"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "rviz"), glob("rviz/*.rviz")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="derek",
    maintainer_email="derekdredmond@gmail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "transforms = ball_moving_robot.transforms:main",
            "localizer = ball_moving_robot.localizer:main",
            "planner = ball_moving_robot.planner:main",
            "driver = ball_moving_robot.driver:main",
        ],
    },
)
