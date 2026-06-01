import os
from glob import glob
from setuptools import setup

package_name = "rrt_planner"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"),
         glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "rviz"),
         glob("rviz/*.rviz")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="you",
    maintainer_email="you@example.com",
    description="Custom RRT / RRT* path planner (Section 3).",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "rrt_planner = rrt_planner.rrt_planner_node:main",
        ],
    },
)
