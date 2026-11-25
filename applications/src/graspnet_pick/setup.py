from glob import glob
import os

from setuptools import setup

package_name = "graspnet_pick"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="xu",
    maintainer_email="xu@todo.todo",
    description="RGB-D grasp pose estimation and service-triggered grasp execution.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "grasp_estimator = graspnet_pick.estimator_node:main",
            "grasp_executor = graspnet_pick.executor_node:main",
        ],
    },
)
