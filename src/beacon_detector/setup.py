from setuptools import find_packages, setup, SetuptoolsDeprecationWarning
import warnings
from glob import glob
import os

warnings.filterwarnings("ignore", category=SetuptoolsDeprecationWarning)

package_name = 'beacon_detector'

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "scripts"),
            glob("beacon_detector_node/scripts/*.*"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="little",
    maintainer_email="delgorguealexis@gmail.com",
    description="This package contains the file to detect balises and find the position of the robot thanks to the signal coming from the lidar",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "beacon_detector_node = beacon_detector.beacon_detector_node:main",
            "beacon_generator_node = beacon_detector.beacon_generator_node:main",
            "onboard_robot_detector_node = beacon_detector.onboard_robot_detector_node:main"
        ],
    },
)
