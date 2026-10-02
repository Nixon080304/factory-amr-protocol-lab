# SPDX-License-Identifier: Apache-2.0
from glob import glob
from setuptools import find_packages, setup

setup(
    name="station_perception",
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/station_perception"]),
        ("share/station_perception", ["package.xml"]),
        ("share/station_perception/markers", glob("markers/*.png")),
        ("share/station_perception/tools", ["tools/generate_markers.py"]),
    ],
    install_requires=["setuptools"],
    extras_require={"test": ["pytest"]},
    zip_safe=False,
    maintainer="Nixon Edward Winata",
    maintainer_email="nixonedwardwinata2004@gmail.com",
    description="ArUco station identity detection and pure confirmation window.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": ["station_detector = station_perception.node:main"]
    },
)
