# SPDX-License-Identifier: Apache-2.0
from setuptools import find_packages, setup

setup(
    name="fleet_manager",
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/fleet_manager"]),
        ("share/fleet_manager", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    extras_require={"test": ["pytest"]},
    zip_safe=False,
    maintainer="Nixon Edward Winata",
    maintainer_email="nixonedwardwinata2004@gmail.com",
    description="Validated fleet configuration and global coordination.",
    license="Apache-2.0",
    entry_points={"console_scripts": ["fleet_manager = fleet_manager.main:main"]},
)
