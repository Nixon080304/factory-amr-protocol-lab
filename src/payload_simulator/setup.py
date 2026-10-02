# SPDX-License-Identifier: Apache-2.0
from setuptools import find_packages, setup

setup(
    name="payload_simulator",
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/payload_simulator"]),
        ("share/payload_simulator", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    extras_require={"test": ["pytest"]},
    zip_safe=False,
    maintainer="Nixon Edward Winata",
    maintainer_email="nixonedwardwinata2004@gmail.com",
    description="Deduplicated logical payload state and bounded Gazebo conveyor visuals.",
    license="Apache-2.0",
    entry_points={"console_scripts": ["payload_simulator = payload_simulator.node:main"]},
)
