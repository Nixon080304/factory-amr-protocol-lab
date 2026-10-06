# SPDX-License-Identifier: Apache-2.0
from setuptools import find_packages, setup

setup(
    name="robot_agent",
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/robot_agent"]),
        ("share/robot_agent", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    extras_require={"test": ["pytest"]},
    zip_safe=False,
    maintainer="Factory AMR Protocol Lab maintainers",
    maintainer_email="maintainers@example.com",
    description="Robot-local health, simulated energy, and bounded Nav2 mission estimates.",
    license="Apache-2.0",
    entry_points={"console_scripts": ["robot_agent = robot_agent.node:main"]},
)
