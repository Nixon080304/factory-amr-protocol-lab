# SPDX-License-Identifier: Apache-2.0
from setuptools import find_packages, setup

setup(
    name="modbus_gateway",
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/modbus_gateway"]),
        ("share/modbus_gateway", ["package.xml"]),
    ],
    install_requires=["setuptools", "pymodbus==3.15.0"],
    extras_require={"test": ["pytest"]},
    zip_safe=False,
    maintainer="Nixon Edward Winata",
    maintainer_email="nixonedwardwinata2004@gmail.com",
    description="Bounded Modbus transfer handshakes for factory stations.",
    license="Apache-2.0",
    entry_points={"console_scripts": ["modbus_gateway = modbus_gateway.node:main"]},
)
