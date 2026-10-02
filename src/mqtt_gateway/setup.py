# SPDX-License-Identifier: Apache-2.0
from setuptools import find_packages, setup


setup(
    name="mqtt_gateway",
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    package_data={"mqtt_gateway": ["schemas/*.json"]},
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/mqtt_gateway"]),
        ("share/mqtt_gateway", ["package.xml"]),
    ],
    install_requires=["setuptools", "jsonschema==4.26.0"],
    extras_require={"test": ["pytest"]},
    zip_safe=False,
    maintainer="Nixon Edward Winata",
    maintainer_email="nixonedwardwinata2004@gmail.com",
    description="Validated MQTT mission identity and reconnect buffering.",
    license="Apache-2.0",
)
