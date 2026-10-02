# SPDX-License-Identifier: Apache-2.0
from setuptools import find_packages, setup


setup(
    name="protocol_observer",
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/protocol_observer"]),
        ("share/protocol_observer", ["package.xml", "README.md"]),
    ],
    install_requires=["setuptools"],
    extras_require={"test": ["pytest"]},
    zip_safe=False,
    maintainer="Nixon Edward Winata",
    maintainer_email="nixonedwardwinata2004@gmail.com",
    description="Append-only protocol traces and deterministic mission reports.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": ["protocol_observer = protocol_observer.node:main"]
    },
)
