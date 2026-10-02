# SPDX-License-Identifier: Apache-2.0
from setuptools import find_packages, setup

setup(
    name='fault_injector', version='0.1.0', packages=find_packages(exclude=['test']),
    data_files=[('share/ament_index/resource_index/packages', ['resource/fault_injector']),
                ('share/fault_injector', ['package.xml'])],
    install_requires=['setuptools'], extras_require={'test': ['pytest']}, zip_safe=False,
    maintainer='Nixon Edward Winata', maintainer_email='nixonedwardwinata2004@gmail.com',
    description='Deterministic protocol fault injection.', license='Apache-2.0',
    entry_points={'console_scripts': ['fault_injector = fault_injector.node:main']},
)
