# Factory AMR Protocol Lab

A ROS 2 lab for factory material transfer, with MQTT at the dispatcher boundary
and Modbus TCP at the station boundary.

## Milestone status

The protocol core is under development. The shared ROS interface package is
available: `ExecuteFactoryMission`, `StationDetection`, `ProtocolEvent`, and
`TransferPart`. Mission execution, protocol adapters, and simulation follow in
later milestones.

## Development setup

Target Ubuntu 22.04, ROS 2 Humble, and Python 3.10. Install ROS development tools
and create the local environment with access to the system ROS Python packages:

```bash
python3 -m venv --system-site-packages .venv
source .venv/bin/activate
python -m pip install -r requirements.txt -r requirements-dev.txt
source /opt/ros/humble/setup.bash
python -m pytest -q tests/contracts/test_interface_contracts.py
colcon build --packages-select factory_interfaces
source install/setup.bash
ros2 interface show factory_interfaces/action/ExecuteFactoryMission
ros2 interface show factory_interfaces/srv/TransferPart
```

Licensed under Apache-2.0. See [LICENSE](LICENSE).
