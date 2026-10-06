# SPDX-License-Identifier: Apache-2.0
"""Fleet manager entry point: one executor thread owns the durable journal."""

import rclpy
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor

from fleet_manager.node import FleetManagerNode


def main(args=None):
    rclpy.init(args=args)
    node = None
    executor = SingleThreadedExecutor()
    try:
        node = FleetManagerNode()
        executor.add_node(node)
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
