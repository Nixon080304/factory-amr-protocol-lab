"""Controlled robot transport for DDS-free orchestration tests."""

from dataclasses import dataclass

from fleet_manager.models import CostEstimate


@dataclass
class Goal:
    robot_id: str
    request: object
    feedback: object
    result: object
    accepted: object
    cancel_ack: object = None


class FakeRobots:
    def __init__(self, journal):
        self.journal = journal
        self.costs = {
            "amr_01": CostEstimate(True, 9, 50),
            "amr_02": CostEstimate(True, 2, 50),
        }
        self.cost_calls = []
        self.waiting_costs = []
        self.goals = []
        self.statuses = []
        self.auto_accept = True

    def estimate(self, robot_id, request, callback):
        self.cost_calls.append((robot_id, request.mission_id))
        estimate = self.costs.get(robot_id)
        if estimate == "timeout":
            self.waiting_costs.append(callback)
        else:
            callback(estimate)

    def send_goal(self, robot_id, request, feedback, result, accepted):
        # Assert the actual durable boundary, not a mocked journal write.
        record = self.journal.get(request.mission_id)
        assert record.state == "ASSIGNED"
        assert record.assigned_robot_id == robot_id
        goal = Goal(robot_id, request, feedback, result, accepted)
        self.goals.append(goal)
        if self.auto_accept:
            accepted(goal, "")

    def cancel(self, handle, callback):
        handle.cancel_ack = callback

    def publish(self, record, state, detail, progress):
        assert self.journal.get(record.request.mission_id) == record
        self.statuses.append((record, state, detail, progress))


def create_ros_robot(context, robot_id, namespace, cost=1.0, *, cost_available=True):
    """Real ROS fake agent; requires unrestricted DDS networking."""
    import rclpy
    from factory_interfaces.action import ExecuteFactoryMission
    from factory_interfaces.msg import RobotState
    from factory_interfaces.srv import EstimateMissionCost
    from rclpy.action import ActionServer, CancelResponse

    node = rclpy.create_node(f"fake_{robot_id}", namespace=namespace, context=context)
    node.goals = []
    node.cost_requests = []
    node.finish = True
    node.failed = False
    node.failure_code = "ROBOT_OFFLINE"
    node.stage = "NAVIGATING_TO_DROPOFF"
    node.active = {}
    publisher = node.create_publisher(RobotState, "factory/robot_state", 10)

    def heartbeat():
        message = RobotState(
            robot_id=robot_id,
            mode="EXECUTING" if node.active else "AVAILABLE",
            frame_id=namespace.strip("/") + "/map",
            battery_percent=80.0,
            payload_state="EMPTY",
        )
        if node.active:
            handle = next(iter(node.active.values()))
            message.mission_id = handle.request.mission_id
        for handle in tuple(node.active.values()):
            if node.finish or handle.is_cancel_requested:
                del node.active[id(handle)]
                handle.execute()
        message.pose.orientation.w = 1.0
        publisher.publish(message)

    node.heartbeat_timer = node.create_timer(0.1, heartbeat)

    def estimate(request, response):
        node.cost_requests.append(request)
        response.feasible = True
        response.path_cost = cost
        response.predicted_final_battery = 50.0
        return response

    if cost_available:
        node.create_service(
            EstimateMissionCost, "factory/estimate_mission_cost", estimate
        )

    def accepted(handle):
        node.goals.append(handle.request)
        node.active[id(handle)] = handle
        handle.publish_feedback(
            ExecuteFactoryMission.Feedback(
                state=node.stage,
                station="inspection",
                detail="robot feedback",
            )
        )

    def execute(handle):
        success = not handle.is_cancel_requested and not node.failed
        if success:
            handle.succeed()
        elif handle.is_cancel_requested:
            handle.canceled()
        else:
            handle.abort()
        return ExecuteFactoryMission.Result(
            success=success,
            final_state="COMPLETED" if success else "FAILED",
            error_code=""
            if success
            else node.failure_code
            if node.failed
            else "CANCELLED",
        )

    node.server = ActionServer(
        node,
        ExecuteFactoryMission,
        "factory/execute_mission",
        execute,
        cancel_callback=lambda handle: CancelResponse.ACCEPT,
        handle_accepted_callback=accepted,
    )
    return node
