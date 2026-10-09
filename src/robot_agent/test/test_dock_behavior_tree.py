"""Check the actual Nav2 goal chooses precision only for charger entry."""

from types import SimpleNamespace

import pytest
from rclpy.task import Future

from test_docking import rig as dock_rig, start
from test_node import Host, api


def transport_rig(tmp_path, *, configured=True):
    r = dock_rig()
    _, _, wire, *_ = r
    tree = tmp_path / "dock_to_pose.xml"
    tree.write_text(
        '<root main_tree_to_execute="Dock"><BehaviorTree ID="Dock"/></root>'
    )
    goals, terminal, cancellations = [], [], []

    def send(goal):
        goals.append(goal)
        accepted, result = Future(), Future()
        terminal.append(result)
        accepted.set_result(
            SimpleNamespace(
                accepted=True,
                get_result_async=lambda: result,
                cancel_goal_async=lambda: (
                    cancellations.append(len(goals) - 1),
                    Future(),
                )[1],
            )
        )
        return accepted

    transport = api().DockTransport(
        Host("/cart_1"),
        SimpleNamespace(server_is_ready=lambda: True, send_goal_async=send),
        {},
        behavior_tree=str(tree) if configured else "",
    )
    wire.navigate = transport.navigate
    return r, tree, goals, terminal, cancellations


def arrive(r, terminals, pose):
    agent, _, _, _, _, controller = r
    stamp = agent._pose_stamp + 1
    terminals[-1].set_result(SimpleNamespace(status=4))
    agent.odometry(stamp, 0, 0, agent.frame_prefix + "odom")
    agent.localization(stamp, *pose, agent.frame_prefix + "map")
    controller.tick()


@pytest.mark.parametrize("phase", ["STAGING", "ENTERING", "EXITING", "CLEARING"])
@pytest.mark.parametrize("configured", [True, False])
def test_nav2_goal_behavior_tree_is_precise_only_for_configured_entry(
    tmp_path, phase, configured
):
    r, tree, goals, terminals, _ = transport_rig(tmp_path, configured=configured)
    _, _, wire, _, _, controller = r
    start(r)
    if phase != "STAGING":
        arrive(r, terminals, controller.staging)
        if phase == "CLEARING":
            wire.reply(granted=False, lease_id="", lease_ttl_sec=0.0)
            controller.cancel()
            wire.reply(lease_id="", reconciliation_required=False)
        else:
            wire.reply(granted=True, lease_id="exact-dock-lease", lease_ttl_sec=10.0)
            if phase == "EXITING":
                arrive(r, terminals, controller.charging)
                controller.contact(True)
                controller.cancel()
    assert controller.state == phase
    expected = str(tree) if configured and phase == "ENTERING" else ""
    assert goals[-1].behavior_tree == expected
    assert goals[-1].pose.header.frame_id == "floor/cart_1/map"
    assert [goal.behavior_tree for goal in goals] == (
        ["", str(tree), ""]
        if configured and phase == "EXITING"
        else ["", str(tree)]
        if configured and phase == "ENTERING"
        else [""] * len(goals)
    )


@pytest.mark.parametrize(
    "value", [None, 1, True, "relative.xml", "   ", "missing_absolute", "directory"]
)
def test_invalid_configured_behavior_tree_fails_closed_before_navigation(
    tmp_path, value
):
    if value == "missing_absolute":
        value = str(tmp_path / "missing.xml")
    elif value == "directory":
        value = str(tmp_path)
    with pytest.raises(ValueError, match="behavior tree"):
        api().DockTransport(Host("/cart_1"), None, {}, behavior_tree=value)


def test_missing_configured_tree_at_entry_preserves_dock_recovery(tmp_path):
    r, tree, goals, terminals, _ = transport_rig(tmp_path)
    agent, _, wire, outcomes, _, controller = r
    start(r)
    arrive(r, terminals, controller.staging)
    tree.unlink()
    wire.reply(granted=True, lease_id="exact-dock-lease", lease_ttl_sec=10.0)
    assert len(goals) == 1
    assert not controller.active and agent.mode == "RECOVERY_REQUIRED"
    assert outcomes[0].error_code == "NAVIGATION_FAILED"
    assert not any(operation == "release" for operation, *_ in wire.requests)


def test_precise_entry_cancel_keeps_terminal_stop_and_default_exit(tmp_path):
    r, tree, goals, terminals, cancellations = transport_rig(tmp_path)
    _, _, wire, outcomes, _, controller = r
    start(r)
    arrive(r, terminals, controller.staging)
    wire.reply(granted=True, lease_id="exact-dock-lease", lease_ttl_sec=10.0)
    assert goals[-1].behavior_tree == str(tree)
    controller.cancel()
    assert cancellations == [1] and len(goals) == 2 and not outcomes
    terminals[-1].set_result(SimpleNamespace(status=5))
    assert controller.state == "EXITING" and goals[-1].behavior_tree == ""
    arrive(r, terminals, controller.exit_pose)
    wire.reply(released=True)
    assert outcomes[0].error_code == "CANCELLED"


def test_tree_removed_after_entry_does_not_block_default_exit(tmp_path):
    r, tree, goals, terminals, _ = transport_rig(tmp_path)
    _, _, wire, outcomes, _, controller = r
    start(r)
    arrive(r, terminals, controller.staging)
    wire.reply(granted=True, lease_id="exact-dock-lease", lease_ttl_sec=10.0)
    arrive(r, terminals, controller.charging)
    controller.contact(True)
    tree.unlink()
    controller.cancel()
    assert controller.state == "EXITING" and goals[-1].behavior_tree == ""
    arrive(r, terminals, controller.exit_pose)
    wire.reply(released=True)
    assert outcomes[0].error_code == "CANCELLED"
