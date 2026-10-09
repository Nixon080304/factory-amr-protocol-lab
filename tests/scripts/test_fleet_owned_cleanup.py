# SPDX-License-Identifier: Apache-2.0
"""The fleet driver must clean descendants even after their leader exits."""

import importlib.util
import io
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
from types import SimpleNamespace

import pytest


ROOT = Path(__file__).resolve().parents[2]


def test_world_pose_evidence_retains_orientation_and_actual_receipt_time():
    pose = SimpleNamespace(
        position=SimpleNamespace(x=4.0, y=-2.0, z=0.0),
        orientation=SimpleNamespace(x=0.0, y=0.0, z=2**-0.5, w=2**-0.5),
    )
    value = driver().world_pose_evidence(pose, receipt_monotonic=123.25)
    assert value["x"] == 4.0 and value["y"] == -2.0 and value["z"] == 0.0
    assert value["quaternion"] == {"x": 0.0, "y": 0.0, "z": 2**-0.5, "w": 2**-0.5}
    assert value["yaw"] == pytest.approx(1.5707963267948966)
    assert value["receipt_monotonic"] == 123.25


def test_navigation_plan_retains_exact_endpoint_and_source_stamp_not_entire_path():
    module = driver()
    pose = SimpleNamespace(
        position=SimpleNamespace(x=2.3, y=-2.2, z=0.0),
        orientation=SimpleNamespace(x=0.0, y=0.0, z=2**-0.5, w=2**-0.5),
    )
    plan = SimpleNamespace(
        header=SimpleNamespace(
            frame_id="cart/map", stamp=SimpleNamespace(sec=142, nanosec=100000000)
        ),
        poses=[SimpleNamespace(pose=pose)] * 50,
    )
    row = module.navigation_plan_evidence("cart", plan, receipt_monotonic=8.5)
    assert row["robot_id"] == "cart" and row["frame_id"] == "cart/map"
    assert row["source_stamp"] == {"sec": 142, "nanosec": 100000000}
    assert row["pose_count"] == 50
    assert row["final_pose"]["x"] == 2.3 and row["final_pose"]["y"] == -2.2
    assert row["final_pose"]["yaw"] == pytest.approx(1.5707963267948966)
    assert "poses" not in row
    plan.poses = []
    assert (
        module.navigation_plan_evidence("cart", plan, receipt_monotonic=9)["final_pose"]
        is None
    )


@pytest.mark.parametrize("distance", [0.266, 0.30, 0.301])
def test_world_sample_retains_collision_evidence_before_enforcing_footprint(distance):
    module = driver()
    log, samples = io.StringIO(), []
    row = {
        "monotonic": 17.0,
        "robots": {
            "left": {"x": 0.0, "y": 0.0},
            "right": {"x": distance, "y": 0.0},
        },
    }
    if distance < 0.30:
        with pytest.raises(AssertionError, match="physical footprint overlap"):
            module.record_world_sample(log, samples, row, robot_radius=0.15)
    else:
        module.record_world_sample(log, samples, row, robot_radius=0.15)
    assert json.loads(log.getvalue()) == row
    assert samples == [row]


def test_charging_roles_follow_battery_eligibility_not_ids_or_robot_order():
    module = driver()
    config = SimpleNamespace(
        robots=[
            SimpleNamespace(robot_id="cart_z", battery_start_percent=31.0),
            SimpleNamespace(robot_id="cart_a", battery_start_percent=25.0),
        ],
        energy=SimpleNamespace(charge_below_percent=30.0),
    )
    assert module.charging_roles(config, {}) == ("cart_a", "cart_z")
    assert module.charging_roles(config, {"cart_z": 25.0, "cart_a": 31.0}) == (
        "cart_z",
        "cart_a",
    )


def test_domain_allocator_avoids_fixed_repository_domains_and_inherited_domain(
    monkeypatch,
):
    driver()
    import fleet_isolation

    monkeypatch.setenv("ROS_DOMAIN_ID", "20")
    monkeypatch.setattr(
        fleet_isolation.random.SystemRandom, "shuffle", lambda self, values: None
    )
    reservation = fleet_isolation.Domain()
    try:
        assert 20 <= reservation.id < 70
        assert reservation.id != 20
        assert reservation.id not in {92, 93, 96, 97, 117, 118}
        # RTPS reserves participant unicast offsets through 249 in its domain.
        # None may overlap this host's Linux ephemeral UDP range (32768–60999).
        assert 7400 + 250 * reservation.id + 249 < 32768
    finally:
        reservation.close()


def test_domain_allocator_refuses_host_ephemeral_range_overlapping_entire_pool(
    monkeypatch,
):
    driver()
    import fleet_isolation

    read = Path.read_text
    monkeypatch.setattr(
        Path,
        "read_text",
        lambda path, *args, **kwargs: (
            "10000 60999\n"
            if str(path) == "/proc/sys/net/ipv4/ip_local_port_range"
            else read(path, *args, **kwargs)
        ),
    )
    with pytest.raises(RuntimeError, match="no isolated fleet ROS domain available"):
        fleet_isolation.Domain()


@pytest.mark.parametrize(
    "content",
    [
        '{"domain_id":20,"quarantined":true}',
        '{"domain_id":21,"quarantined":false}',
        '{"domain_id":20,"quarantined":"false"}',
        "{}",
        "not-json",
    ],
)
def test_fleet_domain_cannot_bypass_v1_quarantine_or_unknown_metadata(
    tmp_path, monkeypatch, content
):
    driver()
    import fleet_isolation

    monkeypatch.setenv("ROS_DOMAIN_ID", "99")
    monkeypatch.setattr(
        fleet_isolation.random.SystemRandom, "shuffle", lambda self, values: None
    )
    original_open = Path.open

    def private_open(path, *args, **kwargs):
        if str(path).startswith("/tmp/factory-fleet-domain-"):
            path = tmp_path / path.name
        return original_open(path, *args, **kwargs)

    monkeypatch.setattr(Path, "open", private_open)
    (tmp_path / "factory-fleet-domain-20.lock").write_text(content)
    reservation = fleet_isolation.Domain()
    try:
        assert reservation.id == 21
        assert (tmp_path / "factory-fleet-domain-20.lock").read_text() == content
    finally:
        reservation.close()


@pytest.mark.parametrize("owner,expected", [("amr_01", False), (None, True)])
def test_charging_clearance_uses_energy_dock_identity(owner, expected):
    module = driver()
    from fleet_manager.config import load_fleet_config

    config = load_fleet_config(ROOT / "src/factory_bringup/config/fleet.yaml")
    proof = {"amr_01": dict(contact=False, reached_80=False, verified_clearance=False)}
    states = {"amr_01": SimpleNamespace(battery_percent=80.0, mode="AVAILABLE")}
    module.update_charging_proof(
        proof,
        states,
        {"amr_01": True},
        [],
        [{"resource_id": config.energy.dock_id, "owner": owner}],
        config.energy.dock_id,
    )
    assert proof["amr_01"] == dict(
        contact=True, reached_80=True, verified_clearance=expected
    )


@pytest.mark.parametrize(
    "subscribed,availability,expected",
    [
        (False, "online", False),
        (True, None, False),
        (True, "offline", False),
        (True, "online", True),
    ],
)
def test_mission_ingress_requires_suback_and_gateway_availability(
    subscribed, availability, expected
):
    module = driver()
    snapshot = {"fleet_state": "RUNNING", "robots": [{"health": "ONLINE"}]}
    evidence = dict(subscribed=subscribed, availability=availability)
    assert module.mission_ingress_ready(snapshot, evidence) is expected


@pytest.mark.parametrize(
    "cost,action,expected",
    [
        (True, True, True),
        (True, False, False),
        (False, True, False),
        (None, True, False),
        ("true", True, False),
    ],
)
def test_ingress_requires_exact_manager_view_for_every_configured_robot(
    cost, action, expected
):
    module = driver()
    rows = [
        {"robot_id": "cart", "cost_service_ready": True, "mission_action_ready": True},
        {
            "robot_id": "forklift",
            "cost_service_ready": cost,
            "mission_action_ready": action,
        },
    ]
    assert module.manager_dispatch_ready(rows, {"cart", "forklift"}) is expected
    assert module.manager_dispatch_ready(rows[:1], {"cart", "forklift"}) is False


def test_cost_evidence_retries_only_asynchronous_pending_response():
    module = driver()
    calls = []
    final = SimpleNamespace(reason="", feasible=True)
    responses = [SimpleNamespace(reason="path pending", feasible=False), final]

    def call(request):
        calls.append(request)
        response = responses.pop(0)
        return SimpleNamespace(done=lambda: True, result=lambda: response)

    assert (
        module.read_cost(
            SimpleNamespace(call_async=call),
            "request",
            lambda: None,
            lambda: None,
            time.monotonic() + 1,
        )
        is final
    )
    assert calls == ["request", "request"]


def test_cost_evidence_pending_never_exceeds_deadline():
    module = driver()
    response = SimpleNamespace(reason="path pending", feasible=False)
    service = SimpleNamespace(
        call_async=lambda request: SimpleNamespace(
            done=lambda: True, result=lambda: response
        )
    )
    with pytest.raises(
        AssertionError, match="production cost evidence deadline expired"
    ):
        module.read_cost(
            service, "request", lambda: None, lambda: None, time.monotonic() - 1
        )


@pytest.mark.parametrize(
    "feasible, reserve, available",
    [(False, 40, True), (True, 19, True), (True, 40, False)],
)
def test_nominal_cost_proof_excludes_ineligible_cheaper_robot(
    feasible, reserve, available
):
    module = driver()
    costs = {
        "cheaper": dict(
            feasible=feasible, path_cost=1.0, predicted_final_battery=reserve
        ),
        "eligible": dict(feasible=True, path_cost=2.0, predicted_final_battery=40),
    }
    candidates = {"eligible", "cheaper"} if available else {"eligible"}
    assert module.cost_winner(costs, candidates) == "eligible"


def test_nominal_cost_proof_breaks_equal_cost_by_generic_identity():
    module = driver()
    cost = dict(feasible=True, path_cost=2.0, predicted_final_battery=40)
    assert (
        module.cost_winner({"zebra": cost, "cart": cost}, {"zebra", "cart"}) == "cart"
    )


def test_successful_probe_without_required_receipt_returns_nonzero(
    tmp_path, monkeypatch
):
    module = driver()

    class NoEvidence:
        def __init__(self, output, deadline):
            self.deadline = deadline
            self.cleanup = []

        def start(self, *args):
            return SimpleNamespace(wait=lambda **kwargs: None, returncode=0)

        def close(self):
            self.cleanup = [dict(name="probe", group_clear=True, exit_code=0)]

    monkeypatch.setattr(module, "Owned", NoEvidence)
    monkeypatch.setattr(module.signal, "signal", lambda *args: None)
    monkeypatch.setattr(
        module.sys,
        "argv",
        [
            "fleet_driver.py",
            "acceptance",
            "--scenario",
            "scale",
            "--output",
            str(tmp_path),
        ],
    )
    assert module.main() == 1
    summary = module.json.loads(next(tmp_path.glob("*/summary.json")).read_text())
    assert summary["success"] is False
    assert summary["error"] == "missing scale receipt"


def test_interrupted_broker_creation_resolves_only_its_owned_exact_id(
    tmp_path, monkeypatch
):
    module = driver()
    owned = module.Owned(tmp_path, time.monotonic() + 20)
    calls = []
    exact_id = "a" * 64

    def docker(command, **kwargs):
        calls.append(command)
        if command[1] == "run":
            raise subprocess.TimeoutExpired(command, 1)
        if command[1] == "inspect":
            return subprocess.CompletedProcess(
                command,
                0,
                module.json.dumps(
                    [
                        dict(
                            Id=exact_id,
                            Config=dict(
                                Labels={"factory.fleet.owner": owned.broker_owner}
                            ),
                        )
                    ]
                ),
                "",
            )
        return subprocess.CompletedProcess(command, 0, "", "")

    monkeypatch.setattr(module.subprocess, "run", docker)
    with pytest.raises(subprocess.TimeoutExpired):
        owned.create_broker(1883)
    owned.close()
    assert calls[-1] == ["docker", "rm", "-f", exact_id]
    assert owned.cleanup[-1]["container"] == exact_id


def test_broker_cleanup_refuses_mismatched_ownership(tmp_path, monkeypatch):
    module = driver()
    owned = module.Owned(tmp_path, time.monotonic() + 20)
    calls = []

    def docker(command, **kwargs):
        calls.append(command)
        if command[1] == "run":
            raise subprocess.TimeoutExpired(command, 1)
        return subprocess.CompletedProcess(
            command,
            0,
            module.json.dumps([dict(Id="b" * 64, Config=dict(Labels={}))]),
            "",
        )

    monkeypatch.setattr(module.subprocess, "run", docker)
    with pytest.raises(subprocess.TimeoutExpired):
        owned.create_broker(1883)
    with pytest.raises(RuntimeError, match="broker ownership mismatch"):
        owned.close()
    assert not any(command[1] == "rm" for command in calls)


def test_invalid_broker_output_never_becomes_a_cleanup_target(tmp_path, monkeypatch):
    module = driver()
    owned = module.Owned(tmp_path, time.monotonic() + 20)
    monkeypatch.setattr(
        module.subprocess,
        "run",
        lambda command, **kwargs: subprocess.CompletedProcess(
            command, 0, "unrelated-existing-name", ""
        ),
    )
    with pytest.raises(AssertionError, match="missing exact owned broker container ID"):
        owned.create_broker(1883)
    assert owned.container is None


def driver():
    sys.path.insert(0, str(ROOT / "tests/system"))
    spec = importlib.util.spec_from_file_location(
        "fleet_cleanup_contract", ROOT / "tests/system/fleet_driver.py"
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_failed_broker_removal_rejects_success_and_preserves_cleanup_receipt(
    tmp_path, monkeypatch
):
    module = driver()
    owned = module.Owned(tmp_path, time.monotonic() + 20)
    owned.container = "a" * 64
    monkeypatch.setattr(
        module.subprocess,
        "run",
        lambda command, **kwargs: subprocess.CompletedProcess(
            command, 1 if command[1] == "rm" else 0, "", "removal refused"
        ),
    )
    with pytest.raises(RuntimeError, match="owned cleanup failed"):
        owned.close()
    rows = json.loads((tmp_path / "cleanup.json").read_text())
    assert rows[-1]["container"] == "a" * 64
    assert rows[-1]["exit_code"] == 1
    assert rows[-1]["output"] == "removal refused"


@pytest.mark.parametrize("failure", ["logs_timeout", "log_write", "remove_timeout"])
def test_broker_diagnostic_failure_still_removes_exact_owner_and_keeps_receipts(
    tmp_path, monkeypatch, failure
):
    module = driver()
    owned = module.Owned(tmp_path, time.monotonic() + 20)
    owned.container = "d" * 64
    plc = owned.start("plc", [sys.executable, "-c", "pass"], os.environ)
    plc.wait(timeout=5)
    commands = []

    def docker(command, **kwargs):
        commands.append(command)
        if (failure == "logs_timeout" and command[1] == "logs") or (
            failure == "remove_timeout" and command[1] == "rm"
        ):
            raise subprocess.TimeoutExpired(command, kwargs["timeout"])
        return subprocess.CompletedProcess(command, 0, "removed", "")

    monkeypatch.setattr(module.subprocess, "run", docker)
    if failure == "log_write":
        (tmp_path / "broker-runtime.log").mkdir()
    primary = RuntimeError("primary scenario failure")
    try:
        raise primary
    except RuntimeError:
        with pytest.raises(RuntimeError, match="primary scenario failure") as caught:
            owned.close()
    assert caught.value is primary and caught.value.__cause__ is not None
    assert commands[-1] == ["docker", "rm", "-f", "d" * 64]
    rows = json.loads((tmp_path / "cleanup.json").read_text())
    assert rows[0]["name"] == "plc" and rows[0]["group_clear"] is True
    removal = next(row for row in rows if "container" in row)
    assert removal["container"] == "d" * 64
    assert removal["exit_code"] == (None if failure == "remove_timeout" else 0)
    assert any(row.get("error") for row in rows)


def test_child_diagnostic_failure_does_not_skip_other_owned_cleanup(
    tmp_path, monkeypatch
):
    module = driver()
    owned = module.Owned(tmp_path, time.monotonic() + 20)
    plc = owned.start("plc", [sys.executable, "-c", "pass"], os.environ)
    launch = owned.start("launch", [sys.executable, "-c", "pass"], os.environ)
    for process in (plc, launch):
        process.wait(timeout=5)
    monkeypatch.setattr(
        module,
        "launch_child_receipt",
        lambda *args: (_ for _ in ()).throw(OSError("log unavailable")),
    )
    owned.container = "e" * 64
    commands = []

    def docker(command, **kwargs):
        commands.append(command)
        return subprocess.CompletedProcess(command, 0, "", "")

    monkeypatch.setattr(module.subprocess, "run", docker)
    with pytest.raises(RuntimeError, match="owned cleanup failed"):
        owned.close()
    assert commands[-1] == ["docker", "rm", "-f", "e" * 64]
    rows = json.loads((tmp_path / "cleanup.json").read_text())
    assert any(row.get("name") == "plc" and row["group_clear"] for row in rows)
    assert any(row.get("name") == "launch" and row.get("error") for row in rows)


def test_unexpected_plc_exit_rejects_success_after_cleaning_other_resources(
    tmp_path, monkeypatch
):
    module = driver()
    owned = module.Owned(tmp_path, time.monotonic() + 20)
    plc = owned.start("plc", [sys.executable, "-c", "raise SystemExit(7)"], os.environ)
    plc.wait(timeout=5)
    owned.container = "b" * 64
    monkeypatch.setattr(
        module.subprocess,
        "run",
        lambda command, **kwargs: subprocess.CompletedProcess(command, 0, "", ""),
    )
    with pytest.raises(RuntimeError, match="owned cleanup failed"):
        owned.close()
    rows = json.loads((tmp_path / "cleanup.json").read_text())
    assert rows[0]["name"] == "plc" and rows[0]["exit_code"] == 7
    assert rows[0]["group_clear"] is True
    assert rows[-1]["container"] == "b" * 64 and rows[-1]["exit_code"] == 0


def test_successful_owned_plc_and_broker_cleanup_is_fully_recorded(
    tmp_path, monkeypatch
):
    module = driver()
    owned = module.Owned(tmp_path, time.monotonic() + 20)
    plc = owned.start("plc", [sys.executable, "-c", "pass"], os.environ)
    plc.wait(timeout=5)
    owned.container = "c" * 64
    monkeypatch.setattr(
        module.subprocess,
        "run",
        lambda command, **kwargs: subprocess.CompletedProcess(command, 0, "", ""),
    )
    owned.close()
    rows = json.loads((tmp_path / "cleanup.json").read_text())
    assert rows[0]["name"] == "plc" and rows[0]["exit_code"] == 0
    assert rows[0]["group_clear"] is True
    assert rows[-1]["container"] == "c" * 64 and rows[-1]["exit_code"] == 0


def test_broker_absence_requires_recorded_exact_not_found_proof(tmp_path, monkeypatch):
    module = driver()
    owned = module.Owned(tmp_path, time.monotonic() + 20)
    owned.broker_name = "factory-fleet-owned-absence"
    error = "Error: No such object: factory-fleet-owned-absence\n"
    monkeypatch.setattr(
        module.subprocess,
        "run",
        lambda command, **kwargs: subprocess.CompletedProcess(
            command, 1, "[]\n", error
        ),
    )
    owned.close()
    assert json.loads((tmp_path / "cleanup.json").read_text()) == [
        {
            "container_name": "factory-fleet-owned-absence",
            "absent": True,
            "inspect_exit_code": 1,
            "output": "[]\n" + error,
        }
    ]


@pytest.mark.parametrize(
    "code,error",
    [
        (1, "Cannot connect to the Docker daemon; transport unavailable\n"),
        (1, "error during connect: connection reset\n"),
        (1, "Error: No such object: unrelated-existing-container\n"),
        (2, "Error: No such object: factory-fleet-owned-absence\n"),
    ],
)
def test_inspect_errors_never_claim_owned_broker_absent(
    tmp_path, monkeypatch, code, error
):
    module = driver()
    owned = module.Owned(tmp_path, time.monotonic() + 20)
    owned.broker_name = "factory-fleet-owned-absence"
    commands = []

    def docker(command, **kwargs):
        commands.append(command)
        return subprocess.CompletedProcess(command, code, "[]\n", error)

    monkeypatch.setattr(module.subprocess, "run", docker)
    with pytest.raises(RuntimeError, match="owned cleanup failed"):
        owned.close()
    rows = json.loads((tmp_path / "cleanup.json").read_text())
    assert rows[0]["absent"] is False
    assert rows[0]["inspect_exit_code"] == code
    assert rows[0]["output"] == "[]\n" + error
    assert commands == [["docker", "inspect", "factory-fleet-owned-absence"]]


def test_uncertain_broker_cleanup_retains_primary_startup_error_and_full_receipt(
    tmp_path, monkeypatch
):
    module = driver()
    owned = module.Owned(tmp_path, time.monotonic() + 20)
    plc = owned.start("plc", [sys.executable, "-c", "pass"], os.environ)
    plc.wait(timeout=5)

    def docker(command, **kwargs):
        if command[1] == "run":
            raise subprocess.TimeoutExpired(command, 1)
        assert command == ["docker", "inspect", owned.broker_name]
        return subprocess.CompletedProcess(command, 1, "", "Docker daemon unavailable")

    monkeypatch.setattr(module.subprocess, "run", docker)
    with pytest.raises(subprocess.TimeoutExpired) as captured:
        try:
            owned.create_broker(1883)
        finally:
            owned.close()
    assert captured.value.cmd[:3] == ["docker", "run", "-d"]
    assert captured.value.timeout == 1
    assert isinstance(captured.value.__cause__, RuntimeError)
    rows = json.loads((tmp_path / "cleanup.json").read_text())
    assert rows[0]["name"] == "plc" and rows[0]["exit_code"] == 0
    assert rows[0]["group_clear"] is True
    assert rows[1]["container_name"] == owned.broker_name
    assert rows[1]["absent"] is False and rows[1]["inspect_exit_code"] == 1
    assert rows[1]["output"] == "Docker daemon unavailable"


def test_cleanup_reaps_owned_group_when_parent_already_exited(tmp_path):
    module = driver()
    owned = module.Owned(tmp_path, time.monotonic() + 20)
    pid_file = tmp_path / "child.pid"
    program = (
        "import os,signal,time,pathlib; pid=os.fork(); "
        "pathlib.Path(__import__('sys').argv[1]).write_text(str(pid)) "
        "if pid else None; "
        "os._exit(0) if pid else None; "
        "signal.signal(signal.SIGINT,signal.SIG_IGN); time.sleep(30)"
    )
    parent = owned.start(
        "probe", [sys.executable, "-c", program, str(pid_file)], os.environ
    )
    parent.wait(timeout=5)
    child = int(pid_file.read_text())
    try:
        owned.close()
        status = Path(f"/proc/{child}/stat")
        assert (
            not status.exists() or status.read_text().split(") ")[1].split()[0] == "Z"
        )
        assert owned.cleanup[0]["group_clear"] is True
    finally:
        try:
            os.kill(child, signal.SIGKILL)
        except ProcessLookupError:
            pass


@pytest.mark.parametrize(
    "case", ["direct_handoff", "quarantine_overlap", "identity_reuse"]
)
def test_exact_lease_audit_rejects_unproven_authority_handoff(case):
    module = driver()
    lease = dict(resource_id="aisle", robot_id="first", mission_id="one", lease_id="L1")
    other = dict(
        resource_id="aisle", robot_id="second", mission_id="two", lease_id="L2"
    )

    def row(sequence, current=None, former=()):
        return dict(
            sequence=sequence,
            resource_id="aisle",
            timestamp=float(sequence),
            evidence=dict(lease=current, former_leases=list(former)),
        )

    if case == "direct_handoff":
        rows = [row(1, lease), row(2, other)]
    elif case == "quarantine_overlap":
        rows = [row(1, None, (lease,)), row(2, other, (lease,))]
    else:
        other["lease_id"] = "L1"
        rows = [row(1, lease), row(2), row(3, other)]
    with pytest.raises(AssertionError):
        module.audit_leases(rows)


def test_exact_lease_audit_records_cleared_distinct_handoffs():
    module = driver()
    rows = [
        dict(
            sequence=sequence,
            timestamp=float(sequence),
            resource_id="aisle",
            evidence=dict(
                lease=None
                if robot is None
                else dict(
                    resource_id="aisle",
                    robot_id=robot,
                    mission_id="mission",
                    lease_id=robot,
                ),
                former_leases=[],
            ),
        )
        for sequence, robot in enumerate(("first", None, "second", None), 1)
    ]
    assert module.audit_leases(rows)["aisle"] == ["first", "second"]


@pytest.mark.parametrize("old_x,accepted", [(-2.0, True), (0.0, False)])
def test_atomic_handoff_requires_exact_clearance_and_fresh_physical_exit(
    old_x, accepted
):
    module = driver()
    rows = [
        dict(
            sequence=i,
            timestamp=float(i),
            resource_id="aisle",
            evidence=dict(
                lease=dict(
                    resource_id="aisle",
                    robot_id=robot,
                    mission_id="work",
                    lease_id=robot,
                ),
                former_leases=[],
            ),
        )
        for i, robot in ((1, "first"), (2, "second"))
    ]
    arguments = dict(
        clearances=[("aisle", "first", "work", "first")],
        samples=[dict(monotonic=1.8, robots={"first": dict(x=old_x, y=0.0)})],
        bounds={"aisle": [-1, -1, 1, 1]},
        physical_radius=0.15,
    )
    if accepted:
        assert module.audit_leases(rows, **arguments)["aisle"] == ["first", "second"]
    else:
        with pytest.raises(AssertionError):
            module.audit_leases(rows, **arguments)


def test_visible_world_preserves_geometry_and_uses_real_payload_entities(tmp_path):
    import xml.etree.ElementTree as ET
    from fleet_manager.config import load_fleet_config

    module = driver()
    config = load_fleet_config(ROOT / "src/factory_bringup/config/fleet.yaml")
    _, path = module.prepare(config, tmp_path, {})
    world = ET.parse(path).getroot().find("world")
    assert world.find("gui/camera/pose") is not None
    assert {item.findtext("name") for item in world.findall("include")} >= {
        robot.robot_id + "_part" for robot in config.robots
    }
    obstacle = world.find("model[@name='dynamic_obstacle']")
    original = (
        ET.parse(ROOT / "src/factory_simulation/worlds/factory_floor.world")
        .getroot()
        .find("world/model[@name='dynamic_obstacle']")
    )
    assert obstacle.findtext("pose") == "5 3 0.25 0 0 0"
    assert ET.tostring(obstacle.find("link")) == ET.tostring(original.find("link"))
    assert world.find("model[@name='charging_dock']/link/visual") is not None


def test_configured_fleet_layout_has_free_map_footprints_and_obstacle_clearance(
    tmp_path,
):
    from fleet_manager.config import load_fleet_config
    from PIL import Image
    import math
    import xml.etree.ElementTree as ET
    import yaml

    config = load_fleet_config(ROOT / "src/factory_bringup/config/fleet.yaml")
    maps = ROOT / "src/factory_bringup/maps"
    metadata = yaml.safe_load((maps / "factory_map.yaml").read_text())
    raster = Image.open(maps / metadata["image"])
    resolution = metadata["resolution"]
    origin_x, origin_y, _ = metadata["origin"]
    poses = [robot.spawn for robot in config.robots]
    poses.extend(
        pose for bays in config.station_exit_poses.values() for pose in bays.values()
    )
    poses.extend(config.station_staging.values())
    for dock in config.docks.values():
        poses.extend([dock.staging_pose, dock.charging_pose, *dock.exit_poses.values()])
    radius = max(d.robot_radius + d.arrival_tolerance for d in config.docks.values())
    for pose in poses:
        for dx in range(
            -math.ceil(radius / resolution), math.ceil(radius / resolution) + 1
        ):
            for dy in range(
                -math.ceil(radius / resolution), math.ceil(radius / resolution) + 1
            ):
                if math.hypot(dx * resolution, dy * resolution) > radius:
                    continue
                column = math.floor((pose.x - origin_x) / resolution) + dx
                row = (
                    raster.height
                    - 1
                    - math.floor((pose.y - origin_y) / resolution)
                    - dy
                )
                assert 0 <= column < raster.width and 0 <= row < raster.height
                assert raster.getpixel((column, row)) >= 254, (
                    f"occupied footprint at {pose}"
                )
    _, generated = driver().prepare(config, tmp_path, {})
    world = ET.parse(generated).getroot().find("world")
    obstacle = world.find("model[@name='dynamic_obstacle']")
    x, y, *_ = map(float, obstacle.findtext("pose").split())
    width, depth, _ = map(
        float, obstacle.findtext("link/collision/geometry/box/size").split()
    )
    for pose in poses:
        distance = math.hypot(
            max(abs(pose.x - x) - width / 2, 0), max(abs(pose.y - y) - depth / 2, 0)
        )
        assert distance > radius, f"relocated obstacle blocks {pose}"


def test_launch_shutdown_does_not_signal_children_twice(tmp_path):
    module = driver()
    owned = module.Owned(tmp_path, time.monotonic() + 20)
    result = tmp_path / "signals"
    program = """
import os, signal, sys, time
path = sys.argv[1]
child = os.fork()
if child == 0:
    count = 0
    def stop(*_):
        global count
        count += 1
        open(path, 'w').write(str(count))
        if count > 1:
            os._exit(1)
        time.sleep(0.3)
        os._exit(0)
    signal.signal(signal.SIGINT, stop)
else:
    def stop(*_):
        os.kill(child, signal.SIGINT)
        os.waitpid(child, 0)
        os._exit(0)
    signal.signal(signal.SIGINT, stop)
while True:
    time.sleep(0.1)
"""
    parent = owned.start(
        "launch", [sys.executable, "-c", program, str(result)], os.environ
    )
    time.sleep(0.3)
    try:
        owned.close()
        assert result.read_text() == "1"
        assert parent.returncode == 0
    finally:
        if owned.group_alive(parent.pid):
            os.killpg(parent.pid, signal.SIGKILL)
