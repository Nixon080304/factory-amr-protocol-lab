# SPDX-License-Identifier: Apache-2.0
"""Load the fleet scaling boundary into immutable, validated models."""

from dataclasses import dataclass, field
import math
from pathlib import Path
import re
from types import MappingProxyType
from typing import Mapping

import yaml


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class RobotConfig:
    robot_id: str
    namespace: str
    frame_prefix: str
    spawn: Pose2D
    battery_start_percent: float


@dataclass(frozen=True)
class ResourceConfig:
    resource_id: str
    kind: str
    capacity: int


@dataclass(frozen=True)
class DockConfig:
    staging_pose: Pose2D
    charging_pose: Pose2D
    robot_radius: float = 0.15
    arrival_tolerance: float = 0.15
    exit_poses: Mapping[str, Pose2D] = field(
        default_factory=lambda: MappingProxyType({})
    )
    departure_stations: tuple[str, ...] = ()


@dataclass(frozen=True)
class EnergyPolicyConfig:
    reserve_percent: float
    charge_below_percent: float
    charge_until_percent: float
    dock_id: str
    idle_percent_per_sec: float = 0.001
    move_percent_per_m: float = 0.1
    operation_percent: float = 0.5
    charge_percent_per_sec: float = 1.0
    dock_allowance_m: float = 10.0


@dataclass(frozen=True)
class RouteSegment:
    resource_id: str
    staging_pose: Pose2D
    exit_pose: Pose2D


@dataclass(frozen=True)
class FleetConfig:
    robots: tuple[RobotConfig, ...]
    resources: tuple[ResourceConfig, ...]
    routes: Mapping[str, tuple[str, ...]]
    docks: Mapping[str, DockConfig]
    energy: EnergyPolicyConfig
    route_segments: Mapping[str, tuple[RouteSegment, ...]] = field(
        default_factory=lambda: MappingProxyType({})
    )
    traffic_bounds: Mapping[str, tuple[float, float, float, float]] = field(
        default_factory=lambda: MappingProxyType({})
    )
    resource_bounds: Mapping[str, tuple[float, float, float, float]] = field(
        default_factory=lambda: MappingProxyType({})
    )
    station_staging: Mapping[str, Pose2D] = field(
        default_factory=lambda: MappingProxyType({})
    )
    station_approach: Mapping[str, Pose2D] = field(
        default_factory=lambda: MappingProxyType({})
    )
    station_exit_poses: Mapping[str, Mapping[str, Pose2D]] = field(
        default_factory=lambda: MappingProxyType({})
    )


class _FleetLoader(yaml.SafeLoader):
    """Reject duplicate YAML keys rather than silently discarding declarations."""


def _yaml_mapping(loader, node):
    result = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node)
        if not isinstance(key, str):
            raise ValueError(
                f"YAML line {key_node.start_mark.line + 1}: keys must be strings"
            )
        if key in result:
            raise ValueError(
                f"YAML line {key_node.start_mark.line + 1}: duplicate key {key!r}"
            )
        result[key] = loader.construct_object(value_node)
    return result


_FleetLoader.add_constructor(
    yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, _yaml_mapping
)


def _mapping(value, path, fields=None):
    if not isinstance(value, dict):
        raise ValueError(f"{path}: expected a mapping")
    if fields is not None:
        for field in fields:
            if field not in value:
                raise ValueError(f"{path}.{field}: required field missing")
        for field in value:
            if field not in fields:
                raise ValueError(f"{path}.{field}: unknown field")
    return value


def _sequence(value, path):
    if not isinstance(value, list) or not value:
        raise ValueError(f"{path}: expected a nonempty list")
    return value


def _identifier(value, path):
    if (
        not isinstance(value, str)
        or re.fullmatch(r"[A-Za-z_][A-Za-z0-9_]*", value) is None
    ):
        raise ValueError(
            f"{path}: expected a nonempty identifier using letters, digits, and underscores"
        )
    return value


def _number(value, path):
    if type(value) not in (int, float):
        raise ValueError(f"{path}: expected a finite number")
    try:
        number = float(value)
    except OverflowError as exc:
        raise ValueError(f"{path}: expected a finite number") from exc
    if not math.isfinite(number):
        raise ValueError(f"{path}: expected a finite number")
    return number


def _percent(value, path):
    number = _number(value, path)
    if not 0 <= number <= 100:
        raise ValueError(f"{path}: expected a percentage between 0 and 100")
    return number


def _pose(value, path):
    if not isinstance(value, list) or len(value) != 3:
        raise ValueError(f"{path}: expected [x, y, yaw]")
    return Pose2D(
        *(
            _number(component, f"{path}[{index}]")
            for index, component in enumerate(value)
        )
    )


def _unique(value, seen, path):
    if value in seen:
        raise ValueError(f"{path}: duplicate value {value!r}")
    seen.add(value)


def _segment_distance(start, end, point):
    dx, dy = end.x - start.x, end.y - start.y
    length_squared = dx * dx + dy * dy
    if length_squared == 0:
        return math.hypot(point.x - start.x, point.y - start.y)
    fraction = max(
        0.0,
        min(
            1.0, ((point.x - start.x) * dx + (point.y - start.y) * dy) / length_squared
        ),
    )
    return math.hypot(
        point.x - start.x - fraction * dx, point.y - start.y - fraction * dy
    )


def _segment_bounds_distance(start, end, bounds):
    """Exact distance to a closed rectangle, including parallel/point segments."""
    left, bottom, right, top = bounds
    lower, upper = 0.0, 1.0
    for origin, delta, first, last in (
        (start.x, end.x - start.x, left, right),
        (start.y, end.y - start.y, bottom, top),
    ):
        if delta == 0:
            if not first <= origin <= last:
                lower, upper = 1.0, 0.0
                break
        else:
            a, b = sorted(((first - origin) / delta, (last - origin) / delta))
            lower, upper = max(lower, a), min(upper, b)
    if lower <= upper:
        return 0.0
    return min(
        *(
            math.hypot(max(left - p.x, 0, p.x - right), max(bottom - p.y, 0, p.y - top))
            for p in (start, end)
        ),
        *(
            _segment_distance(start, end, Pose2D(x, y, 0))
            for x, y in ((left, bottom), (left, top), (right, bottom), (right, top))
        ),
    )


def _parse_config(data):
    data = _mapping(data, "fleet")
    geometry = data.get("route_segments", {})
    zone_geometry = data.get("traffic_bounds", {})
    resource_geometry = data.get("resource_bounds", {})
    station_geometry = data.get("station_staging", {})
    approach_geometry = data.get("station_approach", {})
    exit_geometry = data.get("station_exit_poses", {})
    data = {
        key: value
        for key, value in data.items()
        if key
        not in (
            "route_segments",
            "traffic_bounds",
            "resource_bounds",
            "station_staging",
            "station_approach",
            "station_exit_poses",
        )
    }
    data = _mapping(data, "fleet", ("robots", "resources", "routes", "docks", "energy"))
    robots = []
    seen = {
        field: set() for field in ("robot_id", "namespace", "frame_prefix", "spawn")
    }
    for index, value in enumerate(_sequence(data["robots"], "robots")):
        path = f"robots[{index}]"
        value = _mapping(
            value,
            path,
            ("robot_id", "namespace", "frame_prefix", "spawn", "battery_start_percent"),
        )
        robot_id = _identifier(value["robot_id"], f"{path}.robot_id")
        namespace = value["namespace"]
        if (
            not isinstance(namespace, str)
            or re.fullmatch(r"(?:/[A-Za-z_][A-Za-z0-9_]*)+", namespace) is None
        ):
            raise ValueError(f"{path}.namespace: expected an absolute ROS namespace")
        prefix = value["frame_prefix"]
        if (
            not isinstance(prefix, str)
            or re.fullmatch(r"(?:[A-Za-z_][A-Za-z0-9_]*/)+", prefix) is None
        ):
            raise ValueError(
                f"{path}.frame_prefix: expected a relative frame prefix ending in /"
            )
        spawn = _pose(value["spawn"], f"{path}.spawn")
        for identity_field, identity in (
            ("robot_id", robot_id),
            ("namespace", namespace),
            ("frame_prefix", prefix),
            ("spawn", (spawn.x, spawn.y)),
        ):
            _unique(identity, seen[identity_field], f"{path}.{identity_field}")
        robots.append(
            RobotConfig(
                robot_id,
                namespace,
                prefix,
                spawn,
                _percent(
                    value["battery_start_percent"], f"{path}.battery_start_percent"
                ),
            )
        )

    resources = []
    resource_ids = set()
    for index, value in enumerate(_sequence(data["resources"], "resources")):
        path = f"resources[{index}]"
        value = _mapping(value, path, ("resource_id", "kind", "capacity"))
        resource_id = _identifier(value["resource_id"], f"{path}.resource_id")
        _unique(resource_id, resource_ids, f"{path}.resource_id")
        if value["kind"] not in ("station", "traffic_zone", "dock"):
            raise ValueError(f"{path}.kind: expected station, traffic_zone, or dock")
        if type(value["capacity"]) is not int or value["capacity"] != 1:
            raise ValueError(f"{path}.capacity: only integer capacity 1 is supported")
        resources.append(ResourceConfig(resource_id, value["kind"], value["capacity"]))

    routes = {}
    for name, route in _mapping(data["routes"], "routes").items():
        _identifier(name, f"routes.{name}")
        route = _sequence(route, f"routes.{name}")
        for index, resource in enumerate(route):
            path = f"routes.{name}[{index}]"
            _identifier(resource, path)
            if resource not in resource_ids:
                raise ValueError(f"{path}: unknown resource {resource!r}")
        routes[name] = tuple(route)
        traffic_ids = {r.resource_id for r in resources if r.kind == "traffic_zone"}
        if sum(resource in traffic_ids for resource in route) > 1:
            raise ValueError(
                f"routes.{name}: only one traffic segment is supported until protected handoff bays exist"
            )

    route_segments = {}
    traffic = {r.resource_id for r in resources if r.kind == "traffic_zone"}
    traffic_bounds = {}
    for resource, values in _mapping(zone_geometry, "traffic_bounds").items():
        path = f"traffic_bounds.{resource}"
        if resource not in traffic:
            raise ValueError(f"{path}: expected a configured traffic zone")
        if not isinstance(values, list) or len(values) != 4:
            raise ValueError(f"{path}: expected [xmin, ymin, xmax, ymax]")
        bounds = tuple(_number(v, f"{path}[{i}]") for i, v in enumerate(values))
        if bounds[0] >= bounds[2] or bounds[1] >= bounds[3]:
            raise ValueError(f"{path}: bounds must have positive area")
        traffic_bounds[resource] = bounds
    for name, values in _mapping(geometry, "route_segments").items():
        path = f"route_segments.{name}"
        if name not in routes:
            raise ValueError(f"{path}: unknown route")
        if not isinstance(values, list):
            raise ValueError(f"{path}: expected a list")
        segments = []
        for index, value in enumerate(values):
            item_path = f"{path}[{index}]"
            value = _mapping(
                value, item_path, ("resource_id", "staging_pose", "exit_pose")
            )
            resource = _identifier(value["resource_id"], f"{item_path}.resource_id")
            if resource not in traffic:
                raise ValueError(f"{item_path}.resource_id: expected a traffic zone")
            staging = _pose(value["staging_pose"], f"{item_path}.staging_pose")
            exit_pose = _pose(value["exit_pose"], f"{item_path}.exit_pose")
            if resource not in traffic_bounds:
                raise ValueError(f"{item_path}: requires traffic bounds")
            xmin, ymin, xmax, ymax = traffic_bounds[resource]
            for pose in (staging, exit_pose):
                # Planned clearance covers both 0.15 m footprints and both
                # 0.15 m arrival errors; actual physical overlap is separate.
                if not (
                    pose.x + 0.60 < xmin
                    or pose.y + 0.60 < ymin
                    or pose.x - 0.60 > xmax
                    or pose.y - 0.60 > ymax
                ):
                    raise ValueError(
                        f"{item_path}: staging and exit must be outside traffic bounds"
                    )
            if math.hypot(staging.x - exit_pose.x, staging.y - exit_pose.y) < 0.5:
                raise ValueError(f"{item_path}: staging and exit must be distinct")
            segments.append(RouteSegment(resource, staging, exit_pose))
        if tuple(s.resource_id for s in segments) != tuple(
            r for r in routes[name] if r in traffic
        ):
            raise ValueError(f"{path}: must cover traffic zones in route order")
        route_segments[name] = tuple(segments)

    resource_bounds = {}
    for resource, value in _mapping(resource_geometry, "resource_bounds").items():
        path = f"resource_bounds.{resource}"
        if resource not in resource_ids:
            raise ValueError(f"{path}: unknown resource")
        if not isinstance(value, list) or len(value) != 4:
            raise ValueError(f"{path}: expected [xmin, ymin, xmax, ymax]")
        bounds = tuple(_number(number, path) for number in value)
        if bounds[0] >= bounds[2] or bounds[1] >= bounds[3]:
            raise ValueError(f"{path}: bounds must have positive area")
        resource_bounds[resource] = bounds

    docks = {}
    dock_ids = {
        resource.resource_id for resource in resources if resource.kind == "dock"
    }
    for dock_id, value in _mapping(data["docks"], "docks").items():
        path = f"docks.{dock_id}"
        _identifier(dock_id, path)
        if dock_id not in resource_ids:
            raise ValueError(f"{path}: unknown resource {dock_id!r}")
        if dock_id not in dock_ids:
            raise ValueError(f"{path}: resource must have kind dock")
        value = _mapping(value, path)
        departures = value.get("departure_stations", [])
        if (
            not isinstance(departures, list)
            or any(not isinstance(item, str) for item in departures)
            or len(set(departures)) != len(departures)
            or any(
                station
                not in {
                    resource.resource_id
                    for resource in resources
                    if resource.kind == "station"
                }
                for station in departures
            )
        ):
            raise ValueError(
                f"{path}.departure_stations: requires distinct configured stations"
            )
        exits = value.get("exit_poses")
        exit_poses = {}
        if exits is not None:
            exits = _mapping(exits, path + ".exit_poses")
            if set(exits) != {robot.robot_id for robot in robots}:
                raise ValueError(
                    f"{path}.exit_poses: requires every configured robot exactly once"
                )
            exit_poses = {
                robot_id: _pose(pose, f"{path}.exit_poses.{robot_id}")
                for robot_id, pose in exits.items()
            }
        value = _mapping(
            {
                "robot_radius": 0.15,
                "arrival_tolerance": 0.15,
                **{
                    key: item
                    for key, item in value.items()
                    if key not in {"exit_poses", "departure_stations"}
                },
            },
            path,
            ("staging_pose", "charging_pose", "robot_radius", "arrival_tolerance"),
        )
        bounds = tuple(
            _number(value[name], f"{path}.{name}")
            for name in ("robot_radius", "arrival_tolerance")
        )
        for name, bound in zip(("robot_radius", "arrival_tolerance"), bounds):
            if bound <= 0:
                raise ValueError(f"{path}.{name}: expected a positive number")
        docks[dock_id] = DockConfig(
            _pose(value["staging_pose"], f"{path}.staging_pose"),
            _pose(value["charging_pose"], f"{path}.charging_pose"),
            *bounds,
            MappingProxyType(exit_poses),
            tuple(departures),
        )
    for dock_id in sorted(dock_ids - docks.keys()):
        raise ValueError(
            f"docks.{dock_id}: dock resource requires staging and charging poses"
        )

    simulation_defaults = {
        "idle_percent_per_sec": 0.001,
        "move_percent_per_m": 0.1,
        "operation_percent": 0.5,
        "charge_percent_per_sec": 1.0,
        "dock_allowance_m": 10.0,
    }
    energy_values = _mapping(data["energy"], "energy")
    rates = {}
    for name, default in simulation_defaults.items():
        rates[name] = _number(energy_values.get(name, default), "energy." + name)
        if rates[name] < 0:
            raise ValueError(f"energy.{name}: expected a nonnegative number")
    value = _mapping(
        {
            key: item
            for key, item in energy_values.items()
            if key not in simulation_defaults
        },
        "energy",
        ("reserve_percent", "charge_below_percent", "charge_until_percent", "dock_id"),
    )
    reserve = _percent(value["reserve_percent"], "energy.reserve_percent")
    charge_below = _percent(
        value["charge_below_percent"], "energy.charge_below_percent"
    )
    charge_until = _percent(
        value["charge_until_percent"], "energy.charge_until_percent"
    )
    if not reserve < charge_below < charge_until:
        raise ValueError(
            "energy.charge_below_percent: require reserve_percent < charge_below_percent < charge_until_percent"
        )
    dock_id = _identifier(value["dock_id"], "energy.dock_id")
    if dock_id not in docks:
        raise ValueError(f"energy.dock_id: unknown dock {dock_id!r}")
    energy = EnergyPolicyConfig(reserve, charge_below, charge_until, dock_id, **rates)
    station_staging = {}
    station_ids = {r.resource_id for r in resources if r.kind == "station"}
    route_poses = [
        pose
        for segments in route_segments.values()
        for segment in segments
        for pose in (segment.staging_pose, segment.exit_pose)
    ]
    for station, values in _mapping(station_geometry, "station_staging").items():
        path = f"station_staging.{station}"
        if station not in station_ids or station not in resource_bounds:
            raise ValueError(
                f"{path}: requires a configured station and physical bounds"
            )
        pose = _pose(values, path)
        left, bottom, right, top = resource_bounds[station]
        distance = math.hypot(
            max(left - pose.x, 0, pose.x - right), max(bottom - pose.y, 0, pose.y - top)
        )
        if distance <= 0.60:
            raise ValueError(f"{path}: waiting footprint must clear station bounds")
        if any(
            math.hypot(pose.x - other.x, pose.y - other.y) <= 0.60
            for other in route_poses
        ):
            raise ValueError(
                f"{path}: waiting footprint must clear route staging and exit poses"
            )
        if any(
            math.hypot(pose.x - other.x, pose.y - other.y) <= 0.60
            for other in station_staging.values()
        ):
            raise ValueError(f"{path}: station waiting footprints must be distinct")
        station_staging[station] = pose
    station_approach = {}
    for station, values in _mapping(approach_geometry, "station_approach").items():
        path = f"station_approach.{station}"
        if station not in station_staging:
            raise ValueError(f"{path}: requires station bounds and waiting pose")
        pose = _pose(values, path)
        left, bottom, right, top = resource_bounds[station]
        if not (left < pose.x < right and bottom < pose.y < top):
            raise ValueError(f"{path}: approach must lie inside station bounds")
        station_approach[station] = pose
    if set(station_approach) != set(station_staging):
        raise ValueError("station_approach: every waiting station requires an approach")
    station_exit_poses = {}
    robot_ids = {robot.robot_id for robot in robots}
    for station, values in _mapping(exit_geometry, "station_exit_poses").items():
        path = f"station_exit_poses.{station}"
        if station not in station_staging:
            raise ValueError(f"{path}: requires station bounds and waiting pose")
        values = _mapping(values, path)
        if set(values) != robot_ids:
            raise ValueError(f"{path}: requires every configured robot exactly once")
        station_exit_poses[station] = MappingProxyType(
            {
                robot_id: _pose(value, f"{path}.{robot_id}")
                for robot_id, value in values.items()
            }
        )
    if set(station_exit_poses) != set(station_staging):
        raise ValueError(
            "station_exit_poses: every waiting station requires terminal parking"
        )
    clearance = max(
        2 * (dock.robot_radius + dock.arrival_tolerance) for dock in docks.values()
    )
    dock_bounds = [
        (
            dock.charging_pose.x - dock.arrival_tolerance,
            dock.charging_pose.y - dock.arrival_tolerance,
            dock.charging_pose.x + dock.arrival_tolerance,
            dock.charging_pose.y + dock.arrival_tolerance,
        )
        for dock in docks.values()
    ]
    parking = [
        (station, robot_id, pose)
        for station, poses in station_exit_poses.items()
        for robot_id, pose in poses.items()
    ]
    other_holding = [
        *route_poses,
        *station_staging.values(),
        *(p for d in docks.values() for p in (d.staging_pose, d.charging_pose)),
        *(r.spawn for r in robots),
    ]
    dock_exits = [
        (name, robot_id, pose)
        for name, dock in docks.items()
        for robot_id, pose in dock.exit_poses.items()
    ]
    protected = [*traffic_bounds.values(), *resource_bounds.values()]
    resource_margin = max(
        2 * d.robot_radius + d.arrival_tolerance for d in docks.values()
    )
    travel_holding = [
        *route_poses,
        *station_staging.values(),
        *(p for _, _, p in parking),
    ]

    def validate_dock_segment(start, end, path, label, holding):
        if any(
            _segment_bounds_distance(start, end, zone) <= resource_margin
            for zone in protected
        ):
            raise ValueError(f"{path}: {label} crosses resource bounds")
        if any(_segment_distance(start, end, pose) <= clearance for pose in holding):
            raise ValueError(f"{path}: {label} must clear holding poses")

    for name, robot_id, pose in dock_exits:
        dock = docks[name]
        path = f"docks.{name}.exit_poses.{robot_id}"
        others = [p for n, r, p in dock_exits if (n, r) != (name, robot_id)]
        # A robot can occupy only one of its own vacated bays. Exempt identity,
        # never a coordinate shared with another robot's terminal bay.
        occupied_holding = [
            *route_poses,
            *station_staging.values(),
            *(p for _, r, p in parking if r != robot_id),
        ]
        if (
            math.hypot(
                dock.staging_pose.x - dock.charging_pose.x,
                dock.staging_pose.y - dock.charging_pose.y,
            )
            <= clearance
        ):
            raise ValueError(f"docks.{name}: dock staging must clear charger")
        if any(
            _segment_bounds_distance(pose, pose, zone) <= clearance
            for zone in protected
        ) or any(
            math.hypot(pose.x - p.x, pose.y - p.y) <= clearance
            for p in [
                *route_poses,
                *station_staging.values(),
                *(p for d in docks.values() for p in (d.staging_pose, d.charging_pose)),
                *(r.spawn for r in robots if r.robot_id != robot_id),
                *(p for _, r, p in parking if r != robot_id),
                *others,
            ]
        ):
            raise ValueError(f"{path}: dock exit footprint must clear holding poses")
        validate_dock_segment(
            dock.charging_pose,
            pose,
            path,
            "charging approach",
            [
                *occupied_holding,
                dock.staging_pose,
                *others,
                *(
                    p
                    for n, d in docks.items()
                    if n != name
                    for p in (d.staging_pose, d.charging_pose)
                ),
            ],
        )
        for robot in robots:
            if (
                robot.robot_id == robot_id
                and robot.battery_start_percent < energy.charge_below_percent
            ):
                validate_dock_segment(
                    robot.spawn,
                    dock.staging_pose,
                    path,
                    "dock approach",
                    [
                        *travel_holding,
                        *others,
                        dock.charging_pose,
                        *(r.spawn for r in robots if r.robot_id != robot_id),
                    ],
                )
        for station in dock.departure_stations:
            if station not in station_exit_poses:
                raise ValueError(
                    f"docks.{name}.departure_stations: requires terminal bays"
                )
            start = station_exit_poses[station][robot_id]
            validate_dock_segment(
                start,
                dock.staging_pose,
                path,
                "station dock approach",
                [
                    *occupied_holding,
                    *(p for _, r, p in dock_exits if r != robot_id),
                    *(r.spawn for r in robots if r.robot_id != robot_id),
                    *(
                        p
                        for n, d in docks.items()
                        for p in (d.staging_pose, d.charging_pose)
                        if n != name or p != dock.staging_pose
                    ),
                ],
            )
    for station, robot_id, pose in parking:
        path = f"station_exit_poses.{station}.{robot_id}"
        for left, bottom, right, top in (
            *resource_bounds.values(),
            *traffic_bounds.values(),
            *dock_bounds,
        ):
            if (
                math.hypot(
                    max(left - pose.x, 0, pose.x - right),
                    max(bottom - pose.y, 0, pose.y - top),
                )
                <= clearance
            ):
                raise ValueError(
                    f"{path}: parking footprint must clear station, traffic and dock bounds"
                )
        if any(
            math.hypot(pose.x - p.x, pose.y - p.y) <= clearance
            for p in [
                *other_holding,
                *(p for s, r, p in parking if (s, r) != (station, robot_id)),
            ]
        ):
            raise ValueError(f"{path}: parking footprint must clear all holding poses")
        start = station_approach[station]
        for zone in [
            *traffic_bounds.values(),
            *dock_bounds,
            *(bounds for name, bounds in resource_bounds.items() if name != station),
        ]:
            # A clearance-expanded segment envelope conservatively rejects
            # travel through another resource. The held station is exempt.
            if not (
                max(start.x, pose.x) + clearance < zone[0]
                or min(start.x, pose.x) - clearance > zone[2]
                or max(start.y, pose.y) + clearance < zone[1]
                or min(start.y, pose.y) - clearance > zone[3]
            ):
                raise ValueError(
                    f"{path}: station parking approach crosses resource bounds"
                )
        for other_station, other_robot, other in parking:
            if (other_station, other_robot) != (
                station,
                robot_id,
            ) and _segment_distance(start, pose, other) <= clearance:
                raise ValueError(
                    f"{path}: parking approach must clear other terminal bays"
                )
        if any(
            _segment_distance(start, pose, other) <= clearance
            for other in other_holding
        ):
            raise ValueError(f"{path}: parking approach must clear holding poses")
    recovery_margin = max(
        (2 * d.robot_radius + d.arrival_tolerance for d in docks.values()), default=0.0
    )
    waypoint_margin = max(
        (2 * (d.robot_radius + d.arrival_tolerance) for d in docks.values()),
        default=0.0,
    )
    holding_poses = (
        route_poses
        + list(station_staging.values())
        + [pose for _, _, pose in parking]
        + [
            pose
            for dock in docks.values()
            for pose in (dock.staging_pose, dock.charging_pose)
        ]
    )
    for index, robot in enumerate(robots):
        pose, path = robot.spawn, f"robots[{index}].spawn"
        for left, bottom, right, top in (
            *traffic_bounds.values(),
            *resource_bounds.values(),
        ):
            distance = math.hypot(
                max(left - pose.x, 0, pose.x - right),
                max(bottom - pose.y, 0, pose.y - top),
            )
            if distance <= recovery_margin:
                raise ValueError(
                    f"{path}: initial footprint must clear resource recovery bounds"
                )
        if any(
            math.hypot(pose.x - other.x, pose.y - other.y) <= waypoint_margin
            for other in holding_poses
        ):
            raise ValueError(
                f"{path}: initial footprint must clear holding and dock poses"
            )
        if any(
            math.hypot(pose.x - other.spawn.x, pose.y - other.spawn.y)
            <= waypoint_margin
            for other in robots[:index]
        ):
            raise ValueError(f"{path}: initial robot footprints must be separated")
        for name, segments in route_segments.items():
            if not name.startswith("to_") or not segments:
                continue
            points = [robot.spawn]
            for segment in segments:
                points.extend((segment.staging_pose, segment.exit_pose))
            for start, end in zip(points, points[1:]):
                if any(
                    _segment_distance(start, end, other.spawn) < waypoint_margin
                    for other in robots
                    if other.robot_id != robot.robot_id
                ):
                    raise ValueError(
                        f"{path}: initial route {name} must clear idle startup bays"
                    )
    return FleetConfig(
        tuple(robots),
        tuple(resources),
        MappingProxyType(routes),
        MappingProxyType(docks),
        energy,
        MappingProxyType(route_segments),
        MappingProxyType(traffic_bounds),
        MappingProxyType(resource_bounds),
        MappingProxyType(station_staging),
        MappingProxyType(station_approach),
        MappingProxyType(station_exit_poses),
    )


def load_fleet_config(path: Path) -> FleetConfig:
    """Raise ValueError with the file and field path for invalid fleet inputs."""
    path = Path(path)
    try:
        with path.open(encoding="utf-8") as stream:
            data = yaml.load(stream, Loader=_FleetLoader)
        return _parse_config(data)
    except (OSError, UnicodeError, yaml.YAMLError, ValueError) as exc:
        raise ValueError(f"{path}: {exc}") from exc
