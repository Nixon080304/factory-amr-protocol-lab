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


@dataclass(frozen=True)
class EnergyPolicyConfig:
    reserve_percent: float
    charge_below_percent: float
    charge_until_percent: float
    dock_id: str


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


def _parse_config(data):
    data = _mapping(data, "fleet")
    geometry = data.get("route_segments", {})
    zone_geometry = data.get("traffic_bounds", {})
    data = {
        key: value
        for key, value in data.items()
        if key not in ("route_segments", "traffic_bounds")
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
                # 0.35 m footprint radius plus 0.25 m arrival tolerance.
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
        value = _mapping(value, path, ("staging_pose", "charging_pose"))
        docks[dock_id] = DockConfig(
            _pose(value["staging_pose"], f"{path}.staging_pose"),
            _pose(value["charging_pose"], f"{path}.charging_pose"),
        )
    for dock_id in sorted(dock_ids - docks.keys()):
        raise ValueError(
            f"docks.{dock_id}: dock resource requires staging and charging poses"
        )

    value = _mapping(
        data["energy"],
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
    energy = EnergyPolicyConfig(reserve, charge_below, charge_until, dock_id)
    return FleetConfig(
        tuple(robots),
        tuple(resources),
        MappingProxyType(routes),
        MappingProxyType(docks),
        energy,
        MappingProxyType(route_segments),
        MappingProxyType(traffic_bounds),
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
