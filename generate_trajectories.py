#!/usr/bin/env python3
"""Generate two simple MRS-compatible UAV trajectory files."""

from __future__ import annotations

import argparse
import csv
import math
import random
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable, Sequence

import yaml


# Dataset-scene dimensions chosen for world_temesvar_field_1.  That field is
# approximately 137 x 161 m, with about 47 m of clearance at its safest point.
# Smaller worlds remain supported because both range and travel are reduced by
# the fitting code below when necessary.
DEFAULT_DATASET_MINIMUM_DISTANCE = 5.0
DEFAULT_DATASET_MAXIMUM_DISTANCE_FACTOR = 3.0
DEFAULT_DATASET_TRAVEL_DISTANCE = 40.0
DEFAULT_CONSTRAINTS_PATH = (
    Path(__file__).resolve().parent / "constraints" / "mrs_default.yaml"
)


EARTH_RADIUS_M = 6_378_137.0


class ConfigurationError(ValueError):
    """Raised when a world configuration cannot define a safe trajectory."""


@dataclass(frozen=True)
class SafetyArea:
    polygon: tuple[tuple[float, float], ...]
    min_z: float
    max_z: float
    frame_id: str


@dataclass(frozen=True)
class DynamicConstraints:
    """One named MRS ConstraintManager dynamics profile."""

    name: str
    horizontal: dict[str, float]
    vertical_ascending: dict[str, float]
    vertical_descending: dict[str, float]
    heading: dict[str, float]


@dataclass(frozen=True)
class PlatformConfig:
    """Constraint-related information extracted from an MRS platform overlay."""

    name: str
    motor_count: int | None
    allowed_constraints: tuple[str, ...]


@dataclass(frozen=True)
class ConstraintViolation:
    quantity: str
    sample_index: int
    value: float
    limit: float


@dataclass(frozen=True)
class ConstraintVerification:
    maxima: dict[str, float]
    violations: tuple[ConstraintViolation, ...]

    @property
    def passed(self) -> bool:
        return not self.violations


@dataclass
class RRTVisualization:
    """Planning data rendered with a plotted random-walk trajectory."""

    points: list[tuple[float, float, float]] | None = None
    spline_controls: list[tuple[float, float, float]] | None = None
    observer_xy: tuple[float, float] | None = None
    observer_heading: float | None = None
    horizontal_fov: float | None = None
    minimum_distance: float | None = None
    maximum_distance: float | None = None


def _find_mapping(node: Any, key: str) -> dict[str, Any] | None:
    """Find the first nested mapping named *key* (supports ROS1 and ROS2 YAML)."""
    if not isinstance(node, dict):
        return None
    value = node.get(key)
    if isinstance(value, dict):
        return value
    for child in node.values():
        found = _find_mapping(child, key)
        if found is not None:
            return found
    return None


def _pairs(values: Any) -> tuple[tuple[float, float], ...]:
    if not isinstance(values, list):
        raise ConfigurationError("safety_area.horizontal.points must be a YAML list")
    if values and all(isinstance(item, (list, tuple)) for item in values):
        try:
            points = tuple((float(item[0]), float(item[1])) for item in values)
        except (IndexError, TypeError, ValueError) as exc:
            raise ConfigurationError("each safety-area point must contain two numbers") from exc
    else:
        if len(values) % 2:
            raise ConfigurationError("the flat safety-area points list must have even length")
        try:
            points = tuple((float(values[i]), float(values[i + 1])) for i in range(0, len(values), 2))
        except (TypeError, ValueError) as exc:
            raise ConfigurationError("safety-area points must be numeric") from exc
    if len(points) < 3:
        raise ConfigurationError("the safety-area polygon needs at least three points")
    return points


def _latlon_to_local(
    points: Iterable[tuple[float, float]], origin_lat: float, origin_lon: float
) -> tuple[tuple[float, float], ...]:
    """Convert latitude/longitude degrees to local east/north metres.

    The local tangent-plane approximation is more than adequate for compact UAV
    safety areas. A conservative boundary margin is still applied later.
    """
    result = []
    for latitude, longitude in points:
        mean_latitude = math.radians((latitude + origin_lat) / 2.0)
        east = EARTH_RADIUS_M * math.radians(longitude - origin_lon) * math.cos(mean_latitude)
        north = EARTH_RADIUS_M * math.radians(latitude - origin_lat)
        result.append((east, north))
    return tuple(result)


def load_safety_area(config_path: Path) -> SafetyArea:
    try:
        document = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    except OSError as exc:
        raise ConfigurationError(f"cannot read {config_path}: {exc}") from exc
    except yaml.YAMLError as exc:
        raise ConfigurationError(f"invalid YAML in {config_path}: {exc}") from exc

    safety = _find_mapping(document, "safety_area")
    if safety is None:
        raise ConfigurationError("could not find a 'safety_area' mapping in the config")
    horizontal = safety.get("horizontal")
    vertical = safety.get("vertical")
    if not isinstance(horizontal, dict) or not isinstance(vertical, dict):
        raise ConfigurationError("safety_area must contain horizontal and vertical mappings")

    points = _pairs(horizontal.get("points"))
    horizontal_frame = str(horizontal.get("frame_name", "world_origin"))
    try:
        min_z = float(vertical["min_z"])
        max_z = float(vertical["max_z"])
    except (KeyError, TypeError, ValueError) as exc:
        raise ConfigurationError("vertical safety area needs numeric min_z and max_z") from exc
    if not min_z < max_z:
        raise ConfigurationError("safety-area min_z must be smaller than max_z")

    if horizontal_frame == "latlon_origin":
        origin = _find_mapping(document, "world_origin")
        if origin is None or str(origin.get("units", "")).upper() != "LATLON":
            raise ConfigurationError(
                "latlon_origin points require a LATLON world_origin with origin_x/origin_y"
            )
        try:
            origin_lat = float(origin["origin_x"])
            origin_lon = float(origin["origin_y"])
        except (KeyError, TypeError, ValueError) as exc:
            raise ConfigurationError("world_origin needs numeric origin_x and origin_y") from exc
        points = _latlon_to_local(points, origin_lat, origin_lon)
        frame_id = "world_origin"
    else:
        frame_id = horizontal_frame

    return SafetyArea(points, min_z, max_z, frame_id)


def _load_yaml(config_path: Path) -> Any:
    try:
        return yaml.safe_load(config_path.read_text(encoding="utf-8"))
    except OSError as exc:
        raise ConfigurationError(f"cannot read {config_path}: {exc}") from exc
    except yaml.YAMLError as exc:
        raise ConfigurationError(f"invalid YAML in {config_path}: {exc}") from exc


def _derivative_limits(section: Any, label: str) -> dict[str, float]:
    if not isinstance(section, dict):
        raise ConfigurationError(f"constraint profile is missing the '{label}' mapping")
    result: dict[str, float] = {}
    for quantity in ("speed", "acceleration", "jerk", "snap"):
        try:
            value = float(section[quantity])
        except (KeyError, TypeError, ValueError) as exc:
            raise ConfigurationError(
                f"constraint '{label}.{quantity}' must be a number"
            ) from exc
        if value <= 0.0:
            raise ConfigurationError(f"constraint '{label}.{quantity}' must be positive")
        result[quantity] = value
    return result


def load_dynamic_constraints(config_path: Path, profile: str) -> DynamicConstraints:
    """Load one profile from an MRS ConstraintManager YAML file."""
    document = _load_yaml(config_path)
    manager = _find_mapping(document, "constraint_manager")
    if manager is None:
        manager = document if isinstance(document, dict) else None
    profile_data = manager.get(profile) if isinstance(manager, dict) else None
    if not isinstance(profile_data, dict):
        available = []
        if isinstance(manager, dict):
            available = [
                str(name)
                for name, value in manager.items()
                if isinstance(value, dict) and "horizontal" in value and "vertical" in value
            ]
        suffix = f"; available profiles: {', '.join(available)}" if available else ""
        raise ConfigurationError(f"constraint profile '{profile}' was not found{suffix}")

    vertical = profile_data.get("vertical")
    if not isinstance(vertical, dict):
        raise ConfigurationError("constraint profile is missing the 'vertical' mapping")
    return DynamicConstraints(
        name=profile,
        horizontal=_derivative_limits(profile_data.get("horizontal"), "horizontal"),
        vertical_ascending=_derivative_limits(vertical.get("ascending"), "vertical.ascending"),
        vertical_descending=_derivative_limits(vertical.get("descending"), "vertical.descending"),
        heading=_derivative_limits(profile_data.get("heading"), "heading"),
    )


def load_platform_config(config_path: Path) -> PlatformConfig:
    """Load an MRS platform overlay and collect its constraint policy.

    Platform files such as x500.yaml may contain only vehicle/motor parameters;
    in that case the generic profiles from the managers config remain usable.
    Other platform files restrict the profiles through ``constraints`` or
    ``allowed_constraints`` entries, which are enforced here.
    """
    document = _load_yaml(config_path)
    if not isinstance(document, dict) or not document:
        raise ConfigurationError(f"platform config {config_path} must be a YAML mapping")

    motor_count: int | None = None
    motor_params = document.get("motor_params")
    if isinstance(motor_params, dict) and "n_motors" in motor_params:
        try:
            numeric_motor_count = float(motor_params["n_motors"])
        except (TypeError, ValueError) as exc:
            raise ConfigurationError("platform motor_params.n_motors must be an integer") from exc
        if numeric_motor_count <= 0.0 or not numeric_motor_count.is_integer():
            raise ConfigurationError("platform motor_params.n_motors must be a positive integer")
        motor_count = int(numeric_motor_count)

    allowed: set[str] = set()
    manager = _find_mapping(document, "constraint_manager")
    if manager is not None:
        declared = manager.get("constraints")
        if declared is not None:
            if not isinstance(declared, list) or not all(
                isinstance(item, str) for item in declared
            ):
                raise ConfigurationError("platform constraint_manager.constraints must be a list")
            allowed.update(declared)
        per_odometry = manager.get("allowed_constraints")
        if per_odometry is not None:
            if not isinstance(per_odometry, dict):
                raise ConfigurationError(
                    "platform constraint_manager.allowed_constraints must be a mapping"
                )
            for odometry_mode, profiles in per_odometry.items():
                if not isinstance(profiles, list) or not all(
                    isinstance(item, str) for item in profiles
                ):
                    raise ConfigurationError(
                        "allowed constraints for odometry mode "
                        f"'{odometry_mode}' must be a list of names"
                    )
                allowed.update(profiles)

    return PlatformConfig(config_path.stem, motor_count, tuple(sorted(allowed)))


def validate_platform_constraint_profile(
    platform: PlatformConfig, profile: str
) -> None:
    """Reject a constraint profile explicitly disallowed by a platform overlay."""
    if platform.allowed_constraints and profile not in platform.allowed_constraints:
        available = ", ".join(platform.allowed_constraints)
        raise ConfigurationError(
            f"constraint profile '{profile}' is not allowed by platform "
            f"'{platform.name}'; allowed profiles: {available}"
        )


def _unwrap_headings(headings: Sequence[float]) -> list[float]:
    if not headings:
        return []
    unwrapped = [headings[0]]
    for heading in headings[1:]:
        difference = (heading - unwrapped[-1] + math.pi) % (2.0 * math.pi) - math.pi
        unwrapped.append(unwrapped[-1] + difference)
    return unwrapped


def verify_dynamic_constraints(
    trajectory: Sequence[tuple[float, float, float, float]],
    dt: float,
    constraints: DynamicConstraints,
    tolerance: float = 1e-9,
) -> ConstraintVerification:
    """Verify sampled poses against an MRS dynamics-constraint profile.

    Successive finite differences estimate speed, acceleration, jerk, and snap.
    Horizontal limits apply to the XY vector magnitude. Positive vertical
    derivatives use ascending limits and negative derivatives use descending
    limits. Heading is unwrapped before differentiating across +/-pi.
    """
    if dt <= 0.0:
        raise ConfigurationError("dt must be positive when verifying constraints")
    if tolerance < 0.0:
        raise ConfigurationError("constraint tolerance cannot be negative")
    if len(trajectory) < 5:
        raise ConfigurationError(
            "at least five trajectory samples are required to verify through snap"
        )

    values = [
        [float(point[0]), float(point[1]), float(point[2]), 0.0]
        for point in trajectory
    ]
    headings = _unwrap_headings([float(point[3]) for point in trajectory])
    for row, heading in zip(values, headings):
        row[3] = heading

    quantity_names = ("speed", "acceleration", "jerk", "snap")
    maxima: dict[str, float] = {}
    violations: list[ConstraintViolation] = []
    derivatives = values
    for order, quantity in enumerate(quantity_names, start=1):
        if len(derivatives) < 2:
            break
        derivatives = [
            [(current[axis] - previous[axis]) / dt for axis in range(4)]
            for previous, current in zip(derivatives, derivatives[1:])
        ]
        for derivative_index, derivative in enumerate(derivatives):
            sample_index = derivative_index + order
            horizontal_value = math.hypot(derivative[0], derivative[1])
            vertical_value = derivative[2]
            heading_value = abs(derivative[3])
            measurements = (
                (f"horizontal.{quantity}", horizontal_value, constraints.horizontal[quantity]),
                (
                    f"vertical.{'ascending' if vertical_value >= 0.0 else 'descending'}.{quantity}",
                    abs(vertical_value),
                    (
                        constraints.vertical_ascending[quantity]
                        if vertical_value >= 0.0
                        else constraints.vertical_descending[quantity]
                    ),
                ),
                (f"heading.{quantity}", heading_value, constraints.heading[quantity]),
            )
            for name, measured, limit in measurements:
                maxima[name] = max(maxima.get(name, 0.0), measured)
                if measured > limit + tolerance:
                    violations.append(
                        ConstraintViolation(name, sample_index, measured, limit)
                    )

    return ConstraintVerification(maxima, tuple(violations))


def _inside(point: tuple[float, float], polygon: Sequence[tuple[float, float]]) -> bool:
    x, y = point
    inside = False
    previous = polygon[-1]
    for current in polygon:
        x1, y1 = previous
        x2, y2 = current
        if (y1 > y) != (y2 > y):
            crossing_x = (x2 - x1) * (y - y1) / (y2 - y1) + x1
            if x < crossing_x:
                inside = not inside
        previous = current
    return inside


def _point_segment_distance(
    point: tuple[float, float], start: tuple[float, float], end: tuple[float, float]
) -> float:
    px, py = point
    ax, ay = start
    bx, by = end
    dx, dy = bx - ax, by - ay
    length_squared = dx * dx + dy * dy
    if length_squared == 0.0:
        return math.hypot(px - ax, py - ay)
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / length_squared))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


def _clearance(point: tuple[float, float], polygon: Sequence[tuple[float, float]]) -> float:
    if not _inside(point, polygon):
        return -1.0
    return min(
        _point_segment_distance(point, polygon[i - 1], polygon[i])
        for i in range(len(polygon))
    )


def safest_point(polygon: Sequence[tuple[float, float]]) -> tuple[tuple[float, float], float]:
    """Approximately maximize distance from the boundary using a refined grid."""
    min_x = min(point[0] for point in polygon)
    max_x = max(point[0] for point in polygon)
    min_y = min(point[1] for point in polygon)
    max_y = max(point[1] for point in polygon)
    if min_x == max_x or min_y == max_y:
        raise ConfigurationError("safety-area polygon has zero width or height")

    best = ((min_x + max_x) / 2.0, (min_y + max_y) / 2.0)
    best_clearance = _clearance(best, polygon)
    low_x, high_x, low_y, high_y = min_x, max_x, min_y, max_y
    samples = 31
    for _ in range(6):
        step_x = (high_x - low_x) / (samples - 1)
        step_y = (high_y - low_y) / (samples - 1)
        for ix in range(samples):
            for iy in range(samples):
                candidate = (low_x + ix * step_x, low_y + iy * step_y)
                candidate_clearance = _clearance(candidate, polygon)
                if candidate_clearance > best_clearance:
                    best, best_clearance = candidate, candidate_clearance
        low_x, high_x = best[0] - step_x, best[0] + step_x
        low_y, high_y = best[1] - step_y, best[1] + step_y

    if best_clearance <= 0.0:
        raise ConfigurationError("could not find a point inside the safety-area polygon")
    return best, best_clearance


def generate(
    area: SafetyArea,
    dt: float,
    duration: float,
    separation: float,
    margin: float,
    vertical_margin: float | None = None,
) -> tuple[list[tuple[float, float, float, float]], ...]:
    if dt < 0.01:
        raise ConfigurationError("dt must be at least 0.01 s for the MRS MpcTracker")
    if duration < dt:
        raise ConfigurationError("duration must be at least one dt")
    if separation <= 0.0:
        raise ConfigurationError("separation must be positive")
    vertical_margin = _vertical_margin(margin, vertical_margin)

    center, clearance = safest_point(area.polygon)
    usable_radius = clearance - margin
    half_separation = separation / 2.0
    if usable_radius <= half_separation:
        raise ConfigurationError(
            f"safety area is too small: interior clearance is {clearance:.2f} m, "
            f"but margin + half-separation needs {margin + half_separation:.2f} m"
        )
    half_length = min(2.0, 0.8 * math.sqrt(usable_radius**2 - half_separation**2))
    altitude = (area.min_z + area.max_z) / 2.0
    if not area.min_z + vertical_margin <= altitude <= area.max_z - vertical_margin:
        raise ConfigurationError("vertical safety area is smaller than twice the vertical margin")
    count = math.floor(duration / dt + 1e-9) + 1
    trajectories: list[list[tuple[float, float, float, float]]] = [[], []]
    for index in range(count):
        progress = index / (count - 1) if count > 1 else 0.0
        # Half-cosine motion has zero velocity at both ends.
        along = -half_length * math.cos(math.pi * progress)
        for uav_index, cross_track in enumerate((-half_separation, half_separation)):
            trajectories[uav_index].append(
                (center[0] + along, center[1] + cross_track, altitude, 0.0)
            )

    for trajectory in trajectories:
        for x, y, z, _ in trajectory:
            if not (
                _inside((x, y), area.polygon)
                and area.min_z + vertical_margin <= z <= area.max_z - vertical_margin
            ):
                raise RuntimeError("internal error: generated point lies outside the safety area")
    return tuple(trajectories)


def generate_straight_and_helix(
    area: SafetyArea,
    dt: float,
    duration: float,
    lead_distance: float,
    helix_radius: float,
    helix_turns: float,
    margin: float,
    vertical_margin: float | None = None,
) -> tuple[list[tuple[float, float, float, float]], ...]:
    """Generate a straight follower and a leading UAV flying a 3D helix.

    Both UAVs progress in the positive x direction. UAV 2 stays
    ``lead_distance`` metres ahead in x while circling the forward axis in the
    y-z plane. The complete helix is kept inside the configured safety area.
    """
    if dt < 0.01:
        raise ConfigurationError("dt must be at least 0.01 s for the MRS MpcTracker")
    if duration < dt:
        raise ConfigurationError("duration must be at least one dt")
    if lead_distance <= 0.0:
        raise ConfigurationError("lead distance must be positive")
    if helix_radius <= 0.0:
        raise ConfigurationError("helix radius must be positive")
    if helix_turns <= 0.0:
        raise ConfigurationError("helix turns must be positive")
    vertical_margin = _vertical_margin(margin, vertical_margin)

    center, clearance = safest_point(area.polygon)
    usable_horizontal_radius = clearance - margin
    if usable_horizontal_radius <= helix_radius:
        raise ConfigurationError(
            f"helix radius {helix_radius:.2f} m does not fit within the "
            f"{usable_horizontal_radius:.2f} m usable horizontal radius"
        )

    # Every generated horizontal point stays inside the largest known-safe
    # circle around center. The 0.8 factor leaves room for approximation and
    # makes the resulting trajectory visibly clear of the safety boundary.
    maximum_x_offset = 0.8 * math.sqrt(
        usable_horizontal_radius**2 - helix_radius**2
    )
    available_travel_distance = 2.0 * maximum_x_offset - lead_distance
    if available_travel_distance <= 0.0:
        raise ConfigurationError(
            f"lead distance {lead_distance:.2f} m is too large for this safety area"
        )
    travel_distance = min(4.0, available_travel_distance)

    altitude = (area.min_z + area.max_z) / 2.0
    vertical_room = (area.max_z - area.min_z) / 2.0 - vertical_margin
    if helix_radius > vertical_room:
        raise ConfigurationError(
            f"helix radius {helix_radius:.2f} m does not fit between the "
            f"vertical limits with a {vertical_margin:.2f} m vertical margin"
        )

    count = math.floor(duration / dt + 1e-9) + 1
    straight: list[tuple[float, float, float, float]] = []
    helix: list[tuple[float, float, float, float]] = []
    for index in range(count):
        progress = index / (count - 1) if count > 1 else 0.0
        # Half-cosine progress starts and finishes with zero forward velocity.
        along = -travel_distance / 2.0 + travel_distance * (
            1.0 - math.cos(math.pi * progress)
        ) / 2.0
        phase = 2.0 * math.pi * helix_turns * progress
        straight.append(
            (center[0] - lead_distance / 2.0 + along, center[1], altitude, 0.0)
        )
        helix.append(
            (
                center[0] + lead_distance / 2.0 + along,
                center[1] + helix_radius * math.cos(phase),
                altitude + helix_radius * math.sin(phase),
                0.0,
            )
        )

    trajectories = (straight, helix)
    for trajectory in trajectories:
        for x, y, z, _ in trajectory:
            if not _inside((x, y), area.polygon) or not area.min_z <= z <= area.max_z:
                raise RuntimeError("internal error: generated point lies outside the safety area")
    return trajectories


def _wrap_angle(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def _trajectory_length(
    trajectory: Sequence[tuple[float, float, float, float]],
) -> float:
    """Return the sampled 3D path length in metres."""
    return sum(
        math.dist(previous[:3], current[:3])
        for previous, current in zip(trajectory, trajectory[1:])
    )


def _vertical_margin(horizontal_margin: float, vertical_margin: float | None) -> float:
    """Return an explicit vertical margin, preserving the legacy shared default."""
    if horizontal_margin < 0.0:
        raise ConfigurationError("horizontal margin cannot be negative")
    result = horizontal_margin if vertical_margin is None else vertical_margin
    if result < 0.0:
        raise ConfigurationError("vertical margin cannot be negative")
    return result


_PLACEMENT_DIRECTIONS = {
    "center": (0.0, 0.0),
    "north": (0.0, 1.0),
    "northeast": (1.0, 1.0),
    "east": (1.0, 0.0),
    "southeast": (1.0, -1.0),
    "south": (0.0, -1.0),
    "southwest": (-1.0, -1.0),
    "west": (-1.0, 0.0),
    "northwest": (-1.0, 1.0),
}


def shift_trajectories_to_boundary(
    area: SafetyArea,
    trajectories: Sequence[Sequence[tuple[float, float, float, float]]],
    direction: str,
    boundary_offset: float,
    margin: float,
) -> tuple[list[tuple[float, float, float, float]], ...]:
    """Translate complete trajectories toward one safety-area border.

    ``direction`` uses the world frame (east is +x and north is +y). Before
    placement, the mean XY position across both trajectories is moved to the
    center of the largest inscribed safety-area circle. The trajectories then
    move as far as possible while retaining ``margin`` from the polygon, and
    move back by ``boundary_offset`` metres. Applying translations uniformly
    to every pose retains all relative geometry and dynamics.
    """
    if direction not in _PLACEMENT_DIRECTIONS:
        choices = ", ".join(_PLACEMENT_DIRECTIONS)
        raise ConfigurationError(f"unknown placement direction '{direction}'; choose: {choices}")
    if boundary_offset < 0.0:
        raise ConfigurationError("boundary offset cannot be negative")
    if margin < 0.0:
        raise ConfigurationError("margin cannot be negative")
    if not trajectories or not all(trajectory for trajectory in trajectories):
        raise ConfigurationError("cannot place empty trajectories")

    all_points = [point for trajectory in trajectories for point in trajectory]
    mean_x = sum(point[0] for point in all_points) / len(all_points)
    mean_y = sum(point[1] for point in all_points) / len(all_points)
    circle_center, _ = safest_point(area.polygon)
    center_dx, center_dy = circle_center[0] - mean_x, circle_center[1] - mean_y
    centered = tuple(
        [
            (point[0] + center_dx, point[1] + center_dy, point[2], point[3])
            for point in trajectory
        ]
        for trajectory in trajectories
    )

    def centered_fits() -> bool:
        return all(
            _clearance(point[:2], area.polygon) >= margin - 1e-9
            for trajectory in centered
            for point in trajectory
        )

    dx, dy = _PLACEMENT_DIRECTIONS[direction]
    if dx == 0.0 and dy == 0.0:
        if boundary_offset != 0.0:
            raise ConfigurationError("boundary offset requires a non-center placement direction")
        if not centered_fits():
            raise ConfigurationError(
                "centering the generated trajectory would leave the requested safety margin"
            )
        return centered
    length = math.hypot(dx, dy)
    dx, dy = dx / length, dy / length

    def fits(shift: float) -> bool:
        return all(
            _clearance((point[0] + dx * shift, point[1] + dy * shift), area.polygon)
            >= margin - 1e-9
            for trajectory in centered
            for point in trajectory
        )

    # A shift farther than the safety area's diagonal puts every point beyond
    # its bounding box. The original generator guarantees shift 0 fits, and
    # bisection finds the limiting boundary to sub-micrometre scale.
    min_x = min(point[0] for point in area.polygon)
    max_x = max(point[0] for point in area.polygon)
    min_y = min(point[1] for point in area.polygon)
    max_y = max(point[1] for point in area.polygon)
    lower, upper = 0.0, math.hypot(max_x - min_x, max_y - min_y) + 1.0
    if not fits(lower):
        raise ConfigurationError("generated trajectory does not satisfy the requested safety margin")
    for _ in range(60):
        midpoint = (lower + upper) / 2.0
        if fits(midpoint):
            lower = midpoint
        else:
            upper = midpoint

    shift = max(0.0, lower - boundary_offset)
    shifted = tuple(
        [
            (point[0] + dx * shift, point[1] + dy * shift, point[2], point[3])
            for point in trajectory
        ]
        for trajectory in centered
    )
    if not fits(shift):
        raise RuntimeError("internal error: boundary placement left the safety area")
    return shifted


def _shift_target_to_minimum_separation(
    area: SafetyArea,
    observer: Sequence[tuple[float, float, float, float]],
    target: Sequence[tuple[float, float, float, float]],
    minimum_distance: float,
    margin: float,
    vertical_margin: float | None = None,
) -> list[tuple[float, float, float, float]]:
    """Contract target offsets so the requested collision-free minimum is exact.

    A single scale factor preserves the target's bearing and elevation in UAV
    1's camera frame. Consequently FOV validity is unchanged while the closest
    sampled separation becomes exactly ``minimum_distance``.
    """
    vertical_margin = _vertical_margin(margin, vertical_margin)
    if len(observer) != len(target) or not observer:
        raise RuntimeError("internal error: observer and target samples do not match")
    distances = [
        math.dist(observer_point[:3], target_point[:3])
        for observer_point, target_point in zip(observer, target)
    ]
    current_minimum = min(distances)
    tolerance = 1e-8
    if current_minimum < minimum_distance - tolerance:
        raise ConfigurationError(
            "generated trajectories already violate the requested collision-free "
            f"separation of {minimum_distance:.2f} m"
        )
    if current_minimum <= 0.0:
        raise RuntimeError("internal error: coincident UAV trajectory samples")
    scale = minimum_distance / current_minimum
    shifted_target = [
        (
            observer_point[0] + scale * (target_point[0] - observer_point[0]),
            observer_point[1] + scale * (target_point[1] - observer_point[1]),
            observer_point[2] + scale * (target_point[2] - observer_point[2]),
            target_point[3],
        )
        for observer_point, target_point in zip(observer, target)
    ]
    shifted_distances = [
        math.dist(observer_point[:3], target_point[:3])
        for observer_point, target_point in zip(observer, shifted_target)
    ]
    if min(shifted_distances) < minimum_distance - tolerance or not math.isclose(
        min(shifted_distances), minimum_distance, abs_tol=tolerance
    ):
        raise RuntimeError("internal error: collision-free separation normalization failed")
    for point in shifted_target:
        if not (
            _clearance(point[:2], area.polygon) + tolerance >= margin
            and area.min_z + vertical_margin <= point[2] <= area.max_z - vertical_margin
        ):
            raise ConfigurationError(
                "bringing UAV 2 toward UAV 1 would leave the safety area; "
                "increase the safety margin or choose a different trajectory"
            )
    return shifted_target


def _dataset_relative_pose(
    pattern: str,
    progress: float,
    minimum_distance: float,
    maximum_distance: float,
    bearing_amplitude: float,
    elevation_amplitude: float,
    relative_heading_turns: float,
) -> tuple[float, float, float, float]:
    """Return target range, bearing, elevation, and heading for one sample."""
    if pattern == "weave":
        range_wave = math.sin(math.pi * progress) ** 2
        bearing = bearing_amplitude * math.sin(4.0 * math.pi * progress)
        elevation = elevation_amplitude * math.sin(2.0 * math.pi * progress)
    elif pattern == "orbit":
        range_wave = math.sin(math.pi * progress) ** 2
        phase = 4.0 * math.pi * progress
        bearing = bearing_amplitude * math.cos(phase)
        elevation = elevation_amplitude * math.sin(phase)
    elif pattern == "lissajous":
        range_wave = (1.0 - math.cos(2.0 * math.pi * progress)) / 2.0
        bearing = bearing_amplitude * math.sin(6.0 * math.pi * progress)
        elevation = elevation_amplitude * math.sin(
            4.0 * math.pi * progress + math.pi / 3.0
        )
    else:
        raise ConfigurationError(f"unknown dataset trajectory pattern '{pattern}'")

    distance = minimum_distance + (maximum_distance - minimum_distance) * range_wave
    target_heading = _wrap_angle(
        math.pi + 2.0 * math.pi * relative_heading_turns * progress
    )
    return distance, bearing, elevation, target_heading


def generate_dataset_trajectories(
    area: SafetyArea,
    dt: float,
    duration: float,
    pattern: str,
    minimum_distance: float,
    maximum_distance: float | None = None,
    camera_horizontal_fov: float = 90.0,
    camera_vertical_fov: float = 60.0,
    relative_heading_turns: float = 1.5,
    margin: float = 0.5,
    vertical_margin: float | None = None,
    travel_distance: float | None = None,
    observer_path: str = "straight",
    circle_radius: float | None = None,
) -> tuple[list[tuple[float, float, float, float]], ...]:
    """Generate camera-dataset trajectories for an observer and target UAV.

    UAV 1 is the front-camera observer and follows either a straight line or a
    circle. UAV 2 remains in its moving camera frame while its range, bearing,
    elevation, and heading vary. The requested minimum distance is the 3D
    centre-to-centre distance and is reached exactly by the generated path.
    """
    if dt < 0.01:
        raise ConfigurationError("dt must be at least 0.01 s for the MRS MpcTracker")
    if duration < 4.0 * dt:
        raise ConfigurationError("duration must contain at least five trajectory samples")
    if minimum_distance <= 0.0:
        raise ConfigurationError("minimum distance must be positive")
    if maximum_distance is not None and maximum_distance <= minimum_distance:
        raise ConfigurationError("maximum distance must be greater than minimum distance")
    if not 0.0 < camera_horizontal_fov < 180.0:
        raise ConfigurationError("camera horizontal FOV must be between 0 and 180 degrees")
    if not 0.0 < camera_vertical_fov < 180.0:
        raise ConfigurationError("camera vertical FOV must be between 0 and 180 degrees")
    if relative_heading_turns <= 0.0:
        raise ConfigurationError("relative heading turns must be positive")
    vertical_margin = _vertical_margin(margin, vertical_margin)
    if travel_distance is not None and travel_distance <= 0.0:
        raise ConfigurationError("travel distance must be positive")
    if circle_radius is not None and circle_radius <= 0.0:
        raise ConfigurationError("circle radius must be positive")
    if circle_radius is not None and observer_path != "circle":
        raise ConfigurationError("circle radius requires observer_path='circle'")
    if circle_radius is not None and travel_distance is not None:
        raise ConfigurationError("set either travel distance or circle radius, not both")
    if observer_path not in {"straight", "circle"}:
        raise ConfigurationError(
            f"unknown dataset observer path '{observer_path}'; expected straight or circle"
        )

    safe_center, clearance = safest_point(area.polygon)
    if clearance <= margin:
        raise ConfigurationError("safety area is smaller than the requested boundary margin")
    altitude = (area.min_z + area.max_z) / 2.0
    vertical_room = (area.max_z - area.min_z) / 2.0 - vertical_margin
    if vertical_room <= 0.0:
        raise ConfigurationError("vertical safety area is smaller than twice the margin")

    count = math.floor(duration / dt + 1e-9) + 1
    progresses = [index / (count - 1) for index in range(count)]
    bearing_amplitude = math.radians(camera_horizontal_fov) * 0.4
    requested_maximum = (
        maximum_distance
        if maximum_distance is not None
        else minimum_distance * DEFAULT_DATASET_MAXIMUM_DISTANCE_FACTOR
    )

    def layout(
        selected_maximum: float, travel_distance: float
    ) -> tuple[
        tuple[list[tuple[float, float, float, float]], ...], bool
    ]:
        maximum_height_angle = math.asin(min(1.0, vertical_room / selected_maximum))
        elevation_amplitude = min(
            math.radians(camera_vertical_fov) * 0.4,
            maximum_height_angle * 0.9,
        )
        observer_local: list[tuple[float, float, float, float]] = []
        target_local: list[tuple[float, float, float, float]] = []
        for progress in progresses:
            if observer_path == "circle":
                path_angle = 2.0 * math.pi * progress
                circle_radius = travel_distance / (2.0 * math.pi)
                observer_x = circle_radius * math.cos(path_angle)
                observer_y = circle_radius * math.sin(path_angle)
                observer_heading = _wrap_angle(path_angle + math.pi / 2.0)
            else:
                smooth_progress = (1.0 - math.cos(math.pi * progress)) / 2.0
                observer_x = travel_distance * (smooth_progress - 0.5)
                observer_y = 0.0
                observer_heading = 0.0
            distance, bearing, elevation, relative_target_heading = _dataset_relative_pose(
                pattern,
                progress,
                minimum_distance,
                selected_maximum,
                bearing_amplitude,
                elevation_amplitude,
                relative_heading_turns,
            )
            horizontal_distance = distance * math.cos(elevation)
            camera_x = horizontal_distance * math.cos(bearing)
            camera_y = horizontal_distance * math.sin(bearing)
            relative_z = distance * math.sin(elevation)
            heading_cos = math.cos(observer_heading)
            heading_sin = math.sin(observer_heading)
            relative_x = camera_x * heading_cos - camera_y * heading_sin
            relative_y = camera_x * heading_sin + camera_y * heading_cos
            observer_local.append(
                (observer_x, observer_y, altitude, observer_heading)
            )
            target_local.append(
                (
                    observer_x + relative_x,
                    observer_y + relative_y,
                    altitude + relative_z,
                    _wrap_angle(observer_heading + relative_target_heading),
                )
            )

        all_points = observer_local + target_local
        cloud_center_x = (
            min(point[0] for point in all_points) + max(point[0] for point in all_points)
        ) / 2.0
        cloud_center_y = (
            min(point[1] for point in all_points) + max(point[1] for point in all_points)
        ) / 2.0
        shift_x = safe_center[0] - cloud_center_x
        shift_y = safe_center[1] - cloud_center_y
        observer = [
            (x + shift_x, y + shift_y, z, heading)
            for x, y, z, heading in observer_local
        ]
        target = [
            (x + shift_x, y + shift_y, z, heading)
            for x, y, z, heading in target_local
        ]
        fits = all(
            _clearance((x, y), area.polygon) + 1e-9 >= margin
            and area.min_z + vertical_margin <= z <= area.max_z - vertical_margin
            for trajectory in (observer, target)
            for x, y, z, _ in trajectory
        )
        return (observer, target), fits

    _, formation_fits = layout(requested_maximum, 0.0)
    selected_maximum = requested_maximum
    if not formation_fits:
        if maximum_distance is not None:
            raise ConfigurationError(
                f"maximum distance {maximum_distance:.2f} m does not fit in this safety area"
            )
        minimum_diverse_range = minimum_distance * 1.05
        _, minimum_formation_fits = layout(minimum_diverse_range, 0.0)
        if not minimum_formation_fits:
            raise ConfigurationError(
                f"minimum distance {minimum_distance:.2f} m and camera motion do not fit "
                "in this safety area"
            )
        low, high = minimum_diverse_range, requested_maximum
        for _ in range(32):
            candidate = (low + high) / 2.0
            if layout(candidate, 0.0)[1]:
                low = candidate
            else:
                high = candidate
        selected_maximum = low

    if circle_radius is not None:
        requested_circle_travel = 2.0 * math.pi * circle_radius
        if not layout(selected_maximum, requested_circle_travel)[1]:
            raise ConfigurationError(
                f"circle radius {circle_radius:.2f} m does not fit in this safety area "
                "with the selected target motion"
            )
        fitted_travel = requested_circle_travel
    else:
        desired_travel = (
            travel_distance
            if travel_distance is not None
            else DEFAULT_DATASET_TRAVEL_DISTANCE
        )
        maximum_travel = min(desired_travel, 2.0 * max(0.0, clearance - margin))
        if layout(selected_maximum, maximum_travel)[1]:
            fitted_travel = maximum_travel
        else:
            low, high = 0.0, maximum_travel
            for _ in range(32):
                candidate = (low + high) / 2.0
                if layout(selected_maximum, candidate)[1]:
                    low = candidate
                else:
                    high = candidate
            fitted_travel = low
    if fitted_travel < 0.25:
        raise ConfigurationError(
            "the safety area cannot fit at least 0.25 m of observer motion with this formation"
        )

    trajectories, fits = layout(selected_maximum, fitted_travel)
    if not fits:
        raise RuntimeError("internal error: dataset trajectories lie outside the safety area")
    observer, target = trajectories
    target = _shift_target_to_minimum_separation(
        area, observer, target, minimum_distance, margin, vertical_margin
    )
    distances = [
        math.sqrt(
            (target_point[0] - observer_point[0]) ** 2
            + (target_point[1] - observer_point[1]) ** 2
            + (target_point[2] - observer_point[2]) ** 2
        )
        for observer_point, target_point in zip(observer, target)
    ]
    if min(distances) < minimum_distance - 1e-8:
        raise RuntimeError("internal error: generated inter-UAV distance is too small")
    if _trajectory_length(observer) < 0.25:
        raise RuntimeError("internal error: observer trajectory is stationary")
    if _trajectory_length(target) < 0.25:
        raise RuntimeError("internal error: target trajectory is stationary")
    return observer, target


def generate_dataset_weave(
    area: SafetyArea, dt: float, duration: float, minimum_distance: float, **kwargs: Any
) -> tuple[list[tuple[float, float, float, float]], ...]:
    """Generate lateral/vertical weaving target motion for image diversity."""
    return generate_dataset_trajectories(
        area, dt, duration, "weave", minimum_distance, **kwargs
    )


def generate_dataset_orbit(
    area: SafetyArea, dt: float, duration: float, minimum_distance: float, **kwargs: Any
) -> tuple[list[tuple[float, float, float, float]], ...]:
    """Generate an elliptical target orbit in the observer's image plane."""
    return generate_dataset_trajectories(
        area, dt, duration, "orbit", minimum_distance, **kwargs
    )


def generate_dataset_lissajous(
    area: SafetyArea, dt: float, duration: float, minimum_distance: float, **kwargs: Any
) -> tuple[list[tuple[float, float, float, float]], ...]:
    """Generate a Lissajous target pattern covering varied image positions."""
    return generate_dataset_trajectories(
        area, dt, duration, "lissajous", minimum_distance, **kwargs
    )


def _closest_point_on_segment(
    point: tuple[float, float],
    start: tuple[float, float],
    end: tuple[float, float],
) -> tuple[float, float]:
    px, py = point
    ax, ay = start
    dx, dy = end[0] - ax, end[1] - ay
    length_squared = dx * dx + dy * dy
    if length_squared == 0.0:
        return start
    fraction = max(
        0.0,
        min(1.0, ((px - ax) * dx + (py - ay) * dy) / length_squared),
    )
    return ax + fraction * dx, ay + fraction * dy


def _sample_cubic_bspline(
    waypoints: Sequence[tuple[float, float, float]], count: int
) -> list[tuple[float, float, float]]:
    """Sample a clamped uniform cubic B-spline through randomized controls."""
    controls = [waypoints[0], waypoints[0], *waypoints, waypoints[-1], waypoints[-1]]
    segment_count = len(controls) - 3
    result: list[tuple[float, float, float]] = []
    for sample in range(count):
        scaled = (sample / (count - 1)) * segment_count
        if scaled >= segment_count:
            segment = segment_count - 1
            u = 1.0
        else:
            segment = int(scaled)
            u = scaled - segment
        weights = (
            (1.0 - u) ** 3 / 6.0,
            (3.0 * u**3 - 6.0 * u**2 + 4.0) / 6.0,
            (-3.0 * u**3 + 3.0 * u**2 + 3.0 * u + 1.0) / 6.0,
            u**3 / 6.0,
        )
        result.append(
            tuple(
                sum(weights[index] * controls[segment + index][axis] for index in range(4))
                for axis in range(3)
            )
        )
    return result


def generate_static_camera_random_walk(
    area: SafetyArea,
    dt: float,
    duration: float,
    minimum_distance: float,
    maximum_distance: float | None = None,
    camera_horizontal_fov: float = 90.0,
    camera_vertical_fov: float = 60.0,
    margin: float = 0.5,
    vertical_margin: float | None = None,
    random_seed: int = 1,
    random_waypoints: int = 20,
    camera_placement: str = "circle",
    camera_circle_clearance: float = 7.5,
    camera_circle_point: str = "north",
    camera_edge_inset: float = 5.0,
    camera_heading: float | None = None,
    rrt_visualization: RRTVisualization | None = None,
) -> tuple[list[tuple[float, float, float, float]], ...]:
    """Generate a static camera and a smooth, non-looping random target path.

    The target is a clamped cubic B-spline through randomized, spatially
    separated control points. All samples stay in the camera frustum and safety
    area. A fixed seed makes a dataset exactly reproducible.
    """
    if dt < 0.01:
        raise ConfigurationError("dt must be at least 0.01 s for the MRS MpcTracker")
    if duration < 4.0 * dt:
        raise ConfigurationError("duration must contain at least five trajectory samples")
    if minimum_distance <= 0.0:
        raise ConfigurationError("minimum distance must be positive")
    selected_maximum = maximum_distance or (
        minimum_distance * DEFAULT_DATASET_MAXIMUM_DISTANCE_FACTOR
    )
    if selected_maximum <= minimum_distance:
        raise ConfigurationError("maximum distance must be greater than minimum distance")
    if not 0.0 < camera_horizontal_fov < 180.0:
        raise ConfigurationError("camera horizontal FOV must be between 0 and 180 degrees")
    if not 0.0 < camera_vertical_fov < 180.0:
        raise ConfigurationError("camera vertical FOV must be between 0 and 180 degrees")
    vertical_margin = _vertical_margin(margin, vertical_margin)
    if random_waypoints < 4:
        raise ConfigurationError("random walk needs at least four waypoints")
    if camera_placement not in {"circle", "edge"}:
        raise ConfigurationError("static camera placement must be 'circle' or 'edge'")
    if camera_circle_point not in {
        "north", "northeast", "east", "southeast",
        "south", "southwest", "west", "northwest",
    }:
        raise ConfigurationError("unknown static camera circle point")
    if camera_circle_clearance < margin:
        raise ConfigurationError("circle clearance must be at least the boundary margin")
    if camera_edge_inset < margin:
        raise ConfigurationError("static camera inset must be at least the boundary margin")

    safe_center, clearance = safest_point(area.polygon)
    if clearance <= margin:
        raise ConfigurationError("safety area is smaller than the requested boundary margin")
    if (area.max_z - area.min_z) / 2.0 <= vertical_margin:
        raise ConfigurationError("vertical safety area is smaller than twice the vertical margin")
    if camera_placement == "circle":
        if camera_circle_clearance >= clearance:
            raise ConfigurationError(
                f"circle clearance {camera_circle_clearance:.2f} m does not fit; "
                f"maximum inscribed-circle clearance is {clearance:.2f} m"
            )
        camera_radius = clearance - camera_circle_clearance
        circle_directions = {
            "north": (0.0, 1.0),
            "northeast": (math.sqrt(0.5), math.sqrt(0.5)),
            "east": (1.0, 0.0),
            "southeast": (math.sqrt(0.5), -math.sqrt(0.5)),
            "south": (0.0, -1.0),
            "southwest": (-math.sqrt(0.5), -math.sqrt(0.5)),
            "west": (-1.0, 0.0),
            "northwest": (-math.sqrt(0.5), math.sqrt(0.5)),
        }
        direction_x, direction_y = circle_directions[camera_circle_point]
        observer_xy = (
            safe_center[0] + camera_radius * direction_x,
            safe_center[1] + camera_radius * direction_y,
        )
        required_clearance = camera_circle_clearance
    else:
        boundary_candidates = [
            _closest_point_on_segment(safe_center, area.polygon[index - 1], area.polygon[index])
            for index in range(len(area.polygon))
        ]
        boundary = min(boundary_candidates, key=lambda point: math.dist(point, safe_center))
        inward_x = safe_center[0] - boundary[0]
        inward_y = safe_center[1] - boundary[1]
        inward_length = math.hypot(inward_x, inward_y)
        if inward_length == 0.0:
            raise ConfigurationError("could not orient the edge camera toward the safety area")
        inward_x /= inward_length
        inward_y /= inward_length
        if camera_edge_inset >= clearance:
            raise ConfigurationError(
                f"static camera inset {camera_edge_inset:.2f} m does not fit; "
                f"the available edge-to-interior clearance is {clearance:.2f} m"
            )
        observer_xy = (
            boundary[0] + inward_x * camera_edge_inset,
            boundary[1] + inward_y * camera_edge_inset,
        )
        required_clearance = margin
    if _clearance(observer_xy, area.polygon) + 1e-9 < required_clearance:
        raise ConfigurationError(
            "static camera placement does not satisfy its requested boundary clearance"
        )

    altitude = (area.min_z + area.max_z) / 2.0
    observer_heading = (
        _wrap_angle(camera_heading)
        if camera_heading is not None
        else math.atan2(
            safe_center[1] - observer_xy[1], safe_center[0] - observer_xy[0]
        )
    )
    forward = (math.cos(observer_heading), math.sin(observer_heading))
    left = (-forward[1], forward[0])
    horizontal_limit = math.radians(camera_horizontal_fov) * 0.4
    vertical_limit = math.radians(camera_vertical_fov) * 0.4
    count = math.floor(duration / dt + 1e-9) + 1
    rng = random.Random(random_seed)

    def to_world(distance: float, bearing: float, elevation: float) -> tuple[float, float, float]:
        horizontal = distance * math.cos(elevation)
        camera_forward = horizontal * math.cos(bearing)
        camera_left = horizontal * math.sin(bearing)
        return (
            observer_xy[0] + camera_forward * forward[0] + camera_left * left[0],
            observer_xy[1] + camera_forward * forward[1] + camera_left * left[1],
            altitude + distance * math.sin(elevation),
        )

    def valid_position(point: tuple[float, float, float]) -> bool:
        dx, dy, dz = (
            point[0] - observer_xy[0],
            point[1] - observer_xy[1],
            point[2] - altitude,
        )
        camera_forward = dx * forward[0] + dy * forward[1]
        camera_left = dx * left[0] + dy * left[1]
        horizontal = math.hypot(dx, dy)
        distance = math.sqrt(horizontal * horizontal + dz * dz)
        return (
            _clearance(point[:2], area.polygon) + 1e-9 >= margin
            and area.min_z + vertical_margin <= point[2] <= area.max_z - vertical_margin
            and minimum_distance - 1e-8 <= distance <= selected_maximum + 1e-8
            and camera_forward > 0.0
            and abs(math.atan2(camera_left, camera_forward)) <= horizontal_limit + 1e-9
            and abs(math.atan2(dz, horizontal)) <= vertical_limit + 1e-9
        )

    def random_visible_candidates(
        attempts: int = 300,
    ) -> list[tuple[float, float, float]]:
        """Sample the entire permitted range, retaining only visible positions."""
        pool: list[tuple[float, float, float]] = []
        for _ in range(attempts):
            bearing = rng.uniform(-horizontal_limit, horizontal_limit)
            elevation = rng.uniform(-vertical_limit, vertical_limit)
            # A slanted ray needs a larger ray distance to respect the requested
            # centre-to-centre minimum distance along the camera forward axis.
            direction_forward = math.cos(bearing) * math.cos(elevation)
            lower_distance = max(
                minimum_distance / max(direction_forward, 1e-6), minimum_distance
            )
            if lower_distance >= selected_maximum:
                continue
            candidate = to_world(
                rng.uniform(lower_distance, selected_maximum), bearing, elevation
            )
            if valid_position(candidate):
                pool.append(candidate)
        return pool

    def observer_distance(point: tuple[float, float, float]) -> float:
        return math.dist(point, (observer_xy[0], observer_xy[1], altitude))

    def edge_is_valid(
        start: tuple[float, float, float], end: tuple[float, float, float]
    ) -> bool:
        """Check the local RRT edge, not merely its endpoint."""
        return all(
            valid_position(
                tuple(start[axis] + fraction * (end[axis] - start[axis]) for axis in range(3))
            )
            for fraction in (0.2, 0.4, 0.6, 0.8, 1.0)
        )

    target_positions: list[tuple[float, float, float]] | None = None
    rrt_step = max(1.0, 0.45 * (selected_maximum - minimum_distance))
    rrt_min_spacing = max(0.25, 0.15 * rrt_step)
    for _ in range(30):
        initial_pool = random_visible_candidates()
        if not initial_pool:
            continue
        tree = [min(initial_pool, key=observer_distance)]
        parents = [-1]
        depths = [1]
        # Sample from the entire visible volume, extend the nearest tree node
        # toward each sample, and retain only collision/FOV-valid edges.
        for _ in range(random_waypoints * 120):
            samples = random_visible_candidates(attempts=1)
            if not samples:
                continue
            sample = samples[0]
            parent_index = min(
                range(len(tree)), key=lambda index: math.dist(tree[index], sample)
            )
            parent = tree[parent_index]
            distance = math.dist(parent, sample)
            if distance < rrt_min_spacing:
                continue
            fraction = min(1.0, rrt_step / distance)
            candidate = tuple(
                parent[axis] + fraction * (sample[axis] - parent[axis])
                for axis in range(3)
            )
            if (
                min(math.dist(candidate, node) for node in tree) < rrt_min_spacing
                or not edge_is_valid(parent, candidate)
            ):
                continue
            tree.append(candidate)
            parents.append(parent_index)
            depths.append(depths[parent_index] + 1)

        # Use the RRT nodes as a space-filling frontier, rather than following
        # one tree branch (which can alternate between two directions). Start
        # from an intermediate range, visit the closest reachable sample later,
        # and finish at the farthest reachable sample.
        nearest_index = min(range(len(tree)), key=lambda index: observer_distance(tree[index]))
        farthest_index = max(range(len(tree)), key=lambda index: observer_distance(tree[index]))
        node_distances = [observer_distance(point) for point in tree]
        intermediate_range = (node_distances[nearest_index] + node_distances[farthest_index]) / 2.0
        start_index = min(
            (
                index
                for index in range(len(tree))
                if index not in {nearest_index, farthest_index}
            ),
            key=lambda index: abs(node_distances[index] - intermediate_range),
            default=nearest_index,
        )
        waypoints = [tree[start_index]]
        available = [
            index for index in range(len(tree))
            if index not in {start_index, nearest_index, farthest_index}
        ]
        nearest_control_index = random_waypoints // 2
        nearest_control_repetitions = min(3, random_waypoints - 2)
        while available and len(waypoints) < random_waypoints - 1:
            if len(waypoints) == nearest_control_index:
                # Repeating an interior cubic-B-spline control creates the
                # required knot multiplicity for the curve to reach the close
                # approach, rather than merely being pulled toward it.
                waypoints.extend(
                    [tree[nearest_index]]
                    * min(nearest_control_repetitions, random_waypoints - 1 - len(waypoints))
                )
                continue
            current = waypoints[-1]
            candidates = [
                index for index in available if edge_is_valid(current, tree[index])
            ]
            if not candidates:
                break

            def frontier_score(index: int) -> float:
                candidate = tree[index]
                step = math.dist(current, candidate)
                spatial_novelty = min(
                    math.dist(candidate, previous) for previous in waypoints
                ) / selected_maximum
                step_score = min(1.0, step / rrt_step)
                direction_score = 1.0
                turn_score = 0.0
                if len(waypoints) >= 2:
                    previous = waypoints[-2]
                    incoming = tuple(current[axis] - previous[axis] for axis in range(3))
                    outgoing = tuple(candidate[axis] - current[axis] for axis in range(3))
                    incoming_length = math.sqrt(sum(value * value for value in incoming))
                    if incoming_length > 1e-9 and step > 1e-9:
                        cosine = sum(
                            incoming[axis] * outgoing[axis] for axis in range(3)
                        ) / (incoming_length * step)
                        direction_score = (cosine + 1.0) / 2.0
                        # Favor a lateral direction for intentional tight turns;
                        # perpendicular motion scores 1 while a reversal or
                        # straight continuation scores 0.
                        turn_score = 1.0 - abs(cosine)
                if len(waypoints) % 4 == 0:
                    # Every fourth control deliberately seeks a shorter lateral
                    # step. The spatial-novelty term still prevents a loop.
                    near_step_score = 1.0 - min(1.0, step / (0.55 * rrt_step))
                    return (
                        0.35 * spatial_novelty
                        + 0.20 * step_score
                        + 0.25 * turn_score
                        + 0.20 * near_step_score
                    )
                return 0.50 * spatial_novelty + 0.30 * step_score + 0.20 * direction_score

            selected = max(candidates, key=frontier_score)
            waypoints.append(tree[selected])
            available.remove(selected)
        if len(waypoints) != random_waypoints - 1:
            continue
        waypoints.append(tree[farthest_index])
        candidate_curve = _sample_cubic_bspline(waypoints, count)
        if all(valid_position(point) for point in candidate_curve):
            target_positions = candidate_curve
            if rrt_visualization is not None:
                rrt_visualization.points = tree.copy()
                rrt_visualization.spline_controls = waypoints.copy()
                rrt_visualization.observer_xy = observer_xy
                rrt_visualization.observer_heading = observer_heading
                rrt_visualization.horizontal_fov = camera_horizontal_fov
                rrt_visualization.minimum_distance = minimum_distance
                rrt_visualization.maximum_distance = selected_maximum
            break
    if target_positions is None:
        # A narrow moving-camera frustum can reject every smoothed RRT branch.
        # Retain the established diverse sampler as a safe fallback rather than
        # failing a trajectory that still has a valid visible path.
        for _ in range(30):
            initial_pool = random_visible_candidates()
            if not initial_pool:
                continue
            waypoints = [rng.choice(initial_pool)]
            for _ in range(1, random_waypoints):
                pool = random_visible_candidates()
                if not pool:
                    break

                def fallback_score(candidate: tuple[float, float, float]) -> float:
                    spatial_novelty = min(
                        math.dist(candidate, old) for old in waypoints
                    ) / selected_maximum
                    step_novelty = math.dist(candidate, waypoints[-1]) / selected_maximum
                    candidate_range = math.dist(
                        candidate, (observer_xy[0], observer_xy[1], altitude)
                    )
                    range_novelty = min(
                        abs(
                            candidate_range
                            - math.dist(old, (observer_xy[0], observer_xy[1], altitude))
                        )
                        for old in waypoints
                    ) / (selected_maximum - minimum_distance)
                    return (
                        0.35 * spatial_novelty
                        + 0.40 * step_novelty
                        + 0.25 * range_novelty
                        + rng.uniform(0.0, 0.05)
                    )

                waypoints.append(max(pool, key=fallback_score))
            if len(waypoints) != random_waypoints:
                continue
            candidate_curve = _sample_cubic_bspline(waypoints, count)
            if all(valid_position(point) for point in candidate_curve):
                target_positions = candidate_curve
                break
    if target_positions is None:
        raise ConfigurationError(
            "could not fit a smooth random walk in the visible safety area; "
            "reduce minimum distance, maximum distance, margin, or waypoint count"
        )

    observer = [
        (observer_xy[0], observer_xy[1], altitude, observer_heading)
        for _ in range(count)
    ]
    # The target attitude follows its own smooth random walk rather than its
    # velocity vector. Using unwrapped controls avoids +/-pi discontinuities;
    # headings are wrapped only in the final MRS trajectory samples.
    # Keep attitude variation independent of how many RRT samples were needed
    # to explore the position space.
    heading_rng = random.Random(random_seed)
    heading_controls = [heading_rng.uniform(-math.pi, math.pi)]
    for _ in range(random_waypoints - 1):
        heading_controls.append(
            heading_controls[-1] + heading_rng.uniform(-0.9 * math.pi, 0.9 * math.pi)
        )
    heading_samples = _sample_cubic_bspline(
        [(heading, 0.0, 0.0) for heading in heading_controls], count
    )
    target = [
        (*point, _wrap_angle(heading_sample[0]))
        for point, heading_sample in zip(target_positions, heading_samples)
    ]
    pre_normalization_minimum = min(
        math.dist(observer_point[:3], target_point[:3])
        for observer_point, target_point in zip(observer, target)
    )
    target = _shift_target_to_minimum_separation(
        area, observer, target, minimum_distance, margin, vertical_margin
    )
    if (
        rrt_visualization is not None
        and rrt_visualization.observer_xy is not None
        and pre_normalization_minimum > 0.0
    ):
        normalization_scale = minimum_distance / pre_normalization_minimum

        def normalize_planning_point(
            point: tuple[float, float, float]
        ) -> tuple[float, float, float]:
            return (
                observer_xy[0] + normalization_scale * (point[0] - observer_xy[0]),
                observer_xy[1] + normalization_scale * (point[1] - observer_xy[1]),
                altitude + normalization_scale * (point[2] - altitude),
            )

        if rrt_visualization.points is not None:
            rrt_visualization.points = [
                normalize_planning_point(point) for point in rrt_visualization.points
            ]
        if rrt_visualization.spline_controls is not None:
            rrt_visualization.spline_controls = [
                normalize_planning_point(point)
                for point in rrt_visualization.spline_controls
            ]
        if rrt_visualization.maximum_distance is not None:
            rrt_visualization.maximum_distance *= normalization_scale
    if _trajectory_length(target) < 0.25:
        raise RuntimeError("internal error: random target trajectory is stationary")
    return observer, target


def generate_moving_camera_random_walk(
    area: SafetyArea,
    dt: float,
    duration: float,
    minimum_distance: float,
    maximum_distance: float | None = None,
    camera_horizontal_fov: float = 90.0,
    camera_vertical_fov: float = 60.0,
    margin: float = 0.5,
    vertical_margin: float | None = None,
    random_seed: int = 1,
    random_waypoints: int = 20,
    camera_motion_radius: float = 1.0,
    camera_heading_walk_limit: float = math.radians(12.0),
    camera_placement: str = "circle",
    camera_circle_clearance: float = 7.5,
    camera_circle_point: str = "north",
    camera_edge_inset: float = 5.0,
    camera_heading: float | None = None,
    rrt_visualization: RRTVisualization | None = None,
) -> tuple[list[tuple[float, float, float, float]], ...]:
    """Generate a moving camera after planning a target path in its initial FOV.

    The observer then follows an independent small B-spline random walk. Later
    target samples may leave the moved camera's FOV, while safety and distance
    bounds remain valid.
    """
    vertical_margin = _vertical_margin(margin, vertical_margin)
    if camera_motion_radius <= 0.0:
        raise ConfigurationError("moving-camera radius must be positive")
    if not 0.0 < camera_heading_walk_limit < math.pi / 2.0:
        raise ConfigurationError("moving-camera heading walk limit must be between 0 and 90 degrees")
    selected_maximum = maximum_distance or (
        minimum_distance * DEFAULT_DATASET_MAXIMUM_DISTANCE_FACTOR
    )
    if selected_maximum - minimum_distance <= 2.0 * camera_motion_radius:
        raise ConfigurationError(
            "maximum distance must leave room for the moving-camera radius"
        )

    # Plan the target in the initial camera's complete requested FOV. Camera
    # movement is intentionally independent of later target visibility.
    static_observer, target = generate_static_camera_random_walk(
        area=area,
        dt=dt,
        duration=duration,
        minimum_distance=minimum_distance + camera_motion_radius,
        maximum_distance=selected_maximum - camera_motion_radius,
        camera_horizontal_fov=camera_horizontal_fov,
        camera_vertical_fov=camera_vertical_fov,
        margin=margin,
        vertical_margin=vertical_margin,
        random_seed=random_seed,
        random_waypoints=random_waypoints,
        camera_placement=camera_placement,
        camera_circle_clearance=camera_circle_clearance,
        camera_circle_point=camera_circle_point,
        camera_edge_inset=camera_edge_inset,
        camera_heading=camera_heading,
        rrt_visualization=rrt_visualization,
    )
    base = static_observer[0]
    count = len(target)
    heading = base[3]
    rng = random.Random(random_seed + 100_003)

    def path_is_valid(observer: Sequence[tuple[float, float, float, float]]) -> bool:
        for camera, target_point in zip(observer, target):
            dx = target_point[0] - camera[0]
            dy = target_point[1] - camera[1]
            dz = target_point[2] - camera[2]
            distance = math.sqrt(dx * dx + dy * dy + dz * dz)
            if not (
                _clearance(camera[:2], area.polygon) + 1e-9 >= margin
                and area.min_z + vertical_margin <= camera[2] <= area.max_z - vertical_margin
                and minimum_distance - 1e-8 <= distance <= selected_maximum + 1e-8
            ):
                return False
        return True

    for _ in range(30):
        controls = [(0.0, 0.0, 0.0)]
        for _ in range(random_waypoints - 1):
            radius = camera_motion_radius * math.sqrt(rng.random())
            angle = rng.uniform(-math.pi, math.pi)
            # Keep the filming UAV's motion deliberately smaller than the target.
            controls.append((radius * math.cos(angle), radius * math.sin(angle), 0.0))
        offsets = _sample_cubic_bspline(controls, count)
        # A bounded random walk gives the front camera changing viewpoints while
        # avoiding discontinuities around +/- pi. The cubic spline remains
        # within the control bounds before headings are wrapped for MRS output.
        heading_controls = [0.0]
        for _ in range(random_waypoints - 1):
            proposed = heading_controls[-1] + rng.uniform(
                -0.45 * camera_heading_walk_limit,
                0.45 * camera_heading_walk_limit,
            )
            heading_controls.append(
                max(-camera_heading_walk_limit, min(camera_heading_walk_limit, proposed))
            )
        heading_offsets = _sample_cubic_bspline(
            [(offset, 0.0, 0.0) for offset in heading_controls], count
        )
        observer = [
            (
                base[0] + offset[0],
                base[1] + offset[1],
                base[2],
                _wrap_angle(heading + heading_offset[0]),
            )
            for offset, heading_offset in zip(offsets, heading_offsets)
        ]
        if path_is_valid(observer) and _trajectory_length(observer) >= 0.1:
            pre_normalization_minimum = min(
                math.dist(observer_point[:3], target_point[:3])
                for observer_point, target_point in zip(observer, target)
            )
            shifted_target = _shift_target_to_minimum_separation(
                area, observer, target, minimum_distance, margin, vertical_margin
            )
            if rrt_visualization is not None:
                # Keep actual selected RRT controls visible. A cubic B-spline
                # is not an interpolating curve, so its interior does not pass
                # through those controls; they define the control polygon.
                rrt_visualization.observer_xy = observer[0][:2]
                rrt_visualization.observer_heading = observer[0][3]
                rrt_visualization.horizontal_fov = camera_horizontal_fov
                rrt_visualization.minimum_distance = minimum_distance
                moving_normalization_scale = minimum_distance / pre_normalization_minimum
                if rrt_visualization.points is not None:
                    rrt_visualization.points = [
                        (
                            observer[0][0]
                            + moving_normalization_scale * (point[0] - base[0]),
                            observer[0][1]
                            + moving_normalization_scale * (point[1] - base[1]),
                            observer[0][2]
                            + moving_normalization_scale * (point[2] - base[2]),
                        )
                        for point in rrt_visualization.points
                    ]
                if rrt_visualization.spline_controls is not None:
                    rrt_visualization.spline_controls = [
                        (
                            observer[0][0]
                            + moving_normalization_scale * (point[0] - base[0]),
                            observer[0][1]
                            + moving_normalization_scale * (point[1] - base[1]),
                            observer[0][2]
                            + moving_normalization_scale * (point[2] - base[2]),
                        )
                        for point in rrt_visualization.spline_controls
                    ]
                rrt_visualization.maximum_distance = (
                    (rrt_visualization.maximum_distance or selected_maximum)
                    * moving_normalization_scale
                )
            return observer, shifted_target
    raise ConfigurationError(
        "could not fit a moving camera random walk within safety and distance bounds; "
        "reduce --moving-camera-radius, increase distance, or use circle placement"
    )


def visualize_trajectories(
    area: SafetyArea,
    trajectories: Sequence[Sequence[tuple[float, float, float, float]]],
    dt: float,
    output_path: Path | None = None,
    show: bool = False,
    constraints: DynamicConstraints | None = None,
    rrt_visualization: RRTVisualization | None = None,
) -> Any:
    """Plot the trajectory map separately from velocity and other diagnostics.

    Matplotlib is imported lazily so trajectory generation can still be used in
    minimal or headless environments when no visualization is requested.
    """
    try:
        import matplotlib.pyplot as plt
        from matplotlib.patches import Circle, Wedge
    except ImportError as exc:
        raise ConfigurationError(
            "trajectory visualization requires Matplotlib; install requirements.txt"
        ) from exc

    trajectory_figure, path_axes = plt.subplots(figsize=(9, 8))
    diagnostics_figure, axes = plt.subplot_mosaic(
        [
            ["altitude", "distance"],
            ["relative", "relative"],
            ["velocity", "acceleration"],
            ["heading", "heading"],
        ],
        figsize=(14, 16),
    )
    altitude_axes = axes["altitude"]
    distance_axes = axes["distance"]
    relative_axes = axes["relative"]
    velocity_axes = axes["velocity"]
    acceleration_axes = axes["acceleration"]
    heading_axes = axes["heading"]
    closed_polygon = (*area.polygon, area.polygon[0])
    polygon_x = [point[0] for point in closed_polygon]
    polygon_y = [point[1] for point in closed_polygon]
    path_axes.fill(polygon_x, polygon_y, color="tab:green", alpha=0.12, label="Safety area")
    path_axes.plot(polygon_x, polygon_y, color="tab:green", linewidth=2)

    trajectory_xy = [
        point[:2]
        for trajectory in trajectories
        for point in trajectory
    ]
    if trajectory_xy:
        trajectory_min_x = min(point[0] for point in trajectory_xy)
        trajectory_max_x = max(point[0] for point in trajectory_xy)
        trajectory_min_y = min(point[1] for point in trajectory_xy)
        trajectory_max_y = max(point[1] for point in trajectory_xy)
        trajectory_span = max(
            trajectory_max_x - trajectory_min_x,
            trajectory_max_y - trajectory_min_y,
            1.0,
        )
    else:
        trajectory_min_x = min(polygon_x)
        trajectory_max_x = max(polygon_x)
        trajectory_min_y = min(polygon_y)
        trajectory_max_y = max(polygon_y)
        trajectory_span = max(trajectory_max_x - trajectory_min_x, trajectory_max_y - trajectory_min_y)

    colors = ("tab:blue", "tab:orange")
    for index, trajectory in enumerate(trajectories):
        if not trajectory:
            continue
        label = f"UAV {index + 1}"
        color = colors[index % len(colors)]
        x_values = [point[0] for point in trajectory]
        y_values = [point[1] for point in trajectory]
        z_values = [point[2] for point in trajectory]
        times = [sample * dt for sample in range(len(trajectory))]

        path_axes.plot(x_values, y_values, color=color, linewidth=2, label=label)
        path_axes.scatter(x_values[0], y_values[0], color=color, marker="o", s=60)
        path_axes.scatter(x_values[-1], y_values[-1], color=color, marker="X", s=70)
        arrow_step = max(1, len(trajectory) // 8)
        arrow_indices = range(0, len(trajectory), arrow_step)
        arrow_length = trajectory_span * 0.035
        path_axes.quiver(
            [x_values[sample] for sample in arrow_indices],
            [y_values[sample] for sample in arrow_indices],
            [arrow_length * math.cos(trajectory[sample][3]) for sample in arrow_indices],
            [arrow_length * math.sin(trajectory[sample][3]) for sample in arrow_indices],
            angles="xy",
            scale_units="xy",
            scale=1.0,
            color=color,
            alpha=0.65,
            width=0.004,
        )
        line_style = "-" if index % 2 == 0 else "--"
        altitude_axes.plot(
            times, z_values, color=color, linestyle=line_style, linewidth=2, label=label
        )
        # Unwrap before plotting so a genuine smooth turn is not displayed as a
        # vertical jump when the MRS heading crosses +/- pi.
        heading_axes.plot(
            times,
            [math.degrees(value) for value in _unwrap_headings([point[3] for point in trajectory])],
            color=color,
            linestyle=line_style,
            linewidth=2,
            label=label,
        )
        velocities = [
            tuple((current[axis] - previous[axis]) / dt for axis in range(3))
            for previous, current in zip(trajectory, trajectory[1:])
        ]
        velocity_magnitudes = [
            math.sqrt(sum(component * component for component in velocity))
            for velocity in velocities
        ]
        velocity_times = [sample * dt for sample in range(1, len(trajectory))]
        velocity_axes.plot(
            velocity_times,
            velocity_magnitudes,
            color=color,
            linestyle=line_style,
            linewidth=2,
            label=label,
        )
        if constraints is not None:
            # The MRS profile limits horizontal and vertical velocity separately.
            # This is the largest valid 3D magnitude for each sample's vertical
            # direction; full component-wise validation remains authoritative.
            speed_envelope = [
                math.hypot(
                    constraints.horizontal["speed"],
                    (
                        constraints.vertical_ascending["speed"]
                        if velocity[2] >= 0.0
                        else constraints.vertical_descending["speed"]
                    ),
                )
                for velocity in velocities
            ]
            velocity_axes.plot(
                velocity_times,
                speed_envelope,
                color="black",
                linestyle="--",
                linewidth=1.5,
                label=f"UAV {index + 1} combined speed limit",
            )
        accelerations = [
            tuple((current[axis] - previous[axis]) / dt for axis in range(3))
            for previous, current in zip(velocities, velocities[1:])
        ]
        acceleration_magnitudes = [
            math.sqrt(sum(component * component for component in acceleration))
            for acceleration in accelerations
        ]
        acceleration_times = [sample * dt for sample in range(2, len(trajectory))]
        acceleration_axes.plot(
            acceleration_times,
            acceleration_magnitudes,
            color=color,
            linestyle=line_style,
            linewidth=2,
            label=label,
        )

    if (
        rrt_visualization is not None
        and rrt_visualization.points
        and rrt_visualization.spline_controls
        and rrt_visualization.observer_xy is not None
        and rrt_visualization.observer_heading is not None
        and rrt_visualization.horizontal_fov is not None
        and rrt_visualization.minimum_distance is not None
        and rrt_visualization.maximum_distance is not None
    ):
        observer_x, observer_y = rrt_visualization.observer_xy
        max_distance = rrt_visualization.maximum_distance
        heading_degrees = math.degrees(rrt_visualization.observer_heading)
        half_fov_degrees = rrt_visualization.horizontal_fov / 2.0
        path_axes.add_patch(
            Wedge(
                (observer_x, observer_y),
                max_distance,
                heading_degrees - half_fov_degrees,
                heading_degrees + half_fov_degrees,
                color="tab:purple",
                alpha=0.10,
                label="planning FOV",
            )
        )
        for radius, style, label in (
            (rrt_visualization.minimum_distance, ":", "minimum target distance"),
            (max_distance, "--", "maximum target distance"),
        ):
            path_axes.add_patch(
                Circle(
                    (observer_x, observer_y),
                    radius,
                    fill=False,
                    color="tab:purple",
                    linestyle=style,
                    linewidth=1.2,
                    alpha=0.8,
                    label=label,
                )
            )
        path_axes.scatter(
            [point[0] for point in rrt_visualization.points],
            [point[1] for point in rrt_visualization.points],
            color="tab:gray",
            s=10,
            alpha=0.5,
            label="RRT samples",
            zorder=2,
        )
        path_axes.scatter(
            [point[0] for point in rrt_visualization.spline_controls],
            [point[1] for point in rrt_visualization.spline_controls],
            color="tab:red",
            edgecolor="white",
            linewidth=0.5,
            marker="D",
            s=42,
            label="RRT spline controls",
            zorder=4,
        )
        path_axes.plot(
            [point[0] for point in rrt_visualization.spline_controls],
            [point[1] for point in rrt_visualization.spline_controls],
            color="tab:red",
            linestyle=":",
            linewidth=1,
            alpha=0.7,
            label="RRT control polygon",
            zorder=3,
        )

    path_axes.set_title("Horizontal trajectory")
    path_axes.set_xlabel(f"x [m] ({area.frame_id})")
    path_axes.set_ylabel("y [m]")
    trajectory_padding = max(0.5, trajectory_span * 0.08)
    path_axes.set_xlim(trajectory_min_x - trajectory_padding, trajectory_max_x + trajectory_padding)
    path_axes.set_ylim(trajectory_min_y - trajectory_padding, trajectory_max_y + trajectory_padding)
    path_axes.set_aspect("equal", adjustable="box")
    path_axes.grid(True, alpha=0.3)
    path_axes.legend()

    altitude_axes.axhspan(area.min_z, area.max_z, color="tab:green", alpha=0.12)
    altitude_axes.axhline(area.min_z, color="tab:green", linestyle="--", linewidth=1)
    altitude_axes.axhline(area.max_z, color="tab:green", linestyle="--", linewidth=1)
    altitude_axes.set_title("Altitude profile")
    altitude_axes.set_xlabel("time [s]")
    altitude_axes.set_ylabel("z [m]")
    altitude_axes.grid(True, alpha=0.3)
    altitude_axes.legend()

    synchronized_count = min((len(trajectory) for trajectory in trajectories), default=0)
    if len(trajectories) >= 2 and synchronized_count:
        distance_times = [sample * dt for sample in range(synchronized_count)]
        distances = [
            math.dist(trajectories[0][sample][:3], trajectories[1][sample][:3])
            for sample in range(synchronized_count)
        ]
        minimum_index = min(range(synchronized_count), key=distances.__getitem__)
        minimum_distance = distances[minimum_index]
        distance_axes.plot(
            distance_times,
            distances,
            color="tab:purple",
            linewidth=2,
            label="UAV separation",
        )
        distance_axes.axhline(
            minimum_distance,
            color="tab:red",
            linestyle="--",
            linewidth=1.5,
            label=f"minimum = {minimum_distance:.2f} m",
        )
        distance_axes.scatter(
            distance_times[minimum_index],
            minimum_distance,
            color="tab:red",
            marker="o",
            s=45,
            zorder=3,
        )
        relative_velocity_components = [
            [
                (
                    (trajectories[1][sample][axis] - trajectories[1][sample - 1][axis])
                    - (trajectories[0][sample][axis] - trajectories[0][sample - 1][axis])
                ) / dt
                for sample in range(1, synchronized_count)
            ]
            for axis in range(3)
        ]
        for component, label, color in zip(
            relative_velocity_components,
            ("Δvx (east)", "Δvy (north)", "Δvz (up)"),
            ("tab:red", "tab:green", "tab:blue"),
        ):
            relative_axes.plot(
                distance_times[1:], component, color=color, linewidth=2, label=label
            )
        relative_velocity_magnitudes = [
            math.sqrt(sum(component[sample] ** 2 for component in relative_velocity_components))
            for sample in range(synchronized_count - 1)
        ]
        relative_axes.plot(
            distance_times[1:],
            relative_velocity_magnitudes,
            color="black",
            linestyle="--",
            linewidth=2,
            label="|v₂ − v₁|",
        )
    distance_axes.set_title("Inter-UAV distance")
    distance_axes.set_xlabel("time [s]")
    distance_axes.set_ylabel("distance [m]")
    distance_axes.grid(True, alpha=0.3)
    distance_axes.legend()

    relative_axes.axhline(0.0, color="black", linewidth=1, alpha=0.45)
    relative_axes.set_title("Relative velocity (UAV 2 − UAV 1)")
    relative_axes.set_xlabel("time [s]")
    relative_axes.set_ylabel("velocity [m/s]")
    relative_axes.grid(True, alpha=0.3)
    relative_axes.legend()

    velocity_axes.set_title("Velocity magnitude")
    velocity_axes.set_xlabel("time [s]")
    velocity_axes.set_ylabel("|v| [m/s]")
    velocity_axes.grid(True, alpha=0.3)
    velocity_axes.legend()

    acceleration_axes.set_title("Acceleration magnitude")
    acceleration_axes.set_xlabel("time [s]")
    acceleration_axes.set_ylabel("|a| [m/s²]")
    acceleration_axes.grid(True, alpha=0.3)
    acceleration_axes.legend()

    heading_axes.set_title("UAV heading")
    heading_axes.set_xlabel("time [s]")
    heading_axes.set_ylabel("heading [deg], unwrapped")
    heading_axes.grid(True, alpha=0.3)
    heading_axes.legend()

    trajectory_figure.suptitle(
        "Two-UAV trajectories (circle: start, cross: end, arrows: heading)"
    )
    trajectory_figure.tight_layout()
    diagnostics_figure.suptitle("Two-UAV trajectory diagnostics")
    diagnostics_figure.tight_layout()
    if output_path is not None:
        output_path.parent.mkdir(parents=True, exist_ok=True)
        trajectory_figure.savefig(output_path, dpi=160, bbox_inches="tight")
        diagnostics_path = output_path.with_name(
            f"{output_path.stem}_diagnostics{output_path.suffix}"
        )
        diagnostics_figure.savefig(diagnostics_path, dpi=160, bbox_inches="tight")
    if show:
        plt.show()
    # Keep the diagnostic figure reachable without changing the established
    # return type of this public helper.
    trajectory_figure.diagnostics_figure = diagnostics_figure
    return trajectory_figure


def _write_trajectory(path: Path, trajectory: Sequence[tuple[float, float, float, float]]) -> None:
    with path.open("w", encoding="utf-8", newline="") as output:
        writer = csv.writer(output, lineterminator="\n")
        for row in trajectory:
            writer.writerow(f"{value:.6f}" for value in row)


def _write_loader_config(path: Path, frame_id: str, dt: float) -> None:
    contents = (
        "trajectory:\n"
        f"  frame_id: {frame_id}\n"
        f"  dt: {dt:g}\n"
        "  loop: false\n"
        "  use_heading: true\n"
        "  fly_now: false\n"
        "  offset: [0.0, 0.0, 0.0, 0.0]\n"
    )
    path.write_text(contents, encoding="utf-8")


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Generate two simple x,y,z,heading trajectories inside an MRS safety area."
    )
    parser.add_argument("config", type=Path, help="MRS world/safety-area YAML config")
    parser.add_argument(
        "--platform",
        type=Path,
        help=(
            "MRS platform YAML (for example platforms/x500.yaml); validates "
            "profile compatibility and enables the bundled MRS constraints"
        ),
    )
    parser.add_argument("--output-dir", type=Path, default=Path("generated"))
    parser.add_argument("--dt", type=float, default=0.2, help="sample period in seconds")
    parser.add_argument(
        "--duration",
        type=float,
        default=500.0,
        help="trajectory duration in seconds (default: 500)",
    )
    parser.add_argument(
        "--pattern",
        choices=(
            "parallel",
            "straight-helix",
            "dataset-weave",
            "dataset-orbit",
            "dataset-lissajous",
            "dataset-random-walk-static",
            "dataset-random-walk-moving",
        ),
        default="parallel",
        help="trajectory pattern to generate",
    )
    parser.add_argument("--separation", type=float, default=2.0, help="distance between UAVs in metres")
    parser.add_argument(
        "--lead-distance",
        type=float,
        default=2.0,
        help="how far UAV 2 leads UAV 1 in the straight-helix pattern [m]",
    )
    parser.add_argument(
        "--helix-radius",
        type=float,
        default=0.75,
        help="radius of UAV 2's helix [m]",
    )
    parser.add_argument(
        "--helix-turns",
        type=float,
        default=2.0,
        help="number of helix turns made by UAV 2",
    )
    parser.add_argument(
        "--minimum-distance",
        type=float,
        default=DEFAULT_DATASET_MINIMUM_DISTANCE,
        help=(
            "minimum 3D inter-UAV distance for dataset patterns [m] "
            f"(default: {DEFAULT_DATASET_MINIMUM_DISTANCE:g}, sized for Temesvar field 1)"
        ),
    )
    parser.add_argument(
        "--maximum-distance",
        type=float,
        help=(
            "maximum dataset distance [m]; default adapts up to three times "
            "the minimum"
        ),
    )
    observer_size = parser.add_mutually_exclusive_group()
    observer_size.add_argument(
        "--travel-distance",
        type=float,
        help=(
            "desired path length of the recording UAV [m]; default: "
            f"{DEFAULT_DATASET_TRAVEL_DISTANCE:g} (shortened automatically to fit)"
        ),
    )
    observer_size.add_argument(
        "--circle-radius",
        type=float,
        help=(
            "exact recording-UAV circle radius [m]; requires --observer-path circle "
            "and errors if it cannot fit"
        ),
    )
    parser.add_argument(
        "--observer-path",
        choices=("straight", "circle"),
        default="straight",
        help=(
            "recording UAV path for dataset patterns; on a circle its heading "
            "is tangent to the path (default: straight)"
        ),
    )
    parser.add_argument(
        "--camera-horizontal-fov",
        type=float,
        default=90.0,
        help="front camera horizontal field of view [deg]",
    )
    parser.add_argument(
        "--camera-vertical-fov",
        type=float,
        default=60.0,
        help="front camera vertical field of view [deg]",
    )
    parser.add_argument(
        "--relative-heading-turns",
        type=float,
        default=1.5,
        help="target heading rotations during a dataset trajectory",
    )
    parser.add_argument(
        "--random-seed",
        type=int,
        default=1,
        help="reproducible seed for a random-walk dataset pattern (default: 1)",
    )
    parser.add_argument(
        "--random-waypoints",
        type=int,
        default=20,
        help="number of separated B-spline controls for a random walk (default: 20)",
    )
    parser.add_argument(
        "--static-camera-placement",
        choices=("circle", "edge"),
        default="circle",
        help="static camera placement method for the random walk (default: circle)",
    )
    parser.add_argument(
        "--static-camera-circle-clearance",
        type=float,
        default=7.5,
        help=(
            "clearance from the safety boundary for the maximum inscribed camera "
            "circle [m] (default: 7.5)"
        ),
    )
    parser.add_argument(
        "--static-camera-circle-point",
        choices=(
            "north", "northeast", "east", "southeast",
            "south", "southwest", "west", "northwest",
        ),
        default="north",
        help="compass point selected on the inscribed camera circle (default: north)",
    )
    parser.add_argument(
        "--static-camera-inset",
        type=float,
        default=5.0,
        help="edge-placement inset from the nearest field boundary [m] (default: 5)",
    )
    parser.add_argument(
        "--camera-heading",
        type=float,
        help=(
            "static camera look direction in world-frame degrees; default points toward "
            "the safety-area interior"
        ),
    )
    parser.add_argument(
        "--moving-camera-radius",
        type=float,
        default=1.0,
        help=(
            "maximum horizontal displacement of the filming UAV's small random walk "
            "[m] (default: 1)"
        ),
    )
    parser.add_argument(
        "--moving-camera-heading-walk",
        type=float,
        default=12.0,
        help=(
            "maximum heading deviation of the filming UAV's bounded random walk "
            "[deg] (default: 12)"
        ),
    )
    parser.add_argument(
        "--margin",
        type=float,
        default=0.5,
        help="legacy shared horizontal/vertical safety margin [m] (default: 0.5)",
    )
    parser.add_argument(
        "--horizontal-margin",
        type=float,
        help="minimum XY safety-polygon clearance [m]; overrides --margin",
    )
    parser.add_argument(
        "--vertical-margin",
        type=float,
        help="minimum clearance from min_z and max_z [m]; overrides --margin",
    )
    parser.add_argument(
        "--placement-direction",
        choices=tuple(_PLACEMENT_DIRECTIONS),
        default="center",
        help=(
            "translate the completed trajectories toward this world-frame border "
            "(east = +x, north = +y; default: center)"
        ),
    )
    parser.add_argument(
        "--boundary-offset",
        type=float,
        default=0.0,
        help=(
            "extra standoff from the selected boundary [m], in addition to --horizontal-margin "
            "(default: 0)"
        ),
    )
    parser.add_argument(
        "--constraints",
        type=Path,
        help="MRS ConstraintManager YAML file used to verify generated dynamics",
    )
    parser.add_argument(
        "--constraint-profile",
        default=None,
        help=(
            "named profile in the constraints file (default: fast; medium for "
            "dataset-random-walk-moving)"
        ),
    )
    parser.add_argument(
        "--plot",
        nargs="?",
        type=Path,
        const=Path("trajectories.png"),
        help="save a visualization, optionally with a filename (default: trajectories.png)",
    )
    parser.add_argument(
        "--plot-rrt",
        action="store_true",
        help="overlay RRT samples, spline controls, FOV, and range bounds on a random-walk plot",
    )
    parser.add_argument("--show-plot", action="store_true", help="display the visualization interactively")
    args = parser.parse_args(argv)
    if args.constraint_profile is None:
        args.constraint_profile = (
            "medium" if args.pattern == "dataset-random-walk-moving" else "fast"
        )
    if args.horizontal_margin is None:
        args.horizontal_margin = args.margin
    if args.vertical_margin is None:
        args.vertical_margin = args.margin
    return args


def _generate_from_args(
    area: SafetyArea,
    args: argparse.Namespace,
    duration: float,
    rrt_visualization: RRTVisualization | None = None,
) -> tuple[list[tuple[float, float, float, float]], ...]:
    """Generate the selected spatial path with a supplied duration."""
    if args.pattern == "straight-helix":
        return generate_straight_and_helix(
            area,
            args.dt,
            duration,
            args.lead_distance,
            args.helix_radius,
            args.helix_turns,
            args.horizontal_margin,
            args.vertical_margin,
        )
    if args.pattern == "dataset-random-walk-static":
        return generate_static_camera_random_walk(
            area=area,
            dt=args.dt,
            duration=duration,
            minimum_distance=args.minimum_distance,
            maximum_distance=args.maximum_distance,
            camera_horizontal_fov=args.camera_horizontal_fov,
            camera_vertical_fov=args.camera_vertical_fov,
            margin=args.horizontal_margin,
            vertical_margin=args.vertical_margin,
            random_seed=args.random_seed,
            random_waypoints=args.random_waypoints,
            camera_placement=args.static_camera_placement,
            camera_circle_clearance=args.static_camera_circle_clearance,
            camera_circle_point=args.static_camera_circle_point,
            camera_edge_inset=args.static_camera_inset,
            camera_heading=(
                math.radians(args.camera_heading)
                if args.camera_heading is not None
                else None
            ),
            rrt_visualization=rrt_visualization,
        )
    if args.pattern == "dataset-random-walk-moving":
        return generate_moving_camera_random_walk(
            area=area,
            dt=args.dt,
            duration=duration,
            minimum_distance=args.minimum_distance,
            maximum_distance=args.maximum_distance,
            camera_horizontal_fov=args.camera_horizontal_fov,
            camera_vertical_fov=args.camera_vertical_fov,
            margin=args.horizontal_margin,
            vertical_margin=args.vertical_margin,
            random_seed=args.random_seed,
            random_waypoints=args.random_waypoints,
            camera_motion_radius=args.moving_camera_radius,
            camera_heading_walk_limit=math.radians(args.moving_camera_heading_walk),
            camera_placement=args.static_camera_placement,
            camera_circle_clearance=args.static_camera_circle_clearance,
            camera_circle_point=args.static_camera_circle_point,
            camera_edge_inset=args.static_camera_inset,
            camera_heading=(
                math.radians(args.camera_heading)
                if args.camera_heading is not None
                else None
            ),
            rrt_visualization=rrt_visualization,
        )
    if args.pattern.startswith("dataset-"):
        return generate_dataset_trajectories(
            area=area,
            dt=args.dt,
            duration=duration,
            pattern=args.pattern.removeprefix("dataset-"),
            minimum_distance=args.minimum_distance,
            maximum_distance=args.maximum_distance,
            camera_horizontal_fov=args.camera_horizontal_fov,
            camera_vertical_fov=args.camera_vertical_fov,
            relative_heading_turns=args.relative_heading_turns,
            margin=args.horizontal_margin,
            vertical_margin=args.vertical_margin,
            travel_distance=args.travel_distance,
            observer_path=args.observer_path,
            circle_radius=args.circle_radius,
        )
    return generate(
        area, args.dt, duration, args.separation, args.horizontal_margin, args.vertical_margin
    )


def _verify_trajectories(
    trajectories: Sequence[Sequence[tuple[float, float, float, float]]],
    dt: float,
    constraints: DynamicConstraints,
) -> list[ConstraintVerification]:
    return [
        verify_dynamic_constraints(trajectory, dt, constraints)
        for trajectory in trajectories
    ]


def _resample_to_fastest_valid_duration(
    area: SafetyArea,
    args: argparse.Namespace,
    longest_valid_duration: float,
    constraints: DynamicConstraints,
) -> tuple[float, tuple[list[tuple[float, float, float, float]], ...], list[ConstraintVerification]]:
    """Resample one spatial path at the fastest duration allowed by all limits."""
    longest_sample_count = math.floor(longest_valid_duration / args.dt + 1e-9) + 1
    minimum_sample_count = 5  # Derivative verification needs through snap.
    if longest_sample_count <= minimum_sample_count:
        trajectories = _generate_from_args(area, args, longest_valid_duration)
        return (
            longest_valid_duration,
            trajectories,
            _verify_trajectories(trajectories, args.dt, constraints),
        )

    low, high = minimum_sample_count, longest_sample_count
    while low < high:
        sample_count = (low + high) // 2
        duration = (sample_count - 1) * args.dt
        trajectories = _generate_from_args(area, args, duration)
        results = _verify_trajectories(trajectories, args.dt, constraints)
        if all(result.passed for result in results):
            high = sample_count
        else:
            low = sample_count + 1

    duration = (low - 1) * args.dt
    trajectories = _generate_from_args(area, args, duration)
    return duration, trajectories, _verify_trajectories(trajectories, args.dt, constraints)


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    try:
        area = load_safety_area(args.config)
        platform = load_platform_config(args.platform) if args.platform is not None else None
        actual_duration = args.duration
        trajectories = _generate_from_args(area, args, actual_duration)
        rrt_visualization = (
            RRTVisualization()
            if args.plot_rrt
            and (args.plot is not None or args.show_plot)
            and args.pattern in {"dataset-random-walk-static", "dataset-random-walk-moving"}
            else None
        )
        verification_results: list[ConstraintVerification] = []
        dynamic_constraints: DynamicConstraints | None = None
        constraints_path = args.constraints
        if constraints_path is None and platform is not None:
            constraints_path = DEFAULT_CONSTRAINTS_PATH
        if constraints_path is not None:
            if platform is not None:
                validate_platform_constraint_profile(platform, args.constraint_profile)
            dynamic_constraints = load_dynamic_constraints(
                constraints_path, args.constraint_profile
            )
            verification_results = _verify_trajectories(
                trajectories, args.dt, dynamic_constraints
            )
            for _ in range(32):
                if all(result.passed for result in verification_results):
                    break
                # Increasing duration preserves the spatial path while reducing
                # speed and higher derivatives. Snap to a whole sample interval.
                next_sample_count = math.ceil(actual_duration * 1.1 / args.dt)
                actual_duration = next_sample_count * args.dt
                trajectories = _generate_from_args(area, args, actual_duration)
                verification_results = _verify_trajectories(
                    trajectories, args.dt, dynamic_constraints
                )
            if all(result.passed for result in verification_results):
                actual_duration, trajectories, verification_results = (
                    _resample_to_fastest_valid_duration(
                        area, args, actual_duration, dynamic_constraints
                    )
                )
            violation_lines: list[str] = []
            violation_count = sum(len(result.violations) for result in verification_results)
            for uav_index, result in enumerate(verification_results, start=1):
                worst_by_quantity: dict[str, ConstraintViolation] = {}
                for violation in result.violations:
                    previous = worst_by_quantity.get(violation.quantity)
                    if previous is None or violation.value / violation.limit > previous.value / previous.limit:
                        worst_by_quantity[violation.quantity] = violation
                for violation in worst_by_quantity.values():
                    violation_lines.append(
                        f"UAV {uav_index} {violation.quantity}: {violation.value:.3f} > "
                        f"{violation.limit:.3f} at sample {violation.sample_index}"
                    )
            if violation_lines:
                summary = "\n  ".join(violation_lines)
                raise ConfigurationError(
                    f"dynamic constraints profile '{dynamic_constraints.name}' failed "
                    f"with {violation_count} violating samples:\n  {summary}"
                )
        if rrt_visualization is not None:
            # Regenerate the final time-resampled path once to collect the RRT
            # planning overlay used by the trajectory map.
            trajectories = _generate_from_args(
                area, args, actual_duration, rrt_visualization
            )
        unshifted_trajectories = trajectories
        trajectories = shift_trajectories_to_boundary(
            area,
            trajectories,
            args.placement_direction,
            args.boundary_offset,
            args.horizontal_margin,
        )
        if rrt_visualization is not None and trajectories and trajectories[0]:
            shift_x = trajectories[0][0][0] - unshifted_trajectories[0][0][0]
            shift_y = trajectories[0][0][1] - unshifted_trajectories[0][0][1]
            if rrt_visualization.points is not None:
                rrt_visualization.points = [
                    (point[0] + shift_x, point[1] + shift_y, point[2])
                    for point in rrt_visualization.points
                ]
            if rrt_visualization.spline_controls is not None:
                rrt_visualization.spline_controls = [
                    (point[0] + shift_x, point[1] + shift_y, point[2])
                    for point in rrt_visualization.spline_controls
                ]
            if rrt_visualization.observer_xy is not None:
                rrt_visualization.observer_xy = (
                    rrt_visualization.observer_xy[0] + shift_x,
                    rrt_visualization.observer_xy[1] + shift_y,
                )
        args.output_dir.mkdir(parents=True, exist_ok=True)
        paths = [args.output_dir / "uav1.txt", args.output_dir / "uav2.txt"]
        for path, trajectory in zip(paths, trajectories):
            _write_trajectory(path, trajectory)
        config_path = args.output_dir / "loader_config.yaml"
        _write_loader_config(config_path, area.frame_id, args.dt)
        plot_path = None
        if args.plot is not None:
            plot_path = args.plot if args.plot.is_absolute() else args.output_dir / args.plot
        if plot_path is not None or args.show_plot:
            visualize_trajectories(
                area,
                trajectories,
                args.dt,
                plot_path,
                args.show_plot,
                dynamic_constraints,
                rrt_visualization,
            )
    except (ConfigurationError, OSError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2

    print(f"Generated {len(trajectories[0])} points per UAV in {args.output_dir}")
    print(
        f"MRS frame: {area.frame_id}; dt: {args.dt:g} s; "
        f"duration: {actual_duration:g} s"
    )
    if actual_duration > args.duration + 1e-9:
        print(
            f"Duration automatically extended from {args.duration:g} s to "
            f"{actual_duration:g} s to satisfy dynamic constraints"
        )
    elif actual_duration < args.duration - 1e-9:
        print(
            f"Duration resampled from {args.duration:g} s to {actual_duration:g} s "
            "to use the available dynamic-constraint headroom"
        )
    print(f"Trajectories: {paths[0]}, {paths[1]}")
    print(f"Loader config: {config_path}")
    if args.placement_direction != "center":
        print(
            f"Placement: {args.placement_direction}; boundary offset: "
            f"{args.boundary_offset:g} m (plus {args.horizontal_margin:g} m horizontal margin)"
        )
    if args.pattern.startswith("dataset-"):
        distances = [
            math.dist(observer[:3], target[:3])
            for observer, target in zip(trajectories[0], trajectories[1])
        ]
        circle_summary = ""
        if args.observer_path == "circle":
            observer_x = [point[0] for point in trajectories[0]]
            actual_radius = (max(observer_x) - min(observer_x)) / 2.0
            circle_summary = f"; circle radius: {actual_radius:.2f} m"
        print(
            f"Dataset roles: UAV 1 = front-camera observer, UAV 2 = target; "
            f"distance range: {min(distances):.2f}-{max(distances):.2f} m; "
            f"observer travel: {_trajectory_length(trajectories[0]):.2f} m"
            f"{circle_summary}"
        )
    if platform is not None:
        motor_summary = (
            f", {platform.motor_count} motors"
            if platform.motor_count is not None
            else ""
        )
        print(f"Platform: '{platform.name}' loaded{motor_summary}")
    if constraints_path is not None:
        print(
            f"Dynamic constraints: profile '{args.constraint_profile}' from "
            f"{constraints_path} satisfied"
        )
    if plot_path is not None:
        print(f"Visualization: {plot_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
