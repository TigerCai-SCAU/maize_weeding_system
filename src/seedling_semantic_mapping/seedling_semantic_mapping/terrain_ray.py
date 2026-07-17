"""Pure-numpy helpers for odometry interpolation and 2.5D terrain rays."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Dict, Optional, Sequence, Tuple

import numpy as np


def normalize_quaternion(q_xyzw: np.ndarray) -> np.ndarray:
    q = np.asarray(q_xyzw, dtype=np.float64)
    norm = float(np.linalg.norm(q))
    if norm < 1e-12:
        raise ValueError("quaternion norm is zero")
    return q / norm


def quaternion_slerp(
    q0_xyzw: np.ndarray,
    q1_xyzw: np.ndarray,
    ratio: float,
) -> np.ndarray:
    """Shortest-path SLERP for ROS xyzw quaternions."""
    q0 = normalize_quaternion(q0_xyzw)
    q1 = normalize_quaternion(q1_xyzw)
    dot = float(np.dot(q0, q1))
    if dot < 0.0:
        q1 = -q1
        dot = -dot
    dot = float(np.clip(dot, -1.0, 1.0))
    ratio = float(np.clip(ratio, 0.0, 1.0))
    if dot > 0.9995:
        return normalize_quaternion(q0 + ratio * (q1 - q0))
    theta = math.acos(dot)
    sin_theta = math.sin(theta)
    return normalize_quaternion(
        math.sin((1.0 - ratio) * theta) / sin_theta * q0
        + math.sin(ratio * theta) / sin_theta * q1
    )


def quaternion_to_rotation(q_xyzw: np.ndarray) -> np.ndarray:
    x, y, z, w = normalize_quaternion(q_xyzw)
    return np.array(
        [
            [
                1.0 - 2.0 * (y * y + z * z),
                2.0 * (x * y - z * w),
                2.0 * (x * z + y * w),
            ],
            [
                2.0 * (x * y + z * w),
                1.0 - 2.0 * (x * x + z * z),
                2.0 * (y * z - x * w),
            ],
            [
                2.0 * (x * z - y * w),
                2.0 * (y * z + x * w),
                1.0 - 2.0 * (x * x + y * y),
            ],
        ],
        dtype=np.float64,
    )


@dataclass(frozen=True)
class InterpolatedPose:
    translation: np.ndarray
    quaternion_xyzw: np.ndarray


def interpolate_pose(
    t0: float,
    translation0: np.ndarray,
    quaternion0_xyzw: np.ndarray,
    t1: float,
    translation1: np.ndarray,
    quaternion1_xyzw: np.ndarray,
    query_time: float,
) -> InterpolatedPose:
    if t1 <= t0:
        raise ValueError("pose timestamps must be strictly increasing")
    if query_time < t0 or query_time > t1:
        raise ValueError("query time is outside the pose bracket")
    ratio = (query_time - t0) / (t1 - t0)
    p0 = np.asarray(translation0, dtype=np.float64)
    p1 = np.asarray(translation1, dtype=np.float64)
    return InterpolatedPose(
        translation=(1.0 - ratio) * p0 + ratio * p1,
        quaternion_xyzw=quaternion_slerp(
            quaternion0_xyzw,
            quaternion1_xyzw,
            ratio,
        ),
    )


class TerrainHeightMap:
    """Sparse 2.5D grid with configurable vertical and horizontal axes."""

    def __init__(
        self,
        points: np.ndarray,
        resolution_m: float,
        vertical_axis: int,
        horizontal_axes: Sequence[int],
        lookup_radius_m: float,
        min_neighbors: int,
    ) -> None:
        points = np.asarray(points, dtype=np.float64).reshape(-1, 3)
        axes = (int(horizontal_axes[0]), int(horizontal_axes[1]))
        if sorted((int(vertical_axis), axes[0], axes[1])) != [0, 1, 2]:
            raise ValueError("terrain axes must be a permutation of 0, 1, 2")
        if resolution_m <= 0.0 or lookup_radius_m <= 0.0:
            raise ValueError("terrain resolution and lookup radius must be positive")
        self.resolution_m = float(resolution_m)
        self.vertical_axis = int(vertical_axis)
        self.horizontal_axes = axes
        self.lookup_radius_m = float(lookup_radius_m)
        self.min_neighbors = max(1, int(min_neighbors))
        self._search_cells = max(
            1,
            int(math.ceil(self.lookup_radius_m / self.resolution_m)),
        )
        self._heights: Dict[Tuple[int, int], float] = {}
        if points.size == 0:
            return
        points = points[np.isfinite(points).all(axis=1)]
        keys = np.floor(
            points[:, self.horizontal_axes] / self.resolution_m
        ).astype(np.int64)
        for key, height in zip(keys, points[:, self.vertical_axis]):
            self._heights[(int(key[0]), int(key[1]))] = float(height)

    def __len__(self) -> int:
        return len(self._heights)

    def height_at(self, horizontal: np.ndarray) -> Optional[float]:
        horizontal = np.asarray(horizontal, dtype=np.float64).reshape(2)
        base = np.floor(horizontal / self.resolution_m).astype(np.int64)
        radius_sq = self.lookup_radius_m * self.lookup_radius_m
        values = []
        weights = []
        for di in range(-self._search_cells, self._search_cells + 1):
            for dj in range(-self._search_cells, self._search_cells + 1):
                key = (int(base[0] + di), int(base[1] + dj))
                height = self._heights.get(key)
                if height is None:
                    continue
                centre = (np.asarray(key, dtype=np.float64) + 0.5) * self.resolution_m
                distance_sq = float(np.sum((centre - horizontal) ** 2))
                if distance_sq > radius_sq:
                    continue
                values.append(height)
                weights.append(1.0 / max(distance_sq, 1e-8))
        if len(values) < self.min_neighbors:
            return None
        return float(np.average(values, weights=weights))

    def signed_height_error(self, point: np.ndarray) -> Optional[float]:
        point = np.asarray(point, dtype=np.float64).reshape(3)
        height = self.height_at(point[list(self.horizontal_axes)])
        if height is None:
            return None
        return float(point[self.vertical_axis] - height)

    def intersect_ray(
        self,
        origin: np.ndarray,
        direction: np.ndarray,
        min_range_m: float,
        max_range_m: float,
        step_m: float,
        height_tolerance_m: float,
        max_valid_gap_m: float,
    ) -> Optional[np.ndarray]:
        """Return the first ray/terrain intersection inside observed coverage."""
        origin = np.asarray(origin, dtype=np.float64).reshape(3)
        direction = np.asarray(direction, dtype=np.float64).reshape(3)
        norm = float(np.linalg.norm(direction))
        if norm < 1e-12 or not np.isfinite(origin).all():
            return None
        direction = direction / norm
        if min_range_m < 0.0 or max_range_m <= min_range_m or step_m <= 0.0:
            return None

        best = None
        best_abs_error = float("inf")
        previous = None
        distance = float(min_range_m)
        while distance <= max_range_m + 1e-9:
            point = origin + distance * direction
            error = self.signed_height_error(point)
            if error is None:
                previous = None
                distance += step_m
                continue
            abs_error = abs(error)
            if abs_error < best_abs_error:
                best = point
                best_abs_error = abs_error
            if abs_error <= height_tolerance_m:
                return point
            if previous is not None:
                previous_distance, previous_error = previous
                if (
                    distance - previous_distance <= max_valid_gap_m
                    and error * previous_error < 0.0
                ):
                    low = previous_distance
                    high = distance
                    low_error = previous_error
                    for _ in range(10):
                        middle = 0.5 * (low + high)
                        middle_point = origin + middle * direction
                        middle_error = self.signed_height_error(middle_point)
                        if middle_error is None:
                            break
                        if abs(middle_error) <= height_tolerance_m * 0.1:
                            return middle_point
                        if low_error * middle_error <= 0.0:
                            high = middle
                        else:
                            low = middle
                            low_error = middle_error
                    return origin + 0.5 * (low + high) * direction
            previous = (distance, error)
            distance += step_m
        if best is not None and best_abs_error <= height_tolerance_m:
            return best
        return None
