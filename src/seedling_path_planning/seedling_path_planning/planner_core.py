"""ROS-independent row analysis and safe coverage path generation."""

from __future__ import annotations

from dataclasses import dataclass, field
import heapq
import math
from typing import Iterable, Sequence

import numpy as np


@dataclass(frozen=True)
class PlannerConfig:
    expected_row_spacing: float = 0.55
    expected_plant_spacing: float = 0.20
    row_cluster_threshold: float = 0.22
    plant_spacing_tolerance: float = 0.35
    min_plants_per_row: int = 2
    protection_radius: float = 0.08
    safety_margin: float = 0.015
    coverage_spacing: float = 0.06
    path_resolution: float = 0.02
    lateral_work_margin: float = 0.10
    travel_work_margin: float = 0.05
    max_grid_cells: int = 250_000
    s_sweep_offset: float = 0.11
    s_close_cluster_gap: float = 0.0
    s_first_side: int = 1

    @property
    def obstacle_radius(self) -> float:
        return self.protection_radius + self.safety_margin


@dataclass
class RowModel:
    index: int
    point_indices: list[int]
    slope: float
    intercept: float
    travel_min: float
    travel_max: float
    missing_slots: int = 0
    close_pairs: int = 0
    spacings: list[float] = field(default_factory=list)
    predicted_missing_lt: list[tuple[float, float]] = field(default_factory=list)

    def lateral_at(self, travel: float) -> float:
        return self.slope * travel + self.intercept


@dataclass
class PlanResult:
    rows: list[RowModel]
    path_lt: np.ndarray
    bounds: tuple[float, float, float, float]
    obstacle_radius: float
    seedling_lt: np.ndarray
    row_spacing_measured: list[float]


@dataclass
class DualArmPlanResult:
    rows: list[RowModel]
    arm_paths_lt: list[np.ndarray]
    bounds: tuple[float, float, float, float]
    obstacle_radius: float
    seedling_lt: np.ndarray
    row_spacing_measured: list[float]


def _robust_line_fit(travel: np.ndarray, lateral: np.ndarray) -> tuple[float, float]:
    if travel.size < 2 or float(np.ptp(travel)) < 1e-6:
        return 0.0, float(np.median(lateral))
    mask = np.ones(travel.size, dtype=bool)
    slope, intercept = 0.0, float(np.median(lateral))
    for _ in range(3):
        if int(mask.sum()) < 2:
            break
        slope, intercept = np.polyfit(travel[mask], lateral[mask], 1)
        residual = lateral - (slope * travel + intercept)
        median = float(np.median(residual[mask]))
        mad = float(np.median(np.abs(residual[mask] - median)))
        if mad < 1e-6:
            break
        mask = np.abs(residual - median) <= max(0.025, 3.5 * 1.4826 * mad)
    return float(slope), float(intercept)


def _cluster_rows(lateral: np.ndarray, threshold: float) -> list[list[int]]:
    order = np.argsort(lateral)
    clusters: list[list[int]] = []
    for raw_index in order:
        index = int(raw_index)
        if not clusters:
            clusters.append([index])
            continue
        center = float(np.median(lateral[clusters[-1]]))
        if abs(float(lateral[index]) - center) <= threshold:
            clusters[-1].append(index)
        else:
            clusters.append([index])
    return clusters


def analyze_rows(seedling_lt: np.ndarray, config: PlannerConfig) -> list[RowModel]:
    """Group actual seedlings into rows and diagnose irregular sowing.

    ``seedling_lt`` columns are lateral and travel coordinates. Planting
    standards are used only for diagnostics; no synthetic seedlings are added.
    """
    points = np.asarray(seedling_lt, dtype=np.float64).reshape(-1, 2)
    if points.shape[0] < max(2, config.min_plants_per_row):
        return []
    finite = np.isfinite(points).all(axis=1)
    points = points[finite]
    if points.shape[0] < config.min_plants_per_row:
        return []

    threshold = config.row_cluster_threshold
    if threshold <= 0:
        threshold = max(0.05, 0.40 * config.expected_row_spacing)
    clusters = _cluster_rows(points[:, 0], threshold)
    clusters = [
        indices for indices in clusters if len(indices) >= config.min_plants_per_row
    ]
    clusters.sort(key=lambda ids: float(np.median(points[ids, 0])))

    rows: list[RowModel] = []
    low = config.expected_plant_spacing * (1.0 - config.plant_spacing_tolerance)
    high = config.expected_plant_spacing * (1.0 + config.plant_spacing_tolerance)
    for row_index, indices in enumerate(clusters):
        travel = points[indices, 1]
        lateral = points[indices, 0]
        slope, intercept = _robust_line_fit(travel, lateral)
        sorted_order = np.argsort(travel)
        sorted_travel = travel[sorted_order]
        spacings = np.diff(sorted_travel)
        close_pairs = int(np.sum(spacings < low)) if spacings.size else 0
        missing_slots = 0
        predicted_missing_lt: list[tuple[float, float]] = []
        for gap_index, gap in enumerate(spacings):
            if gap > high:
                slot_count = max(
                    1, int(round(float(gap) / config.expected_plant_spacing)) - 1
                )
                missing_slots += slot_count
                start_travel = float(sorted_travel[gap_index])
                for slot_index in range(1, slot_count + 1):
                    predicted_travel = start_travel + float(gap) * (
                        slot_index / (slot_count + 1)
                    )
                    predicted_missing_lt.append(
                        (
                            float(slope * predicted_travel + intercept),
                            predicted_travel,
                        )
                    )
        rows.append(
            RowModel(
                index=row_index,
                point_indices=list(indices),
                slope=slope,
                intercept=intercept,
                travel_min=float(np.min(travel)),
                travel_max=float(np.max(travel)),
                missing_slots=missing_slots,
                close_pairs=close_pairs,
                spacings=[float(value) for value in spacings],
                predicted_missing_lt=predicted_missing_lt,
            )
        )
    return rows


def _smooth_path_through_knots(
    knots_lt: np.ndarray,
    travel_values: np.ndarray,
) -> np.ndarray:
    """Cosine-interpolate lateral knots on a strictly forward travel grid."""
    knots = np.asarray(knots_lt, dtype=np.float64).reshape(-1, 2)
    output = np.empty((len(travel_values), 2), dtype=np.float64)
    output[:, 1] = travel_values
    for index, travel in enumerate(travel_values):
        right = int(np.searchsorted(knots[:, 1], travel, side="right"))
        if right <= 0:
            output[index, 0] = knots[0, 0]
            continue
        if right >= len(knots):
            output[index, 0] = knots[-1, 0]
            continue
        left = right - 1
        span = float(knots[right, 1] - knots[left, 1])
        if span <= 1e-9:
            output[index, 0] = knots[right, 0]
            continue
        ratio = float((travel - knots[left, 1]) / span)
        blend = 0.5 - 0.5 * math.cos(math.pi * ratio)
        output[index, 0] = (
            float(knots[left, 0])
            + blend * float(knots[right, 0] - knots[left, 0])
        )
    return output


def _row_s_path(
    seedlings: np.ndarray,
    row: RowModel,
    travel_values: np.ndarray,
    config: PlannerConfig,
) -> np.ndarray:
    row_points = seedlings[row.point_indices]
    row_points = row_points[np.argsort(row_points[:, 1])]
    if len(row_points) < 2:
        raise ValueError(f"row {row.index} has fewer than two seedlings")

    side = 1 if config.s_first_side >= 0 else -1
    cluster_gap = config.s_close_cluster_gap
    if cluster_gap <= 0.0:
        cluster_gap = 2.0 * config.obstacle_radius + config.path_resolution
    signs = [side]
    for previous, current in zip(row_points, row_points[1:]):
        if float(current[1] - previous[1]) > cluster_gap:
            side *= -1
        signs.append(side)

    offset = max(config.s_sweep_offset, config.obstacle_radius)
    knots = [
        (
            row.lateral_at(float(travel_values[0])) + signs[0] * offset,
            float(travel_values[0]),
        )
    ]
    targets = [
        (float(point[0]) + sign * offset, float(point[1]))
        for point, sign in zip(row_points, signs)
    ]
    knots.append(targets[0])
    transition_guard = config.obstacle_radius + config.path_resolution
    for point_index in range(1, len(targets)):
        previous_target = targets[point_index - 1]
        current_target = targets[point_index]
        if signs[point_index] != signs[point_index - 1]:
            gap = current_target[1] - previous_target[1]
            guard = min(transition_guard, 0.45 * gap)
            knots.append(
                (previous_target[0], previous_target[1] + guard)
            )
            knots.append(
                (current_target[0], current_target[1] - guard)
            )
        knots.append(current_target)
    knots.append(
        (
            row.lateral_at(float(travel_values[-1])) + signs[-1] * offset,
            float(travel_values[-1]),
        )
    )
    return _smooth_path_through_knots(np.asarray(knots), travel_values)


def plan_dual_arm_s(
    seedling_lt: np.ndarray,
    config: PlannerConfig,
) -> DualArmPlanResult:
    """Plan two synchronized, forward-only S paths, one measured row per arm.

    The vehicle or conveyor supplies the forward motion. Each arm only weaves
    laterally around the actual seedlings assigned to its row. Nominal planting
    spacing remains a prediction/diagnostic prior and never creates obstacles.
    """
    seedlings = np.asarray(seedling_lt, dtype=np.float64).reshape(-1, 2)
    seedlings = seedlings[np.isfinite(seedlings).all(axis=1)]
    rows = analyze_rows(seedlings, config)
    if len(rows) < 2:
        raise ValueError("at least two valid seedling rows are required")
    assigned_rows = [rows[0], rows[-1]]

    travel_margin = max(
        config.travel_work_margin,
        config.obstacle_radius + config.path_resolution,
    )
    travel_min = float(np.min(seedlings[:, 1]) - travel_margin)
    travel_max = float(np.max(seedlings[:, 1]) + travel_margin)
    travel_count = max(
        2,
        int(math.ceil((travel_max - travel_min) / config.path_resolution)) + 1,
    )
    travel_values = np.linspace(travel_min, travel_max, travel_count)
    arm_paths = [
        _row_s_path(seedlings, row, travel_values, config)
        for row in assigned_rows
    ]

    for arm_index, path in enumerate(arm_paths, start=1):
        clearance = minimum_seedling_clearance(path, seedlings)
        if clearance + 1e-9 < config.obstacle_radius:
            raise ValueError(
                f"arm {arm_index} S path clearance {clearance:.3f} m is below "
                f"{config.obstacle_radius:.3f} m"
            )

    lateral_min = min(float(np.min(path[:, 0])) for path in arm_paths)
    lateral_max = max(float(np.max(path[:, 0])) for path in arm_paths)
    reference_travel = float(np.median(seedlings[:, 1]))
    row_spacing_measured = [
        assigned_rows[1].lateral_at(reference_travel)
        - assigned_rows[0].lateral_at(reference_travel)
    ]
    return DualArmPlanResult(
        rows=assigned_rows,
        arm_paths_lt=arm_paths,
        bounds=(lateral_min, lateral_max, travel_min, travel_max),
        obstacle_radius=config.obstacle_radius,
        seedling_lt=seedlings,
        row_spacing_measured=row_spacing_measured,
    )


def _point_is_free(
    point: tuple[float, float],
    seedlings: np.ndarray,
    radius: float,
) -> bool:
    if seedlings.size == 0:
        return True
    delta = seedlings - np.asarray(point, dtype=np.float64)
    return bool(np.all(np.sum(delta * delta, axis=1) >= radius * radius))


def _segment_is_free(
    start: tuple[float, float],
    end: tuple[float, float],
    seedlings: np.ndarray,
    radius: float,
    resolution: float,
) -> bool:
    distance = math.dist(start, end)
    count = max(2, int(math.ceil(distance / max(resolution * 0.5, 1e-3))) + 1)
    for ratio in np.linspace(0.0, 1.0, count):
        point = (
            start[0] + float(ratio) * (end[0] - start[0]),
            start[1] + float(ratio) * (end[1] - start[1]),
        )
        if not _point_is_free(point, seedlings, radius):
            return False
    return True


class _Grid:
    def __init__(
        self,
        bounds: tuple[float, float, float, float],
        resolution: float,
        seedlings: np.ndarray,
        radius: float,
        max_cells: int,
    ) -> None:
        self.lateral_min, self.lateral_max, self.travel_min, self.travel_max = bounds
        self.resolution = max(float(resolution), 0.005)
        self.width = int(
            math.ceil((self.lateral_max - self.lateral_min) / self.resolution)
        ) + 1
        self.height = int(
            math.ceil((self.travel_max - self.travel_min) / self.resolution)
        ) + 1
        if self.width * self.height > max_cells:
            raise ValueError(
                f"planning grid too large: {self.width}x{self.height} "
                f"> {max_cells} cells"
            )
        self.blocked = np.zeros((self.height, self.width), dtype=bool)
        if seedlings.size:
            for row in range(self.height):
                travel = self.travel_min + row * self.resolution
                for column in range(self.width):
                    lateral = self.lateral_min + column * self.resolution
                    self.blocked[row, column] = not _point_is_free(
                        (lateral, travel), seedlings, radius
                    )

    def to_cell(self, point: tuple[float, float]) -> tuple[int, int]:
        column = int(round((point[0] - self.lateral_min) / self.resolution))
        row = int(round((point[1] - self.travel_min) / self.resolution))
        return (
            min(max(row, 0), self.height - 1),
            min(max(column, 0), self.width - 1),
        )

    def to_point(self, cell: tuple[int, int]) -> tuple[float, float]:
        row, column = cell
        return (
            self.lateral_min + column * self.resolution,
            self.travel_min + row * self.resolution,
        )

    def nearest_free(self, cell: tuple[int, int]) -> tuple[int, int] | None:
        if not self.blocked[cell]:
            return cell
        row0, column0 = cell
        limit = max(self.width, self.height)
        for radius in range(1, limit):
            candidates = []
            for row in range(max(0, row0 - radius), min(self.height, row0 + radius + 1)):
                for column in range(
                    max(0, column0 - radius),
                    min(self.width, column0 + radius + 1),
                ):
                    if max(abs(row - row0), abs(column - column0)) != radius:
                        continue
                    if not self.blocked[row, column]:
                        candidates.append((row, column))
            if candidates:
                return min(
                    candidates,
                    key=lambda item: (item[0] - row0) ** 2
                    + (item[1] - column0) ** 2,
                )
        return None

    def astar(
        self,
        start_point: tuple[float, float],
        goal_point: tuple[float, float],
    ) -> list[tuple[float, float]]:
        start = self.nearest_free(self.to_cell(start_point))
        goal = self.nearest_free(self.to_cell(goal_point))
        if start is None or goal is None:
            return []
        if start == goal:
            return [self.to_point(start)]

        moves = (
            (-1, 0, 1.0),
            (1, 0, 1.0),
            (0, -1, 1.0),
            (0, 1, 1.0),
            (-1, -1, math.sqrt(2.0)),
            (-1, 1, math.sqrt(2.0)),
            (1, -1, math.sqrt(2.0)),
            (1, 1, math.sqrt(2.0)),
        )
        queue: list[tuple[float, float, tuple[int, int]]] = []
        heapq.heappush(queue, (0.0, 0.0, start))
        cost = {start: 0.0}
        parent: dict[tuple[int, int], tuple[int, int]] = {}
        while queue:
            _, current_cost, current = heapq.heappop(queue)
            if current == goal:
                cells = [goal]
                while cells[-1] != start:
                    cells.append(parent[cells[-1]])
                cells.reverse()
                return [self.to_point(cell) for cell in cells]
            if current_cost > cost.get(current, float("inf")):
                continue
            for drow, dcolumn, move_cost in moves:
                neighbor = (current[0] + drow, current[1] + dcolumn)
                if not (
                    0 <= neighbor[0] < self.height
                    and 0 <= neighbor[1] < self.width
                ):
                    continue
                if self.blocked[neighbor]:
                    continue
                if drow and dcolumn:
                    if (
                        self.blocked[current[0], neighbor[1]]
                        or self.blocked[neighbor[0], current[1]]
                    ):
                        continue
                new_cost = current_cost + move_cost
                if new_cost >= cost.get(neighbor, float("inf")):
                    continue
                cost[neighbor] = new_cost
                parent[neighbor] = current
                heuristic = math.hypot(
                    neighbor[0] - goal[0], neighbor[1] - goal[1]
                )
                heapq.heappush(queue, (new_cost + heuristic, new_cost, neighbor))
        return []


def _append_safe_connection(
    output: list[tuple[float, float]],
    target: tuple[float, float],
    grid: _Grid,
    seedlings: np.ndarray,
    obstacle_radius: float,
) -> bool:
    if not output:
        output.append(target)
        return True
    start = output[-1]
    if _segment_is_free(
        start, target, seedlings, obstacle_radius, grid.resolution
    ):
        output.append(target)
        return True
    connector = grid.astar(start, target)
    if not connector:
        return False
    output.extend(connector[1:])
    return True


def _simplify_path(
    path: Sequence[tuple[float, float]],
    seedlings: np.ndarray,
    radius: float,
    resolution: float,
) -> list[tuple[float, float]]:
    if len(path) <= 2:
        return list(path)
    result = [path[0]]
    for point in path[1:]:
        if math.dist(result[-1], point) < 1e-9:
            continue
        if len(result) < 2:
            result.append(point)
            continue
        first = np.asarray(result[-1]) - np.asarray(result[-2])
        second = np.asarray(point) - np.asarray(result[-1])
        cross = abs(float(first[0] * second[1] - first[1] * second[0]))
        forward = float(np.dot(first, second)) >= 0.0
        if (
            cross <= 1e-8
            and forward
            and _segment_is_free(
                result[-2], point, seedlings, radius, resolution
            )
        ):
            result[-1] = point
        else:
            result.append(point)
    return result


def plan_coverage(seedling_lt: np.ndarray, config: PlannerConfig) -> PlanResult:
    seedlings = np.asarray(seedling_lt, dtype=np.float64).reshape(-1, 2)
    seedlings = seedlings[np.isfinite(seedlings).all(axis=1)]
    rows = analyze_rows(seedlings, config)
    if len(rows) < 2:
        raise ValueError("at least two valid seedling rows are required")

    # Keep enough free space around edge plants for A* to route outside their
    # protection disks. Smaller user margins would create isolated corner cells.
    travel_margin = max(
        config.travel_work_margin,
        config.obstacle_radius + config.path_resolution,
    )
    lateral_margin = max(
        config.lateral_work_margin,
        config.obstacle_radius + config.path_resolution,
    )
    travel_min = float(np.min(seedlings[:, 1]) - travel_margin)
    travel_max = float(np.max(seedlings[:, 1]) + travel_margin)
    row_left = rows[0]
    row_right = rows[-1]
    samples = np.asarray([travel_min, travel_max])
    left_edge = min(row_left.lateral_at(value) for value in samples)
    right_edge = max(row_right.lateral_at(value) for value in samples)
    lateral_min = left_edge - lateral_margin
    lateral_max = right_edge + lateral_margin
    bounds = (lateral_min, lateral_max, travel_min, travel_max)
    # Inflate only the search occupancy by half a grid cell so diagonal A*
    # segments cannot shave the declared continuous-space safety boundary.
    search_radius = config.obstacle_radius + 0.5 * config.path_resolution

    grid = _Grid(
        bounds,
        config.path_resolution,
        seedlings,
        search_radius,
        config.max_grid_cells,
    )

    lane_count = max(
        2,
        int(math.ceil((lateral_max - lateral_min) / config.coverage_spacing))
        + 1,
    )
    lateral_lanes = np.linspace(lateral_min, lateral_max, lane_count)
    travel_count = max(
        2,
        int(math.ceil((travel_max - travel_min) / config.path_resolution)) + 1,
    )
    travel_forward = np.linspace(travel_min, travel_max, travel_count)

    path: list[tuple[float, float]] = []
    for lane_index, lateral in enumerate(lateral_lanes):
        travel_values: Iterable[float]
        if lane_index % 2 == 0:
            travel_values = travel_forward
        else:
            travel_values = travel_forward[::-1]
        for travel in travel_values:
            target = (float(lateral), float(travel))
            if not _point_is_free(target, seedlings, search_radius):
                continue
            _append_safe_connection(
                path,
                target,
                grid,
                seedlings,
                search_radius,
            )

    simplified = _simplify_path(
        path, seedlings, search_radius, config.path_resolution
    )
    if len(simplified) < 2:
        raise ValueError("no collision-free coverage path found")

    row_spacing_measured = []
    reference_travel = float(np.median(seedlings[:, 1]))
    for first, second in zip(rows, rows[1:]):
        row_spacing_measured.append(
            second.lateral_at(reference_travel)
            - first.lateral_at(reference_travel)
        )
    return PlanResult(
        rows=rows,
        path_lt=np.asarray(simplified, dtype=np.float64),
        bounds=bounds,
        obstacle_radius=config.obstacle_radius,
        seedling_lt=seedlings,
        row_spacing_measured=row_spacing_measured,
    )


def minimum_seedling_clearance(path_lt: np.ndarray, seedling_lt: np.ndarray) -> float:
    path = np.asarray(path_lt, dtype=np.float64).reshape(-1, 2)
    seedlings = np.asarray(seedling_lt, dtype=np.float64).reshape(-1, 2)
    if not path.size or not seedlings.size:
        return float("inf")
    if len(path) == 1:
        return float(np.linalg.norm(seedlings - path[0], axis=1).min())
    minimum = float("inf")
    for start, end in zip(path, path[1:]):
        segment = end - start
        length_sq = float(np.dot(segment, segment))
        if length_sq < 1e-12:
            distance = np.linalg.norm(seedlings - start, axis=1)
        else:
            ratio = ((seedlings - start) @ segment) / length_sq
            ratio = np.clip(ratio, 0.0, 1.0)
            nearest = start + ratio[:, None] * segment
            distance = np.linalg.norm(seedlings - nearest, axis=1)
        minimum = min(minimum, float(np.min(distance)))
    return minimum


def world_to_lateral_travel(
    points_xyz: np.ndarray,
    lateral_axis: int,
    travel_axis: int,
) -> np.ndarray:
    points = np.asarray(points_xyz, dtype=np.float64).reshape(-1, 3)
    return points[:, [lateral_axis, travel_axis]]


def lateral_travel_height_to_world(
    lateral: float,
    travel: float,
    height: float,
    lateral_axis: int,
    travel_axis: int,
    height_axis: int,
) -> np.ndarray:
    if sorted((lateral_axis, travel_axis, height_axis)) != [0, 1, 2]:
        raise ValueError("lateral, travel and height axes must be a permutation of 0,1,2")
    world = np.zeros(3, dtype=np.float64)
    world[lateral_axis] = lateral
    world[travel_axis] = travel
    world[height_axis] = height
    return world


def interpolate_terrain_height(
    lateral: float,
    travel: float,
    terrain_lth: np.ndarray,
    search_radius: float,
    fallback_height: float,
    nearest_count: int = 6,
) -> float:
    terrain = np.asarray(terrain_lth, dtype=np.float64).reshape(-1, 3)
    terrain = terrain[np.isfinite(terrain).all(axis=1)]
    if terrain.size == 0:
        return float(fallback_height)
    distance_sq = (
        (terrain[:, 0] - lateral) ** 2 + (terrain[:, 1] - travel) ** 2
    )
    mask = distance_sq <= search_radius * search_radius
    if not np.any(mask):
        return float(fallback_height)
    candidates = terrain[mask]
    candidate_distance_sq = distance_sq[mask]
    order = np.argsort(candidate_distance_sq)[: max(1, nearest_count)]
    weights = 1.0 / np.maximum(candidate_distance_sq[order], 1e-6)
    return float(np.average(candidates[order, 2], weights=weights))
