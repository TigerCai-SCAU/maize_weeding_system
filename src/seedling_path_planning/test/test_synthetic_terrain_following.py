import math

import numpy as np
import pytest

from seedling_path_planning.planner_core import (
    PlannerConfig,
    estimate_terrain_surface,
    offset_height_from_terrain,
    plan_dual_arm_s,
)


def two_regular_rows() -> np.ndarray:
    travel = np.arange(0.0, 1.81, 0.20)
    left = np.column_stack(
        (-0.275 + 0.006 * np.sin(4.0 * travel), travel)
    )
    right = np.column_stack(
        (0.275 + 0.006 * np.cos(3.0 * travel), travel + 0.01)
    )
    return np.vstack((left, right))


def terrain_grid(height_function, spacing: float = 0.04) -> np.ndarray:
    lateral, travel = np.meshgrid(
        np.arange(-0.55, 0.551, spacing),
        np.arange(-0.20, 2.201, spacing),
    )
    height = height_function(lateral, travel)
    return np.column_stack(
        (lateral.reshape(-1), travel.reshape(-1), height.reshape(-1))
    )


def signed_normal_offset(
    target_height: float,
    surface,
    height_axis_up_sign: int,
) -> float:
    slope_scale = math.sqrt(
        1.0
        + surface.slope_lateral**2
        + surface.slope_travel**2
    )
    return (
        height_axis_up_sign * (target_height - surface.height) / slope_scale
    )


@pytest.mark.parametrize("slope_deg", [0.0, 5.0, 10.0, 15.0])
@pytest.mark.parametrize(
    ("surface_offset", "height_axis_up_sign"),
    [(0.02, -1), (-0.02, 1)],
)
def test_flat_and_sloped_surfaces_keep_signed_two_centimetre_offset(
    slope_deg: float,
    surface_offset: float,
    height_axis_up_sign: int,
):
    slope = math.tan(math.radians(slope_deg))
    terrain = terrain_grid(
        lambda lateral, travel: 1.0 + slope * travel
    )
    plan = plan_dual_arm_s(
        two_regular_rows(),
        PlannerConfig(
            row_cluster_threshold=0.0,
            path_resolution=0.005,
            travel_work_margin=0.10,
        ),
    )
    measured_offsets = []
    measured_slopes = []
    for path in plan.arm_paths_lt:
        for lateral, travel in path[::10]:
            surface = estimate_terrain_surface(
                lateral,
                travel,
                terrain,
                search_radius=0.18,
                nearest_count=24,
                min_neighbors=3,
            )
            assert surface is not None
            target_height = offset_height_from_terrain(
                surface,
                surface_offset=surface_offset,
                height_axis_up_sign=height_axis_up_sign,
            )
            measured_offsets.append(
                signed_normal_offset(
                    target_height,
                    surface,
                    height_axis_up_sign,
                )
            )
            measured_slopes.append(
                math.degrees(
                    math.atan(
                        math.hypot(
                            surface.slope_lateral,
                            surface.slope_travel,
                        )
                    )
                )
            )
    assert np.allclose(measured_offsets, surface_offset, atol=1e-9)
    assert np.max(np.abs(np.asarray(measured_slopes) - slope_deg)) < 0.05


def test_smooth_twenty_millimetre_bump_is_followed_with_small_true_error():
    bump_start = 0.55
    bump_length = 0.60

    def bump_height(lateral, travel):
        phase = (travel - bump_start) / bump_length
        inside = (phase >= 0.0) & (phase <= 1.0)
        bump = np.zeros_like(travel)
        bump[inside] = 0.01 * (
            1.0 - np.cos(2.0 * math.pi * phase[inside])
        )
        return 1.0 + bump

    terrain = terrain_grid(bump_height, spacing=0.025)
    query_travel = np.linspace(0.35, 1.35, 101)
    errors = []
    target_heights = []
    for travel in query_travel:
        surface = estimate_terrain_surface(
            0.0,
            travel,
            terrain,
            search_radius=0.18,
            nearest_count=24,
            min_neighbors=3,
        )
        assert surface is not None
        target = offset_height_from_terrain(
            surface,
            surface_offset=0.02,
            height_axis_up_sign=1,
        )
        true_height = float(
            bump_height(np.asarray([0.0]), np.asarray([travel]))[0]
        )
        true_slope = (
            0.02
            * math.pi
            / bump_length
            * math.sin(2.0 * math.pi * (travel - bump_start) / bump_length)
            if bump_start <= travel <= bump_start + bump_length
            else 0.0
        )
        true_scale = math.sqrt(1.0 + true_slope * true_slope)
        true_offset = (target - true_height) / true_scale
        errors.append(true_offset - 0.02)
        target_heights.append(target)
    assert np.max(np.abs(errors)) < 0.0015
    assert np.ptp(target_heights) > 0.018


def test_noisy_surface_with_sparse_large_outliers_remains_usable():
    rng = np.random.default_rng(42)
    terrain = terrain_grid(
        lambda lateral, travel: 1.0 + 0.03 * lateral - 0.02 * travel,
        spacing=0.03,
    )
    terrain[:, 2] += rng.normal(0.0, 0.002, len(terrain))
    outliers = rng.choice(len(terrain), size=40, replace=False)
    terrain[outliers, 2] += rng.choice((-0.05, 0.05), size=len(outliers))
    errors = []
    for travel in np.linspace(0.2, 1.8, 25):
        surface = estimate_terrain_surface(
            0.0,
            travel,
            terrain,
            search_radius=0.18,
            nearest_count=24,
            min_neighbors=3,
        )
        assert surface is not None
        expected_height = 1.0 - 0.02 * travel
        errors.append(surface.height - expected_height)
    assert np.percentile(np.abs(errors), 95) < 0.004


def test_terrain_gap_and_excessive_slope_are_detectable():
    terrain = terrain_grid(
        lambda lateral, travel: 1.0 + math.tan(math.radians(40.0)) * travel
    )
    gap_mask = (
        (np.abs(terrain[:, 0]) < 0.20)
        & (np.abs(terrain[:, 1] - 1.0) < 0.20)
    )
    terrain_with_gap = terrain[~gap_mask]
    assert (
        estimate_terrain_surface(
            0.0,
            1.0,
            terrain_with_gap,
            search_radius=0.18,
            nearest_count=24,
            min_neighbors=3,
        )
        is None
    )

    steep = estimate_terrain_surface(
        0.0,
        0.4,
        terrain,
        search_radius=0.18,
        nearest_count=24,
        min_neighbors=3,
    )
    assert steep is not None
    steep_angle = math.degrees(
        math.atan(
            math.hypot(steep.slope_lateral, steep.slope_travel)
        )
    )
    assert steep_angle > 35.0
