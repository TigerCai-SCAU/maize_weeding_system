import numpy as np

from seedling_path_planning.planner_core import (
    PlannerConfig,
    analyze_rows,
    interpolate_terrain_height,
    lateral_travel_height_to_world,
    minimum_seedling_clearance,
    plan_dual_arm_s,
    plan_coverage,
)


def irregular_two_rows():
    # Includes jitter, a missing 0.20 m slot on the left, and a close/re-sown
    # pair on the right. The standards must diagnose but not regularize them.
    return np.asarray(
        [
            [-0.275, 0.00],
            [-0.281, 0.20],
            [-0.269, 0.60],
            [-0.278, 0.80],
            [0.274, 0.01],
            [0.281, 0.21],
            [0.270, 0.29],
            [0.277, 0.42],
            [0.272, 0.61],
            [0.280, 0.82],
        ],
        dtype=np.float64,
    )


def test_irregular_rows_keep_real_geometry_and_report_sowing_anomalies():
    config = PlannerConfig(
        expected_row_spacing=0.55,
        expected_plant_spacing=0.20,
        row_cluster_threshold=0.0,
    )
    rows = analyze_rows(irregular_two_rows(), config)
    assert len(rows) == 2
    assert [len(row.point_indices) for row in rows] == [4, 6]
    assert rows[0].missing_slots >= 1
    assert rows[1].close_pairs >= 1
    measured = rows[1].lateral_at(0.4) - rows[0].lateral_at(0.4)
    assert 0.52 < measured < 0.58


def test_coverage_path_stays_outside_effective_protection_radius():
    config = PlannerConfig(
        expected_row_spacing=0.55,
        expected_plant_spacing=0.20,
        row_cluster_threshold=0.0,
        protection_radius=0.08,
        safety_margin=0.015,
        coverage_spacing=0.07,
        path_resolution=0.02,
        lateral_work_margin=0.08,
    )
    result = plan_coverage(irregular_two_rows(), config)
    clearance = minimum_seedling_clearance(result.path_lt, result.seedling_lt)
    assert len(result.path_lt) > 10
    assert clearance + 1e-9 >= config.obstacle_radius


def test_dual_arm_s_paths_are_forward_only_and_one_per_row():
    config = PlannerConfig(
        expected_row_spacing=0.55,
        expected_plant_spacing=0.20,
        row_cluster_threshold=0.0,
        protection_radius=0.08,
        safety_margin=0.015,
        path_resolution=0.01,
        travel_work_margin=0.12,
        s_sweep_offset=0.11,
    )
    result = plan_dual_arm_s(irregular_two_rows(), config)
    assert len(result.rows) == 2
    assert len(result.arm_paths_lt) == 2
    for path in result.arm_paths_lt:
        assert len(path) > 10
        assert np.all(np.diff(path[:, 1]) > 0.0)
        assert (
            minimum_seedling_clearance(path, result.seedling_lt) + 1e-9
            >= config.obstacle_radius
        )
    assert np.median(result.arm_paths_lt[0][:, 0]) < 0.0
    assert np.median(result.arm_paths_lt[1][:, 0]) > 0.0


def test_close_resown_pair_stays_on_same_bypass_side():
    config = PlannerConfig(
        row_cluster_threshold=0.0,
        path_resolution=0.005,
        travel_work_margin=0.12,
        s_sweep_offset=0.11,
    )
    seedlings = irregular_two_rows()
    result = plan_dual_arm_s(seedlings, config)
    right_path = result.arm_paths_lt[1]
    right_row = result.rows[1]
    close_points = seedlings[right_row.point_indices]
    close_points = close_points[np.argsort(close_points[:, 1])]
    pair = close_points[1:3]
    offsets = []
    for lateral, travel in pair:
        path_index = int(np.argmin(np.abs(right_path[:, 1] - travel)))
        offsets.append(right_path[path_index, 0] - lateral)
    assert offsets[0] * offsets[1] > 0.0


def test_missing_predictions_are_diagnostics_not_obstacles():
    config = PlannerConfig(row_cluster_threshold=0.0)
    seedlings = irregular_two_rows()
    result = plan_dual_arm_s(seedlings, config)
    predicted = [
        point for row in result.rows for point in row.predicted_missing_lt
    ]
    assert predicted
    assert result.seedling_lt.shape == seedlings.shape


def test_axis_mapping_supports_bench_travel_z_height_x():
    world = lateral_travel_height_to_world(
        lateral=-0.30,
        travel=1.25,
        height=1.04,
        lateral_axis=1,
        travel_axis=2,
        height_axis=0,
    )
    assert np.allclose(world, [1.04, -0.30, 1.25])


def test_terrain_height_uses_nearby_points_and_safe_fallback():
    terrain = np.asarray(
        [
            [-0.10, 0.00, 1.00],
            [0.00, 0.00, 1.02],
            [0.10, 0.00, 1.04],
        ]
    )
    height = interpolate_terrain_height(
        0.0, 0.01, terrain, search_radius=0.15, fallback_height=0.5
    )
    assert 1.00 < height < 1.04
    assert (
        interpolate_terrain_height(
            2.0, 2.0, terrain, search_radius=0.10, fallback_height=0.5
        )
        == 0.5
    )
