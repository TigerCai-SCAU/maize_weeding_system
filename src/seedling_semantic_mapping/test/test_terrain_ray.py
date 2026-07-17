import math

import numpy as np

from seedling_semantic_mapping.terrain_ray import (
    TerrainHeightMap,
    interpolate_pose,
    quaternion_to_rotation,
)


def test_pose_interpolation_uses_linear_translation_and_slerp():
    pose = interpolate_pose(
        0.0,
        np.array([0.0, 0.0, 0.0]),
        np.array([0.0, 0.0, 0.0, 1.0]),
        2.0,
        np.array([2.0, 4.0, 6.0]),
        np.array([0.0, 0.0, 1.0, 0.0]),
        1.0,
    )
    np.testing.assert_allclose(pose.translation, [1.0, 2.0, 3.0])
    rotated_x = quaternion_to_rotation(pose.quaternion_xyzw) @ [1.0, 0.0, 0.0]
    np.testing.assert_allclose(rotated_x, [0.0, 1.0, 0.0], atol=1e-7)


def test_ray_hits_flat_axis_zero_terrain_without_nearby_lidar_pixel():
    horizontal = np.arange(-0.5, 0.51, 0.05)
    points = np.array(
        [[1.0, y, z] for y in horizontal for z in horizontal],
        dtype=np.float64,
    )
    terrain = TerrainHeightMap(points, 0.05, 0, (1, 2), 0.08, 1)
    hit = terrain.intersect_ray(
        origin=np.array([0.0, 0.0, 0.0]),
        direction=np.array([1.0, 0.1, 0.1]),
        min_range_m=0.1,
        max_range_m=3.0,
        step_m=0.025,
        height_tolerance_m=0.01,
        max_valid_gap_m=0.15,
    )
    assert hit is not None
    assert math.isclose(hit[0], 1.0, abs_tol=0.02)


def test_ray_hits_sloped_axis_zero_terrain():
    horizontal_y = np.arange(-1.0, 1.01, 0.02)
    horizontal_z = np.arange(-0.2, 0.21, 0.02)
    points = np.array(
        [
            [1.0 + 0.5 * y, y, z]
            for y in horizontal_y
            for z in horizontal_z
        ],
        dtype=np.float64,
    )
    terrain = TerrainHeightMap(points, 0.02, 0, (1, 2), 0.04, 1)
    hit = terrain.intersect_ray(
        origin=np.zeros(3),
        direction=np.array([1.0, 0.5, 0.0]),
        min_range_m=0.1,
        max_range_m=3.0,
        step_m=0.01,
        height_tolerance_m=0.005,
        max_valid_gap_m=0.05,
    )
    assert hit is not None
    # Ray: x = 2y. Terrain: x = 1 + 0.5y. Their intersection is x = 4/3.
    assert math.isclose(hit[0], 4.0 / 3.0, abs_tol=0.03)
    assert math.isclose(hit[0], 1.0 + 0.5 * hit[1], abs_tol=0.02)


def test_ray_rejects_unobserved_terrain_gap():
    points = np.array(
        [[1.0, y, z] for y in (1.0, 1.05) for z in (1.0, 1.05)],
        dtype=np.float64,
    )
    terrain = TerrainHeightMap(points, 0.05, 0, (1, 2), 0.08, 1)
    hit = terrain.intersect_ray(
        origin=np.zeros(3),
        direction=np.array([1.0, 0.0, 0.0]),
        min_range_m=0.1,
        max_range_m=3.0,
        step_m=0.025,
        height_tolerance_m=0.01,
        max_valid_gap_m=0.15,
    )
    assert hit is None
