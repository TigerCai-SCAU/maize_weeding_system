import math

import pytest

from oid_wheel_encoder.bench_virtual_odom import (
    mapped_translation,
    quaternion_xyzw,
    vector3,
)


def test_forward_distance_maps_to_negative_camera_init_axis_2():
    translation = mapped_translation(
        distance_m=1.25,
        zero_distance_m=0.25,
        origin_xyz=(0.0, 0.0, 0.0),
        translation_per_meter=(0.0, 0.0, -1.0),
    )
    assert translation == pytest.approx((0.0, 0.0, -1.0))


def test_origin_and_scale_are_applied_per_axis():
    translation = mapped_translation(
        distance_m=2.0,
        zero_distance_m=0.5,
        origin_xyz=(1.0, 2.0, 3.0),
        translation_per_meter=(0.0, 2.0, -1.0),
    )
    assert translation == pytest.approx((1.0, 5.0, 1.5))


def test_quaternion_is_normalized():
    assert quaternion_xyzw((0.0, 0.0, 0.0, 2.0)) == pytest.approx(
        (0.0, 0.0, 0.0, 1.0)
    )


@pytest.mark.parametrize(
    "values",
    [
        (1.0, 2.0),
        (1.0, 2.0, math.inf),
    ],
)
def test_vector3_rejects_invalid_values(values):
    with pytest.raises(ValueError):
        vector3(values, "test_vector")
