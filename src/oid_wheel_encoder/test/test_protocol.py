import math

from oid_wheel_encoder.protocol import (
    counts_to_distance,
    make_read_position_request,
    parse_position_response,
    wrapped_delta,
)


def test_protocol_and_little_endian_position() -> None:
    assert make_read_position_request(1) == bytes((4, 1, 1, 0))
    assert parse_position_response(bytes((7, 1, 1, 0x45, 0x23, 0x01, 0)), 1) == 0x00012345
    assert parse_position_response(bytes((4, 1, 1, 0)), 1) is None


def test_wrap_and_63mm_wheel_distance() -> None:
    assert wrapped_delta(3, 32766, 32768) == 5
    assert wrapped_delta(32766, 3, 32768) == -5
    distance = counts_to_distance(32768, 32768, 0.063, 1.0, 1)
    assert math.isclose(distance, math.pi * 0.063)
