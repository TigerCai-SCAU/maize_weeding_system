import math


READ_POSITION = 0x01


def make_read_position_request(device_id: int) -> bytes:
    if not 1 <= device_id <= 255:
        raise ValueError("device_id must be in [1, 255]")
    return bytes((0x04, device_id, READ_POSITION, 0x00))


def parse_position_response(data: bytes, device_id: int) -> int | None:
    if len(data) < 7:
        return None
    if data[0] != 0x07 or data[1] != device_id or data[2] != READ_POSITION:
        return None
    return int.from_bytes(data[3:7], byteorder="little", signed=False)


def wrapped_delta(current: int, previous: int, counts_per_rev: int) -> int:
    if counts_per_rev <= 1:
        raise ValueError("counts_per_rev must be greater than one")
    half = counts_per_rev // 2
    return (current - previous + half) % counts_per_rev - half


def counts_to_distance(
    delta_counts: int,
    counts_per_rev: int,
    wheel_diameter_m: float,
    encoder_revs_per_wheel_rev: float,
    direction: int,
) -> float:
    if counts_per_rev <= 0 or wheel_diameter_m <= 0.0 or encoder_revs_per_wheel_rev <= 0.0:
        raise ValueError("encoder geometry must be positive")
    if direction not in (-1, 1):
        raise ValueError("direction must be -1 or 1")
    wheel_revolutions = delta_counts / counts_per_rev / encoder_revs_per_wheel_rev
    return direction * wheel_revolutions * math.pi * wheel_diameter_m
