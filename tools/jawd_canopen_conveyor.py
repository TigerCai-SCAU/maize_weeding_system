#!/usr/bin/env python3
"""Minimal JMC JAWD CANopen conveyor bring-up tool (Linux SocketCAN).

The JAWD CAN-bus variant uses CiA 402.  Axis node IDs are normally 1 and 2.
Always run ``probe`` before ``run`` and begin with the belt unloaded.

Examples:
  python3 jawd_canopen_conveyor.py --interface can1 --nodes 1,2 probe
  python3 jawd_canopen_conveyor.py --interface can1 --nodes 1,2 run 0.05 --seconds 3
  python3 jawd_canopen_conveyor.py --interface can1 --nodes 1,2 stop
"""

import argparse
import math
import socket
import struct
import time


CAN_FRAME = struct.Struct("=IB3x8s")
SDO_ABORTS = {
    0x05040000: "SDO protocol timeout",
    0x06010000: "unsupported access",
    0x06020000: "object does not exist",
    0x06090030: "value out of range",
    0x08000000: "general error",
}


def belt_mps_to_motor_rpm(speed_mps, ratio=4.0, radius_m=0.04):
    return speed_mps * 60.0 * ratio / (2.0 * math.pi * radius_m)


def rpm_to_jmc_speed(rpm):
    """JMC 0x6081 uses 0.1 rps: raw 100 means 10 rps (600 rpm)."""
    return round(rpm / 6.0)


def rpm_s_to_jmc_accel(rpm_s):
    """JMC 0x6083/0x6084 use 0.1 rps/s."""
    return round(rpm_s / 6.0)


class Canopen:
    def __init__(self, interface, timeout=2.0, inter_request_delay=0.2):
        self.sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        self.sock.settimeout(timeout)
        self.sock.bind((interface,))
        self.inter_request_delay = inter_request_delay

    def close(self):
        self.sock.close()

    def send(self, can_id, data=b""):
        if len(data) > 8:
            raise ValueError("classic CAN payload exceeds 8 bytes")
        self.sock.send(CAN_FRAME.pack(can_id, len(data), data.ljust(8, b"\0")))

    def recv(self):
        can_id, size, data = CAN_FRAME.unpack(self.sock.recv(CAN_FRAME.size))
        return can_id & 0x7FF, data[:size]

    def nmt_start(self, node=0):
        self.send(0, bytes((1, node)))

    def _sdo_reply(self, node, index, subindex):
        deadline = time.monotonic() + self.sock.gettimeout()
        while time.monotonic() < deadline:
            try:
                can_id, data = self.recv()
            except TimeoutError:
                break
            if can_id != 0x580 + node or len(data) < 4:
                continue
            if data[1:4] != struct.pack("<HB", index, subindex):
                continue
            if data[0] == 0x80:
                code = int.from_bytes(data[4:8], "little")
                raise RuntimeError(
                    f"node {node} SDO 0x{index:04X}:{subindex} aborted: "
                    f"0x{code:08X} {SDO_ABORTS.get(code, '')}".rstrip()
                )
            return data
        raise TimeoutError(f"node {node} did not answer SDO 0x{index:04X}:{subindex}")

    def read(self, node, index, subindex=0, signed=False):
        self.send(0x600 + node, b"\x40" + struct.pack("<HB", index, subindex) + b"\0" * 4)
        data = self._sdo_reply(node, index, subindex)
        command = data[0]
        if command & 0xE0 != 0x40 or not command & 0x02:
            raise RuntimeError("segmented SDO upload is not supported by this tool")
        size = 4 - ((command >> 2) & 3) if command & 0x01 else 4
        value = int.from_bytes(data[4 : 4 + size], "little", signed=signed)
        time.sleep(self.inter_request_delay)
        return value

    def write(self, node, index, value, size, subindex=0, signed=False):
        commands = {1: 0x2F, 2: 0x2B, 3: 0x27, 4: 0x23}
        payload = value.to_bytes(size, "little", signed=signed).ljust(4, b"\0")
        self.send(
            0x600 + node,
            bytes((commands[size],)) + struct.pack("<HB", index, subindex) + payload,
        )
        data = self._sdo_reply(node, index, subindex)
        if data[0] != 0x60:
            raise RuntimeError(f"node {node} rejected SDO write 0x{index:04X}:{subindex}")
        time.sleep(self.inter_request_delay)


def state_name(statusword):
    state = statusword & 0x000F
    return {
        0x0000: "not ready",
        0x0001: "initialized",
        0x0003: "powered",
        0x0007: "operation enabled",
        0x000F: "fault reaction active",
        0x0008: "fault",
    }.get(state, f"unknown(0x{statusword:04X})")


def wait_state(bus, node, expected, timeout=1.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        status = bus.read(node, 0x6041)
        if status & 0x000F == expected:
            return status
        time.sleep(0.05)
    raise RuntimeError(f"node {node} state is {state_name(status)}, expected 0x{expected:02X}")


def enable_velocity_mode(bus, node, accel_rpm_s, decel_rpm_s):
    status = bus.read(node, 0x6041)
    if status & 0x000F in (0x0008, 0x000F):
        bus.write(node, 0x6040, 0x0080, 2)
        time.sleep(0.1)
    # JMC's documented CiA 402 transition is initialize -> power -> enable.
    for controlword, expected in ((0x0001, 0x0001), (0x0003, 0x0003), (0x000F, 0x0007)):
        bus.write(node, 0x6040, controlword, 2)
        wait_state(bus, node, expected)
    bus.write(node, 0x6040, 0x010F, 2)  # pause while parameters are changed
    bus.write(node, 0x6060, 3, 1, signed=True)
    accel = rpm_s_to_jmc_accel(accel_rpm_s)
    decel = rpm_s_to_jmc_accel(decel_rpm_s)
    if not 1 <= accel <= 0xFFFF or not 1 <= decel <= 0xFFFF:
        raise ValueError("acceleration/deceleration is outside the JMC UNSIGNED16 range")
    bus.write(node, 0x6083, accel, 2)
    bus.write(node, 0x6084, decel, 2)


def stop_nodes(bus, nodes, disable=True, settle_timeout=3.0):
    """Ramp targets to zero, then disable only after both axes have stopped."""
    for node in nodes:
        try:
            # Keep operation enabled so profile deceleration (0x6084) is used.
            bus.write(node, 0x6081, 0, 4, signed=True)
        except (TimeoutError, RuntimeError) as exc:
            print(f"node {node}: zero-speed command failed: {exc}")
            return False

    deadline = time.monotonic() + settle_timeout
    readings = {}
    while time.monotonic() < deadline:
        try:
            readings = {
                node: bus.read(node, 0x606C, signed=True)
                for node in nodes
            }
        except (TimeoutError, RuntimeError) as exc:
            print(f"soft-stop speed check failed: {exc}")
            return False
        if all(abs(rpm) <= 2 for rpm in readings.values()):
            break
        time.sleep(0.2)
    else:
        print(
            "soft-stop timeout: "
            + "  ".join(f"node {node}: {rpm} rpm" for node, rpm in readings.items())
        )
        return False

    print(
        "soft-stop complete: "
        + "  ".join(f"node {node}: {rpm} rpm" for node, rpm in readings.items())
    )
    if disable:
        for node in nodes:
            try:
                bus.write(node, 0x6040, 0x0007, 2)
            except (TimeoutError, RuntimeError) as exc:
                print(f"node {node}: disable failed: {exc}")
                return False
    return True


def parse_nodes(text):
    nodes = [int(item) for item in text.split(",")]
    if not nodes or any(node < 1 or node > 127 for node in nodes):
        raise argparse.ArgumentTypeError("node IDs must be comma-separated values from 1 to 127")
    return nodes


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--interface", default="can1")
    parser.add_argument("--nodes", type=parse_nodes, default=parse_nodes("1,2"))
    parser.add_argument("--ratio", type=float, default=4.0, help="motor:roller reduction ratio")
    parser.add_argument("--radius", type=float, default=0.04, help="roller radius in metres")
    parser.add_argument("--max-rpm", type=int, default=3000)
    parser.add_argument("--sdo-timeout", type=float, default=2.0)
    parser.add_argument("--sdo-delay", type=float, default=0.2)
    parser.add_argument("--node-delay", type=float, default=0.5)
    sub = parser.add_subparsers(dest="command", required=True)
    sub.add_parser("probe", help="read state and speed without enabling the motors")
    run = sub.add_parser("run", help="run both conveyor motors at a belt speed")
    run.add_argument("speed", type=float, help="signed belt speed in m/s")
    run.add_argument("--seconds", type=float, default=0, help="0 means run until Ctrl-C")
    run.add_argument("--accel", type=int, default=300, help="motor acceleration in rpm/s")
    run.add_argument("--decel", type=int, default=60, help="motor deceleration in rpm/s")
    sub.add_parser("stop", help="set zero speed and disable both axes")
    args = parser.parse_args()

    if (
        args.ratio <= 0
        or args.radius <= 0
        or args.max_rpm <= 0
        or args.sdo_timeout <= 0
        or args.sdo_delay < 0
        or args.node_delay < 0
    ):
        parser.error("geometry, max-rpm, timeout, and command delays are invalid")

    bus = Canopen(
        args.interface,
        timeout=args.sdo_timeout,
        inter_request_delay=args.sdo_delay,
    )
    try:
        bus.nmt_start()
        time.sleep(args.node_delay)
        if args.command == "probe":
            for node in args.nodes:
                status = bus.read(node, 0x6041)
                rpm = bus.read(node, 0x606C, signed=True)
                print(f"node {node}: {state_name(status)}, actual={rpm} rpm")
            return
        if args.command == "stop":
            stop_nodes(bus, args.nodes)
            print("target speed is zero; axes disabled")
            return

        rpm = round(belt_mps_to_motor_rpm(args.speed, args.ratio, args.radius))
        if abs(rpm) > args.max_rpm:
            parser.error(
                f"requested belt speed needs {rpm} rpm, above --max-rpm {args.max_rpm}"
            )
        target = rpm_to_jmc_speed(rpm)
        print(f"belt={args.speed:.3f} m/s -> motor target={rpm} rpm (0x6081={target})")
        for node in args.nodes:
            enable_velocity_mode(bus, node, args.accel, args.decel)
            time.sleep(args.node_delay)
        for node in args.nodes:
            bus.write(node, 0x6081, target, 4, signed=True)
        for node in args.nodes:
            bus.write(node, 0x6040, 0x000F, 2)

        started = time.monotonic()
        while not args.seconds or time.monotonic() - started < args.seconds:
            readings = [bus.read(node, 0x606C, signed=True) for node in args.nodes]
            print("  ".join(f"node {n}: {v} rpm" for n, v in zip(args.nodes, readings)))
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("\nCtrl-C: stopping")
    except (OSError, TimeoutError, RuntimeError, ValueError) as exc:
        parser.exit(1, f"error: {exc}\n")
    finally:
        if args.command == "run":
            stop_nodes(bus, args.nodes)
        bus.close()


if __name__ == "__main__":
    # Physical conversion check: 1 roller revolution/s at 4:1 requires 240 motor rpm.
    assert round(belt_mps_to_motor_rpm(2 * math.pi * 0.04)) == 240
    assert rpm_to_jmc_speed(600) == 100
    assert rpm_s_to_jmc_accel(6000) == 1000
    main()
