# -*- coding: utf-8 -*-
import math
import time
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node

from trajectory_msgs.msg import JointTrajectory
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PointStamped
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from std_msgs.msg import Bool

from .h5u_modbus import H5UModbus
from . import registers as reg


# 来自 H5U CSP 轨迹缓冲低公共变量表
M_CMD_SERVO_ON = 100
M_CMD_RESET = 101
M_CMD_HOME = 102
M_CMD_STOP = 103
M_CMD_JOG_MODE = 104
M_CMD_SWEEP_START = 105
M_GUIDE_READY = 106

M_HOME_DONE = 203

M_PC_ENABLE = 500

M_PLC_READY = 520
M_PLC_BUSY = 521
M_PLC_ERROR = 522
M_PLC_TIMEOUT = 523
M_PLC_TRAJ_VALID = 524
M_TRACK_ENABLE = 525
M_TRACK_USE_SAFE = 526


def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


class TrajectoryBuffer:
    """Latest trajectory, positions in meters: tool_y/tool_z."""

    def __init__(self):
        self.msg: Optional[JointTrajectory] = None
        self.recv_mono_time: float = 0.0
        self.y_index: int = 0
        self.z_index: int = 1

    def set(self, msg: JointTrajectory, node: Node) -> bool:
        if len(msg.points) < 2:
            node.get_logger().warn("收到轨迹点数 < 2，忽略")
            return False

        names = list(msg.joint_names)
        if "tool_y" in names and "tool_z" in names:
            self.y_index = names.index("tool_y")
            self.z_index = names.index("tool_z")
        elif "y" in names and "z" in names:
            self.y_index = names.index("y")
            self.z_index = names.index("z")
        else:
            self.y_index = 0
            self.z_index = 1

        for p in msg.points:
            if len(p.positions) <= max(self.y_index, self.z_index):
                node.get_logger().warn("轨迹点 positions 长度不足，忽略")
                return False

        self.msg = msg
        self.recv_mono_time = time.monotonic()
        return True

    def sample(self, t_from_recv: float) -> Tuple[float, float]:
        if self.msg is None:
            raise RuntimeError("No trajectory")

        pts = self.msg.points
        t_first = pts[0].time_from_start.sec + pts[0].time_from_start.nanosec * 1e-9
        if t_from_recv <= t_first:
            p = pts[0]
            return p.positions[self.y_index], p.positions[self.z_index]

        for i in range(len(pts) - 1):
            t0 = pts[i].time_from_start.sec + pts[i].time_from_start.nanosec * 1e-9
            t1 = pts[i + 1].time_from_start.sec + pts[i + 1].time_from_start.nanosec * 1e-9
            if t0 <= t_from_recv <= t1:
                a = 0.0 if t1 <= t0 else (t_from_recv - t0) / (t1 - t0)
                y0 = pts[i].positions[self.y_index]
                z0 = pts[i].positions[self.z_index]
                y1 = pts[i + 1].positions[self.y_index]
                z1 = pts[i + 1].positions[self.z_index]
                return y0 + a * (y1 - y0), z0 + a * (z1 - z0)

        p = pts[-1]
        return p.positions[self.y_index], p.positions[self.z_index]


class H5UCspBridge(Node):
    def __init__(self):
        super().__init__("h5u_csp_bridge")

        self.declare_parameter("plc_ip", "192.168.1.88")
        self.declare_parameter("plc_port", 502)
        self.declare_parameter("unit", 1)
        self.declare_parameter("addr_offset", 0)
        self.declare_parameter("coil_addr_offset", 0)
        self.declare_parameter("word_order", "lo_hi")

        self.declare_parameter("write_rate_hz", 20.0)
        self.declare_parameter("point_count", 64)
        self.declare_parameter("point_dt_ticks", 5)
        self.declare_parameter("lookahead_ticks", 50)
        self.declare_parameter("timeout_ticks", 125)
        self.declare_parameter("plc_tick_sec", 0.004)

        self.declare_parameter("trajectory_timeout_sec", 0.30)
        self.declare_parameter("fallback_y_m", 0.0)
        self.declare_parameter("fallback_z_m", -0.10)
        self.declare_parameter("use_fallback_when_no_traj", False)

        self.declare_parameter("y_limit_m", 0.20)
        self.declare_parameter("z_min_m", -0.40)
        self.declare_parameter("z_max_m", -0.02)

        # 一键启动时是否顺手置位这两个
        self.declare_parameter("start_sets_servo_on", True)
        self.declare_parameter("start_sets_guide_ready", True)

        self.plc_ip = self.get_parameter("plc_ip").value
        self.plc_port = int(self.get_parameter("plc_port").value)
        self.unit = int(self.get_parameter("unit").value)
        self.addr_offset = int(self.get_parameter("addr_offset").value)
        self.coil_addr_offset = int(self.get_parameter("coil_addr_offset").value)
        self.word_order = str(self.get_parameter("word_order").value)

        self.write_rate_hz = float(self.get_parameter("write_rate_hz").value)
        self.point_count = int(self.get_parameter("point_count").value)
        self.point_dt_ticks = int(self.get_parameter("point_dt_ticks").value)
        self.lookahead_ticks = int(self.get_parameter("lookahead_ticks").value)
        self.timeout_ticks = int(self.get_parameter("timeout_ticks").value)
        self.plc_tick_sec = float(self.get_parameter("plc_tick_sec").value)
        self.point_dt_sec = self.point_dt_ticks * self.plc_tick_sec

        self.trajectory_timeout_sec = float(self.get_parameter("trajectory_timeout_sec").value)
        self.fallback_y_m = float(self.get_parameter("fallback_y_m").value)
        self.fallback_z_m = float(self.get_parameter("fallback_z_m").value)
        self.use_fallback_when_no_traj = bool(self.get_parameter("use_fallback_when_no_traj").value)

        self.y_limit_m = abs(float(self.get_parameter("y_limit_m").value))
        self.z_min_m = float(self.get_parameter("z_min_m").value)
        self.z_max_m = float(self.get_parameter("z_max_m").value)

        self.start_sets_servo_on = bool(self.get_parameter("start_sets_servo_on").value)
        self.start_sets_guide_ready = bool(self.get_parameter("start_sets_guide_ready").value)

        self.plc = H5UModbus(
            self.plc_ip,
            port=self.plc_port,
            unit=self.unit,
            addr_offset=self.addr_offset,
            word_order=self.word_order,
            coil_addr_offset=self.coil_addr_offset,
        )

        if not self.plc.connect():
            raise RuntimeError(f"无法连接 H5U: {self.plc_ip}:{self.plc_port}")

        self.get_logger().info(
            f"已连接 H5U {self.plc_ip}:{self.plc_port}, word_order={self.word_order}, "
            f"D偏移={self.addr_offset}, M偏移={self.coil_addr_offset}, write_rate={self.write_rate_hz}Hz"
        )

        self.traj = TrajectoryBuffer()
        self.seq = 0
        self.alive = 0
        self.bridge_enable = True

        self.create_subscription(JointTrajectory, "trajectory_yz", self.on_trajectory, 10)
        self.create_subscription(Bool, "bridge_enable", self.on_bridge_enable, 10)

        # ROS 命令话题 -> H5U M 位
        self.create_subscription(Bool, "/weedarm/cmd_start", self.on_cmd_start, 10)
        self.create_subscription(Bool, "/weedarm/cmd_servo_on",
                                 lambda m: self.write_m(M_CMD_SERVO_ON, m.data, "Cmd_ServoOn"), 10)
        self.create_subscription(Bool, "/weedarm/cmd_sweep_start",
                                 lambda m: self.write_m(M_CMD_SWEEP_START, m.data, "Cmd_SweepStart"), 10)
        self.create_subscription(Bool, "/weedarm/cmd_pc_enable",
                                 lambda m: self.write_m(M_PC_ENABLE, m.data, "PC_Enable"), 10)
        self.create_subscription(Bool, "/weedarm/cmd_guide_ready",
                                 lambda m: self.write_m(M_GUIDE_READY, m.data, "Guide_Ready"), 10)
        self.create_subscription(Bool, "/weedarm/cmd_stop",
                                 lambda m: self.write_m(M_CMD_STOP, m.data, "Cmd_Stop"), 10)
        self.create_subscription(Bool, "/weedarm/cmd_home",
                                 lambda m: self.write_m(M_CMD_HOME, m.data, "Cmd_Home"), 10)
        self.create_subscription(Bool, "/weedarm/cmd_reset",
                                 lambda m: self.write_m(M_CMD_RESET, m.data, "Cmd_Reset"), 10)

        self.pub_joint = self.create_publisher(JointState, "joint_state_feedback", 10)
        self.pub_tool = self.create_publisher(PointStamped, "tool_yz_feedback", 10)
        self.pub_target = self.create_publisher(PointStamped, "/weedarm/target_yz_feedback", 10)
        self.pub_error = self.create_publisher(PointStamped, "/weedarm/tracking_error", 10)
        self.pub_diag = self.create_publisher(DiagnosticArray, "diagnostics", 10)

        self.plc.write_int(reg.D_PC_POINT_COUNT, self.point_count)
        self.plc.write_int(reg.D_PC_POINT_DT_TICKS, self.point_dt_ticks)
        self.plc.write_int(reg.D_LOOKAHEAD_TICKS, self.lookahead_ticks)
        self.plc.write_int(reg.D_TIMEOUT_TICKS, self.timeout_ticks)

        period = 1.0 / max(1.0, self.write_rate_hz)
        self.timer = self.create_timer(period, self.on_timer)

    def destroy_node(self):
        try:
            self.plc.close()
        finally:
            super().destroy_node()

    def write_m(self, m_addr: int, value: bool, name: str):
        try:
            self.plc.write_coil(m_addr, bool(value))
            self.get_logger().info(f"{name}={'ON' if value else 'OFF'} 写入 M{m_addr}")
        except Exception as exc:
            self.get_logger().error(f"写 {name} M{m_addr} 失败: {exc}")

    def on_cmd_start(self, msg: Bool):
        if bool(msg.data):
            # 不强制写 Home_Done；必须由 PLC 回零逻辑产生
            try:
                self.plc.write_coil(M_CMD_STOP, False)
                if self.start_sets_servo_on:
                    self.plc.write_coil(M_CMD_SERVO_ON, True)
                if self.start_sets_guide_ready:
                    self.plc.write_coil(M_GUIDE_READY, True)
                self.plc.write_coil(M_PC_ENABLE, True)
                self.plc.write_coil(M_CMD_SWEEP_START, True)
                self.get_logger().info(
                    "作业启动：Cmd_Stop=OFF, ServoOn/GuideReady可选ON, PC_Enable=ON, Cmd_SweepStart=ON"
                )
            except Exception as exc:
                self.get_logger().error(f"作业启动写M失败: {exc}")
        else:
            try:
                self.plc.write_coil(M_CMD_SWEEP_START, False)
                self.plc.write_coil(M_PC_ENABLE, False)
                self.get_logger().info("作业停止：Cmd_SweepStart=OFF, PC_Enable=OFF，伺服保持使能")
            except Exception as exc:
                self.get_logger().error(f"作业停止写M失败: {exc}")

    def on_bridge_enable(self, msg: Bool):
        self.bridge_enable = bool(msg.data)
        self.get_logger().info(f"bridge_enable={self.bridge_enable}")

    def on_trajectory(self, msg: JointTrajectory):
        if self.traj.set(msg, self):
            self.get_logger().debug(f"收到 Y/Z 轨迹，点数={len(msg.points)}")

    def _current_tool_m(self) -> Tuple[float, float]:
        y = self.plc.read_dint(reg.D_PLC_TOOL_Y) / 1000.0 / 1000.0
        z = self.plc.read_dint(reg.D_PLC_TOOL_Z) / 1000.0 / 1000.0
        return y, z

    def _make_points_mm(self) -> List[Tuple[float, float]]:
        now = time.monotonic()
        has_recent_traj = (
            self.traj.msg is not None
            and (now - self.traj.recv_mono_time) <= self.trajectory_timeout_sec
        )

        if not has_recent_traj and not self.use_fallback_when_no_traj:
            # 没有新轨迹时保持当前位置，避免突然回安全点
            y_m, z_m = self._current_tool_m()
            return [(y_m * 1000.0, z_m * 1000.0) for _ in range(self.point_count)]

        points = []
        for i in range(self.point_count):
            future_t = self.lookahead_ticks * self.plc_tick_sec + i * self.point_dt_sec
            if has_recent_traj:
                t_from_recv = (now - self.traj.recv_mono_time) + future_t
                y_m, z_m = self.traj.sample(t_from_recv)
            else:
                y_m, z_m = self.fallback_y_m, self.fallback_z_m

            y_m = clamp(y_m, -self.y_limit_m, self.y_limit_m)
            z_m = clamp(z_m, self.z_min_m, self.z_max_m)
            points.append((y_m * 1000.0, z_m * 1000.0))
        return points

    def on_timer(self):
        if not self.bridge_enable:
            return

        try:
            active_buf = self.plc.read_int(reg.D_PLC_ACTIVE_BUFFER_ID)
            ctrl_tick = self.plc.read_dint(reg.D_PLC_CTRL_TICK)
            write_buf = 1 if active_buf == 0 else 0
            base = reg.D_TRAJ_B_BASE if write_buf == 1 else reg.D_TRAJ_A_BASE

            points_mm = self._make_points_mm()
            self.plc.write_traj_block(base, points_mm)

            start_tick = ctrl_tick + self.lookahead_ticks
            self.plc.write_int(reg.D_PC_BUFFER_ID, write_buf)
            self.plc.write_int(reg.D_PC_POINT_COUNT, self.point_count)
            self.plc.write_int(reg.D_PC_POINT_DT_TICKS, self.point_dt_ticks)
            self.plc.write_dint(reg.D_PC_START_TICK, start_tick)

            self.alive = (self.alive + 1) & 0x7FFF
            self.plc.write_int(reg.D_PC_ALIVE, self.alive)

            self.seq = (self.seq + 1) & 0x7FFF
            self.plc.write_int(reg.D_PC_TRAJ_SEQ, self.seq)

            self._publish_feedback()

        except Exception as exc:
            self.get_logger().error(f"H5U bridge 写入/读取失败: {exc}")

    def _read_m_safe(self, addr: int) -> str:
        try:
            return "1" if self.plc.read_coil(addr) else "0"
        except Exception:
            return "?"

    def _publish_feedback(self):
        now_msg = self.get_clock().now().to_msg()

        state = self.plc.read_int(reg.D_PLC_STATE)
        alarm = self.plc.read_int(reg.D_PLC_ALARM)
        ack = self.plc.read_int(reg.D_PLC_ACK_SEQ)
        idx = self.plc.read_int(reg.D_PLC_TRACK_INDEX)

        pitch_deg = self.plc.read_dint(reg.D_PLC_PITCH_ACTUAL) / 1000.0
        yaw_deg = self.plc.read_dint(reg.D_PLC_YAW_ACTUAL) / 1000.0

        tool_y_m = self.plc.read_dint(reg.D_PLC_TOOL_Y) / 1000.0 / 1000.0
        tool_z_m = self.plc.read_dint(reg.D_PLC_TOOL_Z) / 1000.0 / 1000.0

        target_y_m = self.plc.read_dint(reg.D_PLC_TARGET_Y) / 1000.0 / 1000.0
        target_z_m = self.plc.read_dint(reg.D_PLC_TARGET_Z) / 1000.0 / 1000.0

        err_y_m = target_y_m - tool_y_m
        err_z_m = target_z_m - tool_z_m

        js = JointState()
        js.header.stamp = now_msg
        js.name = ["pitch", "yaw"]
        js.position = [math.radians(pitch_deg), math.radians(yaw_deg)]
        self.pub_joint.publish(js)

        actual = PointStamped()
        actual.header.stamp = now_msg
        actual.header.frame_id = "arm_base"
        actual.point.x = 0.0
        actual.point.y = tool_y_m
        actual.point.z = tool_z_m
        self.pub_tool.publish(actual)

        target = PointStamped()
        target.header.stamp = now_msg
        target.header.frame_id = "arm_base"
        target.point.x = 0.0
        target.point.y = target_y_m
        target.point.z = target_z_m
        self.pub_target.publish(target)

        err = PointStamped()
        err.header.stamp = now_msg
        err.header.frame_id = "arm_base"
        err.point.x = 0.0
        err.point.y = err_y_m
        err.point.z = err_z_m
        self.pub_error.publish(err)

        diag = DiagnosticArray()
        diag.header.stamp = now_msg
        st = DiagnosticStatus()
        st.name = "weedarm_h5u_bridge"
        st.hardware_id = self.plc_ip
        st.level = DiagnosticStatus.OK if alarm == 0 else DiagnosticStatus.WARN
        st.message = "OK" if alarm == 0 else f"PLC alarm {alarm}"
        st.values = [
            KeyValue(key="seq", value=str(self.seq)),
            KeyValue(key="ack", value=str(ack)),
            KeyValue(key="state", value=str(state)),
            KeyValue(key="alarm", value=str(alarm)),
            KeyValue(key="track_index", value=str(idx)),
            KeyValue(key="pitch_deg", value=f"{pitch_deg:.3f}"),
            KeyValue(key="yaw_deg", value=f"{yaw_deg:.3f}"),
            KeyValue(key="tool_y_m", value=f"{tool_y_m:.4f}"),
            KeyValue(key="tool_z_m", value=f"{tool_z_m:.4f}"),
            KeyValue(key="target_y_m", value=f"{target_y_m:.4f}"),
            KeyValue(key="target_z_m", value=f"{target_z_m:.4f}"),
            KeyValue(key="err_y_mm", value=f"{err_y_m * 1000.0:.2f}"),
            KeyValue(key="err_z_mm", value=f"{err_z_m * 1000.0:.2f}"),
            KeyValue(key="M100_CmdServoOn", value=self._read_m_safe(M_CMD_SERVO_ON)),
            KeyValue(key="M105_CmdSweepStart", value=self._read_m_safe(M_CMD_SWEEP_START)),
            KeyValue(key="M106_GuideReady", value=self._read_m_safe(M_GUIDE_READY)),
            KeyValue(key="M203_HomeDone", value=self._read_m_safe(M_HOME_DONE)),
            KeyValue(key="M500_PCEnable", value=self._read_m_safe(M_PC_ENABLE)),
            KeyValue(key="M523_Timeout", value=self._read_m_safe(M_PLC_TIMEOUT)),
            KeyValue(key="M524_TrajValid", value=self._read_m_safe(M_PLC_TRAJ_VALID)),
            KeyValue(key="M525_TrackEnable", value=self._read_m_safe(M_TRACK_ENABLE)),
        ]
        diag.status.append(st)
        self.pub_diag.publish(diag)


def main(args=None):
    rclpy.init(args=args)
    node = H5UCspBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

