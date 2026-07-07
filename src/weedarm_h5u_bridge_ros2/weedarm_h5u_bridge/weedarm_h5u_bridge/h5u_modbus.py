# -*- coding: utf-8 -*-
"""
H5U Modbus TCP client based on pymodbus.

Supported:
- Holding registers D: INT / DINT / trajectory block
- Coils M: read_coil / write_coil

Compatible with pymodbus 2.x/3.x argument names:
slave / unit / device_id
"""

from typing import List, Tuple

from pymodbus.client import ModbusTcpClient


class H5UModbus:
    def __init__(self, ip: str, port: int = 502, unit: int = 1,
                 addr_offset: int = 0, word_order: str = "lo_hi",
                 coil_addr_offset: int = 0):
        self.ip = ip
        self.port = int(port)
        self.unit = int(unit)
        self.addr_offset = int(addr_offset)
        self.coil_addr_offset = int(coil_addr_offset)
        self.word_order = str(word_order)
        self.client = ModbusTcpClient(ip, port=port)

    def addr(self, d_addr: int) -> int:
        return int(d_addr) + int(self.addr_offset)

    def coil_addr(self, m_addr: int) -> int:
        return int(m_addr) + int(self.coil_addr_offset)

    def connect(self) -> bool:
        return bool(self.client.connect())

    def close(self):
        self.client.close()

    def _call_modbus(self, func, **kwargs):
        """
        pymodbus 版本差异：
        - 有些版本用 slave
        - 有些版本用 unit
        - 有些版本用 device_id
        """
        last_exc = None
        for unit_key in ("slave", "unit", "device_id"):
            try:
                return func(**kwargs, **{unit_key: self.unit})
            except TypeError as exc:
                last_exc = exc
                continue
        raise last_exc

    # --------------------
    # M coils
    # --------------------
    def read_coils(self, m_addr: int, count: int) -> List[bool]:
        rr = self._call_modbus(
            self.client.read_coils,
            address=self.coil_addr(m_addr),
            count=int(count),
        )
        if rr.isError():
            raise RuntimeError(f"读取 M{m_addr} 失败：{rr}")
        return [bool(v) for v in rr.bits[:count]]

    def read_coil(self, m_addr: int) -> bool:
        return self.read_coils(m_addr, 1)[0]

    def write_coil(self, m_addr: int, value: bool):
        rr = self._call_modbus(
            self.client.write_coil,
            address=self.coil_addr(m_addr),
            value=bool(value),
        )
        if rr.isError():
            raise RuntimeError(f"写入 M{m_addr} 失败：{rr}")

    def write_coils(self, m_addr: int, values: List[bool]):
        rr = self._call_modbus(
            self.client.write_coils,
            address=self.coil_addr(m_addr),
            values=[bool(v) for v in values],
        )
        if rr.isError():
            raise RuntimeError(f"写入 M{m_addr} 起多个线圈失败：{rr}")

    # --------------------
    # D holding registers
    # --------------------
    def read_regs(self, d_addr: int, count: int) -> List[int]:
        rr = self._call_modbus(
            self.client.read_holding_registers,
            address=self.addr(d_addr),
            count=int(count),
        )
        if rr.isError():
            raise RuntimeError(f"读取 D{d_addr} 失败：{rr}")
        return rr.registers

    def write_regs(self, d_addr: int, regs: List[int]):
        rr = self._call_modbus(
            self.client.write_registers,
            address=self.addr(d_addr),
            values=[int(v) & 0xFFFF for v in regs],
        )
        if rr.isError():
            raise RuntimeError(f"写入 D{d_addr} 失败：{rr}")

    def read_int(self, d_addr: int) -> int:
        value = self.read_regs(d_addr, 1)[0]
        return value - 0x10000 if value >= 0x8000 else value

    def write_int(self, d_addr: int, value: int):
        self.write_regs(d_addr, [int(value) & 0xFFFF])

    @staticmethod
    def _to_signed32(value: int) -> int:
        value &= 0xFFFFFFFF
        return value - 0x100000000 if value >= 0x80000000 else value

    def read_dint(self, d_addr: int) -> int:
        r0, r1 = self.read_regs(d_addr, 2)
        raw = ((r0 << 16) | r1) if self.word_order == "hi_lo" else ((r1 << 16) | r0)
        return self._to_signed32(raw)

    def write_dint(self, d_addr: int, value: int):
        value = int(value) & 0xFFFFFFFF
        hi = (value >> 16) & 0xFFFF
        lo = value & 0xFFFF
        regs = [hi, lo] if self.word_order == "hi_lo" else [lo, hi]
        self.write_regs(d_addr, regs)

    def write_traj_block(self, base: int, points_mm: List[Tuple[float, float]]):
        regs: List[int] = []
        for y_mm, z_mm in points_mm:
            y_i = int(round(float(y_mm) * 1000.0)) & 0xFFFFFFFF
            z_i = int(round(float(z_mm) * 1000.0)) & 0xFFFFFFFF

            y_hi, y_lo = (y_i >> 16) & 0xFFFF, y_i & 0xFFFF
            z_hi, z_lo = (z_i >> 16) & 0xFFFF, z_i & 0xFFFF

            if self.word_order == "hi_lo":
                regs += [y_hi, y_lo, z_hi, z_lo]
            else:
                regs += [y_lo, y_hi, z_lo, z_hi]

        # 分块写，避免一次写太多寄存器失败
        chunk = 64
        for offset in range(0, len(regs), chunk):
            self.write_regs(base + offset, regs[offset:offset + chunk])

