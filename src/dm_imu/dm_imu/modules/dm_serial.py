# models/dm_serial.py
# -*- coding: utf-8 -*-
"""
DM_Serial: 达妙 IMU 串口读取类（支持“后台读线程” + 主线程按需取最新）
- 修改版：支持同时缓存 Acc(0x01)、Gyro(0x02) 和 RPY(0x03)
"""
from __future__ import annotations

import struct
import threading
import time
from typing import Optional, Tuple, List, Dict

import serial  # pip install pyserial

from .dm_crc import dm_crc16

HDR = b'\x55\xAA'
TAIL = 0x0A
FRAME_LEN = 19
VALID_RIDS = {0x01, 0x02, 0x03}

SKIP_HDR_IN_CRC = False

class DM_Serial:
    def __init__(self, port: str, baudrate: int):
        self.port = port
        self.baudrate = int(baudrate)
        self.timeout = 0.0  
        self.ser: Optional[serial.Serial] = None
        self._buf = bytearray()

        # 统计
        self.cnt_ok = 0
        self.cnt_crc = 0
        self.cnt_short = 0
        self.cnt_nohdr = 0

        self._th: Optional[threading.Thread] = None
        self._stop_evt: Optional[threading.Event] = None
        self._read_sleep = 0.001

        # =================【修改点 1：把单变量改成三个抽屉的字典】=================
        self._latest_lock = threading.Lock()
        self._latest_state: Dict[int, Tuple[float, float, float]] = {
            0x01: (0.0, 0.0, 0.0),
            0x02: (0.0, 0.0, 0.0),
            0x03: (0.0, 0.0, 0.0)
        }
        self._latest_ts: float = 0.0
        self._latest_count: int = 0
        self._last_error: Optional[str] = None

        self._open()

    # =================【修改点 2：read 返回所有帧的列表，不再只返回最后一个】=================
    def read(self, max_bytes: int | None = None) -> List[Tuple[int, Tuple[float, float, float]]]:
        if not self.ser or not self.ser.is_open:
            return []
        self._read_into_buf(max_bytes)
        return self._parse_all()

    def start_reader(self, read_sleep: float = 0.001) -> bool:
        if self._th and self._th.is_alive():
            self._read_sleep = read_sleep
            return True
        if not self.is_open:
            if not self._open():
                return False
        self._stop_evt = threading.Event()
        self._read_sleep = read_sleep
        self._th = threading.Thread(target=self._reader_loop, daemon=True)
        self._th.start()
        return True

    def stop_reader(self) -> None:
        if self._stop_evt:
            self._stop_evt.set()
        if self._th:
            self._th.join(timeout=1.0)
        self._th = None
        self._stop_evt = None

    # =================【修改点 3：返回完整的状态字典】=================
    def get_latest(self) -> Tuple[Dict[int, Tuple[float, float, float]], float, int]:
        with self._latest_lock:
            # 使用 dict() 返回一个浅拷贝，防止主线程读取时数据被修改
            return dict(self._latest_state), self._latest_ts, self._latest_count

    def last_error(self) -> Optional[str]:
        return self._last_error

    def destory(self) -> None:
        self.stop_reader()
        if self.ser:
            try:
                self.ser.close()
            finally:
                self.ser = None

    def destroy(self) -> None:
        self.destory()

    def reopen(self) -> bool:
        self.destory()
        return self._open()

    @property
    def is_open(self) -> bool:
        return bool(self.ser and self.ser.is_open)

    def _open(self) -> bool:
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=self.timeout, write_timeout=0)
            try:
                self.ser.reset_input_buffer()
            except Exception:
                pass
            return True
        except Exception as e:
            self._last_error = str(e)
            self.ser = None
            return False

    # =================【修改点 4：循环解析所有帧，分别放进对应的 RID 抽屉】=================
    def _reader_loop(self):
        evt = self._stop_evt
        try:
            while evt and not evt.is_set():
                frames = self.read(None)
                if frames:
                    with self._latest_lock:
                        for rid, data in frames:
                            self._latest_state[rid] = data
                        self._latest_ts = time.time()
                        self._latest_count += 1
                if self._read_sleep > 0.0:
                    time.sleep(self._read_sleep)
        except Exception as e:
            self._last_error = f"reader_loop: {e!r}"

    def _read_into_buf(self, max_bytes: Optional[int]) -> int:
        n = getattr(self.ser, "in_waiting", 0) if self.ser else 0
        if max_bytes is not None and n > max_bytes:
            n = max_bytes
        if n <= 0:
            return 0
        self._buf.extend(self.ser.read(n))
        return n

    def _parse_all(self) -> List[Tuple[int, Tuple[float, float, float]]]:
        results: List[Tuple[int, Tuple[float, float, float]]] = []
        buf = self._buf
        start = 0

        while True:
            j = buf.find(HDR, start)
            if j < 0:
                keep = buf[-1:] if buf else b''
                self._buf = bytearray(keep)
                if buf:
                    self.cnt_nohdr += 1
                break

            if len(buf) - j < FRAME_LEN:
                self._buf = bytearray(buf[j:])
                self.cnt_short += 1
                break

            frame = bytes(buf[j:j + FRAME_LEN])
            start = j + 1

            if frame[-1] != TAIL:
                continue

            rid = frame[3]
            if rid not in VALID_RIDS:
                continue

            if SKIP_HDR_IN_CRC:
                crc_calc = dm_crc16(frame[2:16])
            else:
                crc_calc = dm_crc16(frame[0:16])
            crc_wire = frame[16] | (frame[17] << 8)
            if crc_calc != crc_wire:
                alt = dm_crc16(frame[2:16]) if not SKIP_HDR_IN_CRC else dm_crc16(frame[0:16])
                if alt != crc_wire:
                    self.cnt_crc += 1
                    continue

            f1 = struct.unpack('<f', frame[4:8])[0]
            f2 = struct.unpack('<f', frame[8:12])[0]
            f3 = struct.unpack('<f', frame[12:16])[0]
            results.append((rid, (f1, f2, f3)))

            buf = buf[j + FRAME_LEN:]
            start = 0

        if isinstance(buf, (bytes, bytearray)) and buf is not self._buf:
            self._buf = bytearray(buf)

        self.cnt_ok += len(results)
        return results