#!/usr/bin/env python3
"""Minimal pure-Python CMSIS-DAP v1 (HID) SWD driver for STM32F103.

Zero external dependencies (uses Linux /dev/hidraw* directly) so flashing and
real-time non-intrusive SRAM mailbox I/O work identically on the host and
inside Docker devcontainers without requiring OpenOCD or pyocd.
"""

from __future__ import annotations

import glob
import os
import select
import struct
import time
from typing import Optional

# CMSIS-DAP v1 Command IDs
DAP_INFO = 0x00
DAP_CONNECT = 0x02
DAP_DISCONNECT = 0x03
DAP_TRANSFER_CONFIGURE = 0x04
DAP_TRANSFER = 0x05
DAP_SWJ_CLOCK = 0x11
DAP_SWJ_SEQUENCE = 0x12

DAP_PORT_SWD = 0x01
DAP_TRANSFER_OK = 0x01
DAP_TRANSFER_WAIT = 0x02

# SWD DP/AP Transfer Request Bytes (bit0=APnDP, bit1=RnW, bit[3:2]=A[3:2])
DP_R_IDCODE = 0x02
DP_W_ABORT = 0x00
DP_W_CTRL_STAT = 0x04
DP_R_CTRL_STAT = 0x06
DP_W_SELECT = 0x08

AP_W_CSW = 0x01
AP_W_TAR = 0x05
AP_W_DRW = 0x0D
AP_R_DRW = 0x0F

# Cortex-M3 AHB-AP CSW values
CSW_32BIT_NOINC = 0x23000002
CSW_16BIT_NOINC = 0x23000001

# Cortex-M3 Debug & System Control Registers
DCB_DHCSR = 0xE000EDF0
DCB_DEMCR = 0xE000EDFC
SCB_AIRCR = 0xE000ED0C

DHCSR_DBGKEY = 0xA05F0000
DHCSR_C_DEBUGEN = 0x00000001
DHCSR_C_HALT = 0x00000002
DEMCR_VC_CORERESET = 0x00000001
AIRCR_VECTKEY_SYSRESETREQ = 0x05FA0004

# STM32F103 Flash Registers
FLASH_KEYR = 0x40022004
FLASH_SR = 0x4002200C
FLASH_CR = 0x40022010
FLASH_AR = 0x40022014

FLASH_KEY1 = 0x45670123
FLASH_KEY2 = 0xCDEF89AB
FLASH_SR_BSY = 0x00000001
FLASH_SR_PGERR = 0x00000004
FLASH_SR_WRPRTERR = 0x00000010
FLASH_SR_EOP = 0x00000020
FLASH_CR_PG = 0x00000001
FLASH_CR_PER = 0x00000002
FLASH_CR_STRT = 0x00000040
FLASH_CR_LOCK = 0x00000080

STM32F103_PAGE_SIZE = 1024
STM32F103_FLASH_BASE = 0x08000000

KNOWN_DAP_VID_PIDS = {
    (0x0416, 0x5021),  # WildFire FireDAP CMSIS-DAP
    (0x0D28, 0x0204),  # ARM DAPLink CMSIS-DAP
    (0xC251, 0xF001),  # Keil CMSIS-DAP
    (0x1A86, 0x8011),  # WCH-LinkDAP
    (0x2E8A, 0x000C),  # Raspberry Pi Debugprobe
}


class CmsisDapError(RuntimeError):
    """Raised when CMSIS-DAP or SWD communication fails."""


def find_cmsis_dap_hidraw() -> Optional[tuple[str, str]]:
    """Locate a CMSIS-DAP HID device node (/dev/hidrawN) and its display name."""
    candidates: list[tuple[int, str, str]] = []
    for sys_path in sorted(glob.glob("/sys/class/hidraw/hidraw*")):
        dev_name = os.path.basename(sys_path)
        dev_node = f"/dev/{dev_name}"
        if not os.path.exists(dev_node):
            continue
        uevent_path = os.path.join(sys_path, "device", "uevent")
        try:
            with open(uevent_path, "r", encoding="utf-8", errors="replace") as f:
                uevent = f.read()
        except OSError:
            continue

        hid_name = ""
        vid_pid = None
        for line in uevent.splitlines():
            if line.startswith("HID_NAME="):
                hid_name = line.split("=", 1)[1].strip()
            elif line.startswith("HID_ID="):
                parts = line.split("=", 1)[1].strip().split(":")
                if len(parts) == 3:
                    try:
                        vid_pid = (int(parts[1], 16), int(parts[2], 16))
                    except ValueError:
                        pass

        upper_name = hid_name.upper()
        if "CMSIS-DAP" in upper_name or "FIREDAP" in upper_name:
            candidates.append((0, dev_node, hid_name or dev_name))
        elif vid_pid in KNOWN_DAP_VID_PIDS:
            candidates.append((1, dev_node, hid_name or f"{vid_pid[0]:04x}:{vid_pid[1]:04x}"))

    if not candidates:
        return None
    candidates.sort(key=lambda item: item[0])
    _, node, label = candidates[0]
    return node, label


class CmsisDapSwd:
    """Non-intrusive SWD memory & flash accessor over Linux hidraw."""

    def __init__(self, hidraw_path: Optional[str] = None, clock_hz: int = 1_000_000) -> None:
        if hidraw_path is None:
            found = find_cmsis_dap_hidraw()
            if found is None:
                raise CmsisDapError("No CMSIS-DAP HID debugger found under /dev/hidraw*")
            hidraw_path, self.device_name = found
        else:
            self.device_name = hidraw_path

        self.hidraw_path = hidraw_path
        self.clock_hz = clock_hz
        self.packet_size = 64
        self._fd: Optional[int] = None
        self._csw_cache: Optional[int] = None
        self.dp_idcode: int = 0

    def open(self) -> "CmsisDapSwd":
        try:
            self._fd = os.open(self.hidraw_path, os.O_RDWR | os.O_NONBLOCK)
        except OSError as exc:
            raise CmsisDapError(
                f"Cannot open {self.hidraw_path}: {exc}. Check USB connection and udev permissions."
            ) from exc

        # Drain any stale input reports
        while select.select([self._fd], [], [], 0)[0]:
            try:
                os.read(self._fd, 256)
            except BlockingIOError:
                break

        try:
            self._init_swd()
        except Exception:
            self.close()
            raise
        return self

    def close(self) -> None:
        if self._fd is not None:
            try:
                self._cmd(bytes([DAP_DISCONNECT]))
            except Exception:
                pass
            try:
                os.close(self._fd)
            except OSError:
                pass
            self._fd = None
            self._csw_cache = None

    def __enter__(self) -> "CmsisDapSwd":
        return self.open()

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()

    def _cmd(self, payload: bytes, timeout: float = 1.0) -> bytes:
        if self._fd is None:
            raise CmsisDapError("CMSIS-DAP device is not open")
        if len(payload) > self.packet_size:
            raise CmsisDapError(f"Payload too large ({len(payload)} > {self.packet_size})")

        padded = payload + b"\x00" * (self.packet_size - len(payload))
        # Prepend report ID 0x00 as required by Linux hidraw for unnumbered HID reports
        os.write(self._fd, b"\x00" + padded)

        ready, _, _ = select.select([self._fd], [], [], timeout)
        if not ready:
            raise CmsisDapError("Timeout waiting for CMSIS-DAP HID response")
        resp = os.read(self._fd, self.packet_size + 1)
        if not resp:
            raise CmsisDapError("Empty response from CMSIS-DAP device")
        if resp[0] != payload[0] and len(resp) > 1 and resp[1] == payload[0]:
            resp = resp[1:]
        if resp[0] != payload[0]:
            raise CmsisDapError(
                f"Unexpected CMSIS-DAP response ID 0x{resp[0]:02X} for cmd 0x{payload[0]:02X}"
            )
        return resp

    def _swj_reset_to_swd(self) -> None:
        # >50 clk high, 0xE79E JTAG-to-SWD switch, >50 clk high, 8 idle low clocks
        seq = (b"\xFF" * 7) + bytes([0x9E, 0xE7]) + (b"\xFF" * 7) + b"\x00"
        resp = self._cmd(bytes([DAP_SWJ_SEQUENCE, len(seq) * 8]) + seq)
        if resp[1] != 0x00:
            raise CmsisDapError("DAP_SWJ_Sequence failed")

    def _transfer(self, ops: list[tuple[int, Optional[int]]]) -> list[int]:
        for attempt in range(4):
            buf = bytearray([DAP_TRANSFER, 0x00, len(ops)])
            for req, val in ops:
                buf.append(req)
                if (req & 0x02) == 0:  # Write
                    buf.extend(struct.pack("<I", (val or 0) & 0xFFFFFFFF))

            resp = self._cmd(bytes(buf))
            completed = resp[1]
            ack = resp[2]
            if completed == len(ops) and ack == DAP_TRANSFER_OK:
                reads: list[int] = []
                offset = 3
                for req, _ in ops:
                    if (req & 0x02) != 0:
                        reads.append(struct.unpack_from("<I", resp, offset)[0])
                        offset += 4
                return reads

            # Clear sticky error flags on fault/wait and retry briefly
            try:
                self._cmd(bytes([DAP_TRANSFER, 0x00, 0x01, DP_W_ABORT]) + struct.pack("<I", 0x1E))
            except Exception:
                pass
            if ack == DAP_TRANSFER_WAIT and attempt < 3:
                time.sleep(0.002)
                continue
            if attempt == 2:
                self._swj_reset_to_swd()
                try:
                    self._cmd(bytes([DAP_TRANSFER, 0x00, 0x01, DP_R_IDCODE]))
                except Exception:
                    pass
                self._csw_cache = None

        raise CmsisDapError(
            f"SWD transfer failed (completed={completed}/{len(ops)}, ack=0x{ack:02X})"
        )

    def _init_swd(self) -> None:
        resp = self._cmd(bytes([DAP_CONNECT, DAP_PORT_SWD]))
        if resp[1] != DAP_PORT_SWD:
            raise CmsisDapError(f"DAP_Connect(SWD) failed: status=0x{resp[1]:02X}")

        self._cmd(bytes([DAP_SWJ_CLOCK]) + struct.pack("<I", self.clock_hz))
        self._cmd(bytes([DAP_TRANSFER_CONFIGURE, 0x00]) + struct.pack("<HH", 200, 0))
        self._swj_reset_to_swd()

        self.dp_idcode = self._transfer([(DP_R_IDCODE, None)])[0]
        self._transfer([
            (DP_W_ABORT, 0x0000001E),
            (DP_W_SELECT, 0x00000000),
            (DP_W_CTRL_STAT, 0x50000000),  # CDBGPWRUPREQ | CSYSPWRUPREQ
        ])

        deadline = time.monotonic() + 0.5
        while True:
            stat = self._transfer([(DP_R_CTRL_STAT, None)])[0]
            if (stat & 0xF0000000) == 0xF0000000:
                break
            if time.monotonic() > deadline:
                raise CmsisDapError(f"Debug power-up ACK timeout (CTRL/STAT=0x{stat:08X})")
            time.sleep(0.005)

        self._csw_cache = None

    def _ensure_csw(self, csw: int) -> list[tuple[int, Optional[int]]]:
        if self._csw_cache == csw:
            return []
        self._csw_cache = csw
        return [(AP_W_CSW, csw)]

    def read32(self, addr: int) -> int:
        ops = self._ensure_csw(CSW_32BIT_NOINC)
        ops.extend([
            (AP_W_TAR, addr & 0xFFFFFFFF),
            (AP_R_DRW, None),
        ])
        return self._transfer(ops)[0]

    def write32(self, addr: int, value: int) -> None:
        ops = self._ensure_csw(CSW_32BIT_NOINC)
        ops.extend([
            (AP_W_TAR, addr & 0xFFFFFFFF),
            (AP_W_DRW, value & 0xFFFFFFFF),
        ])
        self._transfer(ops)

    def write16(self, addr: int, halfword: int) -> None:
        if addr & 0x01:
            raise ValueError(f"Unaligned 16-bit address 0x{addr:08X}")
        hw = halfword & 0xFFFF
        # Cortex-M3 AHB-AP byte-lane replication for 16-bit writes
        packed = hw | (hw << 16)
        ops = self._ensure_csw(CSW_16BIT_NOINC)
        ops.extend([
            (AP_W_TAR, addr & 0xFFFFFFFF),
            (AP_W_DRW, packed),
        ])
        self._transfer(ops)

    def halt_core(self) -> None:
        self.write32(DCB_DHCSR, DHCSR_DBGKEY | DHCSR_C_DEBUGEN | DHCSR_C_HALT)
        deadline = time.monotonic() + 0.5
        while time.monotonic() < deadline:
            if self.read32(DCB_DHCSR) & 0x00020000:  # S_HALT
                return
            time.sleep(0.005)
        raise CmsisDapError("Timed out halting Cortex-M3 core")

    def reset_and_run(self) -> None:
        self.write32(DCB_DEMCR, 0x00000000)
        try:
            self.write32(SCB_AIRCR, AIRCR_VECTKEY_SYSRESETREQ)
        except CmsisDapError:
            # SYSRESETREQ can briefly drop AHB-AP transaction ACK during reset
            pass
        time.sleep(0.05)
        self._init_swd()
        # Clear C_DEBUGEN / C_HALT so MCU runs standalone
        self.write32(DCB_DHCSR, DHCSR_DBGKEY)

    def _wait_flash_ready(self, timeout: float = 1.0) -> int:
        deadline = time.monotonic() + timeout
        while True:
            sr = self.read32(FLASH_SR)
            if (sr & FLASH_SR_BSY) == 0:
                if sr & (FLASH_SR_PGERR | FLASH_SR_WRPRTERR):
                    raise CmsisDapError(f"STM32 Flash error (FLASH_SR=0x{sr:08X})")
                return sr
            if time.monotonic() > deadline:
                raise CmsisDapError("Timed out waiting for STM32 Flash BSY clear")
            time.sleep(0.001)

    def flash_binary(self, data: bytes, base_addr: int = STM32F103_FLASH_BASE) -> None:
        """Erase, program, verify, and reset-run an STM32F103 binary image over SWD."""
        if len(data) % 2 != 0:
            data = data + b"\xFF"

        # Reset-halt cleanly
        self.write32(DCB_DHCSR, DHCSR_DBGKEY | DHCSR_C_DEBUGEN | DHCSR_C_HALT)
        self.write32(DCB_DEMCR, DEMCR_VC_CORERESET)
        try:
            self.write32(SCB_AIRCR, AIRCR_VECTKEY_SYSRESETREQ)
        except CmsisDapError:
            pass
        time.sleep(0.03)
        self._init_swd()
        self.halt_core()
        self.write32(DCB_DEMCR, 0x00000000)

        # Unlock FPEC
        if self.read32(FLASH_CR) & FLASH_CR_LOCK:
            self.write32(FLASH_KEYR, FLASH_KEY1)
            self.write32(FLASH_KEYR, FLASH_KEY2)
        if self.read32(FLASH_CR) & FLASH_CR_LOCK:
            raise CmsisDapError("Failed to unlock STM32F103 Flash controller")

        # Clear status flags
        self.write32(FLASH_SR, FLASH_SR_EOP | FLASH_SR_PGERR | FLASH_SR_WRPRTERR)

        # Erase required 1KB pages
        num_pages = (len(data) + STM32F103_PAGE_SIZE - 1) // STM32F103_PAGE_SIZE
        for page_idx in range(num_pages):
            page_addr = base_addr + page_idx * STM32F103_PAGE_SIZE
            self.write32(FLASH_CR, FLASH_CR_PER)
            self.write32(FLASH_AR, page_addr)
            self.write32(FLASH_CR, FLASH_CR_PER | FLASH_CR_STRT)
            self._wait_flash_ready()
            self.write32(FLASH_SR, FLASH_SR_EOP)
        self.write32(FLASH_CR, 0)

        # Program 16-bit halfwords
        self.write32(FLASH_CR, FLASH_CR_PG)
        for offset in range(0, len(data), 2):
            hw = struct.unpack_from("<H", data, offset)[0]
            if hw == 0xFFFF:
                continue
            self.write16(base_addr + offset, hw)
            while self.read32(FLASH_SR) & FLASH_SR_BSY:
                pass
        self._wait_flash_ready()
        self.write32(FLASH_CR, FLASH_CR_LOCK)

        # Verify 32-bit words
        padded = data + (b"\xFF" * ((4 - (len(data) % 4)) % 4))
        for offset in range(0, len(padded), 4):
            expected = struct.unpack_from("<I", padded, offset)[0]
            actual = self.read32(base_addr + offset)
            if actual != expected:
                raise CmsisDapError(
                    f"Flash verify mismatch at 0x{base_addr + offset:08X}: "
                    f"expected 0x{expected:08X}, got 0x{actual:08X}"
                )

        self.reset_and_run()
