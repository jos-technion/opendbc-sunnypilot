"""
Regression tests pinning the Ocean plain 8-bit CheckSum algorithm.
"""

import pytest

from opendbc.car.fisker.fiskercan import fisker_plain_checksum


# Full-frame samples. First byte is the expected checksum.
NON_SECOC_FRAMES = [
  # 0x115 ESP wheel speeds F, DataID=138, DataLength=64
  (0x115, bytes.fromhex("DFF8435B27435BFF")),
  # 0x116 ESP wheel speeds R, DataID=159, DataLength=64
  (0x116, bytes.fromhex("65F84358274354FF")),
  # 0x117 ADAS long-control status, DataID=72, DataLength=64
  (0x117, bytes.fromhex("1701484050000038")),
  (0x117, bytes.fromhex("F002484050000038")),
  (0x117, bytes.fromhex("4A00484050000038")),
  # 0x118 ADAS ESP handshake, DataID=81, DataLength=64
  (0x118, bytes.fromhex("32014A48441080D2")),
]

# EPS_0x1C2 frames captured on-vehicle 2026-10-10. The algorithm (CRC-8 J1850,
# data_id=0x90, length 64) was reverse-engineered by brute-forcing all polynomial +
# init/xorout + data_id combinations against the full 200-frame capture; only the
# shared Fisker-family parameterisation matches on every frame. Pin a representative
# subset so a future accidental change to E2E_PARAMS[0x1C2] or fisker_plain_checksum
# would break the suite loudly.
EPS_0x1C2_FRAMES = [
  (0x1C2, bytes.fromhex("980030bcb37f0500")),  # idle, straight
  (0x1C2, bytes.fromhex("c50130bcb37f0500")),  # alive +1
  (0x1C2, bytes.fromhex("3c0b30bca37f1500")),  # torque sign bit varied
  (0x1C2, bytes.fromhex("f80e30bab37f0500")),  # alive wraparound (14)
  (0x1C2, bytes.fromhex("d10c2a3dab71e50b")),  # steering wheel turned ~43°
  (0x1C2, bytes.fromhex("070e2b2bab6ff50b")),  # non-zero AdasLatCtrlSts byte
  (0x1C2, bytes.fromhex("12082da5ab74050b")),  # mid-sweep
]

NON_SECOC_FRAMES += EPS_0x1C2_FRAMES

# SecOC-protected frames — CRC covers only bytes 1..3 (DataLength=32)
SECOC_FRAMES = [
  # 0x121 ADAS accel command, DataID=23, DataLength=32
  (0x121, bytes.fromhex("E611800540000000")),   # tail zeroed; real cksum stays valid
  (0x121, bytes.fromhex("E611800518C1854D")),   # tail real (cksum ignores it)
  # 0x1D0 ADAS steering command, DataID=16, DataLength=32
  (0x1D0, bytes.fromhex("D20230C04DAEBF77")),
  (0x1D0, bytes.fromhex("5D0330C0517B789F")),
  (0x1D0, bytes.fromhex("D40630C05D3D84B1")),
]


@pytest.mark.parametrize("addr,frame", NON_SECOC_FRAMES + SECOC_FRAMES)
def test_checksum_matches_captured_frame(addr, frame):
  # Zero the checksum byte to simulate mid-pack state
  work = bytes([0]) + frame[1:]
  assert fisker_plain_checksum(addr, work) == frame[0]


def test_secoc_tail_doesnt_affect_checksum():
  # For 0x1D0 (DataLength=32), bytes 4..7 (SecOC tail) must not enter the CRC
  data = bytes.fromhex("D20230C0FFFFFFFF")   # tail = all 0xFF
  assert fisker_plain_checksum(0x1D0, data) == 0xD2

  data = bytes.fromhex("D20230C000000000")   # tail = all 0x00
  assert fisker_plain_checksum(0x1D0, data) == 0xD2
