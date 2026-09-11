"""
Fisker Ocean SecOC — AUTOSAR CP "Short SecOC" style implementation.

MAC computation (data messages, 8-byte classic-CAN "Short SecOC on byte4~7", DLC=8):

    MAC = AES-128-CMAC(key, DataId(2 BE) || PDU(4) || FullFreshness(8 BE))[:3]

  where:
    DataId = CAN identifier as 2 bytes big-endian (e.g. 0x01D0)
    PDU    = frame bytes 0..3, INCLUDING the plain-CRC checksum in byte 0
    FullFreshness (64 bits, big-endian) layout:
        Trip Counter    16 bits   (from GW sync 0x20; constant across an ignition cycle)
        Reset Counter   24 bits   (from GW sync 0x20; increments ~1/second)
        Message Counter 16 bits   (per-PDU; +1 each transmission; low 6 bits shown on wire)
        Reset Flag       2 bits   (= Reset Counter & 0x3), placed in the high bits of byte 7
        Padding          6 bits   (0)
    → byte 7 = (ResetFlag << 6)

On-wire "Truncated Freshness" byte (SSecOC_Fresh_Byte0) = (MessageCounter[5:0] << 2) | ResetFlag.

The GW freshness-sync message (0x20) uses a different, simpler MAC:
    MAC = AES-128-CMAC(key, 0x0020 || Trip(2) || Reset(3))[:3]   (no message counter)
"""

import struct
from Crypto.Cipher import AES
from Crypto.Hash import CMAC


def _cmac24(key: bytes, data: bytes) -> bytes:
  c = CMAC.new(key, ciphermod=AES)
  c.update(data)
  return c.digest()[:3]   # 24 most-significant bits


def build_freshness(trip: int, reset: int, msg_counter: int) -> bytes:
  """Assemble the 8-byte full freshness value used as MAC input."""
  reset_flag = reset & 0x3
  return (struct.pack(">H", trip & 0xFFFF)
          + (reset & 0xFFFFFF).to_bytes(3, "big")
          + struct.pack(">H", msg_counter & 0xFFFF)
          + bytes([(reset_flag << 6) & 0xFF]))


def secoc_mac(key: bytes, can_id: int, pdu: bytes, trip: int, reset: int, msg_counter: int) -> bytes:
  """
  Compute the 3-byte Short-SecOC MAC for a data message.
  `pdu` is the first 4 bytes of the frame (byte 0 = plain-CRC checksum already filled).
  """
  fresh = build_freshness(trip, reset, msg_counter)
  to_auth = struct.pack(">H", can_id & 0xFFFF) + pdu[:4] + fresh
  return _cmac24(key, to_auth)


def wire_freshness_byte(msg_counter: int, reset: int) -> int:
  """The SSecOC_Fresh_Byte0 value transmitted on the wire."""
  return (((msg_counter & 0x3F) << 2) | (reset & 0x3)) & 0xFF


def stamp_secoc(key: bytes, can_id: int, frame8: bytes, trip: int, reset: int, msg_counter: int) -> bytes:
  """
  Given an 8-byte frame with bytes 0..3 populated (incl. plain-CRC checksum), write the
  SecOC tail (byte4 = wire freshness, bytes5..7 = MAC) and return the full frame.
  """
  buf = bytearray(frame8[:8].ljust(8, b"\x00"))
  buf[4] = wire_freshness_byte(msg_counter, reset)
  buf[5:8] = secoc_mac(key, can_id, bytes(buf[0:4]), trip, reset, msg_counter)
  return bytes(buf)


# ---- GW freshness-sync message (0x20) -------------------------------------

SYNC_CAN_ID = 0x20


def sync_mac(key: bytes, trip: int, reset: int) -> bytes:
  """MAC carried by GW_Syn_All (0x20): CMAC(key, 0x0020 || Trip(2) || Reset(3))[:3]."""
  to_auth = struct.pack(">H", SYNC_CAN_ID) + struct.pack(">H", trip & 0xFFFF) + (reset & 0xFFFFFF).to_bytes(3, "big")
  return _cmac24(key, to_auth)


def verify_sync(key: bytes, sync_payload: bytes) -> bool:
  """Validate a received GW_Syn_All payload (8 bytes: Trip2 Reset3 MAC3)."""
  trip = struct.unpack(">H", sync_payload[0:2])[0]
  reset = int.from_bytes(sync_payload[2:5], "big")
  return sync_mac(key, trip, reset) == sync_payload[5:8]


def parse_sync(sync_payload: bytes) -> tuple[int, int]:
  """Return (trip, reset) from a GW_Syn_All payload."""
  trip = struct.unpack(">H", sync_payload[0:2])[0]
  reset = int.from_bytes(sync_payload[2:5], "big")
  return trip, reset
