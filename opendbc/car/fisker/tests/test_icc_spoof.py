"""
Tests for the ICC 0x52A spoof. We don't know ICC 0x52A's E2E DataID, but the
CheckSum is CRC-8 J1850 with init=0/xorout=0 over ([DataID] || payload[1:8]),
which is LINEAR over GF(2). fisker_icc_checksum_delta exploits that linearity
to compute the new checksum from just the byte-diff, without ever needing the
DataID. These tests pin that mathematical property, then check that
create_icc_spoof_0x52a end-to-end preserves every other byte and flips
ICCACCFuncTyp to 2.
"""

import pytest

from opendbc.can import CANPacker
from opendbc.car.fisker.fiskercan import (
  FiskerCAN,
  _crc8_j1850,
  fisker_icc_checksum_delta,
)
from opendbc.car.fisker.values import CANBUS, CAR, DBC
from opendbc.car import Bus


# Exhaustively sanity-check the XOR-linearity: for a range of hypothetical
# DataIDs and payloads, verify that (orig_crc XOR delta) equals the CRC
# recomputed from scratch after the same mutation.
@pytest.mark.parametrize("data_id", [0, 1, 16, 42, 128, 195, 255])
@pytest.mark.parametrize("orig_payload", [
  bytes.fromhex("00000000000000"),
  bytes.fromhex("FFFFFFFFFFFFFF"),
  bytes.fromhex("123456789ABCDE"),
  bytes.fromhex("DEADBEEFCAFE01"),
])
@pytest.mark.parametrize("new_b4_top3", [0, 1, 2, 3, 5, 7])
def test_icc_checksum_delta_matches_from_scratch(data_id, orig_payload, new_b4_top3):
  # `orig_payload` is 7 bytes: what would sit at frame bytes [1..8).
  # frame byte 4 == orig_payload[3].
  b4_orig = orig_payload[3]
  b4_new = (b4_orig & 0x1F) | ((new_b4_top3 & 0x7) << 5)

  orig_crc = _crc8_j1850(bytes([data_id]) + orig_payload)

  new_payload = bytearray(orig_payload)
  new_payload[3] = b4_new
  scratch_crc = _crc8_j1850(bytes([data_id]) + bytes(new_payload))

  diff = bytearray(8)
  diff[4] = b4_orig ^ b4_new
  delta = fisker_icc_checksum_delta(bytes(diff))
  delta_crc = orig_crc ^ delta

  assert delta_crc == scratch_crc, (
    f"linearity broke: data_id={data_id:02X} b4 {b4_orig:02X}->{b4_new:02X} "
    f"scratch={scratch_crc:02X} via_delta={delta_crc:02X}"
  )


def _icc_signal_defaults():
  """Return a dict with every ICC_0x52A signal set to 0 (matches
  fisker.carstate.ICC_0x52A_SIGNALS)."""
  return {
    "ICC_0x52ACheckSum": 0,
    "ICC_0x52AAliveCounter": 0,
    "ICC_FACMDynmcSenstvty": 0,
    "ICCUsrProfTiGapSet": 0,
    "ICC_LKASetting": 0,
    "ICC_FACMSetting": 0,
    "ICC_AEBSensitivity": 0,
    "ICC_BACMSetting": 0,
    "ICC_BACMSensitivity": 0,
    "ICC_AEBJerkSetReq": 0,
    "ICCActvStyGlblSetting": 0,
    "ICC_TSRSetting": 0,
    "ICC_ESASetting": 0,
    "ICCELKASteeringInterventionSet": 0,
    "ICCLaneTrajectorySetting": 0,
    "ICCACCSwt": 0,
    "ICCACCAutoSpdSts": 0,
    "ICCACCSpdStepSize": 0,
    "ICCACCFuncTyp": 0,
    "ICCACCSpdLimOffs": 0,
    "ICCACCSpdLimOffsTyp": 0,
    "ICCACCTerrainSetting": 0,
    "ICC_0x52A_Rsv49": 0,
    "ICCACCTiGapCfm": 0,
    "ICCISASetting": 0,
    "ICC_0x52A_Rsv55": 0,
    "ICC_FCTASensitivity": 0,
    "ICCISAWarnStopReq": 0,
    "ICC_TLRSetting": 0,
    "ICC_FCTA_Setting": 0,
  }


def _make_fcan():
  packer = CANPacker(DBC[CAR.FISKER_OCEAN][Bus.pt])
  return FiskerCAN(CP=None, packer=packer), packer


def test_spoof_byte4_is_fully_overridden_and_routes_to_cam_bus():
  """Byte 4 overrides ALL five ACC-related fields, not just ICCACCFuncTyp:
       ICCLaneTrajectorySetting(1) | ICCACCSwt(1) | ICCACCAutoSpdSts(1)
       | ICCACCSpdStepSize(1) | ICCACCFuncTyp(2)
     Nothing from OEM's byte 4 is preserved — the goal is to flip ACC on at the
     ADAS module, which requires ICCACCSwt=1 (not just the type field)."""
  fcan, _ = _make_fcan()
  addr, data, bus = fcan.create_icc_spoof_0x52a(_icc_signal_defaults())
  assert addr == 0x52A
  assert bus == CANBUS.cam
  # Expected packed byte 4 with ICCACCAutoSpdSts=1 (default):
  #   bits 39..37 ICCACCFuncTyp           = 2 << 5 = 0x40
  #   bit 36      ICCACCSpdStepSize       = 1 << 4 = 0x10
  #   bit 35      ICCACCAutoSpdSts        = 1 << 3 = 0x08
  #   bits 34..33 ICCACCSwt               = 1 << 1 = 0x02
  #   bit 32      ICCLaneTrajectorySetting = 1    = 0x01
  assert data[4] == 0x5B


def test_spoof_byte4_with_acc_auto_speed_off():
  """Toggling off the sunnypilot FiskerACCAutoSpeed param drops bit 35
  (ICCACCAutoSpdSts) only. The remaining ACC-enable bits stay 1 so ACC itself
  still engages."""
  fcan, _ = _make_fcan()
  _, data, _ = fcan.create_icc_spoof_0x52a(_icc_signal_defaults(), acc_auto_speed=False)
  # 0x5B minus bit 35 (0x08) = 0x53
  assert data[4] == 0x53
  # ICCACCSwt (bits 34..33) still 1 — ACC master switch untouched by the toggle.
  assert (data[4] >> 1) & 0x3 == 1
  # ICCACCFuncTyp (bits 39..37) still 2.
  assert (data[4] >> 5) & 0x7 == 2


def test_spoof_byte6_terrain_setting_off_forces_bit_clear():
  """terrain_setting=False forces byte-6 bit 0 to 0 (ICCACCTerrainSetting Off)
  regardless of what OEM sent. The toggle has enable/disable semantics, not
  passthrough — the point is to let the user override OEM's choice."""
  fcan, _ = _make_fcan()
  for oem_terrain in (0, 1):
    vals = _icc_signal_defaults()
    vals["ICCACCTerrainSetting"] = oem_terrain
    _, data, _ = fcan.create_icc_spoof_0x52a(vals, terrain_setting=False)
    assert data[6] & 0x01 == 0, f"with OEM terrain={oem_terrain}, got byte6=0x{data[6]:02X}"


def test_spoof_byte6_terrain_setting_on_forces_bit():
  """terrain_setting=True forces byte-6 bit 0 to 1 regardless of what OEM sent."""
  fcan, _ = _make_fcan()
  for oem_terrain in (0, 1):
    vals = _icc_signal_defaults()
    vals["ICCACCTerrainSetting"] = oem_terrain
    _, data, _ = fcan.create_icc_spoof_0x52a(vals, terrain_setting=True)
    assert data[6] & 0x01 == 1, f"with OEM terrain={oem_terrain}, got byte6=0x{data[6]:02X}"


def test_spoof_byte6_preserves_other_fields_regardless_of_terrain():
  """Flipping the terrain bit must NOT disturb TiGapCfm/ISASetting/reserved."""
  fcan, _ = _make_fcan()
  vals = _icc_signal_defaults()
  vals.update({
    "ICCACCTerrainSetting": 0,       # OEM says Off; we'll force On
    "ICCACCTiGapCfm": 2,             # bits 51..50
    "ICCISASetting": 5,              # bits 54..52
    "ICC_0x52A_Rsv49": 1,
    "ICC_0x52A_Rsv55": 1,
  })
  _, data, _ = fcan.create_icc_spoof_0x52a(vals, terrain_setting=True)
  # byte 6 top 7 bits should match what the OEM pack would have produced.
  _, oem, _ = _make_fcan()[1].make_can_msg("ICC_0x52A", CANBUS.pt, vals)
  assert data[6] & 0xFE == oem[6] & 0xFE
  assert data[6] & 0x01 == 1


def test_eps_spoof_byte6_forces_available_and_routes_to_cam_bus():
  """EPS_0x1C2 spoof forces AdasLatCtrlSts to 1 (Available) in byte 6 bits 1..0,
  preserves every other bit of byte 6 (StsVld + high bits), and sends on bus 2."""
  fcan, _ = _make_fcan()
  from opendbc.car.fisker.carstate import EPS_0x1C2_SIGNALS
  vals = {k: 0 for k in EPS_0x1C2_SIGNALS}
  vals["EPS_AdasLatCtrlSts"] = 2       # Active (what EPS reports under our control)
  vals["EPS_AdasLatCtrlStsVld"] = 1    # Valid (OEM baseline)
  addr, data, bus = fcan.create_eps_spoof_0x1c2(vals)
  assert addr == 0x1C2
  assert bus == CANBUS.cam
  # Low 2 bits of byte 6 now say Available (01); bits 3..2 (StsVld) stayed at 01.
  assert (data[6] & 0x03) == 0x01
  assert ((data[6] >> 2) & 0x03) == 0x01


def test_eps_spoof_checksum_valid_under_verified_algorithm():
  """With the EPS_0x1C2 algorithm now pinned (CRC-8 J1850, data_id=0x90), spoofing
  byte 6 and patching byte 0 via the XOR-delta trick must produce a frame that
  passes fisker_plain_checksum. Verifies both the delta logic AND that our
  override of byte 6 doesn't accidentally disturb the checksum invariant.

  The XOR delta trick is `new_chk = old_chk XOR crc(delta_payload)`, so the input
  frame MUST have a valid old_chk for the output to also be valid — in the live
  path that's always true because the input is a real EPS frame off the bus. We
  mimic that here by first computing the valid checksum for the input values and
  stuffing it into EPS_1C2_CheckSum before calling the spoof."""
  from opendbc.car.fisker.carstate import EPS_0x1C2_SIGNALS
  from opendbc.car.fisker.fiskercan import fisker_plain_checksum
  fcan, packer = _make_fcan()
  vals = {k: 0 for k in EPS_0x1C2_SIGNALS}
  vals["EPS_SteerWhlAgSig"] = 500.0      # non-zero angle
  vals["EPS_1C2_AliveCounter"] = 7
  vals["EPS_AdasLatCtrlSts"] = 2
  vals["EPS_AdasLatCtrlStsVld"] = 1
  vals["EPS_AsscMotCrtTq"] = 3.5
  # Seed a VALID checksum for the input so the spoof's XOR delta has something
  # correct to transform.
  vals["EPS_1C2_CheckSum"] = 0
  _, zeroed, _ = packer.make_can_msg("EPS_0x1C2", CANBUS.pt, vals)
  vals["EPS_1C2_CheckSum"] = fisker_plain_checksum(0x1C2, zeroed)
  # Now spoof and verify the output is still checksum-valid.
  _, spoofed, _ = fcan.create_eps_spoof_0x1c2(vals)
  expected = fisker_plain_checksum(0x1C2, bytes([0]) + spoofed[1:])
  assert spoofed[0] == expected, f"spoofed byte0=0x{spoofed[0]:02X} expected=0x{expected:02X}"


def test_spoof_two_byte_overrides_fix_checksum_once():
  """When both byte 4 and byte 6 differ, the XOR delta must account for both so
  the resulting checksum is still valid under any hypothetical DataID."""
  fcan, packer = _make_fcan()
  vals = _icc_signal_defaults()
  vals.update({
    "ICCACCFuncTyp": 7,            # byte 4 differs from 0x5B / 0x53
    "ICCACCTerrainSetting": 0,     # byte 6 bit 0 differs if we flip it on
    "ICCISASetting": 4,            # extra byte-6 content to prove it's preserved
  })
  for hypothetical_data_id in (0x00, 0x7F, 0xF5, 0xFF):
    vals["ICC_0x52ACheckSum"] = 0
    _, zeroed, _ = packer.make_can_msg("ICC_0x52A", CANBUS.pt, vals)
    valid_chk = _crc8_j1850(bytes([hypothetical_data_id]) + zeroed[1:])
    vals["ICC_0x52ACheckSum"] = valid_chk

    _, spoofed, _ = fcan.create_icc_spoof_0x52a(vals, acc_auto_speed=False, terrain_setting=True)
    scratch = _crc8_j1850(bytes([hypothetical_data_id]) + spoofed[1:])
    assert spoofed[0] == scratch, (
      f"two-byte checksum invalid under data_id={hypothetical_data_id:02X}: "
      f"got {spoofed[0]:02X}, expected {scratch:02X}"
    )


def test_spoof_preserves_non_byte4_signals():
  fcan, packer = _make_fcan()
  # Choose distinct non-zero values across many signals so we can see them survive.
  vals = _icc_signal_defaults()
  vals.update({
    "ICC_0x52AAliveCounter": 11,
    "ICCUsrProfTiGapSet": 5,
    "ICC_LKASetting": 2,
    "ICC_FACMSetting": 3,
    "ICC_AEBSensitivity": 1,
    "ICC_BACMSetting": 2,
    "ICCACCSpdLimOffs": 12,
    "ICCACCSpdLimOffsTyp": 2,
    "ICCACCTerrainSetting": 1,
    "ICCACCTiGapCfm": 2,
    "ICCISASetting": 5,
    "ICC_FCTASensitivity": 3,
    "ICC_TLRSetting": 4,
    "ICC_FCTA_Setting": 2,
    # OEM byte-4 fields — all overridden regardless of the values here.
    "ICCACCFuncTyp": 7,
    "ICCLaneTrajectorySetting": 0,
    "ICCACCSwt": 3,
    "ICCACCAutoSpdSts": 0,
    "ICCACCSpdStepSize": 0,
  })

  # What the OEM would pack (no spoof) — for byte-by-byte comparison.
  _, oem_frame, _ = packer.make_can_msg("ICC_0x52A", CANBUS.pt, vals)

  # Spoof call matches OEM's byte-6 bit 0 (both terrain-on) so passthrough is clean;
  # the test's job is only to prove byte 4 overrides don't disturb anything else.
  _, spoofed, spoofed_bus = fcan.create_icc_spoof_0x52a(vals, terrain_setting=True)
  assert spoofed_bus == CANBUS.cam

  # Only byte 0 (checksum patch) and byte 4 (full ACC overrides) should differ.
  differing = [i for i in range(8) if spoofed[i] != oem_frame[i]]
  assert differing == [0, 4], f"unexpected byte changes: {differing}"
  # Byte 4 is a constant 0x5B regardless of OEM — all five bit-fields overridden.
  assert spoofed[4] == 0x5B


def test_spoof_checksum_stays_valid_for_arbitrary_hypothetical_data_id():
  """Even without knowing 0x52A's real DataID, the XOR-linearity guarantees
  that if the incoming frame's checksum was valid under some data_id, our
  spoofed frame's checksum will also be valid under that same data_id."""
  fcan, packer = _make_fcan()

  vals = _icc_signal_defaults()
  vals.update({
    "ICC_0x52AAliveCounter": 5,
    "ICCACCFuncTyp": 4,          # arbitrary "OEM" value
    "ICCACCSwt": 2,
    "ICCACCSpdLimOffs": 20,
    "ICCISASetting": 3,
  })

  # Simulate: pick an arbitrary DataID, compute the "valid" checksum, and
  # stuff it into ICC_0x52ACheckSum. Then run the spoof and verify the new
  # checksum is still valid under that same DataID.
  for hypothetical_data_id in (0x00, 0x2A, 0x7F, 0xFF):
    vals["ICC_0x52ACheckSum"] = 0
    _, packed_with_zero_chk, _ = packer.make_can_msg("ICC_0x52A", CANBUS.pt, vals)
    valid_chk = _crc8_j1850(bytes([hypothetical_data_id]) + packed_with_zero_chk[1:])
    vals["ICC_0x52ACheckSum"] = valid_chk

    _, spoofed, _ = fcan.create_icc_spoof_0x52a(vals)
    # Re-derive what a CRC-8 J1850 with that DataID would produce for the spoofed payload.
    scratch = _crc8_j1850(bytes([hypothetical_data_id]) + spoofed[1:])
    assert spoofed[0] == scratch, (
      f"spoof checksum invalid under data_id={hypothetical_data_id:02X}: "
      f"got {spoofed[0]:02X}, expected {scratch:02X}"
    )
