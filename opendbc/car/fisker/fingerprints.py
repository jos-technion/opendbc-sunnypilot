from opendbc.car.structs import CarParams
from opendbc.car.fisker.values import CAR

Ecu = CarParams.Ecu


# CAN fingerprint — every non-diagnostic address the Fisker Ocean broadcasts on ADASBUS
# (Bus.pt = 0) with the car in Run mode. openpilot's can_fingerprint uses an EXCLUSIVE
# match (see opendbc.car.fingerprints.is_valid_for_fingerprint): the candidate is kept
# only if every observed CAN address < 0x800 is present in this dict with a matching DLC,
# so a minimal "distinctive-only" fingerprint doesn't work — the FULL bus contents are
# required.
#
# Note: this fingerprint reflects one trim/firmware. If a different trim adds or removes
# any address, the exclusive match fails — extend the address set or switch to FW-based
# identification if that becomes an issue.
FINGERPRINTS = {
  CAR.FISKER_OCEAN: [{
    0x020: 8,  0x036: 8,  0x053: 8,  0x059: 8,  0x0ED: 8,  0x102: 8,
    0x112: 8,  0x113: 8,  0x114: 8,  0x115: 8,  0x116: 8,  0x117: 8,
    0x118: 8,  0x119: 8,  0x120: 8,  0x121: 8,  0x125: 8,  0x12A: 8,
    0x150: 8,  0x151: 8,  0x159: 8,  0x170: 8,  0x174: 8,  0x175: 8,
    0x176: 8,  0x177: 8,  0x178: 8,  0x179: 8,  0x1B6: 8,  0x1B8: 8,
    0x1BA: 8,  0x1C0: 8,  0x1C2: 8,  0x1C4: 8,  0x1D0: 8,  0x1FE: 8,
    0x20A: 8,  0x20B: 8,  0x20C: 8,  0x20D: 8,  0x20E: 8,  0x20F: 8,
    0x210: 8,  0x214: 16, 0x219: 8,  0x225: 8,  0x24C: 32, 0x250: 8,
    0x251: 8,  0x252: 8,  0x255: 8,  0x268: 8,  0x2A6: 8,  0x2AC: 8,
    0x2C7: 8,  0x2CA: 8,  0x2CD: 8,  0x2D0: 8,  0x2D3: 8,  0x2D6: 8,
    0x2D9: 8,  0x2DC: 8,  0x2DF: 8,  0x2E2: 8,  0x2E5: 8,  0x2E8: 8,
    0x2E9: 8,  0x2EA: 8,  0x2FF: 8,  0x30A: 8,  0x310: 8,  0x311: 8,
    0x313: 8,  0x314: 8,  0x315: 8,  0x316: 8,  0x317: 8,  0x318: 8,
    0x31A: 8,  0x31B: 8,  0x31C: 8,  0x321: 8,  0x32B: 8,  0x32D: 8,
    0x32F: 8,  0x331: 16, 0x332: 8,  0x333: 8,  0x334: 8,  0x335: 8,
    0x336: 8,  0x339: 8,  0x33B: 8,  0x33D: 8,  0x33F: 8,  0x340: 8,
    0x342: 8,  0x343: 8,  0x345: 8,  0x348: 8,  0x34B: 8,  0x34D: 8,
    0x34F: 8,  0x350: 8,  0x351: 8,  0x352: 8,  0x353: 8,  0x356: 8,
    0x357: 8,  0x358: 8,  0x359: 8,  0x35B: 8,  0x361: 8,  0x362: 8,
    0x364: 8,  0x369: 8,  0x36B: 8,  0x36D: 8,  0x373: 8,  0x37B: 16,
    0x380: 8,  0x383: 8,  0x387: 8,  0x3DD: 8,  0x429: 8,  0x42D: 8,
    0x42E: 8,  0x43E: 8,  0x471: 8,  0x475: 8,  0x481: 8,  0x482: 8,
    0x487: 8,  0x492: 8,  0x4F0: 8,  0x4F4: 16, 0x507: 8,  0x511: 8,
    0x513: 8,  0x514: 8,  0x51E: 8,  0x525: 8,  0x526: 8,  0x527: 8,
    0x529: 8,  0x52A: 8,  0x531: 8,  0x582: 8,  0x5C4: 16, 0x610: 8,
    0x62F: 8,  0x659: 8,  0x676: 8,  0x67E: 8,  0x680: 8,  0x687: 8,
    0x689: 8,  0x68A: 8,  0x6E0: 8,  0x6E5: 8,  0x6E6: 8,  0x6E7: 8,
  }],
}


# UDS FW-version fingerprints. We do not actually query FW versions on the Ocean —
# FW_QUERY_CONFIG.requests is empty, so no UDS traffic is generated at runtime — but
# match_fw_to_car_exact eliminates a brand candidate by walking its ECU list and
# failing when a live FW doesn't match; with a truly empty ECU dict, the walk is
# skipped and fisker becomes a "matches everything" false-positive against other
# brands' fingerprints (see test_exact_match). This placeholder essential ECU at a
# fisker-unique address ensures the exact matcher can reject fisker for non-Ocean
# live FW versions. Replace with the real EPS FW string once one is captured on
# vehicle; the address 0x7A5 is not queried by any brand today.
FW_VERSIONS: dict = {
  CAR.FISKER_OCEAN: {
    (Ecu.eps, 0x7A5, None): [b'FISKER_OCEAN_PLACEHOLDER'],
  },
}


# WMI (World Manufacturer Identifier) prefixes for VIN-based coarse identification.
FISKER_WMI = {"VCF"}
