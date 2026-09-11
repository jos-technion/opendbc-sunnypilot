from dataclasses import dataclass, field
from enum import IntFlag

from opendbc.car import Bus, CarSpecs, DbcDict, PlatformConfig, Platforms
from opendbc.car.lateral import AngleSteeringLimits
from opendbc.car.structs import CarParams
from opendbc.car.docs_definitions import CarDocs, CarHarness, CarParts
from opendbc.car.fw_query_definitions import FwQueryConfig, Request, StdQueries

Ecu = CarParams.Ecu


@dataclass
class FiskerCarDocs(CarDocs):
  package: str = "All"
  car_parts: CarParts = field(default_factory=CarParts.common([CarHarness.custom]))


@dataclass
class FiskerPlatformConfig(PlatformConfig):
  # Everything openpilot needs is on ADASBUS — the gateway mirrors the body/HMI
  # signals (gear, pedal, doors, seatbelt, blinkers, MFSS buttons) onto it — so
  # the port taps a single bus. fisker_ocean_body remains in opendbc/dbc for
  # reference but is not wired into a CAN parser.
  dbc_dict: DbcDict = field(default_factory=lambda: {
    Bus.pt: 'fisker_ocean_adas',
  })


class CAR(Platforms):
  FISKER_OCEAN = FiskerPlatformConfig(
    [FiskerCarDocs("Fisker Ocean 2023-24")],
    CarSpecs(
      mass=2410.,                # Ocean Extreme curb + typical equipment
      wheelbase=2.921,           # 2921 mm per published Ocean data sheet (was 2.99,
                                 # made the yaw model expect slower turn-in and
                                 # caused MPC to over-command angle on curve entry)
      steerRatio=15.0,           # first-cut; refine from step-response logs
      # centerToFrontRatio kept at default 0.5 — battery-in-floor EV is roughly 50/50.
      tireStiffnessFactor=0.85,  # crossover-typical; default 1.0 assumed sportier tires,
                                 # so MPC expected more grip than the car has → overshoot.
    ),
  )


class CANBUS:
  # Camera-splice harness: panda sits between the vehicle-side gateway (bus 0) and the
  # OEM ADAS module (bus 2). Bus 1 is unused. Verified from a live fingerprint capture:
  # bus 0 has 92 addresses (gateway-mirrored body/HMI + ESP/EPS/YRS/BCM/VCU/MFS/ICC),
  # bus 2 has ~65 addresses (all ADAS-authored: 0x117/0x118/0x121/0x1C0/0x1D0/0x313/
  # 0x314/0x315/0x316/0x317/0x31A/0x31C + ADAS radar object list 0x2C7..0x2E9).
  pt = 0           # ADASBUS, vehicle side (CAN-FD)
  cam = 2          # ADASBUS, OEM ADAS module side (CAN-FD)


# VCU_GearSig enum (from DBC VAL_): 1=P, 2=N, 3=R, 4=D, 6=E, 7=S
GEAR_MAP = {
  0: CarParams.Ecu.unknown,  # placeholder; mapped to GearShifter in carstate.py
}


# MFSS button mapping
BUTTON_MAP = {
  "MFS_RiBtnNorth":  "mainCruise",   # ADAS master on/off
  "MFS_LeRollPress": "setCruise",    # engage cruise
  "MFS_LeRollUp":    "accelCruise",  # +speed
  "MFS_LeRollDwn":   "decelCruise",  # -speed
  "MFS_RiBtnSouth":  "lkas",         # MADS (sunnypilot) toggle
  # cancel / resume / gapAdjustCruise: TBD from bench testing
}


class CarControllerParams:
  # Steering angle command — ADAS_LatCtrl_SteerAnReq in 0x1D0
  # Signal range ±780°, 0.0625°/LSB. Cycle time 10 ms (100 Hz).
  # Fisker's carcontroller uses apply_std_steer_angle_limits (not the VM variant),
  # so ANGLE_LIMITS only carries STEER_ANGLE_MAX + up/down rate lookups.
  ANGLE_LIMITS: AngleSteeringLimits = AngleSteeringLimits(
    600,                                  # STEER_ANGLE_MAX (deg) — DBC permits ±780, conservative 600
    ([0., 5., 25.], [2.5, 1.5, 0.2]),     # ANGLE_RATE_LIMIT_UP (deg/frame by m/s)
    ([0., 5., 25.], [5.0, 2.0, 0.3]),     # ANGLE_RATE_LIMIT_DOWN
  )

  STEER_STEP = 1                          # 0x1D0 sent every frame (10 ms ≈ 100 Hz) — verify cycle from logs
  ACCEL_MAX = 2.0                         # m/s²
  ACCEL_MIN = -3.5                        # m/s²

  # 0x121 ADAS_LgtCtrl_AccelReq scale: 0.0004882 m/s²/LSB, offset -16
  ACCEL_SCALE = 0.0004882
  ACCEL_OFFSET = -16.0


class FiskerSafetyFlags(IntFlag):
  LONG_CONTROL = 1
  SECOC = 2


class FiskerFlags(IntFlag):
  LONG_CONTROL = 1


# SecOC-protected message IDs on ADASBUS (from DBC scan).
# DataId in the AUTOSAR Short SecOC profile equals the CAN message ID for these.
SECOC_TX_MSGS = {
  0x1D0: "ADAS_0x1D0",   # steering angle command
  0x121: "ADAS_0x121",   # accel command
}
SECOC_SYNC_MSG = (0x20, "GW_Syn_All")    # 50 ms, full 40-bit freshness on wire


# No FW-version fingerprinting for Fisker: FW_VERSIONS is intentionally empty
# (single-model port, CAN fingerprint is authoritative), so declare zero requests
# to keep the brand out of fw_versions.get_brand_ecu_matches — otherwise upstream's
# `len(brand_matches[b])` divide-by-zero crashes card at startup. The fw_version_regex
# is not used when requests is empty; supply a matches-nothing pattern for the record.
FW_QUERY_CONFIG = FwQueryConfig(requests=[], fw_version_regex=rb'.*')


DBC = CAR.create_dbc_map()

STEER_THRESHOLD = 0.5  # Nm — driver torque to count as override (EPS_DrvrSteerTq, 0.01 Nm/LSB)
