from opendbc.car import get_safety_config, structs
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.car.fisker.carcontroller import CarController
from opendbc.car.fisker.carstate import CarState
from opendbc.car.fisker.values import FiskerSafetyFlags


class CarInterface(CarInterfaceBase):
  CarState = CarState
  CarController = CarController

  @staticmethod
  def _get_params(ret: structs.CarParams, candidate, fingerprint, car_fw,
                  alpha_long, is_release, docs) -> structs.CarParams:
    ret.brand = "fisker"
    ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.fisker)]

    ret.steerControlType = structs.CarParams.SteerControlType.angle
    ret.steerActuatorDelay = 0.15    # was 0.20 — Ocean EPS is faster than default; the
                                     # higher delay made MPC over-anticipate the commanded
                                     # angle, causing overshoot on curve entry/exit.
    ret.steerLimitTimer = 0.4
    ret.steerAtStandstill = True

    # SecOC is mandatory on Ocean. The 16-byte AES key must be in params under "SecOCKey".
    ret.secOcRequired = True
    ret.safetyConfigs[0].safetyParam |= FiskerSafetyFlags.SECOC.value

    ret.alphaLongitudinalAvailable = True
    if alpha_long:
      ret.openpilotLongitudinalControl = True
      ret.safetyConfigs[0].safetyParam |= FiskerSafetyFlags.LONG_CONTROL.value
      ret.longitudinalActuatorDelay = 0.30
      # vEgoStopping / vEgoStarting were moved to the CarParams deprecated group in
      # this openpilot release — they no longer gate the stop/start state machine, so
      # we don't set them here. Longitudinal stop/start defaults for the Ocean can be
      # tuned via stopAccel / startAccel later once we have on-vehicle logs.

    ret.steerAtStandstill = True
    ret.minSteerSpeed = 0.0

    # No usable radar. The ADAS module publishes a fused object list on ADASBUS
    # (ADAS_Obj0..7, 0x32D-0x34F, 25 Hz) with position + class + a lead pointer,
    # but NO relative velocity — and the raw radar tracks (with velocity) are on a
    # private radar<->ADAS SecOC link we don't tap. openpilot needs dRel+vRel, so
    # longitudinal uses vision-based lead detection (like Tesla), not radar.
    ret.radarUnavailable = True

    # Lateral-only by default: openpilotLongitudinalControl stays False unless alpha_long
    # is enabled, and the fisker panda safety mode without the LONG flag has no accel
    # (0x121) in its TX allow-list, so the firmware physically blocks longitudinal — only
    # steering (0x1D0) can go out. Engagement is additionally gated by secOcRequired +
    # GW-sync key verification in the carcontroller (wrong/absent key => no steering).
    ret.dashcamOnly = False
    return ret
