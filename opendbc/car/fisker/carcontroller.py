import numpy as np

from opendbc.can import CANPacker
from opendbc.car import Bus, structs
from opendbc.car.carlog import carlog
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.lateral import apply_std_steer_angle_limits

from opendbc.car.fisker.fiskercan import FiskerCAN
from opendbc.car.fisker.secoc import stamp_secoc, sync_mac
from opendbc.car.fisker.values import CarControllerParams


VisualAlert = structs.CarControl.HUDControl.VisualAlert

# CAN IDs of the SecOC-protected actuator messages we transmit.
STEER_CAN_ID = 0x1D0
ACCEL_CAN_ID = 0x121


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP, CP_SP):
    super().__init__(dbc_names, CP, CP_SP)
    self.params = CarControllerParams
    self.packer = CANPacker(dbc_names[Bus.pt])
    self.fcan = FiskerCAN(CP, self.packer)

    self.apply_angle_last = 0.0

    # SecOC message counter. The counter is a per-Reset-window frame index, NOT a free
    # monotonic counter. It restarts at 1 on the first 0x1D0/0x121 frame after the GW
    # Reset counter (0x20) increments, then +1 per 100 Hz frame (reaching ~101 before the
    # next Reset ~1 s later). Only the low 6 bits appear on the wire (SSecOC_Fresh_Byte0);
    # the MAC authenticates the full 64-bit freshness. The EPS reconstructs this window
    # index for anti-replay, so we must reproduce it exactly — a free monotonic counter
    # diverges from the window rule and
    # the EPS rejects every frame (no actuation + ADAS fault). 0x1D0 and 0x121 are both
    # 100 Hz and restart on the same boundary, so they share the index value.
    self.secoc_window_ctr = 0
    self.secoc_prev_reset = None

    self.secoc_key_verified = False
    self.secoc_warn_logged = False

    # Once openpilot has been long-active at least once, keep the ADAS heartbeat going for
    # the rest of the drive. If we go silent between engagements the ESP loses our 0x118 AEB
    # state (falls into "AEB unavailable" fault) and the VCU's E2E AliveCounter validator
    # rejects our first re-engage frames as out-of-sequence — surfacing as "ADAS error,
    # emergency brake unavailable" on the cluster and a cruise fault in openpilot.
    self.long_ever_active = False

  def _maybe_verify_key(self, CS) -> None:
    """Verify the stored SecOC key against the GW sync MAC once at startup."""
    if self.secoc_key_verified or not CS.secoc_sync_seen or not self.secoc_key:
      return
    if sync_mac(self.secoc_key, CS.secoc_trip, CS.secoc_reset) == CS.secoc_sync_mac:
      self.secoc_key_verified = True
      carlog.info("Fisker SecOC key verified against GW sync MAC")
    elif not self.secoc_warn_logged:
      carlog.error("Fisker SecOC key mismatch — GW sync MAC does not match stored SecOCKey")
      self.secoc_warn_logged = True

  def _stamp(self, msg, can_id, trip, reset, msg_counter):
    """Fill the SecOC tail (fresh byte + MAC) of a packed frame with the per-window
    message counter (see self.secoc_window_ctr)."""
    addr, data, bus = msg
    stamped = stamp_secoc(self.secoc_key, can_id, data, trip, reset, msg_counter)
    return addr, stamped, bus

  def update(self, CC, CC_SP, CS, now_nanos):
    actuators = CC.actuators
    can_sends = []

    self._maybe_verify_key(CS)
    secoc_ok = self.CP.secOcKeyAvailable and self.secoc_key_verified
    trip, reset = CS.secoc_trip, CS.secoc_reset

    # Maintain the per-Reset-window SecOC frame index (see __init__). It free-runs at the
    # 100 Hz control rate and restarts on every GW Reset change, so at engagement it
    # already matches the OEM/EPS window position: the first injected 0x1D0 is accepted
    # and every subsequent window stays in lockstep (both restart at 1 on each boundary).
    if reset != self.secoc_prev_reset:
      self.secoc_window_ctr = 0
      self.secoc_prev_reset = reset
    self.secoc_window_ctr += 1

    # E2E AliveCounter (byte1 low nibble): free 0..14 counter, +1 per frame, never 15
    # (15 is the E2E invalid sentinel — DBC range [0|14]).
    alive = self.frame % 15

    # ---- Lateral (steering angle 0x1D0 + activation 0x1C0 @ 100 Hz) ----
    # Send for the WHOLE engaged window (cruiseState.enabled matches the panda's
    # controls_allowed / 0x1C0 forwarding-block window) so the cluster/EPS never see the
    # frame disappear.
    #
    # DISABLED: driver-torque override (release EPS on steeringPressed). The Ocean's
    # EPS_DrvrSteerTq reads torque on the whole steering column — including reaction
    # torque from the ADAS assist itself while it's actively steering. That meant
    # `steeringPressed` fired even when the driver wasn't touching the wheel, dropping
    # Req=0 and cancelling engagement. Until we can decouple driver torque from motor
    # reaction (either a smarter threshold, a longer debounce, or a different sensor
    # input), just gate Req on CC.latActive directly and let openpilot's stock nudge
    # behaviour handle overrides. `driver_override` on 0x1C0 is Gateway-only (EPS doesn't
    # read it) so we send False to keep the wire quiet.
    lat_active = CC.latActive and secoc_ok
    engaged = CS.out.cruiseState.enabled and secoc_ok
    self.apply_angle_last = apply_std_steer_angle_limits(
      actuators.steeringAngleDeg, self.apply_angle_last, CS.out.vEgoRaw,
      CS.out.steeringAngleDeg, lat_active, self.params.ANGLE_LIMITS,
    )
    if engaged:
      steer_msg = self.fcan.create_steering_control(self.apply_angle_last, alive)
      can_sends.append(self._stamp(steer_msg, STEER_CAN_ID, trip, reset, self.secoc_window_ctr))
      can_sends.append(self.fcan.create_lat_control(lat_active, alive, driver_override=False))

    # ---- Longitudinal (accel 0x121 + status 0x117/0x118 @ 100 Hz) ----
    # Same architecture as lateral (see comment above): send the whole triple across the
    # WHOLE engaged window so the VCU/ESP never see the ADAS heartbeat vanish. Content
    # modulates on state:
    #   engaged + long_active + not gas_override: Sts=Active(3) + Typ=ACC(1), AccelVld=1,
    #                                             AccelReq = openpilot's commanded accel
    #   engaged + gas_override:                    Sts=Active(3) + Typ=ACC(1), AccelVld=1,
    #                                             AccelReq = 0 (yield to driver pedal —
    #                                             the VCU arbitrates pedal vs request)
    #   engaged + !long_active:                    Sts=Active(3) + Typ=ACC(1) but
    #                                             AccelVld=0 (idle heartbeat)
    #   !engaged:                                   frames not sent (nothing on bus 2 to
    #                                             forward on this trim, but if the OEM
    #                                             ADAS SW is ever restored it will get
    #                                             through and Sts=Off/Typ=Not_Active)
    #
    # NOTE: the CC->ACC transition is a value change (Sts 0->3) INSIDE this continuous
    # stream — it is not a one-shot event message. The VCU is designed to hand accel
    # control to the ADAS the moment it sees Sts=Active + Typ=ACC on 0x117.
    if self.CP.openpilotLongitudinalControl:
      long_active = CC.longActive and secoc_ok
      gas_override = CS.out.gasPressed         # driver commanding accel via pedal
      accel_active = long_active and not gas_override
      accel = 0.0 if not accel_active else float(np.clip(actuators.accel,
                                                          self.params.ACCEL_MIN,
                                                          self.params.ACCEL_MAX))
      if long_active:
        self.long_ever_active = True

      # Continue sending the ADAS heartbeat once we've ever been active, so the AliveCounter
      # never has a gap and the ESP never sees 0x118 (AEB state) disappear. Idle values
      # (long_active=False -> Sts=Off, AccelVld=Init) mimic what a real ADAS would publish
      # when powered but not commanding.
      # Braking authorization: VCU-side ACC deceleration requires ADAS_LgtCtrl_Typ=TJA/ICA
      # (= value 2, ACC_Stop_and_Go). That's set inside create_long_status when long_active
      # is True — no per-frame gating needed here. ISA_CutOffReq is a separate Speed-Limit
      # mechanism that requires driver-activated Speed Limiter mode; not the path openpilot
      # needs.

      if engaged or self.long_ever_active:
        # 0x121 accel command (SecOC)
        accel_msg = self.fcan.create_accel_command(accel, accel_active, alive)
        can_sends.append(self._stamp(accel_msg, ACCEL_CAN_ID, trip, reset, self.secoc_window_ctr))

        # 0x117 status + 0x118 ESP handshake (plain E2E, no SecOC)
        can_sends.append(self.fcan.create_long_status(long_active, gas_override,
                                                       gear_req=0, counter=alive))
        can_sends.append(self.fcan.create_long_esp_handshake(long_active, gas_override,
                                                              counter=alive))

    # ---- HUD ----
    # Forwarding intercept: the OEM ADAS module stays alive on bus 2 and the panda
    # forwards its status/HUD frames (ACC HUD 0x31C, warning HUD 0x317) to the cluster,
    # so openpilot must NOT also transmit them — that would collide with the OEM's. It
    # only injects the steering command; the OEM keeps driving the cluster/HUD.

    new_actuators = actuators.as_builder()
    new_actuators.steeringAngleDeg = self.apply_angle_last

    self.frame += 1
    return new_actuators, can_sends
