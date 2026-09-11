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

    # Per-message AliveCounter (E2E) tracked separately from self.frame so we can seed
    # each engagement transition from OEM's last-observed value on bus 2 and stay in
    # lockstep. Without this, on 2nd/3rd re-engage the EPS rejects our first frame as
    # out-of-sequence (jumped from OEM's last-seen to our free-running self.frame%15) and
    # latches into LKA fault ("ADAS error" on cluster). See tizi's parked LKA-unavailable
    # issue for the same phenomenon on longitudinal.
    self.alive_1d0 = 0
    self.alive_1c0 = 0
    self.was_engaged_prev = False

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

    # E2E AliveCounter (byte1 low nibble): 0..14 counter, +1 per frame, never 15 (15 is
    # the E2E invalid sentinel — DBC range [0|14]). For LATERAL (0x1D0/0x1C0) we track
    # per-PDU below so we can align to OEM's most recent bus-2 value at every engage
    # transition — that way our first re-engage frame is (OEM_last + 1), exactly what EPS
    # expects. If we free-ran with self.frame%15 through disengaged periods, EPS's
    # last-accepted-from-OEM would diverge from our next-transmit and the first re-engage
    # frame would be rejected (surfaces as an LKA fault / "ADAS error" on cluster and
    # MADS auto-off). The longitudinal path (0x121/0x117/0x118, alpha_long only) still
    # uses the free-running counter for now — same alignment could help there too but is
    # scoped separately.
    alive = self.frame % 15

    # ---- Lateral (steering angle 0x1D0 + activation 0x1C0 @ 100 Hz) ----
    # Send for the WHOLE engaged window so the cluster/EPS never see the frame disappear.
    # Engagement can come from EITHER regular cruise (cruiseState.enabled) OR sunnypilot
    # MADS (CC_SP.mads.enabled). The panda's fisker_fwd_hook mirrors this: it blocks the
    # OEM's 0x1D0/0x1C0 on bus 2 whenever (controls_allowed || controls_allowed_lateral),
    # which is the exact same window. If openpilot stops transmitting during that window
    # (e.g. MADS engaged without cruise), the EPS receives NEITHER openpilot's frames
    # nor the OEM's (panda blocks the OEM's) → LKA fault + wheel doesn't move.
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
    mads_engaged = bool(CC_SP.mads.enabled) if CC_SP is not None else False
    engaged = (CS.out.cruiseState.enabled or mads_engaged) and secoc_ok
    self.apply_angle_last = apply_std_steer_angle_limits(
      actuators.steeringAngleDeg, self.apply_angle_last, CS.out.vEgoRaw,
      CS.out.steeringAngleDeg, lat_active, self.params.ANGLE_LIMITS,
    )

    # Stay in PERFECT lockstep with OEM's AliveCounter throughout the engaged window,
    # not just at engage. Both OEM and our carcontroller run nominally at 100 Hz, but
    # any jitter causes drift over an engagement session. At disengage, panda unblocks
    # OEM's 0x1D0/0x1C0 → OEM's next frame lands on bus 0 with OEM's counter. EPS was
    # tracking OUR last-sent counter and expects that + 1. If we drifted, OEM's next
    # value doesn't match — EPS latches "one frame off" and BSM flashes an error.
    # Fix: every tick, snap our alive to (OEM_bus2_current + 1). We're always exactly
    # one ahead of OEM's on-wire counter, so at disengage OEM's next = our_last, EPS
    # expected our_last + 1... wait that's still off by one. See below.
    #
    # Actually: we send OEM_current + 1. EPS accepts this (last was OEM_(current-1)
    # from before panda blocked, expects OEM_current, gets OEM_current + 1 — jump of 1
    # from what EPS expected but strictly increasing so most receivers tolerate). Our
    # last-transmitted = OEM_current + 1. Meanwhile OEM's next on bus 2 = OEM_current
    # + 1 (same). When we disengage and panda unblocks, OEM's NEXT after that would be
    # OEM_current + 2 — which is our_last + 1. Match. Handoff seamless.
    #
    # For SecOC msg counter: same idea, snap the low 6 bits to (OEM_wire + 1) mod 64
    # every tick. Upper bits stay from our free-running window index.
    self.alive_1d0 = (int(CS.oem_1d0_alive) + 1) % 15
    self.alive_1c0 = (int(CS.oem_1c0_alive) + 1) % 15
    snap_lo = (int(CS.oem_1d0_secoc_wire_ctr) + 1) & 0x3F
    self.secoc_window_ctr = (self.secoc_window_ctr & ~0x3F) | snap_lo
    self.was_engaged_prev = engaged

    if engaged:
      steer_msg = self.fcan.create_steering_control(self.apply_angle_last, self.alive_1d0)
      can_sends.append(self._stamp(steer_msg, STEER_CAN_ID, trip, reset, self.secoc_window_ctr))
      can_sends.append(self.fcan.create_lat_control(lat_active, self.alive_1c0, driver_override=False))

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
