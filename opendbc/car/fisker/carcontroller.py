import numpy as np

from opendbc.can import CANPacker
from opendbc.car import Bus, structs
from opendbc.car.carlog import carlog
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.lateral import apply_std_steer_angle_limits

from opendbc.car.fisker.fiskercan import FiskerCAN
from opendbc.car.fisker.secoc import stamp_secoc, sync_mac
from opendbc.car.fisker.values import CarControllerParams
from opendbc.sunnypilot.car.fisker.values_ext import FiskerFlagsSP


VisualAlert = structs.CarControl.HUDControl.VisualAlert

# CAN IDs of the SecOC-protected actuator messages we transmit.
STEER_CAN_ID = 0x1D0
ACCEL_CAN_ID = 0x121

# Phone-over-wifi remote-control gating. See
# openpilot/sunnypilot/selfdrive/remote_control/remote_control_server.py for
# the Params producer. All three are CLEAR_ON_MANAGER_START so a reboot
# always comes up with the app disabled.
#   * stale after this long: carcontroller falls back to sunnypilot's planner.
#     Chosen to be ~10× the server's command interval (~33 ms @ 30 Hz) so a
#     single dropped packet is survivable but a dead wifi link disengages
#     within one 1/3 second window.
REMOTE_STALE_NS = 300_000_000  # 300 ms
#   * low-speed-only. 10 mph in m/s. Above this we refuse to apply the remote
#     angle no matter what the server says — a wifi-latency-limited slider is
#     not an acceptable highway input device.
REMOTE_MAX_SPEED_MS = 10 * 0.44704  # ~4.47 m/s
#   * poll the params lazily — opendbc test harness on macOS doesn't have
#     libparams_c and Params() crashes test_car_interfaces at import. On
#     device the import succeeds. Follows the same shape as
#     carstate._set_mads_arm above.


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
    # index for anti-replay, so we must reproduce it exactly.
    #
    # Counter policy (confirmed on-vehicle 2026-10-19, user's model): SNAP ONCE at the
    # block moment to OEM's reconstructed full counter + 1, then free-run +1 per tick.
    # We become OEM's exact continuation: at the first TX, OEM just delivered counter N
    # to the EPS, we send N+1; we then march in lockstep with OEM's (now-blocked) bus-2
    # stream, so at disengage OEM resumes at our_last + 1 — clean handoff both ways.
    # Snap-once (not continuous-mirror) is essential: mirroring OEM every tick emits a
    # DUPLICATE whenever our 100 Hz read phase catches the same OEM frame twice, which
    # the EPS rejects. Re-snap on a GW Reset boundary (counter restarts there). 0x1D0 and
    # 0x121 share the index (same 100 Hz restart), so the longitudinal path uses it too.
    self.secoc_window_ctr = 0
    self.was_tx_secoc_prev = False  # rising edge of "transmitting any SecOC PDU" -> seed counter once

    self.secoc_key_verified = False
    self.secoc_warn_logged = False

    # Once openpilot has been long-active at least once, keep the ADAS heartbeat going for
    # the rest of the drive. If we go silent between engagements the ESP loses our 0x118 AEB
    # state (falls into "AEB unavailable" fault) and the VCU's E2E AliveCounter validator
    # rejects our first re-engage frames as out-of-sequence — surfacing as "ADAS error,
    # emergency brake unavailable" on the cluster and a cruise fault in openpilot.
    self.long_ever_active = False

    # "We ARE OEM" AliveCounter substitution: every OEM 0x1D0/0x1C0 tick that panda
    # blocks, we transmit a frame with the SAME AliveCounter that OEM would have sent —
    # our stream literally continues OEM's numbering during the blocked window. At
    # disengage panda unblocks OEM, and OEM's next tick lands at our_last + 1 which is
    # exactly what EPS was expecting next — seamless handoff both directions, no
    # momentary BSM error on disengage (see user's diagram). Only transmit when OEM's
    # bus-2 counter has advanced since our previous send, else we'd emit a duplicate
    # (EPS rejects duplicates in the E2E monotonic check).
    self.alive_1d0 = 0
    self.alive_1c0 = 0
    self.was_lat_engaged_prev = False   # tracks lat_engaged (MADS) rising edge for engage delay
    self.was_tx_lateral_prev = False    # rising edge of lateral TX -> snap alive counters from OEM

    # ICC 0x52A spoof: emit ONE frame per OEM tick (detected by AliveCounter change).
    # ICC broadcasts at ~10 Hz while we control at 100 Hz, so a naive tx-every-tick
    # would emit ~10 duplicates per OEM tick — the receiver's E2E monotonic-counter
    # validator would flag those as replays and reject them.
    self.last_spoofed_icc_alive = -1
    # EPS_0x1C2 spoof: same cadence rule (one spoof per real OEM tick, ~50 Hz).
    self.last_spoofed_eps_1c2_alive = -1

    # Lateral fade-out: ticks remaining of post-disengage grace. On MADS falling edge
    # we extend the TX window by this many ticks so the EPS sees Req=0 for a beat
    # and transitions Active→Available smoothly, before our stream fully drops. Then
    # the panda lateral relay (60 ms) lets OEM resume to the EPS right after our last
    # fade frame — our counter is OEM's exact continuation so OEM picks up at our_last+N
    # with a tiny N (clean handoff). Keep the fade short: 10 ticks @ 100 Hz = 100 ms is
    # plenty for the EPS to see Req=0 and release, and keeps the total post-disengage
    # blackout (fade + relay) to ~160 ms instead of the old ~1 s. Was 50 (500 ms).
    self.lat_fade_remaining = 0
    self.FADE_TICKS = 10

    # Engagement delay: wait this many ticks after lat_engaged rising edge
    # before we start TX'ing. Between our carstate's OEM-counter snapshot
    # and our first TX arriving at EPS, OEM can send one more 0x1D0 frame
    # on bus 2 that panda forwards to bus 0 (panda's lateral relay only
    # activates after our FIRST TX). If that OEM frame beats our TX to EPS,
    # EPS's last-accepted counter == our TX counter → duplicate → abort=8.
    # We re-snap each tick during the delay to track OEM's latest wire value.
    # 5 ticks (50 ms) is well beyond any plausible pandad/USB jitter window
    # and ~5x the OEM frame period, so we're guaranteed to have observed
    # OEM's post-engage counter at least a few times before we TX.
    self.engage_delay_remaining = 0
    self.ENGAGE_DELAY_TICKS = 5

    # Phone-over-wifi remote-control state. Lazy-init'd on first update() tick
    # so the opendbc test harness (which has no libparams_c on macOS) can
    # instantiate CarController without blowing up. See REMOTE_* constants.
    self._remote_params = None
    self._remote_warned = False

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

  def _remote_steer_override(self, planner_angle_deg: float, v_ego: float, now_nanos: int) -> float:
    """If the phone remote-control app is armed AND fresh AND we're below the low-speed
    cap, replace the lateral planner's commanded steering angle with the phone's angle.
    All downstream safety (rate limiter, driver intervention, SecOC, panda tx_hook)
    runs on top of the substituted value unchanged — same as if sunnypilot itself had
    commanded that angle.

    Returns the (possibly substituted) commanded angle. Fails open: any error
    (Params unavailable, parse failure, stale timestamp, overspeed) falls back to the
    planner angle so sunnypilot continues to control steering normally."""
    if self._remote_params is None:
      try:
        from openpilot.common.params import Params
        self._remote_params = Params()
      except Exception:  # noqa: BLE001 — no params → never override
        return planner_angle_deg

    try:
      if not self._remote_params.get_bool("RemoteControlEnabled"):
        return planner_angle_deg
      last_raw = self._remote_params.get("RemoteSteerLastUpdate")
      angle_raw = self._remote_params.get("RemoteSteerAngle")
      if not last_raw or not angle_raw:
        return planner_angle_deg
      last_ns = int(last_raw.decode() if isinstance(last_raw, (bytes, bytearray)) else last_raw)
      remote_angle = float(angle_raw.decode() if isinstance(angle_raw, (bytes, bytearray)) else angle_raw)
    except Exception:  # noqa: BLE001 — any corruption → fall back to planner
      return planner_angle_deg

    # Freshness watchdog. The phone POSTs /steer at ~30 Hz; a connection drop
    # or wifi stall shows up here as the age blowing past REMOTE_STALE_NS and
    # we silently hand steering back to sunnypilot's planner. The server-side
    # watchdog on the carcontroller is the authoritative one — the server
    # also auto-disarms if the UI loses connectivity, but we don't rely on
    # that because the carcontroller may still have a stale "armed=True" with
    # a last-update timestamp hours old.
    if now_nanos - last_ns > REMOTE_STALE_NS:
      return planner_angle_deg
    # Low-speed only. The whole point of this feature is to steer the car
    # at parking speeds from outside the vehicle; a wifi-latency-limited
    # slider is not a replacement for an ADAS at any speed above walking pace.
    if v_ego > REMOTE_MAX_SPEED_MS:
      return planner_angle_deg
    return remote_angle

  def update(self, CC, CC_SP, CS, now_nanos):
    actuators = CC.actuators
    can_sends = []

    self._maybe_verify_key(CS)
    secoc_ok = self.CP.secOcKeyAvailable and self.secoc_key_verified
    trip, reset = CS.secoc_trip, CS.secoc_reset

    # SecOC + alive counters are seeded-once-at-block then free-run — see the block
    # right before the lateral TX below (and __init__ for the policy rationale).

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
    # Lateral TX is driven by MADS, NOT by cruise. In ACC-only mode (RiBtnNorth engaged
    # ACC without arming MADS) openpilot stays transparent on the lateral path so the
    # OEM ADAS keeps running its own LKA (gated by the user's ICC_LKASetting). The
    # panda's fisker_fwd_hook has the matching rule: it only blocks OEM 0x1D0/0x1C0 on
    # `controls_allowed_lateral`, not plain `controls_allowed` — otherwise the OEM
    # thinks LKA has become unavailable because its 0x1D0 never reaches bus 0.
    #
    # Engagement variables:
    #   `engaged`     – ACC OR MADS. Gates the long path + SecOC window-counter housekeeping.
    #   `lat_engaged` – MADS only. Gates the lateral TX (0x1D0 + 0x1C0) and the
    #                   OEM-counter seeding (which only matters when we're about to start
    #                   transmitting). Rising edge of lat_engaged is the moment we snap
    #                   our aliveCounter / SecOC wire counter to OEM's current value so
    #                   our first TX continues OEM's numbering seamlessly.
    #
    # Driver-override release. CS.driver_intervening is driven by the EPS's dedicated
    # EPS_DrvrIntvSteerWhlDetd bit (0x1C4) — a conservative "real driver overpower"
    # signal that EPS computes with internal knowledge of its motor contribution, so
    # it doesn't false-fire from reaction torque the way a plain column-torque
    # threshold does. When it trips we drop lat_active so ADAS_LatCtrl_Req=0 and the
    # EPS stops servoing. We intentionally do NOT key this off CS.out.steeringPressed
    # — that's the sensitive, column-torque-based "driver is touching the wheel"
    # signal, and consuming it here caused the EPS to release on every light nudge
    # (which also killed sunnypilot's auto-lane-change nudge detection since ALC
    # watches steeringPressed too). Separating the two sensitivities lets light
    # nudges feed ALC while hard overpower releases the EPS.
    # Use mads.active (= state ∈ {enabled, softDisabling, overriding}) rather than
    # mads.enabled, which also returns True for State.paused. MADS can enter paused
    # directly from disabled (gear/door/seatbelt NO_ENTRY condition + silent LKAS
    # disable), which has no engagement event — so keying off `enabled` makes
    # lat_engaged latch True indefinitely from startup and we TX zeros at 100 Hz
    # even in pure ACC-only. The EPS then sees our Sts=0 stream racing OEM's own
    # lateral once ACC becomes Active and raises abort=8 (CAN).
    #
    # mads.active only lights up when MADS is actually commanding steering, which
    # is the only time we have any business TXing on 0x1D0/0x1C0.
    mads_active = bool(CC_SP.mads.active) if CC_SP is not None else False
    engaged = (CS.out.cruiseState.enabled or mads_active) and secoc_ok
    lat_engaged = mads_active and secoc_ok
    lat_active = CC.latActive and secoc_ok and not CS.driver_intervening

    # Phone remote-control substitution: swap the planner's commanded angle for
    # the phone-supplied angle BEFORE the rate limiter. Keeping it before the
    # rate limiter means the exact same slew/saturation limits that constrain
    # the sunnypilot planner also constrain the phone — the phone can't send a
    # ±600° step command and bypass the EPS's safety envelope. All of our other
    # safety (SecOC, driver intervention, panda tx_hook angle rate limits) is
    # downstream of this substitution and runs unchanged.
    #
    # Gated by lat_engaged: MADS still has to be commanding steering for the
    # phone's commands to reach the EPS. If MADS isn't engaged, we're not
    # transmitting 0x1D0 anyway and the substituted actuators.steeringAngleDeg
    # just affects what new_actuators carries back on the car bus for logging.
    desired_angle_deg = float(actuators.steeringAngleDeg)
    if lat_engaged:
      desired_angle_deg = self._remote_steer_override(desired_angle_deg,
                                                       CS.out.vEgoRaw, now_nanos)
    self.apply_angle_last = apply_std_steer_angle_limits(
      desired_angle_deg, self.apply_angle_last, CS.out.vEgoRaw,
      CS.out.steeringAngleDeg, lat_active, self.params.ANGLE_LIMITS,
    )

    # "We ARE OEM" counter continuation. All three counters (alive_1d0, alive_1c0,
    # secoc_window_ctr) are mirrored from OEM's authoritative bus-2 values + 1 at the
    # top of update() — see there. OEM's counters advance in real time regardless of
    # our control-loop jitter, so mirroring them keeps our injected stream on exactly
    # the value the EPS expects next, both at the engage handoff and on every frame
    # after. Panda blocks OEM's own 0x1D0/0x1C0 from the EPS while we TX, so the EPS
    # sees a seamless continuation of OEM's numbering through our frames.
    #
    #   OEM on bus 2 (all ticks):        1  2  3  4  5  6  7  8  9  10 11 12
    #   Panda forwards to bus 0:         1  2  3  4                    11 12
    #                                    └── ENGAGE ──┘     └─ DISENGAGE
    #   Our TX (= OEM_bus2 + 1):                   6  7  8  9  10 11
    if lat_engaged and not self.was_lat_engaged_prev:
      # Rising edge: start the engage-delay window. During the delay we DO NOT TX —
      # we let OEM's own 0x1D0/0x1C0 keep flowing to the EPS so the handoff point is
      # clean, and our counters are already mirroring OEM continuously (set at the
      # top of update()), so the first TX after the delay lands on OEM_current + 1.
      self.engage_delay_remaining = self.ENGAGE_DELAY_TICKS
    if lat_engaged and self.engage_delay_remaining > 0:
      self.engage_delay_remaining -= 1
    # Fade-out window: on lat_engaged falling edge we extend the TX window by FADE_TICKS
    # so the EPS keeps receiving our Req=0 stream for a beat after disengage. During
    # fade: panda's time-based lateral relay keeps blocking OEM 0x1D0/0x1C0, the EPS
    # 0x1C2 spoof keeps running (OEM ADAS still sees EPS=Available), and the EPS
    # transitions Active→Available smoothly off our commands rather than off a sudden
    # OEM-command injection. The 0x1C0 wire fields during fade keep Sts=1/Typ=1 (we
    # haven't "left") and only drop Req to 0 — observed on-vehicle 2026-10-18 that
    # slamming Sts/Typ to 0 at disengage caused EPS abort=8 the same way the mid-
    # engage driver-override transition did.
    # Fade is PRIMED (reset to FADE_TICKS) on every engaged tick so that the
    # moment we disengage it's already full. It's CONSUMED (counted down)
    # only when not engaged. The subtle part: `tx_lateral` must only treat
    # the fade as "active" during the post-disengage window — never during
    # engagement — otherwise priming the fade on the rising edge immediately
    # satisfies the TX gate and we bypass the engage delay. That's exactly
    # what showed up on-vehicle 2026-10-19: first TX was happening ~15 ms
    # into engagement (one tick) instead of ~50 ms (after the 5-tick delay),
    # which bypassed the OEM-counter settle and let the first few SecOC
    # frames go out with a counter the EPS hadn't caught up to yet.
    if lat_engaged:
      self.lat_fade_remaining = self.FADE_TICKS
      in_fade = False
    else:
      in_fade = self.lat_fade_remaining > 0
      if in_fade:
        self.lat_fade_remaining -= 1
    self.was_lat_engaged_prev = lat_engaged
    tx_lateral = (lat_engaged and self.engage_delay_remaining == 0) or in_fade

    # ---- SecOC + alive counter: seed once at the block moment, then free-run +1 ----
    # We become OEM's exact continuation. On the first TX after an idle gap (the
    # "block moment", i.e. when the panda relay starts stealing OEM's frames), snap
    # to OEM's reconstructed full counter + 1: OEM just delivered N to the EPS, so we
    # send N+1. Then free-run +1 per tick FOREVER (until we stop TXing), marching in
    # lockstep with OEM's now-blocked bus-2 stream — so at disengage OEM resumes at
    # our_last + 1, never a repeat. Snap ONCE (not every tick): re-reading OEM emits a
    # duplicate whenever our 100 Hz read phase lands on the same OEM frame twice, which
    # the EPS rejects (on-vehicle 2026-10-19: continuous-mirror produced STEP-0
    # duplicates).
    #
    # Do NOT re-snap on a GW reset. The msg_counter is continuous across resets
    # (verified on-vehicle: it stepped 37->38 through a reset, did not restart). The
    # separate GW `reset` field rides into the wire byte + MAC from CS.secoc_reset on
    # its own; it must not disturb our free-running msg_counter. An earlier version
    # re-snapped on reset change and jolted the counter at the ~1 Hz reset boundary,
    # faulting the EPS mid-engagement.
    long_txing = self.CP.openpilotLongitudinalControl and (engaged or self.long_ever_active)
    txing_secoc = tx_lateral or long_txing
    if txing_secoc:
      if not self.was_tx_secoc_prev:
        self.secoc_window_ctr = int(CS.oem_1d0_secoc_full_ctr) + 1
      else:
        self.secoc_window_ctr += 1
    self.was_tx_secoc_prev = txing_secoc

    # Lateral E2E alive counters (0x1D0 / 0x1C0): same seed-once-then-+1, from each
    # PDU's own OEM source. These are independent 4-bit counters that do NOT restart
    # on a GW Reset, so no reset re-seed — just wrap mod 15.
    if tx_lateral:
      if not self.was_tx_lateral_prev:
        self.alive_1d0 = (int(CS.oem_1d0_alive) + 1) % 15
        self.alive_1c0 = (int(CS.oem_1c0_alive) + 1) % 15
      else:
        self.alive_1d0 = (self.alive_1d0 + 1) % 15
        self.alive_1c0 = (self.alive_1c0 + 1) % 15
    self.was_tx_lateral_prev = tx_lateral

    if tx_lateral:
      # Counters (alive_1d0/1c0, secoc_window_ctr) were seeded/incremented just above.
      # Lateral control type: sunnypilot FiskerLateralType toggle. Flag set = LCA/TJA
      # (Typ=3), flag clear = LKA (Typ=1, baseline). The Ocean's EPS has historically
      # only honoured LKA, so defaulting to 1 keeps the port working.
      lat_ctrl_typ = 3 if (self.CP_SP.flags & FiskerFlagsSP.LAT_CTRL_LCA.value) else 1
      # Two independent wire bits (see create_lat_control docstring for the full why):
      #   * `lat_engaged_wire` — are we in an engaged lateral session? Drives
      #     Sts/Typ/ReqVld. Stable True across driver-override transients AND
      #     across the fade-out window — only drops once the fade has actually
      #     elapsed and we're about to stop transmitting entirely.
      #   * `lat_active_wire`  — on this tick, are we actively servoing the angle
      #     or backing off for the driver? Drives Req only.
      #
      # Both faults observed on-vehicle 2026-10-18 came from slamming Sts/Typ
      # to 0 while EPS was still in state Active: once on the first driver-
      # torque pulse (carControl.latActive dropped), once on MADS release
      # (lat_engaged dropped and the old fade code TX'd Sts=0/Typ=0). Splitting
      # the bits so Sts/Typ stay 1 while only Req follows lat_active keeps the
      # EPS in Active state right up to the moment we actually stop transmitting.
      lat_engaged_wire = lat_engaged or (self.lat_fade_remaining > 0)
      lat_active_wire = lat_active and lat_engaged
      steer_msg = self.fcan.create_steering_control(self.apply_angle_last, self.alive_1d0)
      can_sends.append(self._stamp(steer_msg, STEER_CAN_ID, trip, reset, self.secoc_window_ctr))
      can_sends.append(self.fcan.create_lat_control(lat_active_wire, self.alive_1c0,
                                                    driver_override=CS.driver_intervening,
                                                    lat_ctrl_typ=lat_ctrl_typ,
                                                    lat_engaged=lat_engaged_wire))

      # EPS_0x1C2 spoof: DISABLED. On-vehicle testing with the verified checksum
      # algorithm (CRC-8 J1850, data_id=0x90) STILL caused ADAS errors in MADS and
      # Both modes, even though the frames themselves are checksum-valid and the
      # byte-6 override does what it claims. Hypothesis: the OEM ADAS doesn't only
      # look at EPS_AdasLatCtrlSts on 0x1C2 — it probably cross-references with
      # EPS_0x1C4 (driver torque) and/or EPS_0x475 (EPS mode) which we don't spoof,
      # so the "Available" story we're telling on 0x1C2 contradicts the "actively
      # being moved" story the other EPS signals still carry. Spoofing all three
      # consistently would be the next step, but requires brute-forcing more
      # checksums and risks more side-effects on the cluster. The fade-out alone
      # addresses the disengage transient, which is the user-visible symptom.
      #
      # Everything stays in place for easy revival once we have a reason to try
      # again with a multi-signal spoof:
      #   * E2E_PARAMS entry (0x1C2: (0x90, 64)) in fiskercan.py — verified against
      #     200 captured frames (test_checksum.py), can't hurt to keep.
      #   * create_eps_spoof_0x1c2() in fiskercan.py — self-contained helper,
      #     passes its own tests.
      #   * EPS_0x1C2_SIGNALS + the carstate snapshot — same, harmless unused.
      # if self.last_spoofed_eps_1c2_alive != CS.eps_1c2_alive and CS.eps_1c2_values:
      #   can_sends.append(self.fcan.create_eps_spoof_0x1c2(CS.eps_1c2_values))
      #   self.last_spoofed_eps_1c2_alive = CS.eps_1c2_alive

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

    # ---- ICC 0x52A spoof (bus 2) ----
    # Replace the OEM ICC settings frame flowing bus 0 -> bus 2 with a mutated copy
    # that forces byte 4 to 0x5B (ICCACCSwt=1 + ICCACCFuncTyp=2 + 3 more — see
    # create_icc_spoof_0x52a). Everything else (AliveCounter, reserved bits, other
    # settings) passes through byte-identical, and the CheckSum is patched via XOR
    # delta (no need to know 0x52A's DataID).
    #
    # IMPORTANT: this must be sent UNCONDITIONALLY, not gated on
    # cruiseState.enabled. ADAS stays at ACC_Off until it sees ICC_ACCSwt=1 (and the
    # other byte-4 fields) in a valid 0x52A — i.e. the spoof is what enables ACC to
    # transition out of Off into Standby/Active. Gating on cruiseState.enabled was
    # chicken-and-egg: cruise can't become enabled without the spoof going out
    # first, and the spoof was never sent because cruise wasn't enabled.
    #
    # Cadence: one spoof per OEM tick (detected by AliveCounter change) so we never
    # emit duplicates that would trip ADAS's E2E monotonic-counter check. ICC runs
    # at ~5 Hz; openpilot runs at 100 Hz, so most carcontroller ticks won't fire.
    #
    # Panda's fisker_fwd_hook blocks the OEM's bus-0 -> bus-2 forwarding of 0x52A
    # whenever openpilot has recently emitted its own copy (time-gated, not
    # controls_allowed-gated — same reason as above).
    if CS.icc_52a_seen and CS.icc_52a_alive != self.last_spoofed_icc_alive:
      # Honour the sunnypilot UI toggles:
      #   FiskerACCAutoSpeed → ACC_AUTO_SPEED_OFF flag → clears byte-4 bit 35
      #   FiskerACCTerrain   → ACC_TERRAIN_ON flag    → sets byte-6 bit 0
      # Flags are stored in whichever polarity makes "no flag" = baseline behaviour.
      acc_auto_speed = not bool(self.CP_SP.flags & FiskerFlagsSP.ACC_AUTO_SPEED_OFF.value)
      terrain_setting = bool(self.CP_SP.flags & FiskerFlagsSP.ACC_TERRAIN_ON.value)
      can_sends.append(self.fcan.create_icc_spoof_0x52a(CS.icc_52a_values,
                                                        acc_auto_speed=acc_auto_speed,
                                                        terrain_setting=terrain_setting))
      self.last_spoofed_icc_alive = CS.icc_52a_alive

    # ---- HUD ----
    # Forwarding intercept: the OEM ADAS module stays alive on bus 2 and the panda
    # forwards its status/HUD frames (ACC HUD 0x31C, warning HUD 0x317) to the cluster,
    # so openpilot must NOT also transmit them — that would collide with the OEM's. It
    # only injects the steering command; the OEM keeps driving the cluster/HUD.

    new_actuators = actuators.as_builder()
    new_actuators.steeringAngleDeg = self.apply_angle_last

    self.frame += 1
    return new_actuators, can_sends
