
from opendbc.can import CANDefine, CANParser
from opendbc.car import Bus, structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import CarStateBase
from opendbc.car.fisker.values import (
  BUTTON_MAP,
  CANBUS,
  DBC,
  STEER_THRESHOLD,
)


ButtonType = structs.CarState.ButtonEvent.Type
GearShifter = structs.CarState.GearShifter

# VCU_GearSig values from DBC VAL_ table
GEAR_MAP = {
  1: GearShifter.park,
  2: GearShifter.neutral,
  3: GearShifter.reverse,
  4: GearShifter.drive,
  6: GearShifter.eco,
  7: GearShifter.sport,
}


# Map from MFSS DBC signal name to openpilot ButtonType
_BUTTON_TYPE = {
  "mainCruise":   ButtonType.mainCruise,
  "setCruise":    ButtonType.setCruise,
  "accelCruise":  ButtonType.accelCruise,
  "decelCruise":  ButtonType.decelCruise,
  # MADS engage/disengage. Sunnypilot's MADS state machine listens for
  # ButtonType.lkas edges in ret.buttonEvents; carrying it through the same
  # BUTTON_MAP → ButtonEvent machinery gives us edge detection for free and
  # keeps MFSS handling in one place.
  "lkas":         ButtonType.lkas,
}
BUTTON_SIGNAL_TO_TYPE = {sig: _BUTTON_TYPE[name] for sig, name in BUTTON_MAP.items()}

# Per-signal predicates for "is this press active" on each MFS button. MFS_0x514 buttons
# are 2-bit (0=No_Pressed, 1=Pressed, 2=Long_Press, 3=Reserved). The car side already
# consumes short-press of MFS_RiBtnSouth for the ACC follow-distance adjustment, so to
# avoid stepping on it we only treat the long-press (value 2) as the MADS trigger.
# Everything else keeps the "any non-zero" meaning.
def _is_pressed(sig: str, cur: int) -> bool:
  if sig == "MFS_RiBtnSouth":
    return cur == 2
  return cur != 0


# Every signal in ICC_0x52A. Enumerated once here so carstate snapshots the whole frame
# and the carcontroller can round-trip it into the spoofed packet with only
# ICCACCFuncTyp overridden. The two "_Rsv" signals cover bits 49 and 55 (undefined in
# the source spec) so byte 6 is fully accounted for.
ICC_0x52A_SIGNALS = (
  "ICC_0x52ACheckSum",
  "ICC_0x52AAliveCounter",
  "ICC_FACMDynmcSenstvty",
  "ICCUsrProfTiGapSet",
  "ICC_LKASetting",
  "ICC_FACMSetting",
  "ICC_AEBSensitivity",
  "ICC_BACMSetting",
  "ICC_BACMSensitivity",
  "ICC_AEBJerkSetReq",
  "ICCActvStyGlblSetting",
  "ICC_TSRSetting",
  "ICC_ESASetting",
  "ICCELKASteeringInterventionSet",
  "ICCLaneTrajectorySetting",
  "ICCACCSwt",
  "ICCACCAutoSpdSts",
  "ICCACCSpdStepSize",
  "ICCACCFuncTyp",
  "ICCACCSpdLimOffs",
  "ICCACCSpdLimOffsTyp",
  "ICCACCTerrainSetting",
  "ICC_0x52A_Rsv49",
  "ICCACCTiGapCfm",
  "ICCISASetting",
  "ICC_0x52A_Rsv55",
  "ICC_FCTASensitivity",
  "ICCISAWarnStopReq",
  "ICC_TLRSetting",
  "ICC_FCTA_Setting",
)


class CarState(CarStateBase):
  def __init__(self, CP, CP_SP):
    super().__init__(CP, CP_SP)
    # The gateway mirrors the body/HMI signals (gear, pedal, doors, seatbelt,
    # blinkers, MFSS buttons) onto ADASBUS, so the port reads everything from
    # a single bus (Bus.pt = ADASBUS). No IBUS1 tap is required.
    self.can_define_pt = CANDefine(DBC[CP.carFingerprint][Bus.pt])

    # SecOC sync state parsed from GW_Syn_All (0x20): the current Trip and Reset
    # counters (shared by all secured PDUs) and the sync MAC for key verification.
    self.secoc_trip = 0
    self.secoc_reset = 0
    self.secoc_sync_mac = b"\x00\x00\x00"
    self.secoc_sync_seen = False

    # OEM ADAS's own counters on 0x1D0 / 0x1C0, snapshotted from bus 2 (cam-side of the
    # camera splice). The EPS validates our re-engage frames against its "last accepted
    # counter + 1" rule — if we transmit a value that isn't OEM_last + 1, EPS rejects and
    # latches an LKA fault. Feeding OEM's latest AliveCounter into our carcontroller lets
    # us seed our tx counter to (OEM + 1) at every engagement transition so hand-offs are
    # seamless (see fisker/carcontroller.py). SecOC wire byte is only 6 bits of the full
    # msg_counter; enough to align lower bits at engage.
    self.oem_1d0_alive = 0
    self.oem_1c0_alive = 0
    self.oem_1d0_secoc_wire_ctr = 0    # (byte >> 2) & 0x3F — lower 6 bits of msg_counter

    # ICC_0x52A cached values. This is the ICC settings frame we spoof on bus 2 when
    # cruise is engaged: we forward every OEM signal untouched EXCEPT ICCACCFuncTyp
    # (which we force to 2 so the ADAS module treats us as "type-2" ACC). Storing the
    # whole signal dict makes the carcontroller a pure passthrough+mutate — every
    # signal ICC set (including the reserved bits ICC_0x52A_Rsv{49,55}) round-trips
    # into our spoofed packet unchanged. `icc_52a_alive` is used to detect a new OEM
    # tick so we emit exactly one spoofed frame per OEM tick (no duplicates, no gaps).
    self.icc_52a_values: dict[str, float] = {}
    self.icc_52a_alive = -1     # sentinel; -1 means "no ICC frame seen yet"
    self.icc_52a_seen = False

    # EPS lateral state cache + fault latches (see update()). Default to Off / No_Abort
    # so a startup race (CS consumed before first 0x1C2) reads as "EPS not yet in
    # control" instead of a false "Active".
    self.eps_lat_sts = 0             # 0=Off, 1=Available_For_Control, 2=Active, 3=Failure
    self.eps_abort = 0               # EPS_AbortFb enum
    self.eps_sts_ever_valid = False  # latches once EPS_AdasLatCtrlStsVld == 1

    # Driver-intervention bit (EPS_DrvrIntvSteerWhlDetd). Separate from steeringPressed:
    # steeringPressed is sensitive (light touch) for auto-lane-change / UI; this is
    # conservative (real overpower) for the EPS release in carcontroller. See update().
    self.driver_intervening = False

    # Per-button MADS arming. Both MFS_RiBtnNorth and MFS_RiBtnEast engage the car's ACC
    # on the vehicle side; this port differentiates them on whether sunnypilot MADS auto-
    # engages lateral alongside. Starts False (ACC-only) each boot — the FiskerMadsArmed
    # param is CLEAR_ON_MANAGER_START so it also comes back False after manager restart.
    # Params is imported lazily (openpilot.common.params requires libparams_c, which isn't
    # built in a plain opendbc test env on macOS); the first time we need to write the
    # sidechannel we construct it, and if even that fails (test harness) we swallow the
    # error so test_car_interfaces still passes.
    self._mads_arm_from_east = False
    self._params = None               # lazily constructed on first write
    self._prev_cc_state = 0           # tracks ADAS_Sts_ACC_ICC for cruise→Off reset edge

    # Button state edge detection
    self._prev_button_state = {sig: 0 for sig in BUTTON_SIGNAL_TO_TYPE}

  def _set_mads_arm(self, armed: bool) -> None:
    """Update the MADS-arm state + its Params sidechannel to sunnypilot MADS.

    Params is imported lazily: the opendbc test harness on macOS doesn't have
    libparams_c built, so instantiating Params at import or __init__ time crashes
    test_car_interfaces. On-device the import succeeds; in tests the ImportError
    is swallowed and the in-memory state still updates (which is all the tests
    see). put_bool failures are swallowed for the same reason."""
    self._mads_arm_from_east = armed
    if self._params is None:
      try:
        from openpilot.common.params import Params
        self._params = Params()
      except Exception:  # noqa: BLE001 - any failure (ImportError, OSError) means "no params available"
        return
    try:
      self._params.put_bool("FiskerMadsArmed", armed)
    except Exception:  # noqa: BLE001
      pass

  def update(self, can_parsers) -> tuple[structs.CarState, structs.CarStateSP]:
    cp_pt = can_parsers[Bus.pt]
    cp_cam = can_parsers[Bus.cam]
    ret = structs.CarState()
    ret_sp = structs.CarStateSP()

    # ---- Speed (wheel speeds + cluster) ----------------------------------
    wsf = cp_pt.vl["ESP_0x115"]
    wsr = cp_pt.vl["ESP_0x116"]
    ret.wheelSpeeds.fl = wsf["ESP_WhlSpd_LF"] * CV.KPH_TO_MS
    ret.wheelSpeeds.fr = wsf["ESP_WhlSpd_RF"] * CV.KPH_TO_MS
    ret.wheelSpeeds.rl = wsr["ESP_WhlSpd_RL"] * CV.KPH_TO_MS
    ret.wheelSpeeds.rr = wsr["ESP_WhlSpd_RR"] * CV.KPH_TO_MS

    ret.vEgoRaw = cp_pt.vl["ESP_0x318"]["ESP_VehSpd"] * CV.KPH_TO_MS
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
    # ICC_DispVehSpd is the number on the cluster, in the driver-selected unit
    # (ICC_DispVehSpdUnit VAL_: 0=KMH, 1=MPH). It MUST be converted with the matching
    # factor — on a US car (mph cluster) treating it as km/h reads ~1.609x low
    # (35 mph shows as 22). vEgoCluster is what the UI displays in preference to vEgo.
    icc = cp_pt.vl["ICC_0x531"]
    icc_to_ms = CV.MPH_TO_MS if icc["ICC_DispVehSpdUnit"] == 1 else CV.KPH_TO_MS
    ret.vEgoCluster = icc["ICC_DispVehSpd"] * icc_to_ms
    ret.standstill = ret.vEgoRaw < 0.01

    # ---- IMU ----
    yrs112 = cp_pt.vl["YRS_0x112"]
    ret.yawRate = yrs112["YRS_YawRate"] * CV.DEG_TO_RAD
    # YRS_LgtAcce is g; convert to m/s^2 via gravity
    ret.aEgo = cp_pt.vl["YRS_0x113"]["YRS_LgtAcce"] * 9.81

    # ---- Steering ----
    eps_ang = cp_pt.vl["EPS_0x1C2"]
    eps_tq = cp_pt.vl["EPS_0x1C4"]
    ret.steeringAngleDeg = eps_ang["EPS_SteerWhlAgSig"]
    # rate from numerical diff is handled by selfdrived; provide raw signal if available

    drvr_tq_dir = eps_tq["EPS_DrvrSteerTqDir"]   # 0=CCW (positive), 1=CW (negative)
    drvr_tq_mag = eps_tq["EPS_DrvrSteerTq"]      # 0..8 Nm (column torque — INCLUDES motor reaction)
    ret.steeringTorque = drvr_tq_mag * (-1.0 if int(drvr_tq_dir) == 1 else 1.0)
    ret.steeringTorqueEps = eps_ang["EPS_AsscMotCrtTq"]

    # Two different "driver is touching the wheel" concepts, with different sources:
    #
    #   ret.steeringPressed  <- light column-torque threshold (sensitive)
    #     Consumed by sunnypilot for auto-lane-change ("blinker on + lean the wheel
    #     that way"), nudge-steer alerts, and the standard steering-pressed UI. A
    #     user tap for a lane change barely registers on the dedicated intervention
    #     bit, so if we drove this off that bit, ALC would silently stop working.
    #     The column torque is noisy during active LKA (includes motor reaction)
    #     but openpilot's lane-change gates it behind blinker anyway, so stray
    #     false-fires don't cause spurious lane changes.
    #
    #   self.driver_intervening  <- EPS's dedicated EPS_DrvrIntvSteerWhlDetd bit
    #     Consumed by carcontroller for the EPS release (drop ADAS_LatCtrl_Req=0
    #     so the EPS stops servoing). We want this to only trip on real driver
    #     overpower, not a tap — the EPS computes it with internal knowledge of
    #     its motor contribution, so it doesn't false-fire from reaction torque
    #     the way a plain column-torque threshold does. Fall back to the torque
    #     threshold only if the EPS reports the intervention bit as
    #     Initializing/Invalid, so we always have *some* release trigger.
    ret.steeringPressed = self.update_steering_pressed(abs(ret.steeringTorque) > STEER_THRESHOLD, 5)
    drvr_intv_vld = int(eps_tq["EPS_DrvrIntvSteerWhlVld"]) == 1
    drvr_intv = int(eps_tq["EPS_DrvrIntvSteerWhlDetd"]) == 1
    if drvr_intv_vld:
      self.driver_intervening = drvr_intv
    else:
      self.driver_intervening = abs(ret.steeringTorque) > STEER_THRESHOLD

    # EPS lateral control state (EPS_AdasLatCtrlSts on 0x1C2):
    #   0=Off, 1=Available_For_Control, 2=Active, 3=Failure
    # When we're commanding steering and the EPS reports anything other than Active or
    # Available (e.g. Failure, or Off while we expect Active), the control loop has
    # unlatched at the EPS end — openpilot previously had no visibility into this and
    # kept TXing angle requests into the void with sunnypilot's UI claiming lateral was
    # engaged. Surface it as steerFaultTemporary so selfdrived mutes the request and
    # the UI shows "Lateral Fault". Clears automatically when the EPS returns to
    # Available/Active.
    #
    # EPS_AbortFb enum (0x1C2):
    #   0=No_Abort, 1=Driver_Interference_StrWhl, 6=EPS_Internal_Failure,
    #   7=Vehicle_Speed_Exceeds_Limits, 8=CAN_communication_issue, 9=Other_Abort_Reasons
    # Driver interference (1) is EXPECTED during co-steering and intentionally does NOT
    # raise a fault — carcontroller already drops Req=0 on steeringPressed to release
    # the EPS. Any other non-zero AbortFb is a real abort worth surfacing.
    eps_lat_sts = int(eps_ang["EPS_AdasLatCtrlSts"])
    eps_lat_sts_vld = int(eps_ang["EPS_AdasLatCtrlStsVld"]) == 1
    eps_abort = int(eps_ang["EPS_AbortFb"])
    # Latch once we've seen the EPS report Valid at least once — avoids faulting
    # during boot while the signal is still Initializing. Reset at ignition cycle.
    self.eps_sts_ever_valid = self.eps_sts_ever_valid or eps_lat_sts_vld
    ret.steerFaultTemporary = self.eps_sts_ever_valid and (
      eps_lat_sts == 3 or (eps_abort not in (0, 1))
    )
    # Stash the raw EPS state for carcontroller / diagnostics.
    self.eps_lat_sts = eps_lat_sts
    self.eps_abort = eps_abort

    # ---- Pedals ----
    # VCU_0x214 is the gateway-mirrored VCU status on ADASBUS (SecOC-protected,
    # but the signal payload is cleartext). Carries gear, accel pedal %, brake.
    vcu = cp_pt.vl["VCU_0x214"]
    ret.gasPressed = vcu["VCU_APSPerc"] > 1.0    # > 1% pedal = pressed
    ret.brakePressed = (cp_pt.vl["ESP_0x318"]["ESP_BrkPedlSts"] == 1) or (vcu["VCU_BrkSig"] == 1)
    # NOTE: ret.brake (analog 0..1 pedal fraction from ESP_0x120 MstCylP) was set in the
    # tizi port. In this openpilot release the CarState.brake field moved to the deprecated
    # group and is no longer consumed by selfdrived — brakePressed alone drives the state
    # machine. We drop the master-cylinder computation entirely; if a future feature needs
    # analog brake force, expose it through CarStateSP instead.

    # ---- Gear ----
    gear_val = int(cp_pt.vl["VCU_0x214"]["VCU_GearSig"])
    ret.gearShifter = GEAR_MAP.get(gear_val, GearShifter.unknown)

    # ---- Doors / seatbelt ----
    doors = cp_pt.vl["BCM_0x343"]
    ret.doorOpen = bool(doors["BCM_DrFrntDoorSts"] or doors["BCM_PasFrntDoorSts"]
                        or doors["BCM_LeReDoorSts"] or doors["BCM_RiReDoorSts"])
    # ACU_BucSwtStFrntDrvr VAL_: 0=Buckled, 1=Not_Buckled, 2=Fault, 3=Invalid
    ret.seatbeltUnlatched = cp_pt.vl["ACU_0x159"]["ACU_BucSwtStFrntDrvr"] == 1

    # ---- Blinkers ----
    bcm335 = cp_pt.vl["BCM_0x335"]
    ret.leftBlinker = bool(bcm335["BCM_LeTrunLampOutpCmd"])
    ret.rightBlinker = bool(bcm335["BCM_RiTrunLampOutpCmd"])

    # ---- Blind spot monitoring ----
    # BSM is authored by the OEM ADAS module on the cam-side bus (Bus.cam = bus 2).
    # 0x315 (BSD_CID_{Le,Ri}DispReq): 4-value threat enum
    # (1=threat, 2=threat+turn-indicator, 3=critical) plus 0=No_threat, 4=Error.
    # 0x314 (BSDSts): 0=Off, 1=Standby, 2=Available, 3=Active, 4=Error — trust the
    # display req only when the feature reports Available/Active so a BSM fault or
    # user-disabled BSM doesn't spuriously block openpilot lane changes.
    # These messages are also physically re-broadcast onto bus 0 by panda forwarding
    # (so the cluster sees them), but pandad records the ORIGINAL src (bus 2), so the
    # CANParser must be attached to bus 2 to receive them.
    bsds_state = int(cp_cam.vl["ADAS_0x314"]["ADAS_BSDSts"])
    bsm_available = bsds_state in (2, 3)
    le_disp = int(cp_cam.vl["ADAS_0x315"]["ADAS_BSD_CID_LeDispReq"])
    ri_disp = int(cp_cam.vl["ADAS_0x315"]["ADAS_BSD_CID_RiDispReq"])
    ret.leftBlindspot = bsm_available and le_disp in (1, 2, 3)
    ret.rightBlindspot = bsm_available and ri_disp in (1, 2, 3)

    # ---- Cruise state (ADAS ACC — ADAS_0x313 + ADAS_0x31C on cam bus) ----
    # Cruise now comes from the ADAS module rather than the VCU basic-CC path. Now that
    # the ICC_0x52A spoof forces ICCACCFuncTyp=2, ADAS enters full ACC mode (not just
    # VCU hold-speed), so its own ADAS_Sts_ACC_ICC and ADAS_AccTrgSpdDisp are the
    # authoritative signals for state + set speed. Both messages originate on bus 2
    # (cam side) — pandad tags them with src=2 even after panda forwards to bus 0, so a
    # CANParser subscribed to bus 0 doesn't see them (see cam parser in
    # get_can_parsers).
    # ADAS_Sts_ACC_ICC enum: 0=ACC_Off, 1=Init, 2=Standby, 3=Active, 4=Override,
    # 5=Standstill_active, 6=Standstill_wait, 7=Deactivation_brake, 8=Deactivation_other,
    # 9=Failure_reversible, 10=Failure_irreversible, 11=Standstill_GoNotification.
    # We treat every "commanding" state (3/4) AND the standstill variants (5/6/11) as
    # engaged — the ACC controller is holding the car in all of them.
    #
    # ADAS_AccTrgSpdDisp (set speed) is 0..254 in the driver-selected unit;
    # 255=no_display. The DBC pairs it with ADAS_DispSpdUnit_ACC but on real cars that
    # bit is hardcoded to 0 (doesn't track the cluster), so trusting it on a car whose
    # ADAS passes the mph cluster number through unchanged silently applies the km/h
    # factor and shows ~0.62x low (20 displayed → 12). ICC_DispVehSpdUnit (read above
    # for vEgoCluster) is the reliable source for the driver's unit, so reuse the
    # icc_to_ms factor here too.
    adas313 = cp_cam.vl["ADAS_0x313"]
    adas31c = cp_cam.vl["ADAS_0x31C"]
    cc_state = int(adas313["ADAS_Sts_ACC_ICC"])
    cc_disp = adas31c["ADAS_AccTrgSpdDisp"]
    cc_speed = 0.0 if cc_disp >= 255 else cc_disp * icc_to_ms

    ret.cruiseState.enabled = cc_state in (3, 4, 5, 6, 11)
    ret.cruiseState.available = cc_state not in (0, 1, 9, 10)
    ret.cruiseState.standstill = cc_state in (5, 6, 11)
    ret.cruiseState.speed = cc_speed
    ret.cruiseState.speedCluster = cc_speed

    # ---- Buttons (MFSS) ----
    # Edge-detect per signal with a signal-specific "is pressed" predicate so that e.g.
    # MFS_RiBtnSouth short press (used by the car for the ACC follow-distance adjustment)
    # doesn't trigger a MADS button event — only its long press does. See _is_pressed.
    #
    # In the same loop we also update the per-button MADS-arm flag:
    #   * RiBtnEast rising → arm (ACC + MADS when cruise engages)
    #   * RiBtnNorth rising → disarm (ACC only, no MADS)
    # The resulting state is written to the FiskerMadsArmed param so sunnypilot MADS
    # (openpilot/sunnypilot/mads/mads.py:block_unified_engagement_mode) can read it.
    # Writes are gated on actual value changes to keep disk I/O at a handful per drive.
    mfs = cp_pt.vl["MFS_0x514"]
    button_events = []
    for sig, btype in BUTTON_SIGNAL_TO_TYPE.items():
      cur = int(mfs[sig])
      prev = self._prev_button_state[sig]
      pressed_now = _is_pressed(sig, cur)
      pressed_prev = _is_pressed(sig, prev)
      rising = pressed_now and not pressed_prev
      if rising:
        button_events.append(structs.CarState.ButtonEvent(pressed=True, type=btype))
        if sig == "MFS_RiBtnEast" and not self._mads_arm_from_east:
          self._set_mads_arm(True)
        elif sig == "MFS_RiBtnNorth" and self._mads_arm_from_east:
          self._set_mads_arm(False)
      elif not pressed_now and pressed_prev:
        button_events.append(structs.CarState.ButtonEvent(pressed=False, type=btype))
      self._prev_button_state[sig] = cur
    ret.buttonEvents = button_events

    # Reset MADS arming when ACC transitions to Off (state 0). Guarantees each cruise
    # session begins in the safer "ACC-only until the user explicitly presses East"
    # state. Watching the edge (prev != 0 → cur == 0) rather than steady-state so we
    # don't re-write the param every tick while ACC is off.
    if cc_state == 0 and self._prev_cc_state != 0 and self._mads_arm_from_east:
      self._set_mads_arm(False)
    self._prev_cc_state = cc_state

    # ---- Faults ----
    # ADAS_Sts_ACC_ICC 9/10 = Failure_reversible/irreversible (same enum position as
    # the old VCU_Sts_CC_ICC path — the fault codes carry the same meaning across
    # both authors).
    ret.accFaulted = cc_state in (9, 10) or bool(cp_pt.vl["ESP_0x114"]["ESP_FltIndcn_AEB"])

    # ---- OEM ADAS lateral counters (bus 2, for takeover alignment) --------
    # Snapshot the OEM ADAS's own AliveCounter and SSecOC_Fresh_Byte0 from bus 2. The
    # EPS validates our re-engage frames against its last-accepted counter+1 rule — if
    # we transmit anything else, EPS latches an LKA fault (surfaces as "ADAS error" on
    # cluster and cruise fault in openpilot). Carcontroller reads these values and seeds
    # our tx counters at every engagement transition so hand-off is seamless.
    oem_1d0 = cp_cam.vl["ADAS_0x1D0"]
    oem_1c0 = cp_cam.vl["ADAS_0x1C0"]
    self.oem_1d0_alive = int(oem_1d0["ADAS_1D0_AliveCounter"])
    self.oem_1c0_alive = int(oem_1c0["ADAS_1C0_AliveCounter"])
    self.oem_1d0_secoc_wire_ctr = (int(oem_1d0["ADAS_1D0_SSecOC_Fresh_Byte0"]) >> 2) & 0x3F

    # ---- ICC settings frame (0x52A) — snapshot for the spoof --------------
    # ICC broadcasts 0x52A on bus 0 at ~10 Hz. We forward every field into a
    # dict; the carcontroller re-packs it (with ICCACCFuncTyp forced to 2) and
    # emits it on bus 2. Include the reserved-bit signals so a firmware that
    # sets bits 49/55 to something non-zero still round-trips faithfully.
    icc52a = cp_pt.vl["ICC_0x52A"]
    self.icc_52a_values = {k: icc52a[k] for k in ICC_0x52A_SIGNALS}
    self.icc_52a_alive = int(icc52a["ICC_0x52AAliveCounter"])
    self.icc_52a_seen = True

    # ---- SecOC sync (GW_Syn_All 0x20) ----
    # Trip counter = 2 bytes BE, Reset counter = 3 bytes BE, MAC = 3 bytes.
    sync = cp_pt.vl["GW_Syn_All"]
    self.secoc_trip = (int(sync["Syn_TripCntrVal_Byte0_All"]) << 8) | int(sync["Syn_TripCntrVal_Byte1_All"])
    self.secoc_reset = ((int(sync["Syn_RstCntrVal_Byte0_All"]) << 16)
                        | (int(sync["Syn_RstCntrVal_Byte1_All"]) << 8)
                        | int(sync["Syn_RstCntrVal_Byte2_All"]))
    self.secoc_sync_mac = bytes([
      int(sync["Syn_MACInfo_Byte0_All"]),
      int(sync["Syn_MACInfo_Byte1_All"]),
      int(sync["Syn_MACInfo_Byte2_All"]),
    ])
    self.secoc_sync_seen = True

    return ret, ret_sp

  @staticmethod
  def get_can_parsers(CP, CP_SP):
    # All signals — including the gateway-mirrored body/HMI messages — are read
    # from ADASBUS (Bus.pt). No IBUS1 tap required.
    pt_msgs = [
      # ESP / IMU / EPS / ADAS (native ADASBUS)
      ("ESP_0x115", 100),
      ("ESP_0x116", 100),
      ("ESP_0x318", 50),
      ("ESP_0x114", 50),
      ("YRS_0x112", 100),
      ("YRS_0x113", 100),
      ("EPS_0x1C2", 50),
      ("EPS_0x1C4", 50),
      # ADAS module frames originate on bus 2 (cam-side of the splice). Cruise is
      # replaced (openpilot supplies its own via VCU basic CC below); BSM is read from
      # the cam-side parser below because pandad reports the packet's original src bus
      # (2) even after panda forwards it to bus 0 for the cluster.
      # Freqs are the real on-vehicle rates; over-declaring makes the CANParser flag a
      # message stale -> carState.canValid=False -> commIssue. Measured on ADASBUS.
      ("ICC_0x531", 10),
      ("ICC_0x52A", 10),    # ICC settings frame — we spoof this on bus 2 with ICCACCFuncTyp=2
      ("GW_Syn_All", 2),    # SecOC sync (~3 Hz)
      # Gateway-mirrored body/HMI (also present on ADASBUS)
      ("VCU_0x214", 50),    # gear, accel pedal %, brake
      ("BCM_0x343", 20),    # doors
      ("BCM_0x335", 20),    # blinkers
      ("ACU_0x159", 20),    # driver seatbelt
      ("MFS_0x514", 20),    # MFSS buttons (~20 Hz)
    ]
    # ADAS-authored messages live on the cam-side bus (bus 2). Panda forwards them
    # onto bus 0 for the cluster, but pandad tags each packet with the src bus it was
    # originally received on — so a CANParser subscribed to bus 0 doesn't see them.
    cam_msgs = [
      ("ADAS_0x313", 50),   # ADAS ACC state (ADAS_Sts_ACC_ICC) — authoritative cruise state
      ("ADAS_0x314", 50),   # BSDSts + LKA/ELKA state enums
      ("ADAS_0x315", 20),   # BSD_CID_{Le,Ri}DispReq — blind-spot alert
      ("ADAS_0x31C", 20),   # ACC HUD — ADAS_AccTrgSpdDisp + ADAS_DispSpdUnit_ACC (set speed)
      # OEM ADAS's lateral commands on bus 2 — we snapshot the AliveCounters and SecOC
      # wire freshness byte so carcontroller can align its transmitted counters to what
      # the EPS was tracking BEFORE the panda takeover blocked OEM's stream. Rejection on
      # the very first re-engage frame is what surfaces as "ADAS error" on the cluster.
      ("ADAS_0x1C0", 100),  # ADAS_1C0_AliveCounter
      ("ADAS_0x1D0", 100),  # ADAS_1D0_AliveCounter + ADAS_1D0_SSecOC_Fresh_Byte0
    ]
    return {
      Bus.pt: CANParser(DBC[CP.carFingerprint][Bus.pt], pt_msgs, CANBUS.pt),
      Bus.cam: CANParser(DBC[CP.carFingerprint][Bus.pt], cam_msgs, CANBUS.cam),
    }
