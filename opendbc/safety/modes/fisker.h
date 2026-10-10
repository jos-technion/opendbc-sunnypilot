#pragma once

#include "opendbc/safety/declarations.h"

// Fisker Ocean — angle steering control, AUTOSAR "Short SecOC" on the
// actuator commands. Panda enforces signal-level bounds and rate limits; it does
// NOT validate the SecOC MAC (the car gateway does that, and the key lives in
// userspace). Panda validates counters + frequency + actuator limits.
//
// Buses: ADASBUS = bus 0.
//   TX  0x1D0  ADAS_LatCtrl_SteerAnReq    steering angle (SecOC)
//   TX  0x1C0  ADAS_LatCtrl activation    plain E2E, lateral activation/status
//   TX  0x121  ADAS_LgtCtrl_AccelReq      accel (SecOC, longitudinal only)
//   TX  0x117  ADAS long control status   plain E2E, Sts/Typ (CC -> ACC transition)
//   TX  0x118  ADAS long/ESP handshake    plain E2E, ESP-side mirror + jerk/prefill/AEB
//   TX  0x52A  ICC settings spoof         bus 2 (to ADAS); plain E2E — see below
//   TX  0x1C2  EPS state spoof            bus 2 (to ADAS); plain E2E — see below
//   RX  0x115  wheel speeds -> vehicle_moving
//   RX  0x318  vehicle speed + brake pedal
//   RX  0x1C2  EPS steering angle
//   RX  0x1C4  EPS driver torque
//   RX  0x313  ADAS ACC state (cruise engaged) — cam-side (bus 2)

static bool fisker_longitudinal = false;

// ICC_0x52A (ICC feature settings, 200 ms): openpilot reads the ICC's frame on bus 0 and
// re-sends it to the ADAS module on bus 2 with byte 4 overridden to enable ACC, keeping
// the ICC's own AliveCounter. The ICC's original is blocked bus 0 -> bus 2 only while
// openpilot's copies are flowing; if openpilot stops (crash, disabled), forwarding
// resumes so ADAS never actually loses 0x52A — just the "ACC enabled" overrides.
// 500 ms = 2.5 missed 200 ms cycles.
#define FISKER_ICC_RELAY_TIMEOUT_US 500000U
static uint32_t fisker_icc_settings_tx_last = 0U;

static bool fisker_icc_relay_active(void) {
  return safety_get_ts_elapsed(microsecond_timer_get(), fisker_icc_settings_tx_last) < FISKER_ICC_RELAY_TIMEOUT_US;
}

// EPS_0x1C2 spoof relay (50 ms): carcontroller re-emits EPS_0x1C2 on bus 2 with
// AdasLatCtrlSts forced to Available during MADS engagement + the 500 ms fade. Panda
// blocks the real EPS_0x1C2 from forwarding bus 0 → bus 2 only while openpilot has TX'd
// its spoofed copy within the last FISKER_EPS_RELAY_TIMEOUT_US. 50 ms is 2.5 missed
// 20 ms cycles — generous enough to tolerate carcontroller jitter without the OEM
// ADAS seeing a bare EPS frame leak through.
#define FISKER_EPS_RELAY_TIMEOUT_US 50000U
static uint32_t fisker_eps_1c2_tx_last = 0U;

static bool fisker_eps_1c2_relay_active(void) {
  return safety_get_ts_elapsed(microsecond_timer_get(), fisker_eps_1c2_tx_last) < FISKER_EPS_RELAY_TIMEOUT_US;
}

// Lateral TX relay (500 ms): carcontroller TXes 0x1D0/0x1C0 during MADS engagement AND
// during the 500 ms post-disengage fade (Req=0 content), so the EPS transitions
// Active→Available on OUR terms rather than from a surprise OEM command injection.
// Panda uses this timestamp to block OEM 0x1D0/0x1C0 forwarding bus 2 → bus 0 for the
// same window, keeping the EPS fed from exactly one source. Replaces the older
// controls_allowed_lateral gate so fade is observed on both openpilot and panda sides.
#define FISKER_LAT_RELAY_TIMEOUT_US 500000U
static uint32_t fisker_lat_tx_last = 0U;

static bool fisker_lat_relay_active(void) {
  return safety_get_ts_elapsed(microsecond_timer_get(), fisker_lat_tx_last) < FISKER_LAT_RELAY_TIMEOUT_US;
}

// Steering: raw CAN at 0.0625 deg/LSB, offset -780 deg. Raw 12480 == 0 deg.
#define FISKER_ANGLE_ZERO_CAN 12480

// Accel: raw CAN at 0.0004882 m/s^2/LSB, offset -16 m/s^2. raw = (accel + 16) / 0.0004882
#define FISKER_ACCEL_INACTIVE 32773  //  0.0 m/s^2
#define FISKER_ACCEL_MAX      36870  // +2.0 m/s^2
#define FISKER_ACCEL_MIN      25604  // -3.5 m/s^2

// Most Ocean messages carry a 4-bit AliveCounter in byte 1 bits 8..11.
static uint8_t fisker_get_counter(const CANPacket_t *msg) {
  return (msg->data[1] >> 4) & 0x0FU;
}


static void fisker_rx_hook(const CANPacket_t *msg) {
  if (msg->bus == 0U) {
    // EPS_0x1C2: measured steering wheel angle (BE 16-bit @ start 23 -> bytes 2,3).
    if (msg->addr == 0x1C2U) {
      int raw = (msg->data[2] << 8) | msg->data[3];
      update_sample(&angle_meas, raw - FISKER_ANGLE_ZERO_CAN);
    }

    // EPS_0x1C4: driver steering torque magnitude (BE 12-bit @ start 23 -> bytes 2, hi-nibble 3).
    if (msg->addr == 0x1C4U) {
      int torque = (msg->data[2] << 4) | (msg->data[3] >> 4);
      update_sample(&torque_driver, torque);
    }

    // ESP_0x318: vehicle speed (BE 16-bit @ start 47 -> bytes 5,6, 0.1 km/h) + brake pedal.
    if (msg->addr == 0x318U) {
      float speed = ((msg->data[5] << 8) | msg->data[6]) * 0.1f * KPH_TO_MS;
      UPDATE_VEHICLE_SPEED(speed);
      brake_pressed = ((msg->data[1] >> 6) & 0x1U) != 0U;  // ESP_BrkPedlSts @ bit 14
    }

    // ESP_0x115: front-right wheel speed (BE 14-bit @ start 21) -> vehicle_moving.
    if (msg->addr == 0x115U) {
      int whl_rf = ((msg->data[1] & 0x3FU) << 8) | msg->data[2];
      vehicle_moving = whl_rf > 0;
    }

    // MFS_0x514: MFSS steering-wheel buttons. MFS_RiBtnSouth (2-bit @ start 33, big-endian
    // -> byte 4 bits 0,1) is sunnypilot's MADS engage/disengage toggle. Values are
    // 0=No_Pressed, 1=Pressed(short), 2=Long_Press, 3=Reserved. We ONLY treat the long
    // press as the MADS trigger — short press is consumed by the car for the ACC
    // follow-distance adjustment, so firing MADS on it would clash. Openpilot's carstate
    // uses the same long-press-only predicate for the ButtonType.lkas edge, keeping the
    // two sides in lockstep.
    if (msg->addr == 0x514U) {
      int rbs = msg->data[4] & 0x03U;
      mads_button_press = (rbs == 2) ? MADS_BUTTON_PRESSED : MADS_BUTTON_NOT_PRESSED;
    }
  }

  // ADAS_0x313 lives on bus 2 (cam side). We read cruise-engaged state from the
  // ADAS module itself instead of the VCU basic-CC path — now that the ICC_0x52A
  // spoof forces ICCACCFuncTyp=2, ADAS enters full ACC mode and is the authoritative
  // source. ADAS_Sts_ACC_ICC is a 4-bit signal @ start bit 35 (big-endian) → the
  // low nibble of data[4] (byte 4 covers bits 32..39 in Motorola numbering; a 4-bit
  // field with MSB at 35 spans bits 35..32).
  //
  // ADAS_Sts_ACC_ICC enum: 0=ACC_Off, 1=Init, 2=Standby, 3=Active, 4=Override,
  //   5=Standstill_active, 6=Standstill_wait, 7=Deactivation_brake,
  //   8=Deactivation_other, 9=Failure_reversible, 10=Failure_irreversible,
  //   11=Standstill_GoNotification.
  // Treat every state where the ACC controller is commanding the car as engaged:
  // Active(3), Override(4), and the three Standstill_* holds (5, 6, 11).
  //
  // acc_main_on is intentionally NOT set from cc_state on fisker (see the deleted
  // VCU_0x358 handler for the full 2-step-ACC rationale — the same argument applies
  // here: RiBtnNorth → cc_state 0→2 must not falling-edge MADS, and brake → 3→2
  // must not disengage MADS while steering_mode_on_brake=Remain Active).
  if (msg->bus == 2U) {
    if (msg->addr == 0x313U) {
      int cc_state = msg->data[4] & 0x0FU;
      bool cruise_engaged = (cc_state == 3) || (cc_state == 4)
                         || (cc_state == 5) || (cc_state == 6) || (cc_state == 11);
      pcm_cruise_check(cruise_engaged);
    }
  }
}


// Steering angle limits (mirror CarControllerParams.ANGLE_LIMITS in values.py). The
// upstream AngleSteeringLimits struct in this openpilot release is minimal — the
// tizi-era fields (max_angle_error, angle_error_min_speed, angle_is_curvature,
// enforce_angle_error, inactive_angle_is_zero) were dropped from the C struct when
// angle-error enforcement moved into the VM-based limits path. Fisker's carcontroller
// still uses the non-VM apply_std_steer_angle_limits, so the 5 fields below are all
// that apply.
static const AngleSteeringLimits FISKER_STEERING_LIMITS = {
  .max_angle = 9600,             // 600 deg * 16 CAN/deg
  .angle_deg_to_can = 16.0f,     // 1 / 0.0625
  .angle_rate_up_lookup = {
    {0., 5., 25.},
    {2.5, 1.5, 0.2}
  },
  .angle_rate_down_lookup = {
    {0., 5., 25.},
    {5.0, 2.0, 0.3}
  },
};

static const LongitudinalLimits FISKER_LONG_LIMITS = {
  .max_accel = FISKER_ACCEL_MAX,
  .min_accel = FISKER_ACCEL_MIN,
  .inactive_accel = FISKER_ACCEL_INACTIVE,
};


static bool fisker_tx_hook(const CANPacket_t *msg) {
  bool tx = true;

  // Steering angle command (0x1D0). 0x1D0 carries no explicit enable bit, so the
  // steer-active state is derived from (controls_allowed || controls_allowed_lateral) —
  // openpilot only transmits this frame when it intends to steer, and the MADS gate
  // opens the lateral path even without cruise engaged.
  if (msg->addr == 0x1D0U) {
    int raw_angle = (msg->data[2] << 8) | msg->data[3];
    int desired_angle = raw_angle - FISKER_ANGLE_ZERO_CAN;

    if (steer_angle_cmd_checks(desired_angle, controls_allowed || controls_allowed_lateral, FISKER_STEERING_LIMITS)) {
      tx = false;
    }
  }

  // Lateral-control activation (0x1C0). Angle actuation is gated by 0x1D0's checks; also
  // only permit declaring the lateral request ACTIVE while lateral is authorized (either
  // by cruise or by MADS).
  // ADAS_LatCtrl_Req @ 31|2 -> data[3] bits 7..6 (0=Not_active, 1=Angle_request_active).
  if (msg->addr == 0x1C0U) {
    int lat_req = (msg->data[3] >> 6) & 0x03U;
    if ((lat_req != 0) && !(controls_allowed || controls_allowed_lateral)) {
      tx = false;
    }
  }

  // Accel command (0x121)
  if (msg->addr == 0x121U) {
    int raw_accel = (msg->data[2] << 8) | msg->data[3];

    if (fisker_longitudinal) {
      if (longitudinal_accel_checks(raw_accel, FISKER_LONG_LIMITS)) {
        tx = false;
      }
    } else {
      // Not controlling longitudinal: only the inactive value may be sent.
      if (raw_accel != FISKER_ACCEL_INACTIVE) {
        tx = false;
      }
    }
  }

  // ICC settings spoof (0x52A, bus 2): record the TX timestamp so the fwd_hook knows
  // whether the ICC's own 0x52A should still be blocked bus 0 -> bus 2.
  if ((msg->addr == 0x52AU) && (msg->bus == 2U)) {
    fisker_icc_settings_tx_last = microsecond_timer_get();
  }
  // EPS state spoof (0x1C2, bus 2): similar timestamp relay so fwd_hook knows whether
  // to keep blocking the real EPS_0x1C2 from forwarding bus 0 → bus 2.
  if ((msg->addr == 0x1C2U) && (msg->bus == 2U)) {
    fisker_eps_1c2_tx_last = microsecond_timer_get();
  }
  // Lateral stream (0x1D0, 0x1C0, bus 0): record the TX timestamp so the fwd_hook can
  // keep blocking OEM 0x1D0/0x1C0 bus 2 → bus 0 across the full engaged window AND
  // the 500 ms post-disengage fade.
  if (((msg->addr == 0x1D0U) || (msg->addr == 0x1C0U)) && (msg->bus == 0U)) {
    fisker_lat_tx_last = microsecond_timer_get();
  }

  return tx;
}


static safety_config fisker_init(uint16_t param) {
  // Forwarding intercept: the OEM ADAS module stays alive on bus 2 and its status/HUD
  // frames are forwarded to the car, so openpilot only injects the command it replaces.
  // disable_static_blocking lets fisker_fwd_hook forward the OEM's 0x1D0 when disengaged
  // and block it only while openpilot steers; check_relay still guards against the OEM's
  // steering leaking onto bus 0.
  // 0x52A + 0x1C2 go to the ADAS module on bus 2. No check_relay: a relay malfunction
  // stops ALL forwarding, and bus 2 presence isn't reliable evidence of one —
  // fisker_fwd_hook blocks the OEM originals via time-based relays instead.
  static const CanMsg FISKER_TX_MSGS[] = {
    {0x1D0, 0, 8, .check_relay = true, .disable_static_blocking = true},    // steering angle
    {0x1C0, 0, 8, .check_relay = true, .disable_static_blocking = true},    // lateral activation
    {0x52A, 2, 8, .check_relay = false},                                    // ICC settings spoof (bus 2)
    {0x1C2, 2, 8, .check_relay = false},                                    // EPS state spoof (bus 2)
  };
  static const CanMsg FISKER_LONG_TX_MSGS[] = {
    {0x1D0, 0, 8, .check_relay = true, .disable_static_blocking = true},    // steering angle
    {0x1C0, 0, 8, .check_relay = true, .disable_static_blocking = true},    // lateral activation
    {0x121, 0, 8, .check_relay = true, .disable_static_blocking = true},    // accel (op long only)
    {0x117, 0, 8, .check_relay = true, .disable_static_blocking = true},    // long control status
    {0x118, 0, 8, .check_relay = true, .disable_static_blocking = true},    // long/ESP handshake
    {0x52A, 2, 8, .check_relay = false},                                    // ICC settings spoof (bus 2)
    {0x1C2, 2, 8, .check_relay = false},                                    // EPS state spoof (bus 2)
  };

  static RxCheck fisker_rx_checks[] = {
    {.msg = {{0x115, 0, 8, 100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},  // wheel speeds
    {.msg = {{0x318, 0, 8,  50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},  // speed + brake
    {.msg = {{0x1C2, 0, 8,  50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},  // steering angle
    {.msg = {{0x1C4, 0, 8,  50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},  // driver torque
    // ADAS_0x313 on bus 2 (cam side): ADAS ACC state (ADAS_Sts_ACC_ICC). Replaces
    // the previous VCU_0x358 read now that we spoof ICC_0x52A into ADAS ACC mode.
    // The safety framework gates rx_hook execution on rx_checks membership, so this
    // must be present for our pcm_cruise_check to actually fire on 0x313.
    {.msg = {{0x313, 2, 8,  50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},  // ADAS ACC state
    // MFS_0x514 (MFSS buttons) whitelisted so our fisker_rx_hook actually runs on it —
    // the safety framework gates rx_hook execution on rx_checks membership. Without this,
    // our MFS_RiBtnSouth read in the rx hook was dead code and mads_button_press never
    // toggled, so panda's controls_allowed_lateral never opened on MADS press.
    {.msg = {{0x514, 0, 8,  20U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},  // MFSS buttons (MADS trigger)
  };

  // Fisker on-vehicle bring-up: honor the LONG_CONTROL bit on release too. Stock openpilot
  // hides longitudinal behind ALLOW_DEBUG for release builds, but this is a personal test
  // fork and we need the flag to work so alpha_long can gate 0x121/0x117/0x118 in this
  // firmware (release-tizi panda is not built with ALLOW_DEBUG). openpilot still gates
  // openpilotLongitudinalControl behind AlphaLongitudinalEnabled, so this is only reachable
  // by an explicit developer opt-in.
  const uint16_t FISKER_FLAG_LONGITUDINAL_CONTROL = 1;
  fisker_longitudinal = GET_FLAG(param, FISKER_FLAG_LONGITUDINAL_CONTROL);

  // Initialise the ICC + EPS relay trackers. Seeding to "now" opens their windows
  // immediately so the OEM's first bus-0 frames can't race openpilot's spoofs —
  // openpilot is already pushing both by the time safety mode starts.
  fisker_icc_settings_tx_last = microsecond_timer_get();
  fisker_eps_1c2_tx_last = microsecond_timer_get();
  // Lateral relay stays idle at init — carcontroller only starts TXing lateral
  // when MADS engages, and we want OEM 0x1D0/0x1C0 to flow through until then.
  fisker_lat_tx_last = 0U;

  // cppcheck-suppress knownConditionTrueFalse
  return fisker_longitudinal ? BUILD_SAFETY_CFG(fisker_rx_checks, FISKER_LONG_TX_MSGS)
                             : BUILD_SAFETY_CFG(fisker_rx_checks, FISKER_TX_MSGS);
}

static bool fisker_fwd_hook(int bus_num, int addr) {
  // Forwarding intercept: keep the OEM ADAS module (bus 2) alive by forwarding all
  // traffic between it and the vehicle (bus 0), so the car stays fault-free. Only steal
  // the steering command — block the OEM's 0x1D0 (bus 2 -> vehicle) while openpilot is
  // actively steering; openpilot injects its own on bus 0. When disengaged the OEM's
  // 0x1D0 is forwarded so the EPS keeps receiving a steering frame.
  bool block_msg = false;
  if (bus_num == 2) {
    // Steering angle (0x1D0) + lateral activation (0x1C0): time-based relay. OEM's
    // bus-2 → bus-0 stream is blocked while openpilot has TX'd its own in the last
    // 500 ms. That window covers both the live MADS engagement AND the post-disengage
    // fade (carcontroller keeps TXing Req=0 for FADE_TICKS so the EPS transitions
    // Active→Available on our terms, not from an OEM command surprise). If openpilot
    // crashes mid-engagement, the window naturally expires and OEM resumes.
    if (((addr == 0x1D0) || (addr == 0x1C0)) && fisker_lat_relay_active()) {
      block_msg = true;
    }
    // Longitudinal (0x121) is unaffected by MADS — MADS is lateral-only. Only block the
    // OEM's accel when cruise is engaged and openpilot is driving longitudinal.
    if (fisker_longitudinal && (addr == 0x121) && controls_allowed) {
      block_msg = true;
    }
  }
  if (bus_num == 0) {
    // ICC settings spoof (0x52A, ICC → ADAS/FCM): block the real ICC_0x52A from
    // reaching bus 2 while openpilot is TXing its overridden copy. The spoof is the
    // ENABLE for ACC — ADAS stays at ACC_Off until it sees our overridden byte 4 —
    // so blocking must NOT be gated on controls_allowed (chicken-and-egg: cruise
    // can't enable because the spoof is gated on cruise). Time-based: block only
    // while openpilot has recently sent one of its own copies. If openpilot stops,
    // the ICC's original resumes flowing to ADAS cleanly.
    if ((addr == 0x52A) && fisker_icc_relay_active()) {
      block_msg = true;
    }
    // EPS state spoof (0x1C2, EPS → ADAS): block the real EPS_0x1C2 from reaching
    // bus 2 while openpilot is TXing its spoofed copy. The spoof masks
    // EPS_AdasLatCtrlSts=Active (which would reveal that another controller is
    // driving the EPS) as Available, keeping the OEM ADAS module from raising its
    // "LKA not available" alert on the state mismatch between its own
    // ADAS_LatCtrl_Req=0 and the real EPS=Active. Time-based relay naturally covers
    // the engagement window AND the fade.
    if ((addr == 0x1C2) && fisker_eps_1c2_relay_active()) {
      block_msg = true;
    }
  }
  return block_msg;
}

const safety_hooks fisker_hooks = {
  .init = fisker_init,
  .rx = fisker_rx_hook,
  .tx = fisker_tx_hook,
  .fwd = fisker_fwd_hook,
  .get_counter = fisker_get_counter,
};
