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
//   RX  0x115  wheel speeds -> vehicle_moving
//   RX  0x318  vehicle speed + brake pedal
//   RX  0x1C2  EPS steering angle
//   RX  0x1C4  EPS driver torque
//   RX  0x358  VCU basic-CC state (cruise engaged / main on)

static bool fisker_longitudinal = false;

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

    // VCU_0x358: basic cruise-control state (VCU_Sts_CC_ICC, 4-bit @ start 35 -> data[4]
    // low nibble). openpilot replaces the ADAS module (isolated on the intercept side, so
    // its ADAS_0x313 ACC state isn't natively on this bus), and the car uses the VCU's
    // basic cruise, which is gateway-sourced and native to bus 0. Enum: 3=Active,
    // 4=Override; 9/10=Fault.
    if (msg->addr == 0x358U) {
      int cc_state = msg->data[4] & 0x0FU;
      bool cruise_engaged = (cc_state == 3) || (cc_state == 4);
      // main on ("available"): on and not off/init/fault
      acc_main_on = (cc_state != 0) && (cc_state != 1) && (cc_state < 9);
      pcm_cruise_check(cruise_engaged);
    }

    // MFS_0x514: MFSS steering-wheel buttons. MFS_RiBtnSouth (2-bit @ start 33, big-endian
    // -> byte 4 bits 0,1) is sunnypilot's MADS engage/disengage toggle. Any non-zero state
    // counts as pressed (1=short_press, 2=long_press, 3=reserved). Openpilot's carstate
    // emits a matching ButtonType.lkas edge on the same signal so the MADS state machine
    // observes both sides. MADS is what enables lateral without cruise engaged.
    if (msg->addr == 0x514U) {
      int rbs = msg->data[4] & 0x03U;
      mads_button_press = (rbs != 0) ? MADS_BUTTON_PRESSED : MADS_BUTTON_NOT_PRESSED;
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

  return tx;
}


static safety_config fisker_init(uint16_t param) {
  // Forwarding intercept: the OEM ADAS module stays alive on bus 2 and its status/HUD
  // frames are forwarded to the car, so openpilot only injects the command it replaces.
  // disable_static_blocking lets fisker_fwd_hook forward the OEM's 0x1D0 when disengaged
  // and block it only while openpilot steers; check_relay still guards against the OEM's
  // steering leaking onto bus 0.
  static const CanMsg FISKER_TX_MSGS[] = {
    {0x1D0, 0, 8, .check_relay = true, .disable_static_blocking = true},    // steering angle
    {0x1C0, 0, 8, .check_relay = true, .disable_static_blocking = true},    // lateral activation
  };
  static const CanMsg FISKER_LONG_TX_MSGS[] = {
    {0x1D0, 0, 8, .check_relay = true, .disable_static_blocking = true},    // steering angle
    {0x1C0, 0, 8, .check_relay = true, .disable_static_blocking = true},    // lateral activation
    {0x121, 0, 8, .check_relay = true, .disable_static_blocking = true},    // accel (op long only)
    {0x117, 0, 8, .check_relay = true, .disable_static_blocking = true},    // long control status
    {0x118, 0, 8, .check_relay = true, .disable_static_blocking = true},    // long/ESP handshake
  };

  static RxCheck fisker_rx_checks[] = {
    {.msg = {{0x115, 0, 8, 100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},  // wheel speeds
    {.msg = {{0x318, 0, 8,  50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},  // speed + brake
    {.msg = {{0x1C2, 0, 8,  50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},  // steering angle
    {.msg = {{0x1C4, 0, 8,  50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},  // driver torque
    {.msg = {{0x358, 0, 8,  10U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},  // VCU cruise-control state
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
    // steering angle (0x1D0) + lateral activation (0x1C0): openpilot replaces both while
    // engaged (cruise or MADS-only), so block the OEM's from reaching the vehicle;
    // forward them when disengaged.
    if (((addr == 0x1D0) || (addr == 0x1C0)) && (controls_allowed || controls_allowed_lateral)) {
      block_msg = true;
    }
    // Longitudinal (0x121) is unaffected by MADS — MADS is lateral-only. Only block the
    // OEM's accel when cruise is engaged and openpilot is driving longitudinal.
    if (fisker_longitudinal && (addr == 0x121) && controls_allowed) {
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
