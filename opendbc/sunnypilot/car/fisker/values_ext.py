"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from enum import IntFlag


class FiskerFlagsSP(IntFlag):
  # Stored as "disable" so the default (no flags set, no param present) matches
  # the baseline on-vehicle behaviour: ACC auto-speed ON. The sunnypilot UI
  # toggles the `FiskerACCAutoSpeed` param, which is 1 by default (feature on);
  # we only set this flag when the user explicitly turns the toggle OFF.
  ACC_AUTO_SPEED_OFF = 1
