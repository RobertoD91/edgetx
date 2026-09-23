/*
 * Copyright (C) EdgeTX
 *
 * Based on code named
 *   opentx - https://github.com/opentx/opentx
 *   th9x - http://code.google.com/p/th9x
 *   er9x - http://code.google.com/p/er9x
 *   gruvin9x - http://code.google.com/p/gruvin9x
 *
 * License GPLv2: http://www.gnu.org/licenses/gpl-2.0.html
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

// Turns raw HID input reports into a normalised joystick state using the
// parsed report descriptor. Works with any joystick/gamepad that follows the
// HID usage tables.
//
// Ported from esp32-joystick2trainer (lib/hid_core, MIT licensed), with
// integer axis values in EdgeTX RESX units (-1024..+1024).

#pragma once

#include <stddef.h>
#include <stdint.h>

#include "hid_report_descriptor.h"

namespace hid {

// Logical axes recognised by the decoder. The HID layer fills them from the
// Generic Desktop / Simulation Controls usages.
enum class Axis : uint8_t {
  X = 0,
  Y,
  Z,
  Rx,
  Ry,
  Rz,
  Slider,
  Dial,
  Wheel,
  Throttle,
  Rudder,
  Accelerator,
  Brake,
  Count
};

constexpr size_t kAxisCount = (size_t)Axis::Count;
constexpr size_t kMaxButtons = 32;

// Full scale of a decoded axis (EdgeTX RESX).
constexpr int16_t kAxisFullScale = 1024;

// Short lower-case axis names ("x", "y", "rz", "slider", ...).
const char* axisName(Axis axis);

struct InputState {
  // Axis positions in [-kAxisFullScale, +kAxisFullScale]. 0 is centre.
  int16_t axis[kAxisCount];
  // Bit i set when axis i is reported by the device (others are always 0).
  uint16_t axis_present;
  // Button bitmask, bit 0 = button 1.
  uint32_t buttons;
  // Number of buttons reported by the device.
  uint8_t button_count;
  // Hat switch direction: -1 = released/centre, 0 = up, 1 = up-right, ...
  // 7 = up-left.
  int8_t hat;
  bool hat_present;

  void clear();

  bool axisPresent(Axis a) const { return (axis_present >> (uint8_t)a) & 1; }
  bool button(size_t index) const
  {
    return index < kMaxButtons && ((buttons >> index) & 1u) != 0;
  }

  // Hat decomposed into a horizontal component: -1 (left), 0, +1 (right).
  int8_t hatX() const;
  // Hat decomposed into a vertical component: -1 (down), 0, +1 (up).
  int8_t hatY() const;
};

class JoystickDecoder
{
 public:
  JoystickDecoder() { reset(); }

  // Build the decode tables from a parsed descriptor. Returns false when the
  // descriptor contains nothing a joystick mapper can use (no axes, buttons
  // or hat).
  bool configure(const ParsedDescriptor& desc);

  // Decode a raw input report (as received from the interrupt IN endpoint,
  // including the report ID byte when the device uses report IDs). Fields
  // belonging to other report IDs are left untouched in `state`. Returns
  // false when the report is not an input report we know about.
  bool decode(const uint8_t* report, size_t len, InputState& state) const;

  struct AxisBinding {
    bool present = false;
    InputField field;
  };
  const AxisBinding& axisBinding(Axis axis) const
  {
    return axes_[(size_t)axis];
  }
  const AxisBinding& hatBinding() const { return hat_; }
  size_t buttonCount() const { return button_count_; }
  size_t axisCount() const;
  bool usesReportIds() const { return uses_report_ids_; }

  // Extract an unsigned/signed field from a payload. Public for testing.
  static int32_t extract(const uint8_t* payload, size_t payload_len,
                         const InputField& f, size_t element, bool& ok);

 private:
  void reset();

  struct ButtonBinding {
    InputField field;
    uint8_t button_index = 0;  // 0-based index into InputState::buttons
  };

  AxisBinding axes_[kAxisCount];
  AxisBinding hat_;
  ButtonBinding buttons_[kMaxButtons];
  size_t button_binding_count_;
  InputField button_arrays_[8];
  size_t button_array_count_;
  size_t button_count_;
  bool uses_report_ids_;
};

// Map a (usage page, usage) pair to a logical axis. Returns false when it is
// not an axis.
bool axisFromUsage(uint16_t usage_page, uint16_t usage, Axis& out);

}  // namespace hid
