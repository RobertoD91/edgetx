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

// Ported from esp32-joystick2trainer (lib/hid_core, MIT licensed).

#include "hid_joystick_decoder.h"

#include <string.h>

namespace hid {

namespace {
const char* const kAxisNames[kAxisCount] = {
    "x",    "y",     "z",        "rx",     "ry",          "rz",   "slider",
    "dial", "wheel", "throttle", "rudder", "accelerator", "brake"};
}

const char* axisName(Axis axis)
{
  size_t idx = (size_t)axis;
  return idx < kAxisCount ? kAxisNames[idx] : "?";
}

void InputState::clear()
{
  memset(this, 0, sizeof(*this));
  hat = -1;
}

int8_t InputState::hatX() const
{
  switch (hat) {
    case 1:
    case 2:
    case 3:
      return 1;
    case 5:
    case 6:
    case 7:
      return -1;
    default:
      return 0;
  }
}

int8_t InputState::hatY() const
{
  switch (hat) {
    case 7:
    case 0:
    case 1:
      return 1;
    case 3:
    case 4:
    case 5:
      return -1;
    default:
      return 0;
  }
}

bool axisFromUsage(uint16_t usage_page, uint16_t usage, Axis& out)
{
  if (usage_page == kPageGenericDesktop) {
    switch (usage) {
      case kUsageX:
        out = Axis::X;
        return true;
      case kUsageY:
        out = Axis::Y;
        return true;
      case kUsageZ:
        out = Axis::Z;
        return true;
      case kUsageRx:
        out = Axis::Rx;
        return true;
      case kUsageRy:
        out = Axis::Ry;
        return true;
      case kUsageRz:
        out = Axis::Rz;
        return true;
      case kUsageSlider:
        out = Axis::Slider;
        return true;
      case kUsageDial:
        out = Axis::Dial;
        return true;
      case kUsageWheel:
        out = Axis::Wheel;
        return true;
      default:
        return false;
    }
  }
  if (usage_page == kPageSimulation) {
    switch (usage) {
      case kUsageSimThrottle:
        out = Axis::Throttle;
        return true;
      case kUsageSimRudder:
        out = Axis::Rudder;
        return true;
      case kUsageSimAccelerator:
        out = Axis::Accelerator;
        return true;
      case kUsageSimBrake:
        out = Axis::Brake;
        return true;
      default:
        return false;
    }
  }
  return false;
}

void JoystickDecoder::reset()
{
  for (size_t i = 0; i < kAxisCount; ++i) axes_[i] = AxisBinding();
  hat_ = AxisBinding();
  for (size_t i = 0; i < kMaxButtons; ++i) buttons_[i] = ButtonBinding();
  for (size_t i = 0; i < 8; ++i) button_arrays_[i] = InputField();
  button_binding_count_ = 0;
  button_array_count_ = 0;
  button_count_ = 0;
  uses_report_ids_ = false;
}

size_t JoystickDecoder::axisCount() const
{
  size_t n = 0;
  for (size_t i = 0; i < kAxisCount; ++i) {
    if (axes_[i].present) ++n;
  }
  return n;
}

bool JoystickDecoder::configure(const ParsedDescriptor& desc)
{
  reset();
  uses_report_ids_ = desc.uses_report_ids;
  uint32_t max_button = 0;

  for (size_t i = 0; i < desc.field_count; ++i) {
    const InputField& f = desc.fields[i];
    if (f.is_relative || f.bit_size == 0 || f.bit_size > 32) {
      continue;
    }
    if (f.usage_page == kPageButton) {
      if (f.is_array) {
        if (button_array_count_ < 8) {
          button_arrays_[button_array_count_++] = f;
          if (f.usage_max > max_button) {
            max_button = f.usage_max;
          }
        }
      } else if (f.usage >= 1 && f.usage <= kMaxButtons &&
                 button_binding_count_ < kMaxButtons) {
        buttons_[button_binding_count_].field = f;
        buttons_[button_binding_count_].button_index = (uint8_t)(f.usage - 1);
        ++button_binding_count_;
        if (f.usage > max_button) {
          max_button = f.usage;
        }
      }
      continue;
    }
    if (f.is_array) {
      continue;
    }
    if (f.usage_page == kPageGenericDesktop && f.usage == kUsageHatSwitch) {
      if (!hat_.present) {
        hat_.present = true;
        hat_.field = f;
      }
      continue;
    }
    Axis axis;
    if (!axisFromUsage(f.usage_page, f.usage, axis)) {
      continue;
    }
    AxisBinding& slot = axes_[(size_t)axis];
    if (slot.present) {
      // Common quirk: two "Slider" usages (throttle + extra lever). Keep the
      // second one reachable by aliasing it to Dial if that is free.
      if (axis == Axis::Slider && !axes_[(size_t)Axis::Dial].present) {
        axes_[(size_t)Axis::Dial].present = true;
        axes_[(size_t)Axis::Dial].field = f;
      }
      continue;
    }
    slot.present = true;
    slot.field = f;
  }

  button_count_ = max_button > kMaxButtons ? kMaxButtons : max_button;
  bool any_axis = false;
  for (size_t i = 0; i < kAxisCount; ++i) {
    any_axis = any_axis || axes_[i].present;
  }
  return any_axis || hat_.present || button_count_ > 0;
}

int32_t JoystickDecoder::extract(const uint8_t* payload, size_t payload_len,
                                 const InputField& f, size_t element, bool& ok)
{
  const uint32_t start_bit = f.bit_offset + (uint32_t)element * f.bit_size;
  const uint32_t end_bit = start_bit + f.bit_size;
  if (f.bit_size == 0 || f.bit_size > 32 || end_bit > payload_len * 8) {
    ok = false;
    return 0;
  }
  ok = true;
  uint32_t value = 0;
  for (uint32_t b = 0; b < f.bit_size; ++b) {
    const uint32_t bit = start_bit + b;
    if (payload[bit >> 3] & (1u << (bit & 7))) {
      value |= (1u << b);
    }
  }
  if (f.logical_min < 0 && f.bit_size < 32) {
    // Sign-extend two's complement values.
    const uint32_t sign = 1u << (f.bit_size - 1);
    if (value & sign) {
      value |= ~((sign << 1) - 1);
    }
  }
  return (int32_t)value;
}

namespace {

int16_t normalise(int32_t v, const InputField& f)
{
  const int32_t span = f.logical_max - f.logical_min;
  if (span <= 0) {
    return 0;
  }
  if (v < f.logical_min) {
    v = f.logical_min;
  }
  if (v > f.logical_max) {
    v = f.logical_max;
  }
  int32_t out;
  if (f.logical_min < 0 && f.logical_max > 0) {
    // Signed range (e.g. -512..511): the device defines 0 as the centre, so
    // scale each side independently to keep the centre exact.
    out = v < 0 ? (v * kAxisFullScale) / (-f.logical_min)
                : (v * kAxisFullScale) / f.logical_max;
  } else {
    out = ((v - f.logical_min) * 2 * kAxisFullScale) / span - kAxisFullScale;
  }
  if (out > kAxisFullScale) out = kAxisFullScale;
  if (out < -kAxisFullScale) out = -kAxisFullScale;
  return (int16_t)out;
}

int8_t hatDirection(int32_t v, const InputField& f)
{
  const int32_t span = f.logical_max - f.logical_min + 1;
  if (v < f.logical_min || v > f.logical_max) {
    return -1;  // null state
  }
  const int32_t rel = v - f.logical_min;
  if (span == 8) {
    return (int8_t)rel;
  }
  if (span == 4) {
    return (int8_t)(rel * 2);
  }
  return -1;
}

}  // namespace

bool JoystickDecoder::decode(const uint8_t* report, size_t len,
                             InputState& state) const
{
  if (report == nullptr || len == 0) {
    return false;
  }
  uint8_t report_id = 0;
  const uint8_t* payload = report;
  size_t payload_len = len;
  if (uses_report_ids_) {
    report_id = report[0];
    payload = report + 1;
    payload_len = len - 1;
  }

  bool decoded_any = false;
  bool ok = false;

  for (size_t i = 0; i < kAxisCount; ++i) {
    const AxisBinding& a = axes_[i];
    if (!a.present || a.field.report_id != report_id) {
      continue;
    }
    const int32_t v = extract(payload, payload_len, a.field, 0, ok);
    if (ok) {
      state.axis[i] = normalise(v, a.field);
      state.axis_present |= (uint16_t)(1u << i);
      decoded_any = true;
    }
  }

  if (hat_.present && hat_.field.report_id == report_id) {
    const int32_t v = extract(payload, payload_len, hat_.field, 0, ok);
    if (ok) {
      state.hat = hatDirection(v, hat_.field);
      state.hat_present = true;
      decoded_any = true;
    }
  }

  bool touched_buttons = false;
  uint32_t buttons = state.buttons;
  for (size_t i = 0; i < button_binding_count_; ++i) {
    const ButtonBinding& b = buttons_[i];
    if (b.field.report_id != report_id) {
      continue;
    }
    const int32_t v = extract(payload, payload_len, b.field, 0, ok);
    if (!ok) {
      continue;
    }
    touched_buttons = true;
    const uint32_t mask = 1u << b.button_index;
    if (v != 0) {
      buttons |= mask;
    } else {
      buttons &= ~mask;
    }
  }
  for (size_t i = 0; i < button_array_count_; ++i) {
    const InputField& f = button_arrays_[i];
    if (f.report_id != report_id) {
      continue;
    }
    // Clear every button of the array's range, then set the reported ones.
    for (uint32_t u = f.usage; u <= f.usage_max && u <= kMaxButtons; ++u) {
      if (u >= 1) {
        buttons &= ~(1u << (u - 1));
      }
    }
    for (size_t e = 0; e < f.count; ++e) {
      const int32_t v = extract(payload, payload_len, f, e, ok);
      if (!ok) {
        break;
      }
      touched_buttons = true;
      if (v < f.logical_min || v > f.logical_max) {
        continue;
      }
      const int32_t usage = f.usage + (v - f.logical_min);
      if (usage >= 1 && usage <= (int32_t)kMaxButtons && usage <= f.usage_max) {
        buttons |= 1u << (usage - 1);
      }
    }
  }
  if (touched_buttons) {
    state.buttons = buttons;
    decoded_any = true;
  }
  state.button_count = (uint8_t)button_count_;
  return decoded_any;
}

}  // namespace hid
