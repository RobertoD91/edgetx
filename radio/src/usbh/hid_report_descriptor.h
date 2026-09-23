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

// Minimal, allocation-free USB HID report descriptor parser.
// It extracts the layout of every INPUT field (bit offset/size, usage,
// logical range) so that raw input reports can be decoded generically for
// any joystick/gamepad.
//
// Ported from esp32-joystick2trainer (lib/hid_core, MIT licensed).

#pragma once

#include <stddef.h>
#include <stdint.h>

namespace hid {

// Usage pages and usages relevant to joysticks (HID Usage Tables 1.x).
constexpr uint16_t kPageGenericDesktop = 0x01;
constexpr uint16_t kPageSimulation = 0x02;
constexpr uint16_t kPageButton = 0x09;

constexpr uint16_t kUsageJoystick = 0x04;
constexpr uint16_t kUsageGamepad = 0x05;
constexpr uint16_t kUsageMultiAxis = 0x08;
constexpr uint16_t kUsageX = 0x30;
constexpr uint16_t kUsageY = 0x31;
constexpr uint16_t kUsageZ = 0x32;
constexpr uint16_t kUsageRx = 0x33;
constexpr uint16_t kUsageRy = 0x34;
constexpr uint16_t kUsageRz = 0x35;
constexpr uint16_t kUsageSlider = 0x36;
constexpr uint16_t kUsageDial = 0x37;
constexpr uint16_t kUsageWheel = 0x38;
constexpr uint16_t kUsageHatSwitch = 0x39;
constexpr uint16_t kUsageSimRudder = 0xBA;
constexpr uint16_t kUsageSimThrottle = 0xBB;
constexpr uint16_t kUsageSimAccelerator = 0xC4;
constexpr uint16_t kUsageSimBrake = 0xC5;

// One input field (a "Report Count" run of identical items is expanded into
// individual variable fields; array items are kept as a single field).
struct InputField {
  uint8_t report_id = 0;   // 0 when the device does not use report IDs.
  uint16_t usage_page = 0;
  uint16_t usage = 0;      // For variable fields. For array fields: usage minimum.
  uint16_t usage_max = 0;  // For array fields only.
  uint16_t bit_offset = 0; // Offset in bits from the start of the report payload
                           // (after the report ID byte when IDs are used).
  uint8_t bit_size = 0;    // Size of one element.
  uint8_t count = 1;       // Number of elements (array fields only, 1 otherwise).
  int32_t logical_min = 0;
  int32_t logical_max = 0;
  bool is_array = false;
  bool is_relative = false;
};

constexpr size_t kMaxInputFields = 64;
constexpr size_t kMaxReportIds = 16;

struct ReportSizeEntry {
  uint8_t report_id = 0;
  uint16_t bits = 0;
};

struct ParsedDescriptor {
  InputField fields[kMaxInputFields];
  size_t field_count = 0;
  bool uses_report_ids = false;
  // Usage of the top-level application collection (Joystick, Gamepad, ...),
  // first one found.
  uint16_t application_usage_page = 0;
  uint16_t application_usage = 0;
  // Payload size (in bits, excluding the report ID byte) of every input report
  // seen.
  ReportSizeEntry report_sizes[kMaxReportIds];
  size_t report_size_count = 0;

  // Payload length in bytes for a given report ID (0 when unknown).
  size_t reportBytes(uint8_t report_id) const;

  // Reset in place (the struct is large: avoid temporaries on small stacks).
  void clear();
};

enum class ParseResult {
  Ok,
  Truncated,      // Descriptor ended in the middle of an item.
  TooManyFields,  // More input fields than kMaxInputFields (descriptor was cut short).
  StackOverflow,  // Push/Pop nesting deeper than supported.
  NoInputFields,  // Nothing usable found.
};

const char* parseResultName(ParseResult r);

// Parse a raw report descriptor. Never allocates. Fields marked "Constant"
// (padding) are skipped but still advance the bit offset.
ParseResult parse(const uint8_t* desc, size_t len, ParsedDescriptor& out);

}  // namespace hid
