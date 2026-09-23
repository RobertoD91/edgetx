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

// Only depends on the portable usbh/ sources: can also be built standalone
// with the system googletest:
//   g++ -std=c++17 -Iradio/src radio/src/usbh/*.cpp radio/src/tests/usbh_hid.cpp
//       -lgtest -lgtest_main -pthread

#include "gtest/gtest.h"

#include "usbh/hid_report_descriptor.h"
#include "usbh/hid_joystick_decoder.h"
#include "usbh/usbh_joystick_trainer.h"
#include "usbh_hid_fixtures.h"

using namespace hid;

static const InputField* findField(const ParsedDescriptor& d, uint8_t report_id,
                                   uint16_t page, uint16_t usage)
{
  for (size_t i = 0; i < d.field_count; ++i) {
    const InputField& f = d.fields[i];
    if (f.report_id == report_id && f.usage_page == page && f.usage == usage) {
      return &f;
    }
  }
  return nullptr;
}

TEST(UsbHostHid, sidewinderDescriptorParses)
{
  ParsedDescriptor d;
  const ParseResult r =
      parse(kSideWinderFfb2Descriptor, kSideWinderFfb2DescriptorLen, d);
  EXPECT_EQ(ParseResult::Ok, r);
  EXPECT_TRUE(d.uses_report_ids);
  EXPECT_EQ(kPageGenericDesktop, d.application_usage_page);
  EXPECT_EQ(kUsageJoystick, d.application_usage);
  EXPECT_EQ(12u, d.reportBytes(1));  // 96 bits of payload
  EXPECT_EQ(2u, d.reportBytes(2));   // PID state report

  const InputField* x = findField(d, 1, kPageGenericDesktop, kUsageX);
  ASSERT_NE(nullptr, x);
  EXPECT_EQ(0, x->bit_offset);
  EXPECT_EQ(10, x->bit_size);
  EXPECT_EQ(-512, x->logical_min);
  EXPECT_EQ(511, x->logical_max);

  const InputField* rz = findField(d, 1, kPageGenericDesktop, kUsageRz);
  ASSERT_NE(nullptr, rz);
  EXPECT_EQ(32, rz->bit_offset);
  EXPECT_EQ(6, rz->bit_size);

  const InputField* hat = findField(d, 1, kPageGenericDesktop, kUsageHatSwitch);
  ASSERT_NE(nullptr, hat);
  EXPECT_EQ(48, hat->bit_offset);
  EXPECT_EQ(4, hat->bit_size);

  for (uint16_t b = 1; b <= 8; ++b) {
    const InputField* btn = findField(d, 1, kPageButton, b);
    ASSERT_NE(nullptr, btn);
    EXPECT_EQ(56 + (b - 1), btn->bit_offset);
    EXPECT_EQ(1, btn->bit_size);
  }
  EXPECT_EQ(nullptr, findField(d, 1, kPageButton, 9));
}

TEST(UsbHostHid, sidewinderDecodeAndTrainerMapping)
{
  ParsedDescriptor d;
  ASSERT_EQ(ParseResult::Ok,
            parse(kSideWinderFfb2Descriptor, kSideWinderFfb2DescriptorLen, d));
  JoystickDecoder dec;
  ASSERT_TRUE(dec.configure(d));
  EXPECT_TRUE(dec.usesReportIds());
  EXPECT_EQ(8u, dec.buttonCount());
  EXPECT_EQ(4u, dec.axisCount());  // X, Y, Rz, Slider
  EXPECT_TRUE(dec.hatBinding().present);
  EXPECT_EQ(4 + 2 + 8, usbhJoystickTrainerChannels(dec));

  // Report ID 1: X = -512, Y = +511, Rz = -32, slider = 127, hat = 2 (right),
  // buttons 1 and 3 pressed, 4 bytes of padding.
  const uint8_t report[13] = {0x01, 0x00, 0x02, 0xFF, 0x01, 0x20, 0x7F,
                              0x02, 0x05, 0,    0,    0,    0};
  InputState s;
  s.clear();
  ASSERT_TRUE(dec.decode(report, sizeof(report), s));
  EXPECT_EQ(-1024, s.axis[(int)Axis::X]);
  EXPECT_EQ(1024, s.axis[(int)Axis::Y]);
  EXPECT_EQ(-1024, s.axis[(int)Axis::Rz]);
  EXPECT_EQ(1024, s.axis[(int)Axis::Slider]);
  EXPECT_TRUE(s.axisPresent(Axis::X));
  EXPECT_FALSE(s.axisPresent(Axis::Z));
  EXPECT_EQ(2, s.hat);
  EXPECT_EQ(1, s.hatX());
  EXPECT_EQ(0, s.hatY());
  EXPECT_EQ(0x05u, s.buttons);

  int16_t ch[16];
  EXPECT_EQ(14, usbhJoystickToTrainer(s, dec, ch, 16));
  EXPECT_EQ(-512, ch[0]);  // X
  EXPECT_EQ(512, ch[1]);   // Y
  EXPECT_EQ(-512, ch[2]);  // Rz
  EXPECT_EQ(512, ch[3]);   // Slider
  EXPECT_EQ(512, ch[4]);   // hat X
  EXPECT_EQ(0, ch[5]);     // hat Y
  EXPECT_EQ(512, ch[6]);   // button 1
  EXPECT_EQ(-512, ch[7]);  // button 2
  EXPECT_EQ(512, ch[8]);   // button 3
  EXPECT_EQ(-512, ch[13]); // button 8
  EXPECT_EQ(0, ch[14]);
  EXPECT_EQ(0, ch[15]);

  // Centre report, hat in null state, buttons released
  const uint8_t centre[13] = {0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40,
                              0x08, 0x00, 0,    0,    0,    0};
  ASSERT_TRUE(dec.decode(centre, sizeof(centre), s));
  EXPECT_EQ(0, s.axis[(int)Axis::X]);
  EXPECT_EQ(0, s.axis[(int)Axis::Y]);
  EXPECT_EQ(0, s.axis[(int)Axis::Rz]);
  EXPECT_NEAR(0, s.axis[(int)Axis::Slider], 10);
  EXPECT_EQ(-1, s.hat);
  EXPECT_EQ(0u, s.buttons);

  // A report with another ID (PID state) must not touch the state
  s.buttons = 0x81;
  const uint8_t pid_state[3] = {0x02, 0xFF, 0xFF};
  EXPECT_FALSE(dec.decode(pid_state, sizeof(pid_state), s));
  EXPECT_EQ(0x81u, s.buttons);

  // Truncated report: X and Y fit, the rest is skipped safely
  const uint8_t truncated[5] = {0x01, 0xFF, 0x01, 0x00, 0x02};
  EXPECT_TRUE(dec.decode(truncated, sizeof(truncated), s));
  EXPECT_EQ(1024, s.axis[(int)Axis::X]);
  EXPECT_EQ(-1024, s.axis[(int)Axis::Y]);
  EXPECT_EQ(0x81u, s.buttons);
}

static const uint8_t kGamepadDescriptor[] = {
    0x05, 0x01,        // Usage Page (Generic Desktop)
    0x09, 0x05,        // Usage (Gamepad)
    0xA1, 0x01,        // Collection (Application)
    0x15, 0x00,        //   Logical Minimum (0)
    0x25, 0x01,        //   Logical Maximum (1)
    0x75, 0x01,        //   Report Size (1)
    0x95, 0x0C,        //   Report Count (12)
    0x05, 0x09,        //   Usage Page (Button)
    0x19, 0x01,        //   Usage Minimum (1)
    0x29, 0x0C,        //   Usage Maximum (12)
    0x81, 0x02,        //   Input (Data,Var,Abs)          -> bits 0..11
    0x95, 0x04,        //   Report Count (4)
    0x81, 0x01,        //   Input (Const)                 -> bits 12..15
    0x05, 0x01,        //   Usage Page (Generic Desktop)
    0x25, 0x07,        //   Logical Maximum (7)
    0x46, 0x3B, 0x01,  //   Physical Maximum (315)
    0x75, 0x04,        //   Report Size (4)
    0x95, 0x01,        //   Report Count (1)
    0x65, 0x14,        //   Unit (degrees)
    0x09, 0x39,        //   Usage (Hat switch)
    0x81, 0x42,        //   Input (Data,Var,Abs,Null)     -> bits 16..19
    0x65, 0x00,        //   Unit (none)
    0x95, 0x01,        //   Report Count (1)
    0x75, 0x04,        //   Report Size (4)
    0x81, 0x01,        //   Input (Const)                 -> bits 20..23
    0x26, 0xFF, 0x00,  //   Logical Maximum (255)
    0x46, 0xFF, 0x00,  //   Physical Maximum (255)
    0x09, 0x30,        //   Usage (X)
    0x09, 0x31,        //   Usage (Y)
    0x09, 0x32,        //   Usage (Z)
    0x09, 0x35,        //   Usage (Rz)
    0x75, 0x08,        //   Report Size (8)
    0x95, 0x04,        //   Report Count (4)
    0x81, 0x02,        //   Input (Data,Var,Abs)          -> bytes 3..6
    0xC0,              // End Collection
};

TEST(UsbHostHid, gamepadWithoutReportIds)
{
  ParsedDescriptor d;
  ASSERT_EQ(ParseResult::Ok,
            parse(kGamepadDescriptor, sizeof(kGamepadDescriptor), d));
  EXPECT_FALSE(d.uses_report_ids);
  EXPECT_EQ(kUsageGamepad, d.application_usage);
  EXPECT_EQ(7u, d.reportBytes(0));

  JoystickDecoder dec;
  ASSERT_TRUE(dec.configure(d));
  EXPECT_EQ(12u, dec.buttonCount());
  EXPECT_EQ(24, dec.axisBinding(Axis::X).field.bit_offset);
  EXPECT_EQ(48, dec.axisBinding(Axis::Rz).field.bit_offset);
  EXPECT_EQ(4 + 2 + 12, usbhJoystickTrainerChannels(dec));

  // buttons 1, 3 and 12; hat = 6 (left); X=0, Y=255, Z=128, Rz=64
  const uint8_t report[7] = {0x05, 0x08, 0x06, 0x00, 0xFF, 0x80, 0x40};
  InputState s;
  s.clear();
  ASSERT_TRUE(dec.decode(report, sizeof(report), s));
  EXPECT_EQ((1u << 0) | (1u << 2) | (1u << 11), s.buttons);
  EXPECT_EQ(6, s.hat);
  EXPECT_EQ(-1, s.hatX());
  EXPECT_EQ(-1024, s.axis[(int)Axis::X]);
  EXPECT_EQ(1024, s.axis[(int)Axis::Y]);
  EXPECT_NEAR(0, s.axis[(int)Axis::Z], 10);
  EXPECT_NEAR(-512, s.axis[(int)Axis::Rz], 10);

  // 18 channels needed, only 16 (MAX_TRAINER_CHANNELS) available: the last
  // two buttons are dropped
  int16_t ch[16];
  EXPECT_EQ(16, usbhJoystickToTrainer(s, dec, ch, 16));
  EXPECT_EQ(-512, ch[4]);  // hat X (left)
  EXPECT_EQ(0, ch[5]);     // hat Y
  EXPECT_EQ(512, ch[6]);   // button 1
  EXPECT_EQ(512, ch[8]);   // button 3
  EXPECT_EQ(-512, ch[15]); // button 10
}

static const uint8_t kArrayButtonDescriptor[] = {
    0x05, 0x01,  // Usage Page (Generic Desktop)
    0x09, 0x04,  // Usage (Joystick)
    0xA1, 0x01,  // Collection (Application)
    0x05, 0x09,  //   Usage Page (Button)
    0x19, 0x01,  //   Usage Minimum (1)
    0x29, 0x10,  //   Usage Maximum (16)
    0x15, 0x01,  //   Logical Minimum (1)
    0x25, 0x10,  //   Logical Maximum (16)
    0x75, 0x08,  //   Report Size (8)
    0x95, 0x02,  //   Report Count (2)
    0x81, 0x00,  //   Input (Data,Array,Abs)
    0x05, 0x01,  //   Usage Page (Generic Desktop)
    0x09, 0x30,  //   Usage (X)
    0x15, 0x81,  //   Logical Minimum (-127)
    0x25, 0x7F,  //   Logical Maximum (127)
    0x75, 0x08,  //   Report Size (8)
    0x95, 0x01,  //   Report Count (1)
    0x81, 0x02,  //   Input (Data,Var,Abs)
    0xC0,        // End Collection
};

TEST(UsbHostHid, arrayButtonsAndSignedAxis)
{
  ParsedDescriptor d;
  ASSERT_EQ(ParseResult::Ok,
            parse(kArrayButtonDescriptor, sizeof(kArrayButtonDescriptor), d));
  EXPECT_EQ(2u, d.field_count);
  EXPECT_TRUE(d.fields[0].is_array);
  EXPECT_EQ(2, d.fields[0].count);

  JoystickDecoder dec;
  ASSERT_TRUE(dec.configure(d));
  EXPECT_EQ(16u, dec.buttonCount());

  InputState s;
  s.clear();
  const uint8_t report1[3] = {3, 7, 0x7F};
  ASSERT_TRUE(dec.decode(report1, sizeof(report1), s));
  EXPECT_EQ((1u << 2) | (1u << 6), s.buttons);
  EXPECT_EQ(1024, s.axis[(int)Axis::X]);

  const uint8_t report2[3] = {0, 0, 0x81};  // released, X = -127
  ASSERT_TRUE(dec.decode(report2, sizeof(report2), s));
  EXPECT_EQ(0u, s.buttons);
  EXPECT_EQ(-1024, s.axis[(int)Axis::X]);
}

TEST(UsbHostHid, parserErrorPaths)
{
  ParsedDescriptor d;
  EXPECT_EQ(ParseResult::NoInputFields, parse(nullptr, 0, d));
  const uint8_t empty[1] = {0xC0};
  EXPECT_EQ(ParseResult::NoInputFields, parse(empty, sizeof(empty), d));
  const uint8_t truncated[2] = {0x05, 0x01};
  EXPECT_EQ(ParseResult::Truncated, parse(truncated, 1, d));
  EXPECT_EQ(ParseResult::Truncated, parse(kSideWinderFfb2Descriptor, 33, d));
  const uint8_t pop_without_push[] = {0x05, 0x01, 0xB4, 0x00};
  EXPECT_EQ(ParseResult::StackOverflow,
            parse(pop_without_push, sizeof(pop_without_push), d));

  parse(kGamepadDescriptor, sizeof(kGamepadDescriptor), d);
  JoystickDecoder dec;
  dec.configure(d);
  InputState s;
  s.clear();
  EXPECT_FALSE(dec.decode(nullptr, 0, s));
  const uint8_t one = 0;
  EXPECT_FALSE(dec.decode(&one, 0, s));
}

TEST(UsbHostHid, fieldExtractionSignExtension)
{
  InputField f;
  f.bit_offset = 4;
  f.bit_size = 6;
  f.logical_min = -32;
  f.logical_max = 31;
  // bits 4..9 of the payload = 0b100000 (-32)
  const uint8_t payload[2] = {0x00, 0x02};
  bool ok = false;
  EXPECT_EQ(-32, JoystickDecoder::extract(payload, sizeof(payload), f, 0, ok));
  EXPECT_TRUE(ok);
  f.logical_min = 0;
  EXPECT_EQ(32, JoystickDecoder::extract(payload, sizeof(payload), f, 0, ok));
  JoystickDecoder::extract(payload, sizeof(payload), f, 2, ok);
  EXPECT_FALSE(ok);
}
