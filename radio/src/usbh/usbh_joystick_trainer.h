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

#pragma once

#include <stdint.h>

#include "hid_joystick_decoder.h"

// Trainer input full scale (see trainerInput[] / sbusProcessFrame()).
constexpr int16_t USBH_TRAINER_FULL_SCALE = 512;

// Number of trainer channels a given decoder configuration produces:
// every axis reported by the device (in hid::Axis order), then the hat
// switch as two channels (horizontal, vertical), then one channel per button.
uint8_t usbhJoystickTrainerChannels(const hid::JoystickDecoder& dec);

// Map a decoded joystick state to trainer channel values, in the same order
// as usbhJoystickTrainerChannels(). Channels beyond the ones produced by the
// device are set to 0. Values are in [-USBH_TRAINER_FULL_SCALE,
// +USBH_TRAINER_FULL_SCALE]; buttons give -512 (released) / +512 (pressed).
// Returns the number of channels filled from the device.
uint8_t usbhJoystickToTrainer(const hid::InputState& state,
                              const hid::JoystickDecoder& dec, int16_t* out,
                              uint8_t max_channels);
