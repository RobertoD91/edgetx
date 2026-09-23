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

#include "usbh_joystick_trainer.h"

uint8_t usbhJoystickTrainerChannels(const hid::JoystickDecoder& dec)
{
  uint32_t n = dec.axisCount();
  if (dec.hatBinding().present) n += 2;
  n += dec.buttonCount();
  return n > 255 ? 255 : (uint8_t)n;
}

uint8_t usbhJoystickToTrainer(const hid::InputState& state,
                              const hid::JoystickDecoder& dec, int16_t* out,
                              uint8_t max_channels)
{
  uint8_t ch = 0;

  for (size_t i = 0; i < hid::kAxisCount && ch < max_channels; ++i) {
    if (!dec.axisBinding((hid::Axis)i).present) continue;
    // RESX (+/-1024) -> trainer units (+/-512)
    out[ch++] = state.axis[i] / 2;
  }

  if (dec.hatBinding().present) {
    if (ch < max_channels) out[ch++] = state.hatX() * USBH_TRAINER_FULL_SCALE;
    if (ch < max_channels) out[ch++] = state.hatY() * USBH_TRAINER_FULL_SCALE;
  }

  for (size_t b = 0; b < dec.buttonCount() && ch < max_channels; ++b) {
    out[ch++] = state.button(b) ? USBH_TRAINER_FULL_SCALE
                                : -USBH_TRAINER_FULL_SCALE;
  }

  uint8_t used = ch;
  while (ch < max_channels) out[ch++] = 0;
  return used;
}
