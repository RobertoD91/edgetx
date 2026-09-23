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

#include <stdbool.h>
#include <stdint.h>

// USB host driver: the radio's USB port acts as a host for a HID joystick
// or gamepad whose axes/buttons are fed into the trainer input channels.
//
// The OTG core is shared with the USB device stack (mass storage, joystick,
// serial): only one of the two can own it at a time. While the host is
// active, usbPlugged() reports "not plugged" so that the device stack stays
// off.

enum UsbHostJoystickStatus {
  USBH_JOYSTICK_OFF,            // host not requested
  USBH_JOYSTICK_USB_BUSY,       // device stack owns the USB core (PC plugged)
  USBH_JOYSTICK_WAIT_DEVICE,    // host running, nothing plugged
  USBH_JOYSTICK_ENUMERATING,    // device plugged, reading descriptors
  USBH_JOYSTICK_READY,          // joystick reports are feeding the trainer
  USBH_JOYSTICK_NOT_SUPPORTED,  // plugged device is not a usable HID joystick
  USBH_JOYSTICK_ERROR,          // enumeration/transfer failure
};

struct UsbHostJoystickInfo {
  uint16_t vid;
  uint16_t pid;
  uint8_t speed;      // 1 = full speed, 2 = low speed
  uint8_t axes;
  uint8_t buttons;
  uint8_t hat;        // 1 when a hat switch is present
  uint8_t channels;   // trainer channels produced by the device
  uint8_t interval;   // poll interval in ms
  uint16_t reports;   // reports received (wraps)
};

// Request the USB host to run. Returns false when the USB device stack is
// currently started (a PC is connected); the caller may retry later.
bool usbHostJoystickStart();

// Release the USB core.
void usbHostJoystickStop();

// true between usbHostJoystickStart() and usbHostJoystickStop()
bool usbHostJoystickEnabled();

// true while the OTG core is configured in host mode (the device stack must
// not touch it)
bool usbHostActive();

UsbHostJoystickStatus usbHostJoystickStatus();
const UsbHostJoystickInfo* usbHostJoystickInfo();

// Called from the OTG interrupt handler while usbHostActive()
void usbHostIRQHandler();
