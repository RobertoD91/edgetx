# USB host: HID joystick as trainer input

Experimental, first enabled on the RadioMaster TX16S (STM32F429, OTG_FS core
on the USB-C port). Build option `USB_HOST_JOYSTICK` (ON for the TX16S).

The radio's USB port is switched to host mode and a USB HID joystick or
gamepad plugged into it feeds the trainer input channels, exactly like an
SBUS/CPPM trainer signal would. Everything after that (trainer mixes, channel
sources `TR1..TR16`, calibration) is the normal EdgeTX trainer machinery.

## Files

- `hid_report_descriptor.*` – HID report descriptor parser (portable).
- `hid_joystick_decoder.*` – turns raw input reports into axes / hat / buttons
  (portable, values in RESX units).
- `usbh_joystick_trainer.*` – maps the decoded state to trainer channels.
- `../targets/common/arm/stm32/usb_host_driver.cpp` – STM32 HAL HCD based
  host: enumeration, HID report descriptor fetch, interrupt IN polling, in a
  dedicated task.
- `../tests/usbh_hid.cpp` – unit tests (SideWinder FFB2 descriptor, generic
  gamepad, array buttons, error paths).

## Using it

1. Model settings → Trainer → Mode: **Master/USB HID**. The line below the
   mode shows the host status (waiting, enumerating, `VID:PID n axes n btn
   -> n ch`, not supported, error).
2. Plug the joystick through a **USB OTG cable with external 5V power input**
   (a "Y" OTG cable). The radio does not switch 5V onto the USB connector,
   so VBUS for the joystick has to come from the cable's power input; that
   5V also reaches the radio's charging circuit on radios that charge from
   USB, so use a supply of 2A or more.
3. Trainer channel order: every axis the device reports (X, Y, Z, Rx, Ry, Rz,
   slider, dial, wheel, throttle, rudder, accelerator, brake, in that order),
   then hat horizontal and vertical, then one channel per button (up to 16
   channels in total). Buttons give -100/+100, the hat -100/0/+100.
4. The trainer signal is considered valid while the device answers on its
   interrupt endpoint (reports or NAKs); on unplug the usual "trainer lost"
   handling applies.

While the host is enabled the device stack (mass storage / joystick /
serial) is kept off: unplug the joystick cable and switch the trainer mode
back to use the radio as a USB device again. If a PC is connected when the
mode is selected, the host waits until the PC is unplugged.

## Known limits

- Single device, no hubs, first HID interface only (boot keyboard/mouse
  interfaces are skipped).
- Report descriptors longer than 2 KB are truncated, configuration
  descriptors longer than 256 bytes too.
- No force feedback / output reports.
