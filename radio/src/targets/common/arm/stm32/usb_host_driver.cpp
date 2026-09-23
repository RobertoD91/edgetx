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

// Minimal USB host on top of the STM32 HAL HCD driver (OTG_FS core),
// limited to what a HID joystick / gamepad needs: single device, default
// control pipe, one interrupt IN endpoint polled at its bInterval.
//
// Everything runs in a dedicated low priority task: enumeration is a plain
// sequence of blocking control transfers, the poll loop sleeps between
// interrupt IN requests. The HAL callbacks only set flags.
//
// Hardware note: the radio does not switch 5V onto the USB connector, VBUS
// has to be supplied to the joystick externally (OTG Y-cable with a power
// input). The core is put in forced host mode, so the ID pin is not used.

#include "hal/usb_host_driver.h"
#include "hal/usb_driver.h"

#include "stm32_hal.h"
#include "stm32_hal_ll.h"
#include "timers_driver.h"

#include "os/sleep.h"
#include "os/task.h"
#include "tasks.h"

#include "hal.h"
#include "debug.h"
#include "trainer.h"

#include "usbh/hid_report_descriptor.h"
#include "usbh/hid_joystick_decoder.h"
#include "usbh/usbh_joystick_trainer.h"

#include <string.h>

// Host channels used
#define USBH_CH_CTRL_OUT 0
#define USBH_CH_CTRL_IN 1
#define USBH_CH_INTR_IN 2

#define USBH_DEVICE_ADDRESS 1
#define USBH_CTRL_TIMEOUT_MS 500
#define USBH_RESET_TIMEOUT_MS 300
#define USBH_MAX_POLL_ERRORS 10

// Standard requests
#define USB_REQ_GET_DESCRIPTOR 0x06
#define USB_REQ_SET_ADDRESS 0x05
#define USB_REQ_SET_CONFIGURATION 0x09
#define USB_REQ_CLEAR_FEATURE 0x01
#define USB_DESC_DEVICE 0x01
#define USB_DESC_CONFIGURATION 0x02
#define USB_DESC_INTERFACE 0x04
#define USB_DESC_ENDPOINT 0x05
#define USB_DESC_HID 0x21
#define USB_DESC_HID_REPORT 0x22
#define USB_CLASS_HID 0x03
#define HID_REQ_SET_IDLE 0x0A

// Buffers (the HAL reads whole packets: keep some slack after the requested
// length)
#define USBH_CFG_DESC_MAX 256
#define USBH_REPORT_DESC_MAX 2048
#define USBH_REPORT_MAX 64

#define USBH_STACK_SIZE 1024
#define USBH_TASK_PRIO MENUS_TASK_PRIO

struct HidEndpoint {
  uint8_t iface;
  uint8_t ep_addr;
  uint16_t mps;
  uint8_t interval;
  uint16_t report_desc_len;
};

static HCD_HandleTypeDef hhcd_USB_HOST;

static task_handle_t usbhTaskId;
TASK_DEFINE_STACK(usbhStack, USBH_STACK_SIZE);
static bool _taskCreated = false;

static volatile bool _enabled = false;     // requested by the trainer code
static volatile bool _coreActive = false;  // OTG core configured as host
static volatile bool _connected = false;   // port connect status
static volatile bool _portEnabled = false;
static volatile UsbHostJoystickStatus _status = USBH_JOYSTICK_OFF;

static UsbHostJoystickInfo _info;

static uint8_t _setup[8];
static uint8_t _dummy[8];
static uint8_t _cfgDesc[USBH_CFG_DESC_MAX + 64];
static uint8_t _reportDesc[USBH_REPORT_DESC_MAX + 64];
static uint8_t _report[USBH_REPORT_MAX];

static hid::ParsedDescriptor _parsed;
static hid::JoystickDecoder _decoder;
static hid::InputState _state;

/*
 * HAL glue
 *
 * The callbacks are registered through the handle (USE_HAL_HCD_REGISTER_CALLBACKS)
 * instead of overriding the weak HAL functions: with LTO the linker inlined
 * the empty weak versions into the HAL, and the clock/IRQ were never enabled.
 */

#if (USE_HAL_HCD_REGISTER_CALLBACKS != 1U)
#error "USB host driver needs USE_HAL_HCD_REGISTER_CALLBACKS"
#endif

static void usbhMspInit(HCD_HandleTypeDef* hhcd)
{
  if (hhcd->Instance == USB_OTG_FS) {
    __HAL_RCC_USB_OTG_FS_CLK_ENABLE();
    NVIC_SetPriority(OTG_FS_IRQn, 11);
    NVIC_EnableIRQ(OTG_FS_IRQn);
  }
}

static void usbhMspDeInit(HCD_HandleTypeDef* hhcd)
{
  if (hhcd->Instance == USB_OTG_FS) {
    NVIC_DisableIRQ(OTG_FS_IRQn);
    __HAL_RCC_USB_OTG_FS_CLK_DISABLE();
  }
}

static void usbhConnectCb(HCD_HandleTypeDef* hhcd) { _connected = true; }

static void usbhDisconnectCb(HCD_HandleTypeDef* hhcd)
{
  _connected = false;
  _portEnabled = false;
}

static void usbhPortEnabledCb(HCD_HandleTypeDef* hhcd) { _portEnabled = true; }

static void usbhPortDisabledCb(HCD_HandleTypeDef* hhcd)
{
  _portEnabled = false;
}

void usbHostIRQHandler()
{
  _info.irqs++;
  HAL_HCD_IRQHandler(&hhcd_USB_HOST);
}

static void updateDiagnostics()
{
  uint32_t USBx_BASE = (uint32_t)USB_OTG_FS;
  _info.hprt = USBx_HPRT0;
  _info.gintsts = USB_OTG_FS->GINTSTS;
}

bool usbHostActive() { return _coreActive; }

bool usbHostJoystickEnabled() { return _enabled; }

UsbHostJoystickStatus usbHostJoystickStatus() { return _status; }

const UsbHostJoystickInfo* usbHostJoystickInfo() { return &_info; }

/*
 * Core
 */

static void coreInit()
{
  memset(&hhcd_USB_HOST, 0, sizeof(hhcd_USB_HOST));
  hhcd_USB_HOST.Instance = USB_OTG_FS;
  hhcd_USB_HOST.Init.Host_channels = 8;
  hhcd_USB_HOST.Init.speed = HCD_SPEED_FULL;
  hhcd_USB_HOST.Init.dma_enable = DISABLE;
  hhcd_USB_HOST.Init.phy_itface = HCD_PHY_EMBEDDED;
  hhcd_USB_HOST.Init.Sof_enable = DISABLE;
  hhcd_USB_HOST.Init.low_power_enable = DISABLE;
  hhcd_USB_HOST.Init.lpm_enable = DISABLE;
  hhcd_USB_HOST.Init.vbus_sensing_enable = DISABLE;
  hhcd_USB_HOST.Init.use_external_vbus = DISABLE;

  // MSP callbacks are looked up by HAL_HCD_Init() itself
  hhcd_USB_HOST.MspInitCallback = usbhMspInit;
  hhcd_USB_HOST.MspDeInitCallback = usbhMspDeInit;

  _connected = false;
  _portEnabled = false;
  _coreActive = true;

  HAL_HCD_Init(&hhcd_USB_HOST);

  // HAL_HCD_Init() installs the (weak, empty) defaults: replace them before
  // the global interrupt is enabled by HAL_HCD_Start()
  hhcd_USB_HOST.ConnectCallback = usbhConnectCb;
  hhcd_USB_HOST.DisconnectCallback = usbhDisconnectCb;
  hhcd_USB_HOST.PortEnabledCallback = usbhPortEnabledCb;
  hhcd_USB_HOST.PortDisabledCallback = usbhPortDisabledCb;

  HAL_HCD_Start(&hhcd_USB_HOST);
  updateDiagnostics();
  TRACE("USBH: host started, HPRT=%08lX", _info.hprt);
}

static void coreDeinit()
{
  HAL_HCD_Stop(&hhcd_USB_HOST);
  HAL_HCD_DeInit(&hhcd_USB_HOST);
  _coreActive = false;
  _connected = false;
  _portEnabled = false;
  TRACE("USBH: host stopped");
}

/*
 * Transfers
 */

static inline HCD_URBStateTypeDef urbState(uint8_t ch)
{
  return *(volatile HCD_URBStateTypeDef*)&hhcd_USB_HOST.hc[ch].urb_state;
}

static inline bool channelEnabled(uint8_t ch)
{
  uint32_t USBx_BASE = (uint32_t)USB_OTG_FS;
  return (USBx_HC(ch)->HCCHAR & USB_OTG_HCCHAR_CHENA) != 0;
}

static bool submit(uint8_t ch, uint8_t direction, uint8_t ep_type,
                   uint8_t token, uint8_t* buf, uint16_t len)
{
  return HAL_HCD_HC_SubmitRequest(&hhcd_USB_HOST, ch, direction, ep_type,
                                  token, buf, len, 0) == HAL_OK;
}

// Halt a channel and make sure it stays halted: the HAL "channel halted"
// handler re-activates a channel whose state is HC_NAK (control/bulk NAK
// retry), so the state is cleared first.
static void forceHalt(uint8_t ch)
{
  for (int i = 0; i < 10; i++) {
    hhcd_USB_HOST.hc[ch].state = HC_IDLE;
    HAL_HCD_HC_Halt(&hhcd_USB_HOST, ch);
    if (!channelEnabled(ch)) return;
    sleep_ms(1);
  }
}

// Wait for a control URB to finish: DONE / STALL / ERROR / timeout, or
// NOTREADY (NAK / transaction error retry) when `stopOnNotReady` is set.
static HCD_URBStateTypeDef waitUrb(uint8_t ch, uint32_t start,
                                   uint32_t timeout_ms, bool stopOnNotReady)
{
  for (;;) {
    HCD_URBStateTypeDef st = urbState(ch);
    if (st == URB_DONE || st == URB_STALL || st == URB_ERROR) return st;
    if (st == URB_NOTREADY && stopOnNotReady) return st;
    if (!_connected || !_enabled) return URB_ERROR;
    if (timersGetMsTick() - start > timeout_ms) {
      forceHalt(ch);
      return URB_ERROR;
    }
    sleep_ms(1);
  }
}

// One stage of a control transfer. On a NAK the HAL re-activates IN
// channels by itself, but OUT channels (SETUP, data OUT, status OUT) are
// left halted with URB_NOTREADY: they have to be re-submitted here, like the
// ST USB host library does.
static bool stage(uint8_t ch, uint8_t direction, uint8_t token, uint8_t* buf,
                  uint16_t len, HCD_URBStateTypeDef* st)
{
  uint32_t start = timersGetMsTick();
  const bool isOut = (direction == 0);

  for (;;) {
    if (channelEnabled(ch)) forceHalt(ch);
    if (!submit(ch, direction, EP_TYPE_CTRL, token, buf, len)) {
      *st = URB_ERROR;
      return false;
    }
    *st = waitUrb(ch, start, USBH_CTRL_TIMEOUT_MS, isOut);
    if (*st != URB_NOTREADY) return *st == URB_DONE;

    if (timersGetMsTick() - start > USBH_CTRL_TIMEOUT_MS) {
      forceHalt(ch);
      *st = URB_ERROR;
      return false;
    }
    sleep_ms(1);
  }
}

// Control transfer on the default pipe. `data` is the IN buffer or the OUT
// payload depending on bmRequestType. Returns true when all stages completed.
static bool ctrlRequest(uint8_t bmRequestType, uint8_t bRequest,
                        uint16_t wValue, uint16_t wIndex, uint8_t* data,
                        uint16_t wLength, HCD_URBStateTypeDef* st)
{
  _setup[0] = bmRequestType;
  _setup[1] = bRequest;
  _setup[2] = wValue & 0xFF;
  _setup[3] = wValue >> 8;
  _setup[4] = wIndex & 0xFF;
  _setup[5] = wIndex >> 8;
  _setup[6] = wLength & 0xFF;
  _setup[7] = wLength >> 8;

  // SETUP stage
  if (!stage(USBH_CH_CTRL_OUT, 0, 0, _setup, 8, st)) return false;

  if (wLength > 0) {
    if (bmRequestType & 0x80) {
      // DATA IN, STATUS OUT
      if (!stage(USBH_CH_CTRL_IN, 1, 1, data, wLength, st)) return false;
      if (!stage(USBH_CH_CTRL_OUT, 0, 1, _dummy, 0, st)) return false;
    } else {
      // DATA OUT (first packet is DATA1), STATUS IN
      hhcd_USB_HOST.hc[USBH_CH_CTRL_OUT].toggle_out = 1;
      if (!stage(USBH_CH_CTRL_OUT, 0, 1, data, wLength, st)) return false;
      if (!stage(USBH_CH_CTRL_IN, 1, 1, _dummy, 0, st)) return false;
    }
  } else {
    // STATUS IN
    if (!stage(USBH_CH_CTRL_IN, 1, 1, _dummy, 0, st)) return false;
  }
  return true;
}

static void openCtrlChannels(uint8_t address, uint8_t speed, uint16_t mps)
{
  HAL_HCD_HC_Init(&hhcd_USB_HOST, USBH_CH_CTRL_OUT, 0x00, address, speed,
                  EP_TYPE_CTRL, mps);
  HAL_HCD_HC_Init(&hhcd_USB_HOST, USBH_CH_CTRL_IN, 0x80, address, speed,
                  EP_TYPE_CTRL, mps);
}

static void haltAllChannels()
{
  for (uint8_t ch = 0; ch <= USBH_CH_INTR_IN; ch++) {
    forceHalt(ch);
  }
}

/*
 * Enumeration
 */

// Find the first HID interface (non boot keyboard/mouse) with an interrupt
// IN endpoint in a configuration descriptor.
static bool findHidInterface(const uint8_t* cfg, uint16_t len,
                             HidEndpoint& out)
{
  memset(&out, 0, sizeof(out));
  bool inHid = false;
  bool haveEp = false;
  uint16_t pos = 0;

  while (pos + 2 <= len) {
    uint8_t bLength = cfg[pos];
    uint8_t bType = cfg[pos + 1];
    if (bLength < 2 || pos + bLength > len) break;

    if (bType == USB_DESC_INTERFACE && bLength >= 9) {
      if (haveEp) break;  // previous HID interface is complete
      uint8_t bClass = cfg[pos + 5];
      uint8_t bProto = cfg[pos + 7];
      inHid = (bClass == USB_CLASS_HID && bProto == 0);
      if (inHid) {
        out.iface = cfg[pos + 2];
        out.report_desc_len = 0;
      }
    } else if (inHid && bType == USB_DESC_HID && bLength >= 9) {
      // bNumDescriptors at [5], then (bDescriptorType, wDescriptorLength)
      for (uint8_t i = 6; i + 3 <= bLength; i += 3) {
        if (cfg[pos + i] == USB_DESC_HID_REPORT) {
          out.report_desc_len = cfg[pos + i + 1] | (cfg[pos + i + 2] << 8);
          break;
        }
      }
    } else if (inHid && bType == USB_DESC_ENDPOINT && bLength >= 7) {
      uint8_t addr = cfg[pos + 2];
      uint8_t attr = cfg[pos + 3] & 0x03;
      if ((addr & 0x80) && attr == EP_TYPE_INTR && !haveEp) {
        out.ep_addr = addr;
        out.mps = cfg[pos + 4] | (cfg[pos + 5] << 8);
        out.interval = cfg[pos + 6];
        haveEp = true;
      }
    }
    pos += bLength;
  }

  return haveEp && out.report_desc_len > 0 && out.mps > 0;
}

static bool enumerate(HidEndpoint& hep)
{
  HCD_URBStateTypeDef st;

  uint8_t speed = HAL_HCD_GetCurrentSpeed(&hhcd_USB_HOST);
  _info.speed = speed;
  uint16_t mps0 = (speed == HCD_DEVICE_SPEED_LOW) ? 8 : 64;
  openCtrlChannels(0, speed, mps0);

  // Device descriptor, first 8 bytes to learn bMaxPacketSize0
  if (!ctrlRequest(0x80, USB_REQ_GET_DESCRIPTOR, USB_DESC_DEVICE << 8, 0,
                   _cfgDesc, 8, &st)) {
    TRACE("USBH: GET_DESCRIPTOR(device, 8) failed (%d)", st);
    return false;
  }
  if (_cfgDesc[7] == 0) return false;
  mps0 = _cfgDesc[7];
  openCtrlChannels(0, speed, mps0);

  if (!ctrlRequest(0x80, USB_REQ_GET_DESCRIPTOR, USB_DESC_DEVICE << 8, 0,
                   _cfgDesc, 18, &st)) {
    TRACE("USBH: GET_DESCRIPTOR(device) failed (%d)", st);
    return false;
  }
  _info.vid = _cfgDesc[8] | (_cfgDesc[9] << 8);
  _info.pid = _cfgDesc[10] | (_cfgDesc[11] << 8);
  TRACE("USBH: device %04X:%04X, %s speed, mps0=%d", _info.vid, _info.pid,
        speed == HCD_DEVICE_SPEED_LOW ? "low" : "full", mps0);

  // Address
  if (!ctrlRequest(0x00, USB_REQ_SET_ADDRESS, USBH_DEVICE_ADDRESS, 0, nullptr,
                   0, &st)) {
    TRACE("USBH: SET_ADDRESS failed (%d)", st);
    return false;
  }
  sleep_ms(2);
  openCtrlChannels(USBH_DEVICE_ADDRESS, speed, mps0);

  // Configuration descriptor
  if (!ctrlRequest(0x80, USB_REQ_GET_DESCRIPTOR, USB_DESC_CONFIGURATION << 8,
                   0, _cfgDesc, 9, &st)) {
    TRACE("USBH: GET_DESCRIPTOR(config, 9) failed (%d)", st);
    return false;
  }
  uint16_t total = _cfgDesc[2] | (_cfgDesc[3] << 8);
  if (total < 9) return false;
  if (total > USBH_CFG_DESC_MAX) total = USBH_CFG_DESC_MAX;
  if (!ctrlRequest(0x80, USB_REQ_GET_DESCRIPTOR, USB_DESC_CONFIGURATION << 8,
                   0, _cfgDesc, total, &st)) {
    TRACE("USBH: GET_DESCRIPTOR(config) failed (%d)", st);
    return false;
  }
  uint8_t configValue = _cfgDesc[5];

  if (!findHidInterface(_cfgDesc, total, hep)) {
    TRACE("USBH: no HID interface with interrupt IN endpoint");
    _status = USBH_JOYSTICK_NOT_SUPPORTED;
    return false;
  }
  TRACE("USBH: HID iface %d ep 0x%02X mps %d interval %d report desc %d",
        hep.iface, hep.ep_addr, hep.mps, hep.interval, hep.report_desc_len);

  if (!ctrlRequest(0x00, USB_REQ_SET_CONFIGURATION, configValue, 0, nullptr,
                   0, &st)) {
    TRACE("USBH: SET_CONFIGURATION failed (%d)", st);
    return false;
  }

  // Ask for continuous reports (some devices only report on change
  // otherwise). Not all devices support it; a STALL is harmless.
  if (!ctrlRequest(0x21, HID_REQ_SET_IDLE, 0, hep.iface, nullptr, 0, &st)) {
    TRACE("USBH: SET_IDLE not accepted (%d)", st);
    if (st == URB_ERROR) return false;
  }

  // HID report descriptor
  uint16_t rlen = hep.report_desc_len;
  if (rlen > USBH_REPORT_DESC_MAX) rlen = USBH_REPORT_DESC_MAX;
  if (!ctrlRequest(0x81, USB_REQ_GET_DESCRIPTOR, USB_DESC_HID_REPORT << 8,
                   hep.iface, _reportDesc, rlen, &st)) {
    TRACE("USBH: GET_DESCRIPTOR(report) failed (%d)", st);
    return false;
  }
  uint32_t got = HAL_HCD_HC_GetXferCount(&hhcd_USB_HOST, USBH_CH_CTRL_IN);
  if (got < rlen) rlen = got;

  hid::ParseResult pr = hid::parse(_reportDesc, rlen, _parsed);
  TRACE("USBH: report descriptor %d bytes, %d input fields, parse: %s",
        (int)rlen, (int)_parsed.field_count, hid::parseResultName(pr));
  if (pr != hid::ParseResult::Ok && pr != hid::ParseResult::TooManyFields) {
    _status = USBH_JOYSTICK_NOT_SUPPORTED;
    return false;
  }
  if (!_decoder.configure(_parsed)) {
    TRACE("USBH: no axes/buttons/hat usable as a joystick");
    _status = USBH_JOYSTICK_NOT_SUPPORTED;
    return false;
  }

  _info.axes = _decoder.axisCount();
  _info.buttons = _decoder.buttonCount();
  _info.hat = _decoder.hatBinding().present ? 1 : 0;
  uint8_t channels = usbhJoystickTrainerChannels(_decoder);
  _info.channels =
      channels > MAX_TRAINER_CHANNELS ? MAX_TRAINER_CHANNELS : channels;
  TRACE("USBH: %d axes, %d buttons, hat %d -> %d trainer channels",
        _info.axes, _info.buttons, _info.hat, _info.channels);

  // Interrupt IN pipe
  HAL_HCD_HC_Init(&hhcd_USB_HOST, USBH_CH_INTR_IN, hep.ep_addr,
                  USBH_DEVICE_ADDRESS, speed, EP_TYPE_INTR, hep.mps);
  hhcd_USB_HOST.hc[USBH_CH_INTR_IN].toggle_in = 0;

  _state.clear();
  return true;
}

/*
 * Report polling
 */

static void pollReports(const HidEndpoint& hep)
{
  uint8_t interval = hep.interval;
  if (interval < 2) interval = 2;
  if (interval > 32) interval = 32;
  _info.interval = interval;

  uint16_t mps = hep.mps;
  if (mps > USBH_REPORT_MAX) mps = USBH_REPORT_MAX;

  uint8_t errors = 0;
  bool gotReport = false;
  HCD_URBStateTypeDef st;

  while (_enabled && _connected) {
    // A NAKed interrupt transfer leaves the channel halted, but make sure
    // nothing is in flight before re-submitting.
    if (channelEnabled(USBH_CH_INTR_IN)) forceHalt(USBH_CH_INTR_IN);

    if (!submit(USBH_CH_INTR_IN, 1, EP_TYPE_INTR, 1, _report, mps)) {
      errors++;
    } else {
      sleep_ms(interval);
      st = urbState(USBH_CH_INTR_IN);

      if (st == URB_DONE) {
        uint32_t n = HAL_HCD_HC_GetXferCount(&hhcd_USB_HOST, USBH_CH_INTR_IN);
        if (n > 0) {
          if (_decoder.decode(_report, n, _state)) {
            usbhJoystickToTrainer(_state, _decoder, trainerInput,
                                  MAX_TRAINER_CHANNELS);
          }
          _info.reports++;
          gotReport = true;
        }
        errors = 0;
        trainerResetTimer();
      } else if (st == URB_STALL) {
        TRACE("USBH: interrupt endpoint stalled, clearing");
        ctrlRequest(0x02, USB_REQ_CLEAR_FEATURE, 0, hep.ep_addr, nullptr, 0,
                    &st);
        hhcd_USB_HOST.hc[USBH_CH_INTR_IN].toggle_in = 0;
        errors++;
      } else if (st == URB_ERROR) {
        errors++;
      } else {
        // NAK: nothing new, the device is still there
        if (gotReport) trainerResetTimer();
      }
    }

    if (errors > USBH_MAX_POLL_ERRORS) {
      TRACE("USBH: too many transfer errors");
      _status = USBH_JOYSTICK_ERROR;
      return;
    }
  }
}

/*
 * Task
 */

static void usbhTask()
{
  for (;;) {
    if (!_enabled) {
      if (_coreActive) coreDeinit();
      _status = USBH_JOYSTICK_OFF;
      sleep_ms(50);
      continue;
    }

    if (!_coreActive) {
      if (usbStarted()) {
        // the device stack owns the core (PC connected): try again later
        _status = USBH_JOYSTICK_USB_BUSY;
        sleep_ms(200);
        continue;
      }
      coreInit();
    }

    if (!_connected) {
      _status = USBH_JOYSTICK_WAIT_DEVICE;
      updateDiagnostics();
      sleep_ms(20);
      continue;
    }

    // Device attached
    _status = USBH_JOYSTICK_ENUMERATING;
    uint16_t irqs = _info.irqs;
    memset(&_info, 0, sizeof(_info));
    _info.irqs = irqs;
    updateDiagnostics();
    TRACE("USBH: device attached");
    sleep_ms(200);  // debounce, let the device power up

    bool ok = false;
    for (int attempt = 0; attempt < 3 && _connected && _enabled; attempt++) {
      _portEnabled = false;
      HAL_HCD_ResetPort(&hhcd_USB_HOST);
      uint32_t start = timersGetMsTick();
      while (!_portEnabled && _connected &&
             timersGetMsTick() - start < USBH_RESET_TIMEOUT_MS) {
        sleep_ms(1);
      }
      if (_portEnabled) {
        ok = true;
        break;
      }
      TRACE("USBH: port reset attempt %d failed", attempt + 1);
    }

    HidEndpoint hep;
    if (ok) {
      sleep_ms(100);
      ok = enumerate(hep);
    }

    if (ok) {
      _status = USBH_JOYSTICK_READY;
      TRACE("USBH: joystick ready");
      pollReports(hep);
    } else if (_status == USBH_JOYSTICK_ENUMERATING) {
      _status = USBH_JOYSTICK_ERROR;
    }

    haltAllChannels();
    updateDiagnostics();
    if (_status == USBH_JOYSTICK_READY) _status = USBH_JOYSTICK_WAIT_DEVICE;
    TRACE("USBH: device released (status %d)", _status);

    if (_connected && _enabled) {
      // Failure with the device still plugged: wait a bit, then retry the
      // enumeration (or pick up the unplug)
      for (int i = 0; i < 40 && _connected && _enabled; i++) sleep_ms(50);
    }

    if (_coreActive && _enabled) {
      // re-arm the port after a disconnect
      HAL_HCD_Start(&hhcd_USB_HOST);
    }
  }
}

/*
 * API
 */

bool usbHostJoystickStart()
{
  if (_enabled) return true;
  if (usbStarted()) return false;

  if (!_taskCreated) {
    task_create(&usbhTaskId, usbhTask, "usbh", usbhStack, USBH_STACK_SIZE,
                USBH_TASK_PRIO);
    _taskCreated = true;
  }

  _enabled = true;
  return true;
}

void usbHostJoystickStop()
{
  _enabled = false;
}
