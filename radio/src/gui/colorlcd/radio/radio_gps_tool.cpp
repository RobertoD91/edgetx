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

#include "radio_gps_tool.h"

#include "edgetx.h"

RadioGpsTool::RadioGpsTool() :
    Page(ICON_RADIO_TOOLS)
{
  init();
  buildHeader(header);
  buildBody(body);
}

void RadioGpsTool::buildHeader(Window* window)
{
  header->setTitle(STR_MENUTOOLS);
  header->setTitle2(STR_GPS_MODEL_LOCATOR);
}

void RadioGpsTool::buildBody(Window* window)
{
  window->padAll(PAD_ZERO);
  gpsLabel = new StaticText(window, {PAD_LARGE, PAD_LARGE, LV_SIZE_CONTENT, 0}, "", COLOR_THEME_PRIMARY1_INDEX, FONT(L));
  gpsQR = new QRCode(window, (window->width() - QR_SZ) / 2, (window->height() - QR_SZ) / 2, QR_SZ, "");
  new TextButton(window, 
                {window->width() - BTN_SZ - PAD_LARGE * 2, window->height() - EdgeTxStyles::UI_ELEMENT_HEIGHT - PAD_LARGE * 2, BTN_SZ, 0},
                STR_REFRESH, [=]() {
                  refresh();
                  return 0;
                });
  refresh();
}

void RadioGpsTool::init()
{
  gpsSensorID = -1;
  for (int i = 0; i < MAX_TELEMETRY_SENSORS; i++) {
    if (isGPSSensor(i+1)) {
      gpsSensorID = i;
      return;
    }
  }
}

// Format a coordinate stored in micro-degrees as a decimal string.
// Integer arithmetic keeps the full 6 decimals (a float only has ~7
// significant digits) and avoids depending on printf float support.
static void formatCoord(char* buf, size_t len, int32_t value)
{
  div_t d = div((int)value, 1000000);
  snprintf(buf, len, "%s%d.%06d", value < 0 ? "-" : "", abs(d.quot), abs(d.rem));
}

void RadioGpsTool::refresh()
{
  if (gpsSensorID >= 0) {
    TelemetryItem& gpsItem = telemetryItems[gpsSensorID];
    char lat[16], lon[16], gps_uri[48];
    formatCoord(lat, sizeof(lat), gpsItem.gps.latitude);
    formatCoord(lon, sizeof(lon), gpsItem.gps.longitude);
    // RFC 5870 geo URI, understood by both iOS and Android
    snprintf(gps_uri, sizeof(gps_uri), "geo:%s,%s", lat, lon);
    gpsQR->setData(gps_uri);
    gpsQR->show();
    gpsLabel->setText(getGPSSensorValue(gpsItem, 0));
  } else {
    gpsQR->hide();
    gpsLabel->setText(STR_NODATA);
  }
}
