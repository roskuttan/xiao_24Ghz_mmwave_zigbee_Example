#include "ZigbeePresence.h"

#include "../common/DebugLog.h"
#include "../config/AppConfig.h"
#include "Zigbee.h"

static ZigbeeOccupancySensor zbPresence(PRESENCE_ENDPOINT);

bool zigbeePresenceBegin() {
  zbPresence.setManufacturerAndModel(ZB_MANUFACTURER, ZB_MODEL);
  zbPresence.addOTAClient(OTA_RUNNING_FILE_VERSION, OTA_DOWNLOADED_FILE_VERSION, OTA_HW_VERSION);
  Zigbee.addEndpoint(&zbPresence);

  if (!Zigbee.begin()) {
    return false;
  }

  while (!Zigbee.connected()) {
    DBG(".");
    delay(100);
  }

  DBGLN("\n[ZB] Connected");
  zbPresence.requestOTAUpdate();
  return true;
}

void zigbeePresenceReport(bool presence) {
  zbPresence.setOccupancy(presence);
  zbPresence.report();
  DBGLN("[ZB] presence=%s", presence ? "TRUE" : "FALSE");
}

void zigbeePresenceHandleFactoryResetButton() {
  if (digitalRead(BTN_PIN) == LOW) {
    delay(100);
    int t0 = millis();
    while (digitalRead(BTN_PIN) == LOW) {
      delay(50);
      if (millis() - t0 > 3000) {
        DBGLN("[SYS] Factory reset");
        delay(300);
        Zigbee.factoryReset();
      }
    }
  }
}

