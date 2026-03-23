#include "AppController.h"

#include <Arduino.h>

#include "../common/DebugLog.h"
#include "../config/AppConfig.h"
#include "../presence/PresenceLogic.h"
#include "../radar/RadarUart.h"
#include "../zigbee/ZigbeePresence.h"

static PresenceLogicState gPresenceState = {};

void appControllerSetup() {
  pinMode(BTN_PIN, INPUT_PULLUP);
  pinMode(RADAR_OUT_PIN, INPUT_PULLDOWN);

  DBG_BEGIN(DEBUG_BAUD_RATE);
  DBGLN("\n[Roskuttan] Presence boot (GPIO occupancy + UART radar diagnostics)");

  radarUartInitAndSyncConfig();

  if (!zigbeePresenceBegin()) {
    DBGLN("[ZB][ERR] start fail");
    delay(300);
    ESP.restart();
  }

  const bool initialPresence = (digitalRead(RADAR_OUT_PIN) == HIGH);
  presenceLogicInit(&gPresenceState, initialPresence);
  zigbeePresenceReport(gPresenceState.stable);
}

void appControllerLoop() {
  const bool pinPresence = (digitalRead(RADAR_OUT_PIN) == HIGH);

  if (presenceLogicUpdate(&gPresenceState, pinPresence)) {
    zigbeePresenceReport(gPresenceState.stable);
  }

  if (presenceLogicHeartbeatDue(&gPresenceState, millis())) {
    DBGLN("[HB] %s", gPresenceState.stable ? "PRESENCE" : "NO PRESENCE");
  }

  zigbeePresenceHandleFactoryResetButton();
  delay(LOOP_DELAY_MS);
}

