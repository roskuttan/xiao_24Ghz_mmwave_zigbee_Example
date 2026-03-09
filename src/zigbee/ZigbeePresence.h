#pragma once

#include <Arduino.h>

bool zigbeePresenceBegin();
void zigbeePresenceReport(bool presence);
void zigbeePresenceHandleFactoryResetButton();

