#pragma once

#include <Arduino.h>

struct PresenceLogicState {
  bool stable;
  uint8_t disagree;
  unsigned long nextHeartbeat;
};

void presenceLogicInit(PresenceLogicState* state, bool initialPresence);
bool presenceLogicUpdate(PresenceLogicState* state, bool pinPresence);
bool presenceLogicHeartbeatDue(PresenceLogicState* state, unsigned long now);

