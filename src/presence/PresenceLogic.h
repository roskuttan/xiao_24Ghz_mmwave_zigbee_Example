#pragma once

#include <Arduino.h>

struct PresenceLogicState {
  bool stable;
  bool debounced;
  bool assertDelayRequired;
  bool assertDelayArmed;
  uint8_t disagree;
  unsigned long assertDelayStartedAt;
  unsigned long offStartedAt;
  unsigned long nextHeartbeat;
};

void presenceLogicInit(PresenceLogicState* state, bool initialPresence);
bool presenceLogicUpdate(PresenceLogicState* state, bool pinPresence);
bool presenceLogicHeartbeatDue(PresenceLogicState* state, unsigned long now);
