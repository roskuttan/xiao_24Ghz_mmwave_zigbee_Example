#include "PresenceLogic.h"

#include "../config/AppConfig.h"

void presenceLogicInit(PresenceLogicState* state, bool initialPresence) {
  if (state == nullptr) {
    return;
  }

  state->stable = initialPresence;
  state->disagree = 0;
  state->nextHeartbeat = 0;
}

bool presenceLogicUpdate(PresenceLogicState* state, bool pinPresence) {
  if (state == nullptr) {
    return false;
  }

  if (pinPresence == state->stable) {
    state->disagree = 0;
    return false;
  }

  if (++state->disagree >= HYSTERESIS_SAMPLES) {
    state->stable = pinPresence;
    state->disagree = 0;
    return true;
  }

  return false;
}

bool presenceLogicHeartbeatDue(PresenceLogicState* state, unsigned long now) {
  if (state == nullptr) {
    return false;
  }

  if (now >= state->nextHeartbeat) {
    state->nextHeartbeat = now + HEARTBEAT_INTERVAL_MS;
    return true;
  }

  return false;
}

