#include "PresenceLogic.h"

#include "../config/AppConfig.h"

void presenceLogicInit(PresenceLogicState* state, bool initialPresence) {
  if (state == nullptr) {
    return;
  }

  const unsigned long now = millis();
  state->debounced = initialPresence;
  state->assertDelayRequired = (PRESENCE_ASSERT_DELAY_MS > 0);
  state->assertDelayArmed = false;
  state->disagree = 0;
  state->assertDelayStartedAt = 0;
  state->offStartedAt = initialPresence ? 0 : now;
  state->nextHeartbeat = 0;

  if (initialPresence && state->assertDelayRequired) {
    state->stable = false;
    state->assertDelayArmed = true;
    state->assertDelayStartedAt = now;
    return;
  }

  state->stable = initialPresence;
}

bool presenceLogicUpdate(PresenceLogicState* state, bool pinPresence) {
  if (state == nullptr) {
    return false;
  }

  const unsigned long now = millis();

  if (pinPresence == state->debounced) {
    state->disagree = 0;
  } else if (++state->disagree >= HYSTERESIS_SAMPLES) {
    state->debounced = pinPresence;
    state->disagree = 0;
  }

  if (state->stable) {
    if (!state->debounced) {
      state->stable = false;
      state->assertDelayArmed = false;
      state->offStartedAt = now;
      return true;
    }
    return false;
  }

  if (!state->debounced) {
    state->assertDelayArmed = false;
    if (!state->assertDelayRequired &&
        PRESENCE_ASSERT_DELAY_MS > 0 &&
        (uint32_t)(now - state->offStartedAt) >= PRESENCE_ASSERT_REARM_OFF_MS) {
      state->assertDelayRequired = true;
    }
    return false;
  }

  if (!state->assertDelayRequired || PRESENCE_ASSERT_DELAY_MS == 0) {
    state->stable = true;
    state->offStartedAt = 0;
    state->assertDelayArmed = false;
    state->assertDelayRequired = false;
    return true;
  }

  if (!state->assertDelayArmed) {
    state->assertDelayArmed = true;
    state->assertDelayStartedAt = now;
    return false;
  }

  if ((uint32_t)(now - state->assertDelayStartedAt) >= PRESENCE_ASSERT_DELAY_MS) {
    state->stable = true;
    state->offStartedAt = 0;
    state->assertDelayArmed = false;
    state->assertDelayRequired = false;
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
