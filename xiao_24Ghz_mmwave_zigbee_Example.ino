#include <Arduino.h>
#include <stdarg.h>
#include <string.h>

#ifndef DEBUG_LOGS
#define DEBUG_LOGS 1
#endif
#if DEBUG_LOGS
  static inline void DBG_BEGIN(unsigned long b) { Serial.begin(b); }
  static inline void DBG(const char* fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    char buf[256];
    vsnprintf(buf, sizeof(buf), fmt, ap);
    Serial.print(buf);
    va_end(ap);
  }
  #define DBGLN(...) do { DBG(__VA_ARGS__); Serial.println(); } while (0)
#else
  #define DBG_BEGIN(...)  ((void)0)
  #define DBG(...)        ((void)0)
  #define DBGLN(...)      ((void)0)
#endif

#ifndef ZIGBEE_MODE_ED
#error "Zigbee end device mode is not selected in Tools->Zigbee mode"
#endif

#include "Zigbee.h"

#define PRESENCE_ENDPOINT               10
#define BTN_PIN                         BOOT_PIN
#define RADAR_OUT_PIN                   D10    // mmWave shield OUT -> D10
#define LOOP_DELAY_MS                   10
#define HYSTERESIS_SAMPLES              3      // debounce for GPIO occupancy truth
#define PRESENCE_ASSERT_DELAY_MS        3000UL // valid presence requires sustained HIGH for this long; set 0 for immediate

// UART radar configuration/readback (LD24xx-compatible command protocol).
#define RADAR_UART_READBACK_ENABLE      1
#define RADAR_UART_WRITE_PROFILE_ENABLE 0
#define RADAR_UART_BAUD                 256000UL
#define RADAR_UART_ACK_TIMEOUT_MS       700UL
#define RADAR_UART_ENABLE_RETRIES       2
#define RADAR_UART_ENABLE_RETRY_DELAY_MS 80UL
#define RADAR_UART_BOOT_SETTLE_MS       250UL
#define RADAR_UART_DIAGNOSTIC_MODE      0
#if RADAR_UART_DIAGNOSTIC_MODE
  #define RADAR_UART_PASSIVE_SNIFF_MS   140UL
  #define RADAR_UART_BAUD_PROBE_ENABLE  1
  #define RADAR_UART_PIN_PROBE_ENABLE   1
  #define RADAR_UART_AT_PROBE_ENABLE    1
  #define RADAR_UART_FALLBACK_PING_ENABLE 1
#else
  #define RADAR_UART_PASSIVE_SNIFF_MS   0UL
  #define RADAR_UART_BAUD_PROBE_ENABLE  0
  #define RADAR_UART_PIN_PROBE_ENABLE   0
  #define RADAR_UART_AT_PROBE_ENABLE    0
  #define RADAR_UART_FALLBACK_PING_ENABLE 0
#endif
#define RADAR_UART_RX_PIN               D2
#define RADAR_UART_TX_PIN               D3
#define RADAR_UART_ALT_RX_PIN           D7
#define RADAR_UART_ALT_TX_PIN           D6
#define RADAR_UART_PORT                 Serial0

struct RadarUartPins {
  int rx;
  int tx;
};

#if RADAR_UART_BAUD_PROBE_ENABLE
static const uint32_t RADAR_UART_BAUD_CANDIDATES[] = {
  RADAR_UART_BAUD, 115200UL, 230400UL, 460800UL, 9600UL, 19200UL, 38400UL, 57600UL
};
#else
static const uint32_t RADAR_UART_BAUD_CANDIDATES[] = {
  RADAR_UART_BAUD
};
#endif

#if RADAR_UART_PIN_PROBE_ENABLE
static const RadarUartPins RADAR_UART_PIN_CANDIDATES[] = {
  {RADAR_UART_RX_PIN, RADAR_UART_TX_PIN},
  {RADAR_UART_TX_PIN, RADAR_UART_RX_PIN},
  {RADAR_UART_ALT_RX_PIN, RADAR_UART_ALT_TX_PIN},
  {RADAR_UART_ALT_TX_PIN, RADAR_UART_ALT_RX_PIN},
};
#else
static const RadarUartPins RADAR_UART_PIN_CANDIDATES[] = {
  {RADAR_UART_RX_PIN, RADAR_UART_TX_PIN},
};
#endif

// Optional write-profile defaults (used only when RADAR_UART_WRITE_PROFILE_ENABLE=1).
#define RADAR_CFG_MAX_MOTION_GATE       8
#define RADAR_CFG_MAX_STATIC_GATE       8
#define RADAR_CFG_NO_PERSON_SEC         5

// Gate 0..8 defaults from Seeed docs.
static const uint8_t RADAR_CFG_MOTION_SENS[9] = {50, 50, 40, 30, 20, 15, 15, 15, 15};
static const uint8_t RADAR_CFG_STATIC_SENS[9] = {100, 100, 40, 40, 30, 30, 20, 20, 20};

// OTA versions
#define OTA_RUNNING_FILE_VERSION        0x01010100
#define OTA_DOWNLOADED_FILE_VERSION     0x01010101
#define OTA_HW_VERSION                  0x0101

static HardwareSerial& radarUart = RADAR_UART_PORT;
static int radarActiveRxPin = RADAR_UART_RX_PIN;
static int radarActiveTxPin = RADAR_UART_TX_PIN;
static bool radarSawIncomingBytes = false;
static bool radarConfigSessionOpened = false;
static bool radarAtProtocolDetected = false;
static bool radarNeedsBootSettleDelay = true;

static const uint8_t RADAR_CMD_HEADER[4] = {0xFD, 0xFC, 0xFB, 0xFA};
static const uint8_t RADAR_CMD_TAIL[4]   = {0x04, 0x03, 0x02, 0x01};
static const size_t RADAR_MAX_ACK_PAYLOAD = 128;
static const size_t RADAR_MAX_CMD_PAYLOAD = 40;

static void radarFlushInput();
static bool radarReadBytes(uint8_t* dst, size_t len, uint32_t timeoutMs);
static bool radarReadAckFrame(uint32_t timeoutMs);
static bool radarSendCommandFrame(uint16_t commandWord, const uint8_t* commandValue, uint16_t valueLen);
static bool radarSendCommandWithAck(uint16_t commandWord, const uint8_t* commandValue, uint16_t valueLen);
static bool radarEnterConfigMode();
static bool radarExitConfigMode();
static bool radarReadParameters();
static void radarLogParameters();
static bool radarWriteMaxDistanceAndNoPerson(uint8_t maxMotionGate, uint8_t maxStaticGate, uint32_t noPersonDurationSec);
static bool radarWriteGateSensitivity(uint16_t gate, uint8_t motionSensitivity, uint8_t staticSensitivity);
static bool radarApplyWriteProfile();
static bool radarProbeATMode();
static bool radarTryEnterConfigAtBaud(uint32_t baud);
static void radarInitAndSyncConfig();

struct RadarAckFrame {
  uint16_t commandWord;
  uint16_t payloadLen;
  uint8_t payload[RADAR_MAX_ACK_PAYLOAD];
};

struct RadarParams {
  uint8_t maxGateN;
  uint8_t configuredMaxMotionGate;
  uint8_t configuredMaxStaticGate;
  uint8_t motionSensitivity[9];
  uint8_t staticSensitivity[9];
  uint16_t noPersonDurationSec;
};

static RadarAckFrame radarAckScratch = {};
static RadarParams radarParamsCache = {};

ZigbeeOccupancySensor zbPresence(PRESENCE_ENDPOINT);

static bool presenceDebounced = false;
static bool presenceStable = false;
static uint8_t debounceDisagree = 0;
static bool presenceAssertDelayArmed = false;
static unsigned long presenceAssertDelayStartedAt = 0;
static unsigned long nextHeartbeat = 0;

static inline uint16_t readLE16(const uint8_t* p) {
  return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static inline void writeLE16(uint8_t* p, uint16_t v) {
  p[0] = (uint8_t)(v & 0xFF);
  p[1] = (uint8_t)((v >> 8) & 0xFF);
}

static inline void writeLE32(uint8_t* p, uint32_t v) {
  p[0] = (uint8_t)(v & 0xFF);
  p[1] = (uint8_t)((v >> 8) & 0xFF);
  p[2] = (uint8_t)((v >> 16) & 0xFF);
  p[3] = (uint8_t)((v >> 24) & 0xFF);
}

static inline void reportPresence(bool p) {
  zbPresence.setOccupancy(p);
  zbPresence.report();
  DBGLN("[ZB] presence=%s", p ? "TRUE" : "FALSE");
}

static inline void handleFactoryReset() {
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

static void radarFlushInput() {
  while (radarUart.available() > 0) {
    (void)radarUart.read();
  }
}

static bool radarReadBytes(uint8_t* dst, size_t len, uint32_t timeoutMs) {
  size_t readCount = 0;
  uint32_t lastByteAt = millis();

  while (readCount < len) {
    int c = radarUart.read();
    if (c >= 0) {
      dst[readCount++] = (uint8_t)c;
      lastByteAt = millis();
      continue;
    }
    if ((uint32_t)(millis() - lastByteAt) >= timeoutMs) {
      return false;
    }
    delay(1);
  }
  return true;
}

static bool radarReadAckFrame(uint32_t timeoutMs) {
  RadarAckFrame& frame = radarAckScratch;
  uint8_t matched = 0;
  uint32_t startAt = millis();

  while ((uint32_t)(millis() - startAt) < timeoutMs) {
    int c = radarUart.read();
    if (c < 0) {
      delay(1);
      continue;
    }

    uint8_t b = (uint8_t)c;
    if (b == RADAR_CMD_HEADER[matched]) {
      matched++;
      if (matched == sizeof(RADAR_CMD_HEADER)) {
        break;
      }
    } else {
      matched = (b == RADAR_CMD_HEADER[0]) ? 1 : 0;
    }
  }

  if (matched != sizeof(RADAR_CMD_HEADER)) {
    return false;
  }

  uint8_t lenBytes[2];
  if (!radarReadBytes(lenBytes, sizeof(lenBytes), timeoutMs)) {
    return false;
  }

  const uint16_t frameDataLen = readLE16(lenBytes);
  if (frameDataLen < 2) {
    return false;
  }

  const uint16_t payloadLen = frameDataLen - 2;
  if (payloadLen > RADAR_MAX_ACK_PAYLOAD) {
    DBGLN("[RADAR][ERR] ACK payload too large: %u", payloadLen);
    return false;
  }

  uint8_t frameData[2 + RADAR_MAX_ACK_PAYLOAD];
  if (!radarReadBytes(frameData, frameDataLen, timeoutMs)) {
    return false;
  }

  uint8_t tail[4];
  if (!radarReadBytes(tail, sizeof(tail), timeoutMs)) {
    return false;
  }

  if (memcmp(tail, RADAR_CMD_TAIL, sizeof(RADAR_CMD_TAIL)) != 0) {
    DBGLN("[RADAR][ERR] ACK tail mismatch");
    return false;
  }

  frame.commandWord = readLE16(frameData);
  frame.payloadLen = payloadLen;
  if (payloadLen > 0) {
    memcpy(frame.payload, frameData + 2, payloadLen);
  }
  return true;
}

static bool radarSendCommandFrame(uint16_t commandWord, const uint8_t* commandValue, uint16_t valueLen) {
  if (valueLen > RADAR_MAX_CMD_PAYLOAD) {
    DBGLN("[RADAR][ERR] TX payload too large: %u", valueLen);
    return false;
  }

  uint8_t frame[4 + 2 + 2 + RADAR_MAX_CMD_PAYLOAD + 4];
  size_t idx = 0;

  memcpy(frame + idx, RADAR_CMD_HEADER, sizeof(RADAR_CMD_HEADER));
  idx += sizeof(RADAR_CMD_HEADER);

  writeLE16(frame + idx, (uint16_t)(2 + valueLen));
  idx += 2;

  writeLE16(frame + idx, commandWord);
  idx += 2;

  if (valueLen > 0) {
    memcpy(frame + idx, commandValue, valueLen);
    idx += valueLen;
  }

  memcpy(frame + idx, RADAR_CMD_TAIL, sizeof(RADAR_CMD_TAIL));
  idx += sizeof(RADAR_CMD_TAIL);

  size_t written = radarUart.write(frame, idx);
  radarUart.flush();
  return written == idx;
}

static bool radarSendCommandWithAck(uint16_t commandWord, const uint8_t* commandValue, uint16_t valueLen) {
  if (!radarSendCommandFrame(commandWord, commandValue, valueLen)) {
    DBGLN("[RADAR][ERR] command TX failed: 0x%04X", commandWord);
    return false;
  }

  const uint16_t expectedAckWord = commandWord | 0x0100;
  const uint32_t startedAt = millis();

  while ((uint32_t)(millis() - startedAt) < RADAR_UART_ACK_TIMEOUT_MS) {
    const uint32_t elapsed = (uint32_t)(millis() - startedAt);
    const uint32_t remaining = (RADAR_UART_ACK_TIMEOUT_MS > elapsed) ? (RADAR_UART_ACK_TIMEOUT_MS - elapsed) : 1;

    if (!radarReadAckFrame(remaining)) {
      continue;
    }

    if (radarAckScratch.commandWord != expectedAckWord) {
      DBGLN("[RADAR][WARN] unexpected ACK cmd=0x%04X expected=0x%04X", radarAckScratch.commandWord, expectedAckWord);
      continue;
    }

    if (radarAckScratch.payloadLen < 2) {
      DBGLN("[RADAR][ERR] ACK too short for cmd=0x%04X", commandWord);
      return false;
    }

    const uint16_t ackStatus = readLE16(radarAckScratch.payload);
    if (ackStatus != 0) {
      DBGLN("[RADAR][ERR] ACK status=%u for cmd=0x%04X", ackStatus, commandWord);
      return false;
    }

    return true;
  }

  DBGLN("[RADAR][ERR] ACK timeout for cmd=0x%04X", commandWord);
  return false;
}

static bool radarEnterConfigMode() {
  const uint8_t value[2] = {0x01, 0x00};
  if (!radarSendCommandWithAck(0x00FF, value, sizeof(value))) {
    return false;
  }

  if (radarAckScratch.payloadLen >= 6) {
    const uint16_t protoVersion = readLE16(radarAckScratch.payload + 2);
    const uint16_t txBufferSize = readLE16(radarAckScratch.payload + 4);
    DBGLN("[RADAR] config enabled (protocol=0x%04X, buffer=%u)", protoVersion, txBufferSize);
  } else {
    DBGLN("[RADAR] config enabled");
  }
  return true;
}

static bool radarExitConfigMode() {
  if (!radarSendCommandWithAck(0x00FE, nullptr, 0)) {
    return false;
  }
  DBGLN("[RADAR] config closed");
  return true;
}

static bool radarReadParameters() {
  if (!radarSendCommandWithAck(0x0061, nullptr, 0)) {
    return false;
  }

  RadarParams& params = radarParamsCache;
  const RadarAckFrame& ack = radarAckScratch;

  memset(&params, 0, sizeof(params));
  memset(params.motionSensitivity, 0xFF, sizeof(params.motionSensitivity));
  memset(params.staticSensitivity, 0xFF, sizeof(params.staticSensitivity));

  if (ack.payloadLen < 8) {
    DBGLN("[RADAR][ERR] read-param ACK too short: %u", ack.payloadLen);
    return false;
  }

  size_t idx = 2;  // skip 2-byte ACK status
  const uint8_t marker = ack.payload[idx++];
  params.maxGateN = ack.payload[idx++];
  params.configuredMaxMotionGate = ack.payload[idx++];
  params.configuredMaxStaticGate = ack.payload[idx++];

  if (marker != 0xAA) {
    DBGLN("[RADAR][WARN] read-param marker mismatch: 0x%02X", marker);
  }

  const uint8_t gateCount = (params.maxGateN > 8) ? 8 : params.maxGateN;

  for (uint8_t gate = 0; gate <= gateCount; gate++) {
    if (idx >= ack.payloadLen) {
      DBGLN("[RADAR][ERR] read-param truncated (motion gate %u)", gate);
      return false;
    }
    params.motionSensitivity[gate] = ack.payload[idx++];
  }

  for (uint8_t gate = 0; gate <= gateCount; gate++) {
    if (idx >= ack.payloadLen) {
      DBGLN("[RADAR][ERR] read-param truncated (static gate %u)", gate);
      return false;
    }
    params.staticSensitivity[gate] = ack.payload[idx++];
  }

  if ((idx + 1) >= ack.payloadLen) {
    DBGLN("[RADAR][ERR] read-param missing no-person duration");
    return false;
  }
  params.noPersonDurationSec = readLE16(ack.payload + idx);
  return true;
}

static void radarLogParameters() {
  const RadarParams& params = radarParamsCache;
  const uint8_t gateCount = (params.maxGateN > 8) ? 8 : params.maxGateN;

  DBGLN("[RADAR] maxGateN=%u configuredMotionGate=%u configuredStaticGate=%u noPerson=%us",
        params.maxGateN,
        params.configuredMaxMotionGate,
        params.configuredMaxStaticGate,
        params.noPersonDurationSec);

  for (uint8_t gate = 0; gate <= gateCount; gate++) {
    DBGLN("[RADAR] gate=%u motion=%u static=%u",
          gate,
          params.motionSensitivity[gate],
          params.staticSensitivity[gate]);
  }
}

static bool radarWriteMaxDistanceAndNoPerson(uint8_t maxMotionGate, uint8_t maxStaticGate, uint32_t noPersonDurationSec) {
  uint8_t value[18] = {0};

  writeLE16(value + 0, 0x0000);
  writeLE32(value + 2, maxMotionGate);
  writeLE16(value + 6, 0x0001);
  writeLE32(value + 8, maxStaticGate);
  writeLE16(value + 12, 0x0002);
  writeLE32(value + 14, noPersonDurationSec);

  return radarSendCommandWithAck(0x0060, value, sizeof(value));
}

static bool radarWriteGateSensitivity(uint16_t gate, uint8_t motionSensitivity, uint8_t staticSensitivity) {
  uint8_t value[18] = {0};

  writeLE16(value + 0, 0x0000);
  writeLE32(value + 2, gate);
  writeLE16(value + 6, 0x0001);
  writeLE32(value + 8, motionSensitivity);
  writeLE16(value + 12, 0x0002);
  writeLE32(value + 14, staticSensitivity);

  return radarSendCommandWithAck(0x0064, value, sizeof(value));
}

static bool radarApplyWriteProfile() {
  bool allOk = true;

  if (!radarWriteMaxDistanceAndNoPerson(RADAR_CFG_MAX_MOTION_GATE, RADAR_CFG_MAX_STATIC_GATE, RADAR_CFG_NO_PERSON_SEC)) {
    DBGLN("[RADAR][ERR] write profile max-distance/no-person failed");
    allOk = false;
  }

  for (uint16_t gate = 0; gate <= 8; gate++) {
    // Datasheet marks static gate 0/1 as not settable; use 100 there to keep them effectively masked.
    const uint8_t staticValue = (gate < 2) ? 100 : RADAR_CFG_STATIC_SENS[gate];
    if (!radarWriteGateSensitivity(gate, RADAR_CFG_MOTION_SENS[gate], staticValue)) {
      DBGLN("[RADAR][ERR] write sensitivity failed at gate=%u", gate);
      allOk = false;
      break;
    }
  }

  if (allOk) {
    DBGLN("[RADAR] write profile applied");
  }
  return allOk;
}

static bool radarProbeATMode() {
  char atBuf[48] = {0};
  size_t idx = 0;
  const char* probe = "+++";
  radarUart.write((const uint8_t*)probe, 3);
  radarUart.flush();

  uint32_t startedAt = millis();
  while ((uint32_t)(millis() - startedAt) < 260UL && idx < (sizeof(atBuf) - 1)) {
    int c = radarUart.read();
    if (c >= 0) {
      atBuf[idx++] = (char)c;
      radarSawIncomingBytes = true;
    } else {
      delay(1);
    }
  }
  atBuf[idx] = '\0';

  if (idx > 0) {
    DBGLN("[RADAR] AT probe RX=\"%s\"", atBuf);
    if (strstr(atBuf, "OK") != nullptr || strstr(atBuf, "ok") != nullptr) {
      return true;
    }
  }
  return false;
}

static bool radarTryEnterConfigAtBaud(uint32_t baud) {
  DBGLN("[RADAR] UART open baud=%lu rx=%d tx=%d", baud, radarActiveRxPin, radarActiveTxPin);
  radarUart.end();
  delay(5);
  radarUart.begin(baud, SERIAL_8N1, radarActiveRxPin, radarActiveTxPin);
  delay(20);
  if (radarNeedsBootSettleDelay && RADAR_UART_BOOT_SETTLE_MS > 0) {
    delay(RADAR_UART_BOOT_SETTLE_MS);
    radarNeedsBootSettleDelay = false;
  }

  uint8_t sniffSample[8] = {0};
  uint8_t sniffSampleLen = 0;
  uint32_t sniffCount = 0;
  uint32_t sniffStartedAt = millis();
  while ((uint32_t)(millis() - sniffStartedAt) < RADAR_UART_PASSIVE_SNIFF_MS) {
    int c = radarUart.read();
    if (c >= 0) {
      if (sniffSampleLen < sizeof(sniffSample)) {
        sniffSample[sniffSampleLen++] = (uint8_t)c;
      }
      sniffCount++;
    } else {
      delay(1);
    }
  }
  if (sniffCount > 0) {
    radarSawIncomingBytes = true;
    char sampleHex[3 * sizeof(sniffSample) + 1] = {0};
    size_t pos = 0;
    for (uint8_t i = 0; i < sniffSampleLen && pos < (sizeof(sampleHex) - 1); ++i) {
      int w = snprintf(sampleHex + pos, sizeof(sampleHex) - pos, (i == 0) ? "%02X" : " %02X", sniffSample[i]);
      if (w <= 0) {
        break;
      }
      pos += (size_t)w;
      if (pos >= sizeof(sampleHex)) {
        break;
      }
    }
    DBGLN("[RADAR] passive RX bytes=%lu sample=%s", sniffCount, sampleHex);
  }

  for (uint8_t attempt = 1; attempt <= RADAR_UART_ENABLE_RETRIES; ++attempt) {
    radarFlushInput();
    if (radarEnterConfigMode()) {
      radarConfigSessionOpened = true;
      if (attempt > 1) {
        DBGLN("[RADAR] config enabled after retry=%u", attempt);
      }
      return true;
    }
    if (attempt < RADAR_UART_ENABLE_RETRIES) {
      delay(RADAR_UART_ENABLE_RETRY_DELAY_MS);
    }
  }

  #if RADAR_UART_FALLBACK_PING_ENABLE
  // Fallback ping: some firmware revisions respond to version query but reject config mode.
  radarFlushInput();
  if (radarSendCommandWithAck(0x00A0, nullptr, 0)) {
    radarSawIncomingBytes = true;
    DBGLN("[RADAR][WARN] UART link alive (0x00A0 ACK) but 0x00FF rejected/unsupported");
    return true;
  }
  #endif

  #if RADAR_UART_AT_PROBE_ENABLE
  radarFlushInput();
  if (radarProbeATMode()) {
    radarAtProtocolDetected = true;
    DBGLN("[RADAR][WARN] module responded to AT probe (\"+++\") but not binary ACK frames");
  }
  #endif
  return false;
}

static void radarInitAndSyncConfig() {
#if RADAR_UART_READBACK_ENABLE || RADAR_UART_WRITE_PROFILE_ENABLE
  radarSawIncomingBytes = false;
  radarConfigSessionOpened = false;
  radarAtProtocolDetected = false;
  radarNeedsBootSettleDelay = true;
  bool configEnabled = false;
  uint32_t activeBaud = RADAR_UART_BAUD;

  for (size_t p = 0; p < (sizeof(RADAR_UART_PIN_CANDIDATES) / sizeof(RADAR_UART_PIN_CANDIDATES[0])); ++p) {
    const RadarUartPins pins = RADAR_UART_PIN_CANDIDATES[p];
    bool duplicatePinPair = false;
    for (size_t q = 0; q < p; ++q) {
      if (RADAR_UART_PIN_CANDIDATES[q].rx == pins.rx && RADAR_UART_PIN_CANDIDATES[q].tx == pins.tx) {
        duplicatePinPair = true;
        break;
      }
    }
    if (duplicatePinPair) {
      continue;
    }
    if (p > 0) {
      DBGLN("[RADAR] probing alternate pins rx=%d tx=%d", pins.rx, pins.tx);
    }
    radarActiveRxPin = pins.rx;
    radarActiveTxPin = pins.tx;

    #if RADAR_UART_BAUD_PROBE_ENABLE
    for (size_t i = 0; i < (sizeof(RADAR_UART_BAUD_CANDIDATES) / sizeof(RADAR_UART_BAUD_CANDIDATES[0])); ++i) {
      const uint32_t baud = RADAR_UART_BAUD_CANDIDATES[i];
      bool duplicateBaud = false;
      for (size_t j = 0; j < i; ++j) {
        if (RADAR_UART_BAUD_CANDIDATES[j] == baud) {
          duplicateBaud = true;
          break;
        }
      }
      if (duplicateBaud) {
        continue;
      }
      if (i > 0) {
        DBGLN("[RADAR] probing alternate baud=%lu", baud);
      }
      if (radarTryEnterConfigAtBaud(baud)) {
        configEnabled = true;
        activeBaud = baud;
        break;
      }
    }
    #else
    configEnabled = radarTryEnterConfigAtBaud(RADAR_UART_BAUD);
    #endif

    if (configEnabled) {
      break;
    }
  }

  if (!configEnabled) {
    if (radarAtProtocolDetected) {
      DBGLN("[RADAR][WARN] detected AT-text protocol path; LD24xx binary config commands unavailable");
    }
    if (!radarSawIncomingBytes) {
      DBGLN("[RADAR][WARN] no UART RX bytes seen on any probe");
    }
    DBGLN("[RADAR][WARN] config mode unavailable; continuing GPIO-only occupancy");
    return;
  }

  if (radarActiveRxPin != RADAR_UART_RX_PIN || radarActiveTxPin != RADAR_UART_TX_PIN) {
    DBGLN("[RADAR][WARN] active pins rx=%d tx=%d differ from configured rx=%d tx=%d",
          radarActiveRxPin,
          radarActiveTxPin,
          RADAR_UART_RX_PIN,
          RADAR_UART_TX_PIN);
  }

  if (activeBaud != RADAR_UART_BAUD) {
    DBGLN("[RADAR][WARN] active baud=%lu differs from configured=%lu", activeBaud, RADAR_UART_BAUD);
  }

  bool operationsOk = true;

  #if RADAR_UART_WRITE_PROFILE_ENABLE
  operationsOk &= radarApplyWriteProfile();
  #endif

  #if RADAR_UART_READBACK_ENABLE
  if (radarReadParameters()) {
    radarLogParameters();
  } else {
    DBGLN("[RADAR][WARN] parameter readback failed");
    operationsOk = false;
  }
  #endif

  if (radarConfigSessionOpened) {
    if (!radarExitConfigMode()) {
      DBGLN("[RADAR][WARN] failed to close config mode cleanly");
      operationsOk = false;
    }
  } else {
    DBGLN("[RADAR][WARN] skipping config close because config mode was never entered");
  }

  if (!operationsOk) {
    DBGLN("[RADAR][WARN] UART config had errors; GPIO occupancy remains active");
  }
#else
  DBGLN("[RADAR] UART config/readback disabled");
#endif
}

void setup() {
  pinMode(BTN_PIN, INPUT_PULLUP);
  pinMode(RADAR_OUT_PIN, INPUT_PULLDOWN);
  DBG_BEGIN(115200);
  DBGLN("\n[Roskuttan] Presence boot (GPIO occupancy + UART radar diagnostics)");

  radarInitAndSyncConfig();

  zbPresence.setManufacturerAndModel("24Ghz presence sensor", "Xiao seed 24Ghz presence sensor");
  zbPresence.addOTAClient(OTA_RUNNING_FILE_VERSION, OTA_DOWNLOADED_FILE_VERSION, OTA_HW_VERSION);
  Zigbee.addEndpoint(&zbPresence);

  if (!Zigbee.begin()) {
    DBGLN("[ZB][ERR] start fail");
    delay(300);
    ESP.restart();
  }

  while (!Zigbee.connected()) {
    DBG(".");
    delay(100);
  }

  DBGLN("\n[ZB] Connected");
  zbPresence.requestOTAUpdate();

  presenceDebounced = (digitalRead(RADAR_OUT_PIN) == HIGH);
  presenceStable = presenceDebounced && (PRESENCE_ASSERT_DELAY_MS == 0);
  if (presenceDebounced && PRESENCE_ASSERT_DELAY_MS > 0) {
    presenceAssertDelayArmed = true;
    presenceAssertDelayStartedAt = millis();
  }
  reportPresence(presenceStable);
}

void loop() {
  bool pinPresence = (digitalRead(RADAR_OUT_PIN) == HIGH);

  if (pinPresence == presenceDebounced) {
    debounceDisagree = 0;
  } else if (++debounceDisagree >= HYSTERESIS_SAMPLES) {
    presenceDebounced = pinPresence;
    debounceDisagree = 0;
  }

  if (presenceStable) {
    if (!presenceDebounced) {
      presenceStable = false;
      presenceAssertDelayArmed = false;
      reportPresence(false);
    }
  } else {
    if (!presenceDebounced) {
      presenceAssertDelayArmed = false;
    } else if (PRESENCE_ASSERT_DELAY_MS == 0) {
      presenceStable = true;
      reportPresence(true);
    } else if (!presenceAssertDelayArmed) {
      presenceAssertDelayArmed = true;
      presenceAssertDelayStartedAt = millis();
    } else if ((uint32_t)(millis() - presenceAssertDelayStartedAt) >= PRESENCE_ASSERT_DELAY_MS) {
      presenceStable = true;
      presenceAssertDelayArmed = false;
      reportPresence(true);
    }
  }

  if (millis() >= nextHeartbeat) {
    nextHeartbeat = millis() + 2000;
    DBGLN("[HB] %s", presenceStable ? "PRESENCE" : "NO PRESENCE");
  }

  handleFactoryReset();
  delay(LOOP_DELAY_MS);
}
