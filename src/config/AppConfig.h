#pragma once

#include <Arduino.h>

#ifndef ZIGBEE_MODE_ED
#error "Zigbee end device mode is not selected in Tools->Zigbee mode"
#endif

#define PRESENCE_ENDPOINT               10
#define BTN_PIN                         BOOT_PIN
#define RADAR_OUT_PIN                   D10
#define LOOP_DELAY_MS                   10
#define HYSTERESIS_SAMPLES              3
#define HEARTBEAT_INTERVAL_MS           2000UL
#define DEBUG_BAUD_RATE                 115200UL

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

#define ZB_MANUFACTURER                 "24Ghz presence sensor"
#define ZB_MODEL                        "Xiao seed 24Ghz presence sensor"

