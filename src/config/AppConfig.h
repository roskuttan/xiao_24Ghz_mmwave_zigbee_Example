#pragma once

#include <Arduino.h>

#ifndef ZIGBEE_MODE_ED
#error "Zigbee end device mode is not selected in Tools->Zigbee mode"
#endif

// Zigbee endpoint ID exposed as the occupancy sensor.
#define PRESENCE_ENDPOINT               10
// Local button used for long-press factory reset.
#define BTN_PIN                         BOOT_PIN
// Radar digital occupancy output pin (HIGH = motion/presence).
#define RADAR_OUT_PIN                   D10
// Main loop delay in milliseconds. Lower = faster response, more CPU usage.
#define LOOP_DELAY_MS                   10
// Debounce count for any GPIO state change (applies to ON and OFF transitions).
#define HYSTERESIS_SAMPLES              3
// Delay before reporting OFF->ON when delayed assert is armed (milliseconds).
#define PRESENCE_ASSERT_DELAY_MS        3000UL
// Continuous OFF time required to rearm delayed OFF->ON assert (milliseconds).
#define PRESENCE_ASSERT_REARM_OFF_MS    120000UL
// Serial heartbeat print interval in milliseconds.
#define HEARTBEAT_INTERVAL_MS           2000UL
// USB serial baud rate for debug logs.
#define DEBUG_BAUD_RATE                 115200UL

// UART radar configuration/readback (LD24xx-compatible command protocol).
// 1 = read and log radar parameters at boot.
#define RADAR_UART_READBACK_ENABLE      1
// 1 = write default profile values to radar at boot.
#define RADAR_UART_WRITE_PROFILE_ENABLE 0
// UART baud expected from the radar module.
#define RADAR_UART_BAUD                 256000UL
// Max wait time for command ACK frame (milliseconds).
#define RADAR_UART_ACK_TIMEOUT_MS       700UL
// Number of attempts when entering radar config mode.
#define RADAR_UART_ENABLE_RETRIES       2
// Delay between config-mode retries (milliseconds).
#define RADAR_UART_ENABLE_RETRY_DELAY_MS 80UL
// Extra wait after UART open to let radar boot (milliseconds).
#define RADAR_UART_BOOT_SETTLE_MS       250UL
// 1 = enable extra probing paths/logs (baud/pin/AT/fallback).
#define RADAR_UART_DIAGNOSTIC_MODE      0
#if RADAR_UART_DIAGNOSTIC_MODE
  // Passive listen window after UART open to detect any incoming bytes.
  #define RADAR_UART_PASSIVE_SNIFF_MS   140UL
  // Probe common alternate baud rates when primary baud fails.
  #define RADAR_UART_BAUD_PROBE_ENABLE  1
  // Probe alternate RX/TX pin pairs when primary pins fail.
  #define RADAR_UART_PIN_PROBE_ENABLE   1
  // Send "+++" probe for modules using AT-text protocol.
  #define RADAR_UART_AT_PROBE_ENABLE    1
  // Try version ping ACK path when config command is rejected.
  #define RADAR_UART_FALLBACK_PING_ENABLE 1
#else
  // Diagnostic probing disabled in normal mode.
  #define RADAR_UART_PASSIVE_SNIFF_MS   0UL
  #define RADAR_UART_BAUD_PROBE_ENABLE  0
  #define RADAR_UART_PIN_PROBE_ENABLE   0
  #define RADAR_UART_AT_PROBE_ENABLE    0
  #define RADAR_UART_FALLBACK_PING_ENABLE 0
#endif

// Primary radar UART RX pin (MCU RX <- radar TX).
#define RADAR_UART_RX_PIN               D2
// Primary radar UART TX pin (MCU TX -> radar RX).
#define RADAR_UART_TX_PIN               D3
// Alternate RX pin used only when pin probing is enabled.
#define RADAR_UART_ALT_RX_PIN           D7
// Alternate TX pin used only when pin probing is enabled.
#define RADAR_UART_ALT_TX_PIN           D6
// Hardware serial port bound to radar UART.
#define RADAR_UART_PORT                 Serial0

// Optional write-profile defaults (used only when RADAR_UART_WRITE_PROFILE_ENABLE=1).
// Farthest motion gate used by radar algorithm (0..8).
#define RADAR_CFG_MAX_MOTION_GATE       8
// Farthest static gate used by radar algorithm (0..8).
#define RADAR_CFG_MAX_STATIC_GATE       8
// Radar "no person" timeout in seconds.
#define RADAR_CFG_NO_PERSON_SEC         5

// Per-gate motion sensitivity for gates 0..8 (index = gate number).
static const uint8_t RADAR_CFG_MOTION_SENS[9] = {50, 50, 40, 30, 20, 15, 15, 15, 15};
// Per-gate static sensitivity for gates 0..8 (index = gate number).
static const uint8_t RADAR_CFG_STATIC_SENS[9] = {100, 100, 40, 40, 30, 30, 20, 20, 20};

// OTA metadata versions advertised to Zigbee OTA cluster.
#define OTA_RUNNING_FILE_VERSION        0x01010100
#define OTA_DOWNLOADED_FILE_VERSION     0x01010101
#define OTA_HW_VERSION                  0x0101

// Zigbee Basic cluster identity strings.
#define ZB_MANUFACTURER                 "24Ghz presence sensor"
#define ZB_MODEL                        "Xiao seed 24Ghz presence sensor"
