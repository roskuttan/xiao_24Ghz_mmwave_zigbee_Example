# XIAO 24GHz mmWave Zigbee Presence Sensor

Arduino firmware that publishes mmWave presence over Zigbee using a GPIO occupancy signal, with optional UART-based radar configuration and diagnostics for LD24xx-compatible modules.

## What This Firmware Does

- Exposes a Zigbee Occupancy Sensor endpoint (`endpoint 10`).
- Reads radar presence from a digital output pin (`D10`).
- Applies simple hysteresis/debounce (`3` consecutive samples) before state changes.
- Reports occupancy changes to Zigbee and sends serial heartbeat logs every 2 seconds.
- Supports factory reset by holding the boot button for more than 3 seconds.
- Optionally supports UART radar command/ack protocol for:
  - Reading current radar parameters (enabled by default in this sketch).
  - Writing a default radar profile (disabled by default in this sketch).

## Hardware

- Seeed XIAO board with Zigbee-capable ESP32 target (project intended for XIAO + Zigbee workflow).
- 24GHz mmWave radar module/shield with:
  - Digital OUT pin for occupancy signal.
  - UART pins for optional diagnostics/config.

## Pin Mapping

From `xiao_24Ghz_mmwave_zigbee_Example.ino`:

- Radar digital OUT -> `D10`
- UART RX -> `D2`
- UART TX -> `D3`
- Alternate UART RX -> `D7` (optional probe path)
- Alternate UART TX -> `D6` (optional probe path)
- Factory reset button -> `BOOT_PIN`

## Software Requirements

- Arduino IDE (or equivalent Arduino build environment).
- ESP32 Arduino core with Zigbee support (`Zigbee.h` used by this sketch).
- Zigbee mode must be configured as **End Device**.

The sketch contains this build guard:

```cpp
#ifndef ZIGBEE_MODE_ED
#error "Zigbee end device mode is not selected in Tools->Zigbee mode"
#endif
```

## Build and Flash

1. Open `xiao_24Ghz_mmwave_zigbee_Example.ino` in Arduino IDE.
2. Select the correct XIAO/ESP32 board target and serial port.
3. In Tools, set Zigbee mode to **End Device**.
4. Verify and upload the sketch.
5. Open Serial Monitor at `115200` baud to view logs.

## Zigbee Behavior

- Device registers `ZigbeeOccupancySensor` on endpoint `10`.
- Manufacturer/model reported as:
  - Manufacturer: `24Ghz presence sensor`
  - Model: `Xiao seed 24Ghz presence sensor`
- OTA client metadata is registered and an OTA check is requested after join.
- On boot:
  - Starts Zigbee stack.
  - Waits until joined/connected.
  - Reports initial occupancy state from `D10`.

## Runtime Logic

- Main loop polls radar OUT every `10 ms`.
- State change requires `HYSTERESIS_SAMPLES` consecutive disagreements.
- Presence updates are reported immediately after debounce confirmation.
- Heartbeat log prints every 2 seconds:
  - `[HB] PRESENCE` or `[HB] NO PRESENCE`

## Factory Reset

- Hold `BOOT_PIN` low for >3 seconds.
- Firmware triggers `Zigbee.factoryReset()`.

## UART Radar Diagnostics and Config

The sketch includes LD24xx-style command framing:

- Header: `FD FC FB FA`
- Tail: `04 03 02 01`

### Default UART Settings

- UART port: `Serial0`
- Baud: `256000`
- RX/TX: `D2/D3`
- ACK timeout: `700 ms`
- Retries: `2`

### Feature Flags

Current defaults in the sketch:

- `RADAR_UART_READBACK_ENABLE = 1`
- `RADAR_UART_WRITE_PROFILE_ENABLE = 0`
- `RADAR_UART_DIAGNOSTIC_MODE = 0`

Meaning:

- Parameter readback is enabled at boot when radar config mode is reachable.
- Profile writing is disabled unless you explicitly enable it.
- Advanced baud/pin/AT probing is disabled unless diagnostic mode is enabled.

### Optional Profile Write (Disabled by Default)

If you set `RADAR_UART_WRITE_PROFILE_ENABLE` to `1`, the sketch attempts to write:

- Max motion gate: `8`
- Max static gate: `8`
- No-person timeout: `5 s`
- Gate sensitivities from the hardcoded motion/static arrays in the sketch

## Important Compile-Time Constants

- `PRESENCE_ENDPOINT`: `10`
- `RADAR_OUT_PIN`: `D10`
- `LOOP_DELAY_MS`: `10`
- `HYSTERESIS_SAMPLES`: `3`
- `DEBUG_LOGS`: `1` (serial diagnostics enabled)

## Troubleshooting

- Build error about Zigbee mode:
  - Set Tools -> Zigbee mode to End Device.
- No occupancy updates:
  - Check radar OUT wiring to `D10`.
  - Confirm module output logic level matches board input expectations.
- UART config/readback warnings:
  - Device still continues in GPIO-only occupancy mode by design.
  - Verify UART RX/TX wiring and baud if you rely on UART diagnostics.
- Frequent resets or no Zigbee join:
  - Confirm board/partition/stack settings for your ESP32 Zigbee target.
  - Check power stability for radar + XIAO.

## File Layout

- `xiao_24Ghz_mmwave_zigbee_Example.ino` - main firmware.

## Notes

- This project currently has one source file and compile-time configuration.
- For production use, consider moving constants into a dedicated config header and adding release/version notes tied to OTA metadata.
