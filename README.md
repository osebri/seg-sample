# ESP32 Baby Monitoring / Safety Prototype (Phase 1)

This project is a **Phase-1 ESP-IDF starter firmware** for ESP32 DevKit V1.

Current scope is intentionally limited to:
- sensor and bus initialization,
- periodic sampling,
- structured serial logging,
- optional compact OLED summary.

No alerts, Wi-Fi notifications, app integration, or decision logic are implemented in this phase.

## Tech Stack

- ESP-IDF
- C
- FreeRTOS tasks
- VS Code + Espressif IDF extension
- Target: `esp32`

## Wiring (Exact Pin Map)

## Shared I2C bus
- `GPIO21` = SDA
- `GPIO22` = SCL

### MAX30102
- VCC -> 3.3V
- GND -> GND
- SDA -> GPIO21
- SCL -> GPIO22
- INT unused in phase 1

### MLX90614
- VCC -> 3.3V
- GND -> GND
- SDA -> GPIO21
- SCL -> GPIO22

### SSD1306 OLED 0.96" 128x64
- VCC -> 3.3V
- GND -> GND
- SDA -> GPIO21
- SCL -> GPIO22

### ADXL345 (optional)
- VCC -> 3.3V
- GND -> GND
- SDA -> GPIO21
- SCL -> GPIO22
- interrupt pins unused in phase 1

### DHT22 module (3-pin breakout)
- VCC -> 3.3V
- GND -> GND
- DATA -> GPIO4

### KY-037 sound sensor
- VCC -> 3.3V
- GND -> GND
- AO -> GPIO34
- DO unused

Practical note:
- KY-037 analog output is often **inverted** on LM393-based modules (louder sound can reduce AO voltage).
- The onboard potentiometer strongly affects both AO and DO behavior; tune it while observing logs.

### MQ-2 gas/smoke sensor
- VCC -> 5V
- GND -> GND
- AO -> GPIO35 (**through voltage divider**)
- DO unused

## Critical Hardware Safety Note (MQ-2)

MQ-2 modules are typically powered at 5V and many modules expose AO up to near VCC.

ESP32 ADC pins are **NOT 5V tolerant**.

**Do not connect MQ-2 AO directly to GPIO35 without scaling.**

Use a resistor divider (for example, 10k/20k or equivalent ratio) so AO stays at or below 3.3V at GPIO35.

This requirement is also documented in code comments (`main/sensor_mq2.c`).

## Project Structure

```
sample_project/
  CMakeLists.txt
  README.md
  main/
    CMakeLists.txt
    main.c
    i2c_manager.c
    adc_manager.c
    sensor_registry.c
    sensor_dht22.c
    sensor_mlx90614.c
    sensor_max30102.c
    sensor_ky037.c
    sensor_mq2.c
    sensor_adxl345.c
    display_ssd1306.c
    include/
      project_config.h
      i2c_manager.h
      adc_manager.h
      sensor_registry.h
      sensor_dht22.h
      sensor_mlx90614.h
      sensor_max30102.h
      sensor_ky037.h
      sensor_mq2.h
      sensor_adxl345.h
      display_ssd1306.h
```

## Runtime Architecture

Shared state is held in `sensor_registry`:
- latest values,
- per-sensor status flags,
- last error code,
- timestamps of last successful update.

I2C access is centralized in `i2c_manager` with a mutex to protect shared bus access from multiple tasks.

Default FreeRTOS task split:
- `task_dht22` (DHT22 polling, conservative interval)
- `task_analog` (KY-037 + MQ-2 ADC reads)
- `task_i2c` (MLX90614 + MAX30102 + ADXL345)
- `task_logger` (structured serial output)
- `task_oled` (optional SSD1306 summary)

## Boot Sequence

On startup firmware performs:
1. NVS init.
2. I2C init and I2C address scan.
3. ADC init for analog channels.
4. DHT22 init.
5. Per-sensor independent init (failures are logged, firmware continues).
6. Start periodic tasks.

## Serial Output Style

Example output format:
- `[DHT22] temp=24.3C hum=46.1%`
- `[MLX90614] object=33.8C ambient=25.0C`
- `[KY037] raw=1720`
- `[MQ2] raw=2380`
- `[MAX30102] red=xxxxx ir=yyyyy`
- `[ADXL345] x=... y=... z=...`
- `[HEARTBEAT] uptime=...s`

Missing/unhealthy sensors are reported as `NOT_INIT` or `ERR` with an error code.

## Build / Flash / Monitor

From project root:

1. Set target:
   - `idf.py set-target esp32`
2. Build:
   - `idf.py build`
3. Flash:
   - `idf.py -p <PORT> flash`
4. Monitor:
   - `idf.py -p <PORT> monitor`

In VS Code, equivalent commands can be run using the ESP-IDF extension commands/tasks.

## Driver/Implementation Assumptions

- DHT22: custom bit-banged timing implementation with retries, startup settle delay, protocol fallback timing, and DHT11-compatible decode fallback for mislabeled modules.
- MLX90614: SMBus register reads for ambient/object temperature (`0x06`, `0x07`).
- MAX30102: phase-1 basic init + FIFO raw read (Red/IR) only; no HR/SpO2 algorithm yet.
- ADXL345: optional; if not connected, firmware continues normally.
- SSD1306: optional compact summary renderer (small built-in font, no external graphics lib).

## Known Limitations (Phase 1)

- No alerting or decision logic.
- MAX30102 values are raw samples only.
- ADC readings are raw values (not gas concentration or calibrated SPL).
- KY-037 is better interpreted as a relative sound activity metric than a calibrated SPL value.
- DHT one-wire timing is software-based and can still fail if wiring/pull-up quality is poor; retries and automatic re-init are included.
- OLED summary is intentionally compact and minimal.
