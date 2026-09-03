# Functional Specification Document
## ESP32 DC Motor PID Position Controller (ESP_AS5600)

---

## 1. Overview

This firmware implements a closed-loop PID position controller for a DC motor on a Waveshare ESP32-C3-Zero. A magnetic rotary encoder (AS5600) provides angle feedback and an L293D H-bridge drives the motor. The user sets a target angle via the serial console, which is also where real-time status is reported.

---

## 2. Hardware

| Component | Part | Interface | GPIO |
|-----------|------|-----------|------|
| MCU | Waveshare ESP32-C3-Zero (ESP-IDF, `CONFIG_IDF_TARGET=esp32c3`) | — | — |
| Rotary encoder | AS5600 | I2C (addr 0x36) | SDA=8, SCL=9 |
| Motor driver | L293D | LEDC PWM | IN1=6, IN2=7 |

The AS5600 is the only device on the I2C bus. The LEDC peripheral generates 5 kHz PWM with 10-bit resolution on **both** motor direction inputs.

**L293D EN is hardwired to logic 1** — it is not driven by the MCU, so the bridge is permanently enabled and speed is modulated on IN1/IN2 instead (see §4.2).

---

## 3. Software Architecture

The firmware is structured as an ESP-IDF component-based project with two reusable driver components and a top-level application.

```
MotorWithAS5600/
├── main/
│   └── main.c            # Application: init, PID task, console task
└── components/
    ├── as5600/           # AS5600 magnetic encoder driver
    └── l293d/            # L293D H-bridge motor driver
```

### 3.1 Tasks

| Task | Priority | Stack | Function |
|------|----------|-------|----------|
| `pid` | 5 | 4096 B | PID control loop + console status output |
| `console` | 3 | 4096 B | Serial input parsing |

A single FreeRTOS mutex (`g_target_mutex`) protects the shared `g_target_deg` float between the two tasks.

---

## 4. Component Specifications

### 4.1 AS5600 Driver (`components/as5600/`)

**Purpose:** Read angle and magnet status from the AS5600 over I2C.

**Registers used:**

| Register | Address | Description |
|----------|---------|-------------|
| STATUS | 0x0B | Magnet detection flags |
| RAW_ANGLE | 0x0C–0x0D | 12-bit unfiltered angle |
| ANGLE | 0x0E–0x0F | 12-bit filtered angle |

**API:**

| Function | Description |
|----------|-------------|
| `as5600_init(bus, dev)` | Add device to I2C bus and probe for presence |
| `as5600_read(dev, data)` | Read STATUS, RAW_ANGLE, and ANGLE registers |

**Output fields (`as5600_data_t`):**

| Field | Type | Description |
|-------|------|-------------|
| `raw_angle` | `uint16_t` | 12-bit unfiltered angle (0–4095) |
| `angle` | `uint16_t` | 12-bit filtered angle (0–4095) |
| `degrees` | `float` | Filtered angle in degrees: `angle × 360 / 4096` |
| `magnet_detected` | `bool` | STATUS bit 5: magnet in range |
| `magnet_too_weak` | `bool` | STATUS bit 4: magnet too far/weak |
| `magnet_too_strong` | `bool` | STATUS bit 3: magnet too close/strong |

**Conversion:** `degrees = angle × 360.0 / 4096.0`, giving 0.0°–359.91° over a full rotation.

---

### 4.2 L293D Driver (`components/l293d/`)

**Purpose:** Control motor direction and speed via LEDC PWM on the two direction inputs.

**PWM configuration:** 5 kHz, 10-bit resolution (duty range 0–1023), LEDC low-speed mode. One shared timer (`LEDC_TIMER_0`) drives two channels — `LEDC_CHANNEL_0` on IN1 and `LEDC_CHANNEL_1` on IN2.

**Drive scheme — sign-magnitude.** Because EN is tied high the bridge cannot be disabled, so the active direction input carries the PWM while the other is held at zero duty:

| Speed | IN1 duty | IN2 duty |
|-------|----------|----------|
| > 0 (forward) | `speed × 1023 / 100` | 0 |
| < 0 (reverse) | 0 | `|speed| × 1023 / 100` |
| 0 (coast stop) | 0 | 0 |
| brake | 1023 | 1023 |

**API:**

| Function | Description |
|----------|-------------|
| `l293d_init(cfg, dev)` | Configure the LEDC timer and one channel per direction input |
| `l293d_set_speed(dev, speed)` | Set speed −100 to +100; 0 coasts to stop |
| `l293d_brake(dev)` | Active brake: both inputs at full duty |
| `l293d_stop(dev)` | Coast stop: both inputs at zero duty |

**Clamping:** Speed values are clamped to [−100, +100] inside `l293d_set_speed`.

---

## 5. PID Controller

### 5.1 Parameters

| Parameter | Value | Description |
|-----------|-------|-------------|
| `PID_PERIOD_MS` | 10 ms | Control loop period (100 Hz) |
| `KP` | 0.8 | Proportional gain |
| `KI` | 0.3 | Integral gain |
| `KD` | 0.05 | Derivative gain |
| `INTEGRAL_LIMIT` | ±50.0 | Anti-windup clamp on integrator |
| `DEADBAND_DEG` | 1.0° | Error window where motor is stopped |
| `MIN_SPEED` | 40% | Minimum PWM when outside deadband (stiction override) |
| `STATUS_PERIOD_MS` | 500 ms | Console status line interval (2 Hz) |

### 5.2 Algorithm

1. **Angle read:** `as5600_read` fetches the current position each period.
2. **Error:** `error = wrap_error(target − measured)`, wrapped to [−180°, +180°] to correctly handle the 0°/360° boundary.
3. **Integral:** Accumulated with anti-windup clamping to `±INTEGRAL_LIMIT`.
4. **Derivative:** Computed on measurement, not error (`−(meas − prev_meas) / dt`), to avoid derivative kick on setpoint changes.
5. **Output:** `output = Kp·e + Ki·∫e·dt + Kd·(−dmeas/dt)`, clamped to [−100, +100].
6. **Minimum drive:** If `|output| < MIN_SPEED` and `output ≠ 0`, it is raised to `±MIN_SPEED` to overcome motor stiction.
7. **Deadband:** If `|error| ≤ DEADBAND_DEG`, the motor is stopped and the integrator is reset.

### 5.3 Timing

The PID task uses `vTaskDelayUntil` for fixed-period execution at 100 Hz. The console status line is emitted every 50 control ticks (2 Hz) — slow enough that it does not bury the echo of a target angle being typed at the prompt.

---

## 6. Console Interface

- **Transport:** UART via `getchar` / `putchar` (standard ESP-IDF UART stdio).
- **Input:** User types a floating-point angle in degrees, terminated by `\n` or `\r`.
- **Parsing:** `sscanf` with `%f`. Invalid input prints an error message.
- **Normalisation:** Input is reduced modulo 360° and mapped to [0°, 360°). Negative values are wrapped positive.
- **Output:** Echoes each typed character; on Enter, prints `Target -> <value> deg`.
- **Thread safety:** Target angle is written under `g_target_mutex`.

---

## 7. Status Output

The `pid` task prints one status line to the console every 500 ms:

```
Pos: <NNN.NN> deg  Tgt: <NNN.NN> deg  Err: <+NNN.NN> deg  Spd: <+NNN>%
```

| Field | Meaning |
|-------|---------|
| `Pos` | Current measured angle |
| `Tgt` | Current target angle |
| `Err` | Signed error, wrapped to [−180°, +180°] |
| `Spd` | Motor speed command (−100 … +100) |

Status output shares the UART with the console prompt (§6); the two are interleaved rather than separated.

---

## 8. Initialization Sequence

1. Configure I2C master bus (GPIO 8/9, internal pull-ups enabled, glitch filter 7 cycles).
2. Initialize AS5600 — add to bus, probe.
3. Initialize L293D — configure the LEDC timer and both direction-input channels.
4. Create `g_target_mutex`.
5. Spawn `pid` task (priority 5) and `console` task (priority 3).

---

## 9. Error Handling

- All hardware init calls are wrapped in `ESP_ERROR_CHECK`; any failure halts the firmware with an error log.
- If `as5600_read` fails mid-loop, the PID task skips that iteration (`continue`) rather than acting on stale data.
- I2C communication uses a 100 ms timeout per transaction.
- Because EN is hardwired high, the motor cannot be disabled in software. A firmware halt (e.g. an `ESP_ERROR_CHECK` abort) leaves the L293D inputs in whatever state they last held, and the GPIOs revert to inputs on reset — the bridge coasts rather than latching a drive command, but there is no hardware kill path under MCU control.
