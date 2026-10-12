# ESP32-WROOM-32 DevKitC — Pinout

Wiring reference for the Arduino sketch at `arduino_sketch/esp32_firmware/`. Cross-check against `pins.h` in that folder (source of truth) before connecting anything.

| GPIO | Function | Notes |
|---|---|---|
| 21 | I2C SDA | Shared bus: OLED (0x3C), BNO055 (0x29 — this breakout's ADR pin is tied high, confirmed by I2C scan), PCA9685 (0x40, also answers on 0x70 All Call — normal) |
| 22 | I2C SCL | Shared bus |
| 4 | PCA9685 `!OE` | Active LOW = outputs enabled. Firmware drives this HIGH whenever disarmed/failsafe, cutting all PWM at the hardware level. |
| 16 | SBUS RX (UART2) | From FS-A8S (FlySky) receiver's SBUS output pad — confirmed wired. Signal is inverted — firmware configures UART invert, do not add an external inverter. |
| 17 | SBUS TX (UART2) | Unused/not wired — SBUS is receive-only here, pin required by the API. |
| 0 | Status LED, red | ⚠️ Boot-strapping pin — must read HIGH at reset or the chip enters UART download mode instead of booting. If boot ever becomes flaky again, suspect this wire first. Safe once past boot. **Actively driven**: lit solid in AUTONOMOUS mode. |
| 2 | Status LED, green | ⚠️ Also a boot-strapping pin, milder risk than GPIO0. **Actively driven**: lit solid in MANUAL mode. Off in DISARMED/FAILSAFE. |
| 1 (TX0) / 3 (RX0) | USB | Reserved — do not reuse. Onboard USB-UART bridge, used as the plain-text serial link to the Orange Pi (see `CLAUDE.md` Architecture — not micro-ROS). |
| 14 | Gimbal stepper `EN` (shared) | Both TMC2208 EN pins tied together. **Not GPIO4** — that's `PIN_PCA9685_OE` above, the drivetrain's hardware kill switch, which the firmware actively drives based on arm state; wiring the stepper EN there too was confirmed as the actual cause of "works grounded, doesn't work on GPIO4." Active LOW (most breakouts) — see `GIMBAL_EN_ACTIVE_LOW` in `config.h`. |
| 18 | Gimbal pan `STEP` | |
| 19 | Gimbal pan `DIR` | |
| 27 | Gimbal tilt `STEP` | **Not GPIO17** — that's `PIN_SBUS_TX` below; the UART2 peripheral claims that pin the moment `sbus.begin()` runs, even though SBUS itself never transmits — a real conflict, not just a physical-wiring one. (GPIO23 was the first choice for this but isn't physically accessible on this board/harness.) |
| 5 | Gimbal tilt `DIR` | ⚠️ Also a (milder) boot-strapping pin (SDIO slave timing) — validated fine in bring-up, but if boot ever gets flaky, this is worth suspecting alongside GPIO0/2. |
| 26 | Right motor speed (DAC) | Genuine analog voltage output via `dacWrite()`, 1V-3.3V active range (0V = off) — see Drivetrain speed section below. |
| 25 | Left motor speed (DAC) | Same as above. **Not GPIO27** as originally requested — GPIO27 has no DAC hardware at all (only GPIO25/26 do, on this chip, full stop) and is also already the gimbal's tilt STEP pin above. |

## Drivetrain speed (analog DAC, not PWM)
The motor driver takes a genuine analog control voltage — **not** an RC-servo pulse, **not** PWM. Below ~1V nothing happens; the driver's usable range is 1V–3.3V, confirmed by the user's own testing (values above 3.3V also did nothing — expected, since 3.3V is the ESP32's own logic rail ceiling). The classic ESP32 has exactly two pins capable of a true analog voltage output — GPIO25 and GPIO26 — via the built-in `dacWrite(pin, 0-255)` (0=0V, 255=3.3V, linear). No PCA9685 channel can do this; PCA9685 only ever produces PWM. `DAC_MIN_VALUE`/`DAC_MAX_VALUE`/`DAC_OFF_VALUE` in `config.h` set the 1V floor / 3.3V ceiling / true-off mapping — "off" (0V) and "slowest commanded speed" (1V) are deliberately distinct values, not the same point.

## PCA9685 channels
| Channel | Function |
|---|---|
| 0–7 | Spare/unassigned (0/1 were briefly right/left motor speed under the old PWM-pulse scheme — retired, see above) |
| 8 (IN8) | Laser designator switch (relay) |
| 9 (IN7) | Tank LEDs (relay) |
| 10 (IN6) | Spare relay, unassigned |
| 11 (IN5) | Spare relay, unassigned |
| 12 (IN4) | Brake relay, motor 2 |
| 13 (IN3) | Brake relay, motor 1 |
| 14 (IN2) | Left motor — reverse relay (ON = reverse) |
| 15 (IN1) | Right motor — reverse relay (ON = reverse) |

**Drivetrain redesign**: motors are no longer dual-ESC (forward channel + backward channel each), and speed is no longer PCA9685-PWM-based at all (see Drivetrain speed section above) — only direction (reverse-select relays) and braking still go through the relay board (IN1–IN8 → PCA channels 15 down to 8 — **this descending order is inferred from how the wiring was described, not independently verified**, confirm it matches reality). Relay channels are driven via the PCA9685's full-on/full-off override bits (see `relays.h`), not real PWM. `RELAY_ACTIVE_LOW` in `config.h` assumes the common cheap relay-board convention (LOW energizes) — **not verified against this specific board**, flip it if every relay comes out backwards.

## Aiming gimbal (differential stepper pan/tilt)
2× NEMA17 + TMC2208 v1.2, STEP/DIR standalone mode, direct ESP32 GPIO (not PCA9685) — see pin table above and `gimbal.h`/`gimbal.cpp`. Requires **MS1=HIGH, MS2=LOW** wired on both TMC2208 boards for the 32 microsteps `config.h`'s math assumes — confirm this before trusting any speed numbers.

Mixing model, confirmed on real hardware via the standalone bring-up sketch (`esp32_firmware/arduino_sketch/differential_gimbal_test/`): `tilt = motorA − motorB`, `pan = motorB` (independent of motorA). The firmware drives this as **rate control** (stick deflection → angular velocity), not absolute position — open-loop steppers have no position feedback, so there's no "go to angle X" without homing hardware that doesn't exist yet. Now includes acceleration ramping (`GIMBAL_ACCEL_STEPS_S2`) and soft position limits in raw steps (`GIMBAL_PAN_LIMIT_STEPS`/`GIMBAL_TILT_LIMIT_STEPS`, config.h) — **limits are in steps, not degrees, since the gear/differential ratio isn't measured yet**; recalibrate both once it's known. `GIMBAL_MAX_STEP_RATE` (2000 steps/sec) is an untuned starting guess carried over from an unrelated bring-up test, not validated against this mechanism's real torque/inertia — watch for stalling and lower it if needed.

Control input reuses the **steer/throttle sticks** as pan/tilt direction, gated by SwC (see the 3-position switch note below): steer→pan, throttle→tilt. VarA/VarB are **speed dials**, not direction — each maps 0..1 to scale `GIMBAL_MAX_STEP_RATE` independently for pan/tilt, so the operator can dial in how fast the gimbal moves live. **This is Claude's interpretation of the requested scheme, not independently confirmed** — correct `esp32_firmware.ino`/`config.h` if it's not what's wanted.

**Autonomous aim+fire hook**: when the mode switch (SwB) is in autonomous AND SwC is in an aim position (middle OR full - both work the same here), pan/tilt direction AND the laser relay both come from a new serial command, `A,<pan>,<tilt>,<fire>\n` (Pi → ESP32, pan/tilt normalized -1..1, fire is 0 or 1, same staleness-timeout convention as `C,<linear>,<angular>`), instead of the sticks/switch. Intended workflow: drive the whole track in manual, then at the shooting area flip SwC to middle and SwB to autonomous - the vision system takes full aim+fire authority at that point, since nobody's holding a stick to aim anymore. Fire fails safe (off) if the command goes stale. **No Pi-side sender exists yet** — this is purely the ESP32-side hook for the vision-based autonomous targeting described in `CLAUDE.md`'s vision strategy, which hasn't been built.

Outside that specific state (manual mode, or autonomous mode without SwC in an aim position), the laser still only fires on SwC's full position via direct operator control - that's unchanged and only applies to non-autonomous-aim situations. Autonomous driving is also blocked while SwC is in an aim position (mirrors manual).

**Not yet validated on the actual robot hardware** — folded into the main firmware in code, but nobody has confirmed the physical gimbal responds correctly to stick input yet. Confirm in person before trusting it.

## Free GPIOs
Not used by this firmware, available for whatever else you have planned: 13, 23, 32, 33, and input-only 34, 35, 36, 39.

**Avoided on purpose:** GPIO 6–11 (internal flash — do not use), GPIO 12/15 (boot strapping pins, unused). GPIO 0/2 are also strapping pins but are in active use for the status LED — see above.

## SBUS channel assignment (`config.h`)
Matches the FlySky i6 10-channel mod: CH1–4 = sticks, CH5–10 = aux (VarA, VarB, SwA–SwD).

| Channel (0-indexed) | Transmitter control | Function |
|---|---|---|
| 0 | Right stick, horizontal / Aileron | Steer normally, pan direction in aim mode (SwC) |
| 1 | Left stick, vertical / Throttle | Throttle normally, tilt direction in aim mode (SwC) |
| 4 | CH5, VarA | Pan speed dial (0..1, scales `GIMBAL_MAX_STEP_RATE`) |
| 5 | CH6, VarB | Tilt speed dial (0..1, scales `GIMBAL_MAX_STEP_RATE`) |
| 6 | CH7, SwA | Arm switch: low = disarmed, high = armed |
| 7 | CH8, SwB | Mode switch: low = manual, high = autonomous |
| 8 | CH9, SwC | **3-position** aim switch: off = drive normally; middle = aim mode (drive forced to zero, CH0/CH1 become pan/tilt); full = aim mode + laser relay on |
| 9 | CH10, SwD | Brake switch: engages both brake relays, forces drive to zero regardless of SwC — aim mode still works while braked |

SwC's 3 positions are read via `SBUS_3POS_LOW_THRESHOLD`/`SBUS_3POS_HIGH_THRESHOLD` (config.h), splitting the raw range into thirds — if your transmitter's 3-position switch doesn't sit cleanly in those thirds, adjust the thresholds. Update `SBUS_CH_*` in `config.h` if your switch/channel assignment on the transmitter differs.

## Arduino IDE setup
1. **Boards Manager**: install "esp32 by Espressif Systems"
2. **Library Manager**, install:
   - `Adafruit BNO055`
   - `Adafruit Unified Sensor`
   - `Adafruit SSD1306`
   - `Adafruit GFX Library`
   - `Adafruit PWM Servo Driver Library`
3. Open `arduino_sketch/esp32_firmware/esp32_firmware.ino`
4. Tools → Board → **ESP32 Dev Module**, Tools → Port → your ESP32's COM port
5. Upload

## Before first flash
- [ ] Confirm none of the pins above are already used by your physical wiring
- [ ] Confirm the motor driver's actual usable voltage range matches `DAC_MIN_VALUE`/`DAC_MAX_VALUE` in `config.h` (currently set for a 1V-3.3V range per the user's own testing) - measure with a multimeter on GPIO25/26 if unsure
- [ ] PCA9685 `!OE` wiring is optional but strongly recommended — without it, firmware can still zero the PWM signal but cannot force outputs off at the hardware level
- [ ] Confirm the relay board's IN1–IN8 → PCA channel 15-down-to-8 mapping actually matches your wiring
- [ ] Confirm `RELAY_ACTIVE_LOW` in `config.h` matches your relay board (most cheap boards: LOW = energized)
- [ ] Confirm MS1=HIGH/MS2=LOW is wired on both TMC2208 drivers (32 microsteps)
- [ ] Confirm SwC actually reads as 3 distinct positions against `SBUS_3POS_LOW_THRESHOLD`/`SBUS_3POS_HIGH_THRESHOLD` in `config.h`

## Note on the `esp32_firmware/` PlatformIO project (parent folder) and `micro_ros_arduino`
Both are superseded/unused. The project moved off micro-ROS entirely (2026-08-24) after the agent never completed a session with the ESP32 despite correct wiring/protocol framing — see `CLAUDE.md` Open Decisions. Active firmware here uses a plain text serial protocol instead (documented in `esp32_firmware.ino`'s header comment).
