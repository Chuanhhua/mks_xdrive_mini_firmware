# MKS XDrive Mini BLDC Motor Controller

STM32F405-based firmware for torque-controlled BLDC motor with SimpleFOC, featuring voltage-mode control, cascade speed limiting, battery voltage monitoring, and CAN bus support.

## Hardware

**Board:** MKS XDrive Mini (STM32F405RG)

**Motor:** BLDC with 15:1 gearbox

**Sensor:** AS5047 (SPI magnetic encoder on SPI3)

**Power:**
- Input: 36V max (via battery connector)
- Output phase current limit: 7A
- Phase resistance: 3.5Ω per phase

**Pinout:**
- **Motor (M0):** INH_A/C/E PA8/PA9/PA10, INL_A/C/E PB13/PB14/PB15, EN_GATE PB12
- **SPI3 encoder:** CLK PC10, MISO PC11, MOSI PC12, CS PA15
- **Voltage sense:** PA6 (19× resistor divider from battery)
- **CAN:** RX PB8, TX PB9

## Software Setup

### Dependencies
- **SimpleFOC** (`askuric/Simple FOC`)
- **SimpleFOCDrivers** (from GitHub dev branch)
- **SimpleCANio** (CAN support)

### Build & Upload

```bash
# Build
pio run -e genericSTM32F405RG

# Upload (DFU mode, hold BOOT then press RST)
pio run -e genericSTM32F405RG --target upload

# Monitor
pio device monitor -b 230400
```

## Operation

### Serial Commands (115200 baud)

Send via Serial Monitor:

- **`T<Nm>`** — Set output torque in Nm (e.g., `T10`, `T21.7` for max)
  - Automatically converts to motor voltage: `V = (Nm / (ratio × eff)) / Kt × R_phase`
  - Clamped to 21.7 Nm (max with 7A @ 15:1 gearbox)
  - Cascade limit cuts torque at 45 rad/s motor speed (3 rad/s output)

- **`MMDS<n>`** — Enable/disable monitoring (0 = off, >0 = on with downsample)

- **`M<cmd>`** — Pass to SimpleFOC motor commander:
  - `P` — print motor status
  - `C<type>` — set controller (0=torque, 1=velocity, 2=angle, 3=voltage)
  - `T<type>` — set torque mode (0=voltage, 1=foc_current) — use 0 (voltage)

### CAN Commands

CAN register protocol at 1Mbps. Node ID 1 by default.

**CAN ID format:**
```
[27:20] = Node Address
[19:16] = Packet Type (1=read, 2=write, 3=response)
[15:8]  = Register (0xE0 for custom torque)
[7:0]   = Motor index
```

**Set output torque to 15 Nm:**
- Write 4-byte float (15.0) to register `0xE0`
- CAN ID: `(1 << 20) | (2 << 16) | (0xE0 << 8) | 0` = `0x1020E0`
- Payload: `0x00 0x00 0x70 0x41` (little-endian float 15.0)

**Standard motor registers:** (e.g., read angle, velocity, voltage)
- `0x09` = angle (float)
- `0x11` = velocity (float)
- `0x20` = Vq (float)
- See `SimpleFOCRegisters.h` for full list

### Monitoring & Diagnostics

On startup, prints:
- Supply voltage (from PA6 sensor)
- Max output torque
- "Ready." when FOC initialized

**Low battery warning:** If supply voltage < demanded voltage, prints warning (e.g., when 24V demand but only 20V available).

**Motion characteristics:**
- Smooth ramping in torque mode with cascade hysteresis
- Speed limit at 45 rad/s motor (3 rad/s output with gearbox)
- Cuts to 0V torque at limit, restores at 85% (27 rad/s motor = 1.8 rad/s output)

## Key Parameters

Edit in `src/main.cpp`:

```cpp
#define KT          0.23f              // motor Nm/A
#define R_PHASE     3.5f               // phase Ω
#define GEAR_RATIO  15.0f              // gearbox ratio
#define GEAR_EFF    0.9f               // gearbox efficiency
#define I_MAX       7.0f               // max phase current (A)
#define VEL_LIMIT   (3.0f * GEAR_RATIO)  // output rad/s → motor rad/s
```

## Architecture

- **20kHz FOC loop** on hardware timer TIM8 (no jitter from Serial/monitoring)
- **Voltage-mode torque control** (`TorqueControlType::voltage`) — reliable, no ADC sync issues
- **Dynamic supply voltage** updated every 1s from PA6 ADC (19× divider)
- **Cascade speed limiting** with hysteresis to prevent chatter
- **Serial Commander** for interactive `T<Nm>` commands
- **CAN Commander** for remote torque commands via register `0xE0`

## Troubleshooting

### Build Error: "undefined reference to `CANCommander::~CANCommander()`"
→ `platformio.ini` missing `-D has_SimpleCANio=1` flag. The `CANCommander.cpp` guards are checked before the header is included, so the macro must be defined globally.

**Fix:** Ensure `build_flags` includes:
```ini
-D has_SimpleCANio=1
```

### Motor Not Responding
1. Check SPI3 wiring (CS PA15, CLK/MISO/MOSI PC10/11/12)
2. Check encoder SPI communication: `M` → `P` in Serial Monitor
3. Verify gate enable pin PB12 is driven low (enabled)

### Jittery Motion
- Caused by inconsistent FOC timing (Serial/ADC interrupting loop). Already fixed with TIM8 hardware timer at 20kHz.
- If regressed: Check for new Serial.println() calls in the 20kHz ISR (they block on UART).

### High Current Spikes
- DRV8301 current sense: defaults to 10× gain (not 80×). Verify in hardware or reduce `I_MAX`.
- Phase resistance mismatch: Confirm 3.5Ω or recalibrate torque equation.

### Low Battery Warning Spamming
- Normal if demanding torque near supply voltage. Reduce `desired_voltage` or increase battery voltage.

## Hardware Timer (TIM8 @ 20kHz)

Configured in `setup()`. Calls `motor.loopFOC()` + `motor.move(torque_target)` at precise intervals. Do NOT call these in `loop()` — they run in interrupt context.

The `torque_target` volatile variable is the interface:
- **Set by:** Serial Commander (`T<Nm>`) or CAN register write (`0xE0`)
- **Read by:** TIM8 ISR every 50µs, subject to cascade hysteresis

## Future Extensions

- **Current feedback:** Add `LowsideCurrentSense` if ADC-PWM sync issues are resolved
- **Multi-motor CAN:** Add `commandc.addMotor()` for each motor, update CAN IDs
- **Velocity control:** Change `motor.controller = MotionControlType::velocity`, use `REG_TARGET` CAN register
- **Angle control:** Similar to velocity; add position encoder if needed