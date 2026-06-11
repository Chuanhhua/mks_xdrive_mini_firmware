/*
 * MKS XDrive Mini — FOC Torque Control
 *
 * Control:
 *   CAN — write a 4-byte float [Nm] to register REG_CUSTOM_START (0xE0)
 *          read  the same register to get the current torque setpoint
 *
 * Torque path:
 *   target_torque_nm  (set by CAN)
 *       → ramped_torque_nm  (rate-limited at TORQUE_RAMP_NM_S)
 *           → Iq setpoint [A]  (= ramped_nm / (GEAR_RATIO * GEAR_EFF * KT))
 *               → foc_current PI  (runs at 20 kHz in hardware timer ISR)
 */

#include <SimpleFOC.h>
#include "SimpleFOCDrivers.h"
#include "comms/can/CANCommander.h"
#include "SimpleCANio.h"

// =============================================================================
// Pin Definitions
// =============================================================================

// 6-PWM gate driver (high-side / low-side pairs per phase)
#define M0_INH_A PA8
#define M0_INH_B PA9
#define M0_INH_C PA10
#define M0_INL_A PB13
#define M0_INL_B PB14
#define M0_INL_C PB15
#define EN_GATE  PB12

// AS5047 magnetic encoder — SPI3
#define SPI3_SCL  PC10
#define SPI3_MISO PC11
#define SPI3_MOSI PC12
#define AS5047_CS PA15

// Low-side current sensing (phases B and C; phase A not connected)
#define M0_IB PC0
#define M0_IC PC1

// Supply voltage divider
#define VSENS  PA6
#define VSCALE 19.0f   // ADC → actual voltage scale factor

// CAN bus
#define CAN0_RX     PB8
#define CAN0_TX     PB9
#define CAN_NODE_ID 1  // change if multiple nodes share the bus

// =============================================================================
// Motor / Gearbox Parameters
// =============================================================================

#define KT         0.23f   // motor torque constant [Nm/A]
#define GEAR_RATIO 15.0f   // gearbox reduction ratio
#define GEAR_EFF   0.6f    // gearbox efficiency

// =============================================================================
// System Limits
// =============================================================================

#define I_MAX          7.0f                                  // peak phase current [A]
#define TORQUE_OUT_MAX (KT * I_MAX * GEAR_RATIO * GEAR_EFF) // max output torque [Nm]

// =============================================================================
// Current Controller Tuning
// =============================================================================
// Raise P for faster current response; raise I to eliminate steady-state error.
// Raise LPF_TF for smoother but slower current feedback filtering.
// PI_OUT_RAMP limits how fast the PI output voltage can change [V/s].

#define PI_Q_P       0.3f    // q-axis (torque) proportional gain
#define PI_Q_I       10.0f   // q-axis integral gain
#define PI_D_P       0.3f    // d-axis (flux) proportional gain
#define PI_D_I       10.0f   // d-axis integral gain
#define LPF_TF       0.1f    // current low-pass filter time constant [s]
#define PI_OUT_RAMP  100.0f  // PI output ramp rate [V/s]

// Torque setpoint ramp rate — prevents step changes from feeling harsh [Nm/s]
#define TORQUE_RAMP_NM_S 3.0f

// =============================================================================
// Hardware Objects
// =============================================================================

MagneticSensorSPI   sensor        = MagneticSensorSPI(AS5047_SPI, AS5047_CS);
SPIClass            SPI_3(SPI3_MOSI, SPI3_MISO, SPI3_SCL);
BLDCDriver6PWM      driver        = BLDCDriver6PWM(M0_INH_A, M0_INL_A,
                                                    M0_INH_B, M0_INL_B,
                                                    M0_INH_C, M0_INL_C, EN_GATE);
BLDCMotor           motor         = BLDCMotor(14); // 14 pole pairs
LowsideCurrentSense current_sense = LowsideCurrentSense(
    0.0005f,  // shunt resistance [Ω] — 0.5 mΩ
    80.0f,    // amplifier gain
    _NC, M0_IB, M0_IC);

// =============================================================================
// Communication Objects
// =============================================================================

CANio        can(CAN0_RX, CAN0_TX);
CANCommander commandc(can, CAN_NODE_ID);

// =============================================================================
// Shared State (written by CAN, read by FOC ISR)
// =============================================================================

volatile float target_torque_nm = 0.0f; // desired output torque [Nm] — 0 on boot
volatile float ramped_torque_nm = 0.0f; // rate-limited copy fed to the current controller

// =============================================================================
// Helper Functions
// =============================================================================

float readSupplyVoltage() {
  return (analogRead(VSENS) / 4095.0f) * 3.3f * VSCALE;
}

// =============================================================================
// CAN Register Handlers — REG_CUSTOM_START (0xE0), 4-byte float [Nm]
// =============================================================================

bool canReadTorque(RegisterIO& comms, FOCMotor*) {
  float nm = target_torque_nm;
  comms << nm;
  return true;
}

bool canWriteTorque(RegisterIO& comms, FOCMotor*) {
  float nm = 0.0f;
  comms >> nm;
  if      (nm >  TORQUE_OUT_MAX) nm =  TORQUE_OUT_MAX;
  else if (nm < -TORQUE_OUT_MAX) nm = -TORQUE_OUT_MAX;
  target_torque_nm = nm;
  return true;
}

// =============================================================================
// Setup
// =============================================================================

void setup() {
  Serial.begin(115200);
  SimpleFOCDebug::enable(&Serial); // startup diagnostics only

  // --- Supply voltage ---
  pinMode(VSENS, INPUT_ANALOG);
  float v = readSupplyVoltage();
  Serial.print("Supply voltage: "); Serial.print(v); Serial.println(" V");

  // --- Encoder ---
  SPI_3.begin();
  sensor.init(&SPI_3);

  // --- Gate driver ---
  driver.pwm_frequency        = 50000; // 50 kHz PWM
  driver.voltage_power_supply = v;
  driver.voltage_limit        = v;
  driver.dead_zone            = 0.05f;
  if (!driver.init()) {
    Serial.println("Driver init failed!");
    return;
  }
  driver.enable();

  // --- Motor ---
  motor.linkDriver(&driver);
  motor.linkSensor(&sensor);

  motor.torque_controller    = TorqueControlType::foc_current; // closed-loop current control
  motor.controller           = MotionControlType::torque;
  motor.current_limit        = I_MAX;
  motor.voltage_limit        = v;
  motor.voltage_sensor_align = 2.0f; // alignment voltage — low enough to avoid current spike, high enough for current sense to register

  // Current PI gains and output filter
  motor.PID_current_q.P           = PI_Q_P;
  motor.PID_current_q.I           = PI_Q_I;
  motor.PID_current_q.output_ramp = PI_OUT_RAMP;
  motor.PID_current_d.P           = PI_D_P;
  motor.PID_current_d.I           = PI_D_I;
  motor.PID_current_d.output_ramp = PI_OUT_RAMP;
  motor.LPF_current_q.Tf          = LPF_TF;
  motor.LPF_current_d.Tf          = LPF_TF;

  // --- CAN commander ---
  commandc.init();
  commandc.addMotor(&motor);
  commandc.addCustomRegister(REG_CUSTOM_START, 4, canReadTorque, canWriteTorque);

  motor.init();

  // --- Current sensing ---
  current_sense.linkDriver(&driver);
  if (!current_sense.init()) {
    Serial.println("Current sense init failed!");
    return;
  }
  current_sense.skip_align = true; // hardware polarity is fixed on MKS XDrive Mini
  motor.linkCurrentSense(&current_sense);

  // --- FOC calibration ---
  motor.initFOC();

  // --- FOC loop at 20 kHz via hardware timer ---
  HardwareTimer* foc_timer = new HardwareTimer(TIM8);
  foc_timer->setOverflow(20000, HERTZ_FORMAT);
  foc_timer->attachInterrupt([]() {
    motor.loopFOC(); // runs current PI and updates PWM

    // Rate-limit the torque setpoint to avoid sudden current steps
    constexpr float step = TORQUE_RAMP_NM_S / 20000.0f; // Nm per tick
    float diff = target_torque_nm - ramped_torque_nm;
    if      (diff >  step) ramped_torque_nm += step;
    else if (diff < -step) ramped_torque_nm -= step;
    else                   ramped_torque_nm  = target_torque_nm;

    // Convert output Nm → motor Iq [A] and pass to current controller
    motor.move(ramped_torque_nm / (GEAR_RATIO * GEAR_EFF * KT));
  });
  foc_timer->resume();

  Serial.println("Ready. Waiting for CAN torque commands on register 0xE0.");
}

// =============================================================================
// Loop
// =============================================================================

void loop() {
  // Refresh supply voltage reading once per second so the driver stays accurate
  static unsigned long last_vsens = 0;
  if (millis() - last_vsens > 1000) {
    last_vsens = millis();
    driver.voltage_power_supply = readSupplyVoltage();
  }

  commandc.run(); // process incoming CAN messages
}
