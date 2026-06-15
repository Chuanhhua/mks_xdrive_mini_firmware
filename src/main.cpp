/*
  This code is an example of the full initialization of the MKS XDrive Mini board with 
  the SimpleFOC library, including the setup of a Serial commander and a CAN commander 
  for controlling the motor. The code initializes the motor, driver, sensor, and current 
  sensing hardware, and sets up a timer to run the FOC algorithm in real-time.


  CAN commander does not use CPU if not used, so it can be included in the project 
  without any overhead if CAN communication is not needed.

  The Serial commander allows to send commands to the motor and read monitoring 
  data through the serial port and is configured to work directly with
  webcontroller.simplefoc.com for easy monitoring and control of the motor.

  The FOC loop is run in a hardware timer interrupt for better real-time performance, 
  loopFOC is run at 20khz while the motion control loop is run every 5 FOC loops
  (4kHz) - this can be adjusted based on the use case. 
*/
#include <Arduino.h>
#include <SimpleFOC.h>
#include "current_sense/hardware_specific/stm32/stm32_adc_utils.h"

#include "SimpleFOCDrivers.h"
#include "comms/can/CANCommander.h"
#include "SimpleCANio.h"


// XDRIVE M0 motor pinout
#define M0_INH_A PA8
#define M0_INH_B PA9
#define M0_INH_C PA10
#define M0_INL_A PB13
#define M0_INL_B PB14
#define M0_INL_C PB15
// M0 enable pin
#define EN_GATE PB12
// M0 currents
#define M0_IB PC0
#define M0_IC PC1

// SPI pinout
#define SPI3_SCL  PC10
#define SPI3_MISO PC11
#define SPI3_MOSI PC12
#define AS5047_CS PA15
#define M0_nCS    PC13

// voltage sensing pin and scale factor
#define VSENS    PA6
#define VSCALE   19.0f // voltage divider scale factor for voltage sensing

// CAN bus
#define CAN0_RX     PB8
#define CAN0_TX     PB9
#define CAN_NODE_ID 15  // change if multiple nodes share the bus

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

volatile float target_torque_nm = 3.0f; // desired output torque [Nm] — 0 on boot
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

  // Print the interpreted payload directly to diagnostic Serial
  Serial.print("MKS CAN RX -> Reg 0xE0 (Write) -> Decoded Torque Target: ");
  Serial.print(nm, 4); // Print with 4 decimal places
  Serial.println(" Nm");

  if      (nm >  TORQUE_OUT_MAX) nm =  TORQUE_OUT_MAX;
  else if (nm < -TORQUE_OUT_MAX) nm = -TORQUE_OUT_MAX;
  __disable_irq();
  target_torque_nm = nm;
  __enable_irq();
  return true;
}

// =============================================================================
// Setup
// =============================================================================

void setup() {
  can.begin(1000000);  // must match Teensy's 1 Mbps

  Serial.begin(115200);
  // enable more verbose output for debugging
  // comment out if not needed
  SimpleFOCDebug::enable(&Serial);

  pinMode(VSENS, INPUT_ANALOG);
  float v = _readRegularADCVoltage(VSENS)*VSCALE;
  SIMPLEFOC_DEBUG(" V sens: ", v);

  // configure the gain of the drv8301 to 80 for better low current sensing resolution
  SPI_3.begin();
  SPI_3.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  digitalWrite(M0_nCS, LOW);
  SPI_3.transfer(0x03); // address of the control register 3
  SPI_3.transfer(0b00000011); // set gain to 80
  digitalWrite(M0_nCS, HIGH); 
  SPI_3.endTransaction();

  // power supply voltage [V]
  driver.voltage_power_supply = v;
  // Max DC voltage allowed - default voltage_power_supply
  driver.voltage_limit = v;
  driver.dead_zone = 0.001f;
  // driver init
  driver.init();
  // link the motor and the driver
  motor.linkDriver(&driver);

  // initialize encoder sensor hardware
  sensor.init(&SPI_3);
  // link the motor to the sensor
  motor.linkSensor(&sensor);
  
  // control loop type and torque mode 
  motor.torque_controller = TorqueControlType::foc_current;
  motor.controller = MotionControlType::torque;

  // max voltage  allowed for motion control 
  motor.voltage_limit = 1.0;
  // alignment voltage limit
  motor.voltage_sensor_align = 1.5;
  
  // comment out if not needed
  motor.useMonitoring(Serial);
  // setup monitoring for webcontroller.simplefoc.com
  motor.monitor_end_char = 'M'; // set monitoring end character to M 
  motor.monitor_start_char = 'M'; // set monitoring start character 
  // add target command T
  command.add('M', doMotor, "motor M0");
  motor.monitor_downsample = 0; // disable at start

  // instantiate the CAN commander
  commandc.init();
  commandc.addMotor(&motor);

  // motor.init();

  // // --- Current sensing ---
  // current_sense.linkDriver(&driver);
  // if (!current_sense.init()) {
  //   Serial.println("Current sense init failed!");
  //   return;
  // }
  // current_sense.skip_align = true; // hardware polarity is fixed on MKS XDrive Mini
  // motor.linkCurrentSense(&current_sense);

  // // --- FOC calibration ---
  // motor.initFOC();

  // // --- FOC loop at 20 kHz via hardware timer ---
  // HardwareTimer* foc_timer = new HardwareTimer(TIM8);
  // foc_timer->setOverflow(20000, HERTZ_FORMAT);
  // foc_timer->attachInterrupt([]() {
  //   motor.loopFOC(); // runs current PI and updates PWM

  //   // Rate-limit the torque setpoint to avoid sudden current steps
  //   constexpr float step = TORQUE_RAMP_NM_S / 20000.0f; // Nm per tick
  //   float diff = target_torque_nm - ramped_torque_nm;
  //   if      (diff >  step) ramped_torque_nm += step;
  //   else if (diff < -step) ramped_torque_nm -= step;
  //   else                   ramped_torque_nm  = target_torque_nm;

  //   // Convert output Nm → motor Iq [A] and pass to current controller
  //   motor.move(ramped_torque_nm / (GEAR_RATIO * GEAR_EFF * KT));
  // });
  // foc_timer->resume();

  Serial.println("Ready. Waiting for CAN torque commands on register 0xE0.");
}


void loop() {
  // Serial.print("help");
  if (can.available() > 0) {
    CanMsg const rxMsg = can.read();
    
    Serial.print("👉 RAW CAN PACKET RECEIVED! ID: 0x");
    if (rxMsg.isExtendedId()) {
      Serial.print(rxMsg.getExtendedId(), HEX);
      Serial.print(" (Extended)");
    } else {
      Serial.print(rxMsg.getStandardId(), HEX);
      Serial.print(" (Standard)");
    }
    
    Serial.print(" | DLC: ");
    Serial.print(rxMsg.data_length);
    
    Serial.print(" | Data: ");
    for (int i = 0; i < rxMsg.data_length; i++) {
      Serial.print(rxMsg.data[i], HEX);
      Serial.print(" ");
    }
    Serial.println();
  }



  // Refresh supply voltage reading once per second so the driver stays accurate
  static unsigned long last_vsens = 0;
  if (millis() - last_vsens > 1000) {
    last_vsens = millis();
    driver.voltage_power_supply = readSupplyVoltage();
  }
  // commandc.run(); // process incoming CAN messages'



}
