// /*
//   This code is an example of the full initialization of the MKS XDrive Mini board with 
//   the SimpleFOC library, including the setup of a Serial commander and a CAN commander 
//   for controlling the motor. The code initializes the motor, driver, sensor, and current 
//   sensing hardware, and sets up a timer to run the FOC algorithm in real-time.


//   CAN commander does not use CPU if not used, so it can be included in the project 
//   without any overhead if CAN communication is not needed.

//   The Serial commander allows to send commands to the motor and read monitoring 
//   data through the serial port and is configured to work directly with
//   webcontroller.simplefoc.com for easy monitoring and control of the motor.

//   The FOC loop is run in a hardware timer interrupt for better real-time performance, 
//   loopFOC is run at 20khz while the motion control loop is run every 5 FOC loops
//   (4kHz) - this can be adjusted based on the use case. 
// */
// #include <Arduino.h>
// #include <SimpleFOC.h>
// #include "current_sense/hardware_specific/stm32/stm32_adc_utils.h"

// #include "SimpleFOCDrivers.h"
// // #include "comms/can/CANCommander.h"
// // #include "SimpleCANio.h"


// // // XDRIVE M0 motor pinout
// #define M0_INH_A PA8
// #define M0_INH_B PA9
// #define M0_INH_C PA10
// #define M0_INL_A PB13
// #define M0_INL_B PB14
// #define M0_INL_C PB15
// // M0 enable pin
// #define EN_GATE PB12
// // M0 currents
// #define M0_IB PC0
// #define M0_IC PC1

// // SPI pinout
// #define SPI3_SCL  PC10
// #define SPI3_MISO PC11
// #define SPI3_MOSI PC12
// #define AS5047_CS PA15
// #define M0_nCS    PC13

// // voltage sensing pin and scale factor
// #define VSENS    PA6
// #define VSCALE   19.0f // voltage divider scale factor for voltage sensing

// // can communication pintout
// #define CAN0_RX PB_8
// #define CAN0_TX PB_9

// // Motor instance
// BLDCMotor motor = BLDCMotor(14);
// BLDCDriver6PWM driver = BLDCDriver6PWM(M0_INH_A,M0_INL_A, M0_INH_B,M0_INL_B, M0_INH_C,M0_INL_C, EN_GATE);

// MagneticSensorPWM sensor = MagneticSensorPWM(PA3, 56, 2170);
// void doPWM(){sensor.handlePWM();}

// // low side current sensing define
// // 0.0005 Ohm resistor
// // gain of 10x, 20x, 40x, 80x possible -
// // we will use 80x for better low current sensing resolution (+-40A range)
// // current sensing on B and C phases, phase A not connected
// LowsideCurrentSense current_sense = LowsideCurrentSense(0.0005f, 80.0f, _NC, M0_IB, M0_IC);

// // initialising the sensor
// // MagneticSensorSPI sensor = MagneticSensorSPI(AS5047_SPI, AS5047_CS, 10000000);
// // SPI instance for sensor and driver configuration
// SPIClass SPI_3(SPI3_MOSI, SPI3_MISO, SPI3_SCL);

// // instantiate the serial commander
// Commander command = Commander(Serial);
// void doMotor(char* cmd) { command.motor(&motor, cmd); }

// // instantiate the CAN commander (does not use CPU if not used)
// // CANio can(CAN0_RX, CAN0_TX); // Create CAN object
// // CANCommander commandc(can, 15);//, false, 1000000, true);


// void setup(){
//   _delay(6000);
//   // use monitoring with serial 
//   Serial.begin(115200);
//   // enable more verbose output for debugging
//   // comment out if not needed
//   SimpleFOCDebug::enable(&Serial);

//   pinMode(VSENS, INPUT_ANALOG);
//   float v = _readRegularADCVoltage(VSENS)*VSCALE;
//   SIMPLEFOC_DEBUG(" V sens: ", v);

//   // configure the gain of the drv8301 to 80 for better low current sensing resolution
//   SPI_3.begin();
//   SPI_3.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
//   digitalWrite(M0_nCS, LOW);
//   SPI_3.transfer(0x03); // address of the control register 3
//   SPI_3.transfer(0b00000011); // set gain to 80
//   digitalWrite(M0_nCS, HIGH); 
//   SPI_3.endTransaction();

//   // power supply voltage [V]
//   driver.voltage_power_supply = v;
//   // Max DC voltage allowed - default voltage_power_supply
//   driver.voltage_limit = v;
//   driver.dead_zone = 0.001f;
//   // driver init
//   driver.init();
//   // link the motor and the driver
//   motor.linkDriver(&driver);

//   // initialize encoder sensor hardware
//   // sensor.init(&SPI_3);
//   sensor.init();
//   // link the motor to the sensor
//   motor.linkSensor(&sensor);
  
//   // control loop type and torque mode 
//   motor.torque_controller = TorqueControlType::foc_current;
//   motor.controller = MotionControlType::torque;

//   // max voltage  allowed for motion control 
//   motor.voltage_limit = 1.0;
//   // alignment voltage limit
//   motor.voltage_sensor_align = 1.5;
  
//   // comment out if not needed
//   motor.useMonitoring(Serial);
//   // setup monitoring for webcontroller.simplefoc.com
//   motor.monitor_end_char = 'M'; // set monitoring end character to M 
//   motor.monitor_start_char = 'M'; // set monitoring start character 
//   // add target command T
//   command.add('M', doMotor, "motor M0");
//   motor.monitor_downsample = 0; // disable at start

//   // instantiate the CAN commander
//   // commandc.init();
//   // commandc.addMotor(&motor);

//   // initialise motor
//   motor.init();

//   // link the driver
//   current_sense.linkDriver(&driver);
//   // init the current sense
//   current_sense.init();  
//   current_sense.skip_align = true;
//   motor.linkCurrentSense(&current_sense);

//   // init FOC  
//   motor.initFOC();  

//   //motor.characteriseMotor(1.0f); // characterise motor with 1.0V
//   motor.tuneCurrentController(100.0f);
//   delay(1000);

  
//   motor.motion_downsample = 5; // run motion control every 10 loops (depends on the use case)
//   // create a hardware timer
//   // For example, we will create a timer that runs at 10kHz on the TIM5
//   HardwareTimer* timer = new HardwareTimer(TIM8);
//   // Set timer frequency to 50kHz
//   timer->setOverflow(20000, HERTZ_FORMAT); 
//   // add the loopFOC and move to the timer
//   timer->attachInterrupt([](){
//     // call the loopFOC and move functions
//     motor.loopFOC();
//     motor.move();
//   });
//   // start the timer
//   timer->resume();
// }


// void loop(){
//   // monitoring 
//   motor.monitor();
//   // user communication
//   command.run();
//   // CAN communication
//   // commandc.run();
// }



#include <SimpleFOC.h>
/**
 * Magnetic sensor reading pwm signal on pin PA3.  The pwm duty cycle is proportional to the sensor angle.
 * 
 * MagneticSensorPWM(uint8_t MagneticSensorPWM, int _min, int _max)
 * - pinPWM         - the pin that is reading the pwm from magnetic sensor
 * - min_raw_count  - the smallest expected reading.  Whilst you might expect it to be 0 it is often ~5.  Getting this wrong results in a small click once per revolution
 * - max_raw_count  - the largest value read.  whilst you might expect it to be 1kHz = 1000 it is often ~910. depending on the exact frequency and saturation
 */
MagneticSensorPWM sensor = MagneticSensorPWM(PA3, 56, 2170);
void doPWM(){sensor.handlePWM();}

void setup() {
  // monitoring port
  Serial.begin(115200);
  
  // initialise magnetic sensor hardware
  sensor.init();
  // comment out to use sensor in blocking (non-interrupt) way
  sensor.enableInterrupt(doPWM);

  Serial.println("Sensor ready");
  _delay(1000);
}

void loop() {
  // iterative function updating the sensor internal variables
  // it is usually called in motor.loopFOC()
  // this function reads the sensor hardware and 
  // has to be called before getAngle nad getVelocity
  sensor.update();
  // display the angle and the angular velocity to the terminal
  Serial.print(sensor.getAngle());
  Serial.print("\t");
  Serial.println(sensor.getVelocity());
  _delay(100);
}



/**
 *
 * Position/angle motion control example
 * Steps:
 * 1) Configure the motor and magnetic sensor
 * 2) Run the code
 * 3) Set the target angle (in radians) from serial terminal
 *
 */
// #include <SimpleFOC.h>

// // magnetic sensor instance - SPI
// // MagneticSensorSPI sensor = MagneticSensorSPI(AS5147_SPI, 10);
// // magnetic sensor instance - MagneticSensorI2C
// // MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);
// // magnetic sensor instance - analog output
// // MagneticSensorAnalog sensor = MagneticSensorAnalog(A1, 14, 1020);

// MagneticSensorPWM sensor = MagneticSensorPWM(PA3, 56, 2170);
// void doPWM(){sensor.handlePWM();}


// // // BLDC motor & driver instance
// // BLDCMotor motor = BLDCMotor(11);
// // BLDCDriver3PWM driver = BLDCDriver3PWM(9, 5, 6, 8);
// // Stepper motor & driver instance
// //StepperMotor motor = StepperMotor(50);
// //StepperDriver4PWM driver = StepperDriver4PWM(9, 5, 10, 6,  8);

// BLDCMotor motor = BLDCMotor(14);
// BLDCDriver6PWM driver = BLDCDriver6PWM(M0_INH_A,M0_INL_A, M0_INH_B,M0_INL_B, M0_INH_C,M0_INL_C, EN_GATE);


// // angle set point variable
// float target_angle = 0;
// // instantiate the commander
// Commander command = Commander(Serial);
// void doTarget(char* cmd) { command.scalar(&target_angle, cmd); }

// void setup() {

//   // use monitoring with serial 
//   Serial.begin(115200);
//   // enable more verbose output for debugging
//   // comment out if not needed
//   SimpleFOCDebug::enable(&Serial);

//   // initialise magnetic sensor hardware
//   sensor.init();
//   // link the motor to the sensor
//   motor.linkSensor(&sensor);

//   // driver config
//   // power supply voltage [V]
//   driver.voltage_power_supply = 31;
//   driver.init();
//   // link the motor and the driver
//   motor.linkDriver(&driver);

//   // choose FOC modulation (optional)
//   motor.foc_modulation = FOCModulationType::SpaceVectorPWM;

//   // set motion control loop to be used
//   motor.controller = MotionControlType::angle;

//   // contoller configuration
//   // default parameters in defaults.h

//   // velocity PI controller parameters
//   motor.PID_velocity.P = 0.2f;
//   motor.PID_velocity.I = 20;
//   motor.PID_velocity.D = 0;
//   // maximal voltage to be set to the motor
//   motor.voltage_limit = 31;

//   // velocity low pass filtering time constant
//   // the lower the less filtered
//   motor.LPF_velocity.Tf = 0.01f;

//   // angle P controller
//   motor.P_angle.P = 20;
//   // maximal velocity of the position control
//   motor.velocity_limit = 20;
  
//   // comment out if not needed
//   motor.useMonitoring(Serial);


//   // initialize motor
//   motor.init();
//   // align sensor and start FOC
//   motor.initFOC();

//   // add target command T
//   command.add('T', doTarget, "target angle");

//   Serial.println(F("Motor ready."));
//   Serial.println(F("Set the target angle using serial terminal:"));
//   _delay(1000);
// }


// void loop() {

//   // main FOC algorithm function
//   // the faster you run this function the better
//   // Arduino UNO loop  ~1kHz
//   // Bluepill loop ~10kHz
//   // sensor.update(); // Temporarily comment out for debugging serial input
//   // Serial.print(sensor.getAngle()); // Temporarily comment out for debugging serial input
//   motor.loopFOC();

//   // Motion control function
//   // velocity, position or voltage (defined in motor.controller)
//   // this function can be run at much lower frequency than loopFOC() function
//   // You can also use motor.move() and set the motor.target in the code
//   motor.move(target_angle);

//   // function intended to be used with serial plotter to monitor motor variables
//   // significantly slowing the execution down!!!!
//   // motor.monitor(); // Temporarily comment out for debugging serial input

//   // user communication
//   command.run();
// }