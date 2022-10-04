#include <Arduino.h>
#include <SimpleFOC.h>
#include <DRV8301.h>
BLDCMotor motor = BLDCMotor(7, 0.5);
BLDCDriver3PWM driver = BLDCDriver3PWM(33, 25, 26);
DRV8301 gate_driver = DRV8301(23, 19, 18, 5, 27, 14);
float target_velocity = 0;
HallSensor sensor = HallSensor(34, 35, 32, 7);
void doA() { sensor.handleA(); }
void doB() { sensor.handleB(); }
void doC() { sensor.handleC(); }

Commander command = Commander(Serial);
void doTarget(char *cmd) { command.scalar(&target_velocity, cmd); }
void setup()
{

  // initialize sensor sensor hardware
  sensor.init();
  sensor.enableInterrupts(doA, doB, doC);
  // software interrupts
  // PciManager.registerListener(&listenerIndex);
  // link the motor to the sensor
  motor.linkSensor(&sensor);

  // driver config
  // power supply voltage [V]
  driver.voltage_power_supply = 36;
  driver.init();
  motor.linkDriver(&driver);
  gate_driver.begin(PWM_INPUT_MODE_3PWM);
  // link the motor and the driver
  motor.linkDriver(&driver);

  // aligning voltage [V]
  // aligning voltage [V]
  motor.voltage_sensor_align = 2;
  // index search velocity [rad/s]
  motor.velocity_index_search = 3;
  // limiting motor movements
  motor.voltage_limit = 1;    // [V]
  motor.velocity_limit = 20; // [rad/s]
  motor.current_limit = 2;

  // set motion control loop to be used
  motor.controller = MotionControlType::torque;

  // contoller configuration
  // default parameters in defaults.h
  // velocity PI controller parameters
  motor.PID_velocity.P = 5;
  motor.PID_velocity.I = 1;
  motor.PID_velocity.D = 0.0;

  motor.P_angle.D = 0.3;
  // default voltage_power_supply
  motor.voltage_limit = 7;
  // jerk control using voltage voltage ramp
  // default value is 300 volts per sec  ~ 0.3V per millisecond
  motor.PID_velocity.output_ramp = 1000;

  // velocity low pass filtering time constant
  motor.LPF_velocity.Tf = 0.01;

  // use monitoring with serial
  Serial.begin(115200);
  // comment out if not needed
  motor.useMonitoring(Serial);

  // initialize motor
  motor.init();
  // align sensor and start FOC
  motor.initFOC();

  // add target command T
  command.add('T', doTarget, "target voltage");
  Serial.println(F("Motor ready."));
  Serial.println(F("Set the target velocity using serial terminal:"));
  _delay(1000);
}
void loop()
{
  motor.loopFOC();
  motor.move(target_velocity);
  motor.monitor();
  command.run();
}