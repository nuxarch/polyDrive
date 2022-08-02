#include <Arduino.h>
#include <SimpleFOC.h>
#include <DRV8301.h>
// BLDCMotor motor = BLDCMotor(24,1.5);
// for quickrun motor 13.5T pp = 6pp
BLDCMotor motor = BLDCMotor(24,1);
BLDCDriver3PWM driver = BLDCDriver3PWM(33, 25, 26);
DRV8301 gate_driver = DRV8301(23, 19, 18, 5, 27, 14);
float target_velocity = 0;
Commander command = Commander(Serial);
void doTarget(char *cmd) { command.scalar(&target_velocity, cmd); }
void setup() {
  
  // pwm frequency to be used [Hz]
  driver.pwm_frequency = 20000;
  // power supply voltage [V]
  driver.voltage_power_supply = 32;
  // Max DC voltage allowed - default voltage_power_supply
  driver.voltage_limit = 12;
  gate_driver.begin(PWM_INPUT_MODE_3PWM);
  motor.linkDriver(&driver);
  // driver init
  driver.init();

  // enable driver
  driver.enable();

  _delay(1000);
}

void loop() {
    // phase (A: 3V, B: 6V, C: high impedance )  
    // set the phase C in high impedance mode - disabled or open
    // 1. 12,z,0
    driver.setPhaseState( _ACTIVE , _HIGH_IMPEDANCE, _ACTIVE);driver.setPwm(12, 0, 3);delay(100);
    // 2. z,12,0
    driver.setPhaseState(_HIGH_IMPEDANCE , _ACTIVE , _ACTIVE);driver.setPwm(0, 12, 3);delay(100);
    // 3. 0,12,z
    driver.setPhaseState( _ACTIVE , _ACTIVE, _HIGH_IMPEDANCE);driver.setPwm(3, 12, 0);delay(100); 
    // 4. 0,z,12
    driver.setPhaseState( _ACTIVE ,  _HIGH_IMPEDANCE,_ACTIVE);driver.setPwm(3, 0, 12);delay(100);
    // 5. z,0,12
    driver.setPhaseState(_HIGH_IMPEDANCE , _ACTIVE , _ACTIVE);driver.setPwm(0, 3, 12);delay(100);
    // 6. 12,0,z
}