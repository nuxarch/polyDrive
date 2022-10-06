/**
 * Comprehensive BLDC motor control example using encoder and the DRV8302 board
 *
 * Using serial terminal user can send motor commands and configure the motor and FOC in real-time:
 * - configure PID controller constants
 * - change motion control loops
 * - monitor motor variabels
 * - set target values
 * - check all the configuration values
 *
 * check the https://docs.simplefoc.com for full list of motor commands
 *
 */
#include <SimpleFOC.h>
#define THROTTLE_PIN    33
// DRV8302 pins connections
// don't forget to connect the common ground pin
#define INH_A 25
#define INH_B 26
#define INH_C 27

#define EN_GATE 14
#define M_PWM 19
#define M_OC 18
#define OC_ADJ 21

BLDCMotor motor = BLDCMotor(23,1);
BLDCDriver3PWM driver = BLDCDriver3PWM(INH_A, INH_B, INH_C, EN_GATE);


// commander interface
Commander command = Commander(Serial);
void onMotor(char *cmd) { command.motor(&motor, cmd); }

void setup()
{
  Serial.begin(115200);
  // DRV8302 specific code
  // M_OC  - enable overcurrent protection
  pinMode(M_OC, OUTPUT);
  digitalWrite(M_OC, LOW);
  // M_PWM  - enable 3pwm mode
  pinMode(M_PWM, OUTPUT);
  digitalWrite(M_PWM, HIGH);
  // OD_ADJ - set the maximum overcurrent limit possible
  // Better option would be to use voltage divisor to set exact value
  pinMode(OC_ADJ, OUTPUT);
  digitalWrite(OC_ADJ, HIGH);

  // driver config
  // power supply voltage [V]
  driver.voltage_power_supply = 30;
  driver.init();
  // link the motor and the driver
  motor.linkDriver(&driver);

  // set control loop type to be used
  motor.controller = MotionControlType::torque;
  // motor.controller = MotionControlType::velocity;

  // default voltage_power_supply
  motor.voltage_limit = 30;

  motor.useMonitoring(Serial);

  // initialise motor
  motor.init();
  command.add('T', onMotor, "motor");

  Serial.println(F("Full control example: "));
  Serial.println(F("Run user commands to configure and the motor (find the full command list in docs.simplefoc.com) \n "));
  Serial.println(F("Initial motion control loop is voltage loop."));
  Serial.println(F("Initial target voltage 2V."));

  _delay(1000);
}
int throttle_value = 0;
void loop()
{

  // throttle_value = analogRead(THROTTLE_PIN);
  
  // throttle_value = map(throttle_value, 1000, 4095, 0,10);
  // // motor.target = throttle_value;
  // Serial.println("raw:"+String(throttle_value));
  // // iterative setting FOC phase voltage
  // motor.loopFOC();

  // iterative function setting the outter loop target
  // velocity, position or voltage
  // if tatget not set in parameter uses motor.target variable
  motor.move();

  // user communication
  command.run();
}