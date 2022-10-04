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

// Motor instance
BLDCMotor motor = BLDCMotor(13,3);
BLDCDriver3PWM driver = BLDCDriver3PWM(INH_A, INH_B, INH_C, EN_GATE);

// SENSOR
HallSensor sensor = HallSensor(32, 35, 34, 13);
void doA() { sensor.handleA(); }
void doB() { sensor.handleB(); }
void doC() { sensor.handleC(); }

// commander interface
Commander command = Commander(Serial);
void onMotor(char *cmd) { command.motor(&motor, cmd); }

void setup()
{
  Serial.begin(115200);
  // initialize encoder sensor hardware
  sensor.pullup = Pullup::USE_INTERN;
  sensor.init();
  sensor.enableInterrupts(doA, doB, doC);
  Serial.println("Sensor ready");
  _delay(1000);

  // link the motor to the sensor
  motor.linkSensor(&sensor);

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
  driver.voltage_power_supply = 35;
  driver.init();
  // link the motor and the driver
  motor.linkDriver(&driver);
  motor.voltage_sensor_align = 14;
  motor.velocity_index_search = 2;
  // motor.phase_resistance = 0.0;

  // choose FOC modulation
  motor.foc_modulation = FOCModulationType::SinePWM;

  // set control loop type to be used
  motor.controller = MotionControlType::torque;
  // motor.controller = MotionControlType::velocity;

  // contoller configuration based on the controll type
  motor.PID_velocity.P = 0.2f;
  motor.PID_velocity.I = 10;
  // default voltage_power_supply
  motor.voltage_limit = 35;

  // velocity low pass filtering time constant
  motor.LPF_velocity.Tf = 0.01f;

  // angle loop controller
  // motor.P_angle.P = 20;
  // angle loop velocity limit
  // motor.velocity_limit = 50;

  // use monitoring with serial for motor init
  // monitoring port
  // Serial.begin(115200);
  // comment out if not needed
  motor.useMonitoring(Serial);

  // initialise motor
  motor.init();
  // align encoder and start FOC
  motor.initFOC();

  // set the inital target value
  // motor.target = 2;

  // define the motor id
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

  throttle_value = analogRead(THROTTLE_PIN);
  
  // throttle_value = map(throttle_value, 0, 4095, );
  Serial.println("raw:"+String(throttle_value));
  // iterative setting FOC phase voltage
  motor.loopFOC();

  // iterative function setting the outter loop target
  // velocity, position or voltage
  // if tatget not set in parameter uses motor.target variable
  motor.move();

  // user communication
  command.run();
}