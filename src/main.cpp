
#include <SimpleFOC.h>
#define THROTTLE_PIN 33
#define INH_A 25
#define INH_B 26
#define INH_C 27

#define EN_GATE 14
#define M_PWM 19
#define M_OC 18
#define OC_ADJ 21

// Motor instance
BLDCMotor motor = BLDCMotor(1,1);
BLDCDriver3PWM driver = BLDCDriver3PWM(INH_A, INH_B, INH_C, EN_GATE);

// SENSOR
// HallSensor sensor = HallSensor(32, 35, 34, 13);
HallSensor sensor = HallSensor(32, 35, 34, 1);
void doA() { sensor.handleA(); }
void doB() { sensor.handleB(); }
void doC() { sensor.handleC(); }
float target_voltage = 2;
Commander command = Commander(Serial);
void onMotor(char *cmd) { command.motor(&motor, cmd); }
void serialReceiveUserCommand()
{

  // a string to hold incoming data
  static String received_chars;

  while (Serial.available())
  {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the string buffer:
    received_chars += inChar;
    // end of user input
    if (inChar == '\n')
    {

      // change the motor target
      target_voltage = received_chars.toFloat();
      Serial.print("Target voltage: ");
      Serial.println(target_voltage);

      // reset the command buffer
      received_chars = "";
    }
  }
}

void setup()
{
  sensor.pullup = Pullup::USE_INTERN;
  sensor.init();
  sensor.enableInterrupts(doA, doB, doC);
  Serial.println("Sensor ready");
  delay(1000);;
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
  driver.voltage_power_supply = 40;
  driver.init();
  motor.linkDriver(&driver);
  motor.init();
  Serial.begin(115200);
  Serial.println("Pole pairs (PP) estimator");
  Serial.println("-\n");
  float pp_search_voltage = 8;     // maximum power_supply_voltage/2
  float pp_search_angle = 6 * _PI; // search electrical angle to turn
  motor.controller = MotionControlType::angle_openloop;
  motor.voltage_limit = pp_search_voltage;
  motor.move(0);
  _delay(1000);
  // read the sensor angle
  sensor.update();
  float angle_begin = sensor.getAngle();
  _delay(50);

  // move the motor slowly to the electrical angle pp_search_angle
  float motor_angle = 0;
  while (motor_angle <= pp_search_angle)
  {
    motor_angle += 0.01f;
    sensor.update(); // keep track of the overflow
    motor.move(motor_angle);
    _delay(1);
  }
  _delay(1000);
  // read the sensor value for 180
  sensor.update();
  float angle_end = sensor.getAngle();
  _delay(50);
  // turn off the motor
  motor.move(0);
  _delay(1000);

  // calculate the pole pair number
  int pp = round((pp_search_angle) / (angle_end - angle_begin));

  Serial.print(F("Estimated PP : "));
  Serial.println(pp);
  Serial.println(F("PP = Electrical angle / Encoder angle "));
  Serial.print(pp_search_angle * 180 / _PI);
  Serial.print(F("/"));
  Serial.print((angle_end - angle_begin) * 180 / _PI);
  Serial.print(F(" = "));
  Serial.println((pp_search_angle) / (angle_end - angle_begin));
  Serial.println();

  // a bit of monitoring the result
  if (pp <= 0)
  {
    Serial.println(F("PP number cannot be negative"));
    Serial.println(F(" - Try changing the search_voltage value or motor/sensor configuration."));
    return;
  }
  else if (pp > 30)
  {
    Serial.println(F("PP number very high, possible error."));
  }
  else
  {
    Serial.println(F("If PP is estimated well your motor should turn now!"));
    Serial.println(F(" - If it is not moving try to relaunch the program!"));
    Serial.println(F(" - You can also try to adjust the target voltage using serial terminal!"));
  }

  // set motion control loop to be used
  motor.controller = MotionControlType::torque;
  // set the pole pair number to the motor
  motor.pole_pairs = pp;
  // align sensor and start FOC
  motor.initFOC();
  _delay(1000);

  Serial.println(F("\n Motor ready."));
  Serial.println(F("Set the target voltage using serial terminal:"));
}

void loop()
{

  // main FOC algorithm function
  // the faster you run this function the better
  // Arduino UNO loop  ~1kHz
  // Bluepill loop ~10kHz
  motor.loopFOC();

  // Motion control function
  // velocity, position or voltage (defined in motor.controller)
  // this function can be run at much lower frequency than loopFOC() function
  // You can also use motor.move() and set the motor.target in the code
  motor.move(target_voltage);

  // communicate with the user
  serialReceiveUserCommand();
}
