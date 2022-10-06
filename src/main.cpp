#include <SimpleFOC.h>
#define INH_A 25
#define INH_B 26
#define INH_C 27

#define EN_GATE 14
#define M_PWM 19
#define M_OC 18
#define OC_ADJ 21
BLDCMotor motor = BLDCMotor(13);
BLDCDriver3PWM driver = BLDCDriver3PWM(INH_A, INH_B, INH_C, EN_GATE);
HallSensor sensor = HallSensor(32, 35, 34, 13);
void doA() { sensor.handleA(); }
void doB() { sensor.handleB(); }
void doC() { sensor.handleC(); }
Commander command = Commander(Serial);
void onMotor(char *cmd) { command.motor(&motor, cmd); }
void setup() {
  Serial.begin(115200);
  sensor.pullup = Pullup::USE_INTERN;
  sensor.init();
  sensor.enableInterrupts(doA, doB, doC);
  Serial.println("Sensor ready");
  _delay(1000);
}

void loop() {
  sensor.update();
  Serial.print(sensor.getAngle());
  Serial.print("\t");
  Serial.println(sensor.getVelocity());
}
