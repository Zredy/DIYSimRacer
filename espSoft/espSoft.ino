#include <SimpleFOC.h>
#include <Wire.h>
#include <I2C_Anything.h>

#define SDA_pin 12
#define SCL_pin 4
#define CS_pin 5
#define MOTA_pin 6
#define MOTB_pin 8
#define MOTC_pin 10
#define MOTD_pin 13

#define angle_range 270 //range of wheel motion (in degrees!)

MagneticSensorSPI sensor = MagneticSensorSPI(AS5047_SPI, CS_pin);
StepperMotor motor = StepperMotor(50, 1.65);
StepperDriver4PWM driver = StepperDriver4PWM(MOTA_pin, MOTB_pin, MOTC_pin, MOTD_pin);

//define values for wire communication
int force_inp = 0;

float angle_range_rad = (angle_range * 3.14 / 180) * 3;

// voltage set point variable
float target_voltage = 0;
// instantiate the commander
//Commander command = Commander(Serial);
//void doTarget(char* cmd) { command.scalar(&target_voltage, cmd); }
//void doMotor(char* cmd) { command.motor(&motor, cmd); }

void setup() {
  Wire.setPins(SDA_pin,SCL_pin);
  Wire.begin(8);
  Wire.onRequest(sendAngleEvent);
  Wire.onReceive(receiveEvent);

  // initialise magnetic sensor hardware
  sensor.init();
  // link the motor to the sensor
  motor.linkSensor(&sensor);

  // power supply voltage
  driver.voltage_power_supply = 12;
  driver.init();
  motor.linkDriver(&driver);

  // aligning voltage 
  motor.voltage_sensor_align = 8;
  // choose FOC modulation (optional)
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  // set motion control loop to be used
  motor.torque_controller = TorqueControlType::voltage;
  motor.controller = MotionControlType::torque;

  // use monitoring with serial 
  Serial.begin(115200);
  // comment out if not needed
  motor.useMonitoring(Serial);

  // initialize motor
  motor.init();
  // align sensor and start FOC
  motor.initFOC();
  motor.sensor_offset = motor.shaft_angle;

  // add target command T
  //command.add('T', doTarget, "target voltage");
  //command.add('M',doMotor,"motor");

  Serial.println(F("Motor ready."));
  delay(1000);
}

void loop() {
  motor.loopFOC();

  motor.move(target_voltage);

  //command.run();
}

void sendAngleEvent() {
  sensor.update();
  float abs_angle = sensor.getAngle() - motor.sensor_offset;
  float sendval = map(abs_angle*100, -angle_range_rad /0.02, angle_range_rad/0.02, 0,1023);
  I2C_writeAnything(sendval);
  //Serial.println(map(abs_angle*100, -angle_range_rad /0.02, angle_range_rad/0.02, 0,1023));
}

void receiveEvent(int howMany) {
  I2C_readAnything(force_inp);
  if(force_inp < -256){
    force_inp +=65535;
  }
  target_voltage = map(force_inp,-255,255,-700,700) / 100;
  //Serial.println(target_voltage);
  //Serial.println((float) map(force_inp,-255,255,-200,200) / 100);
}