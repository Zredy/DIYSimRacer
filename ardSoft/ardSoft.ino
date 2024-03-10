#include <HX711_ADC.h>
#include <Joystick.h>
#include <Wire.h>
#include <I2C_Anything.h>
#include <digitalWriteFast.h>

const int HX711_dout = 4; //mcu > HX711 dout pin
const int HX711_sck = 5; //mcu > HX711 sck pin
const int pedalPin1 = A0;
const int pedalPin2 = A1;


int s0 = 8;
int s1 = 9;
int s2 = 12;
int s3 = 13;

Gains gains[2];
EffectParams effectparams[2];
int32_t forces[2] = {0};
float angle_inp = 0;

int SIG_pin = A2;

HX711_ADC LoadCell(HX711_dout, HX711_sck);

Joystick_ Joystick(JOYSTICK_DEFAULT_REPORT_ID,JOYSTICK_TYPE_JOYSTICK,
  14, 0,                  // Button Count, Hat Switch Count
  true, true, false,     // X and Y, but no Z Axis
  true, true, true,   //  Rx, Ry, Rz
  false, false,          // No rudder or throttle
  false, false, false);    // No accelerator, brake, or steering

const int calVal_eepromAdress = 0;
unsigned long t = 0;
int pedalClutch,pedalAccelerator = 0;
long oldWheelPos  = -999;
int wheelPos = 512;
bool isOutOfRange = false;


void setup() {
  Wire.begin();
  //Serial.begin(57600); delay(10);
  Serial.println();
  Serial.println("Starting...");

  Joystick.setRxAxisRange(0, 1023);
  Joystick.setXAxisRange(-512,512);
  Joystick.setYAxisRange(0,1023);
  Joystick.setRyAxisRange(0, 1023);
  Joystick.setRzAxisRange(0, 1023);
  //Joystick.setSteeringRange(0,1023);

  Joystick.setGains(gains);
  Joystick.begin(true);

  LoadCell.begin();
  //LoadCell.setReverseOutput(); //uncomment to turn a negative output value to positive
  float calibrationValue; // calibration value (see example file "Calibration.ino")
  calibrationValue = 45.60; // uncomment this if you want to set the calibration value in the sketch

  unsigned long stabilizingtime = 2000; // preciscion right after power-up can be improved by adding a few seconds of stabilizing time
  boolean _tare = true; //set this to false if you don't want tare to be performed in the next step
  LoadCell.start(stabilizingtime, _tare);
  if (LoadCell.getTareTimeoutFlag()) {
    Serial.println("Timeout, check MCU>HX711 wiring and pin designations");
    while (1);
  }
  else {
    LoadCell.setCalFactor(calibrationValue); // set calibration value (float)
    Serial.println("Startup is complete");
  }

  pinMode(s0, OUTPUT); 
  pinMode(s1, OUTPUT); 
  pinMode(s2, OUTPUT); 
  pinMode(s3, OUTPUT); 
  pinMode(SIG_pin, INPUT_PULLUP);

  cli();
  TCCR3A = 0; //set TCCR1A 0
  TCCR3B = 0; //set TCCR1B 0
  TCNT3  = 0; //counter init
  OCR3A = 399;
  TCCR3B |= (1 << WGM32); //open CTC mode
  TCCR3B |= (1 << CS31); //set CS11 1(8-fold Prescaler)
  TIMSK3 |= (1 << OCIE3A);
  sei();
}

ISR(TIMER3_COMPA_vect){
  Joystick.getUSBPID();
}

void loop() {
  
  Wire.requestFrom(8, 4);    // request 4 bytes from peripheral device #8
  if (Wire.available()) {
    I2C_readAnything(angle_inp);
  }
  if(angle_inp > 1023){
    isOutOfRange = true;
    angle_inp = 1023;
  } else if(angle_inp<0){
    isOutOfRange = true;
    angle_inp = 0;
  } else {
    isOutOfRange = false;
  }
  angle_inp = constrain(angle_inp, 0, 1023);
  //Serial.println(angle_inp);
  Joystick.setXAxis(angle_inp - 512);
  pedalClutch = map(analogRead(pedalPin1),25,890,0,1023);
  pedalAccelerator = map(analogRead(pedalPin2),200,950,1023,0);
  Joystick.setYAxis(pedalAccelerator);
  //Serial.println(pedal2); //117-950
  //Serial.println(pedal1); //25-890
  //delay(50);

  static boolean newDataReady = 0;
  const int serialPrintInterval = 0; //increase value to slow down serial print activity

  // check for new data/start next conversion:
  if (LoadCell.update()) newDataReady = true;

  // get smoothed value from the dataset:
  if (newDataReady) {
    if (millis() > t + serialPrintInterval) {
      float i = LoadCell.getData();
      //Serial.print("Load_cell output val: ");
      //Serial.println(map(i,100, -8000, 0, 1023));

      Joystick.setRyAxis(map(i,100, -8000, 0, 1023));

      newDataReady = 0;
      t = millis();
    }
  }
  //Joystick data
  //Joystick.setRxAxis(pedalAccelerator);

  Joystick.setRzAxis(pedalClutch); 
  if(millis()%4 == 0){
    for(int i=0; i<16;i++){
      digitalWriteFast(s0, bitRead(i,0));
      digitalWriteFast(s1, bitRead(i,1));
      digitalWriteFast(s2, bitRead(i,2));
      digitalWriteFast(s3, bitRead(i,3));

      Joystick.setButton(i, !digitalRead(SIG_pin));
    }
  }
  

  //Joystick.sendState();
  effectparams[0].springMaxPosition = 512;
  effectparams[0].springPosition = angle_inp - 512; //-512-512
  effectparams[1].springMaxPosition = 255;
  effectparams[1].springPosition = 0;
  Joystick.setEffectParams(effectparams);
  Joystick.getForce(forces);

  //Serial.println(forces[0]);
  if(!isOutOfRange){
    Wire.beginTransmission(8);
    I2C_writeAnything(forces[0]);
    Wire.endTransmission();
  }
}