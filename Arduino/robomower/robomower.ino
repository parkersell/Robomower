
#define HWSERIAL Serial

#include "Motor.h"
#include "PID.h"

//#include <Adafruit_BNO055.h>
//#include <Adafruit_Sensor.h>

//#include <EEPROM.h>

Motor leftMotor(33, 34, 18, A9, 36, 37, 4480);
Motor rightMotor(14, 15, 19, A8, 38, 39, 4480);

//Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

//float heading = 0;
//float headingTurnFactor = 0;
//float headingSetPoint = 0;

//PID headingPID = PID(&heading, &headingTurnFactor, &headingSetPoint, 1, 2, 0, 10);

unsigned long ledTimer = micros();
unsigned long printTimer = micros();
unsigned long imuTimer = micros();
unsigned long stepTimer = micros();

//adafruit_bno055_offsets_t calibrationData;

String input = "";

void setup() {
  // put your setup code here, to run once:
//  Serial.begin(115200);
  HWSERIAL.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  rightMotor.invert(true);

//  if(!bno.begin()) {
//    Serial.println("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
//    while(1);
//  }
//
//  // Restore IMU calibration data on startup.
//  EEPROM.get(0, calibrationData);
//  bno.setSensorOffsets(calibrationData);
//
//  bno.setExtCrystalUse(true);
//
//  headingPID.setLimit(-20, 20);
//  headingPID.setPOnM(true);
//  headingPID.setCircularInputMax(360);
//  headingPID.setDeadband(2.5);

  leftMotor.enable();
  rightMotor.enable();
}

void loop() {
  // put your main code here, to run repeatedly:

  while (HWSERIAL.available()) {
    char temp = HWSERIAL.read();
    Serial.print(temp);
    if (temp == '\n') {
      if (input == 'd') {
        leftMotor.disable();
        rightMotor.disable();
      } else if (input == 'e') {
        leftMotor.enable();
        rightMotor.enable();
      }
      
//      else if (input == 'w') {0
//        Serial.println("Writing BNO055 Calibration to EEPROM...");
//        if (bno.getSensorOffsets(calibrationData)) {
//          EEPROM.put(0, calibrationData);
//          Serial.println("BNO055 calibration written to EEPROM ADDR 0");
//        } else {
//          Serial.println("BNO055 is not fully calibrated yet");
//        }
//      }
      else {
        int commaIndex = input.indexOf(',');
        float speed = commaIndex == -1 ? input.toFloat() : input.substring(0, commaIndex).toFloat();
//        headingSetPoint = speed;
        float turn = commaIndex == -1 ? 0 : input.substring(commaIndex + 1).toFloat();
        leftMotor.setSpeed(speed + turn);
        rightMotor.setSpeed(speed - turn);
      }
      input = "";
    } else input += temp;
  }
//
//  if (micros() - imuTimer > 10000) {    
//    imuTimer = micros();
//    byte system, gyro, accel, mag;
//    bno.getCalibration(&system, &gyro, &accel, &mag);
////    if (mag) {
////      headingPID.setLimit(-100, 100);      
////    } else {
////      headingPID.setLimit(-10, 10);
////    }
//    if (true || system) {
//      sensors_event_t event;
//      bno.getEvent(&event);
//      heading = event.orientation.x;
//    } else {
//      headingSetPoint = 0;
//      headingTurnFactor = 25;
//    }
//    leftMotor.setSpeed(headingTurnFactor);
//    rightMotor.setSpeed(-headingTurnFactor);
//  }

  if (micros() - printTimer > 10000) {
    Serial.println(leftMotor.getSpeed());
     Serial.println(rightMotor.getSpeed());
     
    printTimer = micros();
//    Serial.println(heading);
//    byte system, gyro, accel, mag;
//    bno.getCalibration(&system, &gyro, &accel, &mag);
//    Serial.print("Heading: ");
//    Serial.print(heading);
//    Serial.print("\tSys: ");
//    Serial.print(system);
//    Serial.print("\tGyro: ");
//    Serial.print(gyro);
//    Serial.print("\tAccel: ");
//    Serial.print(accel);
//    Serial.print("\tMag: ");
//    Serial.println(mag);
  }

//  if (micros() - stepTimer > 4000000) {
//    stepTimer = micros();
//    if (headingSetPoint == 90) headingSetPoint = 250;
//    else headingSetPoint = 90;
//  }

  if (micros() - ledTimer > 100000) {
    ledTimer = micros();
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }
  
  leftMotor.compute();
  rightMotor.compute();
//  if (heading != -1) headingPID.compute();
}
