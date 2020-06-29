#define USE_TEENSY_HW_SERIAL
#define HWSERIAL Serial


#include "Movement.h"
#include "Point.h"


//#include <Adafruit_BNO055.h>
//#include <Adafruit_Sensor.h>

//#include <EEPROM.h>


Movement movement;




//Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

//float heading = 0;
//float headingTurnFactor = 0;
//float headingSetPoint = 0;

//PID headingPID = PID(&heading, &headingTurnFactor, &headingSetPoint, 1, 2, 0, 10);

unsigned long ledTimer = micros();
unsigned long printTimer = micros();
unsigned long imuTimer = micros();
unsigned long stepTimer = micros();
unsigned long inchTimer = micros();
boolean inches = false;

//adafruit_bno055_offsets_t calibrationData;

String input = "";
double x = 0;
double y = 0;
Point p0 = Point(x,y);
Point p1 = Point(10, 10);
Point p2 = Point(3, 3);

void setup() {
  


  // put your setup code here, to run once:
  //Serial.begin(115200);
  HWSERIAL.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  movement.initRobot();
}
void loop() {
  // put your main code here, to run repeatedly:

//movement.mowThread(leftspeed,rightspeed,p0,0,p1,p1,p1,p1);
movement.goToPosition( p0,0,p1,0,50,1);
movement.goToPosition(p0,0,p2,0,50,1);
movement.goToPosition(p0,0,p2,0,50,1);
movement.goToPosition(p0,0,p2,0,50,1);
  while (HWSERIAL.available()) {
    p0 = Point(p0.x+1,p0.y+1);
    
    char temp = HWSERIAL.read();
    Serial.print(temp);
    if (temp == '\n') {
  movement.disableM();
      if (input == 'd') {
        movement.disableM();
      } else if (input == 'e') {
       movement.enableM();
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

      else if (input == 'x') {

      }

     
      else {
        int commaIndex = input.indexOf(',');
        float speed = commaIndex == -1 ? input.toFloat() : input.substring(0, commaIndex).toFloat();
        //        headingSetPoint = speed;
        float turn = commaIndex == -1 ? 0 : input.substring(commaIndex + 1).toFloat();
        movement.setSpeedM(speed+turn,speed-turn);
      }
      input = "";
    } else input += temp;
  }

 
  

  if (micros() - inchTimer > 7500000) {
   movement.disableM();
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
  //    hardware.leftMotor.setSpeed(headingTurnFactor);
  //    hardware.rightMotor.setSpeed(-headingTurnFactor);
  //  }

  if (micros() - printTimer > 10000) {
   
    Serial.println("hi");
    //Serial.println(micros() - inchTimer);
    printTimer = micros();
  
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

movement.computeM();
  //  if (heading != -1) headingPID.compute();
}
