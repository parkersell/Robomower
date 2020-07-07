#define USE_TEENSY_HW_SERIAL
#define HWSERIAL Serial1


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
unsigned long newTimer = micros();

//adafruit_bno055_offsets_t calibrationData;

String input = "";
String input2 = "";
double x = 0;
double y = 0;
Point p0 = Point(x, y);
Point p1 = Point(10, 0);
Point p2 = Point(3, 3);

//Motor leftMotor(33, 34, 7, A9, 36, 37, 4480);
//Motor rightMotor(14, 15, 6, A8, 38, 39, 4480);

void setup() {



  // put your setup code here, to run once:
  //Serial1.begin(115200);
  HWSERIAL.begin(57600);
  pinMode(LED_BUILTIN, OUTPUT);
  movement.initRobot();
}
void loop() {
  // put your main code here, to run repeatedly:


  //if (micros() - newTimer > 100000) {
    //newTimer =micros();
 /*   while (HWSERIAL.available()) {
     
      char temp = HWSERIAL.read();
      Serial.print(temp);
      //Serial.println("hi");
      if (temp == '\n') {
        if (input == 'd') {
          movement.disableM();
        } else if (input == 'e') {
          
          movement.enableM();

        }

        else if (input == 'x') {
          movement.goToPosition(0, p1, 0, 10, 1);
        }

        else {
          int commaIndex = input.indexOf(',');
          float speed = commaIndex == -1 ? input.toFloat() : input.substring(0, commaIndex).toFloat();
          //        headingSetPoint = speed;
          float turn = commaIndex == -1 ? 0 : input.substring(commaIndex + 1).toFloat();
          movement.setSpeedM(speed + turn, speed - turn);
        }
        input = "";
      } else input += temp;
    } */

      while (HWSERIAL.available()) {
     
      char temp = HWSERIAL.read();
      HWSERIAL.print(temp);
      //Serial.println("hi");
      if (temp == '\n') {
        if (input2 == 'd') {
          movement.disableM();
          
        } else if (input2 == 'e') {
          
          movement.enableM();

        }

        else if (input2 == 'x') {
          movement.goToPosition(p1, 0, 30, 2);
        }

        else {
          int commaIndex = input2.indexOf(',');
          float speed = commaIndex == -1 ? input2.toFloat() : input2.substring(0, commaIndex).toFloat();
          //        headingSetPoint = speed;
          float turn = commaIndex == -1 ? 0 : input2.substring(commaIndex + 1).toFloat();
          movement.setSpeedM(speed + turn, speed - turn);
        }
        input2 = "";
      } else input2 += temp;
    }
  


  if (micros() - printTimer > 50000) {


    Point p = movement.getPosition();

    //Serial.println(micros() - inchTimer);
    HWSERIAL.println(p.x);
    HWSERIAL.println(p.y);
   // movement.spinOnceM();
    //movement.enableM();
    //movement.goToPosition(0, p1, 0, 10, 1);
    //  Serial.println(movement.getSpeedM());

    printTimer = micros();

  }


  if (micros() - ledTimer > 100000) {
   movement.spinOnceM();
 
    ledTimer = micros();
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }

  movement.computeM();
}
