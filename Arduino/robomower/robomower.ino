#define USE_TEENSY_HW_SERIAL
#define HWSERIAL Serial1

#include "Movement.h"

#include "Point.h"
//

//#include <EEPROM.h>

Movement movement;
Subscriber sub;

unsigned long ledTimer = micros();
unsigned long printTimer = micros();
unsigned long imuTimer = micros();
unsigned long stepTimer = micros();
unsigned long inchTimer = micros();
unsigned long newTimer = micros();

String input = "";
String input2 = "";
double x = 0;
double y = 0;
Point p0 = Point(x, y);
Point p1 = Point(20, 0);
Point p2 = Point(30, 0);

Point s1 = Point(0, 0);
Point s2 = Point(30, 0);
Point s3 = Point(30, 30);
Point s4 = Point(0, 30);

Point t1 = Point(0, 0);
Point t2 = Point(30, 20);

void setup() {

  // put your setup code here, to run once:
  //Serial1.begin(115200);
  HWSERIAL.begin(57600);
  pinMode(LED_BUILTIN, OUTPUT);
  movement.initRobot();
}
void loop() {
  // put your main code here, to run repeatedly:

  // sub.sendIMU(); //Uncomment for IMU topic

  while (HWSERIAL.available()) {

    char temp = HWSERIAL.read();
    HWSERIAL.print(temp);
    //Serial.println("hi");
    if (temp == '\n') {
      if (input2 == 'd') {
        // premo.reset();
        movement.disableM();

        movement.disableGrass();

      } 
      else if (input2 == 'e') {
        movement.enableM();
      } 
      else if (input2 == 'x') {
        //movement.goToPosition(p1, 40, 2);
        movement.goToPosition(p2, 40, 2);
      } 
      else if (input2 == 't') {
        movement.turnTo(90, 2, .2);
      } 
      else if (input2 == 'r') {
        movement.resetIMUM(); //Reset IMU

      } 
      else if (input2 == 'i') {
        movement.setSpeedM(30, 30);
      } 
      else if (input2 == 'k') {
        movement.setSpeedM(-30, -30);
      } 
      else if (input2 == 'j') {
        movement.setSpeedM(-10, 10);
      } 
      else if (input2 == 'l') {
        movement.setSpeedM(10, -10);
      } 
      else if (input2 == 'm') {
        movement.mowLawn(s1, s2, s3, s4, 15);
      } 
      else if (input2 == 's') {
        movement.sTurn(t1, t2, 15);
      } 
      else if (input2 == 'g') {
        movement.enableGrass();
      } 
      else if (input2 == 'h') {
        movement.disableGrass();
      } 
      else if (input2 == 'c') {
        movement.curveTo(true,3);
      }
      else if (input2 == 'v') {
        movement.curveTo(false,3);
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

    Point point = movement.getPosition();
    double laser = sub.getLaser();
    HWSERIAL.println(movement.getIMUM());

    HWSERIAL.println(point.x);
    HWSERIAL.println(point.y);
    //    HWSERIAL.print("yaw");
    //    HWSERIAL.println(sub.getYaw());
        HWSERIAL.println("laser");
        HWSERIAL.println(laser);

    // sub.tracking();

    if (sub.tracking()) {
      HWSERIAL.println("true");
    } else {
      HWSERIAL.println("false");
    }

    printTimer = micros();

  }

  if (micros() - ledTimer > 100000) {
    movement.spinOnceM();

    ledTimer = micros();
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }

  movement.computeM();
}
