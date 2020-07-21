#define USE_TEENSY_HW_SERIAL
#define HWSERIAL Serial1


#include "Movement.h"
#include "Point.h"





//#include <Adafruit_BNO055.h>
//#include <Adafruit_Sensor.h>

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


/*
  // PID TUNING
  // PID for path following (for turning when followiung path)
  const double KP = 50;
  const double KI = 0;
  const double KD = 1;

  // ROBOT MEASUREMENTS]
  const float RADIUS = 7.75; // wheel radius in mm
  const float LENGTH = 20; // wheel base length in mm //11.5


  // PATH FOLLOWING SPEED
  // Target speed of path following in percentage.
  const int PATH_FOLLOW_SPEED = 20;

  PreMo premo(RADIUS, LENGTH, KP, KI, KD);
*/

void setup() {

  //premo.setPathFollowSpeed(PATH_FOLLOW_SPEED);

  // put your setup code here, to run once:
  //Serial1.begin(115200);
  HWSERIAL.begin(57600);
  pinMode(LED_BUILTIN, OUTPUT);
  movement.initRobot();
}
void loop() {
  // put your main code here, to run repeatedly:


  //  premo.loop();
  while (HWSERIAL.available()) {

    char temp = HWSERIAL.read();
    HWSERIAL.print(temp);
    //Serial.println("hi");
    if (temp == '\n') {
      if (input2 == 'd') {
        // premo.reset();
        movement.disableM();
        movement.disableGrass();


      } else if (input2 == 'e') {

        movement.enableM();

      }


      else if (input2 == 'x') {
        movement.goToPosition(p1, 20, 2);
         movement.goToPosition(p2, 40, 2);

      }
      else if (input2 == 'g') {
        movement.enableGrass();
      }

      else if (input2 == 'h') {
        movement.disableGrass();

      }


      else {
        /*
          int commaIndex = input2.indexOf(',');
          int dashIndex = input2.indexOf('-');
          int slashIndex = input2.indexOf('/');
          float kp = commaIndex == -1 ? input2.toFloat() : input2.substring(0, commaIndex).toFloat();
          float ki = commaIndex == -1 ? 0 : input2.substring(commaIndex + 1).toFloat();
          float kd = dashIndex == -1 ? 0 : input2.substring(dashIndex + 1).toFloat();
          float sp = slashIndex == -1 ? 0 : input2.substring(slashIndex + 1).toFloat();
          premo.setPIDPathFollowing(kp, ki, kd);
          premo.setPathFollowSpeed(sp);
          HWSERIAL.print("kp ");
          HWSERIAL.println(kp);
          HWSERIAL.print("ki ");
          HWSERIAL.println(ki);
          HWSERIAL.print("kd ");
          HWSERIAL.println(kd);
        */

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

    HWSERIAL.println(point.x);
    HWSERIAL.println(point.y);
    HWSERIAL.println(laser);
    //HWSERIAL.print("yaw");
    //HWSERIAL.println(sub.getYaw());

    //premo.getLocationData();
   // sub.tracking();
    
         if(sub.tracking()){
            HWSERIAL.println("true");
          }
          else {
            HWSERIAL.println("false");
          } 



    //  premo.printPath();
    // movement.spinOnceM();
    //movement.enableM();
    //movement.goToPosition(0, p1, 0, 10, 1);



    printTimer = micros();

  }


  if (micros() - ledTimer > 100000) {
    movement.spinOnceM();

    ledTimer = micros();
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }

  
  movement.computeM();
}
