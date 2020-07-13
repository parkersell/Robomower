#define USE_TEENSY_HW_SERIAL
#define HWSERIAL Serial1


#include "Movement.h"
#include "Point.h"

#include "PreMo.h"



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
Point p1 = Point(30, -15);
Point p2 = Point(40, -15);
Point h1 = Point(50, -4);
Point h2 = Point(52, -14);
Point h3 = Point(60, -40);
Point h4 = Point(71, -100);
Point h5 = Point(81, -103);

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

//Motor leftMotor(33, 34, 7, A9, 36, 37, 4480);
//Motor rightMotor(14, 15, 6, A8, 38, 39, 4480);

void setup() {

  premo.setPathFollowSpeed(PATH_FOLLOW_SPEED);

  // put your setup code here, to run once:
  //Serial1.begin(115200);
  HWSERIAL.begin(57600);
  pinMode(LED_BUILTIN, OUTPUT);
  movement.initRobot();
}
void loop() {
  // put your main code here, to run repeatedly:


  premo.loop();
  while (HWSERIAL.available()) {

    char temp = HWSERIAL.read();
    HWSERIAL.print(temp);
    //Serial.println("hi");
    if (temp == '\n') {
      if (input2 == 'd') {
        premo.reset();
        movement.disableM();

      } else if (input2 == 'e') {

        movement.enableM();

      }


      else if (input2 == 'x') {
        movement.goToPosition(p1, 40, 2);
        movement.goToPosition(p2, 40, 2);

      }
      else if (input2 == 'y') {
        movement.goToPosition(h1, 20, 2);
        Serial1.println("H1 Done");
        movement.goToPosition(h2, 20, 2);
        Serial1.println("H2 Done");
         movement.goToPosition(h3, 20, 2);
        Serial1.println("H3 Done");
         movement.goToPosition(h4, 20, 2);
        Serial1.println("H4 Done");

      }

      else if (input2 == 'u') {
        movement.goToPosition(h2, 20, 2);
        Serial1.println("H2 Done");

      }

      else if (input2 == 'i') {
        movement.goToPosition(h3, 20, 2);
        Serial1.println("H3 Done");

      }

      else if (input2 == 'o') {
        movement.goToPosition(h4, 20, 2);
        Serial1.println("H4 Done");
      }
      else if (input2 == 'p') {
        premo.goTo(10, 0);

      }
      else if (input2 == 'f') {
        premo.forward(10);

      }
      else if (input2 == 'r') {
        premo.reverse(10);

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
    //Serial.println(micros() - inchTimer);
    HWSERIAL.println(point.x);
    HWSERIAL.println(point.y);
    //HWSERIAL.print("yaw");
    //HWSERIAL.println(sub.getYaw());

    //premo.getLocationData();


    /*  if(sub.tracking()){
        HWSERIAL.println("true");
      }
      else {
        HWSERIAL.println("false");
      }*/


    //HWSERIAL.println(*h);
    //  premo.printPath();
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
