#include "Arduino.h"
#include "Movement.h"
#include <math.h>
#include "Subscriber.h"
#define USE_TEENSY_HW_SERIAL
#define HWSERIAL Serial1
using namespace std;

//#include <ros.h>


Hardware hardware;
Subscriber subscriber;
Movement::Movement() {

}


void Movement::spinOnceM() {
  subscriber.spinOnceS();
}
void Movement::initRobot() {
  disableM();
  subscriber.initSLAM();
}
float Movement::getSpeedM() {
  float h = hardware.leftMotor.getSetSpeed();
  return h;
}




void Movement::enableM() {
  hardware.leftMotor.enable();
  hardware.rightMotor.enable();
}

void Movement::disableM() {
  hardware.leftMotor.disable();
  hardware.rightMotor.disable();
}

void Movement::setSpeedM(int leftpower, int rightpower) {
  hardware.leftMotor.setSpeed(leftpower);
  hardware.rightMotor.setSpeed(rightpower);
}

void Movement::computeM() {
  hardware.leftMotor.compute();
  hardware.rightMotor.compute();
}


Point Movement::getPosition() {
  Point p1 = subscriber.getPosition();
  return p1;
}

inline double Movement::to_radians(double degrees) {
  return degrees * (M_PI / 180.0);
}

inline double Movement::to_degrees(double radians) {
  return radians * (180.0 / M_PI);
}

float Movement::clip(float n, float lower, float upper) {
  return  n <= lower ? lower : n >= upper ? upper : n;
}

void Movement::goToPosition(Point p2, double headingtarget, double rpm, double maxerror) {
  Point p1  = subscriber.getPosition();

  double leftpower = 0;
  double rightpower = 0;

  double xpose = p1.x;
  double ypose = p1.y;
  double theta = subscriber.getYaw();

  double xtarget = p2.x;
  double ytarget = p2.y;

  double xdistance = xtarget - xpose; //Difference between target x and actual x
  double ydistance = ytarget - ypose; //Difference between target y and actual y
  double distance = hypot(xdistance, ydistance);
  String input2 = "";

  while (distance > maxerror) {

    if (HWSERIAL.available()) {

      char temp = HWSERIAL.read();
      HWSERIAL.print(temp);
      //Serial.println("hi");
      if (temp == '\n') {
        if (input2 == 'd') {
          disableM();
        } else if (input2 == 'e') {
          enableM();
        }
        input2 = "";

      } else input2 += temp;
    }

    spinOnceM();
    p1  = subscriber.getPosition();
    theta = subscriber.getYaw();
    xpose = p1.x;
    ypose = p1.y;

    distance = hypot(xdistance, ydistance);
    xdistance = xtarget - xpose; //Difference between target x and actual x
    ydistance = ytarget - ypose; //Difference between target y and actual y
    Serial1.print("x: ");
    Serial1.println(xdistance);
    Serial1.print("y: ");
    Serial1.println(ydistance);
    Serial1.print("dist: ");
    Serial1.println(distance);
    Serial1.print("theta: ");
    Serial1.println(theta);


    double xcorrect = clip(xdistance, -rpm, rpm);
    double ycorrect = clip(ydistance, -rpm, rpm);
    double angleCorrection = headingtarget - theta;
    double yangleCorrection = -90 - theta;
    double kp = .5;

    if (ydistance > maxerror) {
      rightpower = -ycorrect * (cos(to_radians(yangleCorrection)) + kp * sin(to_radians(yangleCorrection)));
      leftpower = -ycorrect * (cos(to_radians(yangleCorrection)) - kp * sin(to_radians(yangleCorrection)));
    }

    else {
      rightpower = xcorrect * (cos(to_radians(angleCorrection)) - kp * sin(to_radians(angleCorrection)));
      leftpower = xcorrect * (cos(to_radians(angleCorrection)) + kp * sin(to_radians(angleCorrection)));

    }
    setSpeedM(leftpower, rightpower);
    computeM();
  }
  setSpeedM(0, 0);
  computeM();
  return;

}

void Movement::mowLawn(double heading, Point topleft, Point topright, Point bottomleft, Point bottomright) {
  Point Pose = subscriber.getPosition();

  double points[] = {topleft.x, topleft.y, topright.x, topright.y, bottomleft.x, bottomleft.y, bottomright.x, bottomright.y};

  //Point Pose, double heading, Point topleft, Point topright, Point bottomleft, Point bottomright
  /*
      double xMin = points[0];
      double xMax = points[0];
      double yMin = points[1];
      double yMax = points[1];
      for (int a = 0; a < 8; a = a + 2) {
       if(points[a] < xMin){xMin = points[a];}
       if(points[a] > xMax){xMax = points[a];}
      }
      for (int a = 1; a < 8; a = a + 2) {
       if(points[a] < yMin){yMin = points[a];}
       if(points[a] > yMax){yMax = points[a];}
    } */

  //assume start in bottom left corner


  goToPosition(bottomright, 90, 20, 1);
  goToPosition(topright, 0, 20, 1);
  goToPosition(topleft, 270, 20, 1);
  goToPosition(bottomleft, 0, 20, 1);


  topleft = Point(topleft.x - 5, topleft.y + 5);
  topright = Point(topright.x - 5, topright.y - 5);
  bottomright = Point(bottomright.x + 5, bottomright.y + 5);
  bottomleft = Point(bottomleft.x + 5, bottomleft.y - 5);

}
