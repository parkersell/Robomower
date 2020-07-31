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
int Movement::input() {
  String input2 = "";
  if (HWSERIAL.available()) {

    char temp = HWSERIAL.read();
    HWSERIAL.print(temp);
    //Serial.println("hi");
    if (temp == '\n') {
      if (input2 == 'd') {
        disableM();
        disableGrass();
      } else if (input2 == 'e') {
        enableM();
      } else if (input2 == 'g') {
        enableGrass();
      } else if (input2 == 'h') {
        disableGrass();

      } else if (input2 == 'z') {
        return 0;
      }
      input2 = "";

    } else input2 += temp;
  }
  return 1;
}

void Movement::spinOnceM() {
  subscriber.spinOnceS();
}
void Movement::initRobot() {
  disableM();
  subscriber.initSLAM();
}

void Movement::enableM() {
  hardware.leftMotor.enable();
  hardware.rightMotor.enable();
}

void Movement::enableGrass() {
  hardware.grassMotor.enable();
  hardware.grassMotor.setSpeed(100);
}

void Movement::disableGrass() {
  hardware.grassMotor.disable();
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
  hardware.grassMotor.setCurrentLimit(25);
  hardware.grassMotor.compute();
}

Point Movement::getPosition() {
  Point p1 = subscriber.getPosition();
  return p1;
}

double Movement::getIMUM() {
  return subscriber.getIMU();
}

void Movement::resetIMUM() {
  subscriber.resetIMU();
}

inline double Movement::to_radians(double degrees) {
  return degrees * (M_PI / 180.0);
}

inline double Movement::to_degrees(double radians) {
  return radians * (180.0 / M_PI);
}

float Movement::clip(float n, float lower, float upper) {
  return n <= lower ? lower : n >= upper ? upper : n;
}

void Movement::goToPosition(Point p2, double rpm, double maxerror) {
  Point p1 = subscriber.getPosition();

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
  bool state;
  double heading = atan2(ydistance, xdistance);
  heading = to_degrees(heading);
  String input2 = "";

  while (distance > maxerror) {

    if (HWSERIAL.available()) {

      char temp = HWSERIAL.read();
      HWSERIAL.print(temp);
      //Serial.println("hi");
      if (temp == '\n') {
        if (input2 == 'd') {
          disableM();
          disableGrass();
        } else if (input2 == 'e') {
          enableM();
        } else if (input2 == 'g') {
          enableGrass();
        } else if (input2 == 'h') {
          disableGrass();

        } else if (input2 == 'z') {
          break;
        }
        input2 = "";

      } else input2 += temp;
    }
    float laser = subscriber.getLaser();
    spinOnceM();
    p1 = subscriber.getPosition();
    theta = subscriber.getYaw();
    xpose = p1.x;
    ypose = p1.y;

    xdistance = xtarget - xpose; //Difference between target x and actual x
    ydistance = ytarget - ypose; //Difference between target y and actual y
    distance = hypot(xdistance, ydistance);
    heading = atan2(ydistance, xdistance);
    heading = to_degrees(heading);

    Serial1.print("x: ");
    Serial1.println(xpose);
    Serial1.print("y: ");
    Serial1.println(ypose);
    Serial1.print("dist: ");
    Serial1.println(distance);
    //    Serial1.print("theta: ");
    //    Serial1.println(theta);
    //    Serial1.print("laser: ");
    //    Serial1.println(laser);

    double dcorrect = distance * 2.5;
    dcorrect = clip(dcorrect, -rpm, rpm);
    double angleCorrection = heading - theta;
    //    Serial1.println(angleCorrection);
    double kp = .55;

    if (isnan(laser)) {
      rightpower = 0;
      leftpower = 0;
    } else if (!subscriber.tracking()) {
      Serial1.println("LOST");
      //      double imuyaw = getIMUM();
      //      int target = int(theta + 360) % 360;
      //      imuyaw = int(imuyaw + 360) % 360;
      //      double angleLost = imuyaw - target;
      //      if (abs(angleLost) >180){
      //        angleLost=-angleLost;
      //      }
      //      if (angleLost < 0) {
      //        Serial1.println("Left");
      rightpower = 3;
      leftpower = -3; //left
      //      }
      //      else if (angleLost > 0) {
      //        Serial1.println("Right");
      //        rightpower = -5;
      //        leftpower = 5;
      //      }
    } else {
      rightpower = dcorrect * (cos(to_radians(angleCorrection)) + kp * sin(to_radians(angleCorrection)));
      leftpower = dcorrect * (cos(to_radians(angleCorrection)) - kp * sin(to_radians(angleCorrection)));
    }
    setSpeedM(leftpower, rightpower);
    computeM();
  }
  setSpeedM(0, 0);
  computeM();
  return;

}

void Movement::turnTo(double theta, int maxerror, double kp) {
  double firstimuyaw = getIMUM();
  double target = int(theta + 360) % 360;
  double imuyaw = int(firstimuyaw + 360) % 360;

  double firstangleDiff = imuyaw - target;

  double power = firstangleDiff * kp; //Power proportional to angledifference
  power = clip(power, -7.5, 7.5); //Set max and min

  double rightpower;
  double leftpower;
  String input2 = "";
  while (abs(firstangleDiff) > maxerror) {

    if (HWSERIAL.available()) {

      char temp = HWSERIAL.read();
      HWSERIAL.print(temp);
      //Serial.println("hi");
      if (temp == '\n') {
        if (input2 == 'd') {
          disableM();
          disableGrass();
        } else if (input2 == 'e') {
          enableM();
        } else if (input2 == 'g') {
          enableGrass();
        } else if (input2 == 'h') {
          disableGrass();

        } else if (input2 == 'z') {
          break;
        }
        input2 = "";

      } else input2 += temp;
    }

    firstimuyaw = getIMUM();

    target = int(theta + 360) % 360;
    imuyaw = int(firstimuyaw + 360) % 360;

    Serial1.print("target: ");
    Serial1.println(target);
    Serial1.print("imuyaw: ");
    Serial1.println(imuyaw);

    firstangleDiff = imuyaw - target;
    double angleDiff = firstangleDiff;
    if (abs(firstangleDiff) > 180) {
      angleDiff = -firstangleDiff;
    }
    power = angleDiff * kp;
    power = -clip(power, -7.5, 7.5);

    Serial1.print("angleDiff:");
    Serial1.println(angleDiff);

    rightpower = power;
    leftpower = -power;
    /* Serial1.print("leftpower: ");
      Serial1.println(leftpower);
      Serial1.print("rightpower: ");
      Serial1.println(rightpower); */
    setSpeedM(leftpower, rightpower);
    computeM();
  }
  setSpeedM(0, 0);
  computeM();
}

void Movement::curveTo(boolean left, int maxerror) {
  double firstimuyaw = getIMUM();
  double target;

  double theta = firstimuyaw + 180;
  String input2 = "";
  double rightpower;
  double leftpower;
  double power;
  double firstangleDiff;
  double kp = .4;

  double imuyaw = int(firstimuyaw + 360) % 360;
  target = int(theta + 360) % 360;
  firstangleDiff = target - imuyaw;
  while (abs(firstangleDiff) > maxerror) {

    if (HWSERIAL.available()) {

      char temp = HWSERIAL.read();
      HWSERIAL.print(temp);
      //Serial.println("hi");
      if (temp == '\n') {
        if (input2 == 'd') {
          disableM();
          disableGrass();
        } else if (input2 == 'e') {
          enableM();
        } else if (input2 == 'g') {
          enableGrass();
        } else if (input2 == 'h') {
          disableGrass();

        } else if (input2 == 'z') {
          break;
        }
        input2 = "";

      } else input2 += temp;
    }

    firstimuyaw = getIMUM();
    imuyaw = int(firstimuyaw + 360) % 360;
    Serial1.print("target: ");
    Serial1.println(target);
    Serial1.print("imuyaw: ");
    Serial1.println(imuyaw);

    firstangleDiff = target - imuyaw;
    double angleDiff = abs(firstangleDiff);
    power = angleDiff * kp;
    power = clip(power, 0, 10);

    

    if (left) {
      rightpower = power;
      leftpower = 0;
    } else {
      rightpower = 0;
      leftpower = power;
    }
    /* Serial1.print("leftpower: ");
      Serial1.println(leftpower);
      Serial1.print("rightpower: ");
      Serial1.println(rightpower); */
    setSpeedM(leftpower, rightpower);
    computeM();
  }
  setSpeedM(0, 0);
  computeM();
}
void Movement::mowLawn(Point bottomright, Point topright, Point topleft, Point bottomleft, int rpm) {

  //  goToPosition(bottomright, rpm, 2);

  while (topright.y <= topleft.y || bottomright.y <= bottomleft.y || bottomleft.x <= topleft.x || bottomright.x <= topright.x) {
    goToPosition(topright, rpm, 2);
    turnTo(90, 4, .2);

    goToPosition(topleft, rpm, 2);
    turnTo(180, 4, .2);

    goToPosition(bottomleft, rpm, 2);
    turnTo(270, 4, .2);
    goToPosition(bottomright, rpm, 2);
    turnTo(0, 4, .2);

    topleft = Point(topleft.x - 8, topleft.y - 8);
    topright = Point(topright.x - 8, topright.y + 9);
    bottomright = Point(bottomright.x + 8, bottomright.y + 8);
    bottomleft = Point(bottomleft.x + 8, bottomleft.y - 8);
  }
  disableM();

}

void Movement::sTurn(Point bottomright, Point topleft, int rpm) {

  if ((topleft.x - bottomright.x) > (topleft.y - bottomright.y)) {
    Point mowPoint = Point(topleft.x, bottomright.y);
    while (bottomright.y <= topleft.y) {

      goToPosition(mowPoint, rpm, 2);
      curveTo(true, 3);
      mowPoint = Point(bottomright.x, bottomright.y + 9);
      goToPosition(mowPoint, rpm, 2);
      curveTo(false, 3);
      mowPoint = Point(topleft.x, bottomright.y + 9);
    }
  }

  else {
    Point mowPoint = Point(bottomright.x, topleft.y);
    while (bottomright.x <= topleft.x) {

      goToPosition(mowPoint, rpm, 2);
      curveTo(false, 3);
      mowPoint = Point(bottomright.x + 9, topleft.y);
      goToPosition(mowPoint, rpm, 2);
      curveTo(true, 3);
      mowPoint = Point(bottomright.x + 9, bottomright.y);
    }
  }
  disableM();

}
