#include "Arduino.h"
#include "Movement.h"
#include <math.h>
#include "Subscriber.h"
using namespace std;

//#include <ros.h>


Hardware hardware;
Subscriber subscriber;
Movement::Movement() {

}

void Movement::initRobot(){
  disableM();
  subscriber.initSlam();
  }

void Movement::enableM(){
    hardware.leftMotor.enable();
    hardware.rightMotor.enable();
  }
  
void Movement::disableM(){
    hardware.leftMotor.disable();
    hardware.rightMotor.disable();
  }

void Movement::setSpeedM(int leftpower, int rightpower){
  hardware.leftMotor.setSpeed(leftpower);
  hardware.rightMotor.setSpeed(rightpower);
  }
  
void Movement::computeM(){
    hardware.leftMotor.compute();
    hardware.rightMotor.compute();
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
  
void Movement::goToPosition(double theta, Point p2, double headingtarget, double rpm, double maxerror) {
  Point p1 = subscriber.getPosition();
  
  double leftpower = 0;
  double rightpower = 0;
  
  double xpose = p1.x;
  double ypose = p1.y;

  double xtarget = p2.x;
  double ytarget = p2.y;

  double xdistance = xtarget - xpose; //Difference between target x and actual x
  double ydistance = ytarget - ypose; //Difference between target y and actual y
  double distance = hypot(xdistance, ydistance);

  while(distance > maxerror) {
    
    
    distance = hypot(xdistance, ydistance);
    xdistance = xtarget - xpose; //Difference between target x and actual x
    ydistance = ytarget - ypose; //Difference between target y and actual y


    /*
        //holonomic kinematics
        double movementAngle = to_degrees(atan2(xdistance, ydistance)); //Basically a unit circle where 0 degrees is forward.
        double xcorrect = sin(to_radians(movementAngle)) * rpm; //Both these methods return how far off the robot is relative to the x and y coordinates regardless of heading
        double ycorrect = cos(to_radians(movementAngle)) * rpm;
    */

    double xcorrect = clip(xdistance, -rpm, rpm);
    double angleCorrection = headingtarget - theta;
    double kp = .25;
    rightpower = xcorrect * (cos(to_radians(angleCorrection)) + kp * sin(to_radians(angleCorrection)));
    leftpower = xcorrect * (cos(to_radians(angleCorrection)) - kp * sin(to_radians(angleCorrection)));
    setSpeedM(leftpower,rightpower);

  }

    setSpeedM(0,0);
 

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


  goToPosition(Pose, heading, bottomright, 90, 20, 1);    
  goToPosition( Pose, heading, topright, 0, 20, 1);
  goToPosition( Pose, heading, topleft, 270, 20, 1);
  goToPosition( Pose, heading, bottomleft, 0, 20, 1);


  topleft = Point(topleft.x - 5, topleft.y + 5);
  topright = Point(topright.x - 5, topright.y - 5);
  bottomright = Point(bottomright.x + 5, bottomright.y + 5);
  bottomleft = Point(bottomleft.x + 5, bottomleft.y - 5);

}
