#include "Arduino.h"
#include "Movement.h"
#include <math.h>
using namespace std;
//#include <ros.h>



Movement::Movement(){
 
  }


inline double Movement::to_radians(double degrees) {
    return degrees * (M_PI/180.0);
}

inline double Movement::to_degrees(double radians) {
    return radians * (180.0 / M_PI);
}

float Movement::clip(float n, float lower, float upper) {
return  n <= lower ? lower : n >= upper ? upper : n;
}


double Movement::goToPosition(double& leftpower, double& rightpower, double xpose, double ypose, double theta, double xtarget, double ytarget, double headingtarget, double rpm, double maxerror){
  double xdistance = xtarget - xpose; //Difference between target x and actual x
  double ydistance = ytarget - ypose; //Difference between target y and actual y
  double distance = hypot(xdistance, ydistance);

  if(distance > maxerror){
    distance = hypot(xdistance, ydistance);
    xdistance = xtarget - xpose; //Difference between target x and actual x
    ydistance = ytarget - ypose; //Difference between target y and actual y

   
/*
    //holonomic kinematics
    double movementAngle = to_degrees(atan2(xdistance, ydistance)); //Basically a unit circle where 0 degrees is forward.    
    double xcorrect = sin(to_radians(movementAngle)) * rpm; //Both these methods return how far off the robot is relative to the x and y coordinates regardless of heading
    double ycorrect = cos(to_radians(movementAngle)) * rpm;
    */
    
    double xcorrect = clip(xdistance,-rpm,rpm);
    double angleCorrection = headingtarget - theta;
    double kp = .25;
     rightpower = xcorrect * (cos(to_radians(angleCorrection)) + kp*sin(to_radians(angleCorrection)));
     leftpower = xcorrect * (cos(to_radians(angleCorrection)) - kp*sin(to_radians(angleCorrection)));
    return leftpower,rightpower;

  }

}
  
 
