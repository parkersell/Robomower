#include "Arduino.h"
#include "Subscriber.h"
#include <math.h>
#define USE_TEENSY_HW_SERIAL
#define HWSERIAL Serial1


Subscriber::Subscriber():  pose_subscriber("/orb_slam2_rgbd/pose", &positionCallback, this), laser_subscriber("/distance", &laserCallback, this)
{}

ros::NodeHandle nh;

geometry_msgs::PoseStamped pos;
geometry_msgs::Pose p;
std_msgs::Float32 msg;

std_msgs::String cmd;
double toast = 0; //x
double eggs = 0; //y
//orientation
double fruit = 0; //w
double cereal = 0; //x
double pancakes = 0;//y
double syrup = 0; //z
double heading = 0;

float scandistance = 0;

unsigned long settTimer = millis();
unsigned long trackTimer = millis();
double oldtoast = 0;
double oldeggs = 0;
boolean state;


void Subscriber::positionCallback(const geometry_msgs::PoseStamped& pos) {
  toast = pos.pose.position.x;
  eggs = pos.pose.position.y;

  fruit = pos.pose.orientation.w;
  cereal = pos.pose.orientation.x;
  pancakes = pos.pose.orientation.y;
  syrup = pos.pose.orientation.z;
  heading = getYaw();
}
bool Subscriber::tracking() {

  if (millis() - settTimer > 500) {
    if (oldtoast == toast && oldeggs == eggs) {
      state = false;
       //HWSERIAL.println("false");
    }
    else {
      oldtoast = toast;
      oldeggs = eggs;
      state = true;
      // HWSERIAL.println("true");
    }
    settTimer = millis();
    return state;
  }
  else {
   return state;
  }

}


void Subscriber::laserCallback(const std_msgs::Float32& msg) {

  //scandistance = 39.37 * (msg.ranges[360]);
    scandistance = msg.data;
  /*
    float[] regions = {
    39.37 * min(msg.ranges[0:143]),
    39.37 * min(msg.ranges[144:287]),
    39.37 * min(msg.ranges[288:431]),
    39.37 * min(msg.ranges[432:575]),
    39.37 * min(msg.ranges[576:713]),

    }*/
  //rospy.loginfo(regions)
}

double Subscriber::getLaser() {
  double laser = scandistance;
  return laser;
}
Point Subscriber::getPosition() {
  Point location = Point(toast, eggs);
  return location;
}

double Subscriber::getRoll() {
  double sinr_cosp = 2 * (fruit * cereal + pancakes * syrup);
  double cosr_cosp = 1 - 2 * (cereal * cereal + pancakes * pancakes);
  double roll = atan2(sinr_cosp, cosr_cosp);
  return roll;
}

double Subscriber::getYaw() {
  double siny_cosp = 2 * (fruit * syrup + cereal * pancakes);
  double cosy_cosp = 1 - 2 * (pancakes * pancakes + syrup * syrup);
  double yaw = atan2(siny_cosp, cosy_cosp);
  yaw = to_degrees(yaw);
  return yaw;
}
double* Subscriber::getXPointer() {
  return &toast;
}

double* Subscriber::getYPointer() {
  return &eggs;
}
double* Subscriber::getHeadingPointer() {

  return &heading;
}
double Subscriber::getX() {
  return toast;
}

double Subscriber::getY() {
  return eggs;
}
double Subscriber::getHeading() {
  heading = heading * DEG_TO_RAD;
  return heading;
}
void Subscriber::initSLAM() {
  nh.initNode();
 nh.subscribe(pose_subscriber);
nh.subscribe(laser_subscriber);

}


void Subscriber::spinOnceS() {
  nh.spinOnce();
  //delay(50);//with no delay we get position, but no motor control
  //nh.spin();
}
