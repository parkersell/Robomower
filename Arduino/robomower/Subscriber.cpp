#include "Arduino.h"
#include "Subscriber.h"
#include <math.h>



Subscriber::Subscriber():  pose_subscriber("/orb_slam2_rgbd/pose", &positionCallback,this)
{}
ros::NodeHandle nh;
geometry_msgs::PoseStamped pos;
geometry_msgs::Pose p;
std_msgs::String cmd;
double toast = 0; //x
double eggs = 0; //y
//orientation
double fruit = 0; //w
double cereal = 0; //x
double pancakes = 0;//y
double syrup =0; //z




void Subscriber::positionCallback(const geometry_msgs::PoseStamped& pos) {
  
  toast = pos.pose.position.x;
  eggs = pos.pose.position.y;
  
  fruit = pos.pose.orientation.w;
  cereal = pos.pose.orientation.x;
  pancakes = pos.pose.orientation.y;
  syrup = pos.pose.orientation.z;
  
}

Point Subscriber::getPosition(){
   Point location = Point(toast, eggs);
   return location;
  }


double Subscriber::getRoll(){
  double sinr_cosp = 2 * (fruit * cereal + pancakes * syrup);
  double cosr_cosp = 1 - 2 * (cereal * cereal + pancakes * pancakes);
  double roll = atan2(sinr_cosp, cosr_cosp);
  return roll;
}

  

void Subscriber::initSLAM(){
   nh.initNode();
  nh.subscribe(pose_subscriber);

  }


void Subscriber::spinOnceS(){
  nh.spinOnce();
  //delay(50);//with no delay we get position, but no motor control
//nh.spin();
  }
  
