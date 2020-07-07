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
double sausage = 0; //w
//char returncmd = '\0';



void Subscriber::positionCallback(const geometry_msgs::PoseStamped& pos) {
 //map planning algorithm that translates pose into a motor command
  toast = pos.pose.position.x;
  eggs = pos.pose.position.y;
  sausage= pos.pose.orientation.w;
  
}
//void Subscriber::cmdCallback(const std_msgs::String& cmd){
  //returncmd = cmd.data;
//}
Point Subscriber::getPosition(){
   Point location = Point(toast, eggs);
   return location;
  }
//char Subscriber::getCommand(){
 // return returncmd;
//}
double Subscriber::getHeading(){
  double heading = to_degrees(2* acos(sausage));
  return heading;
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
  
