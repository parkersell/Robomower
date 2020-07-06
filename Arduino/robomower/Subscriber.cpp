#include "Arduino.h"
#include "Subscriber.h"



Subscriber::Subscriber():  pose_subscriber("/orb_slam2_rgbd/pose", &positionCallback,this)
{}
ros::NodeHandle nh;
geometry_msgs::PoseStamped pos;
geometry_msgs::Pose p;
std_msgs::String cmd;
double toast = 0; //x
double eggs = 0; //y
//char returncmd = '\0';



void Subscriber::positionCallback(const geometry_msgs::PoseStamped& pos) {
 //map planning algorithm that translates pose into a motor command
  toast = pos.pose.position.x;
  eggs = pos.pose.position.y;
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

  

void Subscriber::initSLAM(){
   nh.initNode();
  nh.subscribe(pose_subscriber);

  }


void Subscriber::spinOnceS(){
  nh.spinOnce();
  //delay(50);//with no delay we get position, but no motor control
//nh.spin();
  }
  
