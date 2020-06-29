#include "Arduino.h"
#include "Subscriber.h"



Subscriber::Subscriber() {
}

ros::NodeHandle nh;
geometry_msgs::PoseStamped pos;
geometry_msgs::Pose p;

void Subscriber::positionCallback(const geometry_msgs::PoseStamped& pos) {
 //map planning algorithm that translates pose into a motor command
  String str = pos.pose.position.x;
  nh.loginfo("hi");
}

Point Subscriber::getPosition(){
   Point location = Point(pos.pose.position.x, pos.pose.position.y);
   return location;
  }

void Subscriber::initSLAM(){
   nh.initNode();
  nh.subscribe(pose_subscriber);
  }
