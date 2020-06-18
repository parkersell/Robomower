#include <ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
//include a list or array message for points of obstacles on point cloud

ros::NodeHandle nh;

geometry_msgs::PoseStamped pos;
geometry_msgs::Pose p;
//list variable

void positionCallback(const geometry_msgs::PoseStamped& pos) {
 //map planning algorithm that translates pose into a motor command
  String str = pos.pose.position.x;
  nh.loginfo("hi");
}

/*void objectAvoidanceCallback("list variable"){
  //
}*/


ros::Subscriber<geometry_msgs::PoseStamped> pose_subscriber("/orb_slam2_mono/pose", positionCallback);
//ros::Subscriber<geometry_msgs::Pose> pose_subscriber("/orb_slam2_mono/pose", positionCallback);
//ros::Subscriber<list> list_subscriber("echo", &objectAvoidanceCallback);//echo topic that displays list of x y z coordinates of the objects

void setup() {
  // put your setup code here, to run once:
 nh.initNode();
  nh.subscribe(pose_subscriber);
   ////nh.subscribe(list_subscriber);
}

void loop() {
  // put your main code here, to run repeatedly:
  nh.spinOnce();
  delay(500);
}
