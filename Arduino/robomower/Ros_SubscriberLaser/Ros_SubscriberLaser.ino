#include <ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32.h>
//include a list or array message for points of obstacles on point cloud

ros::NodeHandle nh;

sensor_msgs::LaserScan msg;
//list variable

void positionCallback(const std_msgs::Float32& msg) {
  //double scandistance = 39.37 * (msg.ranges[360]);
  double scandistance = msg;
}

/*void objectAvoidanceCallback("list variable"){
  //
}*/


ros::Subscriber<sensor_msgs::LaserScan> pose_subscriber("scan", positionCallback);

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
