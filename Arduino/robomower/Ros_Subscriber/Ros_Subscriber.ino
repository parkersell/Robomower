#include <ros.h>
#include <geometry_msgs/PoseStamped.h>
//include a list or array message for points of obstacles on point cloud

ros::NodeHandle node_handle;

geometry_msgs::PoseStamped pose;
//list variable

void positionCallback(const geometry_msgs::PoseStamped& pose) {
 //map planning algorithm that translates pose into a motor command
}

void objectAvoidanceCallback("list variable"){
  //
}


ros::Subscriber<geometry_msgs::PoseStamped> pose_subscriber("orb_slam2_mono/pose", &positionCallback);
ros::Subscriber<list> list_subscriber("echo", &objectAvoidanceCallback);//echo topic that displays list of x y z coordinates of the objects

void setup() {
  // put your setup code here, to run once:
 node_handle.initNode();
  node_handle.subscribe(pose_subscriber);
   node_handle.subscribe(list_subscriber);
}

void loop() {
  // put your main code here, to run repeatedly:

}
