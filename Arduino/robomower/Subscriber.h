#ifndef Subscriber_h
#define Subscriber_h
#include <ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include "Point.h"

class Subscriber {
  public:
    Subscriber();
    ros::Subscriber<geometry_msgs::PoseStamped, Subscriber> pose_subscriber;
    
void positionCallback(const geometry_msgs::PoseStamped&); 
Point getPosition();
void initSLAM();
void spinOnceS();


  private:
  
};

#endif
