#ifndef Subscriber_h
#define 
#include <ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>

class Subscriber {
  public:
    Subscriber();
    
void positionCallback(const geometry_msgs::PoseStamped&); 
Point getPosition();
void initSlam();
  private:
  
};

#endif
