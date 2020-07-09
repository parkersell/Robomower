#ifndef Array_h
#define Array_h
#include <std_msgs/String.h>
#include "Point.h"

class Array {
  public:
    Array();
    ros::Array<geometry_msgs::PoseStamped, Array> pose_Array;
    static constexpr float RAD_PER_SEC_TO_RPM = 30.0 / PI;

    void positionCallback(const geometry_msgs::PoseStamped&);
    Point getPosition();
    double getRoll();
    double getYaw();
    void initSLAM();
    void spinOnceS();

  private:
    inline double to_degrees(double radians) {
      return radians * (180.0 / M_PI);
    }

};

#endif
