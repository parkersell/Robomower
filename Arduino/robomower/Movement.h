#ifndef Movement_h
#define Movement_h
#include "Motor.h"
#include "Point.h"
#include "Hardware.h"
class Movement {
  public:
    Movement();

     struct everything {
      Point Pose;
      double heading;
      Point topleft;
      Point topright;
      Point bottomleft;
      Point bottomright;
    };
    
    void goToPosition(double, double, Point, double, Point, double, double, double);
    void mowLawn(double, double, everything);
    //void mowThread(double, double, Point, double, Point, Point, Point, Point);
    void test(double&, double&, everything);
    void disableM();
    void enableM();
    void computeM();
    void setSpeedM(int, int);

   

  private:
    inline double to_radians(double);
    inline double to_degrees(double);
    float clip(float, float, float);


};

#endif
