#ifndef Movement_h
#define Movement_h
#include "Motor.h"
#include "Point.h"

class Movement {
  public:
    Movement();
    double goToPosition(double&, double&, Point, double, Point, double, double, double);
    void mowLawn(double&, double&,Point, double, Point, Point, Point, Point);
    
  private:
    inline double to_radians(double);
    inline double to_degrees(double);
    float clip(float, float, float);


};

#endif
