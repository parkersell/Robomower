#ifndef Movement_h
#define Movement_h
#include "Motor.h"

class Movement {
  public:
    Movement();
    double goToPosition(double&, double&, double, double, double, double, double, double, double, double);
    inline double to_radians(double);
    inline double to_degrees(double);
    float clip(float, float, float);
  private:
    
  
};

#endif
