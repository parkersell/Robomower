#ifndef Movement_h
#define Movement_h
#include "Motor.h"
#include "Point.h"
#include "Hardware.h"
#include "Subscriber.h"
class Movement {
  public:
    Movement();

    
    void goToPosition(Point, double, Point, double, double, double);
    void mowLawn(Point, double, Point, Point, Point, Point);
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
