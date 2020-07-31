#ifndef Movement_h
#define Movement_h
#include "Motor.h"
#include "Point.h"
#include "Hardware.h"
#include "Subscriber.h"
class Movement {
  public:
    Movement();
    
    //Movement functions
    void goToPosition(Point, double, double);
    void mowLawn(Point, Point, Point, Point, int);
    void mowCalibrate(Point, Point, Point, Point, int);
    void turnTo(double, int, double);
    void curveTo(boolean, int);
    void sTurn(Point, Point, int);
    
    int input();

    //Motor functions
    void disableM();
    void enableM();
    void computeM();
    void setSpeedM(int, int);
    void disableGrass();
    void enableGrass();

    //SLAM functions
    void initRobot();
    void spinOnceM();

    //IMU functions
    double getIMUM();
    void resetIMUM();
    
    Point getPosition();
    float clip(float, float, float);

  private:
    inline double to_radians(double);
    inline double to_degrees(double);
    
};

#endif
