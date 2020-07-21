#ifndef Hardware_h
#define Hardware_h
#include "Motor.h"
#include "PID.h"
class Hardware {
  public:
    Hardware();
    Motor leftMotor;
    Motor rightMotor;
    Motor grassMotor;
  private:
  
  
};

#endif
