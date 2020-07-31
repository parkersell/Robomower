#ifndef Hardware_h
#define Hardware_h
#include "Motor.h"
#include "PID.h"

#include <math.h>

class Hardware {
  public:
    Hardware();
    Motor leftMotor;
    Motor rightMotor;
    Motor grassMotor;
    
  private:
  
};

#endif
