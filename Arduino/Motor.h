#ifndef Motor_h
#define Motor_h

#include "PID.h"
#include <Encoder.h>

class Motor {
  public:
    Motor(byte, byte, byte, byte, byte, byte, word);
    float getSetSpeed(void);
    float getSpeed(void);
    void setSpeed(float);
    void compute(void);
    float getCurrentDraw(void);
    void setCurrentLimit(float);
    void disable(void);
    void enable(bool);
    void enable(void);
    void invert(bool);
    void setSpeedPIDTerms(float, float, float);
  private:
    bool inverted;
    bool enabled;
    byte driverPinA;
    byte driverPinB;
    byte driverPinPWM;
    byte driverPinCS;
    byte encoderPinA;
    byte encoderPinB;
    word ticksPerRev;
    float rpm;
    float rpmSetPoint;
    float motorPower;
    float currentDraw;
    float currentLimit;
    unsigned long encoderTimer;
    unsigned long currentTimer;
    PID* speedPID;
    Encoder* encoder;
};

#endif
