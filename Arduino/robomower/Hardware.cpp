#include "Arduino.h"
#include "Hardware.h"

Hardware::Hardware():leftMotor(33, 34, 7, A9, 36, 37, 4480),rightMotor(14, 15, 6, A8, 38, 39, 4480)
{
 rightMotor.invert(true);

}