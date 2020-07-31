#include "Arduino.h"
#include "Hardware.h"

Hardware::Hardware(): leftMotor(33, 34, 7, A9, 36, 37, 4480),
rightMotor(14, 15, 6, A8, 38, 39, 4480), 
grassMotor(31, 30, 28, A13, 4, 5, 4480) //Ports 4 and 5 are empty, grassmotor goes full speed since no encoder input

{
  rightMotor.invert(true);
 
}
