#define USE_TEENSY_HW_SERIAL
#define HWSERIAL Serial


#include "Movement.h"
#include "Point.h"


//#include <Adafruit_BNO055.h>
//#include <Adafruit_Sensor.h>

//#include <EEPROM.h>


Movement movement;


//Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

//float heading = 0;
//float headingTurnFactor = 0;
//float headingSetPoint = 0;

//PID headingPID = PID(&heading, &headingTurnFactor, &headingSetPoint, 1, 2, 0, 10);

unsigned long ledTimer = micros();
unsigned long printTimer = micros();
unsigned long imuTimer = micros();
unsigned long stepTimer = micros();
unsigned long inchTimer = micros();

//adafruit_bno055_offsets_t calibrationData;

String input = "";
double x = 0;
double y = 0;
Point p0 = Point(x, y);
Point p1 = Point(10,0);
Point p2 = Point(3, 3);

//Motor leftMotor(33, 34, 7, A9, 36, 37, 4480);
//Motor rightMotor(14, 15, 6, A8, 38, 39, 4480);

void setup() {



  // put your setup code here, to run once:
  //Serial.begin(115200);
  HWSERIAL.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  movement.initRobot();
}
void loop() {
  // put your main code here, to run repeatedly:
movement.spinOnceM();


  while (HWSERIAL.available()) {

    char temp = HWSERIAL.read();
    Serial.print(temp);
    if (temp == '\n') {
      if (input == 'd') {
       movement.disableM();
      } else if (input == 'e') {
       
        movement.enableM();
     
      }

      else if (input == 'x') {
        movement.goToPosition(0,p1,0,10,1);
      }
      
      else {
        int commaIndex = input.indexOf(',');
        float speed = commaIndex == -1 ? input.toFloat() : input.substring(0, commaIndex).toFloat();
        //        headingSetPoint = speed;
        float turn = commaIndex == -1 ? 0 : input.substring(commaIndex + 1).toFloat();
        movement.setSpeedM(speed + turn, speed - turn);
      }
      input = "";
    } else input += temp;
  }


  if (micros() - printTimer > 10000) {

    Point p = movement.getPosition();

    //Serial.println(micros() - inchTimer);
    Serial.println(p.x);
    Serial.println(p.y);
  //  Serial.println(movement.getSpeedM());

    printTimer = micros();

  }


  if (micros() - ledTimer > 100000) {
    ledTimer = micros();
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }

  movement.computeM();
}
