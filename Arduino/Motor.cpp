#include "Arduino.h"
#include "Motor.h"

Motor::Motor(byte driverPinA, byte driverPinB, byte driverPinPWM, byte driverPinCS, byte encoderPinA, byte encoderPinB, word ticksPerRev) {

  this->driverPinA = driverPinA;
  this->driverPinB = driverPinB;
  this->driverPinPWM = driverPinPWM;
  this->driverPinCS = driverPinCS;
  this->encoderPinA = encoderPinA;
  this->encoderPinB = encoderPinB;
  this->ticksPerRev = ticksPerRev;

  // Set motor driver pins to be outputs.
  pinMode(this->driverPinA, OUTPUT);
  pinMode(this->driverPinB, OUTPUT);
  pinMode(this->driverPinPWM, OUTPUT);

  // Initialize to default values.
  rpm = 0;
  rpmSetPoint = 0;
  motorPower = 0;
  currentLimit = 8;
  inverted = false;

  // Initialize Timers.
  encoderTimer = 0;
  currentTimer = 0;

  // Setup speed PID controller with default parameters (run at 1000 Hz).
  speedPID = new PID(&this->rpm, &this->motorPower, &this->rpmSetPoint, 1.2, 25, 0.01, 1);
  speedPID->setLimit(-100, 100);
  speedPID->setPOnM(true);

  encoder = new Encoder(encoderPinA, encoderPinB);

  analogWriteResolution(11);
  analogWriteFrequency(driverPinPWM, 20000);

  disable();
}

void Motor::setSpeedPIDTerms(float kp, float ki, float kd) {
  speedPID->setTerms(kp, ki, kd);
}

void Motor::setSpeed(float rpm) {
  rpmSetPoint = rpm;
    if (inverted) rpmSetPoint = -rpm;
    else rpmSetPoint = rpm;
}

void Motor::enable(void) {
  enable(true);
}

void Motor::enable(bool enabled) {
  this->enabled = true;
  speedPID->enable(enabled);
  if (!enabled) {
    digitalWrite(driverPinA, HIGH);
    digitalWrite(driverPinB, HIGH);
  }
}

void Motor::disable(void) {
  enable(false);
}

float Motor::getSpeed() {
  return rpm;
}

float Motor::getSetSpeed() {
  return rpmSetPoint;
}

float Motor::getCurrentDraw(void) {
  return currentDraw;
}

void Motor::invert(bool inverted) {
  this->inverted = inverted;
  setSpeed(rpmSetPoint);
}

void Motor::setCurrentLimit(float limit) {
  currentLimit = limit;
}

void Motor::compute(void) {

  // Update PID controller.
  bool computed = speedPID->compute();
  // If controller output has changed, update motor driver.
  if (computed) {
    if (motorPower == 0) {
      digitalWrite(driverPinA, HIGH);
      digitalWrite(driverPinB, HIGH);
    } else if (motorPower > 0) {
      digitalWrite(driverPinA, LOW);
      digitalWrite(driverPinB, HIGH);
    } else {
      digitalWrite(driverPinA, HIGH);
      digitalWrite(driverPinB, LOW);
    }
    // Convert motor power percentage to pwm.
    unsigned int motorPWM = abs(motorPower) * 2047 / 100;

    analogWrite(driverPinPWM, motorPWM);
  }

  // Read encoder at 100Hz.
  if (micros() - encoderTimer >= 1000) {
    unsigned long deltaTime = micros() - encoderTimer;
    encoderTimer = micros();
    rpm = 0.5f * rpm + 0.5f * (encoder->read() * (60000000.0f / deltaTime) / ticksPerRev);
    encoder->write(0);
  }

  // Check current at 100Hz.
  if (micros() - currentTimer >= 1000) {
    currentTimer = micros();
    word motorCurrentValue = analogRead(driverPinCS);
    // Get instaneous current reading in amps.
    float instCurrentDraw = (motorCurrentValue * 5.0f) / (1024 * 0.14f);
    // Run current draw thorugh low pass filter.
    currentDraw = 0.98f * currentDraw + 0.02f * instCurrentDraw;
    if (currentDraw > currentLimit && enabled) {
      disable();
    }
  }
}
