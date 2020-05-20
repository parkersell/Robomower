#include "Arduino.h"
#include "PID.h"

PID::PID(float *input, float *output, float *setPoint, float kp, float ki, float kd, float sampleTimeMillis) {
  // Initialize with user supplied values.
  this->input = input;
  this->prevInput = *input;
  this->output = output;
  this->setPoint = setPoint;

  // Initialize to default values.
  outputSum = 0;
  pOnM = false;
  
  inverted = false;
  limited = false;
  enabled = true;
  lastComputed = 0;
  deadband = 0;
  circularInput = false;

  this->setTerms(kp, ki, kd);
  this->setSampleTime(sampleTimeMillis);
}

bool PID::compute(void) {
  if (!enabled || micros() - lastComputed < sampleTimeMicros) return false;
  lastComputed = micros();
  // Calc error.
  error = *setPoint - *input;
  if (circularInput) {
    if (error < -circularInputMid) error += circularInputMax;
    else if (error > circularInputMid) error -= circularInputMax;
  }
  float output = 0;
  // Only compute output if outside of deadband.
  if (!deadband || abs(error) > deadband) {
    // Calc change in input.
    float dInput = *input - prevInput;
    if (circularInput) {
      if (dInput < -circularInputMid) dInput += circularInputMax;
      else if (dInput > circularInputMid) dInput -= circularInputMax;
    }
    float pTerm = 0;
    // If using pOnM, subtract the product of kp and the change in input from the output sum.
    if (pOnM) outputSum -= adjustedKp * dInput;
    else pTerm = adjustedKp * error;
    // Sum product of ki and error and sample time with the output sum.
    outputSum += adjustedKi * error;
    // Constrain output sum to output min and max (to prevent wind up).
    if (limited) outputSum = constrain(outputSum, outputMin, outputMax);
    // Calc controller output.
    output = pTerm + outputSum - adjustedKd * dInput;
    // Constrain overall output to output min and and max (so that true output is never outside of the limits).
    if (limited) output = constrain(output, outputMin, outputMax);    
  }
  // Write back output.
  *this->output = output;
  // Save input for use next cycle.
  prevInput = *input;
  return true;
}

void PID::invert(bool inverted) {
  this->inverted = inverted;
  setAdjustedTerms();
}

void PID::setDeadband(float band) {
  deadband = band;
}

void PID::setCircularInputMax(float max) {
  circularInputMax = max;
  circularInputMid = circularInputMax / 2.0;
  circularInput = circularInputMax > 0;
}

void PID::setLimit(float min, float max) {
  outputMin = min;
  outputMax = max;
  limited = true;
}

void PID::setPOnM(bool pOnM) {
  this->pOnM = pOnM;
}

void PID::enable(bool enabled) {
  this->enabled = enabled;

  // Seamlessly re-initialize.
  if (this->enabled) {
    outputSum = *output;
    if (limited) outputSum = constrain(outputSum, outputMin, outputMax);
    prevInput = *input;
  }
}

void PID::setTerms(float kp, float ki, float kd) {
  this->kp = kp;
  this->ki = ki;
  this->kd = kd;
  setAdjustedTerms();
}

void PID::setSampleTime(float sampleTimeMillis) {
  this->sampleTimeMillis = sampleTimeMillis;
  sampleTimeMicros = sampleTimeMillis * 1000;
  setAdjustedTerms();
}

void PID::setAdjustedTerms() {
  adjustedKp = kp * (inverted ? -1 : 1);
  adjustedKi = ki * (inverted ? -1 : 1) * sampleTimeMillis / 1000.0f;
  adjustedKd = kd * (inverted ? -1 : 1) / sampleTimeMillis / 1000.0f;
}
