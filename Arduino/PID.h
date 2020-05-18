#ifndef PID_h
#define PID_h

class PID {
  public:
    PID(float*, float*, float*, float, float, float, float);
    bool compute(void);
    void invert(bool);
    void setLimit(float, float);
    void setSampleTime(float);
    void setPOnM(bool);
    void enable(bool);
    void setTerms(float, float, float);
    void setCircularInputMax(float);
    void setDeadband(float);
  private:
    float *input;
    float prevInput;
    float *output;
    float *setPoint;
    float error;
    float outputSum;
    float kp;
    float ki;
    float kd;
    float adjustedKp;
    float adjustedKi;
    float adjustedKd;
    float outputMin;
    float outputMax;
    float sampleTimeMillis;
    float sampleTimeMicros;
    bool inverted;
    bool limited;
    bool pOnM;
    bool enabled;
    unsigned long lastComputed;
    void setAdjustedTerms();
    bool circularInput;
    float circularInputMax;
    float circularInputMid;
    float deadband;
};

#endif
