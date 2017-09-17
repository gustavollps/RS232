#ifndef PID_H
#define PID_H

class PID
{
  public:
    PID(float freq, float kp,float ki,float kd,float min, float max);
    ~PID();
    float Compute(float data);
    float constrain(float value,float min, float max);
    void changeParameters(float kp,float ki,float kd);    
    void resetI();
    float setpoint_;
    float P_;
    float I_;
    float D_;
    float error_;

    float kp_;

  private:
    float period_;
    float min_;
    float max_;


    float ki_;
    float kd_;
    float output_;

    float oerror_;
    float derror_;

};

#endif
