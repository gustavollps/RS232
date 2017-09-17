#include "../../include/control_loop/PID.h"

PID::PID(float freq,float kp,float ki, float kd, float min, float max)
{
  min_ = min;
  max_ = max;
  period_ = 1.00/freq;
  kp_=kp;
  ki_=ki*period_;
  kd_=kd/period_;
  I_ = 0;
  setpoint_ = 0;
}

PID::~PID()
{

}

float PID::Compute(float data)
{
  derror_ = (setpoint_ - data) - error_;
  error_ = setpoint_ - data;

  I_ += ki_*(error_+oerror_)/2.0;

  I_ = constrain(I_,min_,max_);

  P_ = kp_*error_;

  I_ = (int(data)==0 && int(error_)==0)?0:I_;

  D_ = kd_*derror_;
  output_ = P_+I_+D_;
  output_ = constrain(output_,min_,max_);
  oerror_ = error_;

  return output_;
}

void PID::changeParameters(float kp, float ki, float kd)
{
  kp_ = kp;
  ki_ = ki*period_;
  kd_ = kd/period_;
}

float PID::constrain(float value,float min, float max){
  return (value > max)?max:((value<min)?min:value);
}

void PID::resetI()
{
  I_ = 0;
}
