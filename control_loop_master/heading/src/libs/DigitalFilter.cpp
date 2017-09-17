#include "../../include/heading/DigitalFilter.h"

//First order filter
DigitalFilter::DigitalFilter(double Ku0, double Ku1, double Ky0, double Ky1){
  a0_=Ky0;
  a1_=Ky1;
  a2_=0;

  b0_=Ku0;
  b1_=Ku1;
  b2_=0;

  y_0_=0;
  y_1_=0;
  y_2_=0;

  x_0_=0;
  x_1_=0;
  x_2_=0;
}

//Second order filter
DigitalFilter::DigitalFilter(double Ku0, double Ku1, double Ku2, double Ky0, double Ky1, double Ky2){
  a0_=Ky0;
  a1_=Ky1;
  a2_=Ky2;

  b0_=Ku0;
  b1_=Ku1;
  b2_=Ku2;

  y_0_=0;
  y_1_=0;
  y_2_=0;

  x_0_=0;
  x_1_=0;
  x_2_=0;
}

double DigitalFilter::Filter(double read){

  x_0_= read;
  y_0_ = (b0_*x_0_ + b1_*x_1_ + b2_*x_2_ - a1_*y_1_ - a2_*y_2_)/a0_;

  y_2_=y_1_;
  y_1_=y_0_;

  x_2_=x_1_;
  x_1_=x_0_;

  return y_0_;

}
