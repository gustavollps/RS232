#ifndef CONTROLLOOP_H
#define CONTROLLOOP_H

//ROS library
#include <ros/ros.h>

//System
#include <fstream>

//Personal Msgs
#include "tcc_msgs/cmd_vel_msg.h"
#include "tcc_msgs/interrupt_counter.h"
#include "tcc_msgs/pose2D.h"
#include "tcc_msgs/changePID.h"
#include "tcc_msgs/CalibrateInt.h"

//Msgs
#include "geometry_msgs/Twist.h"

//Tools
#include "realtime_tools/realtime_publisher.h"
#include <boost/thread/thread.hpp>

//Personal libraries
#include "PID.h"

#define PI 3.14159265
//#define DEBUG

#define CONTROL_FREQ 50.0

class ControlLoop
{
  public:
    ControlLoop(ros::NodeHandle *nh, float freq);
    ~ControlLoop();    

    tcc_msgs::cmd_vel_msg pwm_msg_;    

    struct motor_info{
      int32_t int_counter, dint;
      float velocity;
      float error;
      double position, setpoint;
    };

    float absf(float value){
      if(value>0)
        return value;
      else
        return -value;
    }

    float constrain(float value, float min, float max){
      if(value > max){
        return max;
      }
      else if(value < min){
        return min;
      }
      else{
        return value;
      }
    }    

  private:

    ros::NodeHandle *nh_;

    ros::Subscriber interrupt_sub_;
    ros::Subscriber cmd_vel_sub_;

    realtime_tools::RealtimePublisher<tcc_msgs::cmd_vel_msg> *realtime_pwm_pub_;

    ros::ServiceServer pid_sever_;

    void interruptCallback(const tcc_msgs::interrupt_counter::ConstPtr &msg);
    void cmd_velCallback(const geometry_msgs::Twist::ConstPtr &msg);    
    bool changePID(tcc_msgs::changePID::Request &req, tcc_msgs::changePID::Response &res);
    void odomTimer(const ros::TimerEvent &event);


    void M1_getParams();
    void M2_getParams();
    void M3_getParams();    

    PID *PID_M1_;
    PID *PID_M2_;
    PID *PID_M3_;

    motor_info M1,M2,M3;


    struct PID_Params{
      float P1,I1,D1,P2,I2,D2,P3,I3,D3;
      int Offset, Deadzone;
      int Threshold1, Threshold2;
    };

    PID_Params M1_params;
    PID_Params M2_params;
    PID_Params M3_params;

    bool controller_;

    float motor1_vel_;
    float motor2_vel_;
    float motor3_vel_;   

    int callback_counter_;
    int cmd_vel_counter_;
    bool cmd_vel_;

    float PULSE_TO_METER_K;
    float ENCODER_PULSE_PER_METER;


};

#endif // CONTROLLOOP_H
