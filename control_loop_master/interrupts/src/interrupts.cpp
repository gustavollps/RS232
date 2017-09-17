#include <ros/ros.h>
#include "tcc_msgs/interrupt_counter.h"
#include "tcc_msgs/cmd_vel_msg.h"
#include <wiringPi.h>
#include <boost/thread/thread.hpp>
#include <realtime_tools/realtime_publisher.h>
#include <tcc_msgs/CalibrateInt.h>

//Interrupts Node
/*pins:
 * 7 & 0
 * 2 & 3
 * 12 & 13
 */

#define M1_I1 3
#define M1_I2 2

#define M2_I1 12
#define M2_I2 13

#define M3_I1 7
#define M3_I2 0

#define INT_FREQ 10000

#define CALIBRATION_TIME 1

int calibration_time = CALIBRATION_TIME;
int int_freq = INT_FREQ;

int32_t counter_int1 = 0;
int32_t counter_int2 = 0;
int32_t counter_int3 = 0;

bool pin1_state;
bool pin2_state;
bool dir1 = false;
bool st1 = false;

bool pin3_state;
bool pin4_state;
bool dir2 = false;
bool st2 = false;

bool pin5_state;
bool pin6_state;
bool dir3 = false;
bool st3 = false;

realtime_tools::RealtimePublisher<tcc_msgs::interrupt_counter> *realtime_pub;

void softwareISR(){
  if(pin1_state != digitalRead(M1_I1)){
    pin1_state = !pin1_state;

    if(st1)
      dir1 = !dir1;

    st1 = true;

    if(dir1)
      counter_int1++;
    else
      counter_int1--;

  }

  if(pin2_state != digitalRead(M1_I2)){
    pin2_state = !pin2_state;

    if(!st1)
      dir1 = !dir1;

    st1 = false;

    if(dir1)
      counter_int1++;
    else
      counter_int1--;

  }

//-------------------------------------------------------------------------------------------------------------

  if(pin3_state != digitalRead(M2_I1)){
    pin3_state = !pin3_state;

    if(st2)
      dir2 = !dir2;

    st2 = true;

    if(dir2)
      counter_int2++;
    else
      counter_int2--;
  }
  if(pin4_state != digitalRead(M2_I2)){
    pin4_state = !pin4_state;

    if(!st2)
      dir2 = !dir2;

    st2 = false;

    if(dir2)
      counter_int2++;
    else
      counter_int2--;
  }

//-------------------------------------------------------------------------------------------------------------

  if(pin5_state != digitalRead(M3_I1)){
    pin5_state = !pin5_state;

    if(st3)
      dir3 = !dir3;

    st3 = true;

    if(dir3)
      counter_int3++;
    else
      counter_int3--;
  }
  if(pin6_state != digitalRead(M3_I2)){
    pin6_state = !pin6_state;

    if(!st3)
      dir3 = !dir3;

    st3 = false;

    if(dir3)
      counter_int3++;
    else
      counter_int3--;
  }


//-------------------------------------------------------------------------------------------------------------

}

bool calibrationCallback(tcc_msgs::CalibrateInt::Request &req, tcc_msgs::CalibrateInt::Response &res){
  tcc_msgs::cmd_vel_msg msg;
  msg.pwm1 = 154;
  msg.pwm2 = 154;
  msg.pwm3 = 154;

  ros::NodeHandle temp_node;
  ros::Publisher temp_pub = temp_node.advertise<tcc_msgs::cmd_vel_msg>("/PWM",10);
  int calibration_counter = 0;

  ros::Rate loop_rate(50);

  while(ros::ok()){
    msg.timestamp = ros::Time::now();
    temp_pub.publish(msg);
    dir1=true;
    dir2=true;
    dir3=true;   

    loop_rate.sleep();

    calibration_counter++;

    if(calibration_counter > calibration_time*50)
      break;

  }
  return true;
}

void loop(){
  ros::spin();
}

void timerCallBack(const ros::TimerEvent &event){
  if(realtime_pub->trylock()){
    realtime_pub->msg_.int1 = counter_int1;
    realtime_pub->msg_.int2 = counter_int2;
    realtime_pub->msg_.int3 = counter_int3;
    //Timestamp for response time monitoring and fail safe (on pwm node)
    realtime_pub->msg_.timestamp = ros::Time::now();
    realtime_pub->unlockAndPublish();
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "interrupts");
  int rate = 10000;
  boost::thread thread(loop);
  ros::NodeHandle nh;
  realtime_pub = new realtime_tools::RealtimePublisher<tcc_msgs::interrupt_counter>(nh,"/Interrupts_counter",10);
  ros::ServiceServer calibration_service = nh.advertiseService("/CalibrateInt",calibrationCallback);
  ros::Timer int_timer = nh.createTimer(ros::Duration(0.02),timerCallBack);

  if(!nh.getParam("/Interrupts/INT_FREQ",int_freq)){
    ROS_ERROR("[INTERRUPTS] Parameters YAML file not loaded");
    ros::shutdown();
  }

  nh.getParam("/Interrupts/CALIBRATION_TIME",calibration_time);

  ros::Rate loop(int_freq);
  //ros::Publisher pub = nh->advertise<tcc_msgs::interrupt_counter>("/Interrupts_counter",10);  

  tcc_msgs::interrupt_counter msg;
  int counter = 0;

  wiringPiSetup();
  pinMode(M1_I1,INPUT);
  pinMode(M1_I2,INPUT);
  pinMode(M2_I1,INPUT);
  pinMode(M2_I2,INPUT);
  pinMode(M3_I1,INPUT);
  pinMode(M3_I2,INPUT);

  pin1_state = digitalRead(M1_I1);
  pin2_state = digitalRead(M1_I2);

  pin3_state = digitalRead(M2_I1);
  pin4_state = digitalRead(M2_I2);

  pin5_state = digitalRead(M3_I1);
  pin6_state = digitalRead(M3_I2);

  while(ros::ok()){    
    softwareISR();    
    loop.sleep();
  }

  thread.join();
  return 0;
}
