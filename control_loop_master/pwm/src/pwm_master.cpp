#include "sys/types.h"
#include "sys/sysinfo.h"
#include <ros/ros.h>
#include "tcc_msgs/cmd_vel_msg.h"
#include <tcc_msgs/CalibrateInt.h>
#include "../include/pwm/Serial.h"
#include <boost/thread/thread.hpp>
#include <std_srvs/Empty.h>

uchar duty_cycle_1=127;
uchar duty_cycle_2=127;
uchar duty_cycle_3=127;
int fd;
ros::Time timestamp;
float physMemUsed;

bool motors_enabled=true;


void generatePWM(){
  uchar *data = new uchar[1];
  data[0] = 255;
  uchar *pwm = new uchar[3];
  pwm[0] = duty_cycle_1;
  pwm[1] = duty_cycle_2;
  pwm[2] = duty_cycle_3;
  write(fd,data,1);
  write(fd,pwm,3);
}

void pwmCallback(tcc_msgs::cmd_vel_msg msg){
  if(motors_enabled){
    duty_cycle_1 = msg.pwm1;
    duty_cycle_2 = msg.pwm2;
    duty_cycle_3 = msg.pwm3;
  }
  else{
    duty_cycle_1 = 127;
    duty_cycle_2 = 127;
    duty_cycle_3 = 127;
  }
  generatePWM();

  //Keep last message's timestamp to fail safe
  timestamp = msg.timestamp;


  //Response time debug --------------------------------------

  ros::Duration dt = ros::Time::now() - timestamp;  
  ROS_INFO("\n dt: %f \n RAM Usage: %f%%", dt.toSec()*1000, physMemUsed);
  //  if(dt.toSec() > 0.020)
  //    ROS_WARN("Non-realtime :/");
}

//Thread for spin (triggers callbacks)
void pwm_loop(){
  ros::spin();
}

bool disable_motors(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
  motors_enabled = false;
}

bool enable_motors(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
  motors_enabled = true;
}


int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "PWM");

  ros::NodeHandle nh;

  ros::Subscriber pwm_sub = nh.subscribe<tcc_msgs::cmd_vel_msg>("/PWM",10,pwmCallback);

  std::string port("");

  boost::thread pwm_thread(pwm_loop); //ros spin

  ros::ServiceClient calibration_client = nh.serviceClient<tcc_msgs::CalibrateInt>("/CalibrateInt");

  ros::ServiceServer motor_srv_client_off = nh.advertiseService("/Disable_motors",disable_motors);
  ros::ServiceServer motor_srv_client_on = nh.advertiseService("/Enable_motors",enable_motors);

  tcc_msgs::CalibrateInt srv;

  nh.param("port",port,std::string("ttyUSB0"));
  port = "/dev/"+port;
  Serial Serial(port.c_str(),38400);
  fd = Serial.begin();
  if(fd == -1){
    ROS_ERROR("Coundn't open selected port! Shutting down...");
    ros::shutdown();
    return 0;
  }

  //usleep(2000*1000);
  calibration_client.call(srv);

  struct sysinfo memInfo;
  sysinfo (&memInfo);


  //Fail safe subscriber (stops if not receiving messages for 100ms)
  ros::Rate loop(50);
  ros::Duration dt;
  bool error = false;
  while(ros::ok())
  {
    //Failsafe time = 100ms (cycle = 20ms)
    dt = ros::Time::now() - timestamp;
    physMemUsed = (float(memInfo.totalram - memInfo.freeram))/float(memInfo.totalram)*100;
    if(dt.toSec()>0.1)
    {
      duty_cycle_1 = 127;
      duty_cycle_2 = 127;
      duty_cycle_3 = 127;      
      if(!error)
      {
        ROS_ERROR("Delay/crash on message receiving callback! (response time > 100ms) - Delay %f",dt.toSec());
        error = true;
      }
      generatePWM();
    }   
    else
    {
      if(error)
      {
        ROS_INFO("Response time normalized.");
        error = false;
      }
    }

    loop.sleep();
  }

  //Proper ending, motor shutdown
  uchar* data = new uchar[4];
  data[0] = 255;data[1]=127;data[2]=127;data[3]=127;
  write(fd,data,4);

  return 0;
}
