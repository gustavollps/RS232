#include "../../include/control_loop/ControlLoop.h"

ControlLoop::ControlLoop(ros::NodeHandle *nh, float freq)
{
  nh_ = nh;

  //Realtime publisher for the pwm node
  realtime_pwm_pub_ = new realtime_tools::RealtimePublisher<tcc_msgs::cmd_vel_msg>(nh_[0],"/PWM",10);

  //Interrupts counter node (callback with PID calculations for better response time)
  interrupt_sub_ = nh_->subscribe("/Interrupts_counter",10,&ControlLoop::interruptCallback,this);
  cmd_vel_sub_ = nh_ ->subscribe("/cmd_vel",10,&ControlLoop::cmd_velCallback,this);
  pid_sever_ = nh_->advertiseService("/ChangePID",&ControlLoop::changePID,this);    

  callback_counter_ = 0;
  cmd_vel_ = false;
  cmd_vel_counter_ = 0;

  M1.position=0;
  M1.setpoint=0;
  M1.velocity=0;  
  M1.int_counter=0;

  M2.position=0;
  M2.setpoint=0;
  M2.velocity=0;  
  M2.int_counter=0;

  M3.position=0;
  M3.setpoint=0;
  M3.velocity=0;  
  M3.int_counter=0;

  int delay = 8;
  if(!nh_->getParam("/Control_Loop/startup_delay",delay)){
    ROS_ERROR("[CONTROL_LOOP] Parameters YAML file not loaded");
    ros::shutdown();
  }

  usleep(delay*1000*1000);


  //Parameters settings ------------------------------------------------------------------
  //--------------------------------------------------------------------------------------
  nh_->getParam("/Control_Loop/ENCODER_PULSE_PER_METER",ENCODER_PULSE_PER_METER);
  float radius = 0.035;
  PULSE_TO_METER_K = 341.2*4.0/(2.0*PI*radius)/CONTROL_FREQ;

  controller_ = 1;
  nh_->getParam("/Control_Loop/Velocity_Controller",controller_);

  M1_getParams();
  M2_getParams();
  M3_getParams();  

  //--------------------------------------------------------------------------------------
  //--------------------------------------------------------------------------------------

  PID_M1_ = new PID(freq,
                    M1_params.P1,
                    M1_params.I1,
                    M1_params.D1,
                    -127,127);
  PID_M2_ = new PID(freq,
                    M2_params.P1,
                    M2_params.I1,
                    M2_params.D1,
                    -127,127);

  PID_M3_ = new PID(freq,
                    M1_params.P1,
                    M1_params.I1,
                    M3_params.D1,
                    -127,127);

}

ControlLoop::~ControlLoop()
{  
  interrupt_sub_.shutdown();
  cmd_vel_sub_.shutdown();  
}

bool ControlLoop::changePID(tcc_msgs::changePID::Request &req, tcc_msgs::changePID::Response &res){

  M1_getParams();
  PID_M1_->changeParameters(M1_params.P1,
                            M1_params.I1,
                            M1_params.D1);
  PID_M1_->resetI();

  M2_getParams();
  PID_M2_->changeParameters(M2_params.P1,
                            M2_params.I1,
                            M2_params.D1);
  PID_M2_->resetI();

  M3_getParams();
  PID_M3_->changeParameters(M3_params.P1,
                            M3_params.I1,
                            M3_params.D1);
  PID_M3_->resetI();

  controller_ = 1;
  nh_->getParam("/Control_Loop/Velocity_Controller",controller_);
  return true;
}

void ControlLoop::M1_getParams()
{
  nh_->getParam("/Control_Loop/M1_P1",M1_params.P1);
  nh_->getParam("/Control_Loop/M1_P2",M1_params.P2);
  nh_->getParam("/Control_Loop/M1_P3",M1_params.P3);

  nh_->getParam("/Control_Loop/M1_I1",M1_params.I1);
  nh_->getParam("/Control_Loop/M1_I2",M1_params.I2);
  nh_->getParam("/Control_Loop/M1_I3",M1_params.I3);

  nh_->getParam("/Control_Loop/M1_D1",M1_params.D1);
  nh_->getParam("/Control_Loop/M1_D2",M1_params.D2);
  nh_->getParam("/Control_Loop/M1_D3",M1_params.D3);

  nh_->getParam("/Control_Loop/M1_Threshold1",M1_params.Threshold1);
  nh_->getParam("/Control_Loop/M1_Threshold2",M1_params.Threshold2);

  nh_->getParam("/Control_Loop/M1_offset",M1_params.Offset);
  nh_->getParam("/Control_Loop/M1_deadzone",M1_params.Deadzone);
}

void ControlLoop::M2_getParams()
{
  nh_->getParam("/Control_Loop/M2_P1",M2_params.P1);
  nh_->getParam("/Control_Loop/M2_P2",M2_params.P2);
  nh_->getParam("/Control_Loop/M2_P3",M2_params.P3);

  nh_->getParam("/Control_Loop/M2_I1",M2_params.I1);
  nh_->getParam("/Control_Loop/M2_I2",M2_params.I2);
  nh_->getParam("/Control_Loop/M2_I3",M2_params.I3);

  nh_->getParam("/Control_Loop/M2_D1",M2_params.D1);
  nh_->getParam("/Control_Loop/M2_D2",M2_params.D2);
  nh_->getParam("/Control_Loop/M2_D3",M2_params.D3);

  nh_->getParam("/Control_Loop/M2_Threshold1",M2_params.Threshold1);
  nh_->getParam("/Control_Loop/M2_Threshold2",M2_params.Threshold2);

  nh_->getParam("/Control_Loop/M2_offset",M2_params.Offset);
  nh_->getParam("/Control_Loop/M2_deadzone",M2_params.Deadzone);
}

void ControlLoop::M3_getParams()
{
  nh_->getParam("/Control_Loop/M3_P1",M3_params.P1);
  nh_->getParam("/Control_Loop/M3_P2",M3_params.P2);
  nh_->getParam("/Control_Loop/M3_P3",M3_params.P3);

  nh_->getParam("/Control_Loop/M3_I1",M3_params.I1);
  nh_->getParam("/Control_Loop/M3_I2",M3_params.I2);
  nh_->getParam("/Control_Loop/M3_I3",M3_params.I3);

  nh_->getParam("/Control_Loop/M3_D1",M3_params.D1);
  nh_->getParam("/Control_Loop/M3_D2",M3_params.D2);
  nh_->getParam("/Control_Loop/M3_D3",M3_params.D3);

  nh_->getParam("/Control_Loop/M3_Threshold1",M3_params.Threshold1);
  nh_->getParam("/Control_Loop/M3_Threshold2",M3_params.Threshold2);

  nh_->getParam("/Control_Loop/M3_offset",M3_params.Offset);
  nh_->getParam("/Control_Loop/M3_deadzone",M3_params.Deadzone);
}

void ControlLoop::cmd_velCallback(const geometry_msgs::Twist::ConstPtr &msg)
{
  //3 wheel omnidirectional equations      
//  M1.velocity = msg->linear.x*PULSE_TO_METER_K;
//  M2.velocity = msg->linear.y*PULSE_TO_METER_K;
//  M3.velocity = msg->linear.z*PULSE_TO_METER_K;
  M1.velocity = PULSE_TO_METER_K*( 0.866025*msg->linear.y + 0.500000*msg->linear.x + 0.333333*msg->angular.z);
  M2.velocity = PULSE_TO_METER_K*(-0.866025*msg->linear.y + 0.500000*msg->linear.x + 0.333333*msg->angular.z);
  M3.velocity = PULSE_TO_METER_K*(                        - 1.000000*msg->linear.x + 0.333333*msg->angular.z);
  ROS_WARN("New velocity: \nM1: %f \nM2: %f \nM3: %f",M1.velocity,M2.velocity,M3.velocity);
  cmd_vel_ = true;
}

void ControlLoop::interruptCallback(const tcc_msgs::interrupt_counter::ConstPtr &msg)
{

  //More than 1 sec without cmd_vel msg = STOP
  if(!cmd_vel_){
    cmd_vel_counter_++;
    if(cmd_vel_counter_>50){
      M1.velocity = 0;
      M2.velocity = 0;
      M3.velocity = 0;
    }
  }
  else{
    cmd_vel_ = false;
    cmd_vel_counter_ = 0;
  }

  M1.dint = msg->int1 - M1.int_counter;
  M2.dint = msg->int2 - M2.int_counter;
  M3.dint = msg->int3 - M3.int_counter;
  M1.int_counter = msg->int1;
  M2.int_counter = msg->int2;
  M3.int_counter = msg->int3;

  //Position Controller ----------------------------------------------------------------------------------------------
  if(!controller_){

    //Wait for subscriber data normalization
    if(callback_counter_< 5){
      M1.setpoint = M1.int_counter;
      M2.setpoint = M2.int_counter;
      M3.setpoint = M3.int_counter;

      M1.position = M1.int_counter;
      M2.position = M2.int_counter;
      M3.position = M3.int_counter;

      callback_counter_++;
    }

    //Update setpoint based on speed (cmd_vel) to simulate a ramp input
    M1.setpoint = M1.position + M1.velocity;
    M1.position = M1.setpoint;
    M1.error = M1.setpoint - M1.int_counter;

    M2.setpoint = M2.position + M2.velocity;
    M2.position = M2.setpoint;
    M2.error = M2.setpoint - M2.int_counter;

    M3.setpoint = M3.position + M3.velocity;
    M3.position = M3.setpoint;
    M3.error = M3.setpoint - M3.int_counter;

    //Set setpoint of each PID controller
    PID_M1_->setpoint_ = M1.setpoint;
    PID_M2_->setpoint_ = M2.setpoint;
    PID_M3_->setpoint_ = M3.setpoint;
    //Control msg generation with PID calculations for the position
    if(absf(M1.error) < M1_params.Deadzone)
      pwm_msg_.pwm1 = 127;
    else
      pwm_msg_.pwm1 = 127+int(PID_M1_->Compute(M1.int_counter));

    if(absf(M2.error) < M2_params.Deadzone)
      pwm_msg_.pwm2 = 127;
    else
      pwm_msg_.pwm2 = 127+int(PID_M2_->Compute(M2.int_counter));

    if(absf(M3.error) < M3_params.Deadzone)
      pwm_msg_.pwm3 = 127;
    else
      pwm_msg_.pwm3 = 127+int(PID_M2_->Compute(M3.int_counter));
  }
  //Velocity Controller ----------------------------------------------------------------------------------------------
  else{
    //Motor 1 controller -----------------------
    PID_M1_->setpoint_ = M1.velocity;
    if(M1.velocity == 0){
      pwm_msg_.pwm1 = 127;
      PID_M1_->resetI();
    }
    else if(absf(M1.dint) < M1_params.Threshold1)
    {
      PID_M1_->changeParameters(M1_params.P1,
                                M1_params.I1,
                                M1_params.D1);      
      if(M1.velocity > 0){
        pwm_msg_.pwm1 = 127+int(PID_M1_->Compute(M1.dint)) + M1_params.Offset;
      }
      else{
        pwm_msg_.pwm1 = 127+int(PID_M1_->Compute(M1.dint)) - M1_params.Offset;
      }
    }
    else if(absf(M1.dint) < M1_params.Threshold2)
    {
      PID_M1_->changeParameters(M1_params.P2,
                                M1_params.I2,
                                M1_params.D2);
      pwm_msg_.pwm1 = 127+int(PID_M1_->Compute(M1.dint));
    }
    else{
      pwm_msg_.pwm1 = 127;
      PID_M1_->resetI();
    }
    //------------------------------------------

    //Motor 2 controller -----------------------
    PID_M2_->setpoint_ = M2.velocity;
    if(M2.velocity == 0){
      pwm_msg_.pwm2 = 127;
      PID_M2_->resetI();
    }
    else if(absf(M2.dint) < M2_params.Threshold1)
    {
      PID_M2_->changeParameters(M2_params.P1,
                                M2_params.I1,
                                M2_params.D1);
      if(M2.velocity > 0){
        pwm_msg_.pwm2 = 127+int(PID_M2_->Compute(M2.dint)) + M2_params.Offset;
      }
      else{
        pwm_msg_.pwm2 = 127+int(PID_M2_->Compute(M2.dint)) - M2_params.Offset;
      }
    }
    else if(absf(M2.dint) < M2_params.Threshold2)
    {
      PID_M2_->changeParameters(M2_params.P2,
                                M2_params.I2,
                                M2_params.D2);
      pwm_msg_.pwm2 = 127+int(PID_M2_->Compute(M2.dint));
    }
    else{
      PID_M2_->resetI();
      pwm_msg_.pwm2 = 127;
    }
    //------------------------------------------

    //Motor 3 controller -----------------------
    PID_M3_->setpoint_ = M3.velocity;
    if(M3.velocity == 0){
      PID_M3_->resetI();
      pwm_msg_.pwm3 = 127;
    }
    else if(absf(M3.dint) < M3_params.Threshold1)
    {
      PID_M3_->changeParameters(M3_params.P1,
                                M3_params.I1,
                                M3_params.D1);
      if(M3.velocity > 0){
        pwm_msg_.pwm3 = 127+int(PID_M3_->Compute(M3.dint)) + M3_params.Offset;
      }
      else{
        pwm_msg_.pwm3 = 127+int(PID_M3_->Compute(M3.dint)) - M3_params.Offset;
      }
    }
    else if(absf(M3.dint) < M3_params.Threshold2)
    {
      PID_M3_->changeParameters(M3_params.P2,
                                M3_params.I2,
                                M3_params.D2);
      pwm_msg_.pwm3 = 127+int(PID_M3_->Compute(M3.dint));
    }
    else{
      PID_M3_->resetI();
      pwm_msg_.pwm3 = 127;
    }
    //------------------------------------------

  }

  //Control msg publisher
  pwm_msg_.timestamp = msg->timestamp;
  if(realtime_pwm_pub_->trylock()){
    realtime_pwm_pub_->msg_ = pwm_msg_;
    realtime_pwm_pub_->unlockAndPublish();
  }

#ifdef DEBUG  
  //system("clear");
  std::cout << "==> Control results for PID 1:" << std::endl
            << "P: " << PID_M1_->P_ << std::endl
            << "I: " << PID_M1_->I_ << std::endl
            << "D: " << PID_M1_->D_ << std::endl
            << "Output1: " << (int)pwm_msg_.pwm1 <<
               "\t Output2: " << (int)pwm_msg_.pwm2 <<
               "\t Output3: " << (int)pwm_msg_.pwm3 << std::endl;
  if(controller_){
  std::cout    << "Setpoint1: " << M1.velocity <<
                  "\t Setpoint2: " << M2.velocity <<
                  "\t Setpoint3: " << M3.velocity << std::endl
               << "Feedback1: " << M1.dint <<
                  "\t Feedback2: " << M2.dint <<
                  "\t Feedback3: " << M3.dint << std::endl;
  }
  else{
  std::cout    << "Setpoint1: " << M1.setpoint<<
                  "\t Setpoint2: " << M2.setpoint<<
                  "\t Setpoint3: " << M3.setpoint<< std::endl
               << "Feedback1: " << M1.int_counter <<
                    "\t Feedback2: " << M2.int_counter <<
                    "\t Feedback3: " << M3.int_counter << std::endl;
  }
#endif
}
