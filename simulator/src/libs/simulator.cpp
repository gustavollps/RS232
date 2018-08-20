#include "../../include/simulator/simulator.h"

Simulator::Simulator(ros::NodeHandle* nh)
{      
    nh_ = nh;
    
    odom_ekf_pub_ = nh_->advertise<geometry_msgs::PoseWithCovarianceStamped>(
          "/robot_pose_ekf/odom_combined", 10);

    int_pub_ = nh_->advertise<tcc_msgs::interrupt_counter>(
        "/Interrupts_counter", 10);
    
    heading_pub_ = nh_->advertise<sensor_msgs::Imu>(
        "/imu_data", 10);

    pwm_sub_ = nh_->subscribe<tcc_msgs::cmd_vel_msg>("/PWM",10,&Simulator::pwmCallback,this);

    odom_sub_ = nh_->subscribe<nav_msgs::Odometry>("/odom",10,&Simulator::odomCallback,this);

    int_timer_ = nh_->createTimer(ros::Duration(0.02),&Simulator::timerCallback,this);

    double PULSE_PER_ROTATION = 341.2 * 4.0;
    double WHEEL_CIRCUNFERENCE = 0.0350 * PI * 2.0000;
    motor_speed_sim = 1;
    PULSE_PER_METER_ = motor_speed_sim * WHEEL_CIRCUNFERENCE / PULSE_PER_ROTATION;

    heading_ = 0;

    imu_data_.header.stamp = ros::Time::now();  //load time on imu message
    imu_data_.header.seq = 0; //initialize message sequence
    imu_data_.header.frame_id = "base_footprint"; //frame whose the sensor relationed

    //initialize imu message data
    imu_data_.angular_velocity.x = 0;
    imu_data_.angular_velocity.y = 0;

    imu_data_.orientation.x = 0;
    imu_data_.orientation.y = 0;

    //initialize imu message covariance
    for (int i = 0; i < 9; i++)
    {
      imu_data_.angular_velocity_covariance[i] = 0;
      imu_data_.linear_acceleration_covariance[i] = 0;
      imu_data_.orientation_covariance[i] = 0;
    }

    for(int i=0;i<50;i++){
      m_speeds_[i].m1 = 0;m_speeds_[i].m2 = 0;m_speeds_[i].m3 = 0;
    }


}

/**
 * @brief Simulator::pwmCallback
 * @param msg
 * Gets cmd_vel_msg and use the kinematics conversion to output the odometry
 */
void Simulator::pwmCallback(tcc_msgs::cmd_vel_msg msg)
{
  for(int i = 0;i<50;i++){
    m_speeds_[i+1] = m_speeds_[i];
  }

  m_speeds_[0].m1 = msg.pwm1-127;
  m_speeds_[0].m2 = msg.pwm2-127;
  m_speeds_[0].m3 = msg.pwm3-127;

  /*
  velocity_[0].y =
      PULSE_PER_METER_ * (0.577350 * msg.pwm1 - 0.577350 * msg.pwm2);
  velocity_[0].x =
      PULSE_PER_METER_ *
      (0.333333 * msg.pwm1 + 0.333333 * msg.pwm2 - 0.666667 * msg.pwm3);
  velocity_[0].w = PULSE_PER_METER_ *
                (0.14 * msg.pwm1 + 0.14 * msg.pwm2 + 0.14 * msg.pwm3);
  */
}

/**
 * @brief Simulator::odomCallback
 */
void Simulator::odomCallback(nav_msgs::Odometry msg)
{
  geometry_msgs::PoseWithCovarianceStamped ekf_msg;
  ekf_msg.pose.pose.position.x = msg.pose.pose.position.x;
  ekf_msg.pose.pose.position.y = msg.pose.pose.position.y;
  ekf_msg.pose.pose.position.z = msg.pose.pose.position.z;
  ekf_msg.pose.pose.orientation = msg.pose.pose.orientation;

  odom_ekf_pub_.publish(ekf_msg);
}

void Simulator::timerCallback(const ros::TimerEvent &event)
{
  int delay = 10;
  int_msg_.int1 += m_speeds_[delay].m1;
  int_msg_.int2 += m_speeds_[delay].m2;
  int_msg_.int3 += m_speeds_[delay].m3;

  int_pub_.publish(int_msg_);

  heading_ -= 10*PULSE_PER_METER_ *
                (0.14 * m_speeds_[delay].m1 + 0.14 * m_speeds_[delay].m2 + 0.14 * m_speeds_[delay].m3);

  tf::Quaternion quat = tf::createQuaternionFromYaw(heading_);

  imu_data_.orientation.w = quat.getW();
  imu_data_.orientation.z = quat.getZ();
  imu_data_.header.stamp = ros::Time::now();
  imu_data_.header.seq++;
  heading_pub_.publish(imu_data_);

  long int m1,m2,m3;
  m1 = int_msg_.int1;
  m2 = int_msg_.int2;
  m3 = int_msg_.int3;

  ROS_INFO("Int: \n%ld \n%ld \n%ld\nHeading: \n%f",m1,m2,m3,heading_);
}
