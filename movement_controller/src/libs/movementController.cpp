#include "../../include/movement_controller/movementController.h"

/**
 * @brief movementController::movementController
 * @param nhh
 */
movementController::movementController(ros::NodeHandle* nh)
{
  nh_ = nh;

  double PULSE_PER_ROTATION = 341.2 * 4.0;
  double WHEEL_CIRCUNFERENCE = 0.0350 * PI * 2.0000;
  PULSE_PER_METER_ = WHEEL_CIRCUNFERENCE / PULSE_PER_ROTATION;

  radius_ = 0.035;
  freq_ = 50;
  Kp_ = 0.02;
  Ki_ = 0;
  Kd_ = 0.002;
  pid_min_ = -1;
  pid_max_ = 1;
  dead_zone_ = 4;
  cmd_vel_ = false;
  int_start_ = false;
  cmd_vel_counter_ = 0;

  angle_ = 0;
  last_angle_ = 0;
  angle_setpoint_ = 0;

  pose_.x = 0;
  pose_.y = 0;
  d_pose_.x = 0;
  d_pose_.y = 0;

  new_int_.m1_counter = 0;
  new_int_.m2_counter = 0;
  new_int_.m3_counter = 0;

  old_int_.m1_counter = 0;
  old_int_.m2_counter = 0;
  old_int_.m3_counter = 0;

  m_spds_.m1 = 0;
  m_spds_.m2 = 0;
  m_spds_.m3 = 0;

  velocity_.x = 0;
  velocity_.y = 0;
  velocity_.w = 0;

  velocity_msg_.Vx = 0;
  velocity_msg_.Vy = 0;

  setpoint_velocity_.Vx = 0;
  setpoint_velocity_.Vy = 0;

  velocity_msg_.Vx = 0;
  velocity_msg_.Vy = 0;

  odom_msg_ekf_.header.frame_id = "robot";
  odom_msg_ekf_.header.seq = 0;
  odom_msg_ekf_.child_frame_id = "robot_odom";

  for (int i = 0; i < 36; i++)
  {
    odom_msg_ekf_.pose.covariance[i] = 0;
    odom_msg_ekf_.twist.covariance[i] = 0;
  }
  odom_msg_ekf_.pose.covariance[0] = 0.00001;
  odom_msg_ekf_.pose.covariance[7] = 0.00001;
  odom_msg_ekf_.pose.covariance[14] = 1000000000000.0;
  odom_msg_ekf_.pose.covariance[21] = 1000000000000.0;
  odom_msg_ekf_.pose.covariance[28] = 1000000000000.0;
  odom_msg_ekf_.pose.covariance[35] = 0.01;

  getParams();

  last_pose_.x = pose_.x;
  last_pose_.y = pose_.y;

  PID_rotation_ = new PID(freq_, Kp_, Ki_, Kd_, pid_min_, pid_max_);

  zeroAngle_client_ = nh_->serviceClient<tcc_msgs::ZeroAngle>("/ZeroAngle");

  // odom_pub_ = nh_->advertise<tcc_msgs::pose2D>("/Odometry", 10);
  ekf_pub_ = nh_->advertise<nav_msgs::Odometry>("/odom", 10);
  cmd_vel_pub_ = nh_->advertise<geometry_msgs::Twist>("/cmd_vel", 10);

  heading_sub_ = nh_->subscribe<sensor_msgs::Imu>(
      "/imu_data", 10, &movementController::headingCallBack, this);
  int_sub_ = nh_->subscribe<tcc_msgs::interrupt_counter>(
      "/Interrupts_counter", 10, &movementController::intCallBack, this);
  remote_control_sub_ = nh_->subscribe<geometry_msgs::Twist>(
      "/virtual_joystick/cmd_vel", 10,
      &movementController::remotecontrolCallBack, this);
  movement_sub_ = nh_->subscribe<tcc_msgs::movement_msg>(
      "/cmd_movement", 10, &movementController::movementCallBack, this);

  control_timer_ = nh_->createTimer(
      ros::Duration(0.01), &movementController::controltimerCallBack, this);
  odom_ekf_timer_ = nh_->createTimer(
      ros::Duration(0.02), &movementController::odomekfCallBack, this);

  pid_change_server_ = nh_->advertiseService(
      "/ChangePID", &movementController::changepidCallBack, this);
  reset_odom_server_ = nh_->advertiseService(
      "/ResetOdom", &movementController::resetodomCallBack, this);

  tcc_msgs::ZeroAngle srv;
  zeroAngle_client_.call(srv);
}

/**
 * @brief movementController::constrain
 * @param value
 * @param max
 * @param min
 * @return
 */
float movementController::constrain(float value, float max, float min)
{
  if (value > max)
    return max;
  else if (value < min)
    return min;
  else
    return value;
}

/**
 * @brief movementController::headingCallBack
 * @param msg
 */
void movementController::headingCallBack(sensor_msgs::Imu msg)
{
  float temp_data = (tf::getYaw(msg.orientation)) * 180.0 / PI;
  if (temp_data > 0)
  {
    temp_data = 180.0 - temp_data;
  }
  else
  {
    temp_data = -180.0 - temp_data;
  }
  temp_data += 180;
  if (temp_data > 180)
  {
    temp_data -= 360;
  }
  angle_ = temp_data;
  ROS_WARN("ANGLE: %f\n Quartenion \n W: %f \n Z: %f ", angle_,
           msg.orientation.w, msg.orientation.z);
}

/**
 * @brief movementController::timerCallBack
 * @param event
 */
void movementController::controltimerCallBack(const ros::TimerEvent& event)
{
  angle_setpoint_ =
      (abs(angle_setpoint_ - angle_) > 180)
          ? ((angle_ > 0) ? angle_setpoint_ + 360 : angle_setpoint_ - 360)
          : angle_setpoint_;
  PID_rotation_->setpoint_ = angle_setpoint_;
  if (abs(angle_setpoint_ - angle_) < dead_zone_ / 2)
  {
    cmd_vel_msg_.angular.z = 0;
  }
  else
  {
    cmd_vel_msg_.angular.z = PID_rotation_->Compute(angle_);
  }

  // Vx acelleration ramps
  if (setpoint_velocity_.Vx >= velocity_msg_.Vx)
  {
    if (setpoint_velocity_.Vx > 0 && velocity_msg_.Vx > 0)
      velocity_msg_.Vx += accel_ramp_;
    else
      velocity_msg_.Vx += decel_ramp_;

    velocity_msg_.Vx = (velocity_msg_.Vx > setpoint_velocity_.Vx)
                           ? setpoint_velocity_.Vx
                           : velocity_msg_.Vx;
  }
  else
  {
    if (setpoint_velocity_.Vx < 0 && velocity_msg_.Vx < 0)
      velocity_msg_.Vx -= accel_ramp_;
    else
      velocity_msg_.Vx -= decel_ramp_;

    velocity_msg_.Vx = (velocity_msg_.Vx < setpoint_velocity_.Vx)
                           ? setpoint_velocity_.Vx
                           : velocity_msg_.Vx;
  }
  if (setpoint_velocity_.Vx == 0 && velocity_.x == 0)
    velocity_msg_.Vx = 0;

  // Vy acceleration ramps
  if (setpoint_velocity_.Vy >= velocity_msg_.Vy)
  {
    if (setpoint_velocity_.Vy > 0 && velocity_msg_.Vy > 0)
      velocity_msg_.Vy += accel_ramp_;
    else
      velocity_msg_.Vy += decel_ramp_;

    velocity_msg_.Vy = (velocity_msg_.Vy > setpoint_velocity_.Vy)
                           ? setpoint_velocity_.Vy
                           : velocity_msg_.Vy;
  }
  else
  {
    if (setpoint_velocity_.Vy < 0 && velocity_msg_.Vy < 0)
      velocity_msg_.Vy -= accel_ramp_;
    else
      velocity_msg_.Vy -= decel_ramp_;

    velocity_msg_.Vy = (velocity_msg_.Vy < setpoint_velocity_.Vy)
                           ? setpoint_velocity_.Vy
                           : velocity_msg_.Vy;
  }
  if (setpoint_velocity_.Vy == 0 && velocity_.y == 0)
    velocity_msg_.Vy = 0;

#ifdef DEBUG
  ROS_INFO("\n Setpoint velocity Vx: %f \n Setpoint velocity Vy: %f \n Vx: %f "
           "\n Vy: %f",
           setpoint_velocity_.Vx, setpoint_velocity_.Vy, velocity_msg_.Vx,
           velocity_msg_.Vy);
#endif

  cmd_vel_msg_.linear.x = velocity_msg_.Vx;
  cmd_vel_msg_.linear.y = velocity_msg_.Vy;

  if (cmd_vel_)
  {
    cmd_vel_pub_.publish(cmd_vel_msg_);
  }
  else
  {
    velocity_msg_.Vx = 0;
    velocity_msg_.Vy = 0;
    cmd_vel_msg_.linear.x = 0;
    cmd_vel_msg_.linear.y = 0;
    cmd_vel_pub_.publish(cmd_vel_msg_);
  }

  static ros::Publisher temp_vel_pub = nh_->advertise<geometry_msgs::Twist>("/velocity_debug",1);
  geometry_msgs::Twist vel_msg;
  vel_msg.linear.x  = velocity_.x*50;
  vel_msg.linear.y  = velocity_.y*50;
  temp_vel_pub.publish(vel_msg);

}

/**
 * @brief movementController::intCallBack
 * @param msg
 */
void movementController::intCallBack(tcc_msgs::interrupt_counter msg)
{
  new_int_.m1_counter = msg.int1;
  new_int_.m2_counter = msg.int2;
  new_int_.m3_counter = msg.int3;

  if (int_start_)
  {
    m_spds_.m1 = new_int_.m1_counter - old_int_.m1_counter;
    m_spds_.m2 = new_int_.m2_counter - old_int_.m2_counter;
    m_spds_.m3 = new_int_.m3_counter - old_int_.m3_counter;
  }
  int_start_ = true;

  old_int_.m1_counter = new_int_.m1_counter;
  old_int_.m2_counter = new_int_.m2_counter;
  old_int_.m3_counter = new_int_.m3_counter;

  // Current local velocity
  velocity_.y =
      Ky_ * PULSE_PER_METER_ * (0.577350 * m_spds_.m1 - 0.577350 * m_spds_.m2);
  velocity_.x =
      Kx_ * PULSE_PER_METER_ *
      (0.333333 * m_spds_.m1 + 0.333333 * m_spds_.m2 - 0.666667 * m_spds_.m3);
  velocity_.w = PULSE_PER_METER_ *
                (0.14 * m_spds_.m1 + 0.14 * m_spds_.m2 + 0.14 * m_spds_.m3);    

  // CURRENT GLOBAL POSE
  pose_.y += (cos(angle_ * PI / 180) * velocity_.y +
              sin((angle_)*PI / 180) * velocity_.x);
  pose_.x += (-sin(angle_ * PI / 180) * velocity_.y +
              cos((angle_)*PI / 180) * velocity_.x);

  d_pose_.y = pose_.y - last_pose_.y;
  d_pose_.x = pose_.x - last_pose_.x;
  last_pose_.x = pose_.x;
  last_pose_.y = pose_.y;
}

/**
 * @brief movementController::remotecontrolCallBack
 * @param msg
 */
void movementController::remotecontrolCallBack(geometry_msgs::Twist msg)
{
  float Xg, Yg;
  angle_setpoint_ = msg.angular.z;
  Xg = -msg.linear.y;
  Yg = msg.linear.x;

  // Global do Local velocity conversion (First sector)
  float final_velocity = sqrt(Xg * Xg + Yg * Yg);

  float cossine = Xg / final_velocity;
  float phi;
  if (cossine != 1 && cossine != -1)
  {
    phi = acos(cossine);
  }
  else if (cossine == 1)
  {
    phi = 0;
  }
  else if (cossine == -1)
  {
    phi = PI;
  }

  phi = phi * 180 / PI;

  if (Yg < 0 && phi != 0)
  {
    phi = -phi;
  }

  // ROS_INFO("\nFinal velocity: %f \nVelocity Angle phi: %f \nXg: %f \nYg:
  // %f",final_velocity, phi,Xg,Yg);

  double alpha = phi + angle_setpoint_;

  // Update velocity setpoints
  if (final_velocity != 0)
  {
    setpoint_velocity_.Vy = final_velocity * sin(alpha * PI / 180.0);
    setpoint_velocity_.Vx = final_velocity * cos(alpha * PI / 180.0);
  }
  else
  {
    setpoint_velocity_.Vy = 0;
    setpoint_velocity_.Vx = 0;
  }

  cmd_vel_ = true;
  cmd_vel_counter_ = 0;
}

/**
 * @brief movementController::movementCallBack
 * @param msg
 */
void movementController::movementCallBack(tcc_msgs::movement_msg msg)
{
  float Xg, Yg;
  angle_setpoint_ = msg.angle;
  Yg = msg.y_spd;
  Xg = msg.x_spd;

  // Global do Local velocity conversion (First sector)
  float final_velocity = sqrt(Xg * Xg + Yg * Yg);

  float cossine = Xg / final_velocity;
  float phi;
  if (cossine != 1 && cossine != -1)
  {
    phi = acos(cossine);
  }
  else if (cossine == 1)
  {
    phi = 0;
  }
  else if (cossine == -1)
  {
    phi = PI;
  }

  phi = phi * 180 / PI;

  if (Yg < 0 && phi != 0)
  {
    phi = -phi;
  }

  // ROS_INFO("\nFinal velocity: %f \nVelocity Angle phi: %f \nXg: %f \nYg:
  // %f",final_velocity, phi,Xg,Yg);

  double alpha = phi + angle_setpoint_;

  // Update velocity setpoints
  if (final_velocity != 0)
  {
    setpoint_velocity_.Vy = final_velocity * sin(alpha * PI / 180.0);
    setpoint_velocity_.Vx = final_velocity * cos(alpha * PI / 180.0);
  }
  else
  {
    setpoint_velocity_.Vy = 0;
    setpoint_velocity_.Vx = 0;
  }

  cmd_vel_ = true;
  cmd_vel_counter_ = 0;
}

/**
 * @brief movementController::changepidCallBack
 * @param req
 * @param res
 * @return
 */
bool movementController::changepidCallBack(tcc_msgs::changePID::Request& req,
                                           tcc_msgs::changePID::Response& res)
{
  nh_->getParam("/Movement_controller/Kp", Kp_);
  nh_->getParam("/Movement_controller/Ki", Ki_);
  nh_->getParam("/Movement_controller/Kd", Kd_);
  nh_->getParam("/Movement_controller/DeadzoneSize", dead_zone_);
  nh_->getParam("/Movement_controller/Angle_Setpoint", angle_setpoint_);
  PID_rotation_->changeParameters(Kp_, Ki_, Kd_);
}

/**
 * @brief movementController::resetodomCallBack
 * @param req
 * @param res
 * @return
 */
bool movementController::resetodomCallBack(tcc_msgs::ResetOdom::Request& req,
                                           tcc_msgs::ResetOdom::Response& res)
{
  pose_.x = 0;
  pose_.y = 0;
}

/**
 * @brief movementController::getParams
 */
void movementController::getParams()
{
  if (!(nh_->getParam("/Movement_controller/Kp", Kp_)))
  {
    ROS_WARN("YAML not found!");
    ros::shutdown();
  }
  nh_->getParam("/Movement_controller/Ki", Ki_);
  nh_->getParam("/Movement_controller/Kd", Kd_);
  nh_->getParam("/Movement_controller/DeadzoneSize", dead_zone_);
  nh_->getParam("/Movement_controller/Angle_Setpoint", angle_setpoint_);
  // nh.getParam("/Movement_controller/PPM_CONSTANT",PULSE_PER_METER);
  nh_->getParam("/Movement_controller/Acceleration_Ramp", accel_ramp_);
  nh_->getParam("/Movement_controller/Deceleration_Ramp", decel_ramp_);

  nh_->getParam("/Movement_controller/InitialPoseX", pose_.x);
  nh_->getParam("/Movement_controller/InitialPoseY", pose_.y);

  nh_->getParam("/Movement_controller/Xgain", Kx_);
  nh_->getParam("/Movement_controller/Ygain", Ky_);
}

/**
 * @brief movementController::odomekfCallBack
 * @param event
 */
void movementController::odomekfCallBack(const ros::TimerEvent& event)
{
  cmd_vel_counter_++;
  if (cmd_vel_counter_ > 25)
  {
    cmd_vel_ = false;
  }

  quaternion_ = tf::createQuaternionFromYaw(angle_ * PI / 180.0);

  odom_msg_ekf_.header.stamp = ros::Time::now();
  odom_msg_ekf_.header.seq++;
  odom_msg_ekf_.pose.pose.orientation.w = quaternion_.getW();
  odom_msg_ekf_.pose.pose.orientation.z = quaternion_.getZ();
  odom_msg_ekf_.pose.pose.orientation.x = quaternion_.getX();
  odom_msg_ekf_.pose.pose.orientation.y = quaternion_.getY();
  odom_msg_ekf_.pose.pose.position.x = pose_.x;
  odom_msg_ekf_.pose.pose.position.y = pose_.y;
  odom_msg_ekf_.pose.pose.position.z = 0;

  float angle_delta = (angle_ - last_angle_) * PI / 180.0;
  if (angle_delta > PI)
    angle_delta -= 2.0 * PI;
  else if (angle_delta < -PI)
    angle_delta += 2.0 * PI;

  odom_msg_ekf_.twist.twist.angular.z = angle_delta;
  odom_msg_ekf_.twist.twist.linear.x = d_pose_.x * 50.0;
  odom_msg_ekf_.twist.twist.linear.y = d_pose_.y * 50.0;

  last_angle_ = angle_;

  ekf_pub_.publish(odom_msg_ekf_);
}
