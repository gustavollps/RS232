/*#include <ros/ros.h>

// Ros messages
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

// Custom msgs and srvs
#include <tcc_msgs/pose2D.h>
#include <tcc_msgs/changePID.h>
#include <tcc_msgs/ResetOdom.h>
#include <tcc_msgs/interrupt_counter.h>
#include <tcc_msgs/movement_msg.h>
#include <tcc_msgs/ZeroAngle.h>

// Personal Library
#include <control_loop/PID.h>

// System tools
#include <boost/thread/thread.hpp>
#include <math.h>

// Ros external libraries
#include <tf/transform_datatypes.h>

#define PI 3.14159265

//#define PULSE_PER_METER 344.0000*(1.0/(0.07*PI))*4000 //344.0* 7 * PI //344
// pulses per minute (encoder) / 7cm diameter * PI
#define CONTROL_FREQ 50.0000
//#define PPM_CONSTANT CONTROL_FREQ/PULSE_PER_METER

float radius = 0.0035;
double PULSE_PER_ROTATION = 344.4 * 4.0;
double WHEEL_CIRCUNFERENCE = 0.0350 * PI * 2.0000;
double PULSE_PER_METER = WHEEL_CIRCUNFERENCE / PULSE_PER_ROTATION;
float freq = 50;
float Kp = 0.02;
float Ki = 0;
float Kd = 0.002;
float pid_min = -1;
float pid_max = 1;
float dead_zone = 4;
geometry_msgs::Twist cmd_vel_msg;
bool cmd_vel = false;
bool int_start = false;
int cmd_vel_counter = 0;

PID* PID_rotation;

float angle = 0;
float angle_setpoint = 0;

float accel_ramp;
float decel_ramp;

struct Velocity
{
  float Vx;
  float Vy;
};

struct intCounter
{
  long int m1_counter;
  long int m2_counter;
  long int m3_counter;
};

struct intDeriv
{
  int m1;
  int m2;
  int m3;
};

struct Twist
{
  float y;
  float x;
  float w;
};

struct position
{
  float x;
  float y;
};

position pose;
intCounter new_int, old_int;
intDeriv m_spds;
Twist velocity;
Velocity setpoint_velocity;
Velocity velocity_msg;

ros::Publisher cmd_vel_pub;

void headingCallBack(std_msgs::Float32 msg) { angle = msg.data; }

float constrain(float value, float max, float min)
{
  if (value > max)
    return max;
  else if (value < min)
    return min;
  else
    return value;
}

// control loop timer callback
void timerCallBack(const ros::TimerEvent& event)
{
  angle_setpoint =
      (abs(angle_setpoint - angle) > 180)
          ? ((angle > 0) ? angle_setpoint + 360 : angle_setpoint - 360)
          : angle_setpoint;
  PID_rotation->setpoint_ = angle_setpoint;
  if (abs(angle_setpoint - angle) < dead_zone / 2)
  {
    cmd_vel_msg.angular.z = 0;
  }
  else
  {
    cmd_vel_msg.angular.z = PID_rotation->Compute(angle);
  }

  // Vx acelleration ramps
  if (setpoint_velocity.Vx >= velocity_msg.Vx)
  {
    if (setpoint_velocity.Vx > 0 && velocity_msg.Vx > 0)
      velocity_msg.Vx += accel_ramp;
    else
      velocity_msg.Vx += decel_ramp;

    velocity_msg.Vx = (velocity_msg.Vx > setpoint_velocity.Vx)
                          ? setpoint_velocity.Vx
                          : velocity_msg.Vx;
  }
  else
  {
    if (setpoint_velocity.Vx < 0 && velocity_msg.Vx < 0)
      velocity_msg.Vx -= accel_ramp;
    else
      velocity_msg.Vx -= decel_ramp;

    velocity_msg.Vx = (velocity_msg.Vx < setpoint_velocity.Vx)
                          ? setpoint_velocity.Vx
                          : velocity_msg.Vx;
  }
  if (setpoint_velocity.Vx == 0 && velocity.x == 0)
    velocity_msg.Vx = 0;

  // Vy acceleration ramps
  if (setpoint_velocity.Vy >= velocity_msg.Vy)
  {
    if (setpoint_velocity.Vy > 0 && velocity_msg.Vy > 0)
      velocity_msg.Vy += accel_ramp;
    else
      velocity_msg.Vy += decel_ramp;

    velocity_msg.Vy = (velocity_msg.Vy > setpoint_velocity.Vy)
                          ? setpoint_velocity.Vy
                          : velocity_msg.Vy;
  }
  else
  {
    if (setpoint_velocity.Vy < 0 && velocity_msg.Vy < 0)
      velocity_msg.Vy -= accel_ramp;
    else
      velocity_msg.Vy -= decel_ramp;

    velocity_msg.Vy = (velocity_msg.Vy < setpoint_velocity.Vy)
                          ? setpoint_velocity.Vy
                          : velocity_msg.Vy;
  }
  if (setpoint_velocity.Vy == 0 && velocity.y == 0)
    velocity_msg.Vy = 0;

  ROS_INFO("\n Setpoint velocity Vx: %f \n Setpoint velocity Vy: %f \n Vx: %f "
           "\n Vy: %f",
           setpoint_velocity.Vx, setpoint_velocity.Vy, velocity_msg.Vx,
           velocity_msg.Vy);

  cmd_vel_msg.linear.x = velocity_msg.Vx;
  cmd_vel_msg.linear.y = velocity_msg.Vy;

  if (cmd_vel)
  {
    cmd_vel_pub.publish(cmd_vel_msg);
  }
  else
  {
    velocity_msg.Vx = 0;
    velocity_msg.Vy = 0;
    cmd_vel_msg.linear.x = 0;
    cmd_vel_msg.linear.y = 0;
    cmd_vel_pub.publish(cmd_vel_msg);
  }
}

// odommetry callback for robot position
void intCallBack(tcc_msgs::interrupt_counter msg)
{

  new_int.m1_counter = msg.int1;
  new_int.m2_counter = msg.int2;
  new_int.m3_counter = msg.int3;

  if (int_start)
  {
    m_spds.m1 = new_int.m1_counter - old_int.m1_counter;
    m_spds.m2 = new_int.m2_counter - old_int.m2_counter;
    m_spds.m3 = new_int.m3_counter - old_int.m3_counter;
  }
  int_start = true;

  old_int.m1_counter = new_int.m1_counter;
  old_int.m2_counter = new_int.m2_counter;
  old_int.m3_counter = new_int.m3_counter;

  // Current local velocity
  velocity.x = (0.866025 * m_spds.m1 - 0.866025 * m_spds.m2) / 2.000000;
  velocity.y =
      (0.500000 * m_spds.m1 + 0.500000 * m_spds.m2 - 1.000000 * m_spds.m3) /
      2.0000000;

  // CURRENT GLOBAL POSE
  pose.y -= PULSE_PER_METER * (sin(angle * PI / 180) * velocity.y +
                               sin((angle - 90) * PI / 180) * velocity.x);
  pose.x += PULSE_PER_METER * (cos(angle * PI / 180) * velocity.y +
                               cos((angle - 90) * PI / 180) * velocity.x);
}

void remotecontrolCallBack(geometry_msgs::Twist msg)
{
  float Xg, Yg;
  angle_setpoint = msg.angular.z;
  Xg = -msg.linear.y;
  Yg = msg.linear.x;

  // Global do Local velocity conversion (First sector)
  float final_velocity = sqrt(Xg * Xg + Yg * Yg);

  float cossine = Xg / final_velocity;
  float phi;
  if (cossine != 1 && cossine != -1)
  {
    phi = acos(cossine);
    // std::cout << "Primeiro phi: "<< phi << " cossine: " << cossine <<
    // std::endl;
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

  double alpha = phi + angle_setpoint;

  // Update velocity setpoints
  if (final_velocity != 0)
  {
    setpoint_velocity.Vy = final_velocity * sin(alpha * PI / 180.0);
    setpoint_velocity.Vx = final_velocity * cos(alpha * PI / 180.0);
  }
  else
  {
    setpoint_velocity.Vy = 0;
    setpoint_velocity.Vx = 0;
  }

  cmd_vel = true;
  cmd_vel_counter = 0;
}

void movementCallBack(tcc_msgs::movement_msg msg)
{
  angle_setpoint = msg.angle;

  cmd_vel_msg.linear.x = msg.x_spd * sin((angle_setpoint - 90) * PI / 180) +
                         msg.y_spd * cos(angle_setpoint * PI / 180);
  cmd_vel_msg.linear.y = msg.y_spd * sin((angle_setpoint)*PI / 180) +
                         msg.x_spd * cos((angle_setpoint - 90) * PI / 180);
}

bool changepidCallBack(tcc_msgs::changePID::Request& req,
                       tcc_msgs::changePID::Response& res)
{
  ros::NodeHandle nh;
  nh.getParam("/Movement_controller/Kp", Kp);
  nh.getParam("/Movement_controller/Ki", Ki);
  nh.getParam("/Movement_controller/Kd", Kd);
  nh.getParam("/Movement_controller/DeadzoneSize", dead_zone);
  nh.getParam("/Movement_controller/Angle_Setpoint", angle_setpoint);
  PID_rotation->changeParameters(Kp, Ki, Kd);
}

bool resetodomCallBack(tcc_msgs::ResetOdom::Request& req,
                       tcc_msgs::ResetOdom::Response& res)
{
  pose.x = 0;
  pose.y = 0;
}

void loop() { ros::spin(); }

void odomekfCallBack(const ros::TimerEvent &event){

}

int main(int argc, char** argv)
{
  pose.x = 0;
  pose.y = 0;

  new_int.m1_counter = 0;
  new_int.m2_counter = 0;
  new_int.m3_counter = 0;

  old_int.m1_counter = 0;
  old_int.m2_counter = 0;
  old_int.m3_counter = 0;

  m_spds.m1 = 0;
  m_spds.m2 = 0;
  m_spds.m3 = 0;

  velocity.x = 0;
  velocity.y = 0;
  velocity.w = 0;

  velocity_msg.Vx = 0;
  velocity_msg.Vy = 0;

  setpoint_velocity.Vx = 0;
  setpoint_velocity.Vy = 0;

  ros::init(argc, argv, "Movement_controller");
  boost::thread thread(loop);
  ros::NodeHandle nh;

  if (!(nh.getParam("/Movement_controller/Kp", Kp)))
  {
    ROS_WARN("YAML not found!");
    ros::shutdown();
    return 0;
  }
  nh.getParam("/Movement_controller/Ki", Ki);
  nh.getParam("/Movement_controller/Kd", Kd);
  nh.getParam("/Movement_controller/DeadzoneSize", dead_zone);
  nh.getParam("/Movement_controller/Angle_Setpoint", angle_setpoint);
  // nh.getParam("/Movement_controller/PPM_CONSTANT",PULSE_PER_METER);
  nh.getParam("/Movement_controller/Acceleration_Ramp", accel_ramp);
  nh.getParam("/Movement_controller/Deceleration_Ramp", decel_ramp);
  PID_rotation = new PID(freq, Kp, Ki, Kd, pid_min, pid_max);

  cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);
  ros::ServiceClient zeroAngle_client =
      nh.serviceClient<tcc_msgs::ZeroAngle>("/ZeroAngle");
  tcc_msgs::ZeroAngle srv;
  zeroAngle_client.call(srv);
  ros::Publisher odom_pub = nh.advertise<tcc_msgs::pose2D>("/Odometry", 10);
  ros::Publisher ekf_pub = nh.advertise<nav_msgs::Odometry>("/odom", 10);
  ros::Subscriber heading_sub =
      nh.subscribe<std_msgs::Float32>("/Heading", 10, headingCallBack);
  ros::Subscriber int_sub = nh.subscribe<tcc_msgs::interrupt_counter>(
      "/Interrupts_counter", 10, intCallBack);
  ros::Subscriber remote_control_sub = nh.subscribe<geometry_msgs::Twist>(
      "/virtual_joystick/cmd_vel", 10, remotecontrolCallBack);
  ros::Subscriber movement_sub = nh.subscribe<tcc_msgs::movement_msg>(
      "/cmd_movement", 10, movementCallBack);

  ros::Timer control_timer = nh.createTimer(ros::Duration(0.01), timerCallBack);
  ros::Timer odom_ekf_timer =
      nh.createTimer(ros::Duration(0.01), odomekfCallBack);

  ros::ServiceServer pid_change_server =
      nh.advertiseService("/ChangePID", changepidCallBack);
  ros::ServiceServer reset_odom_server =
      nh.advertiseService("/ResetOdom", resetodomCallBack);

  tcc_msgs::pose2D odom_msg;
  ros::Rate loop_rate(freq);
  nav_msgs::Odometry odom_msg_ekf;
  tf::Quaternion quaternion;

  odom_msg_ekf.header.frame_id = "robot";
  odom_msg_ekf.header.stamp = ros::Time::now();
  odom_msg_ekf.header.seq = 0;

  for (int i = 0; i < 36; i++)
  {
    odom_msg_ekf.pose.covariance[i] = 0;
    odom_msg_ekf.twist.covariance[i] = 0;
  }
  odom_msg_ekf.pose.covariance[0] = 0.00001;
  odom_msg_ekf.pose.covariance[7] = 0.00001;
  odom_msg_ekf.pose.covariance[14] = 1000000000000.0;
  odom_msg_ekf.pose.covariance[21] = 1000000000000.0;
  odom_msg_ekf.pose.covariance[28] = 1000000000000.0;
  odom_msg_ekf.pose.covariance[35] = 0.01;

  while (ros::ok())
  {
    cmd_vel_counter++;
    if (cmd_vel_counter > 25)
    {
      cmd_vel = false;
    }

    odom_msg_ekf.header.stamp = ros::Time::now();
    odom_msg_ekf.header.seq++;
    //odom_msg_ekf.pose.pose.orientation

        odom_pub.publish(odom_msg_ekf);

    loop_rate.sleep();
  }
}*/

#include "../include/movement_controller/movementController.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "Movement_controller");
  movementController *mainNode = new movementController(new ros::NodeHandle());
  ros::spin();
  return 0;
}
