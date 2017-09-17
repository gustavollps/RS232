// System
#include <math.h>
#include <boost/thread/thread.hpp>
#include <stdlib.h>

// ROS
#include <ros/ros.h>

// Msgs
#include <std_msgs/Float32.h>
#include <std_msgs/Int16.h>
#include <tcc_msgs/magdata.h>
#include <tcc_msgs/CalibrateInt.h>

// Srv
#include <tcc_msgs/ZeroAngle.h>

// WiringPi
#include <wiringPi.h>
#include <wiringPiI2C.h>

// Personal libraries
#include "../include/heading/DigitalFilter.h"
#include "../include/heading/maths.h"
#include "../include/heading/mpu.h"

//#define DEBUG_MAG_GYRO

#define PI 3.14159265
#define FILTER_MERGE 0.005

#define DELAY_CALIBRATION 6000

int freq = 100;
float dt = 1.0 / float(freq);

float mag_offset = 0;
double gyro_angle = 0;
float mag_angle = 0;
int raw_data;
int16_t gyro_data;

bool calibrated = false;
int use_mag = 0;

Mag magnetometer;
MPU gyroscope;

int min_y = 30000, max_y = -30000;
int min_x = 30000, max_x = -30000;
float x_out = 0, y_out = 0, z_out = 0;
int delay_calibration = DELAY_CALIBRATION; // ms
float filter_merge = FILTER_MERGE;
float calibration_time = 2;
float gyro_samples = 500;
double offset = 0;

bool zeroangleCallBack(tcc_msgs::ZeroAngle::Request& req,
                       tcc_msgs::ZeroAngle::Response& res)
{
  gyro_angle = 0;
  ros::NodeHandle nh;
  nh.getParam("/Heading/GYRO_OFFSET", offset);
}

void gyroTimer(const ros::TimerEvent& event)
{
  // Get gyro data
  gyro_data = gyroscope.Gyro_GetDataZ();
  raw_data = gyro_data - offset;

  // convert to degrees and fix circular behavior
  gyro_angle -= (raw_data * SCALE_2000) * dt;
  gyro_angle = gyroscope.constrainGyroAngle(gyro_angle);
}

int main(int argc, char** argv)
{

  //---------------------------------------------------------------------------------------------------------------------
  // ROS and node setup
  // --------------------------------------------------------------------------------------------------
  ros::init(argc, argv, "heading");
  ros::NodeHandle nh;
  ros::Publisher heading_pub = nh.advertise<std_msgs::Float32>("Heading", 10);
  // ros::ServiceServer calibrate_server = nh.advertiseService("/CalibrateInt",
  // calibrationCallback);
  ros::ServiceServer zeroAngle =
      nh.advertiseService("/ZeroAngle", zeroangleCallBack);
  ros::ServiceClient calibrate_client =
      nh.serviceClient<tcc_msgs::CalibrateInt>("/CalibrateInt");

  ros::Rate loop(freq);

  // Getting paramaters
  // --------------------------------------------------------------------------------------------------
  if (!nh.getParam("/Heading/DELAY_CALIBRATION", delay_calibration))
  {
    ROS_ERROR("[HEADING] Parameters YAML file not loaded");
    ros::shutdown();
  }

  nh.getParam("/Heading/FILTER_MERGE", filter_merge);
  nh.getParam("/Heading/CALIBRATION_TIME", calibration_time);
  nh.getParam("/Heading/GYRO_SAMPLES", gyro_samples);
  nh.getParam("/Heading/USE_MAG", use_mag);
  //---------------------------------------------------------------------------------------------------------------------

  // Waiting time for PWM and Interrupts proper initialization
  // -----------------------------------------------------------
  usleep(delay_calibration * 1000);
//---------------------------------------------------------------------------------------------------------------------

#ifdef DEBUG_MAG_GYRO
  ros::Publisher mag_ang_pub = nh.advertise<std_msgs::Float32>("MagAngle", 10);
  ros::Publisher mag_raw_pub = nh.advertise<tcc_msgs::magdata>("Magdata", 10);
#endif

  //---------------------------------------------------------------------------------------------------------------------
  // FILTERING contrusctors
  // ----------------------------------------------------------------------------------------------
  /*
  double a0 = 1000.0, a1 = -1867.3999,a2 = 882.2028;
  double b0 = 3.70071, b1 = 2*b0, b2 = b0;

  DigitalFilter magFilterX(b0,b1,b2,a0,a1,a2);
  DigitalFilter magFilterY(b0,b1,b2,a0,a1,a2);
  */
  //---------------------------------------------------------------------------------------------------------------------

  // MPU setup
  // -----------------------------------------------------------------------------------------------------------
  gyroscope.GetDescriptor();

  gyroscope.Gyro_SetRange2000();
  gyroscope.EnableI2CBypass();

  if (use_mag)
  {
    magnetometer.mag_ = wiringPiI2CSetup(HMC_ADRESS);
    // magnetometer.GetDescriptor();
    wiringPiI2CWriteReg8(magnetometer.mag_, MODE_REG, 0);
    // magnetometer.SetContinuosMode();
  }

  //---------------------------------------------------------------------------------------------------------------------
  // Gyro offset
  // ---------------------------------------------------------------------------------------------------------

  // Throw away the trashy first ones samples
  for (int i = 0; i < 10; i++)
  {
    gyroscope.Gyro_GetDataZ();
  }

  for (int i = 0; i < gyro_samples; i++)
  {
    int raw_data = gyroscope.Gyro_GetDataZ();
    offset += raw_data;
    usleep(1000);
  }

  offset = offset / gyro_samples;
  // nh.setParam("/Heading/GYRO_OFFSET", offset);

  ROS_INFO("Heading offset sampled: %f ", offset);
  nh.getParam("/Heading/GYRO_OFFSET", offset);
  ros::Timer gyro_timer = nh.createTimer(ros::Duration(0.01), gyroTimer);
  //---------------------------------------------------------------------------------------------------------------------

  //---------------------------------------------------------------------------------------------------------------------
  // Calibration service
  // -------------------------------------------------------------------------------------------------
  tcc_msgs::CalibrateInt srv;
  calibrate_client.call(srv);

  if (use_mag)
  {
    usleep(30 * 1000);

    I2C_data msg;
    int calibration_counter = 0;

    gyro_data = gyroscope.Gyro_GetDataZ();

    raw_data = gyro_data;
    raw_data -= offset;

    gyro_angle += (raw_data * SCALE_2000) * dt;

    gyro_angle = gyroscope.constrainGyroAngle(gyro_angle);

    while (ros::ok())
    {
      msg = magnetometer.GetData();
      // std::cout << msg.data1 << " - " << msg.data2 << " - " << msg.data3 <<
      // std::endl;

      x_out = (msg.data1);
      y_out = (msg.data3);

      max_x = (x_out > max_x) ? (x_out) : (max_x);
      max_x = 117;
      min_x = (x_out < min_x) ? (x_out) : (min_x);
      min_x = -250;

      max_y = (y_out > max_y) ? (y_out) : (max_y);
      max_y = 740;
      min_y = (y_out < min_y) ? (y_out) : (min_y);
      min_y = 380;

      gyro_data = gyroscope.Gyro_GetDataZ();

      raw_data = gyro_data;
      raw_data -= offset;

      gyro_angle -= (raw_data * SCALE_2000) * dt;

      gyro_angle = gyroscope.constrainGyroAngle(gyro_angle);

      calibration_counter++;
      if (calibration_counter > calibration_time * freq)
        break;

      loop.sleep();
    }

    msg = magnetometer.GetData();

    x_out = (msg.data1);
    y_out = (msg.data3);

    x_out = map(x_out, min_x, max_x, -500, 500);
    y_out = map(y_out, min_y, max_y, -500, 500);

    mag_angle = atan(y_out / x_out) * 180 / PI;

    mag_angle =
        magnetometer.constrainMagAngle(mag_angle, mag_offset, x_out, y_out);

    mag_offset = mag_angle - gyro_angle;
  }

//---------------------------------------------------------------------------------------------------------------------

#ifdef DEBUG_MAG_CALIBRATION
  int counter = 0; // debug counter
#endif

  while (ros::ok())
  {
    if (use_mag)
    {
      // read mag z data
      I2C_data mag_data_I2C;
      mag_data_I2C = magnetometer.GetData();

      x_out = mag_data_I2C.data1;
      z_out = mag_data_I2C.data2;
      y_out = mag_data_I2C.data3;

      // Remap and constrain mag values
      x_out = constrain(map(x_out, min_x, max_x, -500, 500), -500, 500);
      y_out = constrain(map(y_out, min_y, max_y, -500, 500), -500, 500);

      // Get angle
      mag_angle = atan(y_out / x_out) * 180 / PI;

      // Get angle region and fix
      mag_angle =
          magnetometer.constrainMagAngle(mag_angle, mag_offset, x_out, y_out);

      // Sensor fusing
      if (absf(gyro_angle - mag_angle) < 180)
      {
        gyro_angle = (1 - filter_merge) * gyro_angle + filter_merge * mag_angle;
      }
      else if (mag_angle < gyro_angle)
      {
        gyro_angle =
            (1 - filter_merge) * gyro_angle + filter_merge * (mag_angle + 360);
      }
      else
      {
        gyro_angle =
            (1 - filter_merge) * gyro_angle + filter_merge * (mag_angle - 360);
      }
    }

    // Final message publication
    std_msgs::Float32 msg;
    msg.data = gyro_angle;
    ROS_INFO("\nGYRO ANGLE: %f \nOffset: %f", msg.data, offset);
    heading_pub.publish(msg);

    ros::spinOnce();

    // ROS sleep to maintain the loop freq
    loop.sleep();

#ifdef DEBUG_MAG_GYRO

    system("clear");
    std::cout << "Gyro angle: " << gyro_angle << "\nMag angle: " << mag_angle
              << "\nMag offset: " << mag_offset << "\nX_out: " << x_out
              << "\nY_out: " << y_out << std::endl;

    // load RAW mag message
    tcc_msgs::magdata mag_raw_msg;
    mag_raw_msg.mag_x = mag_data_I2C.data1;
    mag_raw_msg.mag_y = mag_data_I2C.data3;
    mag_raw_msg.mag_z = mag_data_I2C.data2;

    // load mag Angle message
    std_msgs::Float32 mag_angle_msg;
    mag_angle_msg.data = mag_angle;

    // load gyro angle msg
    msg.data = gyro_angle;

    // Publish RAW mag data
    mag_raw_pub.publish(mag_raw_msg);

    // Publish mag Angle
    mag_ang_pub.publish(mag_angle_msg);

#ifdef DEBUG_MAG_CALIBRATION
    std_msgs::Float32 mag_out;
    mag_out.data = mag_angle;
    raw_pub.publish(mag_out);

    mag_data.mag_x = x_out;
    mag_data.mag_y = y_out;

    mag_pub.publish(mag_data);

    counter++;
    if (counter > 20)
    {
      counter = 0;
      ROS_INFO("X - Min: %d - Max: %d", min_x, max_x);
      ROS_INFO("Y - Min: %d - Max: %d", min_y, max_y);
    }
#endif

#endif
  }

  return 0;
}
