#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <vector>
#include "CamVision.h"
#include "service_manager.h"
#include "opencv_node/color.h"

using namespace tcc;
using namespace cv;

cv::Scalar bottom_color, top_color;
cv::Scalar c1_t, c2_t, c3_t;

bool getServerParams(opencv_node::SetParams::Request& req,
                     opencv_node::SetParams::Response& res)
{
  service_manager::getParams();
  return true;
}

void colorcallBack(opencv_node::color msg)
{
  bottom_color[0] = msg.c1_b;
  bottom_color[1] = msg.c2_b;
  bottom_color[2] = msg.c3_b;

  top_color[0] = msg.c1_t;
  top_color[1] = msg.c2_t;
  top_color[2] = msg.c3_t;

  ROS_INFO("New color \n%d \n%d \n %d \n--- \n%d \n%d \n %d", msg.c1_b,
           msg.c2_b, msg.c3_b, msg.c1_t, msg.c2_t, msg.c1_t);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "opencv_node");
  ros::NodeHandle nh;

  service_manager::getParams();

  image_processing::CamVision img_processor(0);

  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("camera/image_raw", 1);
  ros::Subscriber color_sub =
      nh.subscribe<opencv_node::color>("Color", 10, colorcallBack);

  ros::ServiceServer service =
      nh.advertiseService("SetParams", getServerParams);

  ros::Rate loopRate(30);

  cv::Mat cameraFeed;

  cv_bridge::CvImagePtr cv_ptr;

  while (ros::ok())
  {

    img_processor.findObjects(cameraFeed, top_color, bottom_color);

    img_processor.setFilters(service_manager::param_erode,
                             service_manager::param_dilate,
                             service_manager::param_gaussian);

    sensor_msgs::ImagePtr msg =
        cv_bridge::CvImage(std_msgs::Header(), "bgr8", img_processor.threshold_)
            .toImageMsg();

    service_manager::getParams();
    pub.publish(msg);

    ros::spinOnce();
    loopRate.sleep();
  }

  return 0;
}
