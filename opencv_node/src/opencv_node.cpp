#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <vector>
#include "CamVision.h"  
#include "service_manager.h"

using namespace tcc;
using namespace cv;

bool getServerParams(opencv_node::SetParams::Request &req, 
                     opencv_node::SetParams::Response &res)
{
    service_manager::getParams();
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "opencv_node");
    ros::NodeHandle nh;

    service_manager::getParams(); 

    image_processing::CamVision img_processor(1);

    

    image_transport::ImageTransport it(nh);    
    image_transport::Publisher pub = it.advertise("camera/image_raw", 1);

    ros::ServiceServer service = nh.advertiseService("SetParams", getServerParams);    

    ros::Rate loopRate(30);

    cv::Mat cameraFeed;

    cv_bridge::CvImagePtr cv_ptr;

    while(ros::ok()){

        img_processor.findObjects(  cameraFeed,
                                    service_manager::param_top_boundarie, 
                                    service_manager::param_bottom_boundarie);
        
        img_processor.setFilters(   service_manager::param_erode,
                                    service_manager::param_dilate, 
                                    service_manager::param_gaussian);

        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(
                                    std_msgs::Header(), 
                                    "bgr8", 
                                    img_processor.threshold_).toImageMsg();
        
        service_manager::getParams();
        pub.publish(msg);
        
        ros::spinOnce();    
        loopRate.sleep();
    }    

    return 0;    
}
