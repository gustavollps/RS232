#ifndef SERVICE_MANAGER
#define SERVICE_MANAGER

#include <ros/ros.h>

namespace service_manager
{

int param_dilate = 10;
int param_erode = 10;
int param_gaussian = 10;
int camera = 1;
cv::Scalar param_top_boundarie(255,255,255),param_bottom_boundarie(0,0,0);

void getParams(){
	ros::NodeHandle nh_;

    if(!nh_.getParam("/tcc/image_processing/camera",camera))
        nh_.setParam("/tcc/image_processing/camera",camera);

    if(!nh_.getParam("/tcc/image_processing/intensity_dilate",param_dilate))
        nh_.setParam("/tcc/image_processing/intensity_dilate",param_dilate);

    if(!nh_.getParam("/tcc/image_processing/intensity_erode",param_erode))
        nh_.setParam("/tcc/image_processing/intensity_erode",param_erode);

    if(!nh_.getParam("/tcc/image_processing/intensity_gaussian",param_gaussian))
        nh_.setParam("/tcc/image_processing/intensity_gaussian",param_gaussian);  

    if(!nh_.getParam("/tcc/image_processing/TOPCOLOR/L_1",param_top_boundarie[0]))
        nh_.setParam("/tcc/image_processing/TOPCOLOR/L_1",param_top_boundarie[0]);

    if(!nh_.getParam("/tcc/image_processing/TOPCOLOR/L_2",param_top_boundarie[1]))
        nh_.setParam("/tcc/image_processing/TOPCOLOR/L_2",param_top_boundarie[1]);    

    if(!nh_.getParam("/tcc/image_processing/TOPCOLOR/L_3",param_top_boundarie[2]))
        nh_.setParam("/tcc/image_processing/TOPCOLOR/L_3",param_top_boundarie[2]);

    if(!nh_.getParam("/tcc/image_processing/BOTTOMCOLOR/L_1",param_bottom_boundarie[0]))
        nh_.setParam("/tcc/image_processing/BOTTOMCOLOR/L_1",param_bottom_boundarie[0]);
    
    if(!nh_.getParam("/tcc/image_processing/BOTTOMCOLOR/L_2",param_bottom_boundarie[1]))
        nh_.setParam("/tcc/image_processing/BOTTOMCOLOR/L_2",param_bottom_boundarie[1]);

    if(!nh_.getParam("/tcc/image_processing/BOTTOMCOLOR/L_3",param_bottom_boundarie[2]))
        nh_.setParam("/tcc/image_processing/BOTTOMCOLOR/L_3",param_bottom_boundarie[2]);

}

}   //namespace service manager
#endif
