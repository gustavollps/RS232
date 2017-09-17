#ifndef CAM_VISION
#define CAM_VISION

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>
#include "opencv_node/SetParams.h"
#include "definitions.h"


namespace tcc{
namespace image_processing{

using std::vector;
using namespace cv;


struct Object{
    cv::Scalar rangeMax;
    cv::Scalar rangeMin;
    cv::Scalar centerColor;
    int xPos;
    int yPos;
    int area;
};

//----------------------------------------------------------------------------------------------------------
/**
 *
 *
 */
//----------------------------------------------------------------------------------------------------------
class CamVision{
	
    private:
        int intensity_erode_;
        int intensity_dilate_;
        int intensity_gaussian_;  
        int camera_;             
        cv::VideoCapture capture_;           
	
    public:
		CamVision(int camera);
		~CamVision();

        cv::Mat raw_image_;
        cv::Mat threshold_;
        
        vector<Object> findObjects( 
                            cv::Mat cameraFeed_, 
                            cv::Scalar top_boundarie, 
                            cv::Scalar bottom_boundarie);

        void filterMat(     cv::Mat threshold  );

        void gaussianFilter(cv::Mat cameraFeed );

        void setFilters(int intensity_erode, int intensity_dilate, int intensity_gaussian);
          
};


}       //namespace imageprocessing
}     	//namespace tcc

#endif
