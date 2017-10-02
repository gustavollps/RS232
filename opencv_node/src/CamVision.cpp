#include "CamVision.h"

namespace tcc{
namespace image_processing{

//----------------------------------------------------------------------------------------------------------
/**
 *	Class constructor - nothing to implement here 
 */
//----------------------------------------------------------------------------------------------------------
CamVision::CamVision(int camera)
{

	intensity_erode_= 10;
    intensity_dilate_= 10;
    intensity_gaussian_= 9;

	camera_ = camera;  
    capture_.open(camera_);

    capture_.set(	CV_CAP_PROP_FRAME_WIDTH,
            		FRAME_WIDTH);
    capture_.set(	CV_CAP_PROP_FRAME_HEIGHT,
                	FRAME_HEIGHT);
}

//----------------------------------------------------------------------------------------------------------
/**
 *	Class destructor
 */
//----------------------------------------------------------------------------------------------------------
CamVision::~CamVision(){}


//----------------------------------------------------------------------------------------------------------
/**
 *	Find objetcs of a specific color and returns a vector with them
 */
//----------------------------------------------------------------------------------------------------------
vector<Object> CamVision::findObjects(							
							cv::Scalar top_boundarie, 
							cv::Scalar bottom_boundarie 
							)
{
  cv::Mat cameraFeed;
  capture_>>cameraFeed;
	
  //cameraFeed.copyTo(raw_image_);

  //cv::imshow("RAW",cameraFeed);
  //cv::waitKey(1);
	
	vector <Object> tracked;

	vector< vector<cv::Point> > contours;
	vector<Vec4i> hierarchy;



  //cameraFeed.copyTo(threshold_);

  CamVision::gaussianFilter(cameraFeed);


  cv::imshow("RAW",cameraFeed);
  cv::waitKey(1);

  cv::inRange(cameraFeed,bottom_boundarie,top_boundarie,threshold_);
  ROS_INFO("Filtrou aqui ");

  //CamVision::filterMat(threshold_);

  cv::imshow("Image_filtered",threshold_);
  cv::waitKey(1);


  cv::findContours(threshold_,contours,hierarchy,CV_RETR_CCOMP,CV_CHAIN_APPROX_SIMPLE );

	if (hierarchy.size() > 0) {
		int numObjects = hierarchy.size();
		
		if(numObjects<MAX_NUM_OBJECTS){
			for (int i = 0; i >= 0; i = hierarchy[i][0]) {

				Moments moment = moments((cv::Mat)contours[i]);
				double area = moment.m00;

				if(area>MIN_OBJECT_AREAX*MIN_OBJECT_AREAY && area<MAX_OBJECT_AREA){

					Object obj;
					
					obj.xPos = moment.m10/area;
					obj.yPos = moment.m01/area;
					obj.area = area;					

					tracked.push_back(obj);
				}
			}
		}
	}

	return tracked;
}

//----------------------------------------------------------------------------------------------------------
/**
 *
 *
 */
//----------------------------------------------------------------------------------------------------------
void CamVision::filterMat(	cv::Mat threshold )
{
	
	cv::Mat element_erode = cv::getStructuringElement( 	MORPH_ELLIPSE, 
													cv::Size( intensity_erode_ + 1, intensity_erode_ + 1 ), 
													cv::Point( intensity_erode_, intensity_erode_ ) 
												);

	cv::Mat element_dilate = cv::getStructuringElement( MORPH_ELLIPSE, 
													cv::Size( intensity_dilate_ + 1, intensity_dilate_ + 1 ), 
													cv::Point( intensity_dilate_, intensity_dilate_ ) 
												);

	erode(threshold,threshold,element_erode);

	dilate(threshold,threshold,element_dilate);
}

//----------------------------------------------------------------------------------------------------------
/**
 *
 *
 */
//----------------------------------------------------------------------------------------------------------

void CamVision::gaussianFilter(cv::Mat cameraFeed)
{
	cv::GaussianBlur(   cameraFeed,
						cameraFeed,
                        cv::Size(intensity_gaussian_*2+1,
                        intensity_gaussian_*2+1),
                        2.0,
                        0,
                        cv::BORDER_DEFAULT);
}

//----------------------------------------------------------------------------------------------------------
/**
 *
 *
 */
//----------------------------------------------------------------------------------------------------------
void CamVision::setFilters(int erode, int dilate, int gaussian)
{
	intensity_gaussian_ = gaussian;
	intensity_erode_ = erode;
	intensity_dilate_ = dilate;
}


}	//namespace image_processing
}	//namespace tcc

/**
 *	namespace under_scored
 *	variable var_int
 *	function theFunction
 *	class ClassThis	
 *	private member member_
 *	package this_package
 */
