#ifndef _CV_FC_H
#define _CV_FC_H

#include <stdio.h>
#include <iostream>
#include <fstream>
#include <unistd.h>
#include <string>
#include <sstream>
#include <pwd.h>
#include <math.h>
#include <sys/stat.h>
#include <numeric> 
#include <vector>

// OpenCV staff
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/contrib/contrib.hpp>

using namespace cv;
using namespace std;

namespace atools{
    
    /**
    * \brief Open Webcam
    *
	* This method opens the specified webcam using OpenCV
	* Inputs: 
	*	cam_num: system camera number (int)
	* Ouptuts: 
	*	cam: camera handle (cv::VideoCapture)
	*   
	* Returns true if the camera is correctly opened
    *
    */
    bool open_camera(const int& cam_num, VideoCapture& cam);

	/** 
	* \brief Get frame
	* 
	* This method gets a frame from the specified webcam
	*
	* Inputs:
	*	cam: camera handle (cv::VideoCapture)
	* Outputs:
	*	frame: filled frame (cv::Mat)
	* Returns true if the frame is correctly obtained
	*/
    bool get_frame(VideoCapture& cam, Mat& frame);

	/** 
	*\brief Show Frame
	*
	* This method shows the specified frame using OpenCV
	*
	* Inputs:
	*	window_name: Window name inwhich the frame will be displayed (string)
	*	frame: Frame to be displayed (cv::Mat)
	* Returns true if the frame is correctly displayed
	*/
	bool show_frame(const string& window_name, const Mat& frame);

	/** 
	*\brief Get disparity Map (8UINT)
	*
	* This method computes the disparity map in a 8UINT Mat image
	*
	* Inputs:
	*	left: Left image.
	*   right: Right image.
	* 	sbm: sbm object with disparity map paramters such as:
	*
    *  		sbm.state->SADWindowSize = 9;
    *  		sbm.state->numberOfDisparities = 112;
    *  		sbm.state->preFilterSize = 5;
    *  		sbm.state->preFilterCap = 61;
    *  		sbm.state->minDisparity = -39;
    *  		sbm.state->textureThreshold = 507;
    *  		sbm.state->uniquenessRatio = 0;
    *  		sbm.state->speckleWindowSize = 0;
    *  		sbm.state->speckleRange = 8;
    *  		sbm.state->disp12MaxDiff = 1;
	*
	* Output:
	*	disp: Disparity Map.
	*/
    void get_disparity_map_8UINT(const Mat& left, const Mat& right, StereoBM& sbm, Mat& disp);

} // End of atools namespace

#endif