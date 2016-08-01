#ifndef _CV_FC_H
#define _CV_FC_H

// std stuff
#include <iostream>
#include <string>

// OpenCV stuff
#include "opencv2/opencv.hpp"

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
  bool open_camera(const int& cam_num, cv::VideoCapture& cam);

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
  bool get_frame(cv::VideoCapture& cam, cv::Mat& frame);

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
	bool show_frame(const std::string& window_name, const cv::Mat& frame);

} // End of atools namespace

#endif