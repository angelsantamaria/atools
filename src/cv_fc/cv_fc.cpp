#include "cv_fc.h"

namespace atools{

  bool open_camera(const int& cam_num, cv::VideoCapture& cam)
  {
    cv::VideoCapture camera(cam_num);
    if(!camera.isOpened())  // check if succeeded
    {
      std::cerr << "[atools]: cv ERROR: Could not open camera: " << cam_num << std::endl;
      return false;
    }
    cam = camera;
    return true;
  }

  bool get_frame(cv::VideoCapture& cam, cv::Mat& frame)
  {
    try
    {
      while (true) 
      {
        cam >> frame; 
        if(!frame.empty())
          break;
        cv::waitKey(33); // delay 33ms       
      }
    }
    catch( cv::Exception& e )
    {
      std::cout << "[atools]: cv WARN: An exception occurred. Ignoring frame. " << e.err << std::endl;
      return false;
    }
    return true;
  }

  bool show_frame(const std::string& window_name, const cv::Mat& frame)
  {
    try
    {
      cv::imshow(window_name, frame);
    }
    catch( cv::Exception& e )
    {
      std::cout << "[atools]: cv ERROR: An exception occurred. Ignoring frame. " << e.err << std::endl;
      return false;
    }
    return true;
  }

} // End of namespace atools
