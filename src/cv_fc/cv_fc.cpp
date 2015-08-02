#include "cv_fc.h"

using namespace std;
using namespace cv;

namespace atools{

  bool open_camera(const int& cam_num, VideoCapture& cam)
  {
    VideoCapture camera(cam_num);

      if(!camera.isOpened())  // check if we succeeded
      {
          cerr << "ERROR: Could not open camera: " << cam_num << endl;
          return false;
      }

      cam = camera;
      return true;
  }

  bool get_frame(VideoCapture& cam, Mat& frame)
  {
      try
      {
          cam >> frame; 
      }
      catch( Exception& e )
      {
          cout << "An exception occurred. Ignoring frame. " << e.err << endl;
          return false;
      }

      return true;
  }


  bool show_frame(const string& window_name, const Mat& frame)
  {
    try
      {
          imshow(window_name, frame);
      }
      catch( Exception& e )
      {
          cout << "An exception occurred. Ignoring frame. " << e.err << endl;
          return false;
      }

      return true;
  }

//<-- RECTIFICAR ---
  //http://ece631web.groups.et.byu.net/Lectures/ECEn631%2014%20-%20Calibration%20and%20Rectification.pdf

  //stereoRectify
  //initUndistortRectifyMap
  //remap

  void get_disparity_map_8UINT(const Mat& left, const Mat& right, StereoBM& sbm, Mat& disp8)
  {
    Mat disp;
    sbm(left, right, disp);
    normalize(disp, disp8, 0, 255, CV_MINMAX, CV_8U);
  }


} // End of namespace atools
