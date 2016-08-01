#include "debug_fc.h"
#include "cv_fc.h"

namespace cv_test
{
  std::string get_yn(const string& msg)
  {
    std::string input = "y";
    atools::print(msg,cyan);
  
    while (true)
    {
      getline(cin, input);
  
      if (input.compare("y") == 0 || input.compare("n") == 0)
        break;
      else if (input.empty())
      {
        input = "y";
        break;
      }
      else 
        atools::print("Invalid key, please try again.\n",red);
    }
    return input;
  }
}

int main(int argc, char *argv[])
{
  stringstream text;

  // Check print function
  atools::print("Program to Test individual CV\n",green);  

  string perm;
  Eigen::MatrixXf data_m(1,10);

  perm = cv_test::get_yn("Do you want to test cv functions? (Y/n)\n");
  if (perm.compare("y") == 0){
    // Open USB camera
    atools::print("...Openning camera.\n", white);
    cv::VideoCapture cam;
    atools::open_camera(0, cam);

    atools::print("...Acquiring.\n", white);
    atools::print("...press any key to stop visualization.\n", white);
    while (true) 
    {
      // Get frame
      cv::Mat frame;
      atools::get_frame(cam, frame);

      // Show frame
      atools::show_frame("Test frame",frame);

      if(cv::waitKey(30) >= 0) break;
    }     
  }
  else
    atools::print("Test finished.\n",green);  
}
