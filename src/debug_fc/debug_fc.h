#ifndef _DEBUG_FC_H
#define _DEBUG_FC_H

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
#include <eigen3/Eigen/Dense>
/*#include <Eigen/Eigenvalues>
#include <Eigen/SVD>
*/
using namespace Eigen;
using namespace std;

typedef enum {	
	red = 0,
    green = 1,
    yellow = 2,
    blue = 3,
    magenta = 4,
	cyan = 5,
	white = 6} color;

namespace atools{
    /**
    * \brief Print RGB
    * 
    * Print the specified string in the terminal
    */  
    void print(const string& msg, const color& c);
    void print(const float& msg, const color& c);
    void print(const MatrixXf& msg, const color& c);
    void print(const Quaternionf& msg, const color& c);
    string bash_color(const color& c);
    void restart_white(void);
}

#endif
