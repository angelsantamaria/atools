#ifndef _MATH_FC_H
#define _MATH_FC_H

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

//Eigen stuff
#include <eigen3/Eigen/Dense>

using namespace Eigen;
using namespace std;

namespace atools{
    
    /**
    * \brief Sign
    *
    * Sign function returns -1 for negative and +1 for positive numbers.
    */
    template <typename T> int sign(T val) {
        return (T(0) < val) - (val < T(0));
    }

    /**
    * \brief STD. deviation
    *
    * Computes the standard deviation of the data.
    * It normalizes by data size (cols num.) and produces the square root of
    * the second moment of the sample about its mean
    */
    float std_deviation(const MatrixXf& vec,const float mean);
    float std_deviation(const MatrixXf& vec);

    /**
    * \brief Normalize vector
    *
    *  Returns the the unit length vector in the same direction and sense as
    *  V (It is equal to V/norm(V)) and its jacobian.
    *  jacMethod: Jacobian method to apply. 0: scalar. 1: full vector.
    */
    void norm_vector(const MatrixXf& vec, MatrixXf& vecn);
    void norm_vector(const MatrixXf& vec,const int& jacMethod, MatrixXf& vecn, MatrixXf& jacobian);
} // End of atools namespace

#endif