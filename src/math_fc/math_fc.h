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
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <Eigen/SVD>


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
    double std_deviation(const MatrixXd& vec,const double mean);
    double std_deviation(const MatrixXd& vec);

    /**
    * \brief Normalize vector
    *
    *  Returns the the unit length vector in the same direction and sense as
    *  V (It is equal to V/norm(V)) and its jacobian.
    *  jacMethod: Jacobian method to apply. 0: scalar. 1: full vector.
    */
    void norm_vector(const MatrixXd& vec, MatrixXd& vecn);
    void norm_vector(const MatrixXd& vec,const int& jacMethod, MatrixXd& vecn, MatrixXd& jacobian);
} // End of atools namespace

#endif