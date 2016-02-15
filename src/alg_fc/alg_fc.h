#ifndef _ALG_FC_H
#define _ALG_FC_H

#include <stdio.h>
#include <iostream>
#include <fstream>
#include <unistd.h>
#include <string>
#include <sstream>
#include <pwd.h>
#include <math.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <numeric> 
#include <vector>
#include <eigen3/Eigen/Dense>
/*#include <Eigen/Eigenvalues>
#include <Eigen/SVD>*/

#include <math_fc.h>

using namespace Eigen;
using namespace std;

// Sort definitions
typedef pair<int,int> mypair;

namespace atools{


    /**
    * \brief Comparator
    *
    * Simple comparator of pairs.
    */
    bool comparator ( const mypair& l, const mypair& r);

    /**
    * \brief Ourlier removal
    *
    * Outlier removal based on Thompson Tau table.  
    *
    Input:  
    *       data_raw:   First row: indexes; second row: data.
    *       met:    Method to fill outliers: l:last right value; n:nan value;
    *   Output:
    *       data_out: Output data matrix with outliers filled depending on "met".
    *       outliers_idx: Outliers original matrix position indexes.
    */
    void check_outliers(const MatrixXf& data_raw, const char& met, MatrixXf& data_out, MatrixXf& outliers_idx);

    /**
    * \brief Sort using indexes
    *
    * Sort data using the indicated indexes. Return sorted data matrix.
    *   Input:  
    *       First row: indexes.
    *       Second row: data.
    *
    */      
    MatrixXf sort_with_idx(const MatrixXf& data_raw);

    class COutlier_detector
    {
      private:

        float num;
        float mean;
        float M2;
        float data;
        float stddev;

      public:
        COutlier_detector();
        ~COutlier_detector();
        void online_mean_stddev(const float& data);
        bool check_if_outlier(const float& data);

      
    }; 

    /**
    * \brief Get random number
    *
    * Get random number. Seed initialized every time with system clock.
    */    
    float get_rand();

} // End of namespace atools

#endif