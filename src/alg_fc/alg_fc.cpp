#include "alg_fc.h"

using namespace Eigen;
using namespace std;
// using namespace atools;

namespace atools{

  bool comparator ( const mypair& l, const mypair& r)
  { return l.second < r.second; }


  void check_outliers(const MatrixXf& data_raw, const char& met, MatrixXf& data_out, MatrixXf& outliers_idx)
  {
    int len_1 = data_raw.cols();
    int max_outliers = len_1;
    data_out = MatrixXf::Zero(2,len_1);

    for (int ii = 0; ii < len_1; ++ii)
    {
      data_out(0,ii) = ii;
      data_out(1,ii) = data_raw(0,ii);
    }  
     
    MatrixXf data = data_out;  

    data = sort_with_idx(data_raw);

    // Check if sort has deleted values
    int len = data.cols();

    // Math functions object
    // CMath_fc math_fc;

    // Mean and std deviation
    float xbar = data.row(1).mean();
    float stdev = std_deviation(data.row(1),xbar);


    // tau is a vector containing values for Thompson's Tau:
    MatrixXf tau(38,1);
    tau << 1.150,1.393,1.572,1.656,1.711,1.749,1.777,1.798,1.815,1.829,1.840,1.849,1.858,1.865,1.871,1.876,1.881,1.885,1.889,1.893,1.896,1.899,1.902,1.904,1.906,1.908,1.910,1.911,1.913,1.914,1.916,1.917,1.919,1.920,1.921,1.922,1.923,1.924;

    // Determine the value of stdev times Tau
    float tauS;
    if (len > tau.cols()) 
      tauS=1.960*stdev; //For n > 40
    else
      tauS=tau(data.cols(),0)*stdev; //For samples of size 3 < n < 40


    // Compare the values of extreme high/low data points with tauS:
    int ii=0;
    outliers_idx = MatrixXf::Zero(1,data.cols());
    MatrixXf tmp;
    while (max_outliers > 0)
    {  
      if (abs(abs(data(1,data.cols()-1))-xbar) > tauS)
      {
        // Get outlier index and set NAN value
        outliers_idx(0,ii) = data(0,data.cols()-1);
        data_out(1,outliers_idx(0,ii))=NAN;

        // Reduce unexplored list
        tmp = data.block(0,0,2,len-1);
        data.resize(2,len-1);
        data = tmp;
        len = data.cols();

        // Increment explored index
        ii=ii+1;
      
        // Determine the NEW value of S times tau
        xbar = data.row(1).mean();
        stdev = std_deviation(data.row(1),xbar);

        if (len > tau.cols()) 
          tauS=1.960*stdev; //For n > 40
        else
          tauS=tau(len,0)*stdev; //For samples of size 3 < n < 40
      }
      max_outliers=max_outliers-1; //reduce requested num_outliers by 1
    } 

    tmp = outliers_idx.block(0,0,1,ii);
    outliers_idx.resize(1,ii);
    outliers_idx = tmp;

    switch (met){
    	case 'l':
    	  //Assign the last non nan value to every outlier
    	  for (int ii = 0; ii < outliers_idx.cols(); ++ii)
    	  {
    	    uint idx = outliers_idx(0,ii);
    	    while (isnan(data_out(1,outliers_idx(ii))) && idx!=0)
    	    {
    	      data_out(1,outliers_idx(ii)) = data_out(1,idx);
    	      idx=idx-1;
    	    }  
    	  }
    	  break;
    }
  }

  MatrixXf sort_with_idx(const MatrixXf& data_raw)
  {
    MatrixXf data(2,data_raw.cols());  

    vector<mypair> v(data_raw.cols());

    for (int ii = 0; ii < data_raw.cols(); ++ii)
    {
      v[ii].first = ii;
      v[ii].second = abs(data_raw(0,ii));
    }

    // using function as comp
    sort (v.begin(), v.end(), comparator); // 12 32 45 71(26 33 53 80)

    for (int ii = 0; ii < data_raw.cols(); ++ii)
    {
      data(0,ii) = v[ii].first;
      data(1,ii) = v[ii].second;
    }

    return data;
  }

  COutlier_detector::COutlier_detector()
  {
    this->num=0;
    this->mean=0;
    this->M2=0;
    this->stddev = 0;
  }

  COutlier_detector::~COutlier_detector()
  {}

  void COutlier_detector::online_mean_stddev(const float& data)
  {
    this->num = this->num + 1;
    float delta = data - this->mean;
    this->mean = this->mean + delta/this->num;
    this->M2 = this->M2 +delta*(data-mean);

    if (this->num<2)
      this->stddev = 0;
    else
      this->stddev = sqrt(this->M2/(this->num-1));
  }

  bool COutlier_detector::check_if_outlier(const float& data)
  {
    online_mean_stddev(data);

    // tau is a vector containing values for Thompson's Tau:
    MatrixXf tau(38,1);
    tau << 1.150,1.393,1.572,1.656,1.711,1.749,1.777,1.798,1.815,1.829,1.840,1.849,1.858,1.865,1.871,1.876,1.881,1.885,1.889,1.893,1.896,1.899,1.902,1.904,1.906,1.908,1.910,1.911,1.913,1.914,1.916,1.917,1.919,1.920,1.921,1.922,1.923,1.924;

    // Determine the value of stdev times Tau
    float tauS;
    if (this->num > tau.cols()) 
      tauS=this->stddev; //For n > 40
      // tauS=1.960*this->stddev; //For n > 40
    else
      tauS=tau(this->num,0)*this->stddev; //For samples of size 3 < n < 40

    float delta = data - this->mean;

    if(delta > tauS)
      return true;
    else
      return false;
  }

  float get_rand()
  {
    // New seed 
    timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts); // Works on Linux
    srand (ts.tv_nsec);

    float r = ((float) rand()/((float) RAND_MAX));
    return r;
  }
} // End of atools namespace
