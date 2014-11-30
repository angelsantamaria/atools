#include "math_fc.h"

using namespace Eigen;
using namespace std;

namespace atools{

  double std_deviation(const MatrixXd& vec,const double m)
  {
    vector<double> diff(vec.cols());
    transform(vec.data(),vec.data()+vec.cols(), diff.begin(),bind2nd(minus<double>(), m));
    double sq_sum = inner_product(diff.begin(), diff.end(), diff.begin(), 0.0);
    double stdev = sqrt(sq_sum / vec.cols());
    return stdev;
  }

  double std_deviation(const MatrixXd& vec)
  {
    double m = vec.mean();
    return std_deviation(vec,m);
  }

  void norm_vector(const MatrixXd& vec, MatrixXd& vecn)
  {
    VectorXd v;
    if (vec.cols()>vec.rows())
    {
      v = vec.row(0);
    }
    else
    {
      v = vec.col(0);
    }

    double n2 = v.dot(v);
    double n = sqrt(n2);

    vecn = vec/n;
  }
  void norm_vector(const MatrixXd& vec,const int& jacMethod, MatrixXd& vecn, MatrixXd& jacobian)
  {
    VectorXd v;
    if (vec.cols()>vec.rows())
    {
      v = vec.row(0);
    }
    else
    {
      v = vec.col(0);
    }

    double n2 = v.dot(v);
    double n = sqrt(n2);

    vecn = vec/n;

    double s = vec.size();

    switch (jacMethod){
      case 0: //Scalar method
        jacobian = MatrixXd::Identity(s,s)/n;
        break;
      case 1: //Exact method
        double n3 = n*n2;
        MatrixXd vec2 = vec*vec.transpose();
        MatrixXd mvec2 = vec2(0,0)*MatrixXd::Ones(s,s);
        jacobian = ((n2*MatrixXd::Identity(s,s))-mvec2)/n3;
        break;
    }
  }

} // End of namespace atools
