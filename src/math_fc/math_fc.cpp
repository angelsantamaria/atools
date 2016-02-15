#include "math_fc.h"

using namespace Eigen;
using namespace std;

namespace atools{

  float std_deviation(const MatrixXf& vec,const float m)
  {
    vector<float> diff(vec.cols());
    transform(vec.data(),vec.data()+vec.cols(), diff.begin(),bind2nd(minus<float>(), m));
    float sq_sum = inner_product(diff.begin(), diff.end(), diff.begin(), 0.0);
    float stdev = sqrt(sq_sum / vec.cols());
    return stdev;
  }

  float std_deviation(const MatrixXf& vec)
  {
    float m = vec.mean();
    return std_deviation(vec,m);
  }

  void norm_vector(const MatrixXf& vec, MatrixXf& vecn)
  {
    VectorXf v;
    if (vec.cols()>vec.rows())
    {
      v = vec.row(0);
    }
    else
    {
      v = vec.col(0);
    }

    float n2 = v.dot(v);
    float n = sqrt(n2);

    vecn = vec/n;
  }
  void norm_vector(const MatrixXf& vec,const int& jacMethod, MatrixXf& vecn, MatrixXf& jacobian)
  {
    VectorXf v;
    if (vec.cols()>vec.rows())
    {
      v = vec.row(0);
    }
    else
    {
      v = vec.col(0);
    }

    float n2 = v.dot(v);
    float n = sqrt(n2);

    vecn = vec/n;

    float s = vec.size();

    switch (jacMethod){
      case 0: //Scalar method
        jacobian = MatrixXf::Identity(s,s)/n;
        break;
      case 1: //Exact method
        float n3 = n*n2;
        MatrixXf vec2 = vec*vec.transpose();
        MatrixXf mvec2 = vec2(0,0)*MatrixXf::Ones(s,s);
        jacobian = ((n2*MatrixXf::Identity(s,s))-mvec2)/n3;
        break;
    }
  }

} // End of namespace atools
