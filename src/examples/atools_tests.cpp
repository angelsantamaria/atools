#include "alg_fc.h"
#include "debug_fc.h"
#include "math_fc.h"
#include "rot_fc.h"

namespace atools_test
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
  atools::print("Program to Test individual functions involved in ESKF Odometry algorithm\n",green);	

  std::string perm;
  Eigen::MatrixXf data_m(1,10);

  perm = atools_test::get_yn("Do you want to test math functions? (Y/n)\n");
  if (perm=="y"){
    // Check std
    atools::print("_____STD dev.____________\n",magenta);	
    data_m << 1,2,3,4,5,6,7,8,9,10;
    float std_dev = atools::std_deviation(data_m);
    atools::print("Initial data: ",white);
    atools::print(data_m,white);
    text.str(string());
    text << "\nResult: " << std_dev;
    atools::print(text.str(),white); 
    std::cout << std::endl;

    // Check vector norm
    atools::print("_____Vector Norm____________\n",magenta);   
    data_m << 1,2,3,4,5,6,7,8,9,10;
    Eigen::MatrixXf vec_n;
    Eigen::MatrixXf jacobian;
    atools::norm_vector(data_m,0,vec_n,jacobian);
    atools::print("Initial data: ",white);
    atools::print(data_m,white);
    atools::print("\nResult: \n norm = \n",white); 
    atools::print(vec_n,white);  
    atools::print("\n Jacobian: \n",white);
    atools::print(jacobian,white);
    std::cout << std::endl;
  }

  perm = atools_test::get_yn("Do you want to test alg functions? (Y/n)\n");
  if (perm=="y"){
    // Sort with index check
    atools::print("_____Sort matrix with indexes__\n",magenta);	
    data_m << 4,7,3,6,8,9,2,1,10,5;
    Eigen::MatrixXf sorted = atools::sort_with_idx(data_m);
    atools::print("Initial data: ",white);
    atools::print(data_m,white);
    atools::print("\nResult: \n",white);
    atools::print(sorted,white);
    std::cout << std::endl;

    // Check outliers
    atools::print("_____Remove Outliers____________\n",magenta);	
    data_m << 10,9,11,18,8,9,10,25,10,9;
    Eigen::MatrixXf no_out,idx_out;
    atools::check_outliers(data_m,'n',no_out,idx_out);
    atools::print("Initial data: ",white);
    atools::print(data_m,white);
    text.str(std::string());
    text << "\nResult: \n";
    atools::print(text.str(),white);  
    atools::print(no_out,white);
    atools::print("\nOutlier indexes: \n",white);
    atools::print(idx_out,white);
    std::cout << std::endl;

    // Check outliers
    atools::print("_____Online Check if outlier____________\n",magenta);  
    bool out;
    atools::COutlier_detector det_out;
    atools::print("Initial data: ",white);
    atools::print(data_m,white);
    text.str(string());
    text << "\nResult: \n";
    atools::print(text.str(),white);  

    for (int ii = 0; ii < data_m.cols(); ++ii)
    {
      out = det_out.check_if_outlier(data_m(0,ii));
      atools::print(out,white);
      atools::print(",",white);
    }
    std::cout << std::endl;
  }

 perm = atools_test::get_yn("Do you want to test rot functions? (Y/n)\n");
 if (perm=="y"){

   // Vector to skew
   atools::print("_____Vector to skew________\n",magenta);
   Eigen::Vector3f v;
   v << 1,2,3;
   Eigen::Matrix3f M_sk;
   Eigen::MatrixXf V_sk;
   atools::v2skew(v,M_sk,V_sk);
   atools::print("Initial data: \n",white);
   atools::print(v,white);
   atools::print("\nResult: \n",white);
   atools::print("\nSkew symmetric matrix: \n",white);
   atools::print(M_sk,white);
   atools::print("\nJ_sk: \n",white);
   atools::print(V_sk,white);
   std::cout << std::endl;

   // vec to angle-axis
   atools::print("_____Vector to Angle-axis____________\n",magenta);
   float angle;
   Eigen::Vector3f axis;
   Eigen::MatrixXf Aangle_v,Aaxis_v;
   atools::v2aaxis(v,angle,axis,Aangle_v,Aaxis_v);
   atools::print("Initial data: \n",white);
   atools::print(v,white);
   atools::print("\nResult: \n",white);
   atools::print("\nAngle: ",white);
   atools::print(angle,white);
   atools::print("\nAxis: \n",white);
   atools::print(axis,white);
   atools::print("\nJangle_v: \n",white);
   atools::print(Aangle_v,white);
   atools::print("\nJaxis_v: \n",white);
   atools::print(Aaxis_v,white);
   std::cout << std::endl;

   // vec to R
   atools::print("_____Rotation vector to R____________\n",magenta);
   Eigen::Matrix3f R;
   atools::v2R(v,R);
   atools::print("Initial data: \n",white);
   atools::print(v,white);
   atools::print("\nResult: \n",white);
   atools::print(R,white);
   std::cout << std::endl;

   // vec to q
   atools::print("_____Vector to quaternion____________\n",magenta);
   Eigen::Quaternionf q;
   Eigen::MatrixXf Q_v;
   atools::v2q(v,q,Q_v);
   atools::print("Initial data: \n",white);
   atools::print("Vector: \n",white);
   atools::print(v,white);
   atools::print("\nResult: \n",white);
   atools::print("\nQuaternion: \n",white);
   atools::print(q,white);
   atools::print("\nJq_v: \n",white);
   atools::print(Q_v,white);
   std::cout << std::endl;

   // Aaxis to q
   atools::print("_____Angle-axis to Quaternion____________\n",magenta);
   Eigen::MatrixXf Q_angle,Q_axis;
   atools::aaxis2q(angle,axis,q,Q_angle,Q_axis);
   atools::print("Initial data: \n",white);
   atools::print("Angle: ",white);
   atools::print(angle,white);
   atools::print("\nAxis: \n",white);
   atools::print(axis,white);
   atools::print("\nResult: \n",white);
   atools::print("\nQuaternion: \n",white);
   atools::print(q,white);
   atools::print("\nJ_angle: \n",white);
   atools::print(Q_angle,white);
   atools::print("\nJ_axis: \n",white);
   atools::print(Q_axis,white);
   std::cout << std::endl;


   // w to omega
   atools::print("_____w to Omega____________\n",magenta);
   Eigen::Vector3f w;
   w << 1.0,1.5,2.0;
   Eigen::Matrix4f Omega;
   Eigen::MatrixXf O_w;
   atools::w2omega(w,Omega,O_w);
   atools::print("Initial data: \n",white);
   atools::print(w,white);
   atools::print("\nResult: \n",white);
   atools::print("\nOmega: \n",white);
   atools::print(Omega,white);
   atools::print("\nJ_w: \n",white);
   atools::print(O_w,white);
   std::cout << std::endl;

   // theta to q
   atools::print("_____Theta to q____________\n",magenta);
   Eigen::Vector3f theta;
   theta << 1.0,2.0,3.0;
   Eigen::Quaternionf qt;
   atools::theta2q(theta,q);
   atools::print("Initial data: \n",white);
   atools::print("Theta: \n",white);
   atools::print(theta,white);
   atools::print("\nResult: \n",white);
   atools::print("\nq: \n",white);
   atools::print(q,white);
   std::cout << std::endl;

   // R to q
   atools::print("_____R to q____________\n",magenta);
   Eigen::Vector3f v2;
   Eigen::Matrix3f R2;
   Eigen::Quaternionf q2;
   v2 << 5,6,7;
   atools::v2R(v2,R2);
   atools::R2q(R2,q2);
   atools::print("Initial data: \n",white);
   atools::print("Rotation: \n",white);
   atools::print(R2,white);
   atools::print("\nResult: \n",white);
   atools::print("\nQuaternion: \n",white);
   atools::print(q2,white);
   std::cout << std::endl;

   // R to e
   atools::print("_____R to Euler____________\n",magenta);
   Eigen::Vector3f euler;
   atools::R2e(R2,euler);
   atools::print("Initial data: \n",white);
   atools::print("\nRotation Matrix: \n",white);
   atools::print(R2,white);
   atools::print("\nResult: \n",white);
   atools::print("Euler: \n",white);
   atools::print(euler,white);
   std::cout << std::endl;

   // qproduct
   atools::print("_____Quaternion product____________\n",magenta);
   atools::qProd(q,q,qt);
   atools::print("Initial data: \n",white);
   atools::print("q1: \n",white);
   atools::print(q,white);
   atools::print("\nq2: \n",white);
   atools::print(q,white);
   atools::print("\nResult: \n",white);
   atools::print("\nQuaternion: \n",white);
   atools::print(qt,white);
   std::cout << std::endl;

   // qPredict
   atools::print("_____Quaternion Prediction____________\n",magenta);
   float dt = 1;
   int met = 1;
   Eigen::Quaternionf qpred;
   Eigen::MatrixXf Q_q,Q_w;
   atools::qPredict(q,w,qpred,dt,met,Q_q,Q_w);
   atools::print("Initial data: \n",white);
   atools::print("q: \n",white);
   atools::print(q,white);
   atools::print("\nw: \n",white);
   atools::print(w,white);
   atools::print("\ndt: ",white);
   atools::print(dt,white);
   atools::print("\nMethod: ",white);
   atools::print(met,white);
   atools::print("\nResult: \n",white);
   atools::print("\nQuaternion: \n",white);
   atools::print(qpred,white);
   atools::print("\nQ_q: \n",white);
   atools::print(Q_q,white);
   atools::print("\nQ_w: \n",white);
   atools::print(Q_w,white);
   std::cout << std::endl;

   // q2R
   atools::print("_____q to R____________\n",magenta);
   Eigen::MatrixXf JR_q;
   atools::q2R(q,R,JR_q);
   atools::print("Initial data: \n",white);
   atools::print("q: \n",white);
   atools::print(q,white);
   atools::print("\nResult: \n",white);
   atools::print("\nRotation: \n",white);
   atools::print(R,white);
   atools::print("\nJR_q: \n",white);
   atools::print(JR_q,white);
   std::cout << std::endl;

   atools::print("_____Quaternion Conjugate____________\n",magenta);
   Eigen::MatrixXf Q_qc;
   Eigen::Quaternionf qc;
   atools::q2qc(q,qc,Q_qc);
   atools::print("Initial data: \n",white);
   atools::print("q: \n",white);
   atools::print(q,white);
   atools::print("\nResult: \n",white);
   atools::print("\nConjugate: \n",white);
   atools::print(qc,white);
   atools::print("\nQ_qc: \n",white);
   atools::print(Q_qc,white);
   std::cout << std::endl;

   atools::print("_____Quaternion to Euler____________\n",magenta);
   Eigen::MatrixXf E_q;
   Eigen::Vector3f e;
   atools::q2e(q,e,E_q);
   atools::print("Initial data: \n",white);
   atools::print("q: \n",white);
   atools::print(q,white);
   atools::print("\nResult: \n",white);
   atools::print("\nEuler: \n",white);
   atools::print(e,white);
   atools::print("\nE_q: \n",white);
   atools::print(E_q,white);
   std::cout << std::endl;

   // q to Aaxis
   atools::print("_____Quaternion to Angle Axis____________\n",magenta);
   Eigen::MatrixXf Aangle_q,Aaxis_q;
   atools::q2aaxis(q,angle,axis,Aangle_q,Aaxis_q);
   atools::print("Initial data: \n",white);
   atools::print("\nQuaternion: \n",white);
   atools::print(q,white);
   atools::print("\nResult: \n",white);
   atools::print("Angle: ",white);
   atools::print(angle,white);
   atools::print("\nAxis: \n",white);
   atools::print(axis,white);
   atools::print("\nAangle_q: \n",white);
   atools::print(Aangle_q,white);
   atools::print("\nAaxis_q: \n",white);
   atools::print(Aaxis_q,white);
   std::cout << std::endl;

   // e to R
   atools::print("_____Euler to R____________\n",magenta);
   Eigen::MatrixXf JR_e;
   atools::e2R(euler,R,JR_e);
   atools::print("Initial data: \n",white);
   atools::print("Euler: \n",white);
   atools::print(euler,white);
   atools::print("\nResult: \n",white);
   atools::print("\nRotation Matrix: \n",white);
   atools::print(R,white);
   atools::print("\nJR_e: \n",white);
   atools::print(JR_e,white);
   std::cout << std::endl;

   // e to q
   atools::print("_____Euler to Quaternion____________\n",magenta);
   Eigen::MatrixXf Q_e;
   atools::e2q(euler,q,Q_e);
   atools::print("Initial data: \n",white);
   atools::print("Euler: \n",white);
   atools::print(euler,white);
   atools::print("\nResult: \n",white);
   atools::print("\nQuaternion: \n",white);
   atools::print(q,white);
   atools::print("\nQ_e: \n",white);
   atools::print(Q_e,white);
   std::cout << std::endl;

 }
}
