#include "alg_fc.h"
#include "debug_fc.h"
#include "math_fc.h"
#include "cv_fc.h"
#include "rot_fc.h"

using namespace Eigen;
using namespace std;
using namespace cv;
using namespace atools;

string get_yn(const string& msg)
{
  string input = "y";
  print(msg,cyan);

  while (true)
  {
    getline(cin, input);

    if (input=="y" || input=="n")
      break;
    else
      print("Invalid number, please try again.\n",red);
  }
  return input;
}

int main(int argc, char *argv[])
{
  stringstream text;


  // Check print function
  print("Program to Test individual functions involved in ESKF Odometry algorithm\n",green);	

  string perm;
  MatrixXd data_m(1,10);

  // perm = get_yn("Do you want to test math functions? (Y/n)\n");
  // if (perm=="y"){
  //   // Check std
  //   print("_____STD dev.____________\n",magenta);	
  //   data_m << 1,2,3,4,5,6,7,8,9,10;
  //   double std_dev = std_deviation(data_m);
  //   print("Initial data: ",white);
  //   print(data_m,white);
  //   text.str(string());
  //   text << "\nResult: " << std_dev;
  //   print(text.str(),white); 
  //   cout << endl;

  //   // Check vector norm
  //   print("_____Vector Norm____________\n",magenta);   
  //   data_m << 1,2,3,4,5,6,7,8,9,10;
  //   MatrixXd vec_n;
  //   MatrixXd jacobian;
  //   norm_vector(data_m,0,vec_n,jacobian);
  //   print("Initial data: ",white);
  //   print(data_m,white);
  //   print("\nResult: \n norm = \n",white); 
  //   print(vec_n,white);  
  //   print("\n Jacobian: \n",white);
  //   print(jacobian,white);
  //   cout << endl;
  // }

  // perm = get_yn("Do you want to test alg functions? (Y/n)\n");
  // if (perm=="y"){
  //   // Sort with index check
  //   print("_____Sort matrix with indexes__\n",magenta);	
  //   data_m << 4,7,3,6,8,9,2,1,10,5;
  //   MatrixXd sorted = sort_with_idx(data_m);
  //   print("Initial data: ",white);
  //   print(data_m,white);
  //   print("\nResult: \n",white);
  //   print(sorted,white);
  //   cout << endl;

  //   // Check outliers
  //   print("_____Remove Outliers____________\n",magenta);	
  //   data_m << 10,9,11,18,8,9,10,25,10,9;
  //   MatrixXd no_out,idx_out;
  //   check_outliers(data_m,'n',no_out,idx_out);
  //   print("Initial data: ",white);
  //   print(data_m,white);
  //   text.str(string());
  //   text << "\nResult: \n";
  //   print(text.str(),white);  
  //   print(no_out,white);
  //   print("\nOutlier indexes: \n",white);
  //   print(idx_out,white);
  //   cout << endl;

  //   // Check outliers
  //   print("_____Online Check if outlier____________\n",magenta);  
  //   bool out;
  //   COutlier_detector det_out;
  //   print("Initial data: ",white);
  //   print(data_m,white);
  //   text.str(string());
  //   text << "\nResult: \n";
  //   print(text.str(),white);  

  //   for (int ii = 0; ii < data_m.cols(); ++ii)
  //   {
  //     out = det_out.check_if_outlier(data_m(0,ii));
  //     print(out,white);
  //     print(",",white);
  //   }
  //   cout << endl;

  // }

  // perm = get_yn("Do you want to test cv functions? (Y/n)\n");
  // if (perm=="y"){
  //   // Open USB camera
  //   print("_____Openning camera____________\n",magenta); 
  //   VideoCapture cam;
  //   open_camera(0, cam);

  //   // Get frame
  //   print("_____Get frame____________\n",magenta);    
  //   Mat frame;
  //   get_frame(cam, frame);

  //   // Show frame
  //   print("_____Show frame____________\n",magenta); 
  //   show_frame("Test frame",frame);
  // }

  perm = get_yn("Do you want to test rot functions? (Y/n)\n");
  if (perm=="y"){  

    // Vector to skew
    print("_____Vector to skew________\n",magenta);  
    Vector3d v;
    v << 1,2,3;
    Matrix3d M_sk;
    MatrixXd V_sk;
    v2skew(v,M_sk,V_sk);
    print("Initial data: \n",white);
    print(v,white);
    print("\nResult: \n",white);
    print("\nSkew symmetric matrix: \n",white);
    print(M_sk,white);
    print("\nJ_sk: \n",white);
    print(V_sk,white);
    cout << endl;

    // vec to angle-axis
    print("_____Vector to Angle-axis____________\n",magenta);  
    double angle;
    Vector3d axis;
    MatrixXd Aangle_v,Aaxis_v;
    v2aaxis(v,angle,axis,Aangle_v,Aaxis_v);
    print("Initial data: \n",white);
    print(v,white);
    print("\nResult: \n",white);
    print("\nAngle: ",white);
    print(angle,white);
    print("\nAxis: \n",white);
    print(axis,white);
    print("\nJangle_v: \n",white);
    print(Aangle_v,white);
    print("\nJaxis_v: \n",white);
    print(Aaxis_v,white);
    cout << endl;   

    // vec to R
    print("_____Rotation vector to R____________\n",magenta);  
    Matrix3d R;
    v2R(v,R);
    print("Initial data: \n",white);
    print(v,white);
    print("\nResult: \n",white);
    print(R,white);
    cout << endl;

    // vec to q
    print("_____Vector to quaternion____________\n",magenta);  
    Quaterniond q;
    MatrixXd Q_v;
    v2q(v,q,Q_v);
    print("Initial data: \n",white);
    print("Vector: \n",white);
    print(v,white);
    print("\nResult: \n",white);
    print("\nQuaternion: \n",white);
    print(q,white);
    print("\nJq_v: \n",white);
    print(Q_v,white);
    cout << endl;   

    // Aaxis to q
    print("_____Angle-axis to Quaternion____________\n",magenta);  
    MatrixXd Q_angle,Q_axis;
    aaxis2q(angle,axis,q,Q_angle,Q_axis);
    print("Initial data: \n",white);
    print("Angle: ",white);
    print(angle,white);
    print("\nAxis: \n",white);
    print(axis,white);
    print("\nResult: \n",white);
    print("\nQuaternion: \n",white);
    print(q,white);
    print("\nJ_angle: \n",white);
    print(Q_angle,white);
    print("\nJ_axis: \n",white);
    print(Q_axis,white);
    cout << endl;       


    // w to omega
    print("_____w to Omega____________\n",magenta);  
    Vector3d w;
    w << 1,1.5,2;
    Matrix4d Omega;
    MatrixXd O_w;
    w2omega(w,Omega,O_w);
    print("Initial data: \n",white);
    print(w,white);
    print("\nResult: \n",white);
    print("\nOmega: \n",white);
    print(Omega,white);
    print("\nJ_w: \n",white);
    print(O_w,white);
    cout << endl;

    // theta to q
    print("_____Theta to q____________\n",magenta);  
    Vector3d theta;
    theta << 1,2,3;
    Quaterniond qt;
    theta2q(theta,q);
    print("Initial data: \n",white);
    print("Theta: \n",white);
    print(theta,white);
    print("\nResult: \n",white);
    print("\nq: \n",white);
    print(q,white);
    cout << endl;

    // R to q
    print("_____R to q____________\n",magenta);  
    Vector3d v2;
    Matrix3d R2;
    Quaterniond q2;
    v2 << 5,6,7;
    v2R(v2,R2);
    R2q(R2,q2);
    print("Initial data: \n",white);
    print("Rotation: \n",white);
    print(R2,white);
    print("\nResult: \n",white);
    print("\nQuaternion: \n",white);
    print(q2,white);
    cout << endl;

    // R to e
    print("_____R to Euler____________\n",magenta);  
    Vector3d euler;
    R2e(R2,euler);
    print("Initial data: \n",white);
    print("\nRotation Matrix: \n",white);
    print(R2,white);
    print("\nResult: \n",white);
    print("Euler: \n",white);
    print(euler,white);
    cout << endl;    

    // qproduct
    print("_____Quaternion product____________\n",magenta); 
    qProd(q,q,qt);
    print("Initial data: \n",white);
    print("q1: \n",white);
    print(q,white);
    print("\nq2: \n",white);
    print(q,white);    
    print("\nResult: \n",white);
    print("\nQuaternion: \n",white);
    print(qt,white);
    cout << endl;    

    // qPredict
    print("_____Quaternion Prediction____________\n",magenta); 
    double dt = 1;
    int met = 1;
    Quaterniond qpred;
    MatrixXd Q_q,Q_w;
    qPredict(q,w,qpred,dt,met,Q_q,Q_w);
    print("Initial data: \n",white);
    print("q: \n",white);
    print(q,white);
    print("\nw: \n",white);
    print(w,white);    
    print("\ndt: ",white);
    print(dt,white);
    print("\nMethod: ",white);
    print(met,white);
    print("\nResult: \n",white);
    print("\nQuaternion: \n",white);
    print(qpred,white);
    print("\nQ_q: \n",white);
    print(Q_q,white);
    print("\nQ_w: \n",white);
    print(Q_w,white);
    cout << endl;  

    // q2R
    print("_____q to R____________\n",magenta); 
    MatrixXd JR_q;
    q2R(q,R,JR_q);
    print("Initial data: \n",white);
    print("q: \n",white);
    print(q,white);
    print("\nResult: \n",white);
    print("\nRotation: \n",white);
    print(R,white);
    print("\nJR_q: \n",white);
    print(JR_q,white);
    cout << endl; 

    print("_____Quaternion Conjugate____________\n",magenta); 
    MatrixXd Q_qc;
    Quaterniond qc;
    q2qc(q,qc,Q_qc);
    print("Initial data: \n",white);
    print("q: \n",white);
    print(q,white);
    print("\nResult: \n",white);
    print("\nConjugate: \n",white);
    print(qc,white);
    print("\nQ_qc: \n",white);
    print(Q_qc,white);
    cout << endl; 

    print("_____Quaternion to Euler____________\n",magenta); 
    MatrixXd E_q;
    Vector3d e;
    q2e(q,e,E_q);
    print("Initial data: \n",white);
    print("q: \n",white);
    print(q,white);
    print("\nResult: \n",white);
    print("\nEuler: \n",white);
    print(e,white);
    print("\nE_q: \n",white);
    print(E_q,white);
    cout << endl; 

    // q to Aaxis
    print("_____Quaternion to Angle Axis____________\n",magenta); 
    MatrixXd Aangle_q,Aaxis_q;
    q2aaxis(q,angle,axis,Aangle_q,Aaxis_q);
    print("Initial data: \n",white);
    print("\nQuaternion: \n",white);
    print(q,white);
    print("\nResult: \n",white);
    print("Angle: ",white);
    print(angle,white);
    print("\nAxis: \n",white);
    print(axis,white);
    print("\nAangle_q: \n",white);
    print(Aangle_q,white);
    print("\nAaxis_q: \n",white);
    print(Aaxis_q,white);
    cout << endl;  

    // e to R
    print("_____Euler to R____________\n",magenta); 
    MatrixXd JR_e; 
    e2R(euler,R,JR_e);
    print("Initial data: \n",white);
    print("Euler: \n",white);
    print(euler,white);
    print("\nResult: \n",white);
    print("\nRotation Matrix: \n",white);
    print(R,white);
    print("\nJR_e: \n",white);
    print(JR_e,white);
    cout << endl;  

    // e to q
    print("_____Euler to Quaternion____________\n",magenta); 
    MatrixXd Q_e; 
    e2q(euler,q,Q_e);
    print("Initial data: \n",white);
    print("Euler: \n",white);
    print(euler,white);
    print("\nResult: \n",white);
    print("\nQuaternion: \n",white);
    print(q,white);
    print("\nQ_e: \n",white);
    print(Q_e,white);
    cout << endl;  

  }
}
