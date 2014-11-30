#ifndef _ROT_FC_H
#define _ROT_FC_H

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

//atools stuff
#include <math_fc.h>

using namespace Eigen;
using namespace std;

namespace atools{

    /**
    * \brief Vector to skew symmetric matrix
    *
    * Vector to skew symmetric matrix
    *
    * Input:
    *	vec: 3x1 vector
    *
    * Output:
    *	M_sk: Skew symmetric matrix (1rst row: 0 -z -y)
    * 	J_sk(optional): Jacobian r.t. skew symmetric matrix.
    */
    void v2skew(const Vector3d& vec, Matrix3d& M_sk);
    void v2skew(const Vector3d& vec, Matrix3d& M_sk, MatrixXd& V_sk);

    /**
    * \brief Vector to Angle-axis representation
    *
    * Vector to Angle-axis representation.
    *
    * Input:
    *	vec: 3x1 vector	.
    * Output:
    *	angle: Rotated angle (rad).
    * 	axis: Axis unity vector.
    * 	Jangle_v (optional): Jacobian of "angle" wrt the vector.
    *	Jaxis_v (optional):  Jacobian of the "axis" wrt the vector.
    */   
    void v2aaxis(const Vector3d& vec, double& angle, Vector3d& axis);
    void v2aaxis(const Vector3d& vec, double& angle, Vector3d& axis, MatrixXd& Aangle_v);
    void v2aaxis(const Vector3d& vec, double& angle, Vector3d& axis, MatrixXd& Aangle_v, MatrixXd& Aaxis_v);


    /**
    * \brief Rotation vector to rotation matrix
    *
    * Computes the rotation matrix corresponding to the rotation vector vec.
	* Uses rodrigues formula.
    *
    * Input:
    *	vec:  3x1 Rotation vector.
    * Output:
    *	R:    Rotation matrix.
    */
	void v2R(const Vector3d& vec, Matrix3d& R);

    /**
    * \brief Rotation vector to quaternion
    *
    * Rotation vector to quaternion 
    *
    * Input:
    *   v:    Rotation vector.
    * Output:
    *   q:    Quaternion with convenion: q = [qw qx qy qz] with qw the scalar element.
    *   J_v:  Jacobian r.t. rotation vector.
    * 
    */
    void v2q(const Vector3d& vec, Quaterniond& q);
    void v2q(const Vector3d& vec, Quaterniond& q, MatrixXd& Q_v);

    /**
    * \brief Angle-axis to quaternion
    *
    * Rotated angle and rotation axis vector to quaternion.
    *
    * Input:
    *	angle:    Rotation angle (rad).
    *	axis:     Rotation axis (unity vector).
    *
    * Output:
    *	q:        Quaternion with convention: q = [qw qx qy qz] with qw the scalar element.
    *	J_angle (optional):   Jacobian wrt "angle".
    *	J_axis (optional):    Jacobian wrt "axis".
    * 
    */
	void aaxis2q(const double& angle, const Vector3d& axis, Quaterniond& q);
	void aaxis2q(const double& angle, const Vector3d& axis, Quaterniond& q, MatrixXd& Q_angle);
	void aaxis2q(const double& angle, const Vector3d& axis, Quaterniond& q, MatrixXd& Q_angle, MatrixXd& Q_axis);

    /**
    * \brief Angular velocity to Omega matrix
    *
    * Angular velocity to Omega matrix. 
    * It is used to multiply matrix-form a quaternion with the relative rotation due to angular velocity.   
    *
    * Inputs:
    *   w:  Angular velocity.
    *   
    * Outputs:
    *   Omega:  Omega matrix.
    *   J_w (optional): Jacobian r.t. w.
    */
    void w2omega(const Vector3d& w, Matrix4d& Omega);
    void w2omega(const Vector3d& w, Matrix4d& Omega, MatrixXd& O_w);

    /**
    * \brief Theta vector (minimal representation) to quaternion
    *
    * Returns the quaternion representing the rotation rates in minimal 
    * representation theta.
    *
    * Input:
    *   theta:  3x1 Rotation rates vector.
    * Output:
    *   q:  Quaternion with convenion: q = [qw qx qy qz] with qw the scalar element.
    */
    void theta2q(const Vector3d& theta, Quaterniond& q);

    /**
    * \brief Rotation matrix to quaternion conversion.
    *
    * Gives the quaternion corresponding to the body orientation 
    * given by the rotation matrix body-to-world.
    *
    * Input:
    *   q:  Quaternion with convenion: q = [qw qx qy qz] with qw the scalar element.
    * Output:
    *   R:  Rotation matrix body-to-world.
    * 
    */
    void R2q(const Matrix3d& R, Quaterniond& q);

    /**
    * \brief Rotation to Euler angles.
    *
    * Gives the Euler angles corresponding to the rotation matrix body-to-world.
    * Convention ZYX.
    *
    * Input:
    *   R:    Rotation matrix body-to-world.
    * Output:
    *   e:    Euler angles: e = [roll pitch yaw]'.
    */
    void R2e(const Matrix3d& R, Vector3d& e);

    /**
    * \brief Euler to Rotation Matrix
    *
    * Converts a vector containing euler angles to a rotation Matrix.
    * Convention ZYX.
    *
    * Input:
    *   e:    Euler angles: e = [roll pitch yaw]'.
    * Output:
    *   R:  Rotation matrix.
    * 
    */
    void e2R(const Vector3d& e, Matrix3d& R);

    /**
    * \brief Quaternion product
    *
    * Computes Quaternion product q = q1*q2;
    *
    * Input:
    *   q1: Quaternion with convenion: q = [qw qx qy qz] with qw the scalar element.
    *   q2: Quaternion with convenion: q = [qw qx qy qz] with qw the scalar element.
    * Output:
    *   q:  Quaternion with convenion: q = [qw qx qy qz] with qw the scalar element. 
    */
    void qProd(const Quaterniond& q1,const Quaterniond& q2, Quaterniond& q);


    /**
    * \brief Time update function for quaternions.
    *
    * Returns the quaternion after a rotation in body frame expressed by the three angles in DV (roll, pitch, yaw).
    *   Qu = QPREDICT(Q,DV) is the updated quaternion Q after a rotation 
    *   in body frame expressed by the three angles in DV (roll, pitch, yaw).
    *
    *   Qu = QPREDICT(Q,W,DT) assumes a rotation speed W and a 
    *   sampling time DT. It is equivalent to the previous case with DV = W*DT.
    *
    *   [Qu,QU_q,QU_w] = QPREDICT(Q,W,DT) returns Jacobians wrt Q and W.
    *
    *   [...] = QPREDICT(...,MET) allows the specification of the method to
    *   update the quaternion:
    *       0: 'euler' uses Qu = Q + .5*DT*W2OMEGA(W)*Q
    *       1: 'exact'  uses Qu = QPROD(Q,V2Q(W*DT))
    *   The Jacobians are always computed according to the 'Euler' method.
    *   'Euler' is the default method.
    *
    * Input:
    *   q:  Quaternion.
    *   w:  Rotation in body frame or Rotation speed expressed by the three angles (roll, pitch, yaw) or three angle rates.  
    *   dt:   Time step.
    *   met:  Update method specification: 'eu' Euler, 'ex': Exact.
    * Output:
    *   qpred:   Quaternion after a rotation in body frame.
    *   Q_q:  Jacobian wrt quaternion.
    *   Q_w:  Jacobian wrt rotation angles or angular rates.
    */   
    void qPredict(const Quaterniond& q, const Vector3d& w, Quaterniond& qpred);
    void qPredict(const Quaterniond& q, const Vector3d& w, Quaterniond& qpred, const double& dt);
    void qPredict(const Quaterniond& q, const Vector3d& w, Quaterniond& qpred, const double& dt, const int& met);
    void qPredict(const Quaterniond& q, const Vector3d& w, Quaterniond& qpred, const double& dt, const int& met, MatrixXd& Q_q);
    void qPredict(const Quaterniond& q, const Vector3d& w, Quaterniond& qpred, const double& dt, const int& met, MatrixXd& Q_q, MatrixXd& Q_w);

    /**
    * \brief Pi matrix construction from quaternion.
    *
    * Given:  Q     = [a b c d]'  the attitude quaternion
    *         W     = [p q r]'    the angular rates vector
    *         OMEGA = W2OMEGA(W)  a skew symetric matrix 
    *
    *   The output matrix:
    *
    *                |-b -c -d |
    *           PI = | a -d  c |  
    *                | d  a -b |
    *                |-c  b  a |  
    * 
    *   is the Jacobian of OMEGA*Q with respect to W
    */
    void q2Pi(const Quaterniond& q,MatrixXd& Pi);

    /**
    * \brief Quaternion to rotation matrix
    *
    *  Returns the rotation matrix from a quaternion (in the specified 
    *  convenion) and its rotation Jacobian wrt q. if requested.
    *
    * Input:
    *   q:    Quaternion with convenion: q = [qw qx qy qz] with qw th scalar element.
    * Output:
    *   R:    Rotation matrix.
    *   JR_q (optional): Rotation matrix Jacobian wrt q. 
    */
    void q2R(const Quaterniond& q, Matrix3d& R);
    void q2R(const Quaterniond& q, Matrix3d& R, MatrixXd& JR_q);

    /**
    * \brief Quaternion conjugate
    *
    * Returns the quaternion conjugate (in the specified convenion) and 
    * its Jacobian wrt q if requested.
    *
    * Input:
    *   q:    Quaternion with convenion: q = [qw qx qy qz] with qw the scalar element.
    * Output:
    *   q_c:  Quaternion conjugate maintaining the input convenion.
    *   Q_qc (optional): Jacobian of quaternion conjugate.
    */
    void q2qc(const Quaterniond& q, Quaterniond& qc);
    void q2qc(const Quaterniond& q, Quaterniond& qc, MatrixXd& Q_qc);

    /**
    * \brief Quaternion to Euler angles conversion
    *
    * Returns the Euler angles representing the specified quaternion and 
    * its Jacobian wrt q if requested.
    *
    * Input:
    *   q:    Quaternion with convenion: q = [qw qx qy qz] with qw the scalar element.
    * Output:
    *   e:    Euler angles: e = [roll pitch yaw]'.
    *   E_q (optional):  Jacobian wrt quaternion.
    */
    void q2e(const Quaterniond& q, Vector3d& e);
    void q2e(const Quaterniond& q, Vector3d& e, MatrixXd& E_q);

    /**
    * \brief Quaternion to rotated angle and rotation axis vector.
    *
    * Returns the rotation of "angle" rad around the axis defined by 
    * the unity vector "axis", that is equivalent to that defined by 
    * the quaternion q.
    *
    * Input:
    *   q:    Quaternion with convenion: q = [qw qx qy qz] with qw the scalar element.
    * Output:
    *   angle:    Rotation angle (rad).
    *   axis:     Rotation axis (unity vector).
    *   Aangle_q (optional): Angle Jacobian wrt q.
    *   Aaxis_q (optional):  Axis Jacobian wrt q.
    */
    void q2aaxis(const Quaterniond& q, double& angle, Vector3d& axis);
    void q2aaxis(const Quaterniond& q, double& angle, Vector3d& axis, MatrixXd& Aangle_q, MatrixXd& Aaxis_q);


    /**
    * \brief Euler angles to rotation matrix.
    *
    * Gives the rotation matrix body-to-world corresponding to the
    * body orientation given by the Euler angles vector E = [roll; pitch;
    * yaw]; and its Jacobian wrt e if requested.
    *
    * Input:
    *   e:    Euler angles: e = [roll pitch yaw]'.
    * Output:
    *   R:    Rotation matrix body-to-world.
    *   JR_e (optional):  Jacobian wrt Euler angles.
    */
    void e2R(const Vector3d& e, Matrix3d& R);
    void e2R(const Vector3d& e, Matrix3d& R, MatrixXd& JR_e);

    /**
    * \brief  Euler angles to quaternion conversion
    *
    * Returns the quaternion representing the specified Euler angles and 
    * its Jacobian wrt the Euler angles if requested.
    *
    * Input:
    *   e:    Euler angles: e = [roll pitch yaw]'
    * Output:
    *   q:    Quaternion with convenion: q = [qw qx qy qz] with qw the scalar element.
    *   Q_e:  Jacobian wrt Euler angles.
    */
    void e2q(const Vector3d& e, Quaterniond& q);
    void e2q(const Vector3d& e, Quaterniond& q, MatrixXd& Q_e);

} // End of atools namespace

#endif