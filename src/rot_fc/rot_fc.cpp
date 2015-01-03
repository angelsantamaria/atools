#include "rot_fc.h"

using namespace Eigen;
using namespace std;

namespace atools{

double EPS = pow(1,-16);

void v2skew(const Vector3d& v, Matrix3d& M_sk)
{ M_sk << 0,-v(2,0),v(1,0),v(2,0),0,-v(0,0),-v(1,0),v(0,0),0; }

void v2skew(const Vector3d& v, Matrix3d& M_sk, MatrixXd& V_sk)
{
	v2skew(v,M_sk);

	V_sk = MatrixXd::Zero(9,3);
	V_sk.row(0) << 0.0,0.0,0.0;
	V_sk.row(1) << 0.0,0.0,1.0;
	V_sk.row(2) << 0.0,-1.0,0.0;	  
	V_sk.row(3) << 0.0,0.0,-1.0;
	V_sk.row(4) << 0.0,0.0,0.0;
	V_sk.row(5) << 1.0,0.0,0.0;
	V_sk.row(6) << 0.0,1.0,0.0;
	V_sk.row(7) << -1.0,0.0,0.0;
	V_sk.row(8) << 0.0,0.0,0.0;
}

void v2aaxis(const Vector3d& v, double& angle, Vector3d& axis)
{
	angle = sqrt(v.dot(v));
	// if (angle>EPS)
		axis = v/angle;
	// else
	// {
	// 	axis = Vector3d::Zero();
	// 	cout << "here"<<endl;
	// }
}
void v2aaxis(const Vector3d& v, double& angle, Vector3d& axis, MatrixXd& Aangle_v)
{
	v2aaxis(v,angle,axis);
	Aangle_v = MatrixXd::Zero(1,3);
	if (angle>EPS)
		Aangle_v = axis.transpose();
}
void v2aaxis(const Vector3d& v, double& angle, Vector3d& axis, MatrixXd& Aangle_v, MatrixXd& Aaxis_v)
{
	v2aaxis(v,angle,axis,Aangle_v);
	Aaxis_v = MatrixXd::Zero(3,3);
	if (angle>EPS)
	{
		Aaxis_v.row(0) << (1.0/angle)-(axis(0,0)*axis(0,0))/angle, -axis(0,0)/angle*axis(1,0), -axis(0,0)/angle*axis(2,0);
		Aaxis_v.row(1) << -axis(0,0)/angle*axis(1,0), (1.0/angle)-(axis(1,0)*axis(1,0))/angle, -axis(1,0)/angle*axis(2,0);
		Aaxis_v.row(2) << -axis(0,0)/angle*axis(2,0), -axis(1,0)/angle*axis(2,0), (1.0/angle)-(axis(2,0)*axis(2,0))/angle;
	}	
}

void v2R(const Vector3d& v, Matrix3d& R)
{
	double angle;
	Vector3d axis;
	v2aaxis(v,angle,axis); 

	double ca  = cos(angle);
	Vector3d sau = sin(angle)*axis;
	Matrix3d M_sk(3,3);
	v2skew(sau,M_sk);

	R = ca*Matrix3d::Identity() + M_sk + ((1.0-ca)*axis)*axis.transpose();
}

void v2q(const Vector3d& v, Quaterniond& q)
{
	double angle;
	Vector3d axis;
	v2aaxis(v,angle,axis);
    aaxis2q(angle,axis,q);

}
void v2q(const Vector3d& v, Quaterniond& q, MatrixXd& Q_v)
{
	double a = sqrt(v.dot(v));

	if (a<EPS)
	{
		q.w() = 1.0-pow(v.norm(),2.0/8.0);
		q.vec() = v/2.0;
		Q_v.row(0) << -1.0/4.0*v.transpose();
		Q_v.block(1,0,3,3) = 0.5*Matrix3d::Identity();
	}
	else
	{
		double angle;
		Vector3d axis;
		MatrixXd Aangle_v,Aaxis_v,Q_angle,Q_axis;
		v2aaxis(v,angle,axis,Aangle_v,Aaxis_v);
	  	aaxis2q(angle,axis,q,Q_angle,Q_axis);
	    Q_v = Q_angle*Aangle_v + Q_axis*Aaxis_v;		
	}
}

void aaxis2q(const double& angle, const Vector3d& axis, Quaterniond& q)
{ q = AngleAxisd(angle, axis); }

void aaxis2q(const double& angle, const Vector3d& axis, Quaterniond& q, MatrixXd& Q_angle)
{
	aaxis2q(angle,axis,q);
	Q_angle = MatrixXd::Zero(4,1);
	Q_angle << -sin(angle/2.0)/2.0, (cos(angle/2.0)/2.0)*axis;
}
void aaxis2q(const double& angle, const Vector3d& axis, Quaterniond& q, MatrixXd& Q_angle, MatrixXd& Q_axis)
{
	aaxis2q(angle,axis,q,Q_angle);
	Q_axis = MatrixXd::Zero(4,3);
	Q_axis << 0.0,0.0,0.0,sin(angle/2.0)*Matrix3d::Identity();
}

void w2omega(const Vector3d& w, Matrix4d& Omega)
{
	Omega.row(0) << 0.0,-w.transpose();
	Omega.col(0) << 0.0,w;
	Matrix3d M_sk(3,3);
	v2skew(-w,M_sk);
	Omega.block(1,1,3,3) = M_sk;
}
void w2omega(const Vector3d& w, Matrix4d& Omega, MatrixXd& O_w)
{
	w2omega(w,Omega);
	O_w = MatrixXd::Zero(16,3);
	O_w.row(0) << 0.0,0.0,0.0;
	O_w.row(1) << 1.0,0.0,0.0;
	O_w.row(2) << 0.0,1.0,0.0;
	O_w.row(3) << 0.0,0.0,1.0;
	O_w.row(4) << -1.0,0.0,0.0;
	O_w.row(5) << 0.0,0.0,0.0;
	O_w.row(6) << 0.0,0.0,-1.0;
	O_w.row(7) << 0.0,1.0,0.0;
	O_w.row(8) << 0.0,-1.0,0.0;
	O_w.row(9) << 0.0,0.0,1.0;
	O_w.row(10) << 0.0,0.0,0.0;
	O_w.row(11) << -1.0,0.0,0.0;
	O_w.row(12) << 0.0,0.0,-1.0;
	O_w.row(13) << 0.0,-1.0,0.0;
	O_w.row(14) << 1.0,0.0,0.0;
	O_w.row(15) << 0.0,0.0,0.0;
}

void theta2q(const Vector3d& theta, Quaterniond& q)
{
	VectorXd qv = VectorXd::Zero(4,1);
	qv(0,0)=1.0;
	qv.block(1,0,3,1) = theta/2.0;
	q.w()=qv(0,0);
	q.x()=qv(1,0);
	q.y()=qv(2,0);
	q.z()=qv(3,0);
}

void R2q(const Matrix3d& R, Quaterniond& q)
{ q = Quaterniond(R); }

void R2e(const Matrix3d& R, Vector3d& e)
{
	// e = R.eulerAngles(2,1,0);
	Vector3d et = R.eulerAngles(2,1,0);
	e(0,0)=et(2,0);
	e(1,0)=et(1,0);
	e(2,0)=et(0,0);
}

void qProd(const Quaterniond& q1,const Quaterniond& q2, Quaterniond& q)
{	q = q1*q2; }

void qPredict(const Quaterniond& q, const Vector3d& w, Quaterniond& qpred)
{ qPredict(q,w,qpred,1,1); }
void qPredict(const Quaterniond& q, const Vector3d& w, Quaterniond& qpred, const double& dt)
{ qPredict(q,w,qpred,dt,1); }
void qPredict(const Quaterniond& q, const Vector3d& w, Quaterniond& qpred, const double& dt, const int& met)
{
	Quaterniond qn;
	Matrix4d Omega;
	Vector4d qv;

	switch (met)
	{
		case 0: //Euler method
			w2omega(w,Omega);
			qv << q.w(), q.vec();
			qn = qv + 0.5*dt*(Omega*qv);
			break;
		case 1:	//Exact method
			Quaterniond q2;
			v2q(w*dt,q2); 
			qProd(q,q2,qn); // True value - Jacobians based on Euler form
			break;
	}



	qpred = qn.normalized(); // Euler integration - fits with Jacobians
}
void qPredict(const Quaterniond& q, const Vector3d& w, Quaterniond& qpred, const double& dt, const int& met, MatrixXd& Q_q)
{
	qPredict(q,w,qpred,dt,met); //run with method 'exact'

	Matrix4d Omega;
	w2omega(w,Omega);

	Q_q = Matrix4d::Zero();
	Q_q = Matrix4d::Identity()+0.5*dt*Omega;
}
void qPredict(const Quaterniond& q, const Vector3d& w, Quaterniond& qpred, const double& dt, const int& met, MatrixXd& Q_q, MatrixXd& Q_w)
{
	qPredict(q,w,qpred,dt,met,Q_q); //run with method 'exact'
	MatrixXd Pi;
	q2Pi(q,Pi);
	Q_w = MatrixXd::Zero(4,3);
	Q_w = 0.5*dt*Pi;
}

void q2Pi(const Quaterniond& q,MatrixXd& Pi)
{
	Pi = MatrixXd::Zero(4,3);
	Pi.row(0) << -q.x(), -q.y(), -q.z();
    Pi.row(1) <<  q.w(), -q.z(),  q.y();
	Pi.row(2) <<  q.z(),  q.w(), -q.x();
    Pi.row(3) << -q.y(),  q.x(),  q.w();
}

void q2R(const Quaterniond& q, Matrix3d& R)
{ R = q.matrix(); }

void q2R(const Quaterniond& q, Matrix3d& R, MatrixXd& JR_q)
{
	R = q.matrix();
	double a,b,c,d;
	a=2.0*q.w();
	b=2.0*q.x();
	c=2.0*q.y();
	d=2.0*q.z();

	JR_q = MatrixXd::Zero(9,4);
	JR_q.row(0) <<  a, b,-c,-d;
	JR_q.row(1) <<  d, c, b, a;
	JR_q.row(2) << -c, d,-a, b;
	JR_q.row(3) << -d, c, b,-a;
	JR_q.row(4) <<  a,-b, c,-d;
	JR_q.row(5) <<  b, a, d, c;
	JR_q.row(6) <<  c, d, a, b;
	JR_q.row(7) << -b,-a, d, c;
	JR_q.row(8) <<  a,-b,-c, d;
}

void q2qc(const Quaterniond& q, Quaterniond& qc)
{ qc = q.conjugate(); }

void q2qc(const Quaterniond& q, Quaterniond& qc, MatrixXd& Q_qc)
{
	qc = q.conjugate();
	Q_qc = MatrixXd::Zero(4,4);
	Q_qc.diagonal() << 1.0,-1.0,-1.0,-1.0;
}

void q2e(const Quaterniond& q, Vector3d& e)
{
	double a,b,c,d;
	a=q.w();
	b=q.x();
	c=q.y();
	d=q.z();

	double y1,x1,z2,y3,x3;
	y1 =  2.0*c*d + 2.0*a*b;
	x1 =  a*a - b*b - c*c + d*d;
	z2 = -2.0*b*d + 2.0*a*c;
	y3 =  2.0*b*c + 2.0*a*d;
	x3 =  a*a + b*b - c*c - d*d;

    e << atan2(y1,x1),asin(z2),atan2(y3,x3);
}

void q2e(const Quaterniond& q, Vector3d& e, MatrixXd& E_q)
{
	double a,b,c,d;
	a=q.w();
	b=q.x();
	c=q.y();
	d=q.z();

	double y1,x1,z2,y3,x3;
	y1 =  2.0*c*d + 2.0*a*b;
	x1 =  a*a - b*b - c*c + d*d;
	z2 = -2.0*b*d + 2.0*a*c;
	y3 =  2.0*b*c + 2.0*a*d;
	x3 =  a*a + b*b - c*c - d*d;

    e << atan2(y1,x1),asin(z2),atan2(y3,x3);	

    Vector4d dx1dq,dy1dq,dz2dq,dx3dq,dy3dq,de1dq,de2dq,de3dq;
    dx1dq << 2.0*a,-2.0*b,-2.0*c, 2.0*d;
    dy1dq << 2.0*b, 2.0*a, 2.0*d, 2.0*c;
    dz2dq << 2.0*c,-2.0*d, 2.0*a,-2.0*b;
    dx3dq << 2.0*a, 2.0*b,-2.0*c,-2.0*d;
    dy3dq << 2.0*d, 2.0*c, 2.0*b, 2.0*a;

    double de1dx1,de1dy1,de2dz2,de3dx3,de3dy3;
    de1dx1 = -y1/(x1*x1 + y1*y1);
    de1dy1 =  x1/(x1*x1 + y1*y1);
    de2dz2 = 1.0/sqrt(1.0-z2*z2);
    de3dx3 = -y3/(x3*x3 + y3*y3);
    de3dy3 =  x3/(x3*x3 + y3*y3);

    E_q = MatrixXd::Zero(3,4);

    E_q.row(0) = de1dx1*dx1dq + de1dy1*dy1dq;
    E_q.row(1) = de2dz2*dz2dq;
    E_q.row(2) = de3dx3*dx3dq + de3dy3*dy3dq;
}

void q2aaxis(const Quaterniond& q, double& angle, Vector3d& axis)
{
	axis << q.x(),q.y(),q.z();
	axis = axis/axis.norm();
	angle = 2.0*acos(q.w());
}
void q2aaxis(const Quaterniond& q, double& angle, Vector3d& axis, MatrixXd& Aangle_q)
{
	q2aaxis(q,angle,axis);
	Aangle_q = MatrixXd::Zero(1,4);
	Aangle_q << -2.0/sqrt(1.0-q.w()*q.w()),0.0,0.0,0.0;	
}
void q2aaxis(const Quaterniond& q, double& angle, Vector3d& axis, MatrixXd& Aangle_q, MatrixXd& Aaxis_q)
{
	q2aaxis(q,angle,axis,Aangle_q);

	Aaxis_q = MatrixXd::Zero(3,4);
	double den = pow(pow(abs(q.x()),2) + pow(abs(q.y()),2) + pow(abs(q.z()),2),1.5);
	double s = pow(abs(q.x()),2) + pow(abs(q.y()),2) + pow(abs(q.z()),2);

	Aaxis_q(0,0) = 0.0;
	Aaxis_q(0,1) = (s - q.x()*abs(q.x())*sign(q.x()))/den;
	Aaxis_q(0,2) = -(q.x()*abs(q.y())*sign(q.y()))/den;
	Aaxis_q(0,3) = -(q.x()*abs(q.z())*sign(q.z()))/den;
    Aaxis_q(1,0) = 0.0;
    Aaxis_q(1,1) = -(q.x()*abs(q.x())*sign(q.x()))/den;
    Aaxis_q(1,2) = (s - q.y()*abs(q.y())*sign(q.y()))/den;
    Aaxis_q(1,3) = -(q.y()*abs(q.z())*sign(q.z()))/den;
    Aaxis_q(2,0) = 0.0;
    Aaxis_q(2,1) = -(q.z()*abs(q.x())*sign(q.x()))/den;
    Aaxis_q(2,2) = -(q.z()*abs(q.y())*sign(q.y()))/den;
    Aaxis_q(2,3) = (s - q.z()*abs(q.z())*sign(q.z()))/den;
}

void e2R(const Vector3d& e, Matrix3d& R)
{
	R = AngleAxisd(e(2), Vector3d::UnitZ())*
		AngleAxisd(e(1), Vector3d::UnitY())*
		AngleAxisd(e(0), Vector3d::UnitX());
}
void e2R(const Vector3d& e, Matrix3d& R, MatrixXd& JR_e)
{
	e2R(e,R);

	double sr,cr,sp,cp,sy,cy;
	sr = sin(e(0));
	cr = cos(e(0));
	sp = sin(e(1));
	cp = cos(e(1));
	sy = sin(e(2));
	cy = cos(e(2));

	JR_e = MatrixXd::Zero(9,3);
    JR_e.row(0) << 0.0,-sp*cy,-cp*sy;
    JR_e.row(1) << 0.0,-sp*sy,cp*cy;
    JR_e.row(2) << 0.0,-cp,0.0;
    JR_e.row(3) << sr*sy+cr*sp*cy,sr*cp*cy,-cr*cy-sr*sp*sy;
    JR_e.row(4) << -sr*cy+cr*sp*sy,sr*cp*sy,-cr*sy+sr*sp*cy;
    JR_e.row(5) << cr*cp,-sr*sp,0.0;
    JR_e.row(6) << cr*sy-sr*sp*cy,cr*cp*cy,sr*cy-cr*sp*sy;
    JR_e.row(7) << -cr*cy-sr*sp*sy,cr*cp*sy,sr*sy+cr*sp*cy;
    JR_e.row(8) << -sr*cp,-cr*sp,0.0;
}

void e2q(const Vector3d& e, Quaterniond& q)
{
	q = Quaterniond(AngleAxisd(e(2), Vector3d::UnitZ())*
				 AngleAxisd(e(1), Vector3d::UnitY())*
				 AngleAxisd(e(0), Vector3d::UnitX()));

}
void e2q(const Vector3d& e, Quaterniond& q, MatrixXd& Q_e)
{
	e2q(e,q);

	double sr,sp,sy,cr,cp,cy;
    sr = sin(e(0)/2.0);
    sp = sin(e(1)/2.0);
    sy = sin(e(2)/2.0);
    cr = cos(e(0)/2.0);
    cp = cos(e(1)/2.0);
    cy = cos(e(2)/2.0);

	Q_e = MatrixXd::Zero(4,3);
	Q_e.row(0) << -cy*cp*sr+sy*sp*cr, -cy*sp*cr+sy*cp*sr, -sy*cp*cr+cy*sp*sr;
	Q_e.row(1) << cy*cp*cr+sy*sp*sr, -cy*sp*sr-sy*cp*cr, -sy*cp*sr-cy*sp*cr;
	Q_e.row(2) << -cy*sp*sr+sy*cp*cr,  cy*cp*cr-sy*sp*sr, -sy*sp*cr+cy*cp*sr;
	Q_e.row(3) << -sy*cp*sr-cy*sp*cr, -cy*cp*sr-sy*sp*cr,  cy*cp*cr+sy*sp*sr;

	Q_e = 0.5*Q_e;
}



} // End of namespace atools

