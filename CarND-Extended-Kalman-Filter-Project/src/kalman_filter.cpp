#include "kalman_filter.h"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

const double PI  =3.141592653589793238463;

KalmanFilter::KalmanFilter() {
  F_ = MatrixXd(4, 4);
	F_ << 1, 0, 1, 0,
			  0, 1, 0, 1,
			  0, 0, 1, 0,
			  0, 0, 0, 1;
  P_ = MatrixXd(4, 4);
	P_ << 1, 0, 0, 0,
			  0, 1, 0, 0,
			  0, 0, 1000, 0,
			  0, 0, 0, 1000;
}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /**
  TODO:
    * predict the state
  */
 x_ = F_ * x_;
 MatrixXd Ft = F_.transpose();
 P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;
	Update(z, y);
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  
  float px = x_(0);
	float py = x_(1);
	float vx = x_(2);
	float vy = x_(3);

  float dist_sqrt = sqrt(px*px + py*py);
  if(fabs(dist_sqrt) < 0.0001){
		cout << "UpdateEKF () - Error - Division by Zero" << endl;
	} else {
    // see radar-measurement.pdf for transform from x_ to radar measurement z.
    VectorXd z_pred = VectorXd(3);
    float arc = atan2(py, px);
    float rate = (px*vx + py*vy) / dist_sqrt;
    z_pred(0) = dist_sqrt;
    z_pred(1) = arc;
    z_pred(2) = rate;
    VectorXd innovation = z - z_pred;
    while (innovation(1) > PI) innovation(1) -= 2*PI;
    while (innovation(1) < -PI) innovation(1) += 2*PI;
	  Update(z, innovation);
  }
}

void KalmanFilter::Update(const Eigen::VectorXd &z, const Eigen::VectorXd &innovation){
  
	MatrixXd Ht = H_.transpose();
	MatrixXd S = H_ * P_ * Ht + R_;
	MatrixXd Si = S.inverse();
	MatrixXd PHt = P_ * Ht;
	MatrixXd K = PHt * Si;

	//new estimate
	x_ = x_ + (K * innovation);
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * H_) * P_;
}
