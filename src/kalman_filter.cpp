#include "kalman_filter.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;


#define EPSILON 0.0001
/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter()
{
	last_rhodot = EPSILON;
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
   * TODO: predict the state
   */
	x_ = F_ * x_;
	MatrixXd Ft = F_.transpose();
	P_ = F_ * P_ * Ft + Q_;
}



void KalmanFilter::Update(const VectorXd &z) {
  /**
   * TODO: update the state by using Kalman Filter equations
   */
	VectorXd y = z - H_*x_;

	MeasurementUpdate(y);

}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */
	VectorXd y = VectorXd(3);
	float px = x_[0];
	float py = x_[1];
	float vx = x_[2];
	float vy = x_[3];
	float px2py2_sqrt = sqrt((px*px)+(py*py));

	y[0] = z[0] - px2py2_sqrt;
	y[1] = z[1] - atan2(py,px);

	// check division by zero
	if (fabs(px2py2_sqrt) < EPSILON)
	{
		std::cout << "UpdateEKF () - Error - Division by Zero" <<
				"returning the last valid value" << std::endl;
		y[2] = last_rhodot;
	}
	else
	{
		y[2] = z[2] - ((px*vx)+(py*vy))/px2py2_sqrt;
		last_rhodot = y[2];
	}

	//normalize phi
	while (y[1] > M_PI)
	{
		y[1] -= 2 * M_PI;
	}
	while  (y[1] < -M_PI)
	{
		y[1] += 2 * M_PI;
	}


	MeasurementUpdate(y);
}

void KalmanFilter::MeasurementUpdate(const VectorXd &err)
{
	MatrixXd Ht = H_.transpose();
	MatrixXd S = H_ * P_ * Ht + R_;
	MatrixXd Si = S.inverse();
	MatrixXd PHt = P_ * Ht;
	MatrixXd K = PHt * Si;

	//new estimate
	x_ = x_ + (K * err);
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * H_) * P_;
}


