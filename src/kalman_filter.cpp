#include "kalman_filter.h"
#include <stdexcept>
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {
}

KalmanFilter::~KalmanFilter() {}


void KalmanFilter::Predict() {

	x_ = F_ * x_;
	P_ = F_ * P_ * F_.transpose() + Q_;

}

void KalmanFilter::Update(const VectorXd &z) {

	VectorXd y = z - H_laser_ * x_;
	MatrixXd S = H_laser_ * P_ * H_laser_.transpose() + R_laser_;
	MatrixXd K = P_ * H_laser_.transpose()  * S.inverse();

	//new estimate
	x_ = x_ + (K * y);
	P_ = (I_ - K * H_laser_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {

	float ro = sqrt(x_[0]*x_[0]+x_[1]*x_[1]);
	float phi = atan2(x_[1],x_[0]);
	h_ << ro,phi ,(x_[0]*x_[2]+x_[1]*x_[3])/ro;
	VectorXd y = z - h_;
	//cout << y[1] << " ";
	while (y[1] > PI){
		y[1] = y[1] - TWOPI;
		//cout << "> " << y[1] << " ";
	}
	while (y[1] < -PI){
		y[1] = y[1] + TWOPI;
		//cout << "< " << y[1] << " ";
	}
	//cout << endl;
	// normalize y
    try {
    	Hj_ = tools.CalculateJacobian(x_);
    } catch (std::overflow_error e) {
        std::cout << e.what();
        return;
    }
	MatrixXd S = Hj_ * P_ * Hj_.transpose() + R_radar_;
	MatrixXd K = P_ * Hj_.transpose()  * S.inverse();

	//new estimate
	x_ = x_ + (K * y);
	P_ = (I_ - K * Hj_) * P_;

}
