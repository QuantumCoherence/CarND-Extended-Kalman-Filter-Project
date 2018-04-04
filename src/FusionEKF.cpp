#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;
  // initializing matrices
  ekf_.R_laser_ = MatrixXd(2, 2);
  ekf_.R_radar_ = MatrixXd(3, 3);
  ekf_.H_laser_ = MatrixXd(2, 4);

  //measurement covariance matrix - laser
  ekf_.R_laser_ << 0.0225, 0,
                   0, 0.0225;

  //measurement covariance matrix - radar
  ekf_.R_radar_ << 0.09, 0, 0,
                   0, 0.0009, 0,
                   0, 0, 0.09;
  // state vector
  ekf_.x_ = VectorXd(4);

  // state covariance matrix
  ekf_.P_ = MatrixXd(4, 4);
  ekf_.P_ << 50, 0, 0, 0,
	         0, 50, 0, 0,
		     0, 0, 500, 0,
		     0, 0, 0, 500;
  // state transition matrix
  ekf_.F_ = MatrixXd(4, 4);
  ekf_.F_ << 1, 0, 1, 0,
			 0, 1, 0, 1,
			 0, 0, 1, 0,
			 0, 0, 0, 1;
  // process covariance matrix
  ekf_.Q_ = MatrixXd(4, 4);

  //measurement matrix
  ekf_.H_laser_ = MatrixXd(2, 4);
  ekf_.H_laser_ << 1, 0, 0, 0,
			       0, 1, 0, 0;
  //measurement matrix Jacobian
  ekf_.Hj_ = MatrixXd(3, 4);
  ekf_.Hj_ << 0, 0, 0, 0,
			  0, 0, 0, 0,
			  0, 0, 0, 0;
  // Identiy Matrix
  ekf_.I_ = MatrixXd::Identity(4, 4);
  // Nonlinar Measurement  Function for radar input
  ekf_.h_ = VectorXd(3);

  // acceleration noise
  noise_ax = 14;
  noise_ay = 14;

  ekf_.x_ = VectorXd(4);

}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {

  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {

	  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
		  ekf_.x_ << measurement_pack.raw_measurements_[0]*cos(measurement_pack.raw_measurements_[1]), measurement_pack.raw_measurements_[0]*sin(measurement_pack.raw_measurements_[1]), 5.2, 0.1;
	  } else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
		  ekf_.x_ << 3.122427e-01,	5.803398e-01,	5	,0;
    	  //ekf_.x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 5	,0;
      }

      // done initializing, no need to predict or update
	  previous_timestamp_ = measurement_pack.timestamp_;
      is_initialized_ = true;
      return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/
    //compute the time elapsed between the current and previous measurements
   	float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;	//dt - expressed in seconds
   	previous_timestamp_ = measurement_pack.timestamp_;
   	// insert elpased into trasnfer matrix
	ekf_.F_(0,2) = dt;
	ekf_.F_(1,3) = dt;
	// process covariance matrix Q update
	float dt2 = dt*dt/2;
	float dt3 = dt2*dt;
	float dt4 = dt3*dt/2;
	ekf_.Q_ << dt4*noise_ax, 0,    dt3*noise_ax,   0,
			      0,   dt4*noise_ay,    0,   dt3*noise_ay,
			  dt3*noise_ax, 0,    dt2*noise_ax,    0,
			      0,   dt3*noise_ay,    0,   dt2*noise_ay;

  ekf_.Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/


  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
	  ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  } else {
    // Laser updates
	  ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  //cout << "x_ = " << ekf_.x_[0] << " " << ekf_.x_[1] << " " << ekf_.x_[2] << " " << ekf_.x_[3] << endl;
  //cout << "P_ = " << ekf_.P_ << endl;

}
