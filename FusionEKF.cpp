#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"
#include <math.h>
using std::endl;
using std::cout;

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

/**
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;

  /**
   * TODO: Finish initializing the FusionEKF.
   * TODO: Set the process and measurement noises
   */
  // set the acceleration noise components
  noise_ax = 9.0f;
  noise_ay = 9.0f;
  // the initial process covariance matrix Q_
  ekf_.Q_= MatrixXd(4, 4);
  // the initial transition matrix F_
  ekf_.F_= MatrixXd(4, 4);
  ekf_.F_ << 1, 0, 1, 0,
            0, 1, 0, 1,
            0, 0, 1, 0,
            0, 0, 0, 1;
  // state covariance matrix P_, values should actually came from sensor variance
  ekf_.P_ = MatrixXd(4, 4);
  ekf_.P_ << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1000, 0,
            0, 0, 0, 1000;
  // values for H_laser, laser standard matrix
   H_laser_ << 1,0,0,0,
  				0,1,0,0;
  // values for Hj_ are computed each time;

}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  /**
   * Initialization
   */
  if (!is_initialized_) {
    /**
     * TODO: Initialize the state ekf_.x_ with the first measurement.
     * TODO: Create the covariance matrix.
     * You'll need to convert radar from polar to cartesian coordinates.
     */

    // first measurement
    //cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      //cout << "EKF: Radar" << endl;
      // TODO: Convert radar from polar to cartesian coordinates 
      //         and initialize state.
      // set the state with the initial location and zero velocity
      // Asumption: theta is in radians
      // Asumption: not enouh information to initialize acceleration, because a_x and a_y are not known
    	ekf_.x_ << measurement_pack.raw_measurements_(0)*cos(measurement_pack.raw_measurements_(1)), 
              measurement_pack.raw_measurements_(0)*sin(measurement_pack.raw_measurements_(1)), 
              0, 
              0;

    	previous_timestamp_ = measurement_pack.timestamp_;
    	is_initialized_ = true;
        // print the output
  		//cout << "x_ = " << ekf_.x_ << endl;
  		//cout << "P_ = " << ekf_.P_ << endl;
    	return;

    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      //cout << "EKF: Laser" << endl;
      // TODO: Initialize state.
      // set the state with the initial location and zero velocity
    	ekf_.x_ << measurement_pack.raw_measurements_(0), 
              measurement_pack.raw_measurements_(1), 
              0, 
              0;

    	previous_timestamp_ = measurement_pack.timestamp_;
    	is_initialized_ = true;
        // print the output
 		//cout << "x_ = " << ekf_.x_ << endl;
  		//cout << "P_ = " << ekf_.P_ << endl;
    	return;
      
    }

    // done initializing, no need to predict or update
    is_initialized_ = true;
    ekf_.x_ << 1, 1, 1, 1;
    return;
  }
  
  //cout << "EKF: 2nd" << endl;
  /**
   * Prediction
   */

  /**
   * TODO: Update the state transition matrix F according to the new elapsed time.
   * Time is measured in seconds.
   * TODO: Update the process noise covariance matrix.
   * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */
  // dt - expressed in seconds
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;
  // Modify the F matrix so that the time is integrated
  ekf_.F_(0,2) = dt;
  ekf_.F_(1,3) = dt;
  // Set the process covariance matrix Q
  ekf_.Q_ << (pow(dt,4)/4)*noise_ax, 0 , (pow(dt,3)/2)*noise_ax, 0 ,
            0, (pow(dt,4)/4)*noise_ay, 0, (pow(dt,3)/2)*noise_ay,
            (pow(dt,3)/2)*noise_ax, 0 , pow(dt,2)*noise_ax, 0,
            0, (pow(dt,3)/2)*noise_ay, 0, (pow(dt,2))*noise_ay;
  
  
  ekf_.Predict();

  /**
   * Update
   */

  /**
   * TODO:
   * - Use the sensor type to perform the update step.
   * - Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // TODO: Radar updates
    // z = measurement_pack
    ekf_.R_=R_radar_; //pointers
    Hj_ = tools.CalculateJacobian(ekf_.x_);
    ekf_.H_=Hj_;
    //ekf_.H=H_;
    //Update the state and covariance matrices.
	ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  } else {
    // TODO: Laser updates
    ekf_.R_=R_laser_;
    ekf_.H_=H_laser_;
    //Update the state and covariance matrices.
	ekf_.Update(measurement_pack.raw_measurements_); 
  }

  // print the output
  //cout << "x_ = " << ekf_.x_ << endl;
  //cout << "P_ = " << ekf_.P_ << endl;
}
