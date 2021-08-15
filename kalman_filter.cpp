#include "kalman_filter.h"
#include <math.h>
#include<iostream>

using std::endl;
using std::cout;
using Eigen::MatrixXd;
using Eigen::VectorXd;

/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter() {
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
   x_ = F_ * x_;  //x_ = F_ * x_ + u_ process noise is already inside F
   MatrixXd Ft = F_.transpose();
   P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * TODO: update the state by using Kalman Filter equations
   */
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */
  //
  //calculation of z_pred
  float px = x_(0);
  float py =x_(1);
  float vx = x_(2);
  float vy =x_(3);
  float temp = sqrt(pow(px,2)+pow(py,2));
  
    VectorXd h_x;
    // check division by zero
  if (temp < 0.0001) {
    cout << "ERROR: UpdateEKF () -Division by Zero" << endl;
    h_x<<0,0,0;}
 
  h_x<< temp, 
  		atan2(py,px),
  		(px*vx+py*vy)/temp;
  
  VectorXd z_pred = h_x; // z_pred = h_x + w, w=0
  VectorXd y = z - z_pred;
  //y[1] should be from -pi to pi, transforming y
  // Asumption: z is also only from -pi to pi, no module % requiere, rest would be enough
  if (y(1)>M_PI){y(1) -= y(1) - 2*M_PI;}
  else if (y(1)<-M_PI){y(1) = y(1) + 2*M_PI;}
  
  //Extended Kalman Filter Equations, using normal Kalman Filter Equations because in previous equations H_ was replaced by Hj
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}
