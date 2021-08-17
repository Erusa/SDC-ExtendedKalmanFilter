# SDC-ExtendedKalmanFilter

My own Project from SDC Udacity Course (Project 5). This code implements a Observator/Extended Kalman Filter to predict and update the position of a car.
An Kalman Filter is not enough because the radar sensor input is nonlinear, because of that Extended Kalman Filter has to linearize the measurements using 1st Order Taylor.

1. Readme
2. FusionEKF class: This class implements Extended Kalman Filter for radar and laser sensors. 
3. kalman_filter class: This class implements general equation for kalman filter and extended kalman filter.
4. tools class: This class calculares RSME and Jacobian for EKF.

## RSME using both sensor
<p float="left">
<img src="./images/ExtendedKalmanFilter_DataSet1.png" width="400" height="300">
<img src="./images/ExtendedKalmanFilter_DataSet2.png" width="400" height="300">
</p>

## RSME using only Laser sensor
<p float="left">
<img src="./images/ExtendedKalmanFilter_nonLaser_DataSet1.png" width="400" height="300">
<img src="./images/ExtendedKalmanFilter_nonLaser_DataSet2.png" width="400" height="300">
</p>

## RSME using only Radar sensor
<p float="left">
<img src="./images/ExtendedKalmanFilter_nonRadar_DataSet1.png" width="400" height="300">
<img src="./images/ExtendedKalmanFilter_nonRadar_DataSet2.png" width="400" height="300">
</p>

