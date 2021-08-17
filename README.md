# SDC-ExtendedKalmanFilter

My own Project from SDC Udacity Course (Project 5). This code implement a Observator/Extended Kalman Filter to predict and update the position of a car.
An Kalman Filter is not enough because the radar sensor input is nonlinear, because of that Extended Kalman Filter has to linearize the measurements using 1st Order Taylor.

1. Readme
2. FusionEKF class: This class implements Extended Kalman Filter for radar and laser sensors. 
3. kalman_filter class: This class implements general equation for kalman filter and extended kalman filter.
4. tools class: This class calculares RSME and Jacobian for EKF.


[//]: # (Image References)
[image1]: ./images/ExtendedKalmanFilter_DataSet1.png "EKF1"
[image2]: ./images/ExtendedKalmanFilter_DataSet2.png "EKF2"
[image3]: ./images/ExtendedKalmanFilter_nonLaser_DataSet1.png "EKF1"
[image4]: ./images/ExtendedKalmanFilter_nonLaser_DataSet2.png "EKF2"
[image5]: ./images/ExtendedKalmanFilter_nonRadar_DataSet1.png "EKF1"
[image6]: ./images/ExtendedKalmanFilter_nonRadar_DataSet2.png "EKF2"

### RSME using both sensor
![alt_text][image1]<img src="./images/ExtendedKalmanFilter_DataSet1.png" width="100" height="100">
![alt_text][image2]
### RSME using only Laser sensor
![alt_text][image5]
![alt_text][image6]
### RSME using only Radar sensor
![alt_text][image3]
![alt_text][image4]

