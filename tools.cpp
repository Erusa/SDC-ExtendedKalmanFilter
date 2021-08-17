#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;
using std::cout;
using std::endl;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
   * TODO: Calculate the RMSE here.
   */
  VectorXd rmse(4);
  rmse << 0,0,0,0;

  // check the validity of the following inputs:
  //  * the estimation vector size should not be zero
  if (estimations.size() ==0){
      cout<< "Wrong Estimation Vector: Estimation Vector should not be zero"<<endl;
      return rmse;
  }
  //  * the estimation vector size should equal ground truth vector size
  if (estimations.size() != ground_truth.size()){
      cout<< "Wrong Estimation Vector: the estimation vector size should equal ground truth vector size"<<endl;
      return rmse;
  }

  VectorXd asr(4);
  //accumulate squared residuals
  for (unsigned int i=0; i < estimations.size(); ++i) {
    // ... your code here
    asr =  estimations[i] - ground_truth[i];
    asr = asr.array()*asr.array();
    rmse += asr;
  }

  // calculate the mean
  rmse /= estimations.size();

  // calculate the squared root
  rmse = rmse.array().sqrt();

  // return the result
    // print the output
  //cout << "RMSE is calculated " << rmse<< endl;
  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
   * TODO:
   * Calculate a Jacobian here.
   */
    //size check
    MatrixXd Hj(3,4);
  
    if ( x_state.size() != 4 ) {
    cout << "ERROR: The state vector must have size 4." << endl;
    return Hj;
  }

  // recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);
  
  // temp variable 
  float px2_py2 = px*px + py*py;
  // check division by zero
  if (px2_py2 < 0.0001) {cout << "ERROR: CalculateJacobian () -Division by Zero" << endl;
    return Hj;}
      
  // compute the Jacobian matrix
    Hj << px/sqrt(px2_py2), py/sqrt(px2_py2), 0 ,0,
            -py/px2_py2, px/px2_py2, 0 , 0, 
            py*(vx*py-vy*px)/pow(px2_py2,1.5), px*(vy*px-vx*py)/pow(px2_py2,1.5), px/sqrt(px2_py2), py/sqrt(px2_py2);
    
    //cout << "Hj is calculated" << endl;
  return Hj;
}
