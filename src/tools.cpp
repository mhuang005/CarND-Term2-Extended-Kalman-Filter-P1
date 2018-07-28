#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  //initialize the RMSE vector
  VectorXd rmse = VectorXd(4);
  rmse << 0, 0, 0, 0;
  
  //(1) check if two vectors have the same size, and 
  //(2) check if the vector 'estimations' is empty  
  if (estimations.size() != ground_truth.size() || estimations.size() == 0){
     cout << "Invalid estimation or ground truth data size!" << endl;
	 return rmse;
  }
  
  for(unsigned int i = 0; i < estimations.size(); i++){
     VectorXd res = estimations[i] - ground_truth[i];
	 res = res.array() * res.array();
	 rmse += res;
  }
  
  rmse /= estimations.size();
  rmse = rmse.array().sqrt();
  
  return rmse;
}  
  
MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  
  // calculate a Jacobian here.
  MatrixXd Hj = MatrixXd(3, 4);
  
  double px = x_state(0);
  double py = x_state(1);
  double vx = x_state(2);
  double vy = x_state(3);
  double pxy2 = px * px + py * py;
  
  //check division by zero
  if (pxy2 == 0){
     cout << "Error: both px and py are 0, resulting in division by 0 in the Jacobian matrix." << endl;
  }
  else{
     Hj << px/sqrt(pxy2), py/sqrt(pxy2), 0, 0,
           -py/pxy2, px/pxy2, 0, 0,
           py*(vx*py-vy*px)/(pxy2*sqrt(pxy2)), px*(vy*px-vx*py)/(pxy2*sqrt(pxy2)), px/sqrt(pxy2), py/sqrt(pxy2); 
  }
  
  return Hj;
}
