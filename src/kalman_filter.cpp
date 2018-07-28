#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter() {}

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
  x_ = F_ * x_;
  P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  // update the state by using Kalman Filter equations
  MatrixXd PHt = P_ * H_.transpose();
  VectorXd y = z - H_ * x_;
  MatrixXd S = H_ * PHt + R_;
  MatrixXd K = PHt * S.inverse();
  
  x_ = x_ + K * y;
  P_ = P_ - K * H_ * P_;  
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
 
  //update the state by using Extended Kalman Filter equations
 
 //state vector in polar coordinate by a nonlinear transform
  float rho = sqrt(x_[0] * x_[0] + x_[1] * x_[1]);
  VectorXd Hx = VectorXd(3);
  Hx << rho, atan2(x_[1], x_[0]), (x_[0]*x_[2]+x_[1]*x_[3])/rho;
   
  VectorXd y = z - Hx;
  
  //normalize the angle to [-pi, pi]
  while (y[1] < -M_PI) y[1] += 2*M_PI;
  while (y[1] > M_PI) y[1] -= 2*M_PI;
  
  MatrixXd PHt = P_ * H_.transpose();  
  MatrixXd S = H_ * PHt + R_;
  MatrixXd K = PHt * S.inverse();
  
  x_ += K * y;
  P_ -= K * H_ * P_;  
}
