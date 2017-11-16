#include "kalman_filter.h"
#include <cmath>
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

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
  /**
  TODO:
    * predict the state
  */
  Eigen::MatrixXd Ft = F_.transpose(); 
  x_ = F_ * x_ ; 
  P_ = F_ * P_ * Ft + Q_; 
  return ; 
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
  VectorXd y = z - H_*x_; 
  MatrixXd Ht = H_.transpose(); 
  MatrixXd S  = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse(); 
  MatrixXd K  = P_ * Ht * Si; 
  
  // update new state 
  MatrixXd I = Eigen::MatrixXd::Identity(x_.size(), x_.size()); 
  x_ = x_ + K * y ;
  P_ = (I - K * H_) * P_;
  return ;
}

namespace{
  // limit angle difference between [-pi, pi], a1 - a2 
  double angle_sub(double a1, double a2)
  {
    double ret = a1 - a2; 
    double pi2 = 2*M_PI; 
    while(ret >= M_PI) ret -= pi2; 
    while(ret <= -M_PI) ret += pi2;
    return ret; 
  }
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  VectorXd z_pred = h(x_);  // z = h(x), 
  VectorXd y = z - z_pred; 
  y(1) = angle_sub(z(1), z_pred(1)); // angle difference needs specific handling

  // std::cout<<" z_pred: "<<z_pred<<std::endl << " z_meas: "<<z<<std::endl<<" y: "<<y<<std::endl;

  MatrixXd Ht = H_.transpose(); 
  MatrixXd S  = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse(); 
  MatrixXd K  = P_ * Ht * Si; 
  
  // update new state 
  MatrixXd I = Eigen::MatrixXd::Identity(x_.size(), x_.size()); 
  x_ = x_ + K * y ;
  P_ = (I - K * H_) * P_;
  return ; 
}

VectorXd KalmanFilter::h(const VectorXd& x_state)
{
  VectorXd z(3);
  double px = x_state(0);
  double py = x_state(1); 
  double vx = x_state(2); 
  double vy = x_state(3); 

  double c1 = px*px + py*py; 
  double c2 = sqrt(c1); 
  
  z(0) = c2;  // rho 
  z(1) = atan2(py, px); //  phi
  z(2) = (px*vx + py*vy)/c2; // d(rho)/dt

  return z;
}

