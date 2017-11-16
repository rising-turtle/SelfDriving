#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
  
  VectorXd rmse(4); 
  rmse << 0, 0, 0, 0;
  
  if(estimations.size() != ground_truth.size())
  {
    std::cout <<" tools: estimations.size () ! = ground_truth.size() "<<std::endl; 
    return rmse; 
  }

  if(estimations.size() <= 0)
  {
    std::cout <<" tools: no data for rmse!"<<std::endl; 
    return rmse; 
  }

  // accumulate error 
  for(int i=0; i<estimations.size(); i++)
  {
    VectorXd e = estimations[i] - ground_truth[i]; 
    VectorXd e2 = e.array() * e.array(); 
    rmse += e2; 
  }

  // mean 
  rmse = rmse / estimations.size(); 
  
  rmse = rmse.array().sqrt(); 
  return rmse; 
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */
  
  MatrixXd Hj(3, 4); 
  double px = x_state(0);
  double py = x_state(1); 
  double vx = x_state(2); 
  double vy = x_state(3); 

  double c1 = px*px + py*py; 
  double c2 = sqrt(c1); 
  double c3 = c1*c2; 

  if(c1 <= 1e-5) 
  {
    std::cout << "tools: error divide by zero ! "<<std::endl; 
    return Hj; 
  }

  Hj << px/c2, py/c2, 0, 0, 
       -py/c1, px/c1, 0, 0,
        (py *(vx * py - vy *px))/c3, (px * (vy * px - vx * py))/c3, px/c2, py/c2; 
  return Hj ; 

}
