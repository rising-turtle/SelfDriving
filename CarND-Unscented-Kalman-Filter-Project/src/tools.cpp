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
  VectorXd rmse = VectorXd(4); 
  rmse.fill(0); 
  if(estimations.size() <= 0)
  {
    return rmse;
  }
  if(estimations.size() != ground_truth.size())
  {
    return rmse; 
  } 
  
  for(int i=0; i<estimations.size(); i++)
  {
    VectorXd e = estimations[i] - ground_truth[i]; 
    // while(e(3) > M_PI) e(3) -= 2*M_PI;
    // while(e(3) < -M_PI) e(3) += 2*M_PI;
    VectorXd e2 = e.array()*e.array()  ;
    rmse += e2;
  }
  rmse /= estimations.size(); 
  rmse = rmse.array().sqrt();

  return rmse;
}
