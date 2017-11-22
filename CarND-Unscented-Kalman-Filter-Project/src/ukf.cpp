#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 * This is scaffolding, do not modify
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true; // true

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true; // true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 1; // 2  30 

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 2; // 2 30

  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;

  // Parameters above this line are scaffolding, do not modify
  
  /**
  TODO:

  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...
  */

  // for initialization 
  is_initialized_ = false; 
  
  n_x_ = 5; 
  n_aug_ = n_x_ + 2; // 2: std_a_ & std_yawdd_; 
  
  x_aug_ = VectorXd(n_aug_); 
  P_aug_ = MatrixXd(n_aug_, n_aug_); 
  
  lambda_ = 3 - n_aug_; 

  // Initialize sigma points 
  Xsig_pred_ = MatrixXd(n_x_, 2*n_aug_ + 1); 
  Xsig_aug_ = MatrixXd(n_aug_, 2*n_aug_ + 1); 
  
  // process noise matrix 
  Q_ = MatrixXd(2, 2); 
  Q_ << std_a_*std_a_ , 0,
        0, std_yawdd_ * std_yawdd_;

  // radar noise matrix 
  R_radar_ = MatrixXd(3, 3); 
  R_radar_ << std_radr_*std_radr_ , 0, 0,
             0, std_radphi_*std_radphi_,0,
             0, 0, std_radrd_*std_radrd_;
  // laser nosie matrix 
  R_laser_ = MatrixXd(2, 2);
  R_laser_ << std_laspx_ * std_laspx_, 0, 
              0, std_laspy_ * std_laspy_; 
  // weights 
  weights_ = VectorXd(2*n_aug_ + 1); 
  weights_(0) = lambda_/(lambda_ + n_aug_); 
  for(int i=1; i<2*n_aug_+1; i++)
  {
    weights_(i) = 1/(2*(lambda_ + n_aug_)); 
  }

  // laser & radar NIS output file
  laser_ouf_ = new ofstream("laser_NIS.txt");
  radar_ouf_ = new ofstream("radar_NIS.txt"); 
}

UKF::~UKF() 
{
  if(laser_ouf_) { delete laser_ouf_; laser_ouf_ = 0; }
  if(radar_ouf_) { delete radar_ouf_; radar_ouf_ = 0; }
}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Make sure you switch between lidar and radar
  measurements.
  */
  if(!use_laser_ && meas_package.sensor_type_ == MeasurementPackage::LASER) // skip LASER Measurement 
    return ; 

  if(!use_radar_ && meas_package.sensor_type_ == MeasurementPackage::RADAR) // skip RADAR Measurement
    return ;

  if(!is_initialized_)
  {
    double px, py, phi;
    if(meas_package.sensor_type_ == MeasurementPackage::RADAR)
    {
      // x ~ [px, py, vel, phi, phid ]
      double d   = meas_package.raw_measurements_[0];
      phi = meas_package.raw_measurements_[1];  
      px  = d * cos(phi); 
      py  = d * sin(phi); 
    }else if(meas_package.sensor_type_ == MeasurementPackage::LASER)
    {
      px = meas_package.raw_measurements_[0]; 
      py = meas_package.raw_measurements_[1]; 
      phi = atan2(py, px);
    }else
    {
      cerr << "ukf.cpp: failed to identify measurement type for initialization! "<<endl;
      return ; 
    }
   
    // set initial value for x_ and x_aug_
    x_ << px, py, 0, phi, 0; 
    x_aug_ << px, py, 0, phi, 0, 0, 0; 
    cout << "ukf: INITIALIZE x_aug: "<<x_aug_<<endl;

    // init P and P_AUG
    MatrixXd I = MatrixXd::Identity(n_x_, n_x_); 
    P_ = I*10;

    previous_timestamp_ = meas_package.timestamp_;
    is_initialized_ = true;  
    return ;
  }

  // prediction 
  double dt = (meas_package.timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = meas_package.timestamp_; 
  
  Prediction(dt); 

  // update 
  if(meas_package.sensor_type_ == MeasurementPackage::RADAR)
  {
    // std::cout<<"ukf: RADAR update! "<<std::endl;
    UpdateRadar(meas_package);  

  }else if(meas_package.sensor_type_ == MeasurementPackage::LASER)
  {
    // std::cout <<"ukf: LASER update! "<<std::endl;
    UpdateLidar(meas_package); 
  }else
  {
    cout <<"ukf: receive measurement neither RADAR nor LASER!"<<endl; 
    return ;
  }

  return; 
}

/**
 * Generate sigma points 
 */
void UKF::GenerateSigPoints()
{
    // generate P_aug_
    P_aug_.fill(0); 
    P_aug_.block(0, 0, n_x_, n_x_) = P_;
    P_aug_.block(n_x_, n_x_, 2, 2) = Q_;

    // std::cout<<"ukf: GenerateSigPoints: P_aug_: "<<P_aug_<<std::endl;

    // create root matrix
    MatrixXd A = P_aug_.llt().matrixL(); 

    // std::cout<<"ukf: GenerateSigPoints: A: "<<A<<std::endl;

    // create augmented sigma points
    Xsig_aug_.col(0) = x_aug_;
    for(int i=1; i<=n_aug_; i++)
    {
      Xsig_aug_.col(i) = x_aug_ + sqrt(lambda_ + n_aug_) * A.col(i-1); 
      Xsig_aug_.col(i+n_aug_) = x_aug_ - sqrt(lambda_ + n_aug_) * A.col(i-1);
    }
    return ; 
}



/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  /**
  TODO:

  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */
  // 1. generate sig points based on [mean, sigma]
  GenerateSigPoints(); 

  // 2. transform sigma points
  for(int i=0; i<2*n_aug_+1; i++)
  {
    VectorXd x = Xsig_aug_.col(i);
    VectorXd y = VectorXd(n_x_);
    if(fabs(x(4)) <= 1e-3) // phid = 0
    {
      y(0) = x(0) + x(2) * cos(x(3)) * delta_t + 0.5 * delta_t * delta_t * cos(x(3))*x(5); 
      y(1) = x(1) + x(2) * sin(x(3)) * delta_t + 0.5 * delta_t * delta_t * sin(x(3))*x(5); 
      y(2) = x(2) + delta_t * x(5);
      y(3) = x(3) + 0.5 * delta_t * delta_t * x(6); 
      y(4) = x(4) + delta_t * x(6); 
    }else
    {
      y(0) = x(0) + x(2) * (sin(x(3) + x(4) * delta_t) - sin(x(3)))/x(4)  + 0.5 * delta_t * delta_t * cos(x(3))*x(5); 
      y(1) = x(1) + x(2) * (-cos(x(3) + x(4) * delta_t) + cos(x(3)))/x(4)  + 0.5 * delta_t * delta_t * sin(x(3))*x(5); 
      y(2) = x(2) + delta_t * x(5);
      y(3) = x(3) + x(4) * delta_t + 0.5 * delta_t * delta_t * x(6); 
      y(4) = x(4) + delta_t * x(6); 
    }
    Xsig_pred_.col(i) = y; 
  }
  // 3. estimate new [mean, sigma]
  // predict state mean 
  VectorXd xt = VectorXd(n_x_); 
  xt.fill(0); 
  
  for(int i=0; i<2*n_aug_+1; i++)
  {
    xt += weights_(i) * Xsig_pred_.col(i); 
  }
  // estimate new sigma 
  MatrixXd Pt = MatrixXd(n_x_, n_x_); 
  Pt.fill(0); 
  for(int i=0; i<2*n_aug_+1; i++)
  {
    VectorXd e = Xsig_pred_.col(i) - xt; 
    while(e(3) > M_PI) e(3) -= 2*M_PI; 
    while(e(3) < -M_PI) e(3) += 2*M_PI;
    Pt += weights_(i) * e * e.transpose(); 
  }

  // set new prediction 
  x_ = xt;
  x_aug_.head(n_x_) = x_; 
  P_ = Pt; 

  // print prediction 
  // std::cout<<"ukf: PREDICTION X: "<<x_<<std::endl;
  // std::cout<<"ukf: PREDICTION P: "<<P_<<std::endl;

  return ; 
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */
  
  int n_z = 2;
  // 1. compute Zsig_pts = h(Xsig_pts) 
  MatrixXd Zsig_pred = MatrixXd(n_z, 2*n_aug_+1);
  for(int i=0; i<2*n_aug_+1; i++)
  {
    VectorXd z = VectorXd(n_z); 
    z(0) = Xsig_pred_.col(i)(0); 
    z(1) = Xsig_pred_.col(i)(1); 
    Zsig_pred.col(i) = z; 
  }
  
  // 2. compute new [z_mean, z_S]
  VectorXd z_mu = VectorXd(n_z); 
  z_mu.fill(0);
  for(int i=0; i<2*n_aug_+1; i++)
  {
    z_mu += weights_(i) * Zsig_pred.col(i); 
  }

  MatrixXd z_S = MatrixXd(n_z, n_z); 
  z_S.fill(0); 
  for(int i=0; i<2*n_aug_+1; i++)
  {
    VectorXd e = Zsig_pred.col(i) - z_mu; 
    z_S += weights_(i) * e * e.transpose(); 
  }
  z_S += R_laser_; 

  // 3. compute cross-correlation matrix 
  MatrixXd Tc = MatrixXd(n_x_, n_z); 
  Tc.fill(0);
  for(int i=0; i<2*n_aug_+1; i++)
  {
    VectorXd ex = Xsig_pred_.col(i) - x_; 
    VectorXd ez = Zsig_pred.col(i) - z_mu; 
    while(ex(3) > M_PI) ex(3) -= 2.*M_PI;
    while(ex(3) < -M_PI) ex(3) += 2.*M_PI; 
    
    Tc += weights_(i) * ex * ez.transpose(); 
  }
  
  // 4. update [x_, P_]
  VectorXd z = VectorXd(n_z); 
  z << meas_package.raw_measurements_[0], meas_package.raw_measurements_[1]; 
  
  // Kalman gain 
  MatrixXd K = Tc * z_S.inverse(); 
  
  // residual 
  VectorXd z_diff = z - z_mu; 

  // cout<<"ukf LASER: z: "<<z<<endl;
  // cout<<"ukf LASER: z_pred: "<<z_mu<<endl;
  // cout<<"ukf LASER: z_diff: "<<z_diff<<endl;

  // nis 
  double nis = z_diff.transpose() * z_S.inverse() * z_diff;
  laser_nis_.push_back(nis); 
  (*laser_ouf_) << nis<<endl;
  // cout <<"LASER NIS = "<<nis<<" laser_nis size: "<<laser_nis_.size()<<endl; 

  // update state mean and covariance matrix
  x_ = x_ + K * z_diff;
  x_aug_.head(5) = x_; 
  P_ = P_ - K * z_S * K.transpose(); 

  // std::cout <<"ukf: LASER Update x = "<<x_<<std::endl;
  // std::cout <<"ukf: LASER Update P = "<<P_<<std::endl;
  return ; 
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */
  
  int n_z = 3; 
  // 1. compute Zsig_pred = h(Xsig_pred)
  MatrixXd Zsig_pred = MatrixXd(n_z, 2*n_aug_ + 1); 
  for(int i=0; i<2*n_aug_+1; i++)
  {
    VectorXd z = VectorXd(3); 
    VectorXd x = Xsig_pred_.col(i);
    double px = x(0); 
    double py = x(1); 
    double vx = x(2) * cos(x(3)); 
    double vy = x(2) * sin(x(3)); 
    z(0) = sqrt(px * px + py * py); 
    if(fabs(z(0)) < 1e-5) z(0) += 1e-5; 
    z(1) = atan2(py, px); 
    z(2) = (px * vx + py * vy)/z(0);
    Zsig_pred.col(i) = z;
  }
  
  // 2. compute z_mu, z_S
  VectorXd z_mu = VectorXd(n_z); 
  z_mu.fill(0);
  for(int i=0; i<2*n_aug_ + 1; i++)
  {
    z_mu += weights_(i) * Zsig_pred.col(i); 
  }
  
  MatrixXd z_S = MatrixXd(n_z, n_z); 
  z_S.fill(0); 
  for(int i=0; i<2*n_aug_ + 1; i++)
  {
    VectorXd e = Zsig_pred.col(i) - z_mu; 
    while(e(1) > M_PI) e(1) -= 2*M_PI; 
    while(e(1) < -M_PI) e(1) += 2*M_PI; 
    z_S += weights_(i) * e * e.transpose(); 
  }
  z_S += R_radar_; 

  // 3. compute cross-correlation matrix 
  MatrixXd Tc = MatrixXd(n_x_, n_z); 
  Tc.fill(0); 
  for(int i=0; i<2*n_aug_+1; i++)
  {
    VectorXd ez = Zsig_pred.col(i) - z_mu; 
    VectorXd ex = Xsig_pred_.col(i) - x_;
    while(ez(1) > M_PI) ez(1) -= 2*M_PI; 
    while(ez(1) < -M_PI) ez(1) += 2*M_PI; 
    while(ex(3) > M_PI) ex(3) -= 2*M_PI; 
    while(ex(3) < -M_PI) ex(3) += 2*M_PI; 
    
    Tc += weights_(i) * ex * ez.transpose(); 
  }

  // 4. update [x_, P_]

  VectorXd z = VectorXd(n_z); 
  z << meas_package.raw_measurements_[0], meas_package.raw_measurements_[1], meas_package.raw_measurements_[2]; 
  
  // Kalman gain 
  MatrixXd K = Tc * z_S.inverse(); 
  
  // residual 
  VectorXd z_diff = z - z_mu; 
  while(z_diff(1) > M_PI) z_diff(1) -= 2*M_PI; 
  while(z_diff(1) < -M_PI) z_diff(1) += 2*M_PI; 

  // cout<<"ukf RADAR: z: "<<z<<endl;
  // cout<<"ukf RADAR: z_pred: "<<z_mu<<endl;
  // cout<<"ukf RADAR: z_diff: "<<z_diff<<endl;

  // nis 
  double nis = z_diff.transpose() * z_S.inverse() * z_diff;
  radar_nis_.push_back(nis); 
  (*radar_ouf_) << nis << endl; 
  // cout <<"ukf: radar NIS: "<<nis<<" radar_nis_.size() = "<<radar_nis_.size()<<endl;

  // update state mean and covariance matrix
  x_ = x_ + K * z_diff;
  x_aug_.head(5) = x_;
  P_ = P_ - K * z_S * K.transpose(); 

  // std::cout <<"ukf: RADAR Update x = "<<x_<<std::endl;
  // std::cout <<"ukf: RADAR Update P = "<<P_<<std::endl;

  return ;
}
