#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

/**
 * Initializes Unscented Kalman filter
 * This is scaffolding, do not modify
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 30;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 30;
  
  //DO NOT MODIFY measurement noise values below these are provided by the sensor manufacturer.
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
  //DO NOT MODIFY measurement noise values above these are provided by the sensor manufacturer.

  // set to true in first call of ProcessMeasurement
  is_initialized_ = false;

  // time when the state is true, in us
  time_us_ = 0.0;

  // set state dimension
  n_x_ = 5;

  // set augmented dimension
  n_aug_ = n_x_ + 2;

  // define spreading parameter
  lambda_ = 3 - n_aug_;

  // set weights of sigma points
  weights_ = VectorXd((2*n_aug_) + 1);
  weights_.segment(1, (2*n_aug_)).fill(0.5 / (n_aug_ + lambda_));
  weights_(0) = lambda_ / (lambda_ + n_aug_);

  // create vector for predicted state
  x_ = VectorXd(n_x_);

  // create covariance matrix for prediction
  P_ = MatrixXd::Identity(n_x_, n_x_);

  // create predicted sigma points matrix
  Xsig_aug_  = MatrixXd(n_aug_, 2 * n_aug_ + 1);
  Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);
}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
   if (!is_initialized_) {
     double px = 0;
     double py = 0;
     std::cout << "Begin Initialization" << std::endl;

     if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
       double rho = meas_package.raw_measurements_[0];
       double phi = meas_package.raw_measurements_[1];
       px = rho * cos(phi);
       py = rho * sin(phi);

       // Check for zeros, if either are zero, initialize with high uncertainty
       if (fabs(px) < 0.001) {
         px = 1;
         P_(0,0) = 1000;
       }
       if (fabs(py) < 0.001) {
         py = 1;
         P_(1,1) = 1000;
       }
       std::cout << "First measurement is RADAR" << std::endl;
     }
     else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
       std::cout << "First measurement is LASER" << std::endl;
       px = meas_package.raw_measurements_[0];
       py = meas_package.raw_measurements_[1];
     }

     x_ << px, py, 0., 0., 0.;
     previous_timestamp = meas_package.timestamp_;
     is_initialized_ = true;
     return;
  }
  std::cout << "UKF::ProcessMeasurement" << std::endl;

  if ((meas_package.sensor_type_ == MeasurementPackage::RADAR) && (use_radar_ == true)) {
    double delta_t = (meas_package.timestamp_ - previous_timestamp) / 1000000.0;
    previous_timestamp = meas_package.timestamp_;
    Prediction(delta_t);
    UpdateRadar(meas_package);
  }
  else if ((meas_package.sensor_type_ == MeasurementPackage::LASER) && (use_laser_ == true)) {
    double delta_t = (meas_package.timestamp_ - previous_timestamp) / 1000000.0;
    previous_timestamp = meas_package.timestamp_;
    Prediction(delta_t);
    UpdateLidar(meas_package);
  }
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
}
