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
  
  /**
  TODO:

  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...
  */
  // Initialization Knob
  is_initialized_ = false;

  // State dimension
  n_x_ = x_.size();			// 5
  // Augmented state dimension
  n_aug_ = n_x_ + 2;			// 7
  // Lambda
  lambda_ = 3 - n_aug_;

  // Weights
  weights_ = VectorXd(2*n_aug_+1);

  // Xsig_pred
  Xsig_pred_ = MatrixXd(n_aug_, 2*n_aug_+1);

  time_us_ = 0;



}

UKF::~UKF() {}

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

  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {

    // first measurement
    cout << "UKF: " << endl;
    //ukf.x_ = VectorXd(5);
    //ukf.x_ << 1, 1, 1, 1;

    if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
    	float rho = meas_package.raw_measurements_[0];
    	float phi = meas_package.raw_measurements_[1];
    	float rho_dot = meas_package.raw_measurements_[2];

    	x_ << rho*cos(phi), rho*sin(phi), 0,phi,0;
    }
    else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
    	float px = meas_package.raw_measurements_[0];
    	float py = meas_package.raw_measurements_[1];
    	float tan = atan2(py,px);
    	while(tan < -2*M_PI) tan += 2*M_PI;
    	while(tan > 2*M_PI) tan -= 2*M_PI;
    	x_ << px, py, 0, tan, 0;
    }

    P_ << 1,0,0,0,0,
  		  0,1,0,0,0,
  		  0,0,1000,0,0,
  		  0,0,0,1000,0,
  		  0,0,0,0,1000;

    // P_ <<     0.0043,   -0.0013,    0.0030,   -0.0022,   -0.0020,
    //         -0.0013,    0.0077,    0.0011,    0.0071,    0.0060,
    //          0.0030,    0.0011,    0.0054,    0.0007,    0.0008,
    //         -0.0022,    0.0071,    0.0007,    0.0098,    0.0100,
    //         -0.0020,    0.0060,    0.0008,    0.0100,    0.0123;

    // done initializing, no need to predict or update
    time_us_ = meas_package.timestamp_;	// Update time step

    is_initialized_ = true;

    return;
  }
}


  double delta_t = (meas_package.timestamp_ - time_us_)/1000000.0;
  time_us_ = meas_package.timestamp_;

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
  
  /***************************/
  /** Generate Sigma Points **/
  /***************************/
  VectorXd x_aug_ = VectorXd(n_aug_);
  MatrixXd P_aug_ = MatrixXd(n_aug_,n_aug_);
  MatrixXd Xsig_aug_ = MatrixXd(n_aug_, 2*n_aug_+1);

  // Create mean state
  x_aug_.head(5) = x_;
  x_aug_(5) = 0;
  x_aug_(6) = 0;

  // Create Augmented covariance matrix
  P_aug_.topLeftCorner(n_x_, n_x_) << P_;
  P_aug_.bottomRightCorner(n_aug_-n_x_, n_aug_-n_x_) << std_a_*std_a_, 0,
  														0, std_yawdd_*std_yawdd_;
  
  // Derive Sqrt of P_
  MatrixXd A_aug_ = P_aug_.llt().matrixL();

  // Construct Augmented Sigma Points Matrix
  Xsig_aug_.col(0) = x_aug_;

  for(int i=0;i<n_aug_;i++){

  	Xsig_aug_.col(i + 1) = x_aug_ + sqrt(lambda_+n_aug_) * A_aug_.col(i);
  	Xsig_aug_.col(i + n_aug_+1) = x_aug_ - sqrt(lambda_+n_aug_) * A_aug_.col(i);

  }

  /*****************************\
  **** Predict Sigma Points *****
  **** Using Process Model  *****
  \*****************************/

  // Predict Sigma Points using Process Model: State Transfer
  for(int i=0; i<2*n_aug_+1; i++){

  	double px = Xsig_aug_(0,i);
  	double py = Xsig_aug_(1,i);
  	double v = Xsig_aug_(2,i);
  	double yaw = Xsig_aug_(3,i);
  	double yawd = Xsig_aug_(4,i);
  	double nu_a = Xsig_aug_(5,i);
  	double nu_yawdd = Xsig_aug_(6,i);

  	//Predict state values: determinstic items come first!
  	double px_pred_, py_pred_;

  	if(fabs(yawd) > 0.001){

  		px_pred_ = px + v/yawd * (sin(yaw + yawd * delta_t) - sin(yaw));
  		py_pred_ = py + v/yawd * (cos(yaw) - cos(yaw + yawd * delta_t));
  	}else{

  		px_pred_ = px + v * delta_t * cos(yaw);
  		py_pred_ = py + v * delta_t * sin(yaw);

  	}

  	double v_pred_ = v;
  	double yaw_pred_ = yaw + yawd * delta_t;
  	double yawd_pred_ = yawd;

  	// Now Consider the noise items
  	px_pred_ = px_pred_ + 0.5* nu_a*delta_t*delta_t * cos(yaw);
  	py_pred_ = py_pred_ + 0.5* nu_a*delta_t*delta_t * sin(yaw);
  	v_pred_ = v_pred_ + nu_a * delta_t;
  	yaw_pred_ = yaw_pred_ + 0.5* nu_yawdd*delta_t*delta_t;
  	yawd_pred_ = yawd_pred_ + nu_yawdd * delta_t;

  	// Generate Predicted Sigma Points Matrix
  	Xsig_pred_(0,i) = px_pred_;
  	Xsig_pred_(1,i) = py_pred_;
  	Xsig_pred_(2,i) = v_pred_;
  	Xsig_pred_(3,i) = yaw_pred_;
  	Xsig_pred_(4,i) = yawd_pred_;

  }

  /************************************\
  *** Predict MEAN and COVARIANCE ******
  \************************************/
  // Generate Weights
  for(int i=0; i<2*n_aug_+1; i++){

  	if(i == 0){
  		weights_(i) = lambda_ / (lambda_+n_aug_);
  	}else{
  		weights_(i) = 1 / (2*(lambda_+n_aug_));
  	}

  }

  // Predict Mean State
  for(int j=0; j<n_x_;j++){

  	x_(j) = Xsig_pred_.row(j) * weights_;	// Using Inner Product for two vectors

  }

  // Predict State Covariance Matrix
  MatrixXd tempMtx = MatrixXd(n_x_, n_x_);

  for(int k=0; k<2*n_aug_+1; k++){
  	tempMtx << weights_(k) * (Xsig_pred_.col(k) - x_) * (Xsig_pred_.col(k) - x_).transpose();
  	P_ += tempMtx;
  }


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
