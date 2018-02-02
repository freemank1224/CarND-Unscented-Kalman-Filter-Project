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
  Xsig_pred_ = MatrixXd(n_x_, 2*n_aug_+1);

  time_us_ = 0;

  lidar_idx = 0;
  radar_idx = 0;

  delta_t = 0.0;

  nis_value = 0.0;



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
    cout << "=============== STEP0. Initialize UKF ====================== " << endl;
    //ukf.x_ = VectorXd(5);
    //ukf.x_ << 1, 1, 1, 1;

    if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
    	float rho = meas_package.raw_measurements_[0];
    	float phi = meas_package.raw_measurements_[1];
    	float rho_dot = meas_package.raw_measurements_[2];

    	x_ << rho*cos(phi), rho*sin(phi), 0, phi, 0;
    }else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
    	float px = meas_package.raw_measurements_[0];
    	float py = meas_package.raw_measurements_[1];
    	float tan = atan2(py,px);
    	while(tan < -M_PI) tan += 2.*M_PI;
    	while(tan > M_PI) tan -= 2.*M_PI;
    	x_ << px, py, 0, tan, 0;
    }

    // P_ << 1,0,0,0,0,
  		//   0,1,0,0,0,
  		//   0,0,1000,0,0,
  		//   0,0,0,1000,0,
  		//   0,0,0,0,1000;

    P_ <<    0.0043,   -0.0013,    0.0030,   -0.0022,   -0.0020,
            -0.0013,    0.0077,    0.0011,    0.0071,    0.0060,
             0.0030,    0.0011,    0.0054,    0.0007,    0.0008,
            -0.0022,    0.0071,    0.0007,    0.0098,    0.0100,
            -0.0020,    0.0060,    0.0008,    0.0100,    0.0123;

    // done initializing, no need to predict or update
    time_us_ = meas_package.timestamp_;	// Update time step

    is_initialized_ = true;

    cout << "=============== Finish Initialization ====================== " << endl;

    return;
  }


  delta_t = (meas_package.timestamp_ - time_us_)/1000000.0;
  time_us_ = meas_package.timestamp_;

  /********************** PREDICT **********************\
  \*****************************************************/
  Prediction(delta_t);
  //std::cout << "Current time: " << delta_t << std::endl;
  //std::cout << "meas_package.timestamp_" << meas_package.timestamp_ << std::endl;
  //std::cout << "---- After Prediction, P_ matrix is: " << P_ << std::endl;


  /********************** UPDATE ***********************\
  ****** Switch Lidar and Radar Measurement Data  *******
  \*****************************************************/
  if(use_laser_ == true){
  	
  	if (meas_package.sensor_type_ == MeasurementPackage::LASER){
  	  std::cout << "--------------- STEP3. LIDAR Update ----------------" << std::endl;
      UpdateLidar(meas_package);
      lidar_idx ++;
      std::cout << "LIDAR NO. " << lidar_idx << std::endl;
      std::cout << "****** LIDAR Updated P_: ******" << std::endl;
  	  std::cout << P_ << std::endl;

  	  //std::cout << "======================= Finish LIDAR Update =========================== " << std::endl;
  	}

  	//return;
  }

  if(use_radar_ == true){
  	
  	if (meas_package.sensor_type_ == MeasurementPackage::RADAR){
  	  std::cout << "--------------- STEP3. RADAR Update ----------------" << std::endl;
      UpdateRadar(meas_package);
      radar_idx ++;
      std::cout << "RADAR NO. " << radar_idx << std::endl;
  	  std::cout << "****** RADAR Updated P_: ******" << std::endl;
  	  std::cout << P_ << std::endl;  
  	  //std::cout << "======================= Finish RADAR Update =========================== " << std::endl;
  	}

  	//return;
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
  
  GenerateSigmaPoints(&Xsig_pred_);
  std::cout << "------- STEP1. Generate Sigma Points and Predict using Process Model--------OK!" << std::endl;
  //std::cout << "............ Xsig_pred_ for now is: .........." << std::endl;
  std::cout << Xsig_pred_ << std::endl;
  std::cout << "...................... END ..................." << std::endl;

  PredictMeanCovariance();
  std::cout << "------- STEP2. Predict State Mean & Covariance using Sigma Points --------OK!" << std::endl;

  std::cout << "---- After Prediction, P_ matrix is: " << P_ << std::endl;

  std::cout << "======================= Finish Prediction ================================== " << std::endl;
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

  /***************     STAGE 1    ****************\
  *** Predict MEASUREMENT mean and Covariance *****
  -------------------------------------------------
  -------------------------------------------------
  * According to the course, we don't need to *****
  * regenerate sigma points for this process. *****
  * Just use the sigma points and Xsig_pred . *****
  \***********************************************/
  // Parameters for radar measurements
  int n_z_ = 2;
  
  //create matrix for sigma points in measurement space
  MatrixXd Zsig_ = MatrixXd(n_z_, 2 * n_aug_ + 1);

  //mean predicted measurement
  VectorXd z_pred_ = VectorXd(n_z_);
  
  //measurement covariance matrix S
  MatrixXd S_ = MatrixXd(n_z_, n_z_); 

  // Predict Sigma Points using Measurement Model
  for(int i=0; i<2*n_aug_+1; i++){
  	VectorXd xsig_pred_ = Xsig_pred_.col(i);
  	double px = xsig_pred_(0);
    double py = xsig_pred_(1);

    Zsig_.col(i) << px, py;
  }

  // Calculate mean predicted measurement
  for(int j=0; j<n_z_; j++){
  	z_pred_(j) = Zsig_.row(j) * weights_;
  }

  // Calculate innovation covariance matrix S
  MatrixXd tempMtx = MatrixXd(n_z_,n_z_);
  S_.fill(0.0);
  tempMtx.fill(0.0);

  for(int k=0; k<2*n_aug_+1; k++){

  	  VectorXd z_diff_ = Zsig_.col(k) - z_pred_;
      tempMtx << weights_(k) * z_diff_ * z_diff_.transpose();
      S_ += tempMtx;
      //std::cout << "----Predicted S_: " << S_ << std::endl;
  }


  // Take noise into account
  MatrixXd R_ = MatrixXd(n_z_,n_z_);
  R_ << std_laspx_*std_laspx_, 0,
  	   0, std_laspy_*std_laspy_;

  // Combin S and Noise R

  S_ += R_;
  //std::cout << "----Predicted S_: " << S_ << std::endl;

  /***************     STAGE 2    ****************\
  ********* UPDATE Radar Measurements   ***********
  \***********************************************/
  // Load measurements
  VectorXd z_ = VectorXd(n_z_);
  z_ << meas_package.raw_measurements_[0],
  		meas_package.raw_measurements_[1];

  // Create cross correlation matrix
  MatrixXd Tc_ = MatrixXd(n_x_, n_z_);
  MatrixXd tempMtx2 = MatrixXd(n_x_, n_z_);
  Tc_.fill(0.0);
  tempMtx2.fill(0.0);

  // Calculate cross correlation matrix
  for(int k=0; k<2*n_aug_+1; k++){
  	  // NOT z_pred_ - z_ !!!!!!!!!!! 
   	  VectorXd z_diff_ = Zsig_.col(k) - z_pred_;
  	  
  	  VectorXd x_diff_ = Xsig_pred_.col(k) - x_;

      tempMtx2 << weights_(k) * x_diff_ * z_diff_.transpose();
      Tc_ += tempMtx2;
  }

  // Calculate Kalmann Gain
  MatrixXd K_ = Tc_ * S_.inverse();

  // Update state mean and covariance matrix
  x_ = x_ + K_*(z_ - z_pred_);
  P_ = P_ - K_ * S_ * K_.transpose();

  std::cout << "****** Lidar Update P_: ******" << std::endl;
  std::cout << P_ << std::endl;  

  /*********************** NIS ************************\
  ****** Switch Lidar and Radar Measurement Data  *****
  \****************************************************/
  std::cout << "------------ Calculate NIS ---------------" << std::endl;
  //nis = CalculateNIS()
  nis_value = CalculateNIS(z_, z_pred_, S_);

  std::cout << "------------ Calculate NIS: " << nis_value <<"---------------" << std::endl;

  
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

  /***************     STAGE 1    ****************\
  *** Predict MEASUREMENT mean and Covariance *****
  -------------------------------------------------
  -------------------------------------------------
  * According to the course, we don't need to *****
  * regenerate sigma points for this process. *****
  * Just use the sigma points and Xsig_pred . *****
  \***********************************************/
  // Parameters for radar measurements
  int n_z_ = 3;
  
  //create matrix for sigma points in measurement space
  MatrixXd Zsig_ = MatrixXd(n_z_, 2 * n_aug_ + 1);

  //mean predicted measurement
  VectorXd z_pred_ = VectorXd(n_z_);
  
  //measurement covariance matrix S
  MatrixXd S_ = MatrixXd(n_z_, n_z_);

  // Predict Sigma Points using Measurement Model
  for(int i=0; i<2*n_aug_+1; i++){
  	VectorXd xsig_pred_ = Xsig_pred_.col(i);
  	double px = xsig_pred_(0);
    double py = xsig_pred_(1);
    double v = xsig_pred_(2);
    double psi = xsig_pred_(3);

    double v1 = cos(psi)*v;
    double v2 = sin(psi)*v;

    double rho = sqrt(px*px + py*py);
    double phi = atan2(py,px);
    //while(phi > M_PI) phi -= 2.*M_PI;
    //while(phi < -M_PI) phi += 2.*M_PI;
    double rho_dot = (px*v1 + py*v2)/rho;
    //std::cout << "rho: " << std::endl << rho << std::endl;
      
    Zsig_.col(i) << rho, phi, rho_dot;
  }

  // Calculate mean predicted measurement
  for(int j=0; j<n_z_; j++){
  	z_pred_(j) = Zsig_.row(j) * weights_;
  }

  // Calculate innovation covariance matrix S
  MatrixXd tempMtx = MatrixXd(n_z_,n_z_);
  S_.fill(0.0);
  tempMtx.fill(0.0);

  for(int k=0; k<2*n_aug_+1; k++){
  	  VectorXd z_diff_ = Zsig_.col(k) - z_pred_;

  	  while(z_diff_(1) < -M_PI) z_diff_(1) += 2.*M_PI;
  	  while(z_diff_(1) > M_PI) z_diff_(1) -= 2.*M_PI;

      tempMtx << weights_(k) * z_diff_ * z_diff_.transpose();

      S_ += tempMtx;
  }

  // Take noise into account
  MatrixXd R_ = MatrixXd(n_z_,n_z_);
  R_ << std_radr_*std_radr_, 0, 0,
  	   0, std_radphi_*std_radphi_, 0,
  	   0, 0, std_radrd_*std_radrd_;

  // Combin S and Noise R

  S_ += R_;

  /***************     STAGE 2    ****************\
  ********* UPDATE Radar Measurements   ***********
  \***********************************************/
  // Load measurements
  VectorXd z_ = VectorXd(n_z_);
  z_ << meas_package.raw_measurements_[0],
  		meas_package.raw_measurements_[1],
  		meas_package.raw_measurements_[2];

  // Create cross correlation matrix
  MatrixXd Tc_ = MatrixXd(n_x_, n_z_);
  MatrixXd tempMtx2 = MatrixXd(n_x_, n_z_);
  Tc_.fill(0.0);
  tempMtx2.fill(0.0);

  // Calculate cross correlation matrix
  for(int k=0; k<2*n_aug_+1; k++){
  	  // WARNING: HERE use z_pred_ for z_diff calculations !!!!
  	  VectorXd z_diff_ = Zsig_.col(k) - z_pred_;
  	  while(z_diff_(1) < -M_PI) z_diff_(1) += 2.*M_PI;
  	  while(z_diff_(1) > M_PI) z_diff_(1) -= 2.*M_PI;
  	  // NOT z_pred_ - z_ !!!!!!!!!!! 
  	  VectorXd x_diff_ = Xsig_pred_.col(k) - x_;
  	  while(x_diff_(1) < -M_PI) x_diff_(1) += 2.*M_PI;
  	  while(x_diff_(1) > M_PI) x_diff_(1) -= 2.*M_PI;

      tempMtx2 << weights_(k) * x_diff_ * z_diff_.transpose();

      Tc_ += tempMtx2;
  }

  // Calculate Kalmann Gain
  MatrixXd K_ = Tc_ * S_.inverse();

  // Update state mean and covariance matrix
  VectorXd z_err_ = z_ - z_pred_;
  // Also need normalizations
  while(z_err_(1) < -M_PI) z_err_(1) += 2.*M_PI;
  while(z_err_(1) > M_PI) z_err_(1) -= 2.*M_PI;  

  x_ = x_ + K_* z_err_;
  P_ = P_ - K_ * S_ * K_.transpose();

  std::cout << "****** Radar Update P_: ******" << std::endl;
  std::cout << P_ << std::endl;


  /*********************** NIS ************************\
  ****** Switch Lidar and Radar Measurement Data  *****
  \****************************************************/
  
  //nis = CalculateNIS()
  nis_value = CalculateNIS(z_, z_pred_, S_);
  std::cout << "------------ Calculate NIS: " << nis_value <<"---------------" << std::endl;


}

/*************** USEFUL FUNCTIONS ****************\
***********							***************
\***************** Dyson Freeman *****************/

/******* GENERATE + PREDICT Augmented SigmaPoints **********\
\********* Return Xsig_aug_ matrix (Size: 7x15) ************/
void UKF::GenerateSigmaPoints(MatrixXd* Xsig_out){
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
  P_aug_.fill(0.0);
  P_aug_.topLeftCorner(n_x_, n_x_) << P_;
  P_aug_.bottomRightCorner(n_aug_-n_x_, n_aug_-n_x_) << std_a_*std_a_, 0,
  														0, std_yawdd_*std_yawdd_;

  //cout << "TEST TEST P_aug_ " << P_aug_ << endl;
  
  // Derive Sqrt of P_
  MatrixXd A_aug_ = P_aug_.llt().matrixL();

  // Construct Augmented Sigma Points Matrix
  Xsig_aug_.col(0) = x_aug_;

  for(int i=0;i<n_aug_;i++){

  	Xsig_aug_.col(i + 1) = x_aug_ + sqrt(lambda_+n_aug_) * A_aug_.col(i);
  	Xsig_aug_.col(i + n_aug_+1) = x_aug_ - sqrt(lambda_+n_aug_) * A_aug_.col(i);

  }

  //cout << "TEST TEST Xsig_aug_ " << Xsig_aug_ << endl;

  /*********** Predict Sigma Points *************\
  \******* SIZE changes from 7x15 to 5x15 *******/

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

  	// Now consider the noise
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




   	*Xsig_out = Xsig_pred_;

  }

}

/******* PREDICT Mean & Covariance Matrix  *******\
\************** that is x_ and P_  ***************/

void UKF::PredictMeanCovariance(){
  /*****************************************\
  *** Predict STATE Mean and Covariance *****
  \*****************************************/
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

  	//std::cout << "----Predicted x_: " << x_(j) << std::endl;

  }

  // Predict State Covariance Matrix
  MatrixXd tempMtx = MatrixXd(n_x_, n_x_);
  tempMtx.fill(0.0);
  P_.fill(0.0);
  //std::cout << "----tempMtx: " << tempMtx << std::endl;
  //std::cout << "---------------------------" << std::endl;
  //std::cout << "----Predicted P_: " << P_ << std::endl;

  for(int k=0; k<2*n_aug_+1; k++){

  	VectorXd x_diff_ = Xsig_pred_.col(k) - x_;
  	while(x_diff_(3) < -M_PI) x_diff_(3) += 2.*M_PI;
  	while(x_diff_(3) > M_PI) x_diff_(3) -= 2.*M_PI;

  	tempMtx = weights_(k) * x_diff_ * x_diff_.transpose();
  	//std::cout << "----tempMtx: " << tempMtx << std::endl;
  	P_ += tempMtx;
  	//std::cout << "---- After Prediction, P_ matrix is: " << P_ << std::endl;
  }

  // *x_out = x_;
  // *P_out = P_;

}


double UKF::CalculateNIS(const VectorXd& z, const VectorXd& z_pred, const MatrixXd& S){

	VectorXd z_err = z - z_pred;

	if (z_err.size() > 2)
	{
		while(z_err(1) < -M_PI) z_err(1) += 2.*M_PI;
		while(z_err(1) > M_PI) z_err(1) -= 2.*M_PI;
	}

	double eps = z_err.transpose() * S.inverse() * z_err;

	return eps;
}

double UKF::TransferNIS(double nis){

	return nis;
}
