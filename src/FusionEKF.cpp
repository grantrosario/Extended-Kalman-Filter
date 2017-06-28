#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

const bool USE_RADAR = true;
const bool USE_LIDAR = true;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;
  
  // acceleration noise
  noise_ax = 9;
  noise_ay = 9;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);
  ekf_.P_ = MatrixXd(4, 4);
  ekf_.F_ = MatrixXd(4, 4);
  ekf_.Q_ = MatrixXd(4, 4);
  ekf_.x_ = VectorXd(4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
        0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
        0, 0.0009, 0,
        0, 0, 0.09;

  /**
  TODO:
    * Finish initializing the FusionEKF.
    * Set the process and measurement noises
  */
  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;
  
  ekf_.P_ << 1, 0, 0, 0,
             0, 1, 0, 0,
             0, 0, 1000, 0,
             0, 0, 0, 1000;
  
  ekf_.F_ << 1, 0, 1, 0,
             0, 1, 0, 1,
             0, 0, 1, 0,
             0, 0, 0, 1;

  

}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {


  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    /**
    TODO:
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
    // first measurement
    cout << "EKF: " << endl;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      
      // Grab sensor measurements
      
      float rho = measurement_pack.raw_measurements_[0];
      float phi = measurement_pack.raw_measurements_[1];
      //float rhodot = measurement_pack.raw_measurements_[2];
      float px = rho * cos(phi);
      float py = rho * sin(phi);
      ekf_.x_ << px, py, 0, 0;
      previous_timestamp_ = measurement_pack.timestamp_;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
      
      // Grab sensor measurements
      
      float px = measurement_pack.raw_measurements_[0];
      float py = measurement_pack.raw_measurements_[1];
      ekf_.x_ << px, py, 0, 0;
      previous_timestamp_ = measurement_pack.timestamp_;
    }

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  /**
   TODO:
     * Update the state transition matrix F according to the new elapsed time.
      - Time is measured in seconds.
     * Update the process noise covariance matrix.
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;
  
  float dt_2 = dt * dt;
  float dt_3 = dt_2 * dt;
  float dt_4 = dt_3 * dt;
  
  // Update state transition matrix F
  ekf_.F_(0, 2) = dt;
  ekf_.F_(1, 3) = dt;
  
  // Update noise covariance matrix Q
  ekf_.Q_ << dt_4/4*noise_ax, 0, dt_3/2*noise_ax, 0,
             0, dt_4/4*noise_ay, 0, dt_3/2*noise_ay,
             dt_3/2*noise_ax, 0, dt_2*noise_ax, 0,
             0, dt_3/2*noise_ay, 0, dt_2*noise_ay;
  

  ekf_.Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
   TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
    
    // Compute Jacobian Matrix based on predicted values and store it in H_ matrix
    Hj_ = Tools().CalculateJacobian(ekf_.x_);
    ekf_.H_ = Hj_;
    
    ekf_.R_ = R_radar_;
    
    // Call update function using new measurements
    if(USE_RADAR)
    {
      ekf_.UpdateEKF(measurement_pack.raw_measurements_);
    }
    
  } else {
    // Laser updates
    
    // Store original H_laser matrix in H_ matrix
    ekf_.H_ = H_laser_;
    
    ekf_.R_ = R_laser_;
    
    // Call update function using new measurments
    if(USE_LIDAR)
    {
      ekf_.Update(measurement_pack.raw_measurements_);
    }
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
