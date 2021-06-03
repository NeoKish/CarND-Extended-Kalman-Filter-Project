#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

/**
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;
  
  ekf_.P_ = MatrixXd(4, 4);
  ekf_.P_ << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1000, 0,
            0, 0, 0, 1000;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;
  
  H_laser_ << 1, 0, 0, 0,
            0, 1, 0, 0;


  
//    Setting up the the process and measurement noises
   
  noise_ax = 9;
  noise_ay = 9;
//   Initializing F matrix
  ekf_.F_ = MatrixXd(4, 4);
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
  /**
   * Initialization
   */
  if (!is_initialized_) {


    // Initializing the state ekf_.x_ with the first measurement.
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;
    
// Checking whether the first emasurement is from laser or radar
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      
      // Converting radar from polar to cartesian coordinates 
      //         and initialize state.
      float ro_val=measurement_pack.raw_measurements_[0];
      float theta_val=measurement_pack.raw_measurements_[1];
      float ro_dot_val=measurement_pack.raw_measurements_[2];
      ekf_.x_<<ro_val*cos(theta_val),
             ro_val*sin(theta_val),
             ro_dot_val*cos(theta_val),
             ro_dot_val*sin(theta_val);
      previous_timestamp_ = measurement_pack.timestamp_;


    }else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
//       Initialize state.
      ekf_.x_ << measurement_pack.raw_measurements_[0], 
              measurement_pack.raw_measurements_[1], 
              0, 
              0;
 	previous_timestamp_ = measurement_pack.timestamp_;



    }

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /**
   * Prediction
   */

  /**
   * Update the state transition matrix F according to the new elapsed time.
   * Time is measured in seconds.
   * Update the process noise covariance matrix.
 
   */
   // compute the time elapsed between the current and previous measurements
  // dt - expressed in seconds
  
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;

  float dt_2 = dt * dt;
  float dt_3 = dt_2 * dt;
  float dt_4 = dt_3 * dt;
  
// Updating F matrix
  ekf_.F_(0, 2) = dt;
  ekf_.F_(1, 3) = dt;
  
    // set the process covariance matrix Q
  ekf_.Q_ = MatrixXd(4, 4);
  ekf_.Q_ <<  dt_4/4*noise_ax, 0, dt_3/2*noise_ax, 0,
         0, dt_4/4*noise_ay, 0, dt_3/2*noise_ay,
         dt_3/2*noise_ax, 0, dt_2*noise_ax, 0,
         0, dt_3/2*noise_ay, 0, dt_2*noise_ay;
  
  
// Predict function
  ekf_.Predict();

//   Updating Hj
  Hj_ = tools.CalculateJacobian(ekf_.x_);

//      * - Use the sensor type to perform the update step.
//    * - Update the state and covariance matrices.
  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
  // Updating H and R matrices
     ekf_.H_=Hj_;
  	 ekf_.R_=R_radar_;
  // Radar updates
     ekf_.UpdateEKF(measurement_pack.raw_measurements_);

  } else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
    // Updating H and R matrices
    ekf_.H_=H_laser_;
  	ekf_.R_=R_laser_;
	// Laser updates
 	ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
