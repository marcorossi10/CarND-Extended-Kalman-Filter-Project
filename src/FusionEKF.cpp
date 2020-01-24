#include "FusionEKF.h"
#include "Eigen/Dense"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

/**
 * Constructor.
 */
FusionEKF::FusionEKF()
{
  is_initialized_ = false;
  previous_timestamp_ = 0;

  /**
   * Finish initializing the FusionEKF.
   * Set the process and measurement noises
   */

  //Process covariance matrix
  ekf_.Q_ = MatrixXd(4, 4);

  //State update matrix
  ekf_.F_ = MatrixXd(4, 4);
  ekf_.F_ << 1, 0, 0, 0,
      0, 1, 0, 0,
      0, 0, 1, 0,
      0, 0, 0, 1;

  //State vector
  ekf_.x_ = VectorXd(4);

  //state covariance matrix
  ekf_.P_ = MatrixXd(4, 4);
  ekf_.P_ << 1, 0, 0, 0,
      0, 1, 0, 0,
      0, 0, 1000, 0,
      0, 0, 0, 1000;

  //measurement covariance matrix - laser/lidar
  ekf_.R_laser_ = MatrixXd(2, 2);
  ekf_.R_laser_ << 0.00225, 0,
      0, 0.00225;

  //measurement covariance matrix - radar
  ekf_.R_radar_ = MatrixXd(3, 3);

  ekf_.R_radar_ << 0.9, 0, 0,
      0, 0.0009, 0,
      0, 0, 0.09;

  //measurement matrix - laser/lidar
  ekf_.H_laser_ = MatrixXd(2, 4);
  ekf_.H_laser_ << 1, 0, 0, 0,
      0, 1, 0, 0;

  //measurement matrix - radar
  ekf_.H_j_ = MatrixXd(3, 4);

  //identity matrix to avoid multiple assignements
  ekf_.I_ = MatrixXd::Identity(ekf_.x_.size(), ekf_.x_.size());
}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack)
{
  /**
   * Initialization
   */
  if (!is_initialized_)
  {
    /**
     * Initialize the state ekf_.x_ with the first measurement.
     * Create the covariance matrix.
     * You'll need to convert radar from polar to cartesian coordinates.
     */
    cout << "EKF: " << endl;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR)
    {
      //Convert radar from polar to cartesian coordinates
      //         and initialize state.
      cout << "Received the first radar measurement" << endl;

      ekf_.x_ << measurement_pack.raw_measurements_[0] * std::cos(measurement_pack.raw_measurements_[1]),
          measurement_pack.raw_measurements_[0] * std::sin(measurement_pack.raw_measurements_[1]),
          0,
          0;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER)
    {
      // Initialize state.
      cout << "Received the first laser measurement" << endl;

      ekf_.x_ << measurement_pack.raw_measurements_[0],
          measurement_pack.raw_measurements_[1],
          0,
          0;
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

  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0; //[s]
  previous_timestamp_ = measurement_pack.timestamp_;

  float dt_2 = dt * dt;
  float dt_3 = dt_2 * dt;
  float dt_4 = dt_3 * dt;

  // Modify the F matrix so that the time is integrated
  ekf_.F_(0, 2) = dt;
  ekf_.F_(1, 3) = dt;

  // set the process covariance matrix Q
  // float noise_ax = 9.0;
  // float noise_ay = 9.0;

  float noise_ax = 3;
  float noise_ay = 3;

  ekf_.Q_ << (dt_4 / 4.0) * noise_ax, 0, (dt_3 / 2.0) * noise_ax, 0,
      0, (dt_4 / 4.0) * noise_ay, 0, (dt_3 / 2.0) * noise_ay,
      (dt_3 / 2.0) * noise_ax, 0, dt_2 * noise_ax, 0,
      0, (dt_3 / 2.0) * noise_ay, 0, dt_2 * noise_ay;

  ekf_.Predict();

  /**
   * Update
   */

  /**
   * - Use the sensor type to perform the update step.
   * - Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR)
  {
    // Radar updates
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  }

  else
  {
    // Laser updates
    ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
