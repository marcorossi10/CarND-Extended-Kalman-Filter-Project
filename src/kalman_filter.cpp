#include "kalman_filter.h"
#include "FusionEKF.h"

#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

using std::cout;
using std::endl;

/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Predict() 
{
  /**
   * TODO: predict the state
   */
  x_ = F_ * x_;
  P_= F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * TODO: update the state by using Kalman Filter equations (Laser updates)
   */

  VectorXd y = z - H_laser_ * x_; //innovation calculation
  MatrixXd Ht = H_laser_.transpose();
  MatrixXd S = H_laser_ * P_ * Ht + R_laser_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  //new estimate
  x_ = x_ + (K * y);
  P_ = (I_ - K * H_laser_) * P_;
  //P_ = (I_ - K * H_laser_) * P_* (I_ - K * H_laser_).transpose() + K * R_laser_ * K.transpose();
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations (Radar updates)
   */

  VectorXd z_hat = VectorXd(3);
  PredictMeasurement(z_hat);

  VectorXd y = z - z_hat; //innovation calculation

  while (y[1] < -M_PI) 
  {
    y[1] = y[1] + 2 * M_PI;
  }

  while (y[1] > M_PI) 
  {
    y[1] = y[1] - 2 * M_PI;
  }

  H_j_ = tools_.CalculateJacobian(x_);

  MatrixXd H_jt = H_j_.transpose();
  MatrixXd S = H_j_ * P_ * H_jt + R_radar_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * H_jt;
  MatrixXd K = PHt * Si;

  //new estimate
  x_ = x_ + (K * y);
  P_ = (I_ - K * H_j_) * P_;

  //P_ = (I_ - K * H_j_) * P_* (I_ - K * H_j_).transpose() + K * R_radar_ * K.transpose();
}

void KalmanFilter::PredictMeasurement(VectorXd &z_hat) {
  /**
   * x_[0] = px // x_[1] = py // x_[2] = vx // x_[3] = vy
   */
  z_hat[0] = sqrt(x_[0]*x_[0] + x_[1]*x_[1]);
  z_hat[1] = atan2(x_[1],x_[0]);
  z_hat[2] = (x_[0]*x_[2] + x_[1]*x_[3])/z_hat[0];
}
