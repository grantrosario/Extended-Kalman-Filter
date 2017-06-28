#include "kalman_filter.h"
#include <iostream>

using namespace std;

using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /**
  TODO:
    * predict the state
  */
  
  // Calculate new predicitions
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  // Calculate new uncertainty
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
  
  // compare measurments with predictions
  VectorXd y = z - (H_ * x_);
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd K = P_ * Ht * Si;
  
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
  
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  
  float px = x_[0];
  float py = x_[1];
  float vx = x_[2];
  float vy = x_[3];
  float norm = sqrt((px * px) + (py * py));
  
  // create new Hx vector to represent h(x) function for RADAR updating
  VectorXd Hx(3);
  
  // convert cartesian predictions into polar coordinates
  if (norm > 0.00001)
  {
    Hx(0) = norm;
    Hx(1) = atan2(py,px);
    Hx(2) = ((px * vx) + (py * vy)) / norm;
  }
  
  else
  {
    Hx = (H_ * x_);
  }
  
  
  // compare measurements with predictions
  VectorXd y = z - Hx;
  y[1] = atan2(sin(y[1]), cos(y[1]));
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd K = P_ * Ht * Si;

  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
  
  
}
