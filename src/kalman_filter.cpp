#include "kalman_filter.h"
#include <iostream>
#include <math.h>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std; 

#define PI 3.14159265
#define ALMOST_ZERO 0.0001

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in, MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::KeepInPi(float &phi) {
  if (phi > PI){
    phi = phi - 2*PI;
  } else if (phi < -PI){
    phi = phi + 2*PI;
  }
}

void KalmanFilter::KeepInPi(double &phi) {
  if (phi > PI){
    phi = phi - 2*PI;
  } else if (phi < -PI){
    phi = phi + 2*PI;
  }
}

void KalmanFilter::Predict() {
  /**
  TODO:
    * predict the state
  */
  x_ = F_*x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_*P_*Ft + Q_;

}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
  tools.KeepNoneZero(x_[0], ALMOST_ZERO);
  tools.KeepNoneZero(x_[1], ALMOST_ZERO);
  VectorXd z_pred = H_*x_;
  VectorXd y = z - z_pred;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_*P_*Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_*Ht;
  MatrixXd K = PHt*Si;
 
  //new estimate
  x_ = x_ + K*y;
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K*H_)*P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */  
  VectorXd z_pred = VectorXd(3);
  float px = x_[0];
  float py = x_[1];
  float vx = x_[2];
  float vy = x_[3];
  
  tools.KeepNoneZero(px, ALMOST_ZERO);
  tools.KeepNoneZero(py, ALMOST_ZERO);
  float rho = sqrt(px*px + py*py);
  float phi = atan2(py, px);
  float phi_orig = phi;
  KeepInPi(phi);
  tools.KeepNoneZero(rho, ALMOST_ZERO);
  float rho_dot = (px*vx + py*vy)/rho;
  z_pred << rho, phi, rho_dot;

  /**
  * The angle difference should be less than one pi
  */
  //VectorXd y = z - z_pred;
  VectorXd y = z - z_pred;
  KeepDiffInTwoPi(y(1));

  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_*P_*Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_*Ht;
  MatrixXd K = PHt*Si;

  //new estimate
  x_ = x_ + K*y;
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K*H_)*P_;
}

void KalmanFilter::KeepDiffInTwoPi(float &phi_diff) {
  if (phi_diff > PI) { 
    phi_diff = phi_diff - 2*PI;
  } else if (phi_diff < -PI) {
    phi_diff = phi_diff + 2*PI;
  }
}

void KalmanFilter::KeepDiffInTwoPi(double &phi_diff) {
  if (phi_diff > PI) { 
    phi_diff = phi_diff - 2*PI;
  } else if (phi_diff < -PI) {
    phi_diff = phi_diff + 2*PI;
  }
}
