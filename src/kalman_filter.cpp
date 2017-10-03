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
    phi = phi - 2 * PI;
  } else if (phi < -PI){
    phi = phi + 2 * PI;
  }
}

void KalmanFilter::KeepInPi(double &phi) {
  if (phi > PI){
    phi = phi - 2 * PI;
  } else if (phi < -PI){
    phi = phi + 2 * PI;
  }
}

void KalmanFilter::Predict() {
  /**
  TODO:
    * predict the state
  */
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;

}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
  tools.KeepNoneZero(x_[0], ALMOST_ZERO);
  tools.KeepNoneZero(x_[1], ALMOST_ZERO);
  /*
  if ((fabs(x_[0]) < 0.0001) && (fabs(x_[1]) < 0.0001)) {
    std::cout << "step1" << std::endl;
    if (x_[0] > 0) {
      x_[0] = 0.0001;
    } else {
      x_[0] = -0.0001;
    }
    if (x_[1] > 0) {
      x_[1] = 0.0001;
    } else {
      x_[1] = -0.0001;
    }
  }*/

  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;
  std::cout << "y = " << y << std::endl; 
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;
  std::cout << "K = " << K << std::endl;
 
  //new estimate
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
 // VectorXd z_pred = H_ * x_;
  
  VectorXd z_pred = VectorXd(3);
  float px = x_[0];
  float py = x_[1];
  float vx = x_[2];
  float vy = x_[3];
  
  tools.KeepNoneZero(px, ALMOST_ZERO);
  tools.KeepNoneZero(py, ALMOST_ZERO);
  /*
  if ((fabs(px) < 0.0001) && (fabs(py) < 0.0001)) {
    std::cout << "step1" << std::endl;
    if (px > 0) {
      px = 0.0001;
    } else {
      px = -0.0001;
    }
    if (py > 0) {
      py = 0.0001;
    } else {
      py = -0.0001;
    }
  }*/

  std::cout << "step2" << std::endl;
  float rho = sqrt(px*px + py*py);
  std::cout << "step3" << std::endl;
  float phi = atan2(py, px);
  std::cout << "step4" << std::endl;
  float phi_orig = phi;
  std::cout << "e:phi = " << phi_orig << std::endl;
  /*
  if (phi > PI){
    phi = phi - 2 * PI;
  } else if (phi < -PI){
    phi = phi + 2 * PI;
  }*/
  KeepInPi(phi);
  std::cout << "e:phi = " << phi << std::endl;
  std::cout << "e:z(1) = " << z(1) << std::endl;

  tools.KeepNoneZero(rho, ALMOST_ZERO);
  /*
  if (fabs(rho) < 0.0001) {
    if (rho > 0) {
      rho = 0.0001;
    } else {
      rho = -0.0001;
    }
  }*/

  std::cout << "e:rho = " << rho << std::endl;
  std::cout << "e:(px*vx+py*vy)" << px*vx+py*vy << std::endl;
  float rho_dot = (px*vx + py*vy)/rho;
  std::cout << "e:rho_dot = " << rho_dot << std::endl;
  /*
  if (fabs(rho_dot) < 0.0001) {
    if (rho_dot > 0) {
      rho_dot = 0.0001;
    } else {
      rho_dot = -0.0001;
    }
  }*/

  std::cout << "e:rho_dot = " << rho_dot << std::endl;

  std::cout << "step5" << std::endl;
  z_pred << rho, phi, rho_dot;
  //z_pred(0) = rho;
  //z_pred(1) = phi;
  //z_pred(2) = rho_dot;
  std::cout << "e:z = " << z << std::endl;
  std::cout << "e:z_pred = " << z_pred << std::endl;
  VectorXd z_input = z;
  KeepInPi(z_input(1));
  /*
  if (z_input(1) > PI){
    z_input(1) = z_input(1) - 2 * PI;
  } else if (z_input(1) < -PI){
    z_input(1) = z_input(1) + 2 * PI;
  }*/
  //VectorXd y = z - z_pred;
  VectorXd y = z_input - z_pred;
  KeepDiffInTwoPi(y(1));
  std::cout << "e:y = " << y << std::endl; 
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;
  std::cout << "e:K = " << K << std::endl; 
  //new estimate
  std::cout << "e:K*y = " << K*y << std::endl; 
  std::cout << "e:x_ = " << x_ << std::endl; 
  x_ = x_ + (K * y);
  std::cout << "e:x_ = " << x_ << std::endl; 
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}

void KalmanFilter::KeepDiffInTwoPi(float &phi_diff) {
  if (phi_diff > PI) { 
    phi_diff = phi_diff - 2 * PI;
  } else if (phi_diff < -PI) {
    phi_diff = phi_diff + 2 * PI;
  }
}

void KalmanFilter::KeepDiffInTwoPi(double &phi_diff) {
  if (phi_diff > PI) { 
    phi_diff = phi_diff - 2 * PI;
  } else if (phi_diff < -PI) {
    phi_diff = phi_diff + 2 * PI;
  }
}
