#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;
// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

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
    x_ = F_ * x_;
    MatrixXd Ft = F_.transpose();
    P_ = F_ * P_ * Ft + Q_;
    
}

void KalmanFilter::Update(const VectorXd &z) {
    VectorXd y = z - H_ * x_;
    UniversalUpdate(y);
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
    
    float rho = sqrt(x_(0)*x_(0) + x_(1)*x_(1));
    float theta = atan(x_(1) / x_(0));
    float rho_dot = (x_(0)*x_(2) + x_(1)*x_(3)) / rho;
    VectorXd h = VectorXd(3);
    h << rho, theta, rho_dot;
    VectorXd y = z - h;
    UniversalUpdate(y);
}

void KalmanFilter::UniversalUpdate(const VectorXd &y){
    MatrixXd Ht_ = H_.transpose();
    MatrixXd S_ = H_ * P_ * Ht_ + R_;
    MatrixXd Si_ = S_.inverse();
    MatrixXd K_ = P_ * Ht_ * Si_;
    //new state
    long x_size = x_.size();
    MatrixXd I_ = MatrixXd::Identity(x_size, x_size);
    P_ = (I_ - K_ * H_ )*P_;
}
