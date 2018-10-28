#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

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
  /**
  TODO:
    * predict the state
  */
  MatrixXd F_t = F_.transpose();
  x_ = F_ * x_ ;
  P_ = F_ * P_ * F_t + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
  VectorXd y;
  MatrixXd K; 
  MatrixXd S; 
  MatrixXd S_i;
  MatrixXd H_t = H_.transpose();
  MatrixXd I_;
  I_ << 1, 0, 0, 0,
    	0, 1, 0, 0,
  		0, 0, 1, 0,
  		0, 0, 0, 1;
  y = z - H_*x_;
  S = H_ * P_ * H_t + R_;
  S_i = S.inverse();
  K = P_ * H_t * S_i;
  x_ = x_ + K * y;
  P_ = (I_ - K*H_) * P_;
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
  
  VectorXd y;
  MatrixXd K; 
  MatrixXd S; 
  MatrixXd S_i;
  MatrixXd H_t = H_.transpose();
  VectorXd hx;
  MatrixXd I_;
  I_ << 1, 0, 0, 0,
    	0, 1, 0, 0,
  		0, 0, 1, 0,
  		0, 0, 0, 1;
 
  hx << sqrt(pow(px,2) + pow(py,2)), atan2(py,px), (px*vx + py*vy)/sqrt(pow(px,2) + pow(py,2));
  
  y = z - hx;
  
  float phi = y[1];
  if(phi < -3.14){
    phi += 2*3.14;
    y[1] = 3.14;}
  if (phi > 3.14){
    phi -= 2*3.14;
	y[1] = phi;  }
  
  S = H_ * P_ * H_t + R_;
  S_i = S.inverse();
  K = P_ * H_t * S_i;
  x_ = x_ + K * y;
  P_ = (I_ - K * H_) * P_;
  
}
