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
  
  MatrixXd H_t = H_.transpose();
  MatrixXd I_ = MatrixXd(4, 4);
  I_ << 1, 0, 0, 0,
    	0, 1, 0, 0,
  		0, 0, 1, 0,
  		0, 0, 0, 1;
  VectorXd y = z - H_*x_;
  MatrixXd S = H_ * P_ * H_t + R_;
  MatrixXd S_i = S.inverse();
  MatrixXd K = P_ * H_t * S_i;
  x_ = x_ + K * y;
  P_ = (I_ - K*H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  double px = x_(0);
  double py = x_(1);
  double vx = x_(2);
  double vy = x_(3);
  
  
  MatrixXd H_t = H_.transpose();
  VectorXd hx(3);
  MatrixXd I_ = MatrixXd(4, 4);
  I_ << 1, 0, 0, 0,
    	0, 1, 0, 0,
  		0, 0, 1, 0,
  		0, 0, 0, 1;
     	if(fabs(px) < 0.0001){px = 0.0001;}
      	if(fabs(py) < 0.0001){py = 0.0001;}
  hx << sqrt(pow(px,2) + pow(py,2)), atan2(py,px), (px*vx + py*vy)/sqrt(pow(px,2) + pow(py,2));
  
  VectorXd y = z - hx;
  
  
  while( y(1) < -3.14 || y(1) > 3.14 )
  {
  	if(y(1) < -3.14){
    	y(1) += 2*3.14;}
  	if (y(1) > 3.14){
    	y(1) -= 2*3.14;}
  }
  MatrixXd S = H_ * P_ * H_t + R_;
  MatrixXd S_i = S.inverse();
  MatrixXd K = P_ * H_t * S_i;
  x_ = x_ + K * y;
  P_ = (I_ - K * H_) * P_;
  
}
