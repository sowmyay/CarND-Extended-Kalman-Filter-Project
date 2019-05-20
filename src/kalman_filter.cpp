#include <iostream>
#include "kalman_filter.h"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter()
{
  I_ = Eigen::MatrixXd::Identity(4, 4);
}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in)
{
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict()
{
  x_ = F_ * x_; // u is zero vector; omitted for optimization purpose
  P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z)
{
  VectorXd y = z - H_ * x_;

  UpdateOthers(H_, R_laser_, y);
}

void KalmanFilter::UpdateOthers(const MatrixXd &H, const MatrixXd &R, const VectorXd &y)
{
  MatrixXd Ht = H.transpose();
  MatrixXd S = H * P_ * Ht + R;
  MatrixXd Si = S.inverse();
  MatrixXd K = P_ * Ht * Si;

  //update
  x_ = x_ + (K * y);
  P_ = (I_ - K * H) * P_;
}

void print_dim(MatrixXd &mat)
{
  std::cout << "rows: " << mat.rows() << " cols: " << mat.cols() << std::endl;
}

void KalmanFilter::UpdateEKF(const VectorXd &z)
{
  MatrixXd Hj = Tools::CalculateJacobian(x_);

  VectorXd y = z - Tools::Hf(x_);

  //normalise angle
  while (y(1) > M_PI)
    y(1) -= M_PI;
  while (y(1) < -M_PI)
    y(1) += M_PI;

  UpdateOthers(Hj, R_radar_, y);
}