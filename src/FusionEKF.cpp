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
FusionEKF::FusionEKF()
{
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  ekf_.R_laser_ = MatrixXd(2, 2);
  ekf_.R_radar_ = MatrixXd(3, 3);
  ekf_.H_ = MatrixXd(2, 4);

  //measurement covariance matrix - laser
  ekf_.R_laser_ << 0.0225, 0,
      0, 0.0225;

  //measurement covariance matrix - radar
  ekf_.R_radar_ << 0.09, 0, 0,
      0, 0.0009, 0,
      0, 0, 0.09;

  ekf_.H_ << 1, 0, 0, 0,
      0, 1, 0, 0;

  ekf_.F_ = MatrixXd(4, 4);
  ekf_.P_ = MatrixXd(4, 4);
  ekf_.Q_ = MatrixXd(4, 4);

  noise_ax_ = 9.0;
  noise_ay_ = 9.0;
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

    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;

    ekf_.P_ << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1000, 0,
        0, 0, 0, 1000;

    ekf_.F_ << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR)
    {
      double rho = measurement_pack.raw_measurements_[0];
      double phi = measurement_pack.raw_measurements_[1];
      //double rhodot  = measurement_pack.raw_measurements_[2]; //ignoring this

      double px = rho * cos(phi);
      double py = rho * sin(phi);

      // initialise x (state) here
      ekf_.x_ << px, py, 0, 0;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER)
    {
      // use measurements directly to set position
      // velocity is not available at this point
      ekf_.x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0;
    }

    previous_timestamp_ = measurement_pack.timestamp_;

    is_initialized_ = true;
    return;
  }

  // Compute the time from the previous measurement in seconds.
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;

  ekf_.F_(0, 2) = dt;
  ekf_.F_(1, 3) = dt;

  double dt4 = dt * dt * dt * dt;
  double dt3 = dt * dt * dt;

  double s2ax = noise_ax_;
  double s2ay = noise_ay_;

  ekf_.Q_ << dt4 * s2ax / 4.0, 0, dt3 * s2ax / 2.0, 0,
      0, dt4 * s2ay / 4.0, 0, dt3 * s2ay / 2.0,
      dt3 * s2ax / 2.0, 0, dt * dt * s2ax, 0,
      0, dt3 * s2ay / 2.0, 0, dt * dt * s2ay;

  ekf_.Predict();

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR)
  {
    // Radar updates
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  }
  else
  {
    // Lidar updates
    ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  //cout << "x_ = " << ekf_.x_ << endl;
  //cout << "P_ = " << ekf_.P_ << endl;
}
