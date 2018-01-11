#include "ukf_lidar.h"
#include "tools.h"

namespace kf {

UKFLidar::UKFLidar() {}

void UKFLidar::predictMeanAndCovariance() {
  x_m.setZero();
  for (int i = 0; i < predicted_sigma_points_m.cols(); i++) {
    x_m += weights_mean_m[i] * predicted_sigma_points_m.col(i);
  }

  P_m.setZero();
  for (int i = 0; i < predicted_sigma_points_m.cols(); i++) {
    Eigen::VectorXd x_diff = predicted_sigma_points_m.col(i) - x_m;
    x_diff(3) = Tools::wrapAngle(x_diff(3));
    P_m += weights_covariance_m[i] * x_diff * x_diff.transpose();
  }
}

Eigen::VectorXd UKFLidar::g(double timestamp, const Eigen::VectorXd &x,
                            const Eigen::VectorXd &u) {
  double dt = timestamp - timestamp_m;

  Eigen::VectorXd d_x;
  if (fabs(x[4]) < 0.0000000001) {
    d_x = (Eigen::VectorXd(n_x_m) << x[2] * cos(x[3]) * dt,
           x[2] * sin(x[3]) * dt, 0, x[4] * dt,
           0).finished();
  } else {
    d_x = (Eigen::VectorXd(n_x_m)
               << x[2] * (sin(x[3] + x[4] * dt) - sin(x[3])) / x[4],
           x[2] * (-cos(x[3] + x[4] * dt) + cos(x[3])) / x[4], 0, x[4] * dt,
           0).finished();
  }

  Eigen::VectorXd noise_x =
      (Eigen::VectorXd(n_x_m) << (1 / 2.0) * (dt * dt) * cos(x[3]) * x[5],
       (1 / 2.0) * (dt * dt) * sin(x[3]) * x[5], dt * x[5],
       (1 / 2.0) * (dt * dt) * x[6], dt * x[6])
          .finished();

  return x.head(n_x_m) + d_x + u + noise_x;
}

void UKFLidar::updateSAndT() {
  S_m.setZero();
  for (int i = 0; i < predicted_measurements_m.cols(); i++) {
    Eigen::VectorXd z_diff =
        predicted_measurements_m.col(i) - predicted_measurement_m;

    S_m += weights_covariance_m[i] * z_diff * z_diff.transpose();
  }
  S_m += R_m;

  Tc_m.setZero();
  for (int i = 0; i < predicted_measurements_m.cols(); i++) {
    Eigen::VectorXd x_diff = predicted_sigma_points_m.col(i) - x_m;

    x_diff(3) = Tools::wrapAngle(x_diff(3));

    Eigen::VectorXd z_diff =
        predicted_measurements_m.col(i) - predicted_measurement_m;

    Tc_m += weights_covariance_m[i] * x_diff * z_diff.transpose();
  }
}

Eigen::VectorXd UKFLidar::h(const Eigen::VectorXd &x) {
  return Eigen::Vector2d(x[0], x[1]);
}

}  // namespace kf
