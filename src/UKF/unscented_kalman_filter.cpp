#include "unscented_kalman_filter.h"
#include "tools.h"

namespace kf {

UnscentedKalmanFilter::UnscentedKalmanFilter() : KalmanFilter() {
  timestamp_m = 0.0;
  is_first_update_m = true;
}

UnscentedKalmanFilter::~UnscentedKalmanFilter() {}

void UnscentedKalmanFilter::initialize(double timestamp,
                                       const Eigen::VectorXd &x,
                                       const Eigen::MatrixXd &P,
                                       const Eigen::VectorXd &u,
                                       const Eigen::MatrixXd &R) {
  timestamp_m = timestamp;
  KalmanFilter::initialize(
      x, P, u,
      Eigen::MatrixXd::Zero(x.rows(), x.cols()),  // TODO : It is not generic
      Eigen::MatrixXd::Zero(x.rows(), x.cols()),
      Eigen::MatrixXd::Zero(x.rows(), R.cols()), R);

  n_x_m = x.size();
  n_augmented_x_m = n_x_m + 2;
  n_measurement_m = R.cols();
  weights_mean_m = Eigen::VectorXd(2 * n_augmented_x_m + 1);
  weights_covariance_m = Eigen::VectorXd(2 * n_augmented_x_m + 1);
  augmented_sigma_points_m =
      Eigen::MatrixXd(n_augmented_x_m, 2 * n_augmented_x_m + 1);
  predicted_sigma_points_m = Eigen::MatrixXd(n_x_m, 2 * n_augmented_x_m + 1);
  predicted_measurements_m =
      Eigen::MatrixXd(n_measurement_m, 2 * n_augmented_x_m + 1);
  predicted_measurement_m = Eigen::VectorXd(n_measurement_m);
  S_m = Eigen::MatrixXd(n_measurement_m, n_measurement_m);
  Tc_m = Eigen::MatrixXd(n_x_m, n_measurement_m);

  std_a_m = 5.0; // ~ 22 mph
  std_yawdd_m = M_PI / 2.0;
  lambda_m = 3 - n_augmented_x_m;

  calculateWeights();
}

void UnscentedKalmanFilter::predict(double timestamp) {
  generateSigmaPoints();
  predictFromSigmaPoints(timestamp);
  predictMeanAndCovariance();
}

void UnscentedKalmanFilter::update(double timestamp, const Eigen::VectorXd &z) {
  timestamp_m = timestamp;
  predictMeasurements(z);
  updateSAndT();

  Eigen::MatrixXd K = Tc_m * S_m.inverse();

  x_m = x_m + K * (z - predicted_measurement_m);
  P_m = P_m - K * S_m * K.transpose();
}

void UnscentedKalmanFilter::reset(double timestamp, const Eigen::VectorXd &x,
                                  const Eigen::MatrixXd &P) {
  timestamp_m = timestamp;
  x_m = x;
  P_m = P;
}

void UnscentedKalmanFilter::calculateWeights() {
  weights_mean_m[0] = lambda_m / (double)(lambda_m + n_augmented_x_m);
  weights_covariance_m[0] = lambda_m / (double)(lambda_m + n_augmented_x_m);

  for (int i = 0; i < n_augmented_x_m; i++) {
    weights_mean_m[i + 1] = 1.0 / (2 * (lambda_m + n_augmented_x_m));
    weights_mean_m[i + 1 + n_augmented_x_m] =
        1.0 / (2 * (lambda_m + n_augmented_x_m));

    weights_covariance_m[i + 1] = 1.0 / (2 * (lambda_m + n_augmented_x_m));
    weights_covariance_m[i + 1 + n_augmented_x_m] =
        1.0 / (2 * (lambda_m + n_augmented_x_m));
  }
}

void UnscentedKalmanFilter::generateSigmaPoints() {
  Eigen::VectorXd x_aug = Eigen::VectorXd::Zero(n_augmented_x_m);
  Eigen::MatrixXd P_aug =
      Eigen::MatrixXd::Zero(n_augmented_x_m, n_augmented_x_m);

  x_aug.head(n_x_m) = x_m;
  P_aug.topLeftCorner(n_x_m, n_x_m) = P_m;
  P_aug(5, 5) = std_a_m * std_a_m;
  P_aug(6, 6) = std_yawdd_m * std_yawdd_m;

  Eigen::MatrixXd L = P_aug.llt().matrixL();

  augmented_sigma_points_m.col(0) = x_aug;
  for (int i = 0; i < n_augmented_x_m; i++) {
    augmented_sigma_points_m.col(i + 1) =
        x_aug + sqrt(lambda_m + n_augmented_x_m) * L.col(i);
    augmented_sigma_points_m.col(i + 1 + n_augmented_x_m) =
        x_aug - sqrt(lambda_m + n_augmented_x_m) * L.col(i);
  }
}

void UnscentedKalmanFilter::predictFromSigmaPoints(double timestamp) {
  for (int i = 0; i < augmented_sigma_points_m.cols(); i++) {
    predicted_sigma_points_m.col(i) =
        g(timestamp, augmented_sigma_points_m.col(i), u_m);
  }
}

void UnscentedKalmanFilter::predictMeasurements(const Eigen::VectorXd &z) {
  for (int i = 0; i < predicted_measurements_m.cols(); i++) {
    predicted_measurements_m.col(i) = h(predicted_sigma_points_m.col(i));
  }

  predicted_measurement_m.setZero();
  for (int i = 0; i < predicted_measurements_m.cols(); i++) {
    predicted_measurement_m +=
        weights_mean_m[i] * predicted_measurements_m.col(i);
  }
}

}  // namespace kf
