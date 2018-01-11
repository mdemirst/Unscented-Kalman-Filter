#ifndef UNSCENTED_KALMAN_FILTER_H
#define UNSCENTED_KALMAN_FILTER_H

#include "kalman_filter.h"

namespace kf {
class UnscentedKalmanFilter : public KalmanFilter {
 protected:
  bool is_first_update_m;
  double timestamp_m;
  Eigen::VectorXd weights_mean_m;
  Eigen::VectorXd weights_covariance_m;
  Eigen::MatrixXd augmented_sigma_points_m;
  Eigen::MatrixXd predicted_sigma_points_m;
  Eigen::MatrixXd predicted_measurements_m;
  Eigen::VectorXd predicted_measurement_m;
  int n_x_m;
  int n_augmented_x_m;
  int n_measurement_m;
  Eigen::MatrixXd S_m;
  Eigen::MatrixXd Tc_m;
  double std_a_m = 0.2;
  double std_yawdd_m = 0.2;
  double lambda_m = 3 - n_augmented_x_m;

 public:
  UnscentedKalmanFilter();
  virtual ~UnscentedKalmanFilter();
  void initialize(double timestamp, const Eigen::VectorXd& x,
                  const Eigen::MatrixXd& P, const Eigen::VectorXd& u,
                  const Eigen::MatrixXd& R);
  void predict(double timestamp);
  void update(double timestamp, const Eigen::VectorXd& z);
  void reset(double timestamp, const Eigen::VectorXd& x,
             const Eigen::MatrixXd& P);

 protected:
  void calculateWeights();
  void generateSigmaPoints();
  void predictFromSigmaPoints(double timestamp);
  virtual Eigen::VectorXd g(double timestamp, const Eigen::VectorXd& x,
                            const Eigen::VectorXd& u) = 0;
  virtual void predictMeanAndCovariance() = 0;
  void predictMeasurements(const Eigen::VectorXd &z);
  virtual void updateSAndT() = 0;
  virtual Eigen::VectorXd h(const Eigen::VectorXd& x) = 0;
};
}

#endif
