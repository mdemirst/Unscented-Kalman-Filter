#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include "Eigen/Dense"

namespace kf {

class KalmanFilter {
 protected:
  bool is_initialized_m;
  Eigen::VectorXd x_m;
  Eigen::MatrixXd P_m;
  Eigen::VectorXd u_m;
  Eigen::MatrixXd F_m;
  Eigen::MatrixXd Q_m;
  Eigen::MatrixXd H_m;
  Eigen::MatrixXd R_m;
  Eigen::MatrixXd I_m;

 public:
  KalmanFilter();
  void initialize(const Eigen::VectorXd& x, const Eigen::MatrixXd& P,
                  const Eigen::VectorXd& u, const Eigen::MatrixXd& F,
                  const Eigen::MatrixXd& Q, const Eigen::MatrixXd& H,
                  const Eigen::MatrixXd& R);
  void predict();
  void update(const Eigen::VectorXd& z);
  Eigen::VectorXd getX();
  Eigen::MatrixXd getP();
  void setX(const Eigen::VectorXd& x);
  void setP(const Eigen::MatrixXd& P);
};
}

#endif
