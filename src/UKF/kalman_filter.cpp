#include "kalman_filter.h"

namespace kf {
KalmanFilter::KalmanFilter() { is_initialized_m = false; }

void KalmanFilter::initialize(const Eigen::VectorXd &x,
                              const Eigen::MatrixXd &P,
                              const Eigen::VectorXd &u,
                              const Eigen::MatrixXd &F,
                              const Eigen::MatrixXd &Q,
                              const Eigen::MatrixXd &H,
                              const Eigen::MatrixXd &R) {
  x_m = x;
  P_m = P;
  u_m = u;
  F_m = F;
  Q_m = Q;
  H_m = H;
  R_m = R;
  I_m = Eigen::MatrixXd::Identity(x.size(), x.size());

  is_initialized_m = true;
}

void KalmanFilter::predict() {
  x_m = F_m * x_m + u_m;
  P_m = F_m * P_m * F_m.transpose() + Q_m;
}

void KalmanFilter::update(const Eigen::VectorXd &z) {
  Eigen::VectorXd y = z - H_m * x_m;
  Eigen::MatrixXd S = H_m * P_m * H_m.transpose() + R_m;
  Eigen::MatrixXd K = P_m * H_m.transpose() * S.inverse();
  x_m = x_m + K * y;
  P_m = (I_m - K * H_m) * P_m;
}

Eigen::VectorXd KalmanFilter::getX() { return x_m; }

Eigen::MatrixXd KalmanFilter::getP() { return P_m; }

void KalmanFilter::setX(const Eigen::VectorXd &x) { x_m = x; }

void KalmanFilter::setP(const Eigen::MatrixXd &P) { P_m = P; }

}  // namespace kf
