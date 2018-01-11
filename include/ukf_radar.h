#include "UKF/unscented_kalman_filter.h"

namespace kf {
class UKFRadar : public UnscentedKalmanFilter {
 protected:
 public:
  UKFRadar();

 protected:
  void predictMeanAndCovariance();
  Eigen::VectorXd g(double timestamp, const Eigen::VectorXd& x,
                    const Eigen::VectorXd& u);
  void updateSAndT();
  Eigen::VectorXd h(const Eigen::VectorXd& x);
};
}
