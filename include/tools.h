#ifndef TOOLS_H
#define TOOLS_H

#include <Eigen/Dense>
#include <vector>

class Tools {
 public:
  static Eigen::VectorXd CalculateRMSE(
      const std::vector<Eigen::VectorXd>& estimations,
      const std::vector<Eigen::VectorXd>& ground_truth);

  static Eigen::VectorXd polarToCartesian(const Eigen::VectorXd& measurement);
  static Eigen::VectorXd cartesianToPolar(const Eigen::VectorXd& measurement);

  static double wrapAngle(double angle);
  static double wrapAngleAround(double angle, double around);
};

#endif /* TOOLS_H */
