#ifndef MEASUREMENT_PACKAGE_H
#define MEASUREMENT_PACKAGE_H

#include "Eigen/Dense"

class MeasurementPackage {
 public:
  long long timestamp_m;

  enum SensorType { LASER, RADAR } sensor_type_;

  Eigen::VectorXd raw_measurements_m;
};

#endif /* MEASUREMENT_PACKAGE_H */
