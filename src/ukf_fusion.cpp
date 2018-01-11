#include "ukf_fusion.h"
#include <iostream>
#include "tools.h"

namespace kf {

UKFFusion::UKFFusion() {
  is_initialized_m = false;

  P_initial_m = (Eigen::MatrixXd(5, 5) << 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1,
                 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
                 1).finished();

  u_m = (Eigen::VectorXd(5) << 0.0, 0.0, 0.0, 0.0, 0.0).finished();

  R_lidar_m = (Eigen::MatrixXd(2, 2) << 0.0225, 0, 0, 0.0225).finished();

  R_radar_m = (Eigen::MatrixXd(3, 3) << 0.09, 0, 0, 0, 0.0009, 0, 0, 0, 0.09)
                  .finished();
}

void UKFFusion::processMeasurement(const MeasurementPackage &measurement_pack) {
  if (is_initialized_m == false) {
    initializeFilters(measurement_pack);
  } else {
    double timestamp = measurement_pack.timestamp_m / 1000000.0;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      ukf_radar_m.predict(timestamp);
      ukf_radar_m.update(timestamp, measurement_pack.raw_measurements_m);

      ukf_lidar_m.reset(timestamp, ukf_radar_m.getX(), ukf_radar_m.getP());
    } else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      ukf_lidar_m.predict(timestamp);
      ukf_lidar_m.update(timestamp, measurement_pack.raw_measurements_m);

      ukf_radar_m.reset(timestamp, ukf_lidar_m.getX(), ukf_lidar_m.getP());
    }
  }
}

Eigen::VectorXd UKFFusion::getX() { return ukf_radar_m.getX(); }

Eigen::MatrixXd UKFFusion::getP() { return ukf_radar_m.getP(); }

void UKFFusion::initializeFilters(const MeasurementPackage &measurement_pack) {
  double timestamp = measurement_pack.timestamp_m / 1000000.0;

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    Eigen::VectorXd x =
        (Eigen::VectorXd(5) << measurement_pack.raw_measurements_m[0] *
                                   sin(measurement_pack.raw_measurements_m[1]),
         measurement_pack.raw_measurements_m[0] *
             cos(measurement_pack.raw_measurements_m[1]),
         measurement_pack.raw_measurements_m[2], 0.0, 0.0)
            .finished();

    ukf_lidar_m.initialize(timestamp, x, P_initial_m, u_m, R_lidar_m);
    ukf_radar_m.initialize(timestamp, x, P_initial_m, u_m, R_radar_m);

    is_initialized_m = true;
  } else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
    Eigen::VectorXd x =
        (Eigen::VectorXd(5) << measurement_pack.raw_measurements_m[0],
         measurement_pack.raw_measurements_m[1], 0.0, 0.0, 0.0)
            .finished();

    ukf_lidar_m.initialize(timestamp, x, P_initial_m, u_m, R_lidar_m);
    ukf_radar_m.initialize(timestamp, x, P_initial_m, u_m, R_radar_m);

    is_initialized_m = true;
  }
}
}
