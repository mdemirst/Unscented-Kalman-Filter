#include "ukf_lidar.h"
#include "ukf_radar.h"
#include "measurement_package.h"

namespace kf
{

class UKFFusion
{
protected:
    UKFLidar ukf_lidar_m;
    UKFRadar ukf_radar_m;
    bool is_initialized_m;
    Eigen::MatrixXd P_initial_m;
    Eigen::VectorXd u_m;
    Eigen::MatrixXd R_lidar_m;
    Eigen::MatrixXd R_radar_m;

public:
    UKFFusion();
    void processMeasurement(const MeasurementPackage &measurement_pack);
    Eigen::VectorXd getX();
    Eigen::MatrixXd getP();

protected:
    void initializeFilters(const MeasurementPackage &measurement_pack);

};

} // namespace kf
