#include "tools.h"
#include <iostream>

Eigen::VectorXd Tools::CalculateRMSE(const std::vector<Eigen::VectorXd> &estimations,
        const std::vector<Eigen::VectorXd> &ground_truth){

    Eigen::VectorXd rmse(4);
    rmse << 0,0,0,0;

    // check the validity of the following inputs:
    //  * the estimation vector size should not be zero
    //  * the estimation vector size should equal ground truth vector size
    if(estimations.size() != ground_truth.size()
            || estimations.size() == 0){
        std::cout << "Invalid estimation or ground_truth data" << std::endl;
        return rmse;
    }

    //accumulate squared residuals
    for(unsigned int i=0; i < estimations.size(); ++i){

        Eigen::VectorXd residual = estimations[i] - ground_truth[i];

        //coefficient-wise multiplication
        residual = residual.array()*residual.array();
        rmse += residual;
    }

    //calculate the mean
    rmse = rmse/estimations.size();

    //calculate the squared root
    rmse = rmse.array().sqrt();

    //return the result
    return rmse;
}



Eigen::VectorXd Tools::polarToCartesian(const Eigen::VectorXd &measurement)
{
  Eigen::VectorXd measurement_in_cartesian(4);
  measurement_in_cartesian[0] = measurement[0] * sin(measurement[1]);
  measurement_in_cartesian[1] = measurement[0] * cos(measurement[1]);
  measurement_in_cartesian[2] = measurement[2] * sin(measurement[1]);
  measurement_in_cartesian[3] = measurement[2] * cos(measurement[1]);

  return measurement_in_cartesian;
}

Eigen::VectorXd Tools::cartesianToPolar(const Eigen::VectorXd &measurement)
{
  Eigen::VectorXd measurement_in_polar(3);

  double px = measurement[0];
  double py = measurement[1];
  double vx = measurement[2];
  double vy = measurement[3];
  double px2 = px * px;
  double py2 = py * py;
  double sqrt_px2_py2 = sqrt(px2 + py2);

  if(fabs(sqrt_px2_py2) < 0.00001)
  {
    std::cout << "Error in cartesianToPolar. Division by zero!" << std::endl;
  }
  else
  {
    measurement_in_polar[0] = sqrt_px2_py2;
    measurement_in_polar[1] = atan2(py, px);
    measurement_in_polar[2] = (px*vx + py*vy) / sqrt_px2_py2;
  }

  return measurement_in_polar;
}

double Tools::wrapAngle(double angle)
{
  if(angle > M_PI)
    return wrapAngle(angle - 2 * M_PI);
  else if(angle < -M_PI)
    return wrapAngle(angle + 2 * M_PI);
  else
    return angle;
}

double Tools::wrapAngleAround(double angle, double around)
{
    if((angle-around) > M_PI)
      return wrapAngleAround(angle - 2 * M_PI, around);
    else if((angle-around) < -M_PI)
      return wrapAngleAround(angle + 2 * M_PI, around);
    else
        return angle;
}
