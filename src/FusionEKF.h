#ifndef FusionEKF_H_
#define FusionEKF_H_

#include <fstream>
#include <string>
#include <vector>

#include "Eigen/Dense"
#include "kalman_filter.h"
#include "measurement_package.h"

class FusionEKF
{
public:
    FusionEKF();

    virtual ~FusionEKF() = default;

    void ProcessMeasurement(const MeasurementPackage &measurement_pack);

    KalmanFilter ekf_;

private:
    void Initialize(const MeasurementPackage &measurement_pack);
    float CalculateTimeDifferenceAndUpdatePrevious(const MeasurementPackage &measurement_pack);
    void UpdateStateTransitionMatrix(const float &dt);
    void UpdateNoiseCovarianceMatrix(const float &dt);

    bool is_initialized_{false};

    long long previous_timestamp_{0};

    Eigen::MatrixXd R_laser_;
    Eigen::MatrixXd R_radar_;
    Eigen::MatrixXd H_laser_;
    Eigen::MatrixXd Hj_;
};

#endif  // FusionEKF_H_
