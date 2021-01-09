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
    /**
     * Constructor.
     */
    FusionEKF();

    /**
     * Destructor.
     */
    virtual ~FusionEKF();

    /**
     * Run the whole flow of the Kalman Filter from here.
     */
    void ProcessMeasurement(const MeasurementPackage &measurement_pack);

    /**
     * Kalman Filter update and prediction math lives in here.
     */
    KalmanFilter ekf_;

private:
    void Initialize(const MeasurementPackage &measurement_pack);
    bool is_initialized_{false};

    // previous timestamp
    long long previous_timestamp_{0};

    Eigen::MatrixXd R_laser_;
    Eigen::MatrixXd R_radar_;
    Eigen::MatrixXd H_laser_;
    Eigen::MatrixXd Hj_;
};

#endif  // FusionEKF_H_
