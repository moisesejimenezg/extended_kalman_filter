#include "FusionEKF.h"

#include <iostream>

#include "Eigen/Dense"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

FusionEKF::FusionEKF()
{
    is_initialized_ = false;

    previous_timestamp_ = 0;

    R_laser_ = MatrixXd(2, 2);
    R_radar_ = MatrixXd(3, 3);
    H_laser_ = MatrixXd(2, 4);
    // clang-format off
    H_laser_ << 1, 0, 0, 0,
                0, 1, 0, 0;
    // clang-format on
    Hj_ = MatrixXd(3, 4);

    // clang-format off
    R_laser_ << 0.0225, 0,
                0, 0.0225;

    R_radar_ << 0.09, 0, 0,
                0, 0.0009, 0,
                0, 0, 0.09;
    // clang-format on

    Eigen::VectorXd x{4};
    Eigen::MatrixXd P{4, 4};
    // clang-format off
    P <<    1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1000, 0,
            0, 0, 0, 1000;
    Eigen::MatrixXd F{4, 4};
    F <<    1, 0, 1, 0,
            0, 1, 0, 1,
            0, 0, 1, 0,
            0, 0, 0, 1;
    // clang-format on
    Eigen::MatrixXd Q{4, 4};
    ekf_.Init(x, P, F, H_laser_, R_laser_, Q);
}

FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack)
{
    if (!is_initialized_)
    {
        Initialize(measurement_pack);
        return;
    }

    const auto dt{CalculateTimeDifferenceAndUpdatePrevious(measurement_pack)};
    UpdateStateTransitionMatrix(dt);
    UpdateNoiseCovarianceMatrix(dt);

    ekf_.Predict();

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR)
    {
        // TODO: Radar updates
    }
    else
    {
        ekf_.Update(measurement_pack.raw_measurements_);
    }

    cout << "x_ = " << ekf_.x_ << endl;
    cout << "P_ = " << ekf_.P_ << endl;
}

void FusionEKF::Initialize(const MeasurementPackage &measurement_pack)
{
    ekf_.x_ = VectorXd(4);
    previous_timestamp_ = measurement_pack.timestamp_;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR)
    {
        ekf_.x_ << Tools::ToCartesian(measurement_pack.raw_measurements_);
        ekf_.x_(2) = 0;
        ekf_.x_(3) = 0;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER)
    {
        ekf_.x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0,
            0;
    }
    is_initialized_ = true;
}

float FusionEKF::CalculateTimeDifferenceAndUpdatePrevious(
    const MeasurementPackage &measurement_pack)
{
    const static auto millisecond_to_second{1e6};
    const float dt{(measurement_pack.timestamp_ - previous_timestamp_) / millisecond_to_second};
    previous_timestamp_ = measurement_pack.timestamp_;
    return dt;
}

void FusionEKF::UpdateStateTransitionMatrix(const float dt)
{
    ekf_.F_(0, 2) = dt;
    ekf_.F_(1, 3) = dt;
}

void FusionEKF::UpdateNoiseCovarianceMatrix(const float dt)
{
    const static auto noise_ax{9.0f};
    const static auto noise_ay{9.0f};
    const auto dt_2{dt * dt};
    const auto dt_3{dt_2 * dt};
    const auto dt_4{dt_3 * dt};
    // clang-format off
    ekf_.Q_ <<  dt_4/4*noise_ax, 0, dt_3/2*noise_ax, 0,
                0, dt_4/4*noise_ay, 0, dt_3/2*noise_ay,
                dt_3/2*noise_ax, 0, dt_2*noise_ax, 0,
                0, dt_3/2*noise_ay, 0, dt_2*noise_ay;
    // clang-format on
}
