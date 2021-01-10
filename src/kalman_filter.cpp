#include "kalman_filter.h"

#include <cmath>

#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in, MatrixXd &H_in,
                        MatrixXd &R_in, MatrixXd &Q_in)
{
    x_ = x_in;
    P_ = P_in;
    F_ = F_in;
    H_ = H_in;
    R_ = R_in;
    Q_ = Q_in;
}

void KalmanFilter::Predict()
{
    x_ = F_ * x_;
    const MatrixXd Ft{F_.transpose()};
    P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z)
{
    const VectorXd z_pred{H_ * x_};
    const VectorXd y{z - z_pred};
    PerformUpdate(y);
}

void KalmanFilter::UpdateEKF(const VectorXd &z)
{
    const VectorXd z_pred{Tools::ToPolar(x_)};
    VectorXd y{z - z_pred};
    if (y(1) < -M_PI)
    {
        y(1) += 2 * M_PI;
    }
    else if (y(1) > M_PI)
    {
        y(1) -= 2 * M_PI;
    }
    H_ = Tools::CalculateJacobian(x_);
    PerformUpdate(y);
}

void KalmanFilter::PerformUpdate(const VectorXd &y)
{
    const MatrixXd Ht{H_.transpose()};
    const MatrixXd S{H_ * P_ * Ht + R_};
    const MatrixXd Si{S.inverse()};
    const MatrixXd PHt{P_ * Ht};
    const MatrixXd K{PHt * Si};

    x_ = x_ + (K * y);
    const long x_size{x_.size()};
    const MatrixXd I{MatrixXd::Identity(x_size, x_size)};
    P_ = (I - K * H_) * P_;
}
