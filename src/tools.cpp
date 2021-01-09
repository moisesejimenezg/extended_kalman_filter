#include "tools.h"

#include <cmath>
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;
namespace Tools
{
VectorXd CalculateRMSE(const vector<VectorXd> &estimations, const vector<VectorXd> &ground_truth)
{
    VectorXd rmse(4);
    rmse << 0, 0, 0, 0;
    if (estimations.size() != ground_truth.size() || estimations.size() == 0)
    {
        std::cout << "Invalid estimation or ground_truth data" << std::endl;
        return rmse;
    }

    for (unsigned int i = 0; i < estimations.size(); ++i)
    {
        VectorXd residual = estimations[i] - ground_truth[i];
        residual = residual.array() * residual.array();
        rmse += residual;
    }

    rmse = rmse / estimations.size();
    rmse = rmse.array().sqrt();
    return rmse;
}

MatrixXd CalculateJacobian(const VectorXd &x_state)
{
    MatrixXd Hj(3, 4);
    float px = x_state(0);
    float py = x_state(1);
    float vx = x_state(2);
    float vy = x_state(3);

    float c1 = px * px + py * py;
    float c2 = sqrt(c1);
    float c3 = (c1 * c2);

    if (fabs(c1) < 0.0001)
    {
        std::cout << "CalculateJacobian () - Error - Division by Zero" << std::endl;
        return Hj;
    }

    // clang-format off
    Hj <<   (px / c2), (py / c2), 0, 0,
            -(py / c1), (px / c1), 0, 0,
            py * (vx * py - vy * px) / c3, px * (px * vy - py * vx) / c3, px / c2, py / c2;
    // clang-format on
    return Hj;
}

Eigen::VectorXd ToCartesian(const Eigen::VectorXd &polar)
{
    // TODO: Angles must be between -pi and pi
    VectorXd cartesian{4};
    const auto rho{polar[0]};
    const auto theta{polar[1]};
    const auto rho_dot{polar[2]};
    cartesian << std::cos(theta) * rho, std::sin(theta) * rho, std::cos(theta) * rho_dot,
        std::sin(theta) * rho_dot;
    return cartesian;
}
}  // namespace Tools
