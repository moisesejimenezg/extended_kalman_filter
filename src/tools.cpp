#include "tools.h"

#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;
namespace Tools
{
VectorXd CalculateRMSE(const vector<VectorXd> &estimations,
                       const vector<VectorXd> &ground_truth)
{
    /**
     * TODO: Calculate the RMSE here.
     */
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
}  // namespace Tools
