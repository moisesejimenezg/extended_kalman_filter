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
    /**
     * TODO:
     * Calculate a Jacobian here.
     */
}
}  // namespace Tools
