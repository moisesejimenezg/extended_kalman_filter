#ifndef TOOLS_H_
#define TOOLS_H_

#include <vector>

#include "Eigen/Dense"

namespace Tools
{
Eigen::VectorXd CalculateRMSE(const std::vector<Eigen::VectorXd> &estimations,
                              const std::vector<Eigen::VectorXd> &ground_truth);

Eigen::MatrixXd CalculateJacobian(const Eigen::VectorXd &x_state);
Eigen::VectorXd ToCartesian(const Eigen::VectorXd &polar);
Eigen::VectorXd ToPolar(const Eigen::VectorXd &cartesian);
}  // namespace Tools

#endif  // TOOLS_H_
