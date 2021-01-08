#include "tools.h"
#include "gtest/gtest.h"

#include <iostream>

namespace
{
TEST(Tools, CalculateJacobian)
{
    Eigen::VectorXd x_predicted(4);
    x_predicted << 1, 2, 0.2, 0.4;
    Eigen::MatrixXd expected(3, 4);
    // clang-format off
    expected << 0.447214, 0.894427, 0, 0,
                -0.4, 0.2, 0, 0,
                0, 0, 0.447214, 0.894427;
    // clang-format on

    Eigen::MatrixXd Hj = Tools::CalculateJacobian(x_predicted);
    for (auto i{0u}; i < 3; ++i)
    {
        for (auto j{0u}; j < 4; ++j)
        {
            EXPECT_NEAR(Hj(i, j), expected(i, j), 0.001);
        }
    }
}
}
