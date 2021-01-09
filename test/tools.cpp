#include "tools.h"

#include <iostream>

#include "gtest/gtest.h"

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

    EXPECT_TRUE(expected.isApprox(Hj, 1e-4));
}

TEST(Tools, CalculateRMSE)
{
    std::vector<Eigen::VectorXd> estimations;
    std::vector<Eigen::VectorXd> ground_truth;

    Eigen::VectorXd e(4);
    e << 1, 1, 0.2, 0.1;
    estimations.push_back(e);
    e << 2, 2, 0.3, 0.2;
    estimations.push_back(e);
    e << 3, 3, 0.4, 0.3;
    estimations.push_back(e);

    Eigen::VectorXd g(4);
    g << 1.1, 1.1, 0.3, 0.2;
    ground_truth.push_back(g);
    g << 2.1, 2.1, 0.4, 0.3;
    ground_truth.push_back(g);
    g << 3.1, 3.1, 0.5, 0.4;
    ground_truth.push_back(g);

    Eigen::VectorXd expected(4);
    expected << 0.1, 0.1, 0.1, 0.1;

    const auto rmse{Tools::CalculateRMSE(estimations, ground_truth)};

    EXPECT_TRUE(expected.isApprox(rmse, 1e-4));
}

TEST(Tools, ToCartesian)
{
    const static float pi{std::acos(-1)};
    const static auto rho{2.f};
    const static auto theta{pi / 4};
    const static auto rho_dot{4.f};
    Eigen::VectorXd polar(3);
    polar << rho, theta, rho_dot;
    Eigen::VectorXd expected(4);
    expected << rho * std::cos(theta), rho * std::sin(theta), rho_dot * std::cos(theta),
        rho_dot * std::sin(theta);

    const auto cartesian{Tools::ToCartesian(polar)};

    EXPECT_TRUE(expected.isApprox(cartesian, 1e-4));
}
}  // namespace
