#pragma once

#include <Eigen/Eigen>
#include <cmath>
#include <iostream>


namespace trajectory_planning {

template<std::size_t N>
class AugmentedLagrangian {
 public:
  AugmentedLagrangian() = default;

  ~AugmentedLagrangian() = default;

  double value(const double constraint_value, double lambda = 0, double rho = 1) {
    double combined_term = lambda + rho * constraint_value;
    return 0.5 * std::pow(std::max(0.0, combined_term), 2.0) / rho; //  - 0.5 * lambda * lambda / rho;
  }

  Eigen::Matrix<double, N, 1> Jacbian(const double constraint_value, const Eigen::Matrix<double, N, 1>& constraint_jac, double lambda = 0, double rho = 1) {
    double combined_term = lambda + rho * constraint_value;
    if (combined_term <= 0.0) {
      return Eigen::Matrix<double, N, 1>::Zero();
    }
    return combined_term * constraint_jac;
  }

  Eigen::Matrix<double, N, N> Hessian(const double constraint_value, const Eigen::Matrix<double, N, 1>& constraint_jac, const Eigen::Matrix<double, N, N>& constraint_hess = Eigen::MatrixXd::Zero(N, N), double lambda = 0, double rho = 1) {
    double combined_term = lambda + rho * constraint_value;
    if (combined_term <= 0.0) {
      return Eigen::Matrix<double, N, N>::Zero();
    }
    return combined_term * constraint_hess + rho * constraint_jac * constraint_jac.transpose();
  }
};

} // namespace trajectory_planning