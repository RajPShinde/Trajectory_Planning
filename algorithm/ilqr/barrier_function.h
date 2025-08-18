#pragma once

#include <Eigen/Eigen>
#include <cmath>
#include <iostream>


namespace trajectory_planning {

template<std::size_t N>
class ConstraintPenaltyFunction {
 public:
  ConstraintPenaltyFunction() = default;

  virtual ~ConstraintPenaltyFunction() = default;

  virtual double value(const double x) = 0;

  virtual Eigen::Matrix<double, N, 1> Jacbian(const double x, const Eigen::Matrix<double, N, 1>& dx) = 0;
  
  virtual Eigen::Matrix<double, N, N> Hessian(const double x, const Eigen::Matrix<double, N, 1>& dx, const Eigen::Matrix<double, N, N>& ddx = Eigen::MatrixXd::Zero(N, N)) = 0;
};

template<std::size_t N>
class BarrierFunction : public ConstraintPenaltyFunction<N> {
 public:
  BarrierFunction() {
    reciprocal_t_ = 1.0 / t_;
  }

  void SetParam(const double t) {
    t_ = t;
    reciprocal_t_ = 1.0 / t_;
  }

  double GetParam() {
    return t_;
  }

  void SetEpsilon(const double epsilon) {
    epsilon_ = epsilon;
  }


  double value(const double x) override { 
    if (x < -epsilon_) {
      return -reciprocal_t_ * std::log(-x);
    } else {
      return 0.5 * reciprocal_t_ * (std::pow((-x - 2.0 * epsilon_) / epsilon_, 2.0) - 1) - reciprocal_t_ * std::log(epsilon_);
    }
  }

  Eigen::Matrix<double, N, 1> Jacbian(const double x, const Eigen::Matrix<double, N, 1>& dx) override {
    if (x < -epsilon_) {
      return - reciprocal_t_ / x * dx;
    } else {
      return reciprocal_t_ * (x + 2.0 * epsilon_) / epsilon_ / epsilon_ * dx;
    }
  }
  
  Eigen::Matrix<double, N, N> Hessian(const double x, const Eigen::Matrix<double, N, 1>& dx, const Eigen::Matrix<double, N, N>& ddx = Eigen::MatrixXd::Zero(N, N)) override {
    if (x < -epsilon_) {
      return reciprocal_t_ / x / x * dx * dx.transpose() - reciprocal_t_ / x * ddx;
    } else {
      return reciprocal_t_ * (x + 2.0 * epsilon_) / epsilon_ / epsilon_ * dx * dx.transpose();
    }
  }

 private:
  double k_ = 2.0;
  double t_ = 5.0;
  double epsilon_ = 0.01;
  double reciprocal_t_ = 0.0;
};

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