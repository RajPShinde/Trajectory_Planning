#pragma once

#include <Eigen/Eigen>
#include <cmath>
#include <iostream>


namespace trajectory_planning {

template<std::size_t N>
class BarrierFunction{
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


  double value(const double x)  { 
    if (x < -epsilon_) {
      return -reciprocal_t_ * std::log(-x);
    } else {
      return 0.5 * reciprocal_t_ * (std::pow((-x - 2.0 * epsilon_) / epsilon_, 2.0) - 1) - reciprocal_t_ * std::log(epsilon_);
    }
  }

  Eigen::Matrix<double, N, 1> Jacbian(const double x, const Eigen::Matrix<double, N, 1>& dx)  {
    if (x < -epsilon_) {
      return - reciprocal_t_ / x * dx;
    } else {
      return reciprocal_t_ * (x + 2.0 * epsilon_) / epsilon_ / epsilon_ * dx;
    }
  }
  
  Eigen::Matrix<double, N, N> Hessian(const double x, const Eigen::Matrix<double, N, 1>& dx, const Eigen::Matrix<double, N, N>& ddx = Eigen::MatrixXd::Zero(N, N))  {
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

} // namespace trajectory_planning