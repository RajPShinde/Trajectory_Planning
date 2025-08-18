
#include "algorithm/ilqr/ilqr_optimizer.h"

#include <cmath>
#include <limits>
#include <ros/ros.h>

#include "algorithm/math/math_utils.h"
#include "algorithm/utils/timer.h"

namespace trajectory_planning {

IlqrOptimizer::IlqrOptimizer(const IlqrConfig& config, const VehicleParam& param, const double horizon, const double dt) : config_(config),
                                                                                                                           vehicle_param_(param), 
                                                                                                                           horizon_(horizon), 
                                                                                                                           delta_t_(dt) {
  // Initialize
  vehicle_model_ = VehicleModel(config_, vehicle_param_, horizon_, delta_t_);
  num_of_knots_ = std::floor(horizon_ / delta_t_ + 1);
  State goal = Eigen::MatrixXd::Zero(kStateNum, 1);
  goals_.resize(num_of_knots_, goal);

  state_barrier_ = std::make_unique<AugmentedLagrangian<kStateNum>>();
  control_barrier_ = std::make_unique<AugmentedLagrangian<kControlNum>>();

  CalculateDiscRadius();
  cost_.clear();

  As.resize(num_of_knots_ - 1);
  Bs.resize(num_of_knots_ - 1);

  cost_Jx.resize(num_of_knots_);
  cost_Ju.resize(num_of_knots_ - 1);
  cost_Hx.resize(num_of_knots_);
  cost_Hu.resize(num_of_knots_ - 1);
}
  
bool IlqrOptimizer::Plan(
    const TrajectoryPoint& start_state,
    const DiscretizedTrajectory& coarse_traj,
    const CorridorConstraints& corridor,
    const LaneConstraints& left_lane_cons,
    const LaneConstraints& right_lane_cons,
    DiscretizedTrajectory* const opt_trajectory,
    std::vector<DiscretizedTrajectory>* const iter_trajs) {
  start_state_ = start_state;
  cost_.clear();
  
  if (opt_trajectory == nullptr || iter_trajs == nullptr) {
    return false;
  }

  if (corridor.size() == 0 ||
      left_lane_cons.size() == 0 || 
      right_lane_cons.size() == 0) {
    LOG_ERROR("ilqr input constraints error");
    return false;
  }

  if (num_of_knots_ != coarse_traj.trajectory().size()) {
    LOG_ERROR("ilqr input coarse_traj error");
    return false;
  }

  LOG_INFO_STREAM("no_of_knots_: " << num_of_knots_);

  utils::time ilqr_start_time = utils::Time();
  TransformGoals(coarse_traj);

  Optimize(start_state, coarse_traj, corridor, left_lane_cons, right_lane_cons, opt_trajectory, iter_trajs);

  utils::time ilqr_end_time = utils::Time();
  double ilqr_time_cost = utils::Duration(ilqr_start_time, ilqr_end_time);
  LOG_INFO_STREAM("ilqr time cost: " << ilqr_time_cost);
}

void IlqrOptimizer::CalculateDiscRadius() {
  int num_of_disc = config_.num_of_disc;
  double length = vehicle_param_.front_hang_length + vehicle_param_.wheel_base + vehicle_param_.rear_hang_length;
  double width = vehicle_param_.width;

  disc_radius_ = std::hypot(width / 2.0, length / 2.0 / num_of_disc);
}

void IlqrOptimizer::TransformGoals(const DiscretizedTrajectory& coarse_traj) {
  State goal;
  goal << 0.0, 0.0, 0.0, 0.0 ,0.0, 0.0;
  goals_.resize(num_of_knots_, goal);
  int i = 0;
  for (const auto& pt : coarse_traj.trajectory()) {
    goals_[i] << pt.x, pt.y, pt.theta, pt.velocity, pt.a, pt.delta;
    ++i;
  }
  goals_[0] << start_state_.x, start_state_.y, start_state_.theta, start_state_.velocity, 0.0, 0.0;
}

ilqr::SolverStatus IlqrOptimizer::Optimize(const TrajectoryPoint& start_state,
                             const DiscretizedTrajectory& coarse_traj,
                             const CorridorConstraints& corridor,
                             const LaneConstraints& left_lane_cons,
                             const LaneConstraints& right_lane_cons,
                             DiscretizedTrajectory* const opt_trajectory,
                             std::vector<DiscretizedTrajectory>* const iter_trajs) {

  ilqr::SolverStatus status = ilqr::SolverStatus::kUnsolved;

  ShrinkConstraints(corridor, left_lane_cons, right_lane_cons);
  NormalizeHalfPlane();

  // Augmented Lagrangian parameters
  int al_size = 0;
  state_constraints_index_ = al_size;
  al_size += 6 * num_of_knots_;
  control_constraints_index_ = al_size;
  al_size += 4 * (num_of_knots_ - 1);
  lane_constraints_index_ = al_size;
  al_size += 2 * num_of_knots_ * config_.num_of_disc;
  corridor_constraints_index_ = al_size;
  for(int p = 0; p < num_of_knots_; p++){
    Constraints cons = shrinked_corridor_[p];
    al_size += cons.size() * config_.num_of_disc;
  }
  LOG_INFO_STREAM("AL param size " << al_size);

  al_lambda_ = std::vector<double>(al_size, 0);
  rho_ = std::vector<double>(al_size, 1);
  double rho_factor = 1.1;
  double rho_max = 1e6;
  double constraint_tolerance = 1e-2;

  std::vector<State> states(num_of_knots_);
  std::vector<Control> controls(num_of_knots_ - 1);

  // Initial Guess
  iqr(coarse_traj, &states, &controls);
  iter_trajs->emplace_back(TransformToTrajectory(states, controls));

  // Initial Cost Calculation
  Cost cost_data;
  double cost_old = TotalCost(states, controls, &cost_data);
  cost_.push_back(cost_data);
  LOG_INFO_STREAM("init cost: " << cost_old);

  // Check Initial Guess
  if(std::isnan(cost_old) || cost_old > 1e+4){
    LOG_ERROR("LQR Initial Guess Invalid");
    return status;
  }

  // Initializing variables for iLQR
  std::vector<Eigen::Matrix<double, kControlNum, kStateNum>> Ks(num_of_knots_ - 1);
  std::vector<Eigen::Matrix<double, kControlNum, 1>> ks(num_of_knots_ - 1);
  std::vector<Eigen::Matrix<double, kControlNum, 1>> Qus;
  std::vector<Eigen::Matrix<double, kControlNum, kControlNum>> Quus;

  bool is_forward_pass_updated = true;

  double dcost = 0.0;
  double lambda = 1.0;
  double dlambda = 1.0;
  double z = 0.0;
  double cost_new = 0.0;

  static std::array<double, 11> alpha_list_{1.0000, 0.5012, 0.2512, 0.1259, 0.0631, 0.0316, 0.0158, 0.0079, 0.0040, 0.0020, 0.0010};
  
  // AL Loop
  for (int outer_iter = 1; outer_iter <= config_.max_outer_iter_num; ++outer_iter) {
    LOG_INFO_STREAM("AL Iteration: " << outer_iter);

    // Initial Cost Calculation
    cost_old = TotalCost(states, controls, &cost_data);
    cost_.push_back(cost_data);
    LOG_INFO_STREAM("init cost: " << cost_old);

    // iLQR Loop
    for (int iter = 1; iter <= config_.max_iter_num; ++iter) {
      LOG_INFO_STREAM("iLQR Iteration: " << iter);

      if (is_forward_pass_updated) {
        // Construct Cost Jacobian and Hessian
        for (int i = 0; i < num_of_knots_ - 1; ++i) {
          vehicle_model_.DynamicsJacbian(states[i], controls[i], &(As[i]), &(Bs[i]));
          
          CostJacbian(i, states[i], controls[i], &(cost_Jx[i]), &(cost_Ju[i]));
          CostHessian(i, states[i], controls[i], &(cost_Hx[i]), &(cost_Hu[i]));
        }
      
        Eigen::Matrix<double, kControlNum, 1> temp_Ju = Eigen::MatrixXd::Zero(kControlNum, 1);
        Eigen::Matrix<double, kControlNum, kControlNum> temp_Hu = Eigen::MatrixXd::Zero(kControlNum, kControlNum);
      
        CostJacbian(num_of_knots_ - 1, states.back(), {0.0, 0.0}, &(cost_Jx.back()), &temp_Ju);
        CostHessian(num_of_knots_ - 1, states.back(), {0.0, 0.0}, &(cost_Hx.back()), &temp_Hu);
        is_forward_pass_updated = false;
      }

      // Backward Pass
      bool is_backward_pass_done = false;
      while (!is_backward_pass_done) {
        bool is_diverge = Backward(lambda, states, controls, &Ks, &ks, &Qus, &Quus);
        
        if (is_diverge) {
          // Increase regularization
          dlambda = std::fmax(dlambda * options_.regularization_ratio, options_.regularization_ratio);
          lambda = std::fmax(lambda * dlambda, options_.regularization_min);
          
          if (lambda > options_.regularization_max) {
            LOG_ERROR("Backward Pass: Ilqr Solver Failed: lambda > options_.regularization_max!");
            TotalCost(states, controls, true);
            *opt_trajectory = TransformToTrajectory(states, controls);
            status = ilqr::SolverStatus::kUnsolved;
            return status;
          } 
          else {
            continue;
          }
        }
        LOG_INFO_STREAM("Backward Pass Done");
        is_backward_pass_done = true;
      }

      // Terminal Condition 2 [The feedforward gains go to zero]
      double gnorm = CalGradientNorm(ks, controls);
      if (gnorm < options_.gradient_norm_min && lambda < 1e-5) {
        *opt_trajectory = TransformToTrajectory(states, controls);
        
        LOG_SUCCESS("Ilqr Solver kSuccess. [gnorm < options_.gradient_norm_min]");
        status = ilqr::SolverStatus::kSuccess;
        break;
      }

      // Forward Pass
      bool is_forward_pass_done = false;
      if (is_backward_pass_done) {
        std::vector<State> old_state = states;
        std::vector<Control> old_controls = controls;

        // Line Search
        for (int i = 0; i < alpha_list_.size(); ++i) {

          double alpha = alpha_list_[i];
          
          Forward(alpha, &states, &controls, Ks, ks, Qus, Quus);
          
          cost_new = TotalCost(states, controls, &cost_data);
          dcost = cost_old - cost_new;
          double expected = -alpha * (delta_V_[0] + alpha * delta_V_[1]);
          
          z = dcost / expected;
          if ((z > options_.beta_min && z < options_.beta_max) && dcost > 0) {
            is_forward_pass_done = true;
            break;
          }

          states = old_state;
          controls = old_controls;
        }
      }

      if (is_forward_pass_done) {
        // Decrease Regularization
        dlambda = std::fmin(dlambda / options_.regularization_ratio, 1.0 / options_.regularization_ratio);
        lambda = lambda * dlambda * (lambda > options_.regularization_min);

        is_forward_pass_updated = true;
        LOG_INFO_STREAM("Forward Pass Done");
        LOG_INFO_STREAM("cost_old: " << cost_old << " cost_new: " << cost_new << " dcost: " << dcost);
        
        // Terminal Condition 1 [The cost decrease between iterations Jprev − J is less than some intermediate tolerance]
        if (dcost < config_.abs_cost_tol || dcost / cost_old < config_.rel_cost_tol) {
          cost_.push_back(cost_data);
          cost_old = cost_new;
          
          *opt_trajectory = TransformToTrajectory(states, controls);

          if (dcost < config_.abs_cost_tol)
            LOG_SUCCESS("Ilqr Solver kSuccess. [dcost < config_.rel_cost_tol]");
          else
            LOG_SUCCESS("Ilqr Solver kSuccess. [dcost < config_.abs_cost_tol]");
        
          status = ilqr::SolverStatus::kSuccess;
          break;
        }

        iter_trajs->emplace_back(TransformToTrajectory(states, controls));
        cost_old = cost_new;
        cost_.push_back(cost_data);
      } 
      else {
        // Increase Regularization
        dlambda = std::fmax(dlambda * options_.regularization_ratio, options_.regularization_ratio);
        lambda = std::fmax(lambda * dlambda, options_.regularization_min);

        LOG_WARN_STREAM("Forward Pass Failed");
        LOG_WARN_STREAM("cost_old: " << cost_old << " cost_new: " << cost_new << " dcost: " << dcost);

        if (lambda > options_.regularization_max) {
          *opt_trajectory = TransformToTrajectory(states, controls);
          LOG_FAILURE("Ilqr Solver kUnsolved.");
          status = ilqr::SolverStatus::kUnsolved;
          break;
        }
      }

      // Terminal Condition 3 [Solver hits a maximum number of iterations]
      if (iter == config_.max_iter_num) {
        LOG_WARN("Ilqr Solver Reach Max Iter.");
        *opt_trajectory = TransformToTrajectory(states, controls);
        status = ilqr::SolverStatus::kMaxIterations;
      }
    }
    // End of iLQR Loop

    std::vector<std::pair<ilqr::ConstraintType, double>> h_values = GetAllConstraintViolations(states, controls);
    h_values_ = h_values;
    double max_violation = CalculateMaxConstraintViolation(h_values);
    if(max_violation < constraint_tolerance){
      *opt_trajectory = TransformToTrajectory(states, controls);
      LOG_SUCCESS("AL-Ilqr Solver kSuccess. [max_violation < constraint_tolerance]");
      TotalCost(states, controls, true);
      status = ilqr::SolverStatus::kSuccess;
      break;
    }
    else{
      // Update the Augmented Lagrangian lambda and rho
      for(int v = 0; v < h_values.size(); v++){
        if(h_values[v].first == ilqr::ConstraintType::kInEquality){
          al_lambda_[v] = std::fmax(0.0, al_lambda_[v] + rho_[v] * h_values[v].second);
          if (rho_[v] < rho_max) {
            rho_[v] *= rho_factor;
          }
        }
        else{
          // TO-DO: Add update rule for Equality Constraints.
        }
      }
    }

    if(outer_iter == config_.max_outer_iter_num){
      LOG_WARN("AL-Ilqr Solver Reach Max Iter.");
      *opt_trajectory = TransformToTrajectory(states, controls);
      TotalCost(states, controls, true);
      status = ilqr::SolverStatus::kMaxIterations;
    }
  }
  // End of AL-iLQR Loop

  return status;
}

std::vector<std::pair<ilqr::ConstraintType, double>> IlqrOptimizer::GetAllConstraintViolations(const std::vector<State>& states, const std::vector<Control>& controls) {
  std::vector<std::pair<ilqr::ConstraintType, double>> h_values;

  // Compute Violation h(x)
  // if h(x) > 0, then the constraint is violated
  // if h(x) <= 0, then the constraint is satisfied
  // all in equality constraints are transformed to the form g(x) <= 0

  // State Constraints
  for (int i = 0; i < num_of_knots_; i++) {
    // Vmin - V <= 0
    // V - Vmax >= 0
    // a_min - a <= 0
    // a - a_max <= 0
    // delta_min - delta <= 0
    // delta - delta_max <= 0
    h_values.push_back(std::make_pair(ilqr::ConstraintType::kInEquality, vehicle_param_.min_velocity -states[i](3, 0)));
    h_values.push_back(std::make_pair(ilqr::ConstraintType::kInEquality, states[i](3, 0) - vehicle_param_.max_velocity));
    h_values.push_back(std::make_pair(ilqr::ConstraintType::kInEquality, vehicle_param_.min_acceleration - states[i](4, 0)));
    h_values.push_back(std::make_pair(ilqr::ConstraintType::kInEquality, states[i](4, 0) - vehicle_param_.max_acceleration));
    h_values.push_back(std::make_pair(ilqr::ConstraintType::kInEquality, vehicle_param_.delta_min - states[i](5, 0)));
    h_values.push_back(std::make_pair(ilqr::ConstraintType::kInEquality, states[i](5, 0) - vehicle_param_.delta_max));
  } 

  // Control Constraints
  for (int i = 0; i < num_of_knots_ - 1; i++) {
    // jerk_min - jerk <= 0
    // jerk - jerk_max <= 0
    // delta_rate_min - delta_rate <= 0
    // delta_rate - delta_rate_max <= 0
    h_values.push_back(std::make_pair(ilqr::ConstraintType::kInEquality, vehicle_param_.jerk_min - controls[i](0, 0)));
    h_values.push_back(std::make_pair(ilqr::ConstraintType::kInEquality, controls[i](0, 0) - vehicle_param_.jerk_max));
    h_values.push_back(std::make_pair(ilqr::ConstraintType::kInEquality, vehicle_param_.delta_rate_min - controls[i](1, 0)));
    h_values.push_back(std::make_pair(ilqr::ConstraintType::kInEquality, controls[i](1, 0) - vehicle_param_.delta_rate_max));
  }

  double L = (vehicle_param_.rear_hang_length + vehicle_param_.wheel_base + vehicle_param_.front_hang_length) / config_.num_of_disc;
  double rf = vehicle_param_.rear_hang_length;
  
  // Lane Constraints
  for (int i = 0; i < num_of_knots_; ++i) {

    for (int j = 0; j < config_.num_of_disc; j++) {
      double x = states[i](0, 0) + (L * (j - 0.5) - rf) * std::cos(states[i](2, 0));
      double y = states[i](1, 0) + (L * (j - 0.5) - rf) * std::sin(states[i](2, 0));
      
      // Left Lane Violation
      auto cons_left = FindNeastLaneSegment(x, y, shrinked_left_lane_cons_);
      // Ax + By - C <= 0
      double left_h_val = cons_left[0] * x + cons_left[1] * y - cons_left[2];
      h_values.push_back(std::make_pair(ilqr::ConstraintType::kInEquality, left_h_val));
      
      // Right Lane Violation
      auto cons_right = FindNeastLaneSegment(x, y, shrinked_right_lane_cons_);
      // Ax + By - C <= 0
      double right_h_val = cons_right[0] * x + cons_right[1] * y - cons_right[2];
      h_values.push_back(std::make_pair(ilqr::ConstraintType::kInEquality, right_h_val));
    }
  }

  // Corridor Constraints
  for (int i = 0; i < num_of_knots_; i++) {
    Constraints cons = shrinked_corridor_[i];

    for (int j = 0; j < config_.num_of_disc; j++) {
      double x = states[i](0, 0) + (L * (j - 0.5) - rf) * std::cos(states[i](2, 0));
      double y = states[i](1, 0) + (L * (j - 0.5) - rf) * std::sin(states[i](2, 0));

      // Corridor Violation
      for (const auto& c : cons) {
        double corridor_h_val = c[0] * x + c[1] * y - c[2];
        // Ax + By - C <= 0
        h_values.push_back(std::make_pair(ilqr::ConstraintType::kInEquality, corridor_h_val));
      }
    }
  }

  return h_values;
}

double IlqrOptimizer::CalculateMaxConstraintViolation(std::vector<std::pair<ilqr::ConstraintType, double>> h_values){
  double max_violation = 0.0;
  int max_violation_index = 0;
  for (int i = 0; i < h_values.size(); i++) {
    double violation = std::fmax(0.0, h_values[i].second);
    if(max_violation < violation) {
        max_violation = violation;
        max_violation_index = i;
    }
  }

  if(max_violation_index < control_constraints_index_)
    LOG_WARN_STREAM("state constraint violation: " << max_violation << " at index: " << max_violation_index << " al_lambda " << al_lambda_[max_violation_index] << " rho " << rho_[max_violation_index]);
  else if(max_violation_index < lane_constraints_index_)
    LOG_WARN_STREAM("control boundary constraint violation: " << max_violation << " at index: " << max_violation_index << " al_lambda " << al_lambda_[max_violation_index] << " rho " << rho_[max_violation_index]);
  else if(max_violation_index < corridor_constraints_index_)
    LOG_WARN_STREAM("lane constraint violation: " << max_violation << " at index: " << max_violation_index << " al_lambda " << al_lambda_[max_violation_index] << " rho " << rho_[max_violation_index]);
  else
    LOG_WARN_STREAM("corridor constraint violation: " << max_violation << " at index: " << max_violation_index << " al_lambda " << al_lambda_[max_violation_index] << " rho " << rho_[max_violation_index]);

  return max_violation;
}


double IlqrOptimizer::CalGradientNorm(const std::vector<Eigen::Matrix<double, kControlNum, 1>>& ks, const std::vector<Control>& controls) {
  std::vector<double> vals(ks.size());
  Eigen::VectorXd v;
  for (int i = 0; i < vals.size(); ++i) {
    v = ks[i].cwiseAbs().array() / (controls[i].cwiseAbs().array() + 1);
    vals[i] = v.maxCoeff();
  }
  return std::accumulate(vals.begin(), vals.end(), 0.0) / vals.size();
}


bool IlqrOptimizer::Backward(
    const double lambda,
    const std::vector<State>& states,
    const std::vector<Control>& controls,
    std::vector<Eigen::Matrix<double, kControlNum, kStateNum>>* const Ks,
    std::vector<Eigen::Matrix<double, kControlNum, 1>>* const ks,
    std::vector<Eigen::Matrix<double, kControlNum, 1>>* const Qus,
    std::vector<Eigen::Matrix<double, kControlNum, kControlNum>>* const Quus) {
  delta_V_[0] = 0.0; delta_V_[1] = 0.0;
  Eigen::Matrix<double, kStateNum, 1> Vx = cost_Jx.back();
  Eigen::Matrix<double, kStateNum, kStateNum> Vxx = cost_Hx.back();
  Qus->clear();
  Quus->clear();
  for (int i = num_of_knots_ - 2; i >=0; --i) {
    auto Qx = cost_Jx[i] + As[i].transpose() * Vx;
    auto Qu = cost_Ju[i] + Bs[i].transpose() * Vx;

    auto Qxx = cost_Hx[i] + As[i].transpose() * Vxx * As[i];
    auto Quu = cost_Hu[i] + Bs[i].transpose() * Vxx * Bs[i];
    auto Qux = Bs[i].transpose() * Vxx * As[i]; 

    // Regularize
    auto Quu_tem = Quu + lambda * Eigen::MatrixXd::Identity(kControlNum, kControlNum);

    auto Quu_sym = 0.5 * (Quu_tem + Quu_tem.transpose());
    Eigen::LLT<Eigen::MatrixXd> Quu_fact(Quu_tem);
    if (Quu_fact.info() != Eigen::Success) {
      LOG_WARN("Backward Pass: LLT decomposition failed in iqr, matrix is not positive definite.");
      return true;
    }

    Ks->at(i) = -Qux;
    ks->at(i) = -Qu;
    Quu_fact.solveInPlace(Ks->at(i));
    Quu_fact.solveInPlace(ks->at(i));

    // auto Quu_tem = Quu + lambda * Eigen::MatrixXd::Identity(kControlNum, kControlNum);

    // auto Quu_inv = Quu_tem.inverse();

    // Ks->at(i) = -Quu_inv * Qux;
    // ks->at(i) = -Quu_inv * Qu;

    Vx = Qx + Ks->at(i).transpose() * Quu * ks->at(i) + Ks->at(i).transpose() * Qu + Qux.transpose() * ks->at(i);
    Vxx = Qxx + Ks->at(i).transpose() * Quu * Ks->at(i) + Ks->at(i).transpose() * Qux + Qux.transpose() * Ks->at(i);
    Vxx = 0.5 * (Vxx + Vxx.transpose());

    delta_V_[0] += ks->at(i).transpose() * Qu; 
    delta_V_[1] += 0.5 * ks->at(i).transpose() * Quu * ks->at(i);

    Qus->push_back(Qu);
    Quus->push_back(Quu);
  }
  return false;
}


void IlqrOptimizer::Forward(
    const double alpha,
    std::vector<State>* const states,
    std::vector<Control>* const controls,
    const std::vector<Eigen::Matrix<double, kControlNum, kStateNum>>& Ks,
    const std::vector<Eigen::Matrix<double, kControlNum, 1>>& ks,
    const std::vector<Eigen::Matrix<double, kControlNum, 1>>& Qus,
    const std::vector<Eigen::Matrix<double, kControlNum, kControlNum>>& Quus) {
  
  std::vector<State> new_state = *states;
  std::vector<Control> new_controls = *controls;

  State x = goals_.front();
  new_state.front() = x;
  for (int i = 0; i < num_of_knots_ - 1; ++i) {
    new_controls[i] = new_controls[i] + Ks[i] * (x - states->at(i)) + alpha * ks[i];
    new_controls[i](1, 0) = math::NormalizeAngle(new_controls[i](1, 0));
    vehicle_model_.Dynamics(x, new_controls[i], &x);
    new_state.at(i + 1) = x;
  }

  *controls = new_controls;
  *states = new_state;
}

double IlqrOptimizer::TotalCost(const std::vector<State>& states, const std::vector<Control>& controls, Cost* const cost) {

  double j_cost = JCost(states, controls);
  double dynamics_cost = DynamicsCost(states, controls);
  double corridor_cost = CorridorCost(states);
  double lane_boundary_cost = LaneBoundaryCost(states);
  double total_cost = j_cost + dynamics_cost + corridor_cost + lane_boundary_cost;

  *cost = Cost(total_cost, j_cost, dynamics_cost, corridor_cost, lane_boundary_cost);
  
  return total_cost;
}

double IlqrOptimizer::TotalCost(const std::vector<State>& states, const std::vector<Control>& controls, bool print) {

  double j_cost = JCost(states, controls);
  double dynamics_cost = DynamicsCost(states, controls);
  double corridor_cost = CorridorCost(states);
  double lane_boundary_cost = LaneBoundaryCost(states);
  double total_cost = j_cost + dynamics_cost + corridor_cost + lane_boundary_cost;

  if(print){
    LOG_INFO_STREAM("###########################################");
    LOG_INFO_STREAM("################# Summary #################");
    LOG_INFO_STREAM("###########################################");
    LOG_INFO_STREAM("Quadratic Cost: " << j_cost);
    LOG_INFO_STREAM("AL-Dynamics Cost: " << dynamics_cost);
    LOG_INFO_STREAM("AL-Corridor Cost: " << corridor_cost);
    LOG_INFO_STREAM("AL-Lane Cost: " << lane_boundary_cost);
    LOG_INFO_STREAM("AL-Total Cost: " << total_cost);

    double s3_max = std::numeric_limits<double>::lowest();
    double s4_max = std::numeric_limits<double>::lowest();
    double s5_max = std::numeric_limits<double>::lowest();
    double c0_max = std::numeric_limits<double>::lowest();
    double c1_max = std::numeric_limits<double>::lowest();

    double s3_min = std::numeric_limits<double>::max();
    double s4_min = std::numeric_limits<double>::max();
    double s5_min = std::numeric_limits<double>::max();
    double c0_min = std::numeric_limits<double>::max();
    double c1_min = std::numeric_limits<double>::max();
    
    for (int i = 0; i < num_of_knots_; ++i) {
      s3_max = std::max(s3_max, states[i](3, 0));
      s4_max = std::max(s4_max, states[i](4, 0));
      s5_max = std::max(s5_max, states[i](5, 0));

      s3_min = std::min(s3_min, states[i](3, 0));
      s4_min = std::min(s4_min, states[i](4, 0));
      s5_min = std::min(s5_min, states[i](5, 0));

      if(i < num_of_knots_ - 1) {
        c0_max = std::max(c0_max, controls[i](0, 0));
        c1_max = std::max(c1_max, controls[i](1, 0));
        
        c0_min = std::min(c0_min, controls[i](0, 0));
        c1_min = std::min(c1_min, controls[i](1, 0));
      }
    }

    if(s3_max >= vehicle_param_.max_velocity)
      LOG_INFO_STREAM("Violation on Constraint, State v:     " << s3_max << " [Max velocity: " << vehicle_param_.max_velocity << "]");
    if(s4_max >= vehicle_param_.max_acceleration)
      LOG_INFO_STREAM("Violation on Constraint, State a:     " << s4_max << " [Max acceleration: " << vehicle_param_.max_acceleration << "]");
    if(s5_max >= vehicle_param_.delta_max)
      LOG_INFO_STREAM("Violation on Constraint, State delta: " << s5_max << " [Max delta: " << vehicle_param_.delta_max << "]");
    if(s3_min <= vehicle_param_.min_velocity)
      LOG_INFO_STREAM("Violation on Constraint, State v:     " << s3_min << " [Min velocity: " << vehicle_param_.min_velocity << "]");
    if(s4_min <= vehicle_param_.min_acceleration)
      LOG_INFO_STREAM("Violation on Constraint, State a:     " << s4_min << " [Min acceleration: " << vehicle_param_.min_acceleration << "]");
    if(s5_min <= vehicle_param_.delta_min)
      LOG_INFO_STREAM("Violation on Constraint, State delta: " << s5_min << " [Min delta: " << vehicle_param_.delta_min << "]");

    if(c0_max >= vehicle_param_.jerk_max)
      LOG_INFO_STREAM("Violation on Constraint, Control c0:  " << c0_max << " [Max jerk: " << vehicle_param_.jerk_max << "]");
    if(c1_max >= vehicle_param_.delta_rate_max)
      LOG_INFO_STREAM("Violation on Constraint, Control c1:  " << c1_max << " [Max delta rate: " << vehicle_param_.delta_rate_max << "]");
    if(c0_min <= vehicle_param_.jerk_min)
      LOG_INFO_STREAM("Violation on Constraint, Control c0:  " << c0_min << " [Min jerk: " << vehicle_param_.jerk_min << "]");
    if(c1_min <= vehicle_param_.delta_rate_min)
      LOG_INFO_STREAM("Violation on Constraint, Control c1:  " << c1_min << " [Min delta rate: " << vehicle_param_.delta_rate_min << "]");

    double L = (vehicle_param_.rear_hang_length + vehicle_param_.wheel_base + vehicle_param_.front_hang_length) / config_.num_of_disc;
    double rf = vehicle_param_.rear_hang_length;

      int lane = 0;
      int corridor = 0;
      for(int i = 0; i < 15; i++){
        Constraints cons = shrinked_corridor_[i];

        if(h_values_[state_constraints_index_ + i*6 + 0].second >= 0)
          LOG_INFO_STREAM("knot " << i << " h(x) vmin - v" << h_values_[state_constraints_index_ + i*6 + 0].second << " lambda " << al_lambda_[state_constraints_index_ + i*6 + 0] << " rho " << rho_[state_constraints_index_ + i*6 + 0]);
        if(h_values_[state_constraints_index_ + i*6 + 1].second >= 0)
          LOG_INFO_STREAM("knot " << i << " h(x) v - vmax" << h_values_[state_constraints_index_ + i*6 + 1].second << " lambda " << al_lambda_[state_constraints_index_ + i*6 + 1] << " rho " << rho_[state_constraints_index_ + i*6 + 1]);
        if(h_values_[state_constraints_index_ + i*6 + 2].second >= 0)
          LOG_INFO_STREAM("knot " << i << " h(x) amin - a" << h_values_[state_constraints_index_ + i*6 + 2].second << " lambda " << al_lambda_[state_constraints_index_ + i*6 + 2] << " rho " << rho_[state_constraints_index_ + i*6 + 2]);
        if(h_values_[state_constraints_index_ + i*6 + 3].second >= 0)
          LOG_INFO_STREAM("knot " << i << " h(x) a - amax" << h_values_[state_constraints_index_ + i*6 + 3].second << " lambda " << al_lambda_[state_constraints_index_ + i*6 + 3] << " rho " << rho_[state_constraints_index_ + i*6 + 3]);
        if(h_values_[state_constraints_index_ + i*6 + 4].second >= 0)
          LOG_INFO_STREAM("knot " << i << " h(x) dmin - d" << h_values_[state_constraints_index_ + i*6 + 4].second << " lambda " << al_lambda_[state_constraints_index_ + i*6 + 4] << " rho " << rho_[state_constraints_index_ + i*6 + 4]);
        if(h_values_[state_constraints_index_ + i*6 + 5].second >= 0)
          LOG_INFO_STREAM("knot " << i << " h(x) d - dmax" << h_values_[state_constraints_index_ + i*6 + 5].second << " lambda " << al_lambda_[state_constraints_index_ + i*6 + 5] << " rho " << rho_[state_constraints_index_ + i*6 + 5]);


        if(h_values_[control_constraints_index_ + i*4 + 0].second >= 0)
          LOG_INFO_STREAM("knot " << i << " h(x) jmin - j " << h_values_[control_constraints_index_ + i*4 + 0].second << " lambda " << al_lambda_[control_constraints_index_ + i*4 + 0] << " rho " << rho_[control_constraints_index_ + i*4 + 0]);
        if(h_values_[control_constraints_index_ + i*4 + 1].second >= 0)
          LOG_INFO_STREAM("knot " << i << " h(x) j - jmax " << h_values_[control_constraints_index_ + i*4 + 1].second << " lambda " << al_lambda_[control_constraints_index_ + i*4 + 1] << " rho " << rho_[control_constraints_index_ + i*4 + 1]);
        if(h_values_[control_constraints_index_ + i*4 + 2].second >= 0)
          LOG_INFO_STREAM("knot " << i << " h(x) drmin - dr " << h_values_[control_constraints_index_ + i*4 + 2].second << " lambda " << al_lambda_[control_constraints_index_ + i*4 + 2] << " rho " << rho_[control_constraints_index_ + i*4 + 2]);
        if(h_values_[control_constraints_index_ + i*4 + 3].second >= 0)
          LOG_INFO_STREAM("knot " << i << " h(x) dr - drmax " << h_values_[control_constraints_index_ + i*4 + 3].second << " lambda " << al_lambda_[control_constraints_index_ + i*4 + 3] << " rho " << rho_[control_constraints_index_ + i*4 + 3]);

        for (int j = 0; j < config_.num_of_disc; j++) {
          
          // Left Lane Violation
          if(h_values_[lane_constraints_index_ + lane].second >= 0)
            LOG_INFO_STREAM("knot " << i << " disc " << j << " h(x) left lane" << h_values_[lane_constraints_index_ + lane].second << " lambda " << al_lambda_[lane_constraints_index_ + lane] << " rho " << rho_[lane_constraints_index_ + lane]);
          lane++;

          // Right Lane Violation
          if(h_values_[lane_constraints_index_ + lane].second >= 0) 
            LOG_INFO_STREAM("knot " << i << " disc " << j << " h(x) right lane" << h_values_[lane_constraints_index_ + lane].second << " lambda " << al_lambda_[lane_constraints_index_ + lane] << " rho " << rho_[lane_constraints_index_ + lane]);
          lane++;


          // Corridor Violation
          for (const auto& c : cons) {
            if(h_values_[corridor_constraints_index_ + corridor].second >= 0)
              LOG_INFO_STREAM("knot " << i << ", disc " << j << ", corridor " << corridor << ", h(x) corridor " << h_values_[corridor_constraints_index_ + corridor].second << ", lambda " << al_lambda_[corridor_constraints_index_ + corridor] << ", rho " << rho_[corridor_constraints_index_ + corridor]);
            corridor++;
          }
        }
      }
    }

  return total_cost;
}

void IlqrOptimizer::ShrinkConstraints(const CorridorConstraints& corridor, const LaneConstraints& left_lane_cons, const LaneConstraints& right_lane_cons) {
  shrinked_corridor_.resize(corridor.size());
  for (int i = 0; i < corridor.size(); ++i) {
    shrinked_corridor_[i] = corridor[i];
    for (auto& e : shrinked_corridor_[i]) {
      double cccc = e[2];
      e[2] = e[2] - (disc_radius_ + config_.safe_margin) * (e[0] * e[0] + e[1] * e[1]) / std::hypot(e[0], e[1]);
    }
  }

  shrinked_left_lane_cons_.resize(left_lane_cons.size());
  for (int i = 0; i < left_lane_cons.size(); ++i) {
    shrinked_left_lane_cons_[i] = left_lane_cons[i];
    auto& e = shrinked_left_lane_cons_[i].first;
    double cccc = e[2];
    e[2] = e[2] - disc_radius_ * (e[0] * e[0] + e[1] * e[1]) / std::hypot(e[0], e[1]);
  }

  shrinked_right_lane_cons_.resize(right_lane_cons.size());
  for (int i = 0; i < right_lane_cons.size(); ++i) {
    shrinked_right_lane_cons_[i] = right_lane_cons[i];
    auto& e = shrinked_right_lane_cons_[i].first;
    e[2] = e[2] - disc_radius_ * (e[0] * e[0] + e[1] * e[1]) / std::hypot(e[0], e[1]);
  }
}

void IlqrOptimizer::NormalizeHalfPlane() {

  for (auto& corridor : shrinked_corridor_) {
    for (auto& hpoly : corridor) {
      double norm = std::hypot(std::hypot(hpoly[0], hpoly[1]), hpoly[2]);
      hpoly = hpoly / norm;
    }
  }

  for (auto& cons : shrinked_left_lane_cons_) {
    auto& hpoly = cons.first;
    double norm = std::hypot(std::hypot(hpoly[0], hpoly[1]), hpoly[2]);
    hpoly = hpoly / norm;
  }

  for (auto& cons : shrinked_right_lane_cons_) {
    auto& hpoly = cons.first;
    double norm = std::hypot(std::hypot(hpoly[0], hpoly[1]), hpoly[2]);
    hpoly = hpoly / norm;
  }
}

double IlqrOptimizer::JCost(const std::vector<State>& states, const std::vector<Control>& controls) {
  double cost = 0.0;
  for (int i = 0; i < num_of_knots_; ++i) {
    cost += config_.weights.x_target * std::pow((states[i](0, 0) - goals_[i](0, 0)), 2) +
            config_.weights.y_target * std::pow((states[i](1, 0) - goals_[i](1, 0)), 2) +
            config_.weights.theta * std::pow((states[i](2, 0) - goals_[i](2, 0)), 2) +
            config_.weights.v * std::pow((states[i](3, 0) - goals_[i](3, 0)), 2) +
            config_.weights.a * std::pow((states[i](4, 0) - goals_[i](4, 0)), 2) +
            config_.weights.delta * std::pow((states[i](5, 0) - goals_[i](5, 0)), 2);
  }

  for (int i = 0; i < num_of_knots_ - 1; ++i) {
    cost += config_.weights.jerk * std::pow(controls[i](0, 0), 2) +
            config_.weights.delta_rate * std::pow(controls[i](1, 0), 2);
  }

  return cost;
}

double IlqrOptimizer::DynamicsCost(const std::vector<State>& states, const std::vector<Control>& controls) {

  double x_cost = 0.0;
  for (int i = 0; i < num_of_knots_; ++i) {
    int base = i * 6;
    x_cost += state_barrier_->value(vehicle_param_.min_velocity - states[i](3, 0), al_lambda_[state_constraints_index_ + base], rho_[state_constraints_index_ + base]);
    x_cost += state_barrier_->value(states[i](3, 0) - vehicle_param_.max_velocity, al_lambda_[state_constraints_index_ + base + 1], rho_[state_constraints_index_ + base + 1]);
    x_cost += state_barrier_->value(vehicle_param_.min_acceleration - states[i](4, 0), al_lambda_[state_constraints_index_ + base + 2], rho_[state_constraints_index_ + base + 2]);
    x_cost += state_barrier_->value(states[i](4, 0) - vehicle_param_.max_acceleration, al_lambda_[state_constraints_index_ + base + 3], rho_[state_constraints_index_ + base + 3]);
    x_cost += state_barrier_->value(vehicle_param_.delta_min - states[i](5, 0), al_lambda_[state_constraints_index_ + base + 4], rho_[state_constraints_index_ + base + 4]);
    x_cost += state_barrier_->value(states[i](5, 0) - vehicle_param_.delta_max, al_lambda_[state_constraints_index_ + base + 5], rho_[state_constraints_index_ + base + 5]);
  } 

  double u_cost = 0.0;

  for (int i = 0; i < num_of_knots_ - 1; ++i) {
    int base = i * 4;
    u_cost += control_barrier_->value(vehicle_param_.jerk_min - controls[i](0, 0), al_lambda_[control_constraints_index_ + base], rho_[control_constraints_index_ + base]);
    u_cost += control_barrier_->value(controls[i](0, 0) - vehicle_param_.jerk_max, al_lambda_[control_constraints_index_ + base + 1], rho_[control_constraints_index_ + base + 1]);
    u_cost += control_barrier_->value(vehicle_param_.delta_rate_min - controls[i](1, 0), al_lambda_[control_constraints_index_ + base + 2], rho_[control_constraints_index_ + base + 2]);
    u_cost += control_barrier_->value(controls[i](1, 0) - vehicle_param_.delta_rate_max, al_lambda_[control_constraints_index_ + base + 3], rho_[control_constraints_index_ + base + 3]);
  }

  return x_cost + u_cost;
}

double IlqrOptimizer::CorridorCost(const std::vector<State>& states) {
  double cost = 0.0;
  double L = (vehicle_param_.rear_hang_length + vehicle_param_.wheel_base + vehicle_param_.front_hang_length) / config_.num_of_disc;
  double rf = vehicle_param_.rear_hang_length;

  int k = 0;
  for (int i = 0; i < num_of_knots_; ++i) {
    Constraints cons = shrinked_corridor_[i];

    for (int j = 0; j < config_.num_of_disc; ++j) {
      double x = states[i](0, 0) + (L * (j - 0.5) - rf) * std::cos(states[i](2, 0));
      double y = states[i](1, 0) + (L * (j - 0.5) - rf) * std::sin(states[i](2, 0));
      
      for (const auto& c : cons) {
        cost += state_barrier_->value(c[0] * x + c[1] * y - c[2], al_lambda_[corridor_constraints_index_ + k], rho_[corridor_constraints_index_ + k]);
        k++;
      }
    }
  }

  return cost;
}

double IlqrOptimizer::LaneBoundaryCost(const std::vector<State>& states) {
  double cost = 0.0;
  double L = (vehicle_param_.rear_hang_length + vehicle_param_.wheel_base + vehicle_param_.front_hang_length) / config_.num_of_disc;
  double rf = vehicle_param_.rear_hang_length;

  int k = 0;
  for (int i = 0; i < num_of_knots_; ++i) {
    for (int j = 0; j < config_.num_of_disc; ++j) {
      double x = states[i](0, 0) + (L * (j - 0.5) - rf) * std::cos(states[i](2, 0));
      double y = states[i](1, 0) + (L * (j - 0.5) - rf) * std::sin(states[i](2, 0));
      
      auto cons_left = FindNeastLaneSegment(x, y, shrinked_left_lane_cons_);
      cost += state_barrier_->value(cons_left[0] * x + cons_left[1] * y - cons_left[2], al_lambda_[lane_constraints_index_ + k], rho_[lane_constraints_index_ + k]);
      k++;
      auto cons_right = FindNeastLaneSegment(x, y, shrinked_right_lane_cons_);
      cost += state_barrier_->value(cons_right[0] * x + cons_right[1] * y - cons_right[2], al_lambda_[lane_constraints_index_ + k], rho_[lane_constraints_index_ + k]);
      k++;
    }
  }

  return cost;
}

Eigen::Vector3d IlqrOptimizer::FindNeastLaneSegment(const double x, const double y, 
                                                    std::vector<std::pair<Eigen::Vector3d, math::LineSegment2d>> lane_segs) {
  double min_dis = std::numeric_limits<double>::max();
  int min_index = -1;
  for (int i = 0; i < lane_segs.size(); ++i) {
    double dis = lane_segs[i].second.DistanceTo(math::Vec2d(x, y));
    if (dis < min_dis) {
      min_dis = dis;
      min_index = i;
    }
  }
  return lane_segs[min_index].first;
}

void IlqrOptimizer::CostJacbian(const int index, const State& state, const Control& control, State* const cost_Jx, Control* cost_Ju) {
  *cost_Jx << 2.0 * config_.weights.x_target * (state(0, 0) - goals_[index](0, 0)),
              2.0 * config_.weights.y_target * (state(1, 0) - goals_[index](1, 0)),
              2.0 * config_.weights.theta * (state(2, 0) - goals_[index](2, 0)),
              2.0 * config_.weights.v * (state(3, 0) - goals_[index](3, 0)),
              2.0 * config_.weights.a * (state(4, 0) - goals_[index](4, 0)),
              2.0 * config_.weights.delta * (state(5, 0) - goals_[index](5, 0));

  *cost_Ju << 2.0 * config_.weights.jerk * control(0, 0),
              2.0 * config_.weights.delta_rate * control(1, 0);
  
  DynamicsConsJacbian(index, state, control, cost_Jx, cost_Ju);
  CorridorConsJacbian(index, state, cost_Jx);
  LaneBoundaryConsJacbian(index, state, cost_Jx);
}

void IlqrOptimizer::CostHessian(const int index, const State& state, const Control& control,
                                Eigen::Matrix<double, kStateNum, kStateNum>* const cost_Hx, 
                                Eigen::Matrix<double, kControlNum, kControlNum>* const cost_Hu) {
  *cost_Hx << 2.0 * config_.weights.x_target, 0.0, 0.0, 0.0, 0.0, 0.0,
              0.0, 2.0 * config_.weights.y_target, 0.0, 0.0, 0.0, 0.0,
              0.0, 0.0, 2.0 * config_.weights.theta, 0.0, 0.0, 0.0,
              0.0, 0.0, 0.0, 2.0 * config_.weights.v, 0.0, 0.0,
              0.0, 0.0, 0.0, 0.0, 2.0 * config_.weights.a, 0.0,
              0.0, 0.0, 0.0, 0.0, 0.0, 2.0 * config_.weights.delta;

  *cost_Hu << 2.0 * config_.weights.jerk, 0.0,
              0.0, 2.0 * config_.weights.delta_rate;

  DynamicsConsHessian(index, state, control, cost_Hx, cost_Hu);
  CorridorConsHessian(index, state, cost_Hx);
  LaneBoundaryConsHessian(index, state, cost_Hx);
}

void IlqrOptimizer::DynamicsConsJacbian(const int index, const State& state, const Control& control, State* const cost_Jx, Control* cost_Ju) {
  int base = index * 6;

  *cost_Jx += state_barrier_->Jacbian(vehicle_param_.min_velocity - state(3, 0), (Eigen::Matrix<double, 6, 1>() << 0.0, 0.0, 0.0, -1.0, 0.0, 0.0).finished(), al_lambda_[state_constraints_index_ + base], rho_[state_constraints_index_ + base]) +
              state_barrier_->Jacbian(state(3, 0) - vehicle_param_.max_velocity, (Eigen::Matrix<double, 6, 1>() << 0.0, 0.0, 0.0, 1.0, 0.0, 0.0).finished(), al_lambda_[state_constraints_index_ + base + 1], rho_[state_constraints_index_ + base + 1]) +
              state_barrier_->Jacbian(vehicle_param_.min_acceleration - state(4, 0), (Eigen::Matrix<double, 6, 1>() <<0.0, 0.0, 0.0, 0.0, -1.0, 0.0).finished(), al_lambda_[state_constraints_index_ + base + 2], rho_[state_constraints_index_ + base + 2]) +
              state_barrier_->Jacbian(state(4, 0) - vehicle_param_.max_acceleration, (Eigen::Matrix<double, 6, 1>() <<0.0, 0.0, 0.0, 0.0, 1.0, 0.0).finished(), al_lambda_[state_constraints_index_ + base + 3], rho_[state_constraints_index_ + base + 3]) +
              state_barrier_->Jacbian(vehicle_param_.delta_min - state(5, 0), (Eigen::Matrix<double, 6, 1>() <<0.0, 0.0, 0.0, 0.0, 0.0, -1.0).finished(), al_lambda_[state_constraints_index_ + base + 4], rho_[state_constraints_index_ + base + 4]) +
              state_barrier_->Jacbian(state(5, 0) - vehicle_param_.delta_max, (Eigen::Matrix<double, 6, 1>() <<0.0, 0.0, 0.0, 0.0, 0.0, 1.0).finished(), al_lambda_[state_constraints_index_ + base + 5], rho_[state_constraints_index_ + base + 5]);
  
  base = index * 4;
  *cost_Ju += control_barrier_->Jacbian(vehicle_param_.jerk_min - control(0, 0), (Eigen::Matrix<double, 2, 1>() <<-1.0, 0.0).finished(), al_lambda_[control_constraints_index_ + base], rho_[control_constraints_index_ + base]) +
              control_barrier_->Jacbian(control(0, 0) - vehicle_param_.jerk_max, (Eigen::Matrix<double, 2, 1>() <<1.0, 0.0).finished(), al_lambda_[control_constraints_index_ + base + 1], rho_[control_constraints_index_ + base + 1]) +
              control_barrier_->Jacbian(vehicle_param_.delta_rate_min - control(1, 0), (Eigen::Matrix<double, 2, 1>() <<0.0, -1.0).finished(), al_lambda_[control_constraints_index_ + base + 2], rho_[control_constraints_index_ + base + 2]) +
              control_barrier_->Jacbian(control(1, 0) - vehicle_param_.delta_rate_max, (Eigen::Matrix<double, 2, 1>() <<0.0, 1.0).finished(), al_lambda_[control_constraints_index_ + base + 3], rho_[control_constraints_index_ + base + 3]);
}

void IlqrOptimizer::DynamicsConsHessian(const int index, const State& state, const Control& control,
                                        Eigen::Matrix<double, kStateNum, kStateNum>* const cost_Hx,
                                        Eigen::Matrix<double, kControlNum, kControlNum>* const cost_Hu) {
  int base = index * 6;

  *cost_Hx += state_barrier_->Hessian(vehicle_param_.min_velocity - state(3, 0), (Eigen::Matrix<double, 6, 1>() <<0.0, 0.0, 0.0, -1.0, 0.0, 0.0).finished(), Eigen::MatrixXd::Zero(6, 6), al_lambda_[state_constraints_index_ + base], rho_[state_constraints_index_ + base]) +
              state_barrier_->Hessian(state(3, 0) - vehicle_param_.max_velocity, (Eigen::Matrix<double, 6, 1>() <<0.0, 0.0, 0.0, 1.0, 0.0, 0.0).finished(), Eigen::MatrixXd::Zero(6, 6), al_lambda_[state_constraints_index_ + base + 1], rho_[state_constraints_index_ + base + 1]) +
              state_barrier_->Hessian(vehicle_param_.min_acceleration - state(4, 0), (Eigen::Matrix<double, 6, 1>() <<0.0, 0.0, 0.0, 0.0, -1.0, 0.0).finished(), Eigen::MatrixXd::Zero(6, 6), al_lambda_[state_constraints_index_ + base + 2], rho_[state_constraints_index_ + base + 2]) +
              state_barrier_->Hessian(state(4, 0) - vehicle_param_.max_acceleration, (Eigen::Matrix<double, 6, 1>() <<0.0, 0.0, 0.0, 0.0, 1.0, 0.0).finished(), Eigen::MatrixXd::Zero(6, 6), al_lambda_[state_constraints_index_ + base + 3], rho_[state_constraints_index_ + base + 3]) +
              state_barrier_->Hessian(vehicle_param_.delta_min - state(5, 0), (Eigen::Matrix<double, 6, 1>() <<0.0, 0.0, 0.0, 0.0, 0.0, -1.0).finished(), Eigen::MatrixXd::Zero(6, 6), al_lambda_[state_constraints_index_ + base + 4], rho_[state_constraints_index_ + base + 4]) +
              state_barrier_->Hessian(state(5, 0) - vehicle_param_.delta_max, (Eigen::Matrix<double, 6, 1>() <<0.0, 0.0, 0.0, 0.0, 0.0, 1.0).finished(), Eigen::MatrixXd::Zero(6, 6), al_lambda_[state_constraints_index_ + base + 5], rho_[state_constraints_index_ + base + 5]);

  base = index * 4;

  *cost_Hu += control_barrier_->Hessian(vehicle_param_.jerk_min - control(0, 0), (Eigen::Matrix<double, 2, 1>() <<-1.0, 0.0).finished(), Eigen::MatrixXd::Zero(2, 2), al_lambda_[control_constraints_index_ + base], rho_[control_constraints_index_ + base]) +
              control_barrier_->Hessian(control(0, 0) - vehicle_param_.jerk_max, (Eigen::Matrix<double, 2, 1>() <<1.0, 0.0).finished(), Eigen::MatrixXd::Zero(2, 2), al_lambda_[control_constraints_index_ + base + 1], rho_[control_constraints_index_ + base + 1]) +
              control_barrier_->Hessian(vehicle_param_.delta_rate_min - control(1, 0), (Eigen::Matrix<double, 2, 1>() <<0.0, -1.0).finished(), Eigen::MatrixXd::Zero(2, 2), al_lambda_[control_constraints_index_ + base + 2], rho_[control_constraints_index_ + base + 2]) +
              control_barrier_->Hessian(control(1, 0) - vehicle_param_.delta_rate_max, (Eigen::Matrix<double, 2, 1>() <<0.0, 1.0).finished(), Eigen::MatrixXd::Zero(2, 2), al_lambda_[control_constraints_index_ + base + 3], rho_[control_constraints_index_ + base + 3]);
}

void IlqrOptimizer::CorridorConsJacbian(const int index, const State& state, State* const cost_Jx) {
  double L = (vehicle_param_.rear_hang_length + vehicle_param_.wheel_base + vehicle_param_.front_hang_length) / config_.num_of_disc;
  double rf = vehicle_param_.rear_hang_length;
  Constraints cons = shrinked_corridor_[index];

  int prev_constraints_size = 0;
  for(int i = 0; i < index; i++){
    Constraints cons_temp = shrinked_corridor_[i];
    prev_constraints_size +=  cons_temp.size() * config_.num_of_disc;
  }

  for (int i = 0; i < config_.num_of_disc; ++i) {
    double length_cos = (L * (i - 0.5) - rf) * std::cos(state(2, 0));
    double length_sin = (L * (i - 0.5) - rf) * std::sin(state(2, 0));
    double x = state(0, 0) + length_cos;
    double y = state(1, 0) + length_sin;

    for (const auto& c : cons) {
      *cost_Jx += state_barrier_->Jacbian(c[0] * x + c[1] * y - c[2],
                                          (Eigen::Matrix<double, 6, 1>() << c[0], c[1], -c[0] * length_sin + c[1] * length_cos, 0.0, 0.0, 0.0).finished(),
                                          al_lambda_[corridor_constraints_index_ + prev_constraints_size],
                                          rho_[corridor_constraints_index_ + prev_constraints_size]);
      prev_constraints_size++;
    }
  }
}

void IlqrOptimizer::CorridorConsHessian(const int index, const State& state, Eigen::Matrix<double, kStateNum, kStateNum>* const cost_Hx) {
  double L = (vehicle_param_.rear_hang_length + vehicle_param_.wheel_base + vehicle_param_.front_hang_length) / config_.num_of_disc;
  double rf = vehicle_param_.rear_hang_length;
  Constraints cons = shrinked_corridor_[index];

  int prev_constraints_size = 0;
  for(int i = 0; i < index; i++){
    Constraints cons_temp = shrinked_corridor_[i];
    prev_constraints_size +=  cons_temp.size() * config_.num_of_disc;
  }

  Eigen::Matrix<double, kStateNum, kStateNum> ddx = Eigen::MatrixXd::Zero(kStateNum, kStateNum);
  for (int i = 0; i < config_.num_of_disc; ++i) {
    double length_cos = (L * (i - 0.5) - rf) * std::cos(state(2, 0));
    double length_sin = (L * (i - 0.5) - rf) * std::sin(state(2, 0));
    double x = state(0, 0) + length_cos;
    double y = state(1, 0) + length_sin;

    for (const auto& c : cons) {
      ddx(2, 2) = -c[0] * length_cos - c[1] * length_sin;
      *cost_Hx += state_barrier_->Hessian(c[0] * x + c[1] * y - c[2],
                                          (Eigen::Matrix<double, 6, 1>() << c[0], c[1], -c[0] * length_sin + c[1] * length_cos, 0.0, 0.0, 0.0).finished(), ddx,
                                          al_lambda_[corridor_constraints_index_ + prev_constraints_size],
                                          rho_[corridor_constraints_index_ + prev_constraints_size]);
      prev_constraints_size++;
    }
  }
}

void IlqrOptimizer::LaneBoundaryConsJacbian(const int index, const State& state, State* const cost_Jx) {
  double L = (vehicle_param_.rear_hang_length + vehicle_param_.wheel_base + vehicle_param_.front_hang_length) / config_.num_of_disc;
  double rf = vehicle_param_.rear_hang_length;

  int prev_constraints_size =  2 * index * config_.num_of_disc;

  for (int i = 0; i < config_.num_of_disc; ++i) {
    double length_cos = (L * (i - 0.5) - rf) * std::cos(state(2, 0));
    double length_sin = (L * (i - 0.5) - rf) * std::sin(state(2, 0));
    double x = state(0, 0) + length_cos;
    double y = state(1, 0) + length_sin;
    
    auto cons_left = FindNeastLaneSegment(x, y, shrinked_left_lane_cons_);
    *cost_Jx += state_barrier_->Jacbian(cons_left[0] * x + cons_left[1] * y - cons_left[2],
                                        (Eigen::Matrix<double, 6, 1>() <<cons_left[0], cons_left[1], -cons_left[0] * length_sin + cons_left[1] * length_cos, 0.0, 0.0, 0.0).finished(),
                                        al_lambda_[lane_constraints_index_ + prev_constraints_size],
                                        rho_[lane_constraints_index_ + prev_constraints_size]);
    prev_constraints_size++;
      
    auto cons_right = FindNeastLaneSegment(x, y, shrinked_right_lane_cons_);
    *cost_Jx += state_barrier_->Jacbian(cons_right[0] * x + cons_right[1] * y - cons_right[2],
                                        (Eigen::Matrix<double, 6, 1>() <<cons_right[0], cons_right[1], -cons_right[0] * length_sin + cons_right[1] * length_cos, 0.0, 0.0, 0.0).finished(),
                                        al_lambda_[lane_constraints_index_ + prev_constraints_size],
                                        rho_[lane_constraints_index_ + prev_constraints_size]);
    prev_constraints_size++;
  }
}

void IlqrOptimizer::LaneBoundaryConsHessian(const int index, const State& state, Eigen::Matrix<double, kStateNum, kStateNum>* const cost_Hx) {
  double L = (vehicle_param_.rear_hang_length + vehicle_param_.wheel_base + vehicle_param_.front_hang_length) / config_.num_of_disc;
  double rf = vehicle_param_.rear_hang_length;
  Eigen::Matrix<double, kStateNum, kStateNum> ddx = Eigen::MatrixXd::Zero(kStateNum, kStateNum);

  int prev_constraints_size =  2 * index * config_.num_of_disc;

  for (int i = 0; i < config_.num_of_disc; ++i) {
    double length_cos = (L * (i - 0.5) - rf) * std::cos(state(2, 0));
    double length_sin = (L * (i - 0.5) - rf) * std::sin(state(2, 0));
    double x = state(0, 0) + length_cos;
    double y = state(1, 0) + length_sin;
    
    auto cons_left = FindNeastLaneSegment(x, y, shrinked_left_lane_cons_);
    ddx(2, 2) = -cons_left[0] * length_cos - cons_left[1] * length_sin;
    *cost_Hx += state_barrier_->Hessian(cons_left[0] * x + cons_left[1] * y - cons_left[2],
                                        (Eigen::Matrix<double, 6, 1>() <<cons_left[0], cons_left[1], -cons_left[0] * length_sin + cons_left[1] * length_cos, 0.0, 0.0, 0.0).finished(),
                                        ddx,
                                        al_lambda_[lane_constraints_index_ + prev_constraints_size],
                                        rho_[lane_constraints_index_ + prev_constraints_size]);
    prev_constraints_size++;
      
    auto cons_right = FindNeastLaneSegment(x, y, shrinked_right_lane_cons_);
    ddx(2, 2) = -cons_right[0] * length_cos - cons_right[1] * length_sin;
    *cost_Hx += state_barrier_->Hessian(cons_right[0] * x + cons_right[1] * y - cons_right[2],
                                        (Eigen::Matrix<double, 6, 1>() <<cons_right[0], cons_right[1], -cons_right[0] * length_sin + cons_right[1] * length_cos, 0.0, 0.0, 0.0).finished(),
                                        ddx,
                                        al_lambda_[lane_constraints_index_ + prev_constraints_size],
                                        rho_[lane_constraints_index_ + prev_constraints_size]);
    prev_constraints_size++;
  }
}

DiscretizedTrajectory IlqrOptimizer::TransformToTrajectory(const std::vector<State>& states, const std::vector<Control>& controls) {
  std::vector<TrajectoryPoint> traj(num_of_knots_);

  for (int i = 0; i < num_of_knots_; ++i) {
    traj[i].time = i * delta_t_;
    traj[i].x = states[i](0, 0);
    traj[i].y = states[i](1, 0);
    traj[i].theta = states[i](2, 0);
    traj[i].velocity = states[i](3, 0);
    traj[i].a = states[i](4, 0);
    traj[i].delta = states[i](5, 0);
    traj[i].kappa = std::tan(states[i](5, 0)) / vehicle_param_.wheel_base; 

    if (i < num_of_knots_ - 1) {
      traj[i].jerk = controls[i](0, 0);
      traj[i].delta_rate = controls[i](1, 0);
    }
  }
  return DiscretizedTrajectory(traj);
}

void IlqrOptimizer::iqr(
    const DiscretizedTrajectory& coarse_traj,
    std::vector<State>* const guess_state,
    std::vector<Control>* const guess_control) {
  guess_state->resize(num_of_knots_);
  guess_control->resize(num_of_knots_ - 1);
  
  std::vector<Eigen::Matrix<double, kControlNum, kStateNum>> Ks(num_of_knots_ - 1);
  Eigen::Matrix<double, kStateNum, kStateNum> Q = Eigen::MatrixXd::Zero(6, 6);
  Q(0, 0) = 0.001;
  Q(1, 1) = 0.001;
  Q(2, 2) = 0.001;
  Q(3, 3) = 0.001;
  Q(4, 4) = 0.01;
  Q(5, 5) = 0.005;

  Eigen::Matrix<double, kControlNum, kControlNum> R;
  R(0, 0) = 0.2;
  R(1, 1) = 0.05;
  Eigen::Matrix<double, kStateNum, kStateNum> P = Q;
  
  SystemMatrix A;
  InputMatrix B;
  Control control;
  control << 0.0, 0.0;

  // Find Recursive Ricatti equation 
  for (int i = num_of_knots_ - 2; i >= 0; --i) { 
    vehicle_model_.DynamicsJacbian(goals_[i], control, &A, &B); 
    Eigen::Matrix<double, kControlNum, kControlNum> R_plus_BPB = R + B.transpose() * P * B;
  
    // Check if R + B^T * P * B is positive definite 
    Eigen::LLT<Eigen::Matrix<double, kControlNum, kControlNum>> llt(R_plus_BPB);

    if (llt.info() != Eigen::Success) {
      LOG_FAILURE("LQR: LLT decomposition failed in iqr, matrix is not positive definite.");
      return;
    }
    
    // Solve for gain K 
    Ks[i] = llt.solve(B.transpose() * P * A);
    P = Q + A.transpose() * P * (A - B * Ks[i]);
  }

  auto clamp = [](const double x, const double min, const double max) {
    return std::fmin(max, std::fmax(x, min));
  };
  
  State x;
  x = goals_[0];
  guess_state->front() = x;
  for (int i = 0; i < num_of_knots_ - 1; ++i) {
    guess_control->at(i) = -Ks[i] * (x - goals_[i]);
    guess_control->at(i)(0, 0) = clamp(guess_control->at(i)(0, 0), vehicle_param_.jerk_min, vehicle_param_.jerk_max);
    guess_control->at(i)(1, 0) = clamp(guess_control->at(i)(1, 0), vehicle_param_.delta_rate_min, vehicle_param_.delta_rate_max);
    // guess_control->at(i)(0, 0) = 0.0;
    // guess_control->at(i)(1, 0) = 0.0;
    vehicle_model_.Dynamics(x, guess_control->at(i), &(guess_state->at(i + 1)));
    x = guess_state->at(i + 1);
  }
}

} // namespace trajectory_planning