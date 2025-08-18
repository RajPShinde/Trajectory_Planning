#pragma once

#include <vector>

#include "algorithm/params/planner_config.h"
#include "algorithm/ilqr/corridor.h"
#include "algorithm/utils/discretized_trajectory.h"
#include "algorithm/ilqr/vehicle_model.h"
#include "algorithm/ilqr/barrier_function.h"
#include "algorithm/ilqr/typedefs.h"
#include "algorithm/ilqr/logger.hpp"

namespace trajectory_optimization {

struct Cost {
  double total_cost = 0.0;
  double target_cost = 0.0;
  double dynamic_cost = 0.0;
  double corridor_cost = 0.0;
  double lane_boundary_cost = 0.0;

  Cost() = default;

  Cost(const double c0, const double c1, const double c2, const double c3, const double c4): total_cost(c0),
                                                                                             target_cost(c1),
                                                                                             dynamic_cost(c2),
                                                                                             corridor_cost(c3),
                                                                                             lane_boundary_cost(c4){
}
};

class IlqrOptimizer {
 public:
  IlqrOptimizer() = default;    

  IlqrOptimizer(const IlqrConfig& config, const VehicleParam& param, const double horizon, const double dt);
  
  bool Plan(const TrajectoryPoint& start_state, const DiscretizedTrajectory& coarse_traj,
            const CorridorConstraints& corridor, const LaneConstraints& left_lane_cons,
            const LaneConstraints& right_lane_cons, DiscretizedTrajectory* const opt_trajectory, 
            std::vector<DiscretizedTrajectory>* const iter_trajs);

  std::vector<Cost> cost() {
    return cost_;
  }

 private:
  void iqr(const DiscretizedTrajectory& coarse_traj, std::vector<State>* const guess_state, std::vector<Control>* const guess_control);

  void CalculateDiscRadius();

  void TransformGoals(const DiscretizedTrajectory& coarse_traj);

  ilqr::SolverStatus Optimize(const TrajectoryPoint& start_state,
                              const DiscretizedTrajectory& coarse_traj,
                              const CorridorConstraints& corridor,
                              const LaneConstraints& left_lane_cons,
                              const LaneConstraints& right_lane_cons,
                              DiscretizedTrajectory* const opt_trajectory,
                              std::vector<DiscretizedTrajectory>* const iter_trajs);

  std::vector<std::pair<ilqr::ConstraintType, double>> GetAllConstraintViolations(const std::vector<State>& states, const std::vector<Control>& controls);

  double CalculateMaxConstraintViolation(std::vector<std::pair<ilqr::ConstraintType, double>> h_values);

  bool Backward(const double lambda,
                const std::vector<State>& states,
                const std::vector<Control>& controls,
                std::vector<Eigen::Matrix<double, kControlNum, kStateNum>>* const Ks,
                std::vector<Eigen::Matrix<double, kControlNum, 1>>* const ks,
                std::vector<Eigen::Matrix<double, kControlNum, 1>>* const Qus,
                std::vector<Eigen::Matrix<double, kControlNum, kControlNum>>* const Quus);

  void Forward(const double alpha,
               std::vector<State>* const states,
               std::vector<Control>* const controls,
               const std::vector<Eigen::Matrix<double, kControlNum, kStateNum>>& Ks,
               const std::vector<Eigen::Matrix<double, kControlNum, 1>>& ks,
               const std::vector<Eigen::Matrix<double, kControlNum, 1>>& Qus,
               const std::vector<Eigen::Matrix<double, kControlNum, kControlNum>>& Quus);

  double TotalCost(const std::vector<State>& states, const std::vector<Control>& controls, Cost* const cost);

  double TotalCost(const std::vector<State>& states, const std::vector<Control>& controls, bool print);

  double JCost(const std::vector<State>& states, const std::vector<Control>& controls);

  double DynamicsCost(const std::vector<State>& states, const std::vector<Control>& controls);

  double CorridorCost(const std::vector<State>& states);

  double LaneBoundaryCost(const std::vector<State>& states);

  void CostJacbian(const int index,
                   const State& state, const Control& control,
                   State* const cost_Jx, Control* cost_Ju);

  void CostHessian(const int index,
                   const State& state, const Control& control,
                   Eigen::Matrix<double, kStateNum, kStateNum>* const cost_Hx,
                   Eigen::Matrix<double, kControlNum, kControlNum>* const cost_Hu);

  void ShrinkConstraints(const CorridorConstraints& corridor, const LaneConstraints& left_lane_cons, const LaneConstraints& right_lane_cons);

  Eigen::Vector3d FindNeastLaneSegment(const double x, const double y, std::vector<std::pair<Eigen::Vector3d, math::LineSegment2d>> lane_segs);

  void DynamicsConsJacbian(const int index, const State& state, const Control& control, State* const cost_Jx, Control* cost_Ju);

  void DynamicsConsHessian(const int index, const State& state, const Control& control,
                           Eigen::Matrix<double, kStateNum, kStateNum>* const cost_Hx,
                           Eigen::Matrix<double, kControlNum, kControlNum>* const cost_Hu);

  void CorridorConsJacbian(const int index, const State& state, State* const cost_Jx);

  void CorridorConsHessian(const int index, const State& state, Eigen::Matrix<double, kStateNum, kStateNum>* const cost_Hx);

  void LaneBoundaryConsJacbian(const int index, const State& state, State* const cost_Jx);

  void LaneBoundaryConsHessian(const int index, const State& state, Eigen::Matrix<double, kStateNum, kStateNum>* const cost_Hx);

  DiscretizedTrajectory TransformToTrajectory(const std::vector<State>& states, const std::vector<Control>& controls);

  void NormalizeHalfPlane();

  double CalGradientNorm(const std::vector<Eigen::Matrix<double, kControlNum, 1>>& ks, const std::vector<Control>& controls);

 private:
  double horizon_;
  double delta_t_;
  int num_of_knots_;
  double disc_radius_;

  ilqr::Options options_;

  IlqrConfig config_;
  VehicleParam vehicle_param_;

  VehicleModel vehicle_model_;

  std::unique_ptr<AugmentedLagrangian<kStateNum>> state_barrier_;
  std::unique_ptr<AugmentedLagrangian<kControlNum>> control_barrier_;
  
  TrajectoryPoint start_state_;
  std::vector<State> goals_;

  CorridorConstraints shrinked_corridor_;
  LaneConstraints shrinked_left_lane_cons_;
  LaneConstraints shrinked_right_lane_cons_;

  std::vector<Cost> cost_;

  std::vector<double> al_lambda_;
  std::vector<double> rho_;
  std::vector<std::pair<ilqr::ConstraintType, double>> h_values_;
  int state_constraints_index_ = 0;
  int control_constraints_index_ = 0;
  int lane_constraints_index_ = 0;
  int corridor_constraints_index_ = 0;

  double rhoMin_ = 1e-8;
  double rhoMax_ = 1e11;
  double drho_ = 1.0;
  double rhoFactor_ = 1.6;

  std::vector<SystemMatrix> As;
  std::vector<InputMatrix> Bs;

  std::vector<Eigen::Matrix<double, kStateNum, 1>> cost_Jx;
  std::vector<Eigen::Matrix<double, kControlNum, 1>> cost_Ju;
  std::vector<Eigen::Matrix<double, kStateNum, kStateNum>> cost_Hx;
  std::vector<Eigen::Matrix<double, kControlNum, kControlNum>> cost_Hu;
  
  double delta_V_[2];
};

} // namespace trajectory_optimization