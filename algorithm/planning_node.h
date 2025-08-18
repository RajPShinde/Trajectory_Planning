#include <ros/ros.h>

#include "geometry_msgs/PoseStamped.h"
#include "trajectory_optimization/CenterLine.h"
#include "trajectory_optimization/Obstacles.h"
#include "trajectory_optimization/DynamicObstacles.h"

#include "algorithm/planner/trajectory_planner.h"
#include "algorithm/visualization/plot.h"

namespace trajectory_optimization {

class PlanningNode {
 public:
  explicit PlanningNode(const ros::NodeHandle& nh);

 private:
  void CenterLineCallback(const CenterLineConstPtr& msg);

  void ObstaclesCallback(const ObstaclesConstPtr& msg);

  void DynamicObstaclesCallback(const DynamicObstaclesConstPtr& msg); 

  void PlanCallback(const geometry_msgs::PoseStampedConstPtr& msg);

  void PlotVehicle(const int id, const math::Pose& pt, const double phi);

  std::array<math::Box2d, 4> GenerateTireBoxes(const math::Pose& pose, const double phi = 0.0) const;

  ros::NodeHandle nh_;
  PlannerConfig config_;
  Env env_;
  std::shared_ptr<TrajectoryPlanner> planner_;
  StartState state_;

  ros::Subscriber center_line_subscriber_; 
  ros::Subscriber obstacles_subscriber_; 
  ros::Subscriber dynamic_obstacles_subscriber_; 
  ros::Subscriber goal_subscriber_;
};

} // namsepace planning