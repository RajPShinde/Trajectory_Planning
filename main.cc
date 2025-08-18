#include "algorithm/planning_node.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "trajectory_planner_node");

  ros::NodeHandle nh;
  trajectory_optimization::visualization::Init(nh, "map", "trajectory_planner_markers");

  trajectory_optimization::PlanningNode node(nh);
  ros::spin();
  return 0;
}