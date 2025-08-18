#include "algorithm/planning_node.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "trajectory_planner_node");

  ros::NodeHandle nh;
  trajectory_planning::visualization::Init(nh, "map", "trajectory_planner_markers");

  trajectory_planning::PlanningNode node(nh);
  ros::spin();
  return 0;
}