#include "common/path/path_generator.h"
#include <iostream>


using namespace planning;
typedef data_types::PncPoint Point;

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "pathgenerator_node");
  PathGenerator pathgenerator;
  std::vector<Point> ref_line;
  pathgenerator.GenerateRefline(ref_line);

  // record data
   Point start_pose, target_pose;
  start_pose.set_x(4.0);
  start_pose.set_y(1.0);
  start_pose.set_yaw(2.0);
  start_pose.set_kappa(0.0);

  target_pose.set_x(-2.4);
  target_pose.set_y(4.37);
  target_pose.set_yaw(-2.56);
  target_pose.set_kappa(0);

//   start_pose.set_x(-3.432074);
//   start_pose.set_y(3.321616);
//   start_pose.set_yaw(0.796507);
//   start_pose.set_kappa(0.0);

//   target_pose.set_x(4.881914);
//   target_pose.set_y(0.046644);
//   target_pose.set_yaw(0.024264);
//   target_pose.set_kappa(0);

  std::vector<Point> desired_path;
  // 生成的是一条基于Frenet坐标系的轨迹
  pathgenerator.GenerateFrenetBasedSpline(start_pose, target_pose, ref_line,
                                           &desired_path);

  ROS_INFO("convert desired_path.size()=%zu",desired_path.size());
 // pathgenerator.publishMarker(desired_path);

  return 0;
}
