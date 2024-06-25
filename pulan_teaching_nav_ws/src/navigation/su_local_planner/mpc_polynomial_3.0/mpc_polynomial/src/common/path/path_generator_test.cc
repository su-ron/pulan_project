#include "common/path/path_generator.h"
#include <iostream>


using namespace planning;
typedef data_types::PncPoint Point;
Point target_pose_,start_pose_;

void GoalCB(const geometry_msgs::PoseStamped::ConstPtr& goalMsg)
{
  ROS_INFO("goal Received ");
  target_pose_.set_x(goalMsg->pose.position.x);
  target_pose_.set_y(goalMsg->pose.position.y);
  target_pose_.set_yaw(tf::getYaw(goalMsg->pose.orientation));
  std::vector<Point> ref_line;
  std::vector<Point> desired_path;
  auto path_generator = planning::PathGenerator::Instance();
  // 生成的是一条基于Frenet坐标系的轨迹
  path_generator.GenerateFrenetBasedSpline(start_pose_, target_pose_, ref_line,
                                           &desired_path);
  path_generator.publishMarker(desired_path);
  
}

void InitposeCB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
  ROS_INFO("initpose Received ");
  ROS_INFO("Received pose message with frame_id: %s", msg->header.frame_id.c_str());
  start_pose_.set_x(msg->pose.pose.position.x);
  start_pose_.set_y(msg->pose.pose.position.y);
  start_pose_.set_yaw(tf::getYaw(msg->pose.pose.orientation));

}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "pathgenerator_node");
  ros::NodeHandle nh;
  ros::Subscriber sub_goal_ = nh.subscribe( "/move_base_simple/goal", 5,GoalCB);
  ros::Subscriber sub_initpose_ = nh.subscribe("/initialpose", 5, InitposeCB);
  ros::spin();
  return 0;
}
