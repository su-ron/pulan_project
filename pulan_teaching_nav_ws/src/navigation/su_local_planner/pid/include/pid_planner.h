/***********************************************************
 *
 * @file: pid_planner.h
 * @breif: Contains the Proportional–Integral–Derivative (PID) controller local
 *planner class
 * @author: Yang Haodong, Guo Zhanyu, Wu Maojia
 * @update: 2023-10-1
 * @version: 1.1
 *
 * Copyright (c) 2023，Yang Haodong
 * All rights reserved.
 * --------------------------------------------------------
 *
 **********************************************************/

#ifndef PID_PLANNER_H_
#define PID_PLANNER_H_

#include <geometry_msgs/Twist.h>
#include <tf2_ros/buffer.h>

#include <ackermann_msgs/AckermannDriveStamped.h>
#include <ackermann_msgs/AckermannDrive.h>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/Marker.h>

#include "local_planner.h"
#include "custom_msgs_srvs/Control.h"
#include "custom_msgs_srvs/Position.h"
#include <cmath>
namespace pid_planner {
/**
 * @brief A class implementing a local planner using the PID
 */
class PIDPlanner : public local_planner::LocalPlanner {
 public:
  /**
   * @brief Construct a new PIDPlanner object
   */
  PIDPlanner();

  /**
   * @brief Destroy the PIDPlanner object
   */
  ~PIDPlanner();
  /**
   * @brief 初始化rviz marker
   */
  void initMarker();
  /**
   * @brief 获取当前的角速度信息
   */
  void robotVelCB(const geometry_msgs::Twist::ConstPtr& msg);
  /**
   * @brief odom回调函数
   */
  void odomCB(const nav_msgs::Odometry::ConstPtr& odomMsg);
  /**
   * @brief 全局路径回调函数
   */
  void pathCB(const nav_msgs::Path::ConstPtr& pathMsg);
  /**
   * @brief Set the plan that the controller is following
   * @param orig_global_plan the plan to pass to the controller
   * @return true if the plan was updated successfully, else false
   */
  bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);

  /*
   * @brief 获取目标点信息
   */
  void goalCB(const geometry_msgs::PoseStamped::ConstPtr& goalMsg);
  /**
   * @brief 判断目前车辆与前瞻点的距离是否大于设定的前瞻距离
   * @param wayPt 前瞻点
   * @param car_pos 表示现在车辆的位置
   * @return 大于返回true
   */
  bool isWayPtAwayFromLfwDist(const geometry_msgs::Point& wayPt,
                              const geometry_msgs::Point& car_pos);
  /**
   * @brief 寻找车辆位置与全局路径距离最近的路径点
   * @param wayPt 前瞻点
   * @param car_pos 表示现在车辆的位置
   * @return 返回最近路径点的序号
   */
  bool calcNearestPoseInterp(
      const std::vector<geometry_msgs::PoseStamped>& global_plan,
      const geometry_msgs::Pose& self_pose, unsigned int& nearest_index);
  /**
   * @brief 返回前瞻点在车辆坐标系上的位置
   * @param carPose odom回调函数得到的位置信息
   * @return 返回前瞻点在车辆坐标系上的坐标
   */
  double getYawFromPose(const geometry_msgs::Pose& carPose);
  /**
   * @brief 获取车辆与前瞻点朝向误差
   * @param carPose odom回调函数得到的位置信息
   * @return 朝向误差
   */
  double getEta(const geometry_msgs::Pose& carPose);
  /**
   * @brief 获取车辆与前瞻点位置偏差
   * @param carPose odom回调函数得到的位置信息
   * @return 位置偏差
   */
  double getCar2WayPtDist(const geometry_msgs::Pose& carPose);
  /**
   * @brief 返回前瞻点在车辆坐标系上的位置
   * @param carPose odom回调函数得到的位置信息
   * @return 返回前瞻点在车辆坐标系上的坐标
   */
  geometry_msgs::Point get_odom_car2WayPtVec(
      const geometry_msgs::Pose& carPose);
  /**
   * @brief 得到amcl的定位信息
   */
  void amclCB(
      const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& amclMsg);
  /**
   * @brief 定时计算速度
   */
  void controlLoopCB(const ros::TimerEvent&);
  /**
   * @brief Given the current position, orientation, and velocity of the robot,
   * compute the velocity commands
   * @param cmd_vel will be filled with the velocity command to be passed to the
   * robot base
   * @return true if a valid trajectory was found, else false
   */
  bool computeVelocityCommands();

  /**
   * @brief linear velocity regularization
   * @param base_odometry odometry of the robot, to get velocity
   * @param v_d           desired velocity magnitude
   * @return v            regulated linear velocity
   */
  double linearRegularization(nav_msgs::Odometry& base_odometry, double v_d);

  /**
   * @brief angular velocity regularization
   * @param base_odometry odometry of the robot, to get velocity
   * @param w_d           desired angular velocity
   * @return  w           regulated angular velocity
   */
  double angularRegularization(nav_msgs::Odometry& base_odometry, double w_d);

void publish_controlindicators();

 private:
  bool initialized_;    // initialized flag
  bool goal_reached_;   // goal reached flag
  tf2_ros::Buffer tf_;  // transform buffer

  int plan_index_;
  std::vector<geometry_msgs::PoseStamped> global_plan_;
  geometry_msgs::PoseStamped target_ps_, current_ps_;

  double p_window_;  // next point distance
  double d_t_;       // control time step

  double k_v_p_, k_v_i_, k_v_d_;  // pid controller params
  double k_w_p_, k_w_i_, k_w_d_;  // pid controller params
  double k_theta_;                // pid controller params

  double e_v_, e_w_;
  double i_v_, i_w_;

  ros::Publisher target_pose_pub_, current_pose_pub_;

  // goal parameters
  double goal_x_, goal_y_;
  Eigen::Vector3d goal_rpy_;

  bool cmd_vel_mode, foundForwardPt_, goal_received_;
  ros::NodeHandle n_;
  ros::Publisher ackermann_pub, cmdvel_pub, marker_pub;
  ros::Publisher control_indicators_;
  custom_msgs_srvs::Control control_;
  ros::Subscriber path_sub, goal_sub, amcl_sub, odom_sub, current_vel_sub;
  geometry_msgs::Twist cmd_vel;
  geometry_msgs::Point odom_goal_pos, goal_pos;
  ackermann_msgs::AckermannDrive ackermann_cmd;
  visualization_msgs::Marker points, line_strip, goal_circle;
  geometry_msgs::PoseStamped current_ps_odom_;
  nav_msgs::Odometry odom;
  nav_msgs::Path map_path;
  geometry_msgs::PoseStamped map_path_pose_;
  ros::Timer timer1;
  std::string robot_type_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  double Lfw_, goal_radius_, cmd_z;
  double minimum_lookahead_distance_ = 1.5;
  double lookahead_distance_ratio_ = 2.0;
  double math_path_proportion_;
  double wheelbase_;
};
};  // namespace pid_planner

#endif