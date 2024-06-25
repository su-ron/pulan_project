/***********************************************************
 *
 * @file: polynomial_curve.h
 * @breif: Polynomial curve generation
 * @author: Yang Haodong
 * @update: 2023-12-26
 * @version: 1.0
 *
 * Copyright (c) 2023, Yang Haodong
 * All rights reserved.
 * --------------------------------------------------------
 *
 **********************************************************/
#ifndef POLYNOMIAL_CURVE_H
#define POLYNOMIAL_CURVE_H

#define POLY_DIR_ACC 0
#define POLY_DIR_JERK 1

#include "curve.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>

namespace trajectory_generation
{
// x, y, yaw, v, a
using PolyState = std::tuple<double, double, double, double, double>;

// Polynomial interpolation solver
class Poly
{
public:
  Poly(std::tuple<double, double, double> state_0, std::tuple<double, double, double> state_1, double t);
  ~Poly();

  double x(double t);
  double dx(double t);
  double ddx(double t);
  double dddx(double t);

protected:
  double p0, p1, p2, p3, p4, p5;  // Quintic polynomial coefficient
};

class PolyTrajectory
{
public:
  /**
   * @brief Construct a new Polynomial trajectory  object
   */
  PolyTrajectory();

  /**
   * @brief Destroy the Polynomial trajectory object
   */
  ~PolyTrajectory();
 
  /**
   * @brief Clear the Polynomial trajectory
   */
  void clear();

  /**
   * @brief Append the state to Polynomial trajectory
   * @param time  Current time step
   * @param x     Current x-position
   * @param y     Current y-position
   * @param yaw   Current yaw angle
   * @param v     Current speed
   * @param a     Current acceleration
   * @param jerk  Current jerk
   */
  void append(double time, double x, double y, double yaw, double v, double a, double jerk);

  /**
   * @brief Determine the direction of the motion trajectory
   * @param mode  trajectory mode, i.e., acceleration or jerk
   * @return direction  1 is positive direction and -1 is nagetive, 0 is invalid
   */
  int dir(int mode);

  /**
   * @brief Determine whether the generated trajectory is valid
   * @param max_acc     Maximum acceleration
   * @param max_jerk    Maximum jerk
   * @return flag       true is valid else invalid
   */
  bool valid(double max_acc, double max_jerk);

  /**
   * @brief Calculate the size of trajectory
   * @return size   the size of trajectory
   */
  double size();

  /**
   * @brief Convert the trajectory to path points
   * @return path  path points (x, y)
   */
  Points2d toPath();

  void getinformation();

 public:
  std::vector<double> time;
  std::vector<double> x;
  std::vector<double> y;
  std::vector<double> yaw;
  std::vector<double> v;
  std::vector<double> a;
  std::vector<double> jerk;

protected:
  std::vector<double> time_;
  std::vector<double> x_;
  std::vector<double> y_;
  std::vector<double> yaw_;
  std::vector<double> v_;
  std::vector<double> a_;
  std::vector<double> jerk_;
};

class Polynomial : public Curve
{
public:
  /**
   * @brief Construct a new Polynomial generation object
   * @param max_acc     Maximum acceleration (default: 1.0)
   * @param max_jerk    Maximum jerk (default: 0.5)
   */
  Polynomial();
  Polynomial(double step, double max_acc, double max_jerk);
  
  /**
   * @brief Destroy the Polynomial generation object
   */
  ~Polynomial();

  void initMarker();
  /**
   * @brief 初始位置回调函数
   */
  void initposeCB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
    /**
   * @brief 目标位置回调函数
   */
  void goalCB(const geometry_msgs::PoseStamped::ConstPtr& goalMsg);
  /**
   * @brief Generate a valid trajectory from start state to goal state
   * @param start_state   start state
   * @param goal_state    goal state
   * @param traj    the trajectory
   */
  void generation(PolyState start_state, PolyState goal_state, PolyTrajectory& traj);
  /*
   * @brief 将得到的起点和终点信息geometry_msgs::Point转换成Points2d格式
   */
  Points2d convertToPoint2d(const geometry_msgs::Point& point);
  /**
   * @brief Running trajectory generation
   * @param points path points <x, y>
   * @param path generated trajectory
   * @return true if generate successfully, else failed
   */
  bool run(const Points2d points, Points2d& path,nav_msgs::Path&desired_path);

  bool converttrajToNavPath(PolyTrajectory&traj,nav_msgs::Path&nav_path);
  /**
   * @brief Running trajectory generation
   * @param points path points <x, y, theta>
   * @param path generated trajectory
   * @return true if generate successfully, else failed
   */
  bool run(const Poses2d points, Points2d& path,nav_msgs::Path&desired_path);


  /**
   * @brief Configure the maximum acceleration.
   * @param max_acc  The maximum acceleration
   */
  void setMaxAcceleration(double max_acc);

  /**
   * @brief Configure the maximum jerk.
   * @param max_jerk  The maximum jerk
   */
  void setMaxJerk(double max_jerk);
public:
   ros::Publisher _pub_polynomialtraj,marker_pub_;
   ros::Subscriber _sub_initpose,_sub_goal;
   bool initpose_received_,initpose_changed_, goal_received_,goal_changed_;
   double initpose_x_, initpose_y_,initpose_yaw_, goal_x_, goal_y_,goal_yaw_;
   geometry_msgs::Point goal_pos_;
   ros::NodeHandle nh_;
   geometry_msgs::PoseStamped initpose_map_, initpose_base_,initpose_odom_;
   geometry_msgs::PoseStamped midpointpose_map_, midpointpose_base_,midpointpose_odom_;
   geometry_msgs::PoseStamped pointpose_x_4_map_, pointpose_x_4_base_,pointpose_x_4_odom_;
   geometry_msgs::PoseStamped pointpose_x_2_map_, pointpose_x_2_base_,pointpose_x_2_odom_;
   geometry_msgs::PoseStamped pointpose_y_2_map_, pointpose_y_2_base_,pointpose_y_2_odom_;
   geometry_msgs::PoseStamped goalpose_map_, goalpose_base_,goalpose_odom_;

   tf::TransformListener tf_listener_;

   visualization_msgs::Marker points,mid_points, line_strip,forward_line_strip, goal_circle;
  protected:
   double max_acc_;   // Maximum acceleration
   double max_jerk_;  // Maximum jerk
};
}  // namespace trajectory_generation
#endif