/***********************************************************
 *
 * @file: polynomial_curve.cpp
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
#include <Eigen/Dense>
#include <algorithm>
#include <cassert>
#include "polynomial_curve.h"

namespace trajectory_generation
{
// Polynomial interpolation solver 多项式插值求解器
//这部分实现的是五次多项式的系数计算
Poly::Poly(std::tuple<double, double, double> state_0, std::tuple<double, double, double> state_1, double t)
{
  double x0, v0, a0;
  double xt, vt, at;
  std::tie(x0, v0, a0) = state_0;
  std::tie(xt, vt, at) = state_1;

  Eigen::Matrix3d A;
  A << std::pow(t, 3), std::pow(t, 4), std::pow(t, 5), 3 * std::pow(t, 2), 4 * std::pow(t, 3), 5 * std::pow(t, 4),
      6 * t, 12 * std::pow(t, 2), 20 * std::pow(t, 3);

  Eigen::Vector3d b(xt - x0 - v0 * t - a0 * t * t / 2, vt - v0 - a0 * t, at - a0);
  Eigen::Vector3d x = A.lu().solve(b);

  // Quintic polynomial coefficient 五次多项式
  //根据给定的初始状态、终止状态和时间计算五次多项式系数的功能，使得我们可以根据这些系数构建出一个五次多项式，用于描述在给定时间范围内的运动状态变化。
  p0 = x0;
  p1 = v0;
  p2 = a0 / 2.0;
  p3 = x(0);
  p4 = x(1);
  p5 = x(2);
}

Poly::~Poly()
{
}

double Poly::x(double t)
{
  return p0 + p1 * t + p2 * std::pow(t, 2) + p3 * std::pow(t, 3) + p4 * std::pow(t, 4) + p5 * std::pow(t, 5);
}
double Poly::dx(double t)
{
  return p1 + 2 * p2 * t + 3 * p3 * std::pow(t, 2) + 4 * p4 * std::pow(t, 3) + 5 * p5 * std::pow(t, 4);
}
double Poly::ddx(double t)
{
  return 2 * p2 + 6 * p3 * t + 12 * p4 * std::pow(t, 2) + 20 * p5 * std::pow(t, 3);
}
double Poly::dddx(double t)
{
  return 6 * p3 + 24 * p4 * t + 60 * p5 * std::pow(t, 2);
}

/**
 * @brief Construct a new Polynomial trajectory  object
 */
PolyTrajectory::PolyTrajectory()
{
  clear();
}

/**
 * @brief Destroy the Polynomial trajectory object
 */
PolyTrajectory::~PolyTrajectory()
{
}

/**
 * @brief Clear the Polynomial trajectory
 */
void PolyTrajectory::clear()
{
  time_.clear();
  x_.clear();
  y_.clear();
  yaw_.clear();
  v_.clear();
  a_.clear();
  jerk_.clear();
}

/**
 * @brief Append the state to Polynomial trajectory  将状态附加到多项式轨迹
 * @param time  Current time step
 * @param x     Current x-position
 * @param y     Current y-position
 * @param yaw   Current yaw angle
 * @param v     Current speed
 * @param a     Current acceleration
 * @param jerk  Current jerk
 */
void PolyTrajectory::append(double time, double x, double y, double yaw, double v, double a, double jerk)
{
  time_.push_back(time);
  x_.push_back(x);
  y_.push_back(y);
  yaw_.push_back(yaw);
  v_.push_back(v);
  a_.push_back(a);
  jerk_.push_back(jerk);
}

/**
 * @brief Determine the direction of the motion trajectory
 * 确定运动轨迹的方向
 * @param mode  trajectory mode, i.e., acceleration or jerk
 * @return direction  1 is positive direction and -1 is nagetive, 0 is invalid
 */
int PolyTrajectory::dir(int mode)
{
  if (mode == POLY_DIR_ACC)
  {
    if (v_.size() >= 2)
    {
      if (v_[v_.size() - 1] < v_[v_.size() - 2])
        return -1;
      else
        return 1;
    }
    else
      return 0;
  }
  else
  {
    if (a_.size() >= 2)
    {
      if (a_[a_.size() - 1] < a_[a_.size() - 2])
        return -1;
      else
        return 1;
    }
    else
      return 0;
  }
}

/**
 * @brief Determine whether the generated trajectory is valid
 * @param max_acc     Maximum acceleration
 * @param max_jerk    Maximum jerk
 * @return flag       true is valid else invalid
 */
bool PolyTrajectory::valid(double max_acc, double max_jerk)
{
  if ((*std::max_element(a_.begin(), a_.end()) <= max_acc) &&
      (*std::max_element(jerk_.begin(), jerk_.end()) <= max_jerk))
    return true;
  else
    return false;
}

/**
 * @brief Calculate the size of trajectory
 * @return size   the size of trajectory
 */
double PolyTrajectory::size()
{
  assert(time_.size() == x_.size());
  assert(x_.size() == y_.size());
  assert(y_.size() == yaw_.size());
  assert(yaw_.size() == v_.size());
  assert(v_.size() == a_.size());
  assert(a_.size() == jerk_.size());
  return time_.size();
}

/**
 * @brief Convert the trajectory to path points
 * @return path  path points (x, y)
 */
Points2d PolyTrajectory::toPath()
{
  Points2d path;
  for (size_t i = 0; i < size(); i++)
    path.push_back({ x_[i], y_[i] });
  return path;
}

 void PolyTrajectory::getinformation(){
  time=time_;
  x=x_;
  y=y_;
  yaw=yaw_;
  v=v_;
  a=a_;
  jerk=jerk_;
 }

/**
 * @brief Construct a new Polynomial generation object
 * @param max_acc     Maximum acceleration (default: 1.0)
 * @param max_jerk    Maximum jerk (default: 0.5)
 */
Polynomial::Polynomial(double step, double max_acc, double max_jerk)
  : Curve(step), max_acc_(max_acc), max_jerk_(max_jerk)
{
}
Polynomial::Polynomial() : Curve(2.0), max_acc_(3.0), max_jerk_(1.0)
{
}

/**
 * @brief Destroy the Polynomial generation object
 */
Polynomial::~Polynomial()
{
}

/**
 * @brief Generate a valid trajectory from start state to goal state
 * 生成从起始状态到目标状态的有效轨迹
 * @param start_state   start state
 * @param goal_state    goal state
 * @param traj    the trajectory
 */
void Polynomial::generation(PolyState start_state, PolyState goal_state, PolyTrajectory& traj)
{
  //  simulation parameters
  double t_min = 1.0;
  double t_max = 30.0;
  double dt = 0.5;

  double sx, sy, syaw, sv, sa;
  double gx, gy, gyaw, gv, ga;
  //得到起点和终点的信息
  std::tie(sx, sy, syaw, sv, sa) = start_state;
  std::tie(gx, gy, gyaw, gv, ga) = goal_state;
  //起点速度和终点速度
  double sv_x = sv * cos(syaw);
  double sv_y = sv * sin(syaw);
  double gv_x = gv * cos(gyaw);
  double gv_y = gv * sin(gyaw);
  //起点位置和终点位置
  double sa_x = sa * cos(syaw);
  double sa_y = sa * sin(syaw);
  double ga_x = ga * cos(gyaw);
  double ga_y = ga * sin(gyaw);

  traj.clear();

  double T = t_min;
  while (T < t_max)
  {
    Poly x_psolver({ sx, sv_x, sa_x }, { gx, gv_x, ga_x }, T);
    Poly y_psolver({ sy, sv_y, sa_y }, { gy, gv_y, ga_y }, T);
    double t = 0.0;
    while (t < T + dt)
    {
      double vx = x_psolver.dx(t);
      double vy = y_psolver.dx(t);
      double v = hypot(vx, vy);
      double yaw = atan2(vy, vx);

      double ax = x_psolver.ddx(t);
      double ay = y_psolver.ddx(t);
      double a = hypot(ax, ay);
      a = traj.dir(POLY_DIR_ACC) ? a * traj.dir(POLY_DIR_ACC) : a;

      double jx = x_psolver.dddx(t);
      double jy = y_psolver.dddx(t);
      double j = hypot(jx, jy);
      j = traj.dir(POLY_DIR_JERK) ? j * traj.dir(POLY_DIR_JERK) : j;
      //将状态附加到多项式轨迹
      traj.append(t, x_psolver.x(t), y_psolver.x(t), v, yaw, a, j);

      t += dt;
    }
    //检查生成的多项式轨迹是否满足给定的最大加速度和最大 jerk 的限制。
    if (traj.valid(max_acc_, max_jerk_))
      break;
    else
      traj.clear();

    T += step_;
  }
}
/*
* @brief 将得到的起点和终点信息geometry_msgs::Point转换成Points2d格式
*/
Points2d Polynomial::convertToPoint2d(const geometry_msgs::Point& point)
{
    Points2d result;
    result.push_back(std::make_pair(point.x, point.y));
    return result;
}

/**
 * @brief Running trajectory generation
 * @param points path points <x, y>
 * @param path generated trajectory
 * @return true if generate successfully, else failed
 */
bool Polynomial::run(const Points2d points, Points2d& path,nav_msgs::Path desired_path)
{
    Poses2d poses;
    poses.emplace_back(points.begin()->first, points.begin()->second, 0);
    //poses包含x,y,theta,theta通过插值获取
    double theta1 = helper::angle(points[0], points[1]);
    double theta2 = 0;
    poses.emplace_back(points[0].first, points[0].second,
                       (theta1 + theta2) / 2);
    poses.emplace_back(points.back().first, points.back().second, 0);
    return run(poses, path,desired_path);
}

/*需要将得到的Points2d& path信息转化成nav_msgs::Path信息才能给mpc算法调用
*
*/
 bool Polynomial::converttrajToNavPath(PolyTrajectory&traj,nav_msgs::Path nav_path)
{
    nav_path.header.stamp = ros::Time::now();
    nav_path.header.frame_id = "map"; // 设置frame_id
    double size = traj.size();
    traj.getinformation();
    for (double i = 0; i < size; i++) {
      geometry_msgs::PoseStamped pose_stamped;
      pose_stamped.pose.position.x = traj.x[i];
      pose_stamped.pose.position.y = traj.y[i];
      pose_stamped.pose.position.z = 0;  // 可根据实际情况设置
      // 添加四元数
    // 将四元数转换为geometry_msgs::Quaternion类型
    pose_stamped.pose.orientation= tf::createQuaternionMsgFromYaw(traj.yaw[i]);

    nav_path.poses.push_back(pose_stamped);
    }
    return !nav_path.poses.empty();
}

//只是存在两个点，起点和终点
/**
 * @brief Running trajectory generation
 * 在轨迹规划过程中，只需要遍历路径点，在相邻路径点间进行插值即可
 * @param points path points <x, y, theta>
 * @param path generated trajectory
 * @return true if generate successfully, else failed
 */
bool Polynomial::run(const Poses2d points, Points2d& path,nav_msgs::Path desired_path)
{
    path.clear();
    // generate velocity and acceleration constraints heuristically
    //启发式生成速度和加速度约束
    //创建了一个 std::vector<double> 类型的向量 v，其长度为 points.size()，并且初始化每个元素的值为 1.0。
    std::vector<double> v(points.size(), 1.0);
    v[0] = 0.0;

    std::vector<double> a;
    //获取加速度
    for (size_t i = 0; i < points.size() - 1; i++)
      a.push_back((v[i + 1] - v[i]) / 5);//加速度为什么这么取
    a.push_back(0.0);
    
      PolyTrajectory traj;
      PolyState start(std::get<0>(points[0]), std::get<1>(points[0]), std::get<2>(points[0]), v[0], a[0]);
      //std::get<0>(points[i]):x,std::get<2>(points[i]):y,std::get<2>(points[i]):theta
      PolyState goal(std::get<0>(points[1]), std::get<1>(points[1]), std::get<2>(points[1]), v[1],
                     a[1]);
      //生成有效轨迹
      generation(start, goal, traj);
      //转换轨迹变成路径  
      Points2d path_i = traj.toPath();
      //将一个容器（path_i）中的元素添加到另一个容器（path）的末尾。
      path.insert(path.end(), path_i.begin(), path_i.end());
      //将轨迹转换成nav_msgs信息
      converttrajToNavPath(traj,desired_path);

      return !path.empty();
}



/**
 * @brief Configure the maximum acceleration.
 * @param max_acc  The maximum acceleration
 */
void Polynomial::setMaxAcceleration(double max_acc)
{
  assert(max_acc > 0);
  max_acc_ = max_acc;
}

/**
 * @brief Configure the maximum jerk.
 * @param max_jerk  The maximum jerk
 */
void Polynomial::setMaxJerk(double max_jerk)
{
  assert(max_jerk > 0);
  max_jerk_ = max_jerk;
}

}  // namespace trajectory_generation