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
//时间t决定速度和加速度的大小,代表的是从起点到终点所使用的时间
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
  //根据得到的起点和终点，就能确认曲线的形状，那么曲线的具体形状是怎么样的呢？
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

//这几个函数是为了获得在某一点的位置
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
  //ROS_INFO("size()=%f", size());
  for (size_t i = 0; i < size(); i++) path.push_back({x_[i], y_[i]});
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
  initpose_received_ = false;
  initpose_changed_ = false;
  goal_received_ = false;
  goal_changed_ = false;
  _pub_polynomialtraj = nh_.advertise<nav_msgs::Path>(
      "/desired_path", 1);  // MPC trajectory output
  _sub_goal   = nh_.subscribe( "/move_base_simple/goal", 1, &Polynomial::goalCB, this);
  _sub_initpose   = nh_.subscribe("/initialpose", 5, &Polynomial::initposeCB, this);

  marker_pub_ =
      nh_.advertise<visualization_msgs::Marker>("/polynomial_path_marker", 10);

  // Visualization Marker Settings
  initMarker();
  
}
Polynomial::Polynomial() : Curve(2.0), max_acc_(3.0), max_jerk_(1.0)
{
  initpose_received_ = false;
  initpose_changed_ = false;
  goal_received_ = false;
  goal_changed_ = false;
  _pub_polynomialtraj = nh_.advertise<nav_msgs::Path>(
      "/desired_path", 1);  // MPC trajectory output
  _sub_goal   = nh_.subscribe( "/move_base_simple/goal", 1, &Polynomial::goalCB, this);
  _sub_initpose   = nh_.subscribe("/initialpose", 5, &Polynomial::initposeCB, this);
  marker_pub_ =
      nh_.advertise<visualization_msgs::Marker>("/polynomial_path_marker", 10);
  // Visualization Marker Settings
  initMarker();
}

/**
 * @brief Destroy the Polynomial generation object
 */
Polynomial::~Polynomial()
{
}

void Polynomial::initMarker() {
  points.header.frame_id =mid_points.header.frame_id = line_strip.header.frame_id 
  =forward_line_strip.header.frame_id =goal_circle.header.frame_id = "odom";

  points.ns =mid_points.ns = line_strip.ns =forward_line_strip.ns = goal_circle.ns = "Markers";

  points.action = mid_points.action = line_strip.action = forward_line_strip.action = goal_circle.action =
      visualization_msgs::Marker::ADD;
  points.pose.orientation.w = mid_points.pose.orientation.w = line_strip.pose.orientation.w 
  = forward_line_strip.pose.orientation.w =goal_circle.pose.orientation.w = 1.0;

  points.id = 0;
  line_strip.id = 1;
  goal_circle.id = 2;
  mid_points.id = 3;
  forward_line_strip.id = 4;

  points.type = visualization_msgs::Marker::POINTS;
  mid_points.type = visualization_msgs::Marker::POINTS;
  line_strip.type = visualization_msgs::Marker::LINE_STRIP;
  forward_line_strip.type = visualization_msgs::Marker::LINE_STRIP;
  goal_circle.type = visualization_msgs::Marker::CYLINDER;
  // POINTS markers use x and y scale for width/height respectively
  points.scale.x = 0.2;
  points.scale.y = 0.2;

  // LINE_STRIP markers use only the x component of scale, for the line width
  line_strip.scale.x = 0.1;
  forward_line_strip.scale.x = 0.1;


  // Points are red
  points.color.r = 1.0f;
  points.color.a = 1.0;
  

  // Line strip is blue
  line_strip.color.b = 1.0;
  line_strip.color.a = 1.0;
  
  //mid_points 
  mid_points.color.b = 1.0;
  mid_points.color.a = 1.0;

  //forward_line_strip is red
  forward_line_strip.color.r = 1.0;
  forward_line_strip.color.a = 1.0;


  // goal_circle is yellow
  goal_circle.color.r = 1.0;
  goal_circle.color.g = 1.0;
  goal_circle.color.b = 0.0;
  goal_circle.color.a = 0.5;
}

void Polynomial::initposeCB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
  ROS_INFO("initpose Received ");
  ROS_INFO("Received pose message with frame_id: %s", msg->header.frame_id.c_str());
  initpose_received_ = true;
  initpose_changed_ = true;
  initpose_x_ = msg->pose.pose.position.x;
  initpose_y_=msg->pose.pose.position.y;
  initpose_yaw_ = tf::getYaw(msg->pose.pose.orientation);

  initpose_map_.header.stamp =ros::Time::now(); 
  initpose_map_.header.frame_id = "map";
  initpose_map_.pose.position = msg->pose.pose.position;
  initpose_map_.pose.orientation = msg->pose.pose.orientation;

  try {
        tf_listener_.transformPose("odom", ros::Time(0) , 
                                            initpose_map_,"map",initpose_odom_);   
      } catch (tf::TransformException& ex) {
        ROS_ERROR("1111 Failed to transform pose: %s", ex.what());
      }

    try {
        tf_listener_.transformPose("base_link", ros::Time(0) , 
                                            initpose_map_,"map",initpose_base_);   
      } catch (tf::TransformException& ex) {
        ROS_ERROR("1111 Failed to transform pose: %s", ex.what());
      }
      ROS_INFO(
          "initpose_map_.pose.position.x=%f,initpose_map_.pose.position.y=%f",
          initpose_map_.pose.position.x, initpose_map_.pose.position.y);
      ROS_INFO(
          "initpose_base_.pose.position.x=%f,initpose_base_.pose.position.y=%f",
          initpose_base_.pose.position.x, initpose_base_.pose.position.y);
      // double theta=msg->pose.pose.orientation;
}

// CallBack: Update goal status
void Polynomial::goalCB(const geometry_msgs::PoseStamped::ConstPtr& goalMsg)
{
  ROS_INFO("goal Received ");
  goal_received_ = true;
  goal_changed_ = true;
  goal_pos_ = goalMsg->pose.position;
  goal_x_ = goalMsg->pose.position.x;
  goal_y_ = goalMsg->pose.position.y;


  goalpose_map_.header.stamp =ros::Time::now(); 
  goalpose_map_.header.frame_id = "map";
  goalpose_map_.pose.position = goalMsg->pose.position;
  goalpose_map_.pose.orientation =goalMsg->pose.orientation;
    try {
        // 将姿态信息从base_frame_转换到目标坐标系
        tf_listener_.transformPose("odom", ros::Time(0) , 
                                            goalpose_map_,"map",goalpose_odom_);   
      } catch (tf::TransformException& ex) {
        ROS_ERROR("[%s][%d] Failed to transform pose: %s",__func__,__LINE__, ex.what());
      }
  
  try {
        // 将姿态信息从base_frame_转换到目标坐标系
        tf_listener_.transformPose("base_link", ros::Time(0) , 
                                            goalpose_map_,"map",goalpose_base_);   
      } catch (tf::TransformException& ex) {
        ROS_ERROR("2222 Failed to transform pose: %s", ex.what());
      }
      ROS_INFO(
          "goalpose_map_.pose.position.x=%f,goalpose_map_.pose.position.y=%f",
          goalpose_map_.pose.position.x, goalpose_map_.pose.position.y);
      ROS_INFO(
          "goalpose_base_.pose.position.x=%f,goalpose_base_.pose.position.y=%f",
          goalpose_base_.pose.position.x, goalpose_base_.pose.position.y);
  //中点位置
  midpointpose_base_.header.stamp =ros::Time::now(); 
  midpointpose_base_.header.frame_id = "base_link";
  midpointpose_base_.pose.position.x = goalpose_base_.pose.position.x/2 ;
  midpointpose_base_.pose.position.y = 0;
  midpointpose_base_.pose.position.z = 0;
  midpointpose_base_.pose.orientation = goalMsg->pose.orientation;
    try {
        // 将姿态信息从base_frame_转换到目标坐标系
        tf_listener_.transformPose("map", ros::Time(0) , 
                                            midpointpose_base_,"base_link",midpointpose_map_);   
      } catch (tf::TransformException& ex) {
        ROS_ERROR("3333 Failed to transform pose: %s", ex.what());
      }

    try {
        // 将姿态信息从base_frame_转换到目标坐标系
        tf_listener_.transformPose("odom", ros::Time(0) , 
                                            midpointpose_map_,"map",midpointpose_odom_);   
      } catch (tf::TransformException& ex) {
        ROS_ERROR("3333 Failed to transform pose: %s", ex.what());
      }
      ROS_INFO(
          "midpointpose_map_.pose.position.x=%f,midpointpose_map_.pose.position.y=%f",
          midpointpose_map_.pose.position.x, midpointpose_map_.pose.position.y);
      ROS_INFO(
          "midpointpose_base_.pose.position.x=%f,midpointpose_base_.pose.position.y=%f",
          midpointpose_base_.pose.position.x, midpointpose_base_.pose.position.y);

  pointpose_y_2_base_.header.stamp =ros::Time::now(); 
  pointpose_y_2_base_.header.frame_id = "base_link";
  pointpose_y_2_base_.pose.position.x = goalpose_base_.pose.position.x;
  pointpose_y_2_base_.pose.position.y = goalpose_base_.pose.position.y/2;
  pointpose_y_2_base_.pose.position.z = 0;
  pointpose_y_2_base_.pose.orientation = goalMsg->pose.orientation;

    try {
        // 将姿态信息从base_frame_转换到目标坐标系
        tf_listener_.transformPose("map", ros::Time(0) , 
                                            pointpose_y_2_base_,"base_link",pointpose_y_2_map_);   
      } catch (tf::TransformException& ex) {
        ROS_ERROR("3333 Failed to transform pose: %s", ex.what());
      }

    try {
        // 将姿态信息从base_frame_转换到目标坐标系
        tf_listener_.transformPose("odom", ros::Time(0) , 
                                            pointpose_y_2_map_,"map",pointpose_y_2_odom_);   
      } catch (tf::TransformException& ex) {
        ROS_ERROR("3333 Failed to transform pose: %s", ex.what());
      }
  
  /*Visualized Target Point on RVIZ*/
  /*Clear former target point Marker*/
  points.points.clear();
  geometry_msgs::Point waypoints;
  
  waypoints = initpose_odom_.pose.position;
  points.points.push_back(waypoints);
  waypoints = goalpose_odom_.pose.position;
  points.points.push_back(waypoints);
  waypoints = midpointpose_odom_.pose.position;
  points.points.push_back(waypoints);
  waypoints = pointpose_y_2_odom_.pose.position;
  points.points.push_back(waypoints);
  
  marker_pub_.publish(points);
  

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
  // ROS_INFO("sx=%f,sy=%f,syaw=%f,sv=%f,sa=%f",sx, sy,syaw,sv,sa);
  // ROS_INFO("gx=%f,gy=%f,gyaw=%f,gv=%f,ga=%f",gx, gy,gyaw,gv,ga);
  //起点速度和终点速度
  double sv_x = sv * cos(syaw);
  double sv_y = sv * sin(syaw);
  double gv_x = gv * cos(gyaw);
  double gv_y = gv * sin(gyaw);
  //ROS_INFO("sv_x=%f,sv_y=%f,syaw=%f,gv_x=%f,gv_y=%f,gyaw=%f",sv_x, sv_y,syaw, gv_x, gv_y,gyaw);
  // 起点位置和终点位置
  double sa_x = sa * cos(syaw);
  double sa_y = sa * sin(syaw);
  double ga_x = ga * cos(gyaw);
  double ga_y = ga * sin(gyaw);
  //ROS_INFO("sa_x=%f,sa_y=%f,ga_x=%f,ga_y=%f",sa_x, sa_y, ga_x, ga_y);
  traj.clear();

  double T = t_min;
  while (T < t_max)
  {
    //先以t_min来计算五次多项式，如果这样计算出来的曲线速度与加速度太大，那么T增加step_.
    //多项式中，x,y均为t的函数
    Poly x_psolver({ sx, sv_x, sa_x }, { gx, gv_x, ga_x }, T);
    Poly y_psolver({ sy, sv_y, sa_y }, { gy, gv_y, ga_y }, T);
    double t = 0.0;
    double vx, vy, v, yaw, ax, ay, a, jx, jy, j;
    while (t < T + dt) {
       vx = x_psolver.dx(t);
       vy = y_psolver.dx(t);
       v = hypot(vx, vy);
       //yaw角是由当前x,y方向的速度来决定的
       yaw = atan2(vy, vx);

       ax = x_psolver.ddx(t);
       ay = y_psolver.ddx(t);
       a = hypot(ax, ay);
       a = traj.dir(POLY_DIR_ACC) ? a * traj.dir(POLY_DIR_ACC) : a;

       jx = x_psolver.dddx(t);
       jy = y_psolver.dddx(t);
       j = hypot(jx, jy);
       j = traj.dir(POLY_DIR_JERK) ? j * traj.dir(POLY_DIR_JERK) : j;
      //将状态附加到多项式轨迹
      traj.append(t, x_psolver.x(t), y_psolver.x(t), v, yaw, a, j);
      // ROS_INFO("vx=%f,vy=%f,v=%f,yaw=%f,ax=%f,ay=%f,a=%f,jx=%f,jy=%f,j=%f", vx,
      //        vy, v, yaw, ax, ay, a, jx, jy, j);
      //以0.5s为一个时间间隔添加轨迹点
      t += dt;
    }
    //检查生成的多项式轨迹是否满足给定的最大加速度和最大 jerk 的限制。
    if (traj.valid(max_acc_, max_jerk_)){
    // ROS_INFO("-----traj.valid-----");
    // ROS_INFO("vx=%f,vy=%f,v=%f,yaw=%f,ax=%f,ay=%f,a=%f,jx=%f,jy=%f,j=%f", vx,
    //         vy, v, yaw, ax, ay, a, jx, jy, j);
    // ROS_INFO("traj.size()=%f", traj.size());
    break;
      }
    else{
    //ROS_INFO("-----traj.NOT valid-----");
      traj.clear();
    }
    //step_=2.0
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
 * @brief Running trajectory generations
 * @param points path points <x, y>
 * @param path generated trajectory
 * @return true if generate successfully, else failed
 */
bool Polynomial::run(const Points2d points, Points2d& path,nav_msgs::Path&desired_path)
{
    //ROS_INFO("--------3---------");
    Poses2d poses;
    //输入位置和位姿
    //poses.emplace_back(points.begin()->first, points.begin()->second, initpose_yaw_);
    poses.emplace_back(points.begin()->first, points.begin()->second,0);
    //ROS_INFO("initpose_yaw_=%f", initpose_yaw_);
    // poses包含x,y,theta,theta通过插值获取
    // double theta1 = helper::angle(points[0], points[1]);
    //只有起点和终点时，不会进入该循环
    for (size_t i = 1; i < points.size() - 1; i++) {
      //ROS_INFO("-------7----------");
      double theta1 = atan2(points[i].second - points[i-1].second,
                          points[i].first - points[i-1].first);
      double theta2 = atan2(points[i+1].second - points[i].second,
                          points[i+1].first - points[i].first);
      poses.emplace_back(points[i].first, points[i].second, (theta1 + theta2) / 2);
    }
    poses.emplace_back(points.back().first, points.back().second, 0);
    return run(poses, path,desired_path);
}

/*需要将得到的Points2d& path信息转化成nav_msgs::Path信息才能给mpc算法调用
*
*/
 bool Polynomial::converttrajToNavPath(PolyTrajectory&traj,nav_msgs::Path &nav_path)
{
    //ROS_INFO("-----------------");
    nav_path.header.stamp = ros::Time::now();
    nav_path.header.frame_id = "map"; // 设置frame_id
    double size = traj.size();
    //ROS_INFO("convert traj.size()=%f", traj.size());
    traj.getinformation();
    for (double i = 0; i < size; i++) {
      geometry_msgs::PoseStamped pose_stamped;
      pose_stamped.header.stamp = ros::Time::now();
      pose_stamped.header.frame_id = "map"; 
      pose_stamped.pose.position.x = traj.x[i];
      pose_stamped.pose.position.y = traj.y[i];
      pose_stamped.pose.position.z = 0;  // 可根据实际情况设置
      // 添加四元数
    // 将四元数转换为geometry_msgs::Quaternion类型
    pose_stamped.pose.orientation= tf::createQuaternionMsgFromYaw(traj.yaw[i]);
    
    nav_path.poses.push_back(pose_stamped);
    }
    //ROS_INFO("nav_path.poses.size()=%zu", nav_path.poses.size());
    return !nav_path.poses.empty();
}

//只是存在两个点，起点和终点
/**
 * @brief Running trajectory generations
 * 在轨迹规划过程中，只需要遍历路径点，在相邻路径点间进行插值即可
 * @param points path points <x, y, theta>
 * @param path generated trajectory
 * @return true if generate successfully, else failed
 */
bool Polynomial::run(const Poses2d points, Points2d& path,nav_msgs::Path&desired_path)
{
    path.clear();
    // generate velocity and acceleration constraints heuristically
    //启发式生成速度和加速度约束
    //创建了一个 std::vector<double> 类型的向量 v，其长度为 points.size()，并且初始化每个元素的值为 1.0。
    std::vector<double> v(points.size(), 1.0);
    //起点终点速度设置为0
    v[0] = 0.0;
    v[points.size() - 1] = 0;

    std::vector<double> a;
    //获取加速度
    for (size_t i = 0; i < points.size() - 1; i++)
       a.push_back(0.0);
      // a.push_back((v[i + 1] - v[i]) / 5);//加速度为什么这么取
    a.push_back(0.0);
    
    // /ROS_INFO("points.size()=%zu",points.size());
    for (size_t i = 0; i < points.size() - 1; i++) {
      //ROS_INFO("----------5----------");
      PolyTrajectory traj;
      nav_msgs::Path nav_path;
      //ROS_INFO("std::get<0>(points[i])=%f,std::get<1>(points[i])=%f,std::get<2>(points[i])=%f",std::get<0>(points[i]),std::get<1>(points[i]),std::get<2>(points[i]));
      PolyState start(std::get<0>(points[i]), std::get<1>(points[i]), std::get<2>(points[i]), v[i], a[i]);
      PolyState goal(std::get<0>(points[i+1]), std::get<1>(points[i+1]), std::get<2>(points[i+1]), v[i+1],a[i+1]);
      // 生成有效轨迹,问题出在这里？
      generation(start, goal, traj);
      //转换轨迹变成路径  
      Points2d path_i = traj.toPath();
      //将一个容器（path_i）中的元素添加到另一个容器（path）的末尾。
      path.insert(path.end(), path_i.begin(), path_i.end());
      // /ROS_INFO("path.size()=%zu", path.size());
      //将轨迹转换成nav_msgs信息
      if (!converttrajToNavPath(traj, nav_path)) break;
      // ROS_INFO("nav_path.poses.size()=%zu", nav_path.poses.size());
      desired_path.header.stamp = ros::Time::now();
      desired_path.header.frame_id = "map"; // 设置frame_id
      desired_path.poses.insert(desired_path.poses.end(),
                                nav_path.poses.begin(), nav_path.poses.end());                      
    }
    //ROS_INFO("Desired path frame ID: %s", desired_path.header.frame_id.c_str());
    _pub_polynomialtraj.publish(desired_path);
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

/*****************/
/* MAIN FUNCTION */
/*****************/
int main(int argc, char **argv)
{
    //Initiate ROS
    ros::init(argc, argv, "path_converter_node");
    ros::NodeHandle nh;
    // 假设已有 Points2d path
    //Points2d points = {{0.0, 0.0},{5.0,5.0}};
    trajectory_generation::Polynomial polynomial;
    // ROS_INFO("polynomial.initpose_received_: %s", polynomial.initpose_received_ ? "true" : "false");
    // ROS_INFO("polynomial.goal_received_: %s", polynomial.goal_received_ ? "true" : "false");
    while(ros::ok()){
    if(polynomial.initpose_received_&&polynomial.goal_received_)
    {
      polynomial.goal_received_=false;
      Points2d points = {{polynomial.initpose_x_, polynomial.initpose_y_},
                         {polynomial.midpointpose_map_.pose.position.x,
                          polynomial.midpointpose_map_.pose.position.y},
                         {polynomial.pointpose_y_2_map_.pose.position.x,
                          polynomial.pointpose_y_2_map_.pose.position.y},
                         {polynomial.goal_x_, polynomial.goal_y_}};
      //Points2d points = {{polynomial.initpose_x_, polynomial.initpose_y_},{polynomial.goal_x_, polynomial.goal_y_}};
      Points2d path;
      nav_msgs::Path desired_path;
      polynomial.run(points, path, desired_path);
      for (size_t i = 0; i < desired_path.poses.size(); i++) {
        double yaw = tf::getYaw(desired_path.poses[i].pose.orientation);
    }
     }
    ros::spinOnce();
    }
    return 0;
}

