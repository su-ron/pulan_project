/***********************************************************
 *
 * @file: bezier_curve.cpp
 * @breif: Bezier curve generation
 * @author: Yang Haodong
 * @update: 2023-12-22
 * @version: 1.0
 *
 * Copyright (c) 2023, Yang Haodong
 * All rights reserved.
 * --------------------------------------------------------
 *
 **********************************************************/
#include <cassert>
#include "bezier_curve.h"

namespace trajectory_generation
{
/**
 * @brief Construct a new Bezier generation object
 * @param step        Simulation or interpolation size (default: 0.1)
 * @param offset      The offset of control points (default: 3.0)
 */
Bezier::Bezier(double step, double offset) : Curve(step), offset_(offset)
{
}
Bezier::Bezier() : Curve(0.1), offset_(3)
{
}

/**
 * @brief Destroy the Bezier generation object
 */
Bezier::~Bezier()
{
}

/**
 * @brief Calculate the Bezier curve point.
 * @param t scale factor
 * @param control_pts control points
 * @return point point in Bezier curve with t
//  * 问题，生成的贝塞尔曲线路径点的偏航角yaw怎么取？double yaw = std::atan2(tangent_vector.y, tangent_vector.x);？
// 假设deriveBezier函数用来计算贝塞尔曲线的导数
Point2d tangent_vector = deriveBezier(t);
// Calculate the yaw angle from the tangent vector
double yaw = std::atan2(tangent_vector.y, tangent_vector.x);

 */
Point2d Bezier::bezier(double t, Points2d control_pts)
{
  size_t n = control_pts.size() - 1;
  Point2d pt(0, 0);
  //把n-1视为n
  for (size_t i = 0; i < n + 1; i++)
  {
    pt.first += _comb(n, i) * std::pow(t, i) * std::pow(1 - t, n - i) * control_pts[i].first;
    pt.second += _comb(n, i) * std::pow(t, i) * std::pow(1 - t, n - i) * control_pts[i].second;
  }
  return pt;
}

//贝塞尔曲线求导，为了得到点的yaw角
Point2d Bezier::bezierDerivative(double t, Points2d control_pts)
{
  //n代表贝塞尔曲线的控制点数-1
  size_t n = control_pts.size() - 1;
  Point2d pt(0, 0);
  for (size_t i = 1; i < n + 2; i++)
  {
    pt.first += _comb(n, i-1) * std::pow(t, i) * std::pow(1 - t, n+1 - i) * (n+1)*(control_pts[i].first-control_pts[i-1].first);
    pt.second += _comb(n, i-1) * std::pow(t, i) * std::pow(1 - t, n+1 - i) * (n+1)*(control_pts[i].second-control_pts[i-1].second);
  }
  return pt;
}

/**
 * @brief Calculate control points heuristically.  启发式计算控制点。
 * @param start Initial pose (x, y, yaw)
 * @param goal  Target pose (x, y, yaw)
 * @return control_pts control points 最终，control_pts 包含四个控制点，这些点可用于生成三次贝塞尔曲线。函数返回这些控制点。
 */
Points2d Bezier::getControlPoints(Pose2d start, Pose2d goal)
{
  double sx, sy, syaw;
  double gx, gy, gyaw;
  std::tie(sx, sy, syaw) = start;
  std::tie(gx, gy, gyaw) = goal;

  double d = dist(Point2d(sx, sy), Point2d(gx, gy)) / offset_;

  Points2d control_pts;
  control_pts.emplace_back(sx, sy);
  control_pts.emplace_back(sx + d * cos(syaw), sy + d * sin(syaw));
  control_pts.emplace_back(gx - d * cos(gyaw), gy - d * sin(gyaw));
  control_pts.emplace_back(gx, gy);

  return control_pts;
}

/**
 * @brief Generate the path. 
 * @param start Initial pose (x, y, yaw)
 * @param goal  Target pose (x, y, yaw)
 * @return path The smoothed trajectory points
 */
Poses2d Bezier::generation(Pose2d start, Pose2d goal)
{
  double sx, sy, syaw;
  double gx, gy, gyaw;
  std::tie(sx, sy, syaw) = start;
  std::tie(gx, gy, gyaw) = goal;
  
  //这里有点不理解
  //step_ 仿真步长 ，决定得到贝塞尔曲线组成点的个数
  int n_points = (int)(dist(Point2d(sx, sy), Point2d(gx, gy)) / step_);
  //通过起始点和终点获得另外两个控制点
  Points2d control_pts = getControlPoints(start, goal);

  Poses2d poses;
  for (size_t i = 0; i < n_points; i++) {
    double t = (double)(i) / (double)(n_points - 1);
    // 假设deriveBezier函数用来计算贝塞尔曲线的导数
    Point2d tangent_vector = bezierDerivative(t,control_pts);
    double yaw = std::atan2(tangent_vector.second, tangent_vector.first);
    Point2d point=bezier(t,control_pts);
    poses.emplace_back(point.first, point.second, yaw);
  }

  return poses;
}

/**
 * @brief Running trajectory generation
 * @param points path points <x, y>
 * @param path generated trajectory
 * @return true if generate successfully, else failed
 */
bool Bezier::run(const Points2d points, Poses2d& path)
{
  if (points.size() < 4)
    return false;
  else
  {
    Poses2d poses;
    poses.emplace_back(points.begin()->first, points.begin()->second, 0);
    //这里只是给每一段贝塞尔曲线的起点和终点赋偏航角yaw
    for (size_t i = 1; i < points.size() - 1; i++)
    {
      double theta1 = angle(points[i - 1], points[i]);
      double theta2 = angle(points[i], points[i + 1]);
      poses.emplace_back(points[i].first, points[i].second, (theta1 + theta2) / 2);
    }
    poses.emplace_back(points.back().first, points.back().second, 0);

    return run(poses, path);
  }
}


/*需要将得到的Points2d& path信息转化成nav_msgs::Path信息才能给mpc算法调用
*
*/
 bool Bezier::converttrajToNavPath(Poses2d& path,nav_msgs::Path &nav_path)
{
    //ROS_INFO("-----------------");
    nav_path.header.stamp = ros::Time::now();
    nav_path.header.frame_id = "map"; // 设置frame_id

    for (Poses2d::iterator i= path.begin(); i !=path.end(); i++) {
      geometry_msgs::PoseStamped pose_stamped;
      pose_stamped.header.stamp = ros::Time::now();
      pose_stamped.header.frame_id = "map"; 
      pose_stamped.pose.position.x = std::get<0>(*i);
      pose_stamped.pose.position.y = std::get<1>(*i);
      pose_stamped.pose.position.z = 0;  // 可根据实际情况设置
      // 添加四元数，贝塞尔曲线获得的路径点只有坐标，没有yaw角
    // 将四元数转换为geometry_msgs::Quaternion类型
    pose_stamped.pose.orientation= tf::createQuaternionMsgFromYaw(std::get<2>(*i));
    
    nav_path.poses.push_back(pose_stamped);
    }
    //ROS_INFO("nav_path.poses.size()=%zu", nav_path.poses.size());
    return !nav_path.poses.empty();
}

/**
 * @brief Running trajectory generation
 * @param points path points <x, y, theta>
 * @param path generated trajectory
 * @return true if generate successfully, else failed
 */
bool Bezier::run(const Poses2d points, Poses2d& path)
{
  if (points.size() < 4)
    return false;
  else
  {
    //当输入的点大于4的时候，那么每两个点间就会生成一段贝塞尔曲线
    path.clear();
    for (size_t i = 0; i < points.size() - 1; i++)
    {
      //根据起点和终点生成路径点
      Poses2d path_i = generation(points[i], points[i + 1]);
      path.insert(path.end(), path_i.begin(), path_i.end());

      nav_msgs::Path nav_path,desired_path;
      if (!converttrajToNavPath(traj, nav_path)) break;
      // ROS_INFO("nav_path.poses.size()=%zu", nav_path.poses.size());
      desired_path.header.stamp = ros::Time::now();
      desired_path.header.frame_id = "map"; // 设置frame_id
      desired_path.poses.insert(desired_path.poses.end(),
                                nav_path.poses.begin(), nav_path.poses.end());   
    }
    return !path.empty();
  }
}

/**
 * @brief Configure the offset of control points.
 * @param offset  The offset of control points
 */
void Bezier::setOffset(double offset)
{
  assert(offset > 0);
  offset_ = offset;
}

// Calculate the number of combinations
/*这个函数计算的是二项式系数，也被称为组合数。
在数学上，组合数计算了从n个不同元素中选择r个元素的方法数，无论这些元素的排列如何，都被认定为同一组合。
这个公式意味着：从n个元素中选择r个元素的组合数
等于从n-1个元素中选择r-1个元素的组合数与从n-1个元素中选择r个元素的组合数之和。
*/
int Bezier::_comb(int n, int r)
{
  if ((r == 0) || (r == n))
    return 1;
  else
    return _comb(n - 1, r - 1) + _comb(n - 1, r);
}

double Bezier::dist(const std::pair<double, double>& node1, const std::pair<double, double>& node2)
{
  return std::hypot(node1.first - node2.first, node1.second - node2.second);
}

double Bezier::angle(const std::pair<double, double>& node1, const std::pair<double, double>& node2)
{
  return atan2(node2.second - node1.second, node2.first - node1.first);
}
}  // namespace trajectory_generation


/*****************/
/* MAIN FUNCTION */
/*****************/
int main(int argc, char **argv)
{
    //Initiate ROS
    ros::init(argc, argv, "bezier_converter_node");
    ros::NodeHandle nh;
    // 假设已有 Points2d path
    //Points2d points = {{0.0, 0.0},{5.0,5.0}};
    trajectory_generation::Bezier bezier;
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