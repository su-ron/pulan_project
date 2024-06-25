
#pragma once

#include <limits>
#include <vector>

#include "common/line/line_utils.h"
#include "common/math/cartesian_frenet_conversion.h"
#include "common/math/quintic_polynomial.h"
#include "common/data_types/pnc_point.h"
#include "common/math/curve_math.h"
#include "common/math/math_utils.h"

#include "visualization_msgs/MarkerArray.h"
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Path.h>

namespace planning {


typedef data_types::PncPoint Point;

/**
 * @brief 提供生成路径的操作
 * */

class PathGenerator {
 public:
  PathGenerator() ;
  ~PathGenerator() = default;

 public:
  void initparam();
  void initposeCB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
  void goalCB(const geometry_msgs::PoseStamped::ConstPtr& goalMsg);
  void initMarker();
  void publishMarker(const std::vector<Point>& desired_path);
  double fx(const double& s);
  double dfx(const double& s);
  double ddfx(const double& s);
  double dddfx(const double& s);
  double fy(const double& s);
  double dfy(const double& s);
  double ddfy(const double& s);
  double dddfy(const double& s);
  double ComputeCurvature(const double& dx, const double& ddx,
                                         const double& dy, const double& ddy);
  double ComputeCurvatureDerivative(
      const double& dx, const double& ddx, const double& dddx, const double& dy,
      const double& ddy, const double& dddy);
  bool GenerateRefline(std::vector<Point>& ref_line);

  /**
   * @brief 基于Frenet坐标系生成五次多项式，最后再转回笛卡尔坐标系
   * @details
   * 首先把起点和终点转到frenet坐标系，然后基于两个端点生成五次多项式，最后再转到笛卡尔坐标系
   * @param allowed_max_kappa 曲线上的点允许的最大曲率
   * @param actual_max_kappa_ptr 曲线上的点允许的最大曲率，绝对值
   * */
  static bool GenerateFrenetBasedSpline(
      const Point& start_point, const Point& target_point,
      const std::vector<Point>& ref_line, std::vector<Point>* desired_path,
      const double& allowed_max_kappa = std::numeric_limits<double>::infinity(),
      double* actual_max_kappa_ptr = nullptr);

  bool converttrajToNavPath(const std::vector<Point>& desired_path, nav_msgs::Path& nav_path);


  ros::NodeHandle nh_;
  ros::Publisher marker_pub_;
  ros::Subscriber _sub_initpose,_sub_goal;
  visualization_msgs::Marker marker_;
  Point start_pose_, target_pose_;
  bool initpose_received_,goal_received_;
  tf::TransformListener tf_listener_;
  geometry_msgs::PoseStamped initpose_map_, initpose_odom_;
  geometry_msgs::PoseStamped goalpose_map_, goalpose_odom_;
};

}  // namespace planning
