
#pragma once

#include <limits>
#include <vector>

#include "common/line/line_utils.h"
#include "common/math/cartesian_frenet_conversion.h"
#include "common/math/quintic_polynomial.h"
#include "common/data_types/pnc_point.h"
#include "common/math/curve_math.h"
#include "common/math/math_utils.h"

#include <visualization_msgs/MarkerArray.h>
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
  static PathGenerator& Instance() {
    static PathGenerator planning_generator;
    return planning_generator;
  }
  PathGenerator() ;
  ~PathGenerator() = default;
  void initparam();
  void publishReflineMarker(const std::vector<Point>&desired_path);
  void publishMarker(const std::vector<Point>& desired_path);
  bool GenerateFrenetBasedSpline(
      const Point& start_point, const Point& target_point,
      const std::vector<Point>& ref_line, std::vector<Point>* desired_path,
      const double& allowed_max_kappa = std::numeric_limits<double>::infinity(),
      double* actual_max_kappa_ptr = nullptr);

  bool converttrajToNavPath(const std::vector<Point>& desired_path, nav_msgs::Path& nav_path);

  private:
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
  bool GenerateRefline(std::vector<Point>&ref_line);
  bool GenerateLineRefline(const Point& start_point, const Point& target_point,std::vector<Point>&ref_line);



  ros::Publisher marker_pub_;
  ros::Publisher refline_marker_pub_;
  std::string  frame_id_ = "map";
  // std::string  frame_id_ = "odom";
  bool initial_status_ = false;
};

}  // namespace planning
