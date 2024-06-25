
#pragma once

#include <math.h>
#include <stdint.h>
#include <tf/tf.h>

#include <string>
#include <unordered_map>

#include "common/math/math_utils.h"
#include "nav_msgs/Odometry.h"

namespace data_types {

class RoadInfo {
 public:
  RoadInfo(const int &direction = 1, const double &speed_limit_mps = 2.5,
           const bool &allow_u_turn = false,
           const bool &allow_avoidance = false,
           const bool &allow_lane_change = false,
           const bool &need_to_lift_brush = false,
           const bool &allow_edgewise = false, const std::string &lane_id = "");

  const int direction() const { return direction_; }
  const double speed_limit_mps() const { return speed_limit_mps_; }
  const bool u_turn() const { return allow_u_turn_; }
  const bool avoidance() const { return allow_avoidance_; }
  const bool lane_change() const { return allow_lane_change_; }
  const bool lift_brush() const { return need_to_lift_brush_; }
  const bool segment_end() const { return is_segment_end_; }
  const bool edgewise() const { return allow_edgewise_; }
  const std::string lane_id() const { return lane_id_; }
  const std::string attached_lane_id() const { return attached_lane_id_; }
  const double s_on_lane() const { return s_on_lane_; }
  const int lane_type() const { return lane_type_; }
  const bool is_rerouting() const { return is_rerouting_; }
  const bool is_slope() const { return is_slope_; }
  const double slope() const { return slope_; }
  const bool is_winding() const { return is_winding_; }
  const bool is_low_obs() const { return is_low_obs_; }
  const double low_obs_height() const { return low_obs_height_; }

  void set_direction(const int &direction);
  void set_speed_limit_mps(const double &speed_limit_mps);
  void set_u_turn(const bool &allow_u_turn);
  void set_avoidance(const bool &avoidance);
  void set_lane_change(const bool &lane_change);
  void set_lift_brush(const bool &lift_brush);
  void set_segment_end(const bool &seg_end);
  void set_edgewise(const bool &edegwise);
  void set_lane_id(const std::string &lane_id);
  void set_attached_lane_id(const std::string &attached_lane_id);
  void set_s_on_lane(const double &s_on_lane);
  void set_lane_type(const int &lane_type);
  void set_is_rerouting(const bool &is_rerouting);

  void set_is_slope(const bool &is_slope);
  void set_slope(const double &slope);
  void set_is_winding(const bool &is_winding);
  void set_is_low_obs(const bool &is_low_obs);
  void set_low_obs_height(const double &low_obs_height);

 private:
  int direction_ = 1;  // 道路方向，1为forward，2为backward，3为bidirection
  double speed_limit_mps_ = 2.5;     // 道路限速
  bool allow_u_turn_ = false;        // 是否允许调头
  bool allow_avoidance_ = false;     // 是否允许避障
  bool allow_lane_change_ = false;   // 是否允许变道
  bool need_to_lift_brush_ = false;  // 是否需要抬扫刷
  bool is_segment_end_ = false;  // 是否是当前段的终点，用于路径分段的标识符
  bool allow_edgewise_ = false;  // 是否允许沿边
  std::string lane_id_ = "";     // 所属lane的id，不在lane上时为空
  std::string attached_lane_id_ = "";  // 关联lane的id，在lane上时为空
  double s_on_lane_ = -1.0;            // 在所属道路上的s
  int lane_type_ = 0;  // 所属道路类型，0为正常道路，1为人行道，2为调头路段
  bool is_rerouting_ = false;    // 是否是重规划路线点
  bool is_slope_ = false;        // 是否是坡道
  double slope_ = 0.0;           // 坡度
  bool is_winding_ = false;      // 是否有缠绕物
  bool is_low_obs_ = false;      // 是否有低矮障碍物
  double low_obs_height_ = 0.0;  // 低矮障碍物的高度
};

class MotionInfo {
 public:
  MotionInfo() {}
  MotionInfo(const bool &brake, const double &linear_velocity,
             const double &a = 0, const double &t = 0,
             const double &angular_velocity = 0)
      : emergency_brake_(brake),
        linear_v_mps_(linear_velocity),
        a_(a),
        t_(t),
        angular_v_(angular_velocity) {}

  const bool emergency_brake() const { return emergency_brake_; }
  const double linear_velocity() const { return linear_v_mps_; }
  const double a() const { return a_; }
  const double t() const { return t_; }
  const double angular_velocity() const { return angular_v_; }
  const bool reverse() const { return is_reverse_; }

  void set_emergency_brake(const bool &brake) { emergency_brake_ = brake; }
  void set_linear_velocity(const double &linear_velocity) {
    linear_v_mps_ = linear_velocity;
  }
  void set_a(const double &a) { a_ = a; }
  void set_t(const double &t) { t_ = t; }
  void set_angular_velocity(const double &angular_velocity) {
    angular_v_ = angular_velocity;
  }
  void set_reverse(const bool &reverse) { is_reverse_ = reverse; }

 private:
  bool emergency_brake_ = false;  // 急停
  double linear_v_mps_ = 0.0;     // 线速度，正为前进，负为倒车
  bool is_reverse_ = false;       // 倒车
  double a_ = 0.0;
  double t_ = 0.0;
  double angular_v_ = 0.0;
};

class PncPoint {
 public:
  PncPoint() {}
  PncPoint(const double &x, const double &y, const double &yaw)
      : x_(x), y_(y), yaw_(yaw_ = math::NormalizeAngle(yaw)) {}
  PncPoint(const double &x, const double &y, const double &z, const double &yaw)
      : PncPoint(x, y, yaw) {
    z_ = z;
  }
  explicit PncPoint(const nav_msgs::Odometry &point)
      : x_(point.pose.pose.position.x),
        y_(point.pose.pose.position.y),
        yaw_(math::NormalizeAngle(tf::getYaw(point.pose.pose.orientation))) {}
  // explicit PncPoint(const autocity_msgs::PlanningTrajectoryPoint tp);
  ~PncPoint() {}

  // ToMsg(autocity_msgs::PlanningTrajectoryPoint* msg_obj);

  const int id() const { return id_; }
  const double x() const { return x_; }
  const double y() const { return y_; }
  const double dx() const { return dx_; }
  const double dy() const { return dy_; }
  const double ddx() const { return ddx_; }
  const double ddy() const { return ddy_; }
  const double z() const { return z_; }
  const double yaw() const { return yaw_; }
  const double s() const { return s_; }
  const double ds() const { return ds_; }
  const double dds() const { return dds_; }
  const double l() const { return l_; }
  const double dl() const { return dl_; }
  const double ddl() const { return ddl_; }
  const double kappa() const { return kappa_; }
  const double dkappa() const { return dkappa_; }
  const double ddkappa() const { return ddkappa_; }
  const RoadInfo road_info() const { return road_info_; }
  RoadInfo *const mutable_road_info() { return &road_info_; }
  const MotionInfo motion_info() const { return motion_info_; }
  MotionInfo *const mutable_motion_info() { return &motion_info_; }

  void set_id(const int &id) { id_ = id; }
  void set_x(const double &x) { x_ = x; }
  void set_y(const double &y) { y_ = y; }
  void set_dx(const double &dx) { dx_ = dx; }
  void set_dy(const double &dy) { dy_ = dy; }
  void set_ddx(const double &ddx) { ddx_ = ddx; }
  void set_ddy(const double &ddy) { ddy_ = ddy; }
  void set_z(const double &z) { z_ = z; }
  void set_yaw(const double &yaw) {
    yaw_ = yaw;
    NormalizeAngle();
  }
  void set_s(const double &s) { s_ = s; }
  void set_ds(const double &ds) { ds_ = ds; }
  void set_dds(const double &dds) { dds_ = dds; }
  void set_l(const double &l) { l_ = l; }
  void set_dl(const double &dl) { dl_ = dl; }
  void set_ddl(const double &ddl) { ddl_ = ddl; }
  void set_kappa(const double &kappa) { kappa_ = kappa; }
  void set_dkappa(const double &dkappa) { dkappa_ = dkappa; }
  void set_ddkappa(const double &ddkappa) { ddkappa_ = ddkappa; }

  /**
   * @brief RoadInfo赋值函数，所有属性都赋值
   */
  void set_road_info(const RoadInfo &road_info) { road_info_ = road_info; }

  /**
   * @brief RoadInfo赋值函数，针对点的属性不赋值，重置为默认值
   */
  void set_road_info_partial(const RoadInfo &road_info) {
    road_info_ = road_info;
    road_info_.set_segment_end(false);
  }
  void set_motion_info(const MotionInfo &motion_info) {
    motion_info_ = motion_info;
  }

  void NormalizeAngle();
  PncPoint Multiply(const PncPoint &other_point) const;
  PncPoint Inverse() const;

  /**
   * @brief 计算两点距离
   */
  template <typename P>
  double operator-(const P &p) const {
    return sqrt((x_ - p.x()) * (x_ - p.x()) + (y_ - p.y()) * (y_ - p.y()));
  }

 private:
  int id_ = 0;
  // 几何属性
  double x_ = 0.0;
  double y_ = 0.0;
  double dx_ = 0.0;
  double dy_ = 0.0;
  double ddx_ = 0.0;
  double ddy_ = 0.0;
  double z_ = 0.0;
  double yaw_ = 0.0;
  double s_ = 0.0;
  double ds_ = 0.0;
  double dds_ = 0.0;
  double l_ = 0.0;
  double dl_ = 0.0;
  double ddl_ = 0.0;
  double kappa_ = 0.0;
  double dkappa_ = 0.0;
  double ddkappa_ = 0.0;
  // 道路属性
  RoadInfo road_info_;
  // 运动属性
  MotionInfo motion_info_;
};
}  // namespace data_types
