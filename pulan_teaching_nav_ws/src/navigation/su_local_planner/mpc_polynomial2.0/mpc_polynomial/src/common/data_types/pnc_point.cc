
#include "common/data_types/pnc_point.h"

#include <iostream>

namespace data_types {

RoadInfo::RoadInfo(const int &direction, const double &speed_limit_mps,
                   const bool &allow_u_turn, const bool &allow_avoidance,
                   const bool &allow_lane_change,
                   const bool &need_to_lift_brush, const bool &allow_edgewise,
                   const std::string &lane_id)
    : direction_(direction),
      speed_limit_mps_(speed_limit_mps),
      allow_u_turn_(allow_u_turn),
      allow_avoidance_(allow_avoidance),
      allow_lane_change_(allow_lane_change),
      need_to_lift_brush_(need_to_lift_brush),
      allow_edgewise_(allow_edgewise),
      lane_id_(lane_id) {}

void RoadInfo::set_direction(const int &direction) { direction_ = direction; }
void RoadInfo::set_speed_limit_mps(const double &speed_limit_mps) {
  speed_limit_mps_ = speed_limit_mps;
}
void RoadInfo::set_u_turn(const bool &allow_u_turn) {
  allow_u_turn_ = allow_u_turn;
}
void RoadInfo::set_avoidance(const bool &avoidance) {
  allow_avoidance_ = avoidance;
}
void RoadInfo::set_lane_change(const bool &lane_change) {
  allow_lane_change_ = lane_change;
}
void RoadInfo::set_lift_brush(const bool &lift_brush) {
  need_to_lift_brush_ = lift_brush;
}
void RoadInfo::set_segment_end(const bool &seg_end) {
  is_segment_end_ = seg_end;
}
void RoadInfo::set_edgewise(const bool &edegwise) {
  allow_edgewise_ = edegwise;
}
void RoadInfo::set_lane_id(const std::string &lane_id) { lane_id_ = lane_id; }
void RoadInfo::set_attached_lane_id(const std::string &attached_lane_id) {
  attached_lane_id_ = attached_lane_id;
}
void RoadInfo::set_s_on_lane(const double &s_on_lane) {
  s_on_lane_ = s_on_lane;
}
void RoadInfo::set_lane_type(const int &lane_type) { lane_type_ = lane_type; }
void RoadInfo::set_is_rerouting(const bool &is_rerouting) {
  is_rerouting_ = is_rerouting;
}
void RoadInfo::set_is_slope(const bool &is_slope) { is_slope_ = is_slope; }
void RoadInfo::set_slope(const double &slope) { slope_ = slope; }
void RoadInfo::set_is_winding(const bool &is_winding) {
  is_winding_ = is_winding;
}
void RoadInfo::set_is_low_obs(const bool &is_low_obs) {
  is_low_obs_ = is_low_obs;
}
void RoadInfo::set_low_obs_height(const double &low_obs_height) {
  low_obs_height_ = low_obs_height;
}

void PncPoint::NormalizeAngle() { yaw_ = math::NormalizeAngle(yaw_); }

PncPoint PncPoint::Multiply(const PncPoint &other_point) const {
  PncPoint res;
  res.set_x(other_point.x() * cos(yaw_) - other_point.y() * sin(yaw_) + x_);
  res.set_y(other_point.x() * sin(yaw_) + other_point.y() * cos(yaw_) + y_);
  res.set_yaw(yaw_ + other_point.yaw());
  res.NormalizeAngle();
  return res;
}

PncPoint PncPoint::Inverse() const {
  Eigen::Matrix4d mat_d, mat_d2;

  mat_d.setZero();

  mat_d.data()[0] = cos(yaw_);
  mat_d.data()[4] = -sin(yaw_);
  mat_d.data()[12] = x_;
  mat_d.data()[1] = sin(yaw_);
  mat_d.data()[5] = cos(yaw_);
  mat_d.data()[13] = y_;
  mat_d.data()[10] = 1.0f;
  mat_d.data()[15] = 1.0f;

  mat_d2 = mat_d.inverse();

  return PncPoint(mat_d2.data()[12], mat_d2.data()[13],
                  std::atan2(mat_d2.data()[1], mat_d2.data()[0]));  // A1/A0
}

}  // namespace data_types
