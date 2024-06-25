
#include "common/math/transform.h"

namespace planning {

std::vector<double> Transform::ProjectTo(const Point &source_point,
                                         const Point &target_point) {
  double dx = source_point.x() - target_point.x();
  double dy = source_point.y() - target_point.y();
  double d_yaw = math::NormalizeAngle(source_point.yaw() - target_point.yaw());
  double distance = source_point - target_point;

  double target_yaw = target_point.yaw();
  double longitude_error = dx * cos(target_yaw) + dy * sin(target_yaw);
  double lateral_error = -dx * sin(target_yaw) + dy * cos(target_yaw);

  return {dx, dy, d_yaw, distance, longitude_error, lateral_error};
}

void Transform::VehicleToOdom(const Point &vehicle_point,
                              const Point &odom_point, Point *target_point) {
  double x = vehicle_point.x();
  double y = vehicle_point.y();
  double yaw = odom_point.yaw();

  double target_x = x * cos(yaw) - y * sin(yaw) + odom_point.x();
  double target_y = x * sin(yaw) + y * cos(yaw) + odom_point.y();
  double target_yaw = math::NormalizeAngle(yaw + vehicle_point.yaw());

  target_point->set_x(target_x);
  target_point->set_y(target_y);
  target_point->set_yaw(target_yaw);
}

void Transform::OdomToVehicle(const Point &vehicle_point,
                              const Point &odom_point, Point *target_point) {
  double dx = odom_point.x() - vehicle_point.x();
  double dy = odom_point.y() - vehicle_point.y();
  double yaw = vehicle_point.yaw();

  double target_x = dx * cos(yaw) + dy * sin(yaw);
  double target_y = dx * sin(yaw) - dy * cos(yaw);
  double target_yaw = math::NormalizeAngle(odom_point.yaw() - yaw);

  target_point->set_x(target_x);
  target_point->set_y(target_y);
  target_point->set_yaw(target_yaw);
}

void Transform::VehicleToOdom(const std::vector<Point> &vehicle_path,
                              const Point &odom_point,
                              std::vector<Point> *target_path) {
  target_path->clear();

  for (auto &vehicle_point : vehicle_path) {
    Point target_point;
    VehicleToOdom(vehicle_point, odom_point, &target_point);
    target_path->push_back(target_point);
  }

  // Recalculate yaw to ensure accuracy
  unsigned int path_size = target_path->size();
  if (path_size < 3) return;

  for (int i = 1; i < path_size - 1; ++i) {
    double dx =
        target_path->operator[](i + 1).x() - target_path->operator[](i - 1).x();
    double dy =
        target_path->operator[](i + 1).y() - target_path->operator[](i - 1).y();
    double yaw = atan2(dy, dx);
    target_path->operator[](i).set_yaw(yaw);
  }
  target_path->operator[](0).set_yaw(target_path->operator[](1).yaw());
  target_path->back().set_yaw(target_path->operator[](path_size - 2).yaw());
}

void Transform::DriftToOdom(const double &lon_drift, const double &lat_drift,
                            const Point &base_point, Point *target_point) {
  double x = lon_drift;
  double y = lat_drift;
  double yaw = base_point.yaw();

  double target_x = x * cos(yaw) - y * sin(yaw) + base_point.x();
  double target_y = x * sin(yaw) + y * cos(yaw) + base_point.y();
  double target_yaw = math::NormalizeAngle(yaw);

  *target_point = base_point;
  target_point->set_x(target_x);
  target_point->set_y(target_y);
  target_point->set_yaw(target_yaw);
}

}  // namespace planning

