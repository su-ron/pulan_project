#pragma once

#include <vector>

#include "common/data_types/pnc_point.h"

namespace planning {

typedef data_types::PncPoint Point;

/**
 * @brief 提供各种坐标系转换
 * */
class Transform {
 public:
  Transform() = default;
  ~Transform() = default;

 public:
  /**
   * @brief 把target_point投影到source_point上
   * @return 返回值依次包含dx, dy, yaw角误差，欧式距离，纵向误差和横向误差
   * */
  static std::vector<double> ProjectTo(const Point& source_point,
                                       const Point& target_point);

  /**
   * @brief 把基于车身坐标系的点转到全局坐标系
   * @param vehicle_point 基于车身坐标系的点
   * @param odom_point 车子当前基于全局坐标系的位置
   * @param target_point 转换后基于全局坐标系的点
   * */
  static void VehicleToOdom(const Point& vehicle_point, const Point& odom_point,
                            Point* target_point);

  /**
   * @brief 把基于全局坐标系的点转到车身坐标系
   * @param vehicle_point 待转换的点，当前基于全局坐标系位置
   * @param odom_point 车子当前基于全局坐标系的位置
   * @param target_point 转换后点，基于车身坐标系
   * */
  static void OdomToVehicle(const Point& vehicle_point, const Point& odom_point,
                            Point* target_point);

  /**
   * @brief 把基于车身坐标系的路径转到全局坐标系
   * @param vehicle_path 基于车身坐标系的路径
   * @param odom_point 车子当前基于全局坐标系的位置
   * @param target_path 转换后基于全局坐标系的路径
   * */
  static void VehicleToOdom(const std::vector<Point>& vehicle_path,
                            const Point& odom_point,
                            std::vector<Point>* target_path);

  /**
   * @brief 把偏移量转到全局坐标系
   * @param drift 偏移量 左正右负
   * @param base_point 转换的原点
   * @param target_point 转换后基于全局坐标系的点
   * */
  static void DriftToOdom(const double& lon_drift, const double& lat_drift,
                          const Point& base_point, Point* target_point);
};

}  // namespace planning

