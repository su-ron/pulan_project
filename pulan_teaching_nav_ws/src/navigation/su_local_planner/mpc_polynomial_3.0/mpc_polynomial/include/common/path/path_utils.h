
#pragma once

#include "Eigen/Dense"
#include "common/data_types/pnc_point.h"
#include "common/math/lowpass_filter.h"

namespace planning {

typedef data_types::PncPoint Point;

using Matrix = Eigen::MatrixXd;

/**
 * @brief 提供关于路径的基本操作
 * */

class PathUtils {
 public:
  PathUtils() = default;
  ~PathUtils() = default;

 public:
  /**
   * @brief 道路沿着路径最后一个点的朝向角延长一定距离
   * @param distance 延长的距离
   * @param initial_path 传进来的路径
   * @param direction 1是向前面延长，-1是沿着后面延长
   */
  static void ExtendAlong(const double& distance,
                          std::vector<Point>* initial_path,
                          const double& interval = 0.1,
                          const double& direction = 1);

  /**
   * @brief 估算路径上每一个点的曲率
   * @param max_kappa 最大允许曲率
   * @param use_filter 是否用滤波器去平滑估算的曲率
   * */
  static void EstimateKappa(const double& max_kappa, std::vector<Point>* path,
                            bool use_filter = true);
};

}  // namespace planning
