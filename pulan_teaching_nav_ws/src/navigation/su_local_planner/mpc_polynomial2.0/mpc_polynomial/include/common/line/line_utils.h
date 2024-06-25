#pragma once

#include <vector>

#include "common/utils/linear_interpolation.h"
#include "common/data_types/pnc_point.h"
#include "common/math/math_utils.h"
#include "common/math/line_segment2d.h"
namespace planning {

typedef data_types::PncPoint Point;

/**
 * @brief 提供关于线的基本操作，包括路线，参考线等
 * */

class LineUtils {
 public:
  LineUtils() = default;
  ~LineUtils() = default;

 public:
  /**
   * @brief 计算路径中每个点相对于起始点的累计距离
   * */
  static void CalculateS(std::vector<Point>& path);

  /**
   * @brief 在一条线上找到距离目标点最近点的索引
   * */
  static bool FindNearestIndex(const Point& point,
                               const std::vector<Point>& line,
                               int* target_index,
                               double* target_distance = nullptr,
                               const bool& flag_use_yaw_constraint = false);

  /**
   * @brief 在一条线上找到与输入点左右垂直方向上的线段相交的索引
   * */
  static bool FindIndexByLineSegmentIntersect(const Point& point,
                                              const std::vector<Point>& line,
                                              const bool is_left,
                                              int* target_index);

  /**
   * @brief 在当前索引的点加了ds的距离之后，目标点在这条线上的索引
   * */
  static bool FindAddDsIndex(const int& index, const std::vector<Point>& line,
                             const double& ds, int* target_index);

  /**
   * @brief 在当前索引的点加了ds的距离且gear不变，目标点在这条线上的索引
   * */
  static bool FindAddDsIndexByGear(const int& index,
                                   const std::vector<Point>& line,
                                   const double& ds, int* target_index);

  /**FindProjPoint
   * @brief 在给定的一条线上找到投影点
   * @details 点的属性包括坐标姿态和曲率及导数
   * */
  static bool FindProjPoint(const std::vector<Point>& ref_line,
                            const Point& cur_pose, int* const match_index,
                            Point* const proj_point,
                            const bool& flag_use_yaw_constraint = false);

  /**
   * @brief 找到给定线上指定s的对应点
   * @details 采用二分查找，找到第一个大于等于目标距离的点
   * @details 点的属性包括坐标姿态和曲率及导数，s, v, acc, t
   * */
  static bool GetNearestPointFromS(const std::vector<Point>& ref_line,
                                   const double& target_s,
                                   Point* const target_point);

  /**
   * @brief 将(x,y)投影到参考线上，得到s, l
   * @details l的方向，正为左侧，负为右侧
   * @details 曲率大时可能存在误差
   * */
  static bool XYToSL(const std::vector<Point>& ref_line,
                     const Point& target_pose, double* const s, double* const l,
                     const bool& flag_use_yaw_constraint = false);

  /**
   * @brief 根据路径的档位把路径拆分，同档位的放在一段
   *
   * @param origin_refline 原路径
   * @param refline_seg 拆分后的路径
   */
  static bool SplitPathByGear(const std::vector<Point>& origin_refline,
                              std::vector<std::vector<Point>>* refline_seg);
};

}  // namespace planning
