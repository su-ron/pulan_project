
#include "common/line/line_utils.h"


namespace planning {

void LineUtils::CalculateS(std::vector<Point> &path) {
  if (path.empty()) {
    printf("CalculateS::Path is empty. ");
    return;
  }
  path.front().set_s(0.0);
  path.front().set_id(0);
  std::vector<Point> tmp_path{path.front()};
  for (int i = 1; i < path.size(); ++i) {
    if (path[i] - tmp_path.back() < 1e-3 &&
        (path[i].motion_info().reverse() ==
         tmp_path.back().motion_info().reverse())) {
      continue;
    }
    path[i].set_s(path[i] - tmp_path.back() + tmp_path.back().s());
    path[i].set_id(tmp_path.back().id() + 1);
    tmp_path.emplace_back(path[i]);
  }
  path.swap(tmp_path);
}

bool LineUtils::FindNearestIndex(const Point &point,
                                 const std::vector<Point> &line,
                                 int *target_index, double *target_distance,
                                 const bool &flag_use_yaw_constraint) {
  if (line.size() == 0) return false;

  double min_distance = std::numeric_limits<double>::max();
  int min_index = -1;

  for (int i = 0; i < line.size(); i++) {
    double d = point - line[i];
    if (d < min_distance) {
      if (flag_use_yaw_constraint) {  // 加入yaw角检查保护
        double yaw_diff = math::AngleDiff(point.yaw(), line[i].yaw());
        if (fabs(yaw_diff) < M_PI / 3.0) {
          min_distance = d;
          min_index = i;
        }
      } else {
        min_distance = d;
        min_index = i;
      }
    }
  }

  if (min_index == -1) {
    return false;
  }

  if (target_distance) {
    *target_distance = min_distance;
  }
  if (target_index) {
    *target_index = min_index;
  }
  return true;
}

bool LineUtils::FindIndexByLineSegmentIntersect(const Point &point,
                                                const std::vector<Point> &line,
                                                const bool is_left,
                                                int *target_index) {
  int size = line.size();
  if (size == 0) {
    return false;
  } else if (size == 1) {
    *target_index = 0;
    return true;
  }

  math::Vec2d current_vec;
  if (is_left) {
    current_vec = math::Vec2d::CreateUnitVec2d(point.yaw() + M_PI / 2);
  } else {
    current_vec = math::Vec2d::CreateUnitVec2d(point.yaw() - M_PI / 2);
  }
  current_vec *= 15;
  current_vec = current_vec + math::Vec2d(point.x(), point.y());
  math::LineSegment2d current_seg(current_vec,
                                  math::Vec2d(point.x(), point.y()));

  for (int i = 0; i + 1 < size; i++) {
    math::LineSegment2d line_seg(math::Vec2d(line[i].x(), line[i].y()),
                                 math::Vec2d(line[i + 1].x(), line[i + 1].y()));
    if (current_seg.HasIntersect(line_seg)) {
      *target_index = i;
      return true;
    }
  }

  return FindNearestIndex(point, line, target_index);
}

bool LineUtils::FindAddDsIndex(const int &index, const std::vector<Point> &line,
                               const double &ds, int *target_index) {
  // 检查index范围
  if (index < 0 || index >= line.size() || target_index == nullptr) {
    return false;
  }
  *target_index = index;
  // 目标的s
  double s = line[index].s() + ds;
  // 向前查找
  if (ds > 0) {
    for (; *target_index < line.size(); (*target_index)++) {
      if (line[*target_index].s() > s) {
        if (*target_index == 0) break;
        (*target_index)--;
        break;
      }
    }
    // 向后查找
  } else {
    for (; *target_index >= 0; (*target_index)--) {
      if (line[*target_index].s() < s) {
        if (*target_index == line.size() - 1) break;
        (*target_index)++;
        break;
      }
    }
  }
  // 如果没有满足目标距离的点，就会把端点返回
  if (*target_index == -1)
    *target_index = 0;
  else if (*target_index == line.size())
    *target_index = line.size() - 1;
  return true;
}

bool LineUtils::FindAddDsIndexByGear(const int &index,
                                     const std::vector<Point> &line,
                                     const double &ds, int *target_index) {
  // 检查index范围
  if (index < 0 || index >= line.size() || target_index == nullptr) {
    return false;
  }
  *target_index = index;
  // 目标的s
  double s = line[index].s() + ds;
  // index所在点的gear
  bool flag_closest_reverse = line[index].motion_info().reverse();
  // 向前查找
  if (ds > 0) {
    for (; *target_index < line.size(); (*target_index)++) {
      bool flag_cur_reverse = line[*target_index].motion_info().reverse();
      if (line[*target_index].s() > s ||
          flag_closest_reverse != flag_cur_reverse) {
        if (*target_index == 0) break;
        (*target_index)--;
        break;
      }
    }
    // 向后查找
  } else {
    for (; *target_index >= 0; (*target_index)--) {
      bool flag_cur_reverse = line[*target_index].motion_info().reverse();
      if (line[*target_index].s() < s ||
          flag_closest_reverse != flag_cur_reverse) {
        if (*target_index == line.size() - 1) break;
        (*target_index)++;
        break;
      }
    }
  }
  // 如果没有满足目标距离的点，就会把端点返回
  if (*target_index == -1)
    *target_index = 0;
  else if (*target_index == line.size())
    *target_index = line.size() - 1;
  return true;
}

bool LineUtils::FindProjPoint(const std::vector<Point> &ref_line,
                              const Point &cur_pose, int *const match_index,
                              Point *const proj_point,
                              const bool &flag_use_yaw_constraint) {
  if (ref_line.empty()) {
    printf("Refline is empty. Can't find ProjPoint! ");
    return false;
  }
  if (!FindNearestIndex(cur_pose, ref_line, match_index, nullptr,
                        flag_use_yaw_constraint)) {
    return false;
  }

  const Point &match_point = ref_line[*match_index];
  double x_v = cur_pose.x() - match_point.x();
  double y_v = cur_pose.y() - match_point.y();
  double ds = cos(match_point.yaw()) * x_v + sin(match_point.yaw()) * y_v;

  proj_point->set_x(match_point.x() + cos(match_point.yaw()) * ds);
  proj_point->set_y(match_point.y() + sin(match_point.yaw()) * ds);
  double yaw_r =
      math::NormalizeAngle(match_point.yaw() + match_point.kappa() * ds);
  proj_point->set_yaw(yaw_r);
  proj_point->set_s(match_point.s() + ds);

  // Calculate kappa and dkappa
  double r_kappa, r_dkappa;
  if (*match_index == 0 || *match_index == ref_line.size() - 1) {
    r_kappa = match_point.kappa();
    r_dkappa = match_point.dkappa();
  } else {
    if (ds > 0) {
      r_kappa = (ref_line[*match_index].kappa() +
                 ref_line[*match_index + 1].kappa()) /
                2.0;
      r_dkappa = (ref_line[*match_index + 1].kappa() -
                  ref_line[*match_index].kappa()) /
                 (ref_line[*match_index + 1].s() - ref_line[*match_index].s());
    } else {
      r_kappa = (ref_line[*match_index].kappa() +
                 ref_line[*match_index - 1].kappa()) /
                2.0;
      r_dkappa = (ref_line[*match_index].kappa() -
                  ref_line[*match_index - 1].kappa()) /
                 (ref_line[*match_index].s() - ref_line[*match_index - 1].s());
    }
  }

  proj_point->set_kappa(r_kappa);
  proj_point->set_dkappa(r_dkappa);

  return true;
}

bool LineUtils::GetNearestPointFromS(const std::vector<Point> &ref_line,
                                     const double &target_s,
                                     Point *const target_point) {
  if (ref_line.empty()) {
    printf("The reference line is empty");
    return false;
  }

  auto comparator = [](double val, const Point &p) { return val < p.s(); };

  auto iter =
      std::upper_bound(ref_line.begin(), ref_line.end(), target_s, comparator);

  Point desired_point;
  if (iter == ref_line.begin()) {
    desired_point = ref_line.front();
  } else if (iter == ref_line.end()) {
    desired_point = ref_line.back();
  } else {
    if (fabs((*iter).s() - target_s) < 1.0e-6) {
      desired_point = *iter;
    } else {
      Point cur_point = *iter;
      Point last_point = *(iter - 1);
      desired_point =
          LinearInterpolation::InterpolateByS(last_point, cur_point, target_s);
    }
  }

  (*target_point) = std::move(desired_point);
  return true;
}

bool LineUtils::XYToSL(const std::vector<Point> &ref_line,
                       const Point &target_pose, double *const s,
                       double *const l, const bool &flag_use_yaw_constraint) {
  int match_index;
  Point project_point;
  // ref_line is empty
  if (!FindProjPoint(ref_line, target_pose, &match_index, &project_point,
                     flag_use_yaw_constraint)) {
    return false;
  }

  double dx = target_pose.x() - project_point.x();
  double dy = target_pose.y() - project_point.y();

  const double cos_theta_r = std::cos(project_point.yaw());
  const double sin_theta_r = std::sin(project_point.yaw());
  const double cross_rd_nd = cos_theta_r * dy - sin_theta_r * dx;

  (*s) = project_point.s();
  (*l) = std::copysign(std::sqrt(dx * dx + dy * dy), cross_rd_nd);
  return true;
}

bool LineUtils::SplitPathByGear(const std::vector<Point> &origin_refline,
                                std::vector<std::vector<Point>> *refline_seg) {
  if (origin_refline.empty()) {
    printf("Empty origin path");
    return false;
  }
  refline_seg->clear();

  Point last_point = origin_refline.front();
  std::vector<Point> cur_seg({last_point});

  for (int i = 1; i < origin_refline.size(); ++i) {
    const auto &p = origin_refline[i];
    if (p.motion_info().reverse() != last_point.motion_info().reverse()) {
      refline_seg->emplace_back(cur_seg);
      cur_seg.clear();
    }
    cur_seg.emplace_back(p);

    last_point = p;
  }
  refline_seg->emplace_back(cur_seg);

  return true;
}

}  // namespace planning
