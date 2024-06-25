
#include "common/path/path_generator.h"

namespace planning {
  
PathGenerator::PathGenerator()
{
   initparam();
}

void PathGenerator::initparam()
{
  if (initial_status_) {
    return;
  }
  ros::NodeHandle nh;
  marker_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/polynomial_path_marker", 10,true);
  refline_marker_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/refline_polynomial_path_marker", 10,true);
  initial_status_ = true;
}

void PathGenerator::publishReflineMarker(const std::vector<Point>&desired_path)
{

  visualization_msgs::Marker marker;
  marker.header.frame_id = frame_id_;
  marker.header.stamp = ros::Time::now();
  marker.ns ="Refline_Markers";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::ARROW;
  marker.action = visualization_msgs::Marker::ADD;

  marker.scale.x = 0.1;
  marker.scale.y = 0.05;
  marker.scale.z = 0.05;

  marker.color.a = 1.0; // Don't forget to set the alpha!
  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  visualization_msgs::MarkerArray marray;

  int size = desired_path.size();
  int tag = 0;
  for (int j = 0; j < size; j++) {
    marker.header.stamp = ros::Time::now();
    marker.id = tag++;
    marker.pose.position.x = desired_path[j].x();
    marker.pose.position.y = desired_path[j].y();
    marker.pose.position.z = 0;
    tf::Quaternion q;
    q.setRPY(0, 0, desired_path[j].yaw());
    marker.pose.orientation.x = q.x();
    marker.pose.orientation.y = q.y();
    marker.pose.orientation.z = q.z();
    marker.pose.orientation.w = q.w();
    marker.lifetime = ros::Duration();
    marray.markers.emplace_back(marker);
  }
  refline_marker_pub_.publish(marray);
}

void PathGenerator::publishMarker(const std::vector<Point>&desired_path)
{

  visualization_msgs::Marker marker;
  marker.header.frame_id = frame_id_;
  marker.header.stamp = ros::Time::now();
  marker.ns ="Markers";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::ARROW;
  marker.action = visualization_msgs::Marker::ADD;

  marker.scale.x = 0.1;
  marker.scale.y = 0.05;
  marker.scale.z = 0.05;

  marker.color.a = 1.0; // Don't forget to set the alpha!
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  visualization_msgs::MarkerArray marray;
  marker_pub_.publish(marray);
  int size = desired_path.size();
  int tag = 0;
  for (int j = 0; j < size; j++) {
    marker.header.stamp = ros::Time::now();
    marker.id = tag++;
    marker.pose.position.x = desired_path[j].x();
    marker.pose.position.y = desired_path[j].y();
    marker.pose.position.z = 0;
    tf::Quaternion q;
    q.setRPY(0, 0, desired_path[j].yaw());
    marker.pose.orientation.x = q.x();
    marker.pose.orientation.y = q.y();
    marker.pose.orientation.z = q.z();
    marker.pose.orientation.w = q.w();
    marker.lifetime = ros::Duration();
    marray.markers.emplace_back(marker);
  }
  marker_pub_.publish(marray);
}

double PathGenerator::fx(const double& s) {
    return -1.2e-5 * pow(s, 5) + 4.73e-4 * pow(s, 4) - 8.037e-4 * pow(s, 3) -
           9.78e-2 * pow(s, 2) - 2.08e-3 * s + 5;
}

double PathGenerator::dfx(const double& s) {
    return -1.2e-5 * pow(s, 4) * 5 + 4.73e-4 * pow(s, 3) * 4 -
           8.037e-4 * pow(s, 2) * 3 - 9.78e-2 * s * 2 - 2.08e-3;
}

double PathGenerator::ddfx(const double& s) {
    return -1.2e-5 * pow(s, 3) * 20 + 4.73e-4 * pow(s, 2) * 12 -
           8.037e-4 * s * 6 - 9.78e-2;
}

double PathGenerator::dddfx(const double& s) {
    return -1.2e-5 * pow(s, 2) * 60 + 4.73e-4 * s * 24 - 8.037e-4 * 6;
}

double PathGenerator::fy(const double& s) {
    return 2.94e-4 * pow(s, 4) - 9.237e-3 * pow(s, 3) + 9.687e-3 * pow(s, 2) +
           0.987 * s + 7.56e-4;
}

double PathGenerator::dfy(const double& s) {
    return 2.94e-4 * pow(s, 3) * 4 - 9.237e-3 * pow(s, 2) * 3 +
           9.687e-3 * s + 0.987;
}

double PathGenerator::ddfy(const double& s) {
    return 2.94e-4 * pow(s, 2) * 12 - 9.237e-3 * s * 6 + 9.687e-3;
}

double PathGenerator::dddfy(const double& s) {
    return 2.94e-4 * s * 24 - 9.237e-3 * 6;
}

double PathGenerator::ComputeCurvature(const double& dx, const double& ddx,
                        const double& dy, const double& ddy) {
    double a = dx*ddy - dy*ddx;
    double norm_square = dx*dx+dy*dy;
    double norm = sqrt(norm_square);
    double b = norm*norm_square;
    return 2 * a / b;
}

double PathGenerator::ComputeCurvatureDerivative(
    const double& dx, const double& ddx, const double& dddx,
    const double& dy, const double& ddy, const double& dddy) {
    double a = dx*ddy-dy*ddx;
    double b = dx*dddy-dy*dddx;
    double c = dx*ddx+dy*ddy;
    double d = dx*dx+dy*dy;
    return (b*d-3.0*a*c)/pow((d*d*d), 2.5);
}

bool PathGenerator::GenerateRefline(std::vector<Point>&ref_line)
{
    double length = 5 * M_PI;
    double interval = length / 100;
    for (double s = 0.0; s <= length; s += interval) {
      double rx = fx(s);
      double ry = fy(s);
      double ryaw = atan2(dfy(s), dfx(s));
      double rkappa = ComputeCurvature(dfx(s), ddfx(s), dfy(s), ddfy(s));
      double rdkappa =
          ComputeCurvatureDerivative(dfx(s), ddfx(s), dddfx(s),
                                     dfy(s), ddfy(s), dddfy(s));

      Point tmp_point;
      tmp_point.set_x(rx);
      tmp_point.set_y(ry);
      tmp_point.set_yaw(ryaw);
      tmp_point.set_s(s);
      tmp_point.set_kappa(rkappa);
      tmp_point.set_dkappa(rdkappa);

      ref_line.push_back(tmp_point);
    }
    publishReflineMarker(ref_line);
}

bool PathGenerator::GenerateLineRefline(const Point& start_point,
                                        const Point& target_point,
                                        std::vector<Point>&ref_line)

{
  ref_line.clear();
  double resolution = 0.1;
  double search_length = 0.0;
  // double max_search_lenght = hypot(start_point.x()-target_point.x(),start_point.y()-target_point.y()) + 1.0;
  double max_search_lenght = hypot(start_point.x()-target_point.x(),start_point.y()-target_point.y()) ;
  double yaw = math::NormalizeAngle(atan2(target_point.y() - start_point.y(),target_point.x() - start_point.x()));
  while (search_length <= max_search_lenght) {
      Point tmp_point;
      tmp_point.set_x(start_point.x() + search_length * std::cos(yaw));
      tmp_point.set_y(start_point.y() + search_length * std::sin(yaw));
      tmp_point.set_s(search_length);
      tmp_point.set_yaw(yaw);
      tmp_point.set_kappa(0.0);
      tmp_point.set_dkappa(0.0);
      ref_line.emplace_back(tmp_point);
      search_length += resolution;
  }
  publishReflineMarker(ref_line);
  return true;
}

/*
   输入的起点和终点的坐标系是什么，输出的路径的坐标系就是什么
*/
bool PathGenerator::GenerateFrenetBasedSpline(
    const Point &start_point, const Point &target_point,
    const std::vector<Point> &ref_line, std::vector<Point> *desired_path,
    const double &allowed_max_kappa, double *actual_max_kappa_ptr) {
  // 若参考线为空，则将起点与终点连线作为参考线
  std::vector<Point> new_ref_line;
  if (ref_line.empty()) {
    GenerateLineRefline(start_point, target_point, new_ref_line);
  } else {
    new_ref_line = ref_line;
  }
 // Find Projected point of the start point and the end point
  Point start_proj_pose, target_proj_point;
  int nearest_start_index, nearest_target_index;
  if (!LineUtils::FindProjPoint(new_ref_line, start_point, &nearest_start_index,
                                &start_proj_pose) ||
      !LineUtils::FindProjPoint(new_ref_line, target_point, &nearest_target_index,
                                &target_proj_point)) {
    ROS_WARN("FindProjPoint error " );
    return false;
  }
  double start_angle_diff =
      math::AngleDiff(start_proj_pose.yaw(), start_point.yaw());
  double target_angle_diff =
      math::AngleDiff(target_proj_point.yaw(), target_point.yaw());
  double angle_diff_thresh_rad = 60.0 * M_PI / 180.0;
  if ((fabs(start_angle_diff) > angle_diff_thresh_rad) ||
      fabs(target_angle_diff) > angle_diff_thresh_rad) {
    // 输入点与投影点角度差过大，生成路径可能有问题
    ROS_WARN("AngleDiff error : %f : %f" ,start_angle_diff,target_angle_diff);
    return false;
  }
  // 查找车辆位置
  double vehicle_s = 0.0, vehicle_l = 0.0;
  if (!LineUtils::XYToSL(new_ref_line, start_point, &vehicle_s, &vehicle_l)) {
    ROS_WARN("Cannot calculate the sl boundary\n");
    return false;
  } else {
    ROS_INFO("Cannot calculate the sl boundary , vehicle_s %f",vehicle_s);
    const double SAFE_DISTANCE = 0.2;
    if (vehicle_s < new_ref_line.front().s() - SAFE_DISTANCE ||
        vehicle_s > new_ref_line.back().s() + SAFE_DISTANCE) {
      //  当前车的位置不在参考线范围内
      ROS_WARN("vehicle point is out of refline's range\n");
      return false;
    }
  }

  // Calculate the condition of the start point and the target point
  std::array<double, 3> start_s, target_s;
  std::array<double, 3> start_l, target_l;
  CartesianFrenetConverter::cartesian_to_frenet(
      start_proj_pose.s(), start_proj_pose.x(), start_proj_pose.y(),
      start_proj_pose.yaw(), start_proj_pose.kappa(), start_proj_pose.dkappa(),
      start_point.x(), start_point.y(),
      start_point.motion_info().linear_velocity(),
      start_point.motion_info().a(), start_point.yaw(), start_point.kappa(),
      &start_s, &start_l);

  CartesianFrenetConverter::cartesian_to_frenet(
      target_proj_point.s(), target_proj_point.x(), target_proj_point.y(),
      target_proj_point.yaw(), target_proj_point.kappa(),
      target_proj_point.dkappa(), target_point.x(), target_point.y(),
      target_point.motion_info().linear_velocity(),
      target_point.motion_info().a(), target_point.yaw(), target_point.kappa(),
      &target_s, &target_l);

  // Generate the fifth order spline based on frenet frame
  std::array<double, 3> start{start_l[0], start_l[1], start_l[2]};
  std::array<double, 3> target{target_l[0], target_l[1], target_l[2]};
  QuinticPolynomial spline(start, target, target_s[0] - start_s[0]);

  double actual_max_kappa = -1;
  // Convert the point from frenet to cartesian frame
  ROS_INFO("target s: %f, start s : %f, diff: %f",target_s[0] , start_s[0],target_s[0] - start_s[0]);
  for (double ds = 0.0; ds <= target_s[0] - start_s[0]; ds += 0.05) {
    double s = start_proj_pose.s() + ds;
    Point target_ref_point;
    LineUtils::GetNearestPointFromS(new_ref_line, s, &target_ref_point);

    double l = spline.Evaluate(0, ds);
    double dl = spline.Evaluate(1, ds);
    double ddl = spline.Evaluate(2, ds);

    double x, y, yaw, kappa, v, a;
    std::array<double, 3> s_condition{s, 0.0, 0.0};
    std::array<double, 3> d_condition{l, dl, ddl};
    //从frenet坐标系转回cartesian坐标系
    CartesianFrenetConverter::frenet_to_cartesian(
        target_ref_point.s(), target_ref_point.x(), target_ref_point.y(),
        target_ref_point.yaw(), target_ref_point.kappa(),
        target_ref_point.dkappa(), s_condition, d_condition, &x, &y, &yaw,
        &kappa, &v, &a);

    Point tmp_point;
    tmp_point.set_x(x);
    tmp_point.set_y(y);
    tmp_point.set_yaw(yaw);
    tmp_point.set_kappa(kappa);

    if (fabs(kappa) > allowed_max_kappa) {
      desired_path->clear();
      ROS_WARN("Cannot calculate allowed_max_kappa");
      return false;
    }
    if (fabs(kappa) > actual_max_kappa) {
      actual_max_kappa = fabs(kappa);
    }
    desired_path->emplace_back(tmp_point);
  }

  if (desired_path->empty()) {
    ROS_WARN("desired_path is empty");
    return false;
  }

  if (actual_max_kappa_ptr) {
    (*actual_max_kappa_ptr) = actual_max_kappa;
  }
  LineUtils::CalculateS(*desired_path);
  publishMarker(*desired_path);
  return true;
}

/*需要将得到的std::vector<Point> *desired_path信息转化成nav_msgs::Path信息才能给mpc算法调用
*
*/
 bool PathGenerator::converttrajToNavPath(const std::vector<Point> &desired_path,nav_msgs::Path &nav_path)
{
    //ROS_INFO("-----------------");
    nav_path.header.stamp = ros::Time::now();
    nav_path.header.frame_id = frame_id_; // 设置frame_id
    ROS_INFO("convert desired_path.size()=%zu",desired_path.size());
    for (size_t i = 0; i < desired_path.size(); i++) {
      geometry_msgs::PoseStamped pose_stamped;
      pose_stamped.header.stamp = ros::Time::now();
      pose_stamped.header.frame_id = frame_id_;
      pose_stamped.pose.position.x = desired_path[i].x();
      pose_stamped.pose.position.y = desired_path[i].y();
      pose_stamped.pose.position.z = 0;  // 可根据实际情况设置
      // 添加四元数
    // 将四元数转换为geometry_msgs::Quaternion类型
    pose_stamped.pose.orientation= tf::createQuaternionMsgFromYaw(desired_path[i].yaw());
    
    nav_path.poses.push_back(pose_stamped);
    }
    ROS_INFO("nav_path.poses.size()=%zu", nav_path.poses.size());
    return !nav_path.poses.empty();
}
}  // namespace planning

