/***********************************************************
 *
 * @file: pid_planner.cpp
 * @breif: Contains the Proportional–Integral–Derivative (PID) controller local
 *planner class
 * @author: Yang Haodong, Guo Zhanyu, Wu Maojia
 * @update: 2023-10-1
 * @version: 1.1
 *
 * Copyright (c) 2023，Yang Haodong
 * All rights reserved.
 * --------------------------------------------------------
 *
 **********************************************************/

#include "pid_planner.h"

namespace pid_planner {

/**
 * @brief Construct a new PIDPlanner object
 */
PIDPlanner::PIDPlanner()
    : initialized_(false), tf_listener_(tf_buffer_), plan_index_(0) {
  if (!initialized_) {
    initialized_ = true;
    tf2_ros::TransformListener tf_listener(tf_);
    // costmap_ros_ = costmap_ros;

    // ros::NodeHandle nh("~");
    ros::NodeHandle nh;
    nh.param("/pid_planner/PIDPlanner/p_window", p_window_, 0.5);

    nh.param("/pid_planner/PIDPlanner/goal_dist_tolerance", goal_dist_tol_,
             0.5);
    nh.param("/pid_planner/PIDPlanner/rotate_tolerance", rotate_tol_, 0.5);

    nh.param("/pid_planner/PIDPlanner/max_v", max_v_, 1.0);
    nh.param("/pid_planner/PIDPlanner/min_v", min_v_, 0.0);
    nh.param("/pid_planner/PIDPlanner/max_v_inc", max_v_inc_, 0.5);

    nh.param("/pid_planner/PIDPlanner/max_w", max_w_, 1.57);
    nh.param("/pid_planner/PIDPlanner/min_w", min_w_, 0.0);
    nh.param("/pid_planner/PIDPlanner/max_w_inc", max_w_inc_, 1.57);

    nh.param("/pid_planner/PIDPlanner/k_v_p", k_v_p_, 1.00);
    nh.param("/pid_planner/PIDPlanner/k_v_i", k_v_i_, 0.01);
    nh.param("/pid_planner/PIDPlanner/k_v_d", k_v_d_, 0.10);

    nh.param("/pid_planner/PIDPlanner/k_w_p", k_w_p_, 1.00);
    nh.param("/pid_planner/PIDPlanner/k_w_i", k_w_i_, 0.01);
    nh.param("/pid_planner/PIDPlanner/k_w_d", k_w_d_, 0.10);

    nh.param("/pid_planner/PIDPlanner/k_theta", k_theta_, 0.5);
    
    //前瞻距离
    nh.param("/pid_planner/PIDPlanner/Lfw", Lfw_,
             1.5);  // forward look ahead distance (m)
    nh.param("/pid_planner/PIDPlanner/goal_radius", goal_radius_, 1.0);

    double controller_freqency;
    nh.param("/pid_planner/PIDPlanner/controller_frequency",
             controller_freqency, 10.0);
    // 车辆类型
    nh.param(
        "/pid_planner/PIDPlanner/robot_type", robot_type_,
        std::string("yuanpan"));  // whether or not publishing this->cmd_vel
    // Controller parameter
    nh.param("/pid_planner/PIDPlanner/cmd_vel_mode", cmd_vel_mode,
             true);  // whether or not publishing this->cmd_vel
    //前瞻点
    nh.param("/pid_planner/PIDPlanner/math_path_proportion",
             math_path_proportion_, 1.1);
    //轮距
    nh.param("/pid_planner/PIDPlanner/wheelbase",
             wheelbase_, 1.1);
    d_t_ = 1 / controller_freqency;

    e_v_ = i_v_ = 0.0;
    e_w_ = i_w_ = 0.0;

    odom_sub = n_.subscribe("/pid/odom", 1, &PIDPlanner::odomCB, this);

    path_sub = n_.subscribe("/pid/global_plan", 1, &PIDPlanner::pathCB, this);
    goal_sub =
        n_.subscribe("/move_base_simple/goal", 1, &PIDPlanner::goalCB, this);
    amcl_sub = n_.subscribe("/localization_pose", 5, &PIDPlanner::amclCB, this);
    current_vel_sub =
        n_.subscribe("/cmd_vel", 1, &PIDPlanner::robotVelCB, this);

    marker_pub =
        n_.advertise<visualization_msgs::Marker>("/pid/path_marker", 10);
    ackermann_pub = nh.advertise<ackermann_msgs::AckermannDrive>(
        "/pid/ackermann_cmd", 1, true);
    if (cmd_vel_mode)
      cmdvel_pub =
          nh.advertise<geometry_msgs::Twist>("/pid/round_cmd_vel", 1, true);

    target_pose_pub_ =
        nh.advertise<geometry_msgs::PoseStamped>("/target_pose", 10);
    current_pose_pub_ =
        nh.advertise<geometry_msgs::PoseStamped>("/current_pose", 10);
    //发布控制指标
    control_indicators_ =
        nh.advertise<custom_msgs_srvs::Control>("/control_indicators", 1);
    // Timer
    timer1 = n_.createTimer(ros::Duration((1.0) / controller_freqency),
                            &PIDPlanner::controlLoopCB,
                            this);  // Duration(0.05) -> 20Hz
    // Init variables
    foundForwardPt_ = false;
    goal_received_ = false;
    goal_reached_ = false;
    // Visualization Marker Settings
    initMarker();
    cmd_vel = geometry_msgs::Twist();
    ackermann_cmd = ackermann_msgs::AckermannDrive();

    ROS_INFO("PID planner initialized!");
  } else
    ROS_WARN("PID planner has already been initialized.");
}

/**
 * @brief Destroy the PIDPlanner object
 */
PIDPlanner::~PIDPlanner() {}
/**
 * @brief 初始化rviz marker
 */
void PIDPlanner::initMarker() {
  points.header.frame_id = line_strip.header.frame_id =
      goal_circle.header.frame_id = "odom";
  points.ns = line_strip.ns = goal_circle.ns = "Markers";
  points.action = line_strip.action = goal_circle.action =
      visualization_msgs::Marker::ADD;
  points.pose.orientation.w = line_strip.pose.orientation.w =
      goal_circle.pose.orientation.w = 1.0;
  points.id = 0;
  line_strip.id = 1;
  goal_circle.id = 2;

  points.type = visualization_msgs::Marker::POINTS;
  line_strip.type = visualization_msgs::Marker::LINE_STRIP;
  goal_circle.type = visualization_msgs::Marker::CYLINDER;
  // POINTS markers use x and y scale for width/height respectively
  points.scale.x = 0.2;
  points.scale.y = 0.2;

  // LINE_STRIP markers use only the x component of scale, for the line width
  line_strip.scale.x = 0.1;

  goal_circle.scale.x = goal_radius_;
  goal_circle.scale.y = goal_radius_;
  goal_circle.scale.z = 0.1;

  // Points are green
  points.color.g = 1.0f;
  points.color.a = 1.0;

  // Line strip is blue
  line_strip.color.b = 1.0;
  line_strip.color.a = 1.0;

  // goal_circle is yellow
  goal_circle.color.r = 1.0;
  goal_circle.color.g = 1.0;
  goal_circle.color.b = 0.0;
  goal_circle.color.a = 0.5;
}

void PIDPlanner::robotVelCB(const geometry_msgs::Twist::ConstPtr& msg) {
  //    this->Vcmd = msg->linear.x;
  this->cmd_z = msg->angular.z;
}

void PIDPlanner::odomCB(const nav_msgs::Odometry::ConstPtr& odomMsg) {
  // ROS_INFO("enter odomCB!");
  this->odom = *odomMsg;
  current_ps_odom_.pose = this->odom.pose.pose;
  current_ps_odom_.header.stamp = this->odom.header.stamp;
  current_ps_odom_.header.frame_id = this->odom.header.frame_id;

  double maximum_lookahead_distance = odomMsg->twist.twist.linear.x * 10;
  double ld = odomMsg->twist.twist.linear.x * lookahead_distance_ratio_;
  this->Lfw_ =
      ld < minimum_lookahead_distance_
          ? minimum_lookahead_distance_
          : ld > maximum_lookahead_distance ? maximum_lookahead_distance : ld;
  if (fabs(this->cmd_z) > 0.2) {
    this->Lfw_ = 1.0;
  }
}

void PIDPlanner::pathCB(const nav_msgs::Path::ConstPtr& pathMsg) {
  //规划全局路径后，这个消息要重新定位才能拿到
  ROS_INFO("---------enter pathCB-----------");
  this->map_path = *pathMsg;
  setPlan(map_path.poses);
}

/**
 * @brief Set the plan that the controller is following
 * @param orig_global_plan the plan to pass to the controller
 * @return true if the plan was updated successfully, else false
 */
bool PIDPlanner::setPlan(
    const std::vector<geometry_msgs::PoseStamped>& orig_global_plan) {
  if (!initialized_) {
    ROS_ERROR(
        "This planner has not been initialized, please call initialize() "
        "before using this planner");
    return false;
  }

  // ROS_INFO("Got new plan");

  // set new plan
  global_plan_.clear();
  global_plan_ = orig_global_plan;

  // reset plan parameters
  // 将得到的路径的第四个点作为第一个前瞻点
  // 在pure_pursuit算法中，如果摆角需要大于30，就不取该点为前瞻点
  plan_index_ = std::min(
      4, (int)global_plan_.size() -
             1);  // help getting a future plan, since the plan may delay

  // receive a plan for a new goal
  if (goal_x_ != global_plan_.back().pose.position.x ||
      goal_y_ != global_plan_.back().pose.position.y) {
    goal_x_ = global_plan_.back().pose.position.x;
    goal_y_ = global_plan_.back().pose.position.y;
    goal_rpy_ = getEulerAngles(global_plan_.back());
    goal_reached_ = false;

    e_v_ = i_v_ = 0.0;
    e_w_ = i_w_ = 0.0;
  }

  return true;
}

/*
 * @brief 获取目标点信息
 */
void PIDPlanner::goalCB(const geometry_msgs::PoseStamped::ConstPtr& goalMsg) {
  // ROS_INFO("-----------enter goalCB--------------");
  this->map_path.poses.clear();
  this->goal_pos = goalMsg->pose.position;
  try {
    geometry_msgs::PoseStamped odom_goal;
    if (!tf_buffer_.canTransform(odom_frame_, goalMsg->header.frame_id,
                                 ros::Time(0), ros::Duration(0.1))) {
      ROS_WARN("1.Failed to find a transform from %s to %s",
               odom_frame_.c_str(), goalMsg->header.frame_id.c_str());
    }
    tf_buffer_.transform(*goalMsg, odom_goal, odom_frame_);
    odom_goal_pos = odom_goal.pose.position;
    goal_received_ = true;
    goal_reached_ = false;

    /*Draw Goal on RVIZ*/
    goal_circle.pose = odom_goal.pose;
    marker_pub.publish(goal_circle);
  } catch (tf2::TransformException& ex) {
    ROS_ERROR("3333 %s", ex.what());
    // 处理转换异常的代码
  }
}
/**
 * @brief 判断目前车辆与前瞻点的距离是否大于设定的前瞻距离
 * @param wayPt 前瞻点
 * @param car_pos 表示现在车辆的位置
 * @return 大于返回true
 */
bool PIDPlanner::isWayPtAwayFromLfwDist(const geometry_msgs::Point& wayPt,
                                        const geometry_msgs::Point& car_pos) {
  double dx = wayPt.x - car_pos.x;
  double dy = wayPt.y - car_pos.y;
  double dist = sqrt(dx * dx + dy * dy);
  //
  if (dist < Lfw_)
    return false;
  else if (dist >= Lfw_) {
    ROS_INFO("dist =%f,Lfw_=%f", dist, Lfw_);
    return true;
  }
}
/**
 * @brief 寻找车辆位置与全局路径距离最近的路径点
 * @param wayPt 前瞻点
 * @param car_pos 表示现在车辆的位置
 * @return 返回最近路径点的序号
 */
bool PIDPlanner::calcNearestPoseInterp(
    const std::vector<geometry_msgs::PoseStamped>& global_plan,
    const geometry_msgs::Pose& self_pose, unsigned int& nearest_index) {
  geometry_msgs::PoseStamped target_ps;
  double theta_dir;
  if (global_plan.size() == 0) {
   // ROS_WARN("[calcNearestPoseInterp] trajectory size is zero");
    return false;
  }
  const double my_x = self_pose.position.x;
  const double my_y = self_pose.position.y;
  const double my_yaw = getYawFromPose(self_pose);
  // const double my_yaw = tf2::getYaw(self_pose.orientation);

  int nearest_index_tmp = -1;
  double min_dist_squared = std::numeric_limits<double>::max();
  for (uint i = 0; i < global_plan.size(); ++i) {
    const double dx = my_x - global_plan[i].pose.position.x;
    const double dy = my_y - global_plan[i].pose.position.y;
    const double dist_squared = dx * dx + dy * dy;

    target_ps = global_plan[i];
    double x_d = target_ps.pose.position.x;
    double y_d = target_ps.pose.position.y;
    // from robot to plan point
    double traj_yaw = atan2((y_d - my_y), (x_d - my_x));
    /* ignore when yaw error is large, for crossing path
     * 在交叉路径时，当方向角误差较大，大于60°时忽略*/
    // double traj_yaw = tf2::getYaw(global_plan[i].pose.orientation);
    double err_yaw = my_yaw - traj_yaw;
    regularizeAngle(err_yaw);
    // if (fabs(err_yaw) < (M_PI / 3.0)) {
    if (dist_squared < min_dist_squared) {
      /* save nearest index */
      min_dist_squared = dist_squared;
      nearest_index_tmp = i;
      // ROS_INFO("-------------nearest_index_tmp=%d,dist_squared=%f-------------",
      //          nearest_index_tmp, dist_squared);
      //}
    }
  }
  if (nearest_index_tmp == -1) {
    ROS_WARN(
        "[calcNearestPoseInterp] yaw error is over PI/3 for all waypoints. no "
        "closest waypoint found.");
    return false;
  }
  nearest_index = nearest_index_tmp;
  ROS_INFO("nearest_index=%d", nearest_index);
  return true;
}  // namespace pid_planner

/**
 * @brief 返回前瞻点在车辆坐标系上的位置
 * @param carPose odom回调函数得到的位置信息
 * @return 返回前瞻点在车辆坐标系上的坐标
 */
double PIDPlanner::getYawFromPose(const geometry_msgs::Pose& carPose) {
  float x = carPose.orientation.x;
  float y = carPose.orientation.y;
  float z = carPose.orientation.z;
  float w = carPose.orientation.w;

  double tmp, yaw;
  tf2::Quaternion q(x, y, z, w);
  tf2::Matrix3x3 quaternion(q);
  quaternion.getRPY(tmp, tmp, yaw);
  while (yaw >= M_PI) {
    yaw -= 2 * M_PI;
  }
  while (yaw <= -M_PI) {
    yaw += 2 * M_PI;
  }

  return yaw;
}
/**
 * @brief 返回前瞻点在车辆坐标系上的位置
 * @param carPose odom回调函数得到的位置信息
 * @return 返回前瞻点在车辆坐标系上的坐标
 */
geometry_msgs::Point PIDPlanner::get_odom_car2WayPtVec(
    const geometry_msgs::Pose& carPose) {
  geometry_msgs::Point carPose_pos = carPose.position;
  double carPose_yaw = getYawFromPose(carPose);
  //  ROS_INFO("carPose_yaw=%f", carPose_yaw);
  geometry_msgs::Point forwardPt;
  geometry_msgs::Point odom_car2WayPtVec;
  foundForwardPt_ = false;
  unsigned int nearest_index;
  bool calcNearest =
      calcNearestPoseInterp(global_plan_, current_ps_.pose, nearest_index);

  if (goal_reached_) {
    forwardPt = odom_goal_pos;
    foundForwardPt_ = false;
    ROS_INFO("goal REACHED!");
  } else if (!goal_reached_ && map_path.poses.size() > 1) {
    ROS_INFO("---------------enter----------");
    map_path_pose_ =
        map_path.poses[map_path.poses.size() / math_path_proportion_];
    geometry_msgs::PoseStamped odom_path_pose;

    try {
      if (!tf_buffer_.canTransform(odom_frame_, map_frame_, ros::Time(0),
                                   ros::Duration(0.1))) {
        ROS_WARN("5.Failed to find a transform from %s to %s",
                 map_frame_.c_str(), odom_frame_.c_str());
      }
      tf_buffer_.transform(map_path_pose_, odom_path_pose, odom_frame_);
      forwardPt = odom_path_pose.pose.position;

    } catch (tf2::TransformException& ex) {
      ROS_ERROR("5555 %s ", ex.what());
      // 处理转换异常的代码
    }
    foundForwardPt_ = true;
  }

  /*Visualized Target Point on RVIZ*/
  /*Clear former target point Marker*/
  points.points.clear();
  line_strip.points.clear();

  if (foundForwardPt_ && !goal_reached_) {
    points.points.push_back(carPose_pos);
    points.points.push_back(forwardPt);
    line_strip.points.push_back(carPose_pos);
    line_strip.points.push_back(forwardPt);
  }

  marker_pub.publish(points);
  marker_pub.publish(line_strip);
  //获得前瞻点在车辆坐标系上的坐标
  odom_car2WayPtVec.x = cos(carPose_yaw) * (forwardPt.x - carPose_pos.x) +
                        sin(carPose_yaw) * (forwardPt.y - carPose_pos.y);
  odom_car2WayPtVec.y = -sin(carPose_yaw) * (forwardPt.x - carPose_pos.x) +
                        cos(carPose_yaw) * (forwardPt.y - carPose_pos.y);

  // ROS_INFO(
  //     "carPose -%f : %f : %f, forwardPt: %f : %f, odom_car2WayPtVec: %f :
  //     %f", carPose_pos.x, carPose_pos.y, carPose_yaw, forwardPt.x,
  //     forwardPt.y, odom_car2WayPtVec.x, odom_car2WayPtVec.y);
  return odom_car2WayPtVec;
}

/**
 * @brief 获取车辆与前瞻点朝向误差
 * @param carPose odom回调函数得到的位置信息
 * @return 朝向误差
 */
double PIDPlanner::getEta(const geometry_msgs::Pose& carPose) {
  //获取前瞻点
  geometry_msgs::Point odom_car2WayPtVec = get_odom_car2WayPtVec(carPose);
  ROS_INFO("odom_car2WayPtVec.x=%f,odom_car2WayPtVec.y=%f", odom_car2WayPtVec.x,
           odom_car2WayPtVec.y);
  // 返回朝向误差
  return atan2(odom_car2WayPtVec.y, odom_car2WayPtVec.x);
}
/**
 * @brief 获取车辆与前瞻点距离误差
 * @param  odom回调函数得到的信息
 * @return 朝向误差
 */
double PIDPlanner::getCar2WayPtDist(const geometry_msgs::Pose& carPose) {
  //获取前瞻点
  geometry_msgs::Point odom_car2WayPtVec = get_odom_car2WayPtVec(carPose);
  return sqrt(odom_car2WayPtVec.x * odom_car2WayPtVec.x +
              odom_car2WayPtVec.y * odom_car2WayPtVec.y);
}

/**
 * @brief 得到amcl定位信息，map坐标系上的
 */
void PIDPlanner::amclCB(
    const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& amclMsg) {
  // ROS_INFO("--------enter amclCB-------------");
  current_ps_.header = amclMsg->header;
  current_ps_.pose = amclMsg->pose.pose;

  if (this->goal_received_) {
    double car2goal_x = this->goal_pos.x - amclMsg->pose.pose.position.x;
    double car2goal_y = this->goal_pos.y - amclMsg->pose.pose.position.y;
    double dist2goal = sqrt(car2goal_x * car2goal_x + car2goal_y * car2goal_y);
    if (dist2goal < this->goal_radius_) {
      this->goal_reached_ = true;
      this->goal_received_ = false;
      ROS_INFO("Goal Reached !");
    }
  }
}
/**
 * @brief 定时计算速度
 */
void PIDPlanner::controlLoopCB(const ros::TimerEvent&) {
  // ROS_INFO("enter controlLoopCB!");
  computeVelocityCommands();
  //发布控制指标
  publish_controlindicators();
}
/**
 * @brief Given the current position, orientation, and velocity of the robot,
 * compute the velocity commands
 * @param this->cmd_vel will be filled with the velocity command to be passed to
 * the robot base
 * @return true if a valid trajectory was found, else false
 */
bool PIDPlanner::computeVelocityCommands() {
  if (!initialized_) {
    ROS_ERROR("PID planner has not been initialized");
    return false;
  }
  if (global_plan_.empty()) {
    // ROS_ERROR("global_plan_ is empty");
    return false;
  }
  // current_ps_ 要得到map坐标系上机器人的位置信息
  //this->odom得到的信息是在odom坐标系上的
  // current angle
  geometry_msgs::Pose carPose = this->odom.pose.pose;
  geometry_msgs::Twist carVel = this->odom.twist.twist;
  // odometry observation - getting robot velocities in robot frame
  // 还是odom坐标系上的信息，但是好像下面只是用到了base_odom的twist信息
  nav_msgs::Odometry base_odom;
  base_odom = this->odom;

  //  position reached 位置到达
  if (robot_type_ == "yuanpan") {
    // posistion not reached
    if (this->goal_received_) {
      double eta = getEta(carPose);
      if (foundForwardPt_) {
        if (!this->goal_reached_) {
          //距离误差
          this->cmd_vel.linear.x =
              linearRegularization(base_odom, getCar2WayPtDist(carPose) / d_t_);
          //通过航向偏差获取前轮打角
          this->cmd_vel.angular.z =
              angularRegularization(base_odom, eta / d_t_);
          ROS_INFO("eta=%f,d_t_=%f,eta / d_t_=%f", eta, d_t_, eta / d_t_);
          ROS_INFO("velocity=%f,angular=%f", this->cmd_vel.linear.x,
                   this->cmd_vel.angular.z);
        }
      } else {
        ROS_INFO("Don't find forward point");
        this->cmd_vel.linear.x = 0.0;
        this->cmd_vel.angular.z = 0.0;
      }
    }
    if (this->goal_reached_) {
      ROS_INFO("GOAL REACHED");
      this->cmd_vel.linear.x = 0.0;
      this->cmd_vel.angular.z = 0.0;
    }
  }

  // publish cmd_vel发布速度
  this->ackermann_cmd.steering_angle = atan(this->cmd_vel.angular.z*wheelbase_)/this->cmd_vel.linear.x;
  this->ackermann_cmd.speed = this->cmd_vel.linear.x;
  this->ackermann_pub.publish(this->ackermann_cmd);

  if (this->cmd_vel_mode) {
    this->cmdvel_pub.publish(this->cmd_vel);
  }

  // publish next target_ps_ pose
  // target_ps_.header.frame_id = "map";
  // target_ps_.header.stamp = ros::Time::now();
  target_pose_pub_.publish(target_ps_);

  // publish robot pose
  // current_ps_.header.frame_id = "map";
  // current_ps_.header.stamp = ros::Time::now();
  current_pose_pub_.publish(current_ps_);

  return true;
}  // namespace pid_planner

/**
 * @brief linear velocity regularization
 * @param base_odometry odometry of the robot, to get velocity
 * @param v_d           desired velocity magnitude

 * @return v            regulated linear velocity
 */
double PIDPlanner::linearRegularization(nav_msgs::Odometry& base_odometry,
                                        double v_d) {
  double v = std::hypot(base_odometry.twist.twist.linear.x,
                        base_odometry.twist.twist.linear.y);
  if (std::fabs(v_d) > max_v_) v_d = std::copysign(max_v_, v_d);

  double e_v = v_d - v;
  i_v_ += e_v * d_t_;
  double d_v = (e_v - e_v_) / d_t_;
  e_v_ = e_v;

  double v_inc = k_v_p_ * e_v + k_v_i_ * i_v_ + k_v_d_ * d_v;

  if (std::fabs(v_inc) > max_v_inc_) v_inc = std::copysign(max_v_inc_, v_inc);

  double v_cmd = v + v_inc;
  if (std::fabs(v_cmd) > max_v_)
    v_cmd = std::copysign(max_v_, v_cmd);
  else if (std::fabs(v_cmd) < min_v_)
    v_cmd = std::copysign(min_v_, v_cmd);

  return v_cmd;
}

/**
 * @brief angular velocity regularization
 * @param base_odometry odometry of the robot, to get velocity
 * @param w_d           desired angular velocity
 * @return  w           regulated angular velocity
 */
double PIDPlanner::angularRegularization(nav_msgs::Odometry& base_odometry,
                                         double w_d) {
  if (std::fabs(w_d) > max_w_) w_d = std::copysign(max_w_, w_d);

  double w = base_odometry.twist.twist.angular.z;
  double e_w = w_d - w;
  i_w_ += e_w * d_t_;
  double d_w = (e_w - e_w_) / d_t_;
  e_w_ = e_w;
  double w_inc = k_w_p_ * e_w + k_w_i_ * i_w_ + k_w_d_ * d_w;

  if (std::fabs(w_inc) > max_w_inc_) w_inc = std::copysign(max_w_inc_, w_inc);
  double w_cmd = w + w_inc;
  if (std::fabs(w_cmd) > max_w_)
    w_cmd = std::copysign(max_w_, w_cmd);
  else if (std::fabs(w_cmd) < min_w_)
    w_cmd = std::copysign(min_w_, w_cmd);

  ROS_INFO("w=%f,w_d=%f,w_inc= %f,w_cmd=%f", w, w_d, w_inc, w_cmd);
  return w_cmd;
}

void PIDPlanner::publish_controlindicators() {
  //获取前瞻点在车辆坐标系的位置
  // current angle
  geometry_msgs::Pose carPose = this->odom.pose.pose;
  geometry_msgs::Point odom_car2WayPtVec = get_odom_car2WayPtVec(carPose);
  //横纵向误差
  control_.lat_err = odom_car2WayPtVec.x;
  control_.lon_err = odom_car2WayPtVec.y;
  //速度误差
  control_.vel_err = e_v_;
  control_.angular_err = e_w_;
  //发送速度和反馈速度
  control_.send_vel = this->cmd_vel.linear.x;
  control_.recv_vel = this->odom.twist.twist.linear.x;
  //加速度
  control_.acce = 0;
  //曲率
  control_.cur = this->cmd_vel.angular.z / this->cmd_vel.linear.x;
  //当前位置
  control_.current_position.x = current_ps_.pose.position.x;
  control_.current_position.y = current_ps_.pose.position.y;
  control_.current_position.yaw = getYawFromPose(current_ps_.pose);
  //前瞻点位置
  control_.preview_position.x = map_path_pose_.pose.position.x;
  control_.preview_position.y = map_path_pose_.pose.position.y;
  control_.preview_position.yaw = getYawFromPose(map_path_pose_.pose);
  control_indicators_.publish(control_);
}

}  // namespace pid_planner

/*****************/
/* MAIN FUNCTION */
/*****************/
int main(int argc, char** argv) {
  // Initiate ROS
  ros::init(argc, argv, "pid");
  pid_planner::PIDPlanner controller;
  ros::AsyncSpinner spinner(2);  // Use multi threads
  spinner.start();
  ros::waitForShutdown();
  ROS_INFO("------------3------------!");
  return 0;
}
