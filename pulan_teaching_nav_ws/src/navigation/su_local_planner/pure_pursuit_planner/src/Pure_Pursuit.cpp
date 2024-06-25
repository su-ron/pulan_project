/*
# Copyright 2018 HyphaROS Workshop.
# Latest Modifier: HaoChih, LIN (hypha.ros@gmail.com)
# Original Author: ChanYuan KUO & YoRu LU
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
*/

#include <ackermann_msgs/AckermannDriveStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <iostream>
#include "custom_msgs_srvs/Laterr.h"
#include "ros/ros.h"
/********************/
/* CLASS DEFINITION */
/********************/
class PurePursuit {
 public:
  PurePursuit();
  void initMarker();
  bool isForwardWayPt(const geometry_msgs::Point& wayPt,
                      const geometry_msgs::Pose& carPose);
  bool isWayPtAwayFromLfwDist(const geometry_msgs::Point& wayPt,
                              const geometry_msgs::Point& car_pos);
  double getYawFromPose(const geometry_msgs::Pose& carPose);
  double getEta(const geometry_msgs::Pose& carPose);
  double getCar2GoalDist();
  double getSteering(double eta);
  geometry_msgs::Point get_odom_car2WayPtVec(
      const geometry_msgs::Pose& carPose);

 private:
  ros::NodeHandle n_;
  ros::Subscriber odom_sub, path_sub, goal_sub, amcl_sub, current_vel_sub,
      dwa_path_sub;
  ros::Publisher ackermann_pub, cmdvel_pub, marker_pub, head_err;
  ros::Timer timer1, timer2;
  tf::TransformListener tf_listener;
  tf::TransformListener tf_;

  visualization_msgs::Marker points, line_strip, goal_circle;
  geometry_msgs::Point odom_goal_pos, goal_pos;
  geometry_msgs::Twist cmd_vel;
  ackermann_msgs::AckermannDriveStamped ackermann_cmd;
  nav_msgs::Odometry odom;
  nav_msgs::Path map_path, odom_path;

  double L, Lfw, Vcmd, lfw, steering, velocity;
  double steering_gain, base_angle, goal_radius, speed_incremental, cmd_z;
  int controller_freq;
  bool foundForwardPt, goal_received, goal_reached, cmd_vel_mode, debug_mode,
      smooth_accel;
  double minimum_lookahead_distance_ = 1.5;
  double lookahead_distance_ratio_ = 2.0;

  void odomCB(const nav_msgs::Odometry::ConstPtr& odomMsg);
  void pathCB(const nav_msgs::Path::ConstPtr& pathMsg);
  void dwaPathCB(const nav_msgs::Path::ConstPtr& pathMsg);
  void goalCB(const geometry_msgs::PoseStamped::ConstPtr& goalMsg);
  void amclCB(
      const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& amclMsg);
  void controlLoopCB(const ros::TimerEvent&);
  void robotVelCB(const geometry_msgs::Twist::ConstPtr& msg);

};  // end of class

PurePursuit::PurePursuit() {
  // Private parameters handler
  ros::NodeHandle pn("~");

  // Car parameter
  pn.param("L", L, 0.26);       // length of car
  pn.param("Vcmd", Vcmd, 1.0);  // reference speed (m/s)
  pn.param("Lfw", Lfw, 3.0);    // forward look ahead distance (m)
  pn.param("lfw", lfw, 0.13);   // distance between front the center of car

  // Controller parameter
  pn.param("controller_freq", controller_freq, 20);
  pn.param("steering_gain", steering_gain, 1.0);
  pn.param("goal_radius", goal_radius, 0.5);  // goal radius (m)
  pn.param("base_angle", base_angle, 0.0);    // neutral point of servo (rad)
  pn.param("cmd_vel_mode", cmd_vel_mode,
           true);  // whether or not publishing cmd_vel
  pn.param("debug_mode", debug_mode, true);  // debug mode
  pn.param("smooth_accel", smooth_accel,
           true);  // smooth the acceleration of car
  pn.param("speed_incremental", speed_incremental,
           0.5);  // speed incremental value (discrete acceleraton), unit: m/s

  // Publishers and Subscribers
  odom_sub = n_.subscribe("/pure_pursuit/odom", 1, &PurePursuit::odomCB, this);
  path_sub =
      n_.subscribe("/pure_pursuit/global_plan", 1, &PurePursuit::pathCB, this);
  dwa_path_sub = n_.subscribe("/pure_pursuit/local_plan", 1,
                              &PurePursuit::dwaPathCB, this);
  goal_sub =
      n_.subscribe("/move_base_simple/goal", 1, &PurePursuit::goalCB, this);
  amcl_sub = n_.subscribe("/localization_pose", 5, &PurePursuit::amclCB, this);
  current_vel_sub = n_.subscribe("/cmd_vel", 1, &PurePursuit::robotVelCB, this);
  marker_pub =
      n_.advertise<visualization_msgs::Marker>("/pure_pursuit/path_marker", 10);
  ackermann_pub = n_.advertise<ackermann_msgs::AckermannDriveStamped>(
      "/pure_pursuit/ackermann_cmd", 1, true);
  if (cmd_vel_mode)
    cmdvel_pub = n_.advertise<geometry_msgs::Twist>("/round_cmd_vel", 1, true);

  head_err = n_.advertise<custom_msgs_srvs::Laterr>("/heading_err", 1, true);
  // Timer
  timer1 = n_.createTimer(ros::Duration((1.0) / controller_freq),
                          &PurePursuit::controlLoopCB,
                          this);  // Duration(0.05) -> 20Hz

  ros::Time last_error = ros::Time::now();
  std::string tf_error;
  while (ros::ok() && !tf_.waitForTransform("odom", "base_footprint",
                                            ros::Time(), ros::Duration(0.1),
                                            ros::Duration(0.01), &tf_error)) {
    ros::spinOnce();
    if (last_error + ros::Duration(5.0) < ros::Time::now()) {
      ROS_WARN(
          "Timed out waiting for transform from odom to base_footprint to "
          "become available before running costmap, tf error: %s",
          tf_error.c_str());
      last_error = ros::Time::now();
    }
    // The error string will accumulate and errors will typically be the same,
    // so the last will do for the warning above. Reset the string here to avoid
    // accumulation.
    tf_error.clear();
  }

  // Init variables
  foundForwardPt = false;
  goal_received = false;
  goal_reached = false;
  velocity = 0.0;
  steering = base_angle;

  // Show info
  ROS_INFO("[param] base_angle: %f", base_angle);
  ROS_INFO("[param] Vcmd: %f", Vcmd);
  ROS_INFO("[param] Lfw: %f", Lfw);

  // Visualization Marker Settings
  initMarker();

  cmd_vel = geometry_msgs::Twist();
  ackermann_cmd = ackermann_msgs::AckermannDriveStamped();
}

void PurePursuit::initMarker() {
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

  goal_circle.scale.x = goal_radius;
  goal_circle.scale.y = goal_radius;
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

void PurePursuit::robotVelCB(const geometry_msgs::Twist::ConstPtr& msg) {
  //    this->Vcmd = msg->linear.x;
  this->cmd_z = msg->angular.z;
}

void PurePursuit::odomCB(const nav_msgs::Odometry::ConstPtr& odomMsg) {
  tf::Stamped<tf::Pose> global_pose;
  global_pose.setIdentity();
  tf::Stamped<tf::Pose> robot_pose;
  robot_pose.setIdentity();
  robot_pose.frame_id_ = "base_footprint";
  robot_pose.stamp_ = ros::Time();
  ros::Time current_time =
      ros::Time::now();  // save time for checking tf delay later

  // get the global pose of the robot
  try {
    tf_.transformPose("odom", robot_pose, global_pose);
  } catch (tf::LookupException& ex) {
    ROS_ERROR_THROTTLE(
        1.0, "No Transform available Error looking up robot pose: %s\n",
        ex.what());
    return;
  } catch (tf::ConnectivityException& ex) {
    ROS_ERROR_THROTTLE(1.0, "Connectivity Error looking up robot pose: %s\n",
                       ex.what());
    return;
  } catch (tf::ExtrapolationException& ex) {
    ROS_ERROR_THROTTLE(1.0, "Extrapolation Error looking up robot pose: %s\n",
                       ex.what());
    return;
  }
  // check global_pose timeout
  if (current_time.toSec() - global_pose.stamp_.toSec() > 0.5) {
    ROS_WARN_THROTTLE(1.0,
                      "Costmap2DROS transform timeout. Current time: %.4f, "
                      "global_pose stamp: %.4f, tolerance: %.4f",
                      current_time.toSec(), global_pose.stamp_.toSec(), 0.5);
    return;
  }

  this->odom.pose.pose.position.x = global_pose.getOrigin().getX();
  this->odom.pose.pose.position.y = global_pose.getOrigin().getY();
  this->odom.pose.pose.orientation =
      tf::createQuaternionMsgFromYaw(tf::getYaw(global_pose.getRotation()));

  this->odom.twist.twist.linear.x = odomMsg->twist.twist.linear.x;
  this->odom.twist.twist.angular.z = odomMsg->twist.twist.angular.z;

  // this->odom = *odomMsg;
  double maximum_lookahead_distance = odomMsg->twist.twist.linear.x * 10;
  double ld = odomMsg->twist.twist.linear.x * lookahead_distance_ratio_;

  this->Lfw =
      ld < minimum_lookahead_distance_
          ? minimum_lookahead_distance_
          : ld > maximum_lookahead_distance ? maximum_lookahead_distance : ld;
  if (fabs(this->cmd_z) > 0.2) {
    this->Lfw = 1.0;
  }
}

void PurePursuit::dwaPathCB(const nav_msgs::Path::ConstPtr& pathMsg) {
  // this->Lfw  = pathMsg->poses.size() * 0.05;
}

void PurePursuit::pathCB(const nav_msgs::Path::ConstPtr& pathMsg) {
  ROS_INFO("-----------------enter pathCB----------------");
  this->map_path = *pathMsg;
}

void PurePursuit::goalCB(const geometry_msgs::PoseStamped::ConstPtr& goalMsg) {
  ROS_INFO("-----------------enter goalCB----------------");
  this->map_path.poses.clear();
  this->goal_pos = goalMsg->pose.position;
  try {
    geometry_msgs::PoseStamped odom_goal;
    tf_listener.transformPose("odom", ros::Time(0), *goalMsg, "map", odom_goal);
    odom_goal_pos = odom_goal.pose.position;
    goal_received = true;
    goal_reached = false;

    /*Draw Goal on RVIZ*/
    goal_circle.pose = odom_goal.pose;
    marker_pub.publish(goal_circle);
  } catch (tf::TransformException& ex) {
    ROS_ERROR("2222 %s", ex.what());
    ros::Duration(0.1).sleep();
  }
}

double PurePursuit::getYawFromPose(const geometry_msgs::Pose& carPose) {
  float x = carPose.orientation.x;
  float y = carPose.orientation.y;
  float z = carPose.orientation.z;
  float w = carPose.orientation.w;

  double tmp, yaw;
  tf::Quaternion q(x, y, z, w);
  tf::Matrix3x3 quaternion(q);
  quaternion.getRPY(tmp, tmp, yaw);
  while (yaw >= M_PI) {
    yaw -= 2 * M_PI;
  }
  while (yaw <= -M_PI) {
    yaw += 2 * M_PI;
  }

  return yaw;
}

bool PurePursuit::isForwardWayPt(const geometry_msgs::Point& wayPt,
                                 const geometry_msgs::Pose& carPose) {
  float car2wayPt_x = wayPt.x - carPose.position.x;
  float car2wayPt_y = wayPt.y - carPose.position.y;
  double car_theta = getYawFromPose(carPose);

  float car_car2wayPt_x =
      cos(car_theta) * car2wayPt_x + sin(car_theta) * car2wayPt_y;
  float car_car2wayPt_y =
      -sin(car_theta) * car2wayPt_x + cos(car_theta) * car2wayPt_y;

  if (car_car2wayPt_x > 0) /*is Forward WayPt*/
    return true;
  else
    return false;
}

bool PurePursuit::isWayPtAwayFromLfwDist(const geometry_msgs::Point& wayPt,
                                         const geometry_msgs::Point& car_pos) {
  double dx = wayPt.x - car_pos.x;
  double dy = wayPt.y - car_pos.y;
  double dist = sqrt(dx * dx + dy * dy);

  if (dist < Lfw)
    return false;
  else if (dist >= Lfw)
    return true;
}

//返回前瞻点在车辆坐标系上的位置
geometry_msgs::Point PurePursuit::get_odom_car2WayPtVec(
    const geometry_msgs::Pose& carPose) {
  geometry_msgs::Point carPose_pos = carPose.position;
  double carPose_yaw = getYawFromPose(carPose);
  geometry_msgs::Point forwardPt;
  geometry_msgs::Point odom_car2WayPtVec;
  foundForwardPt = false;

  if (!goal_reached) {
    for (int i = 5; i < map_path.poses.size(); i++) {
      geometry_msgs::PoseStamped map_path_pose = map_path.poses[i];
      geometry_msgs::PoseStamped odom_path_pose;

      try {
        tf_listener.transformPose("odom", ros::Time(0), map_path_pose, "map",
                                  odom_path_pose);
        geometry_msgs::Point odom_path_wayPt = odom_path_pose.pose.position;
        // bool _isForwardWayPt = isForwardWayPt(odom_path_wayPt,carPose);
        double angle_diff = atan2(odom_path_pose.pose.position.y,
                                  odom_path_pose.pose.position.x);
        // ROS_INFO("-------------1------------------ %f", angle_diff);
        if (fabs(angle_diff) < 1.1) {
          //  ROS_INFO("-------------2------------------ %f", angle_diff);
          bool _isWayPtAwayFromLfwDist =
              isWayPtAwayFromLfwDist(odom_path_wayPt, carPose_pos);
          if (_isWayPtAwayFromLfwDist) {
            //    ROS_INFO("-------------3------------------");
            forwardPt = odom_path_wayPt;

            foundForwardPt = true;
            break;
          }
        }
      } catch (tf::TransformException& ex) {
        ROS_INFO("frame_id:%s", map_path.poses[i].header.frame_id);
        ROS_ERROR("1111 %s", ex.what());
      }
    }

  }

  else if (goal_reached) {
    forwardPt = odom_goal_pos;
    foundForwardPt = false;
    ROS_INFO("goal REACHED!");
  }
  if (!foundForwardPt && !goal_reached && map_path.poses.size() > 1) {
    ROS_INFO("---------------enter----------");
    geometry_msgs::PoseStamped map_path_pose =
        map_path.poses[map_path.poses.size() / 2];
    geometry_msgs::PoseStamped odom_path_pose;

    try {
      tf_listener.transformPose("odom", ros::Time(0), map_path_pose, "map",
                                odom_path_pose);
      forwardPt = odom_path_pose.pose.position;

    } catch (tf::TransformException& ex) {
      ROS_ERROR("3331 %s", ex.what());
    }
    foundForwardPt = true;
  }

  /*Visualized Target Point on RVIZ*/
  /*Clear former target point Marker*/
  points.points.clear();
  line_strip.points.clear();

  if (foundForwardPt && !goal_reached) {
    points.points.push_back(carPose_pos);
    points.points.push_back(forwardPt);
    line_strip.points.push_back(carPose_pos);
    line_strip.points.push_back(forwardPt);
  }

  marker_pub.publish(points);
  marker_pub.publish(line_strip);

  odom_car2WayPtVec.x = cos(carPose_yaw) * (forwardPt.x - carPose_pos.x) +
                        sin(carPose_yaw) * (forwardPt.y - carPose_pos.y);
  odom_car2WayPtVec.y = -sin(carPose_yaw) * (forwardPt.x - carPose_pos.x) +
                        cos(carPose_yaw) * (forwardPt.y - carPose_pos.y);

  //   ROS_INFO(
  //       "carPose -%f : %f : %f, forwardPt: %f : %f, odom_car2WayPtVec: %f :
  //       %f", carPose_pos.x, carPose_pos.y, carPose_yaw, forwardPt.x,
  //       forwardPt.y, odom_car2WayPtVec.x, odom_car2WayPtVec.y);
  return odom_car2WayPtVec;
}

//获取朝向误差
double PurePursuit::getEta(const geometry_msgs::Pose& carPose) {
  //获取前瞻点
  geometry_msgs::Point odom_car2WayPtVec = get_odom_car2WayPtVec(carPose);
  //返回朝向误差
  return atan2(odom_car2WayPtVec.y, odom_car2WayPtVec.x);
}

double PurePursuit::getCar2GoalDist() {
  geometry_msgs::Point car_pose = this->odom.pose.pose.position;
  double car2goal_x = this->odom_goal_pos.x - car_pose.x;
  double car2goal_y = this->odom_goal_pos.y - car_pose.y;

  return sqrt(car2goal_x * car2goal_x + car2goal_y * car2goal_y);
}

//计算前轮摆角
double PurePursuit::getSteering(double eta) {
  return atan2((this->L * sin(eta)), (this->Lfw / 2 + this->lfw * cos(eta)));
}

void PurePursuit::amclCB(
    const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& amclMsg) {
  if (this->goal_received) {
    double car2goal_x = this->goal_pos.x - amclMsg->pose.pose.position.x;
    double car2goal_y = this->goal_pos.y - amclMsg->pose.pose.position.y;
    double dist2goal = sqrt(car2goal_x * car2goal_x + car2goal_y * car2goal_y);
    if (dist2goal < this->goal_radius) {
      this->goal_reached = true;
      this->goal_received = false;
      ROS_INFO("Goal Reached !");
    }
  }
}

void PurePursuit::controlLoopCB(const ros::TimerEvent&) {
  geometry_msgs::Pose carPose = this->odom.pose.pose;
  geometry_msgs::Twist carVel = this->odom.twist.twist;

  if (this->goal_received) {
    /*Estimate Steering Angle*/
    double eta = getEta(carPose);
    custom_msgs_srvs::Laterr msg;
    msg.data = eta;
    // ROS_INFO("heading_err:%f",eta);
    this->head_err.publish(msg);
    if (foundForwardPt) {
      //通过横向偏差获取前轮打角
      this->steering =
          this->base_angle + getSteering(eta) * this->steering_gain;
      //   ROS_INFO("steering:%f", this->steering);
      /*Estimate Gas Input*/
      if (!this->goal_reached) {
        if (this->smooth_accel)
          this->velocity =
              std::min(this->velocity + this->speed_incremental, this->Vcmd);
        else
          this->velocity = this->Vcmd;
        //      if(debug_mode) ROS_INFO("Velocity = %.2f, Steering = %.2f,
        //      steering_gain: %.2f", this->velocity, this->steering,
        //      this->steering_gain);
      }
    }
  }

  if (this->goal_reached) {
    this->velocity = 0.0;
    this->steering = 0.0;
    if (debug_mode)
      ROS_INFO("2 Velocity = %.2f, Steering = %.2f, steering_gain: %.2f",
               this->velocity, this->steering, this->steering_gain);
  }

  this->ackermann_cmd.drive.steering_angle = this->steering;
  this->ackermann_cmd.drive.speed = this->velocity;
  this->ackermann_pub.publish(this->ackermann_cmd);

  if (this->cmd_vel_mode) {
    this->cmd_vel.linear.x = this->velocity;
    this->cmd_vel.angular.z = this->steering;
    this->cmdvel_pub.publish(this->cmd_vel);
  }
}

/*****************/
/* MAIN FUNCTION */
/*****************/
int main(int argc, char** argv) {
  // Initiate ROS
  ros::init(argc, argv, "PurePursuit");
  PurePursuit controller;
  ros::AsyncSpinner spinner(2);  // Use multi threads
  spinner.start();
  ros::waitForShutdown();
  return 0;
}
