/*
# Copyright 2018 HyphaROS Workshop.
# Developer: HaoChih, LIN (hypha.ros@gmail.com)
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

#include <iostream>
#include <map>
#include <math.h>

#include "ros/ros.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <std_msgs/Float32.h>

// #include <tf/transform_datatypes.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "trackRefTraj.h"
#include "polynomial_curve.h"
#include <Eigen/Core>
#include <Eigen/QR>

#include "common/path/path_generator.h"

// inlcude iostream and string libraries
#include <iostream>
#include <fstream>
#include <string>


using namespace std;
using namespace Eigen;
typedef data_types::PncPoint Point;
Point target_pose_,start_pose_;

/********************/
/* CLASS DEFINITION */
/********************/
class MPCNode
{
    public:
        MPCNode();
        void initMarker();
        void publishSupplementaryMarker(const nav_msgs::Path& mpc_path, double n);
        void publishPredictionMarker(const nav_msgs::Path& mpc_path);
        ~MPCNode();
        int get_thread_numbers();
        
    private:
        ros::NodeHandle _nh;
        ros::Subscriber _sub_odom, _sub_gen_path, _sub_path, _sub_initpose,_sub_goal, _sub_amcl;
        ros::Publisher _pub_totalcost, _pub_ctecost, _pub_ethetacost,_pub_odompath, _pub_twist, _pub_ackermann, _pub_mpctraj,marker_pub,Supplementary_marker_pub_,Prediction_marker_pub_;
        ros::Timer _timer1;
        tf::TransformListener _tf_listener;
        
        visualization_msgs::Marker points,forward_points,marker, line_strip,forward_line_strip, goal_circle;

        geometry_msgs::Point _goal_pos;
        nav_msgs::Odometry _odom;
        nav_msgs::Path _odom_path, _mpc_traj; 
	//ackermann_msgs::AckermannDriveStamped _ackermann_msg;
        geometry_msgs::Twist _twist_msg;

        string _globalPath_topic, _goal_topic;
        string _map_frame, _odom_frame, _car_frame;

        MPC _mpc;
        map<string, double> _mpc_params;
        double _mpc_steps, _ref_cte, _ref_etheta, _ref_vel, _w_cte, _w_etheta, _w_vel, 
               _w_angvel, _w_accel, _w_angvel_d, _w_accel_d, _max_angvel, _max_throttle, _bound_value;

        //double _Lf; 
        double _dt, _w, _throttle, _speed, _max_speed;
        double _pathLength, _goalRadius, _waypointsDist;
        int _controller_freq, _downSampling, _thread_numbers;
        bool _goal_received, _goal_reached, _path_computed, _pub_twist_flag, _debug_info, _delay_mode;

        double polyeval(Eigen::VectorXd coeffs, double x);
        Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order);

        void odomCB(const nav_msgs::Odometry::ConstPtr& odomMsg);
        void pathCB(const nav_msgs::Path::ConstPtr& pathMsg);
        void desiredPathCB(const nav_msgs::Path& totalPathMsg);
        void initposeCB(
            const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
        void goalCB(const geometry_msgs::PoseStamped::ConstPtr& goalMsg);
        void amclCB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& amclMsg);
        void controlLoopCB(const ros::TimerEvent&);
       
        //For making global planner
        nav_msgs::Path _gen_path;
        unsigned int min_idx;
        
        double _mpc_etheta;
        double _mpc_cte;
        fstream file;
        unsigned int idx;

        //五次多项式初始化
        planning::PathGenerator pathgenerator_;
        //参考线
        std::vector<Point> ref_line_;

         geometry_msgs::PoseStamped initpose_map_, initpose_odom_;
         geometry_msgs::PoseStamped goalpose_map_, goalpose_odom_;

         bool initpose_received_, goal_received_;

}; // end of class


MPCNode::MPCNode()
{
    //Private parameters handler
    ros::NodeHandle pn("~");

    //Parameters for control loop
    pn.param("thread_numbers", _thread_numbers, 2); // number of threads for this ROS node
    pn.param("pub_twist_cmd", _pub_twist_flag, true);
    pn.param("debug_info", _debug_info, true);
    pn.param("delay_mode", _delay_mode, true);
    pn.param("max_speed", _max_speed, 0.50); // unit: m/s
    pn.param("waypoints_dist", _waypointsDist, -1.0); // unit: m
    pn.param("path_length", _pathLength, 2.0); // unit: m
    pn.param("goal_radius", _goalRadius, 0.5); // unit: m
    pn.param("controller_freq", _controller_freq, 10);
    //pn.param("vehicle_Lf", _Lf, 0.290); // distance between the front of the vehicle and its center of gravity
    _dt = double(1.0/_controller_freq); // time step duration dt in s 

    //Parameter for MPC solver
    pn.param("mpc_steps", _mpc_steps, 20.0);
    pn.param("mpc_ref_cte", _ref_cte, 0.0);
    pn.param("mpc_ref_vel", _ref_vel, 1.0);
    pn.param("mpc_ref_etheta", _ref_etheta, 0.0);
    pn.param("mpc_w_cte", _w_cte, 5000.0);
    pn.param("mpc_w_etheta", _w_etheta, 5000.0);
    pn.param("mpc_w_vel", _w_vel, 1.0);
    pn.param("mpc_w_angvel", _w_angvel, 100.0);
    pn.param("mpc_w_angvel_d", _w_angvel_d, 10.0);
    pn.param("mpc_w_accel", _w_accel, 50.0);
    pn.param("mpc_w_accel_d", _w_accel_d, 10.0);
    pn.param("mpc_max_angvel", _max_angvel, 3.0); // Maximal angvel radian (~30 deg)
    pn.param("mpc_max_throttle", _max_throttle, 1.0); // Maximal throttle accel
    pn.param("mpc_bound_value", _bound_value, 1.0e3); // Bound value for other variables

    //Parameter for topics & Frame name
    pn.param<std::string>("global_path_topic", _globalPath_topic, "/move_base/TrajectoryPlannerROS/global_plan" );
    pn.param<std::string>("goal_topic", _goal_topic, "/move_base_simple/goal" );
    pn.param<std::string>("map_frame", _map_frame, "odom" ); //*****for mpc, "odom"
    pn.param<std::string>("odom_frame", _odom_frame, "odom");
    pn.param<std::string>("car_frame", _car_frame, "base_footprint" );

    //Display the parameters
    cout << "\n===== Parameters =====" << endl;
    cout << "pub_twist_cmd: "  << _pub_twist_flag << endl;
    cout << "debug_info: "  << _debug_info << endl;
    cout << "delay_mode: "  << _delay_mode << endl;
    //cout << "vehicle_Lf: "  << _Lf << endl;
    cout << "frequency: "   << _dt << endl;
    cout << "mpc_steps: "   << _mpc_steps << endl;
    cout << "mpc_ref_vel: " << _ref_vel << endl;
    cout << "mpc_w_cte: "   << _w_cte << endl;
    cout << "mpc_w_etheta: "  << _w_etheta << endl;
    cout << "mpc_max_angvel: "  << _max_angvel << endl;

    //Publishers and Subscribers
    _sub_odom   = _nh.subscribe("/odom", 1, &MPCNode::odomCB, this);
    //_sub_path   = _nh.subscribe( _globalPath_topic, 1, &MPCNode::pathCB, this);
    //_sub_gen_path   = _nh.subscribe( "desired_path", 1, &MPCNode::desiredPathCB, this);
    //_sub_gen_path   = _nh.subscribe( "/mpc/global_plan", 1, &MPCNode::desiredPathCB, this);
    _sub_initpose   = _nh.subscribe("/initialpose", 5, &MPCNode::initposeCB, this);
    _sub_goal   = _nh.subscribe( "/mpc/goal", 1, &MPCNode::goalCB, this);
    _sub_amcl   = _nh.subscribe("/localization_pose", 5, &MPCNode::amclCB, this);
    
    _pub_odompath  = _nh.advertise<nav_msgs::Path>("/mpc_reference", 1); // reference path for MPC ///mpc_reference 
    _pub_mpctraj   = _nh.advertise<nav_msgs::Path>("/mpc_trajectory", 1);// MPC trajectory output
    _pub_ackermann = _nh.advertise<ackermann_msgs::AckermannDriveStamped>("/ackermann_cmd", 1);
    if(_pub_twist_flag)
        _pub_twist = _nh.advertise<geometry_msgs::Twist>("/mpc/round_cmd_vel", 1); //for stage (Ackermann msg non-supported)
    
    marker_pub =
      _nh.advertise<visualization_msgs::Marker>("/path_marker", 10);
    Supplementary_marker_pub_ =
      _nh.advertise<visualization_msgs::MarkerArray>("/supplementary_path_marker", 10);
    Prediction_marker_pub_=_nh.advertise<visualization_msgs::MarkerArray>("/prediction_path_marker", 10);
    _pub_totalcost  = _nh.advertise<std_msgs::Float32>("/total_cost", 1); // Global path generated from another source
    _pub_ctecost  = _nh.advertise<std_msgs::Float32>("/cross_track_error", 1); // Global path generated from another source
    _pub_ethetacost  = _nh.advertise<std_msgs::Float32>("/theta_error", 1); // Global path generated from another source
    
    //Timer
    _timer1 = _nh.createTimer(ros::Duration((1.0)/_controller_freq), &MPCNode::controlLoopCB, this); // 10Hz //*****mpc
    // Visualization Marker Settings
    initMarker();
    //Init variables
    _goal_received = false;
    _goal_reached  = false;
    _path_computed = false;
    _throttle = 0.0; 
    _w = 0.0;
    _speed = 0.0;

    //_ackermann_msg = ackermann_msgs::AckermannDriveStamped();
    _twist_msg = geometry_msgs::Twist();
    _mpc_traj = nav_msgs::Path();

    //Init parameters for MPC object
    _mpc_params["DT"] = _dt;
    //_mpc_params["LF"] = _Lf;
    _mpc_params["STEPS"]    = _mpc_steps;
    _mpc_params["REF_CTE"]  = _ref_cte;
    _mpc_params["REF_ETHETA"] = _ref_etheta;
    _mpc_params["REF_V"]    = _ref_vel;
    _mpc_params["W_CTE"]    = _w_cte;
    _mpc_params["W_EPSI"]   = _w_etheta;
    _mpc_params["W_V"]      = _w_vel;
    _mpc_params["W_ANGVEL"]  = _w_angvel;
    _mpc_params["W_A"]      = _w_accel;
    _mpc_params["W_DANGVEL"] = _w_angvel_d;
    _mpc_params["W_DA"]     = _w_accel_d;
    _mpc_params["ANGVEL"]   = _max_angvel;
    _mpc_params["MAXTHR"]   = _max_throttle;
    _mpc_params["BOUND"]    = _bound_value;
    _mpc.LoadParams(_mpc_params);

    min_idx = 0;
    idx = 0;
    _mpc_etheta = 0;
    _mpc_cte = 0;
    //file.open("/home/geonhee/catkin_ws/src/mpc_ros/mpc.csv");
}

void MPCNode::initMarker() {
  points.header.frame_id =forward_points.header.frame_id = line_strip.header.frame_id 
  =forward_line_strip.header.frame_id =goal_circle.header.frame_id =marker.header.frame_id= "odom";

  points.ns =forward_points.ns = line_strip.ns =forward_line_strip.ns = goal_circle.ns=marker.ns= "Markers";

  points.action = forward_points.action = line_strip.action = forward_line_strip.action = goal_circle.action =marker.action=
      visualization_msgs::Marker::ADD;
  points.pose.orientation.w = forward_points.pose.orientation.w = line_strip.pose.orientation.w 
  = forward_line_strip.pose.orientation.w =goal_circle.pose.orientation.w=marker.pose.orientation.w = 1.0;

  points.id = 0;
  line_strip.id = 1;
  goal_circle.id = 2;
  forward_points.id = 3;
  forward_line_strip.id = 4;
  marker.id = 5;

  points.type = visualization_msgs::Marker::POINTS;
  forward_points.type = visualization_msgs::Marker::POINTS;
  line_strip.type = visualization_msgs::Marker::LINE_STRIP;
  forward_line_strip.type = visualization_msgs::Marker::LINE_STRIP;
  goal_circle.type = visualization_msgs::Marker::CYLINDER;
  marker.type = visualization_msgs::Marker::ARROW;
  // POINTS markers use x and y scale for width/height respectively
  points.scale.x = 0.2;
  points.scale.y = 0.2;

  // LINE_STRIP markers use only the x component of scale, for the line width
  line_strip.scale.x = 0.03;
  forward_line_strip.scale.x = 0.03;

  goal_circle.scale.x = _goalRadius;
  goal_circle.scale.y = _goalRadius;
  goal_circle.scale.z = 0.1;

  // Points are green
  points.color.g = 1.0f;
  points.color.a = 1.0;
 
  // Line strip is blue
  line_strip.color.b = 1.0;
  line_strip.color.a = 1.0;
  
  //forward_points
  forward_points.color.g = 1.0f;
  forward_points.color.b = 1.0;
  forward_points.color.a = 1.0;

  //forward_line_strip is red
  forward_line_strip.color.r = 1.0;
  forward_line_strip.color.a = 1.0;


  // goal_circle is yellow
  goal_circle.color.r = 1.0;
  goal_circle.color.g = 1.0;
  goal_circle.color.b = 0.0;
  goal_circle.color.a = 0.5;
}


void MPCNode::publishSupplementaryMarker(const nav_msgs::Path& mpc_path,double n)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = "odom";
  marker.header.stamp = ros::Time::now();
  marker.ns ="Supplementary_Markers";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::ARROW;
  marker.action = visualization_msgs::Marker::ADD;

  marker.scale.x = 0.1;
  marker.scale.y = 0.05;
  marker.scale.z = 0.05;

  marker.color.a = 1.0; // Don't forget to set the alpha!
  marker.color.r = 0.0;
  marker.color.g = 0.0;
  marker.color.b = 1.0;
  visualization_msgs::MarkerArray marray;

  int size = mpc_path.poses.size();
  int tag = 0;
  for (int j = n; j < size; j++) {
    marker.header.stamp = ros::Time::now();
    marker.id = tag++;
    marker.pose.position.x = mpc_path.poses[j].pose.position.x;
    marker.pose.position.y = mpc_path.poses[j].pose.position.y;
    marker.pose.position.z = 0;
    marker.pose.orientation = mpc_path.poses[j].pose.orientation;
    marker.lifetime = ros::Duration();
    marray.markers.emplace_back(marker);
  }
  Supplementary_marker_pub_.publish(marray);
}

void MPCNode::publishPredictionMarker(const nav_msgs::Path& mpc_path)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = "odom";
  marker.header.stamp = ros::Time::now();
  marker.ns ="Prediction_Markers";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::ARROW;
  marker.action = visualization_msgs::Marker::ADD;

  marker.scale.x = 0.1;
  marker.scale.y = 0.05;
  marker.scale.z = 0.05;

  marker.color.a = 1.0; // Don't forget to set the alpha!
  marker.color.r = 0.0;
  marker.color.g = 0.0;
  marker.color.b = 1.0;
  visualization_msgs::MarkerArray marray;

  int size = mpc_path.poses.size();
  int tag = 0;
  for (int j = 0; j < size; j++) {
    marker.header.stamp = ros::Time::now();
    marker.id = tag++;
    marker.pose.position.x = mpc_path.poses[j].pose.position.x;
    marker.pose.position.y = mpc_path.poses[j].pose.position.y;
    marker.pose.position.z = 0;
    // tf::Quaternion q;
    // q.setRPY(0, 0, desired_path[j].yaw());
    // marker.pose.orientation.x = q.x();
    // marker.pose.orientation.y = q.y();
    // marker.pose.orientation.z = q.z();
    // marker.pose.orientation.w = q.w();
    marker.pose.orientation = mpc_path.poses[j].pose.orientation;
    marker.lifetime = ros::Duration();
    marray.markers.emplace_back(marker);
  }
  Prediction_marker_pub_.publish(marray);
}

MPCNode::~MPCNode()
{
   // file.close();
    
};

// Public: return _thread_numbers
int MPCNode::get_thread_numbers()
{
    return _thread_numbers;
}


// Evaluate a polynomial.
double MPCNode::polyeval(Eigen::VectorXd coeffs, double x) 
{
    double result = 0.0;
    for (int i = 0; i < coeffs.size(); i++) 
    {
        result += coeffs[i] * pow(x, i);
    }
    return result;
}


// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd MPCNode::polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order) 
{
    assert(xvals.size() == yvals.size());
    assert(order >= 1 && order <= xvals.size() - 1);
    Eigen::MatrixXd A(xvals.size(), order + 1);

    for (int i = 0; i < xvals.size(); i++)
        A(i, 0) = 1.0;

    for (int j = 0; j < xvals.size(); j++) 
    {
        for (int i = 0; i < order; i++) 
            A(j, i + 1) = A(j, i) * xvals(j);
    }

    auto Q = A.householderQr();
    auto result = Q.solve(yvals);
    return result;
}

// CallBack: Update odometry
void MPCNode::odomCB(const nav_msgs::Odometry::ConstPtr& odomMsg)
{
    _odom = *odomMsg; 
}

// CallBack: Update generated path (conversion to odom frame)
//从最近点开始，在全局路径中获得期望路径
void MPCNode::desiredPathCB(const nav_msgs::Path& totalPathMsg)
{
    // 打印路径消息的信息
    ROS_INFO("-------------Enter desiredPathCB---------------------");
    ROS_INFO("Received path message with %zu poses", totalPathMsg.poses.size());
     if (totalPathMsg.header.frame_id.empty()){
        ROS_WARN("Invalid frame ID detected in pose");
     }
    nav_msgs::Path transformedPath;
    //这里将路径从map坐标系转到odom坐标系上
    for (size_t i = 0; i < totalPathMsg.poses.size(); i++) {
      geometry_msgs::PoseStamped poseInMap = totalPathMsg.poses[i];
      geometry_msgs::PoseStamped poseInOdom;

      try {
        // 将姿态信息从map转换到目标坐标系
        _tf_listener.transformPose(_odom_frame, ros::Time(0) , 
                                            poseInMap, "map", poseInOdom);   
        // 添加转换后的姿态信息到新路径中
        transformedPath.poses.push_back(poseInOdom);
      } catch (tf::TransformException& ex) {
        ROS_ERROR("[%s][%d] Failed to transform pose: %s",__func__,__LINE__, ex.what());
      }
    }
    //_gen_path = *totalPathMsg;
    _gen_path = transformedPath;
    _goal_received = true;
    nav_msgs::Path mpc_path = nav_msgs::Path();   // For generating mpc reference path  
    geometry_msgs::PoseStamped tempPose;
    nav_msgs::Odometry odom = _odom; 
    try
    {
        double total_length = 0.0;
        //find waypoints distance
        if(_waypointsDist <= 0.0)
        {   
            ROS_INFO("transformedPath.poses[1].pose.position.x=%f,transformedPath.poses[0].pose.position.x=%f,transformedPath.poses[1].pose.position.y=%f,transformedPath.poses[0].pose.position.y=%f", transformedPath.poses[1].pose.position.x, transformedPath.poses[0].pose.position.x,
                     transformedPath.poses[1].pose.position.y,transformedPath.poses[0].pose.position.y);
            double gap_x = transformedPath.poses[1].pose.position.x - transformedPath.poses[0].pose.position.x;
            double gap_y = transformedPath.poses[1].pose.position.y - transformedPath.poses[0].pose.position.y;
            _waypointsDist = sqrt(gap_x*gap_x + gap_y*gap_y);
            ROS_INFO("gap_x=%f,gap_y=%f,_waypointsDist=%f", gap_x, gap_y,
                     _waypointsDist);
        }                       

        // Find the nearst point for robot position
        int min_val = 100; 

        int N = transformedPath.poses.size(); // Number of waypoints        
        const double px = odom.pose.pose.position.x; //pose: odom frame
        const double py = odom.pose.pose.position.y;
        const double ptheta = odom.pose.pose.position.y;
        
        double dx, dy; // difference distance
        double pre_yaw = 0;
        double roll, pitch, yaw = 0;
        for(int i = min_idx; i < N; i++) 
        {
            dx = transformedPath.poses[i].pose.position.x - px;
            dy = transformedPath.poses[i].pose.position.y - py;
                    
            tf::Quaternion q(
                transformedPath.poses[i].pose.orientation.x,
                transformedPath.poses[i].pose.orientation.y,
                transformedPath.poses[i].pose.orientation.z,
                transformedPath.poses[i].pose.orientation.w);
            tf::Matrix3x3 m(q);
            m.getRPY(roll, pitch, yaw);
            
            if(abs(pre_yaw - yaw) > 5)
            {
                cout << "abs(pre_yaw - yaw)" << abs(pre_yaw - yaw) << endl;
                pre_yaw = yaw;
            }
       
            if(min_val > sqrt(dx*dx + dy*dy) && abs((int)(i - min_idx)) < 50)
            {
                min_val = sqrt(dx*dx + dy*dy);
                min_idx = i;
            }
        }
        for(int i = min_idx; i < N ; i++)
        {
            if(total_length > _pathLength)
                break;
            tempPose = transformedPath.poses[i];
            mpc_path.poses.push_back(tempPose);                          
            total_length = total_length + _waypointsDist;           
        }   
        // Connect the end of path to the front
        //如果剩下的路径长度不够，那就拿最开始的路径来补，这个有问题，因为原来的路径是闭环的
        //这里对于预测是否存在问题
        double endpoint_yaw = tf::getYaw(transformedPath.poses[N - 1].pose.orientation);
        double n = total_length / _waypointsDist;
        if (total_length < _pathLength) {
          for (int i = 1; i < N; i++) {
            if (total_length > _pathLength) break;
            // 从输入路径的最后一点开始，沿该点的yaw角度预测新的路径点
            tempPose.pose.position.x =transformedPath.poses[N - 1].pose.position.x +_waypointsDist * i * std::cos(endpoint_yaw);
            tempPose.pose.position.y =transformedPath.poses[N - 1].pose.position.y +_waypointsDist * i * std::sin(endpoint_yaw);
            tempPose.pose.orientation =transformedPath.poses[N - 1].pose.orientation;
            mpc_path.poses.push_back(tempPose);
            total_length = total_length + _waypointsDist;
          }

          //这里把预测部分的mpc_path可视化出来
          publishSupplementaryMarker(mpc_path, n);
        }
        
        if(mpc_path.poses.size() >= _pathLength )
        {
            _odom_path = mpc_path; // Path waypoints in odom frame
            _path_computed = true;
            // publish odom path
            mpc_path.header.frame_id = _odom_frame;
            mpc_path.header.stamp = ros::Time::now();
            _pub_odompath.publish(mpc_path);
        }
        else
        {
            cout << "Failed to path generation" << endl;
            _waypointsDist = -1;
        }     
    }
    catch(tf::TransformException &ex)
    {
        ROS_ERROR("[%s][%d]  %s",__func__,__LINE__,ex.what());
        ros::Duration(1.0).sleep();
    }
}

// CallBack: Update path waypoints (conversion to odom frame)
void MPCNode::pathCB(const nav_msgs::Path::ConstPtr& pathMsg)
{    
}

void MPCNode::initposeCB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
  ROS_INFO("initpose Received ");
  ROS_INFO("Received pose message with frame_id: %s", msg->header.frame_id.c_str());
  start_pose_.set_x(msg->pose.pose.position.x);
  start_pose_.set_y(msg->pose.pose.position.y);
  start_pose_.set_yaw(tf::getYaw(msg->pose.pose.orientation));
}
// CallBack: Update goal status
void MPCNode::goalCB(const geometry_msgs::PoseStamped::ConstPtr& goalMsg)
{
    _goal_pos = goalMsg->pose.position;
    _goal_received = true;
    _goal_reached = false;
  
  ROS_INFO("goal Received ");
  target_pose_.set_x(goalMsg->pose.position.x);
  target_pose_.set_y(goalMsg->pose.position.y);
  target_pose_.set_yaw(tf::getYaw(goalMsg->pose.orientation));
  std::vector<Point> ref_line;
  std::vector<Point> desired_path;
  nav_msgs::Path nav_path;
  auto path_generator = planning::PathGenerator::Instance();
  // 
  path_generator.GenerateFrenetBasedSpline(start_pose_, target_pose_, ref_line,
                                           &desired_path);
  //path_generator.publishMarker(desired_path);
  path_generator.converttrajToNavPath(desired_path,nav_path);
  // 将得到的五次多项式转成需要的mpc_path
  desiredPathCB(nav_path);
 
}


// Callback: Check if the car is inside the goal area or not 
void MPCNode::amclCB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& amclMsg)
{
   
//   ROS_INFO("position.x=%f,position.y=%f", amclMsg->pose.pose.position.x,
//            amclMsg->pose.pose.position.y);
  if (_goal_received) {
    // ROS_INFO("goal_position.x=%f,goal_position.y=%f", _goal_pos.x,
    //        _goal_pos.y);
    double car2goal_x = _goal_pos.x - amclMsg->pose.pose.position.x;
    double car2goal_y = _goal_pos.y - amclMsg->pose.pose.position.y;
    double dist2goal = sqrt(car2goal_x * car2goal_x + car2goal_y * car2goal_y);
    if (dist2goal < _goalRadius) {
      _goal_received = false;
      _goal_reached = true;
      _path_computed = false;
      ROS_INFO("Goal Reached !");
    }
  }
}


// Timer: Control Loop (closed loop nonlinear MPC)
void MPCNode::controlLoopCB(const ros::TimerEvent&)
{     
    if(_goal_received &&!_goal_reached){ //received goal & goal not reached
    ROS_INFO("-------------Enter controlLoopCB2---------------------");    
    nav_msgs::Odometry odom = _odom;
    nav_msgs::Path odom_path = _odom_path;

    // Update system states: X=[x, y, theta, v]
    const double px = odom.pose.pose.position.x;  // pose: odom frame
    const double py = odom.pose.pose.position.y;
    // 将odom.pose.pose中的姿态信息转换为tf::Pose类型并存储到之前声明的pose对象中
    tf::Pose pose;
    tf::poseMsgToTF(odom.pose.pose, pose);
    const double theta = tf::getYaw(pose.getRotation());
    const double v = odom.twist.twist.linear.x;  // twist: body fixed frame
    // Update system inputs: U=[w, throttle]
    const double w = _w;  // steering -> w
    // const double steering = _steering;  // radian
    const double throttle = _throttle;  // accel: >0; brake: <0
    const double dt = _dt;
    // const double Lf = _Lf;

    // Waypoints related parameters
    const int N = odom_path.poses.size();  // Number of waypoints
    const double costheta = cos(theta);
    const double sintheta = sin(theta);

    // Convert to the vehicle coordinate system
    // 转换到机器人坐标系,frenet坐标系、
    // Odom坐标系转到frenet坐标系
    VectorXd x_veh(N);
    VectorXd y_veh(N);
    for (int i = 0; i < N; i++) {
      const double dx = odom_path.poses[i].pose.position.x - px;
      const double dy = odom_path.poses[i].pose.position.y - py;
      x_veh[i] = dx * costheta + dy * sintheta;
      y_veh[i] = dy * costheta - dx * sintheta;
        }
        
        // Fit waypoints
        /*这行代码使用了一个名为 polyfit 的函数，它的作用是用一个三阶多项式对一组数据点 (x_veh, y_veh) 进行拟合，
        返回多项式的系数。x_veh 和 y_veh 各自包含了车辆坐标系下 waypoints 的 x 和 y 坐标数组。
        3 表示多项式的阶数，即返回的 coeffs 将是一个四元素向量（因为三阶多项式具有四个系数）。*/
        auto coeffs = polyfit(x_veh, y_veh, 5); 
        /*这行代码定义了一个名为 cte （Cross Track Error，横向偏差误差）的常量
        并使用 polyeval 函数来评估上一步拟合的多项式在 x=0 处的值*/
        const double cte  = polyeval(coeffs, 0.0);
        //再定义一个常量 etheta（误差角度），通过计算多项式导数在 x=0 处的反正切值来估计。
        const double etheta = atan(coeffs[1]);
        ROS_INFO("cte=%f,etheta=%f", cte,etheta);

        _mpc_cte = cte;
        _mpc_etheta = etheta;

        VectorXd state(6);
        if(_delay_mode)
        {
            // Kinematic model is used to predict vehicle state at the actual moment of control (current time + delay dt)
            //这部分有点疑问
            const double px_act = v * dt;
            const double py_act = 0;
            const double theta_act = w * dt; //(steering) theta_act = v * steering * dt / Lf;角度
            const double v_act = v + throttle * dt; //v = v + a * dt
            
            const double cte_act = cte + v * sin(etheta) * dt;
            const double etheta_act = etheta - theta_act;  
            
            state << px_act, py_act, theta_act, v_act, cte_act, etheta_act;
        }
        else
        {
            state << 0, 0, 0, v, cte, etheta;
        }
        
        // Solve MPC Problem
        vector<double> mpc_results = _mpc.Solve(state, coeffs);
        //获取mpc预测步长内预测的点的位姿
        // nav_msgs::Path predict_path;
        //  cout << "---------------------1--------------------------" <<endl;
        // for (const auto& t : _mpc.predict_points_tuples_) {
        //   double x, y, yaw;
        //   static int i = 0;
        //   x = std::get<0>(t);
        //   y = std::get<1>(t);
        //   yaw = std::get<2>(t);
        //   cout << "x=" << x << ", y=" << y << ", yaw=" << yaw << endl;
        //   cout << "---------------------3--------------------------" << endl;
        //   predict_path.poses[i].pose.position.x = x;
        //   predict_path.poses[i].pose.position.y = y;
        //   predict_path.poses[i].pose.position.z = 0;
        //   cout << "---------------------4--------------------------" <<endl;
        //   tf::Quaternion q;
        //   q.setRPY(0, 0, yaw);
        //   cout << "---------------------5--------------------------" <<endl;
        //   predict_path.poses[i].pose.orientation.x = q.x();
        //   predict_path.poses[i].pose.orientation.y = q.y();
        //   predict_path.poses[i].pose.orientation.z = q.z();
        //   predict_path.poses[i].pose.orientation.w = q.w();
        //   i++; 
        // }
        // cout << "------------------------2-----------------------" <<endl;
        // publishPredictionMarker(predict_path);
        ROS_INFO("------------mpc solve----------");
        // MPC result (all described in car frame), output = (acceleration, w)        
        _w = mpc_results[0]; // radian/sec, angular velocity
        _throttle = mpc_results[1]; // acceleration
        _speed = v + _throttle*dt;  // speed
        if (_speed >= _max_speed)
            _speed = _max_speed;
        if(_speed <= 0.0)
            _speed = 0.0;

        if(_debug_info)
        {
            cout << "\n\nDEBUG" << endl;
            cout << "theta: " << theta << endl;
            cout << "V: " << v << endl;
            //cout << "odom_path: \n" << odom_path << endl;
            //cout << "x_points: \n" << x_veh << endl;
            //cout << "y_points: \n" << y_veh << endl;
            cout << "coeffs: \n" << coeffs << endl;
            cout << "_w: \n" << _w << endl;
            cout << "_throttle: \n" << _throttle << endl;
            cout << "_speed: \n" << _speed << endl;
        }

        // Display the MPC predicted trajectory
        _mpc_traj = nav_msgs::Path();
        _mpc_traj.header.frame_id = _car_frame; // points in car coordinate        
        _mpc_traj.header.stamp = ros::Time::now();
        for(int i=0; i<_mpc.mpc_x.size(); i++)
        {
            geometry_msgs::PoseStamped tempPose;
            tempPose.header = _mpc_traj.header;
            tempPose.pose.position.x = _mpc.mpc_x[i];
            tempPose.pose.position.y = _mpc.mpc_y[i];
            tempPose.pose.orientation.w = 1.0;
            _mpc_traj.poses.push_back(tempPose); 
        }     
        // publish the mpc trajectory
        _pub_mpctraj.publish(_mpc_traj);
    }
    else{
      _throttle = 0.0;
      _speed = 0.0;
      _w = 0;
    }


    //除非重新设定起点或者终点，机器才会再次启动

    // publish general cmd_vel 
    if(_pub_twist_flag)
    {
        _twist_msg.linear.x  = _speed; 
        _twist_msg.angular.z = _w;
        _pub_twist.publish(_twist_msg);

        std_msgs::Float32 mpc_total_cost;
        mpc_total_cost.data = static_cast<float>(_mpc._mpc_totalcost);
        _pub_totalcost.publish(mpc_total_cost);

        std_msgs::Float32 mpc_cte_cost;
        mpc_cte_cost.data = static_cast<float>(_mpc._mpc_ctecost);
        _pub_ctecost.publish(mpc_cte_cost);

        std_msgs::Float32 mpc_etheta_cost;
        mpc_etheta_cost.data = static_cast<float>(_mpc._mpc_ethetacost);
        _pub_ethetacost.publish(mpc_etheta_cost);

        //cout << "_mpc_totalcost: "<< _mpc._mpc_totalcost << endl;
        //cout << "_mpc_ctecost: "<< _mpc._mpc_ctecost << endl;
        //cout << "_mpc_ethetacost: "<< _mpc._mpc_ethetacost << endl;
        //cout << "_mpc_velcost: "<< _mpc._mpc_velcost << endl;
        //writefile
        idx++;
        //cout << "idx: "<< idx << endl;
        //file << idx<< "," << _mpc_cte<< "," <<  _mpc_etheta << "," << _twist_msg.linear.x<< "," << _twist_msg.angular.z  << ",";

    }
    else
    {
        _twist_msg.linear.x  = 0; 
        _twist_msg.angular.z = 0;
        _pub_twist.publish(_twist_msg);
    }

}

/*****************/
/* MAIN FUNCTION */
/*****************/
int main(int argc, char **argv)
{
    //Initiate ROS
    ros::init(argc, argv, "MPC_Node");
    MPCNode mpc_node;

    ROS_INFO("Waiting for global path msgs ~");
    ros::AsyncSpinner spinner(mpc_node.get_thread_numbers()); // Use multi threads
    spinner.start();
    ros::waitForShutdown();
    return 0;
}
