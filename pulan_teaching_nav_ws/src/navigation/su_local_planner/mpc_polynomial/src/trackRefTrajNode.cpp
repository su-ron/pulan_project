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

#include "trackRefTraj.h"
#include "polynomial_curve.h"
#include <Eigen/Core>
#include <Eigen/QR>


// inlcude iostream and string libraries
#include <iostream>
#include <fstream>
#include <string>


using namespace std;
using namespace Eigen;




/********************/
/* CLASS DEFINITION */
/********************/
class MPCNode
{
    public:
        MPCNode();
        void initMarker();
        ~MPCNode();
        int get_thread_numbers();
        
    private:
        ros::NodeHandle _nh;
        ros::Subscriber _sub_odom, _sub_gen_path, _sub_path, _sub_goal, _sub_amcl;
        ros::Publisher _pub_totalcost, _pub_ctecost, _pub_ethetacost,_pub_odompath, _pub_twist, _pub_ackermann, _pub_mpctraj,marker_pub;
        ros::Timer _timer1;
        tf::TransformListener _tf_listener;
        
        visualization_msgs::Marker points,forward_points, line_strip,forward_line_strip, goal_circle;

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
        void desiredPathCB(const nav_msgs::Path::ConstPtr& pathMsg);
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
    _sub_gen_path   = _nh.subscribe( "/mpc/global_plan", 1, &MPCNode::desiredPathCB, this);
    _sub_goal   = _nh.subscribe( "/mpc/goal", 1, &MPCNode::goalCB, this);
    _sub_amcl   = _nh.subscribe("/localization_pose", 5, &MPCNode::amclCB, this);
    
    _pub_odompath  = _nh.advertise<nav_msgs::Path>("/mpc_reference", 1); // reference path for MPC ///mpc_reference 
    _pub_mpctraj   = _nh.advertise<nav_msgs::Path>("/mpc_trajectory", 1);// MPC trajectory output
    _pub_ackermann = _nh.advertise<ackermann_msgs::AckermannDriveStamped>("/ackermann_cmd", 1);
    if(_pub_twist_flag)
        _pub_twist = _nh.advertise<geometry_msgs::Twist>("/mpc/round_cmd_vel", 1); //for stage (Ackermann msg non-supported)
    
    marker_pub =
      _nh.advertise<visualization_msgs::Marker>("/path_marker", 10);

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
  =forward_line_strip.header.frame_id =goal_circle.header.frame_id = "odom";

  points.ns =forward_points.ns = line_strip.ns =forward_line_strip.ns = goal_circle.ns = "Markers";

  points.action = forward_points.action = line_strip.action = forward_line_strip.action = goal_circle.action =
      visualization_msgs::Marker::ADD;
  points.pose.orientation.w = forward_points.pose.orientation.w = line_strip.pose.orientation.w 
  = forward_line_strip.pose.orientation.w =goal_circle.pose.orientation.w = 1.0;

  points.id = 0;
  line_strip.id = 1;
  goal_circle.id = 2;
  forward_points.id = 3;
  forward_line_strip.id = 4;

  points.type = visualization_msgs::Marker::POINTS;
  forward_points.type = visualization_msgs::Marker::POINTS;
  line_strip.type = visualization_msgs::Marker::LINE_STRIP;
  forward_line_strip.type = visualization_msgs::Marker::LINE_STRIP;
  goal_circle.type = visualization_msgs::Marker::CYLINDER;
  // POINTS markers use x and y scale for width/height respectively
  points.scale.x = 0.2;
  points.scale.y = 0.2;

  // LINE_STRIP markers use only the x component of scale, for the line width
  line_strip.scale.x = 0.1;
  forward_line_strip.scale.x = 0.1;

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
void MPCNode::desiredPathCB(const nav_msgs::Path::ConstPtr& totalPathMsg)
{
    // 打印路径消息的信息
    //ROS_INFO("Received path message with %zu poses", totalPathMsg->poses.size());
     if (totalPathMsg->header.frame_id.empty()){
        ROS_WARN("Invalid frame ID detected in pose");
     }
    nav_msgs::Path transformedPath;
    for (size_t i = 0; i < totalPathMsg->poses.size(); i++) {
      geometry_msgs::PoseStamped poseInMap = totalPathMsg->poses[i];
      geometry_msgs::PoseStamped poseInOdom;

      try {
        // 将姿态信息从map转换到目标坐标系
        _tf_listener.transformPose(_odom_frame, ros::Time(0) , 
                                            poseInMap, "map", poseInOdom);   
        // 添加转换后的姿态信息到新路径中
        transformedPath.poses.push_back(poseInOdom);
      } catch (tf::TransformException& ex) {
        ROS_ERROR("Failed to transform pose: %s", ex.what());
      }
    }

  /*Visualized Target Point on RVIZ*/
  /*Clear former target point Marker*/
  points.points.clear();
  line_strip.points.clear();

  for (size_t i = 0; i < transformedPath.poses.size(); i++) {
    geometry_msgs::Point waypoints =transformedPath.poses[i].pose.position;
    points.points.push_back(waypoints);
    line_strip.points.push_back(waypoints);
  }
  marker_pub.publish(points);
  marker_pub.publish(line_strip);
    
    //_gen_path = *totalPathMsg;
    //_gen_path得到的路径是基于odom坐标系的
    _gen_path = transformedPath;
    _goal_received = true;
    //_goal_reached = false;
    nav_msgs::Path mpc_path = nav_msgs::Path();   // For generating mpc reference path  
    geometry_msgs::PoseStamped tempPose;
    nav_msgs::Odometry odom = _odom; 
    try
    {
        double total_length = 0.0;
        //find waypoints distance
        if(_waypointsDist <= 0.0)
        {        
            double gap_x = transformedPath.poses[1].pose.position.x - transformedPath.poses[0].pose.position.x;
            double gap_y = transformedPath.poses[1].pose.position.y - transformedPath.poses[0].pose.position.y;
            _waypointsDist = sqrt(gap_x*gap_x + gap_y*gap_y);             
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
        //从map坐标系转换到odom坐标系，得到mpc_path(从最近点开始)
        for(int i = min_idx; i < N ; i++)
        {
            if(total_length > _pathLength)
                break;
            
            _tf_listener.transformPose(_odom_frame, ros::Time(0) , 
                                            transformedPath.poses[i], _odom_frame, tempPose);                     
            mpc_path.poses.push_back(tempPose);                          
            total_length = total_length + _waypointsDist;           
        }   
        // Connect the end of path to the front
        //如果剩下的路径长度不够，那就拿最开始的路径来补
        if(total_length < _pathLength )
        {
            for(int i = 0; i < N ; i++)
            {
                if(total_length > _pathLength)                
                    break;
                _tf_listener.transformPose(_odom_frame, ros::Time(0) , 
                                                transformedPath.poses[i], _odom_frame, tempPose);                     
                mpc_path.poses.push_back(tempPose);                          
                total_length = total_length + _waypointsDist;    
            }
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
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }
    
}

// CallBack: Update path waypoints (conversion to odom frame)
void MPCNode::pathCB(const nav_msgs::Path::ConstPtr& pathMsg)
{    
}

// CallBack: Update goal status
void MPCNode::goalCB(const geometry_msgs::PoseStamped::ConstPtr& goalMsg)
{
    _goal_pos = goalMsg->pose.position;
    _goal_received = true;
    _goal_reached = false;

      try {
    geometry_msgs::PoseStamped odom_goal;
    _tf_listener.transformPose("odom", ros::Time(0), *goalMsg, "map", odom_goal);

    /*Draw Goal on RVIZ*/
    goal_circle.pose = odom_goal.pose;
    marker_pub.publish(goal_circle);
  } catch (tf::TransformException& ex) {
    ROS_ERROR("1111 %s", ex.what());
    ros::Duration(0.1).sleep();
  }


    ROS_INFO("Goal Received :goalCB!");
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
    if(_goal_received&&!_goal_reached&&_path_computed){ //received goal & goal not reached    

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
        auto coeffs = polyfit(x_veh, y_veh, 3); 
        /*这行代码定义了一个名为 cte （Cross Track Error，横向偏差误差）的常量
        并使用 polyeval 函数来评估上一步拟合的多项式在 x=0 处的值*/
        const double cte  = polyeval(coeffs, 0.0);
        //再定义一个常量 etheta（误差角度），通过计算多项式导数在 x=0 处的反正切值来估计。
        const double etheta = atan(coeffs[1]);

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
      if(_goal_reached)
        ROS_INFO("Goal Reached: control loop !");
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
    
  
    /*
    file.open("/home/geonhee/catkin_ws/src/mpc_ros/write.csv");
    string line;
    while (getline(file, line,'\n')) 
    {
        istringstream templine(line); 
        string data;
        while (getline( templine, data,',')) 
        {
            cout << "data.c_str(): "<< data << endl;
            matrix.push_back(atof(data.c_str()));  
        }
    }
    file.close();*/

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
