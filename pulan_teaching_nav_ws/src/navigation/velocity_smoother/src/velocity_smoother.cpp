/**
 * @file /src/velocity_smoother_nodelet.cpp
 *
 * @brief Velocity smoother implementation.
 *
 * License: BSD
 *   https://raw.github.com/yujinrobot/yujin_ocs/hydro/yocs_velocity_smoother/LICENSE
 **/
/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include <ros/ros.h>
#include "velocity_smoother/velocity_smoother.hpp"

/*****************************************************************************
 ** Preprocessing
 *****************************************************************************/

#define PERIOD_RECORD_SIZE    5
#define ZERO_VEL_COMMAND      geometry_msgs::Twist();
#define IS_ZERO_VEOCITY(a)   ((a.linear.x == 0.0) && (a.angular.z == 0.0))

namespace velocity_smoother {
  /*********************
  ** Implementation
  **********************/
  VelocitySmoother::VelocitySmoother() {
    ros::NodeHandle nh("~");
    // Optional parameters
    int feedback;
    is_Smoother_Init = false;

    if(!is_Smoother_Init) {
      VelocitySmootherInit();
    }

    nh.param("is_debug",       is_debug,     false);
    nh.param("frequency",      frequency,     15.0);
    nh.param("decel_factor",   decel_factor,   1.0);
    nh.param("robot_feedback", feedback, (int)NONE);

    if ((int(feedback) < NONE) || (int(feedback) > COMMANDS)) {
      ROS_WARN("Invalid robot feedback type (%d). Valid options are 0 (NONE, default), 1 (ODOMETRY) and 2 (COMMANDS)",
              feedback);
      feedback = NONE;
    }

    robot_feedback = static_cast<RobotFeedbackType>(feedback);

    // Mandatory parameters
    nh.param("speed_lim_v",     speed_lim_v,           1.5);
    nh.param("speed_lim_w",     speed_lim_w,           5.4);
    nh.param("accel_lim_v",     accel_lim_v,           1.0);
    nh.param("accel_lim_w",     accel_lim_w,           4.5);
    
    // kalman params
    nh.param("Q_noise",          Q_noise,            0.1  );
    nh.param("R_noise",          R_noise,            0.5  );
    nh.param("use_calmanFilter", use_calmanFilter,   false);

    //move filter params
    nh.param("use_movingFilter", use_moving_filter,  false);
    nh.param("move_sample_N",    sample_n_filter,    5    );

    // 后退状态限制开关
    nh.param("use_limit_back",    use_limit_back,    true);

    //limit the start speed
    nh.param("use_limit_speedup", use_limit_speedup, false);

    nh.param("use_fault_checked", use_fault_checked_, true);

    // 错误状态监测  
    nh.param("odom_delay_s",      odom_delay_s,      0.10 );      /*!> odom数据延迟时间：ms */ 
    nh.param("dev_num_max",       dev_num_max,       20.0 );      /*!> 偏差次数上限 */ 
    nh.param("observe_time_s",    observe_time_s,    2.0  );      /*!> 里程监测时间 */
    nh.param("vel_dev",           vel_dev,           0.40 );      /*!> 前进 Vx 速度偏差量 */
    nh.param("rot_dev",           rot_dev,           0.40 );      /*!> 旋转 Vw 速度偏差量 */
    nh.param("sum_x_dev",         sum_x_dev,         0.50 );      /*!> x方向里程偏差 */
    nh.param("sum_y_dev",         sum_y_dev,         0.50 );      /*!> y方向里程偏差 */
    nh.param("sum_w_dev",         sum_w_dev,         0.50 );      /*!> w方向里程偏差 */

    // Deceleration can be more aggressive, if necessary
    decel_lim_v = decel_factor * accel_lim_v;
    decel_lim_w = decel_factor * accel_lim_w;

    // Publishers and subscribers
    odometry_sub      = nh.subscribe("odometry",      1, &VelocitySmoother::odometryCB, this);
    current_vel_sub   = nh.subscribe("robot_cmd_vel", 1, &VelocitySmoother::robotVelCB, this);
    raw_in_vel_sub    = nh.subscribe("raw_cmd_vel",   1, &VelocitySmoother::velocityCB, this);
    smooth_vel_pub    = nh.advertise <geometry_msgs::TwistStamped> ("smooth_cmd_vel", 1, true);
    fault_state_pub   = nh.advertise <system_msgs::DiagnosticStatus> ("/robot_status/fault_status", 1, true);
    lift_state_sub    = nh.subscribe("/elevator/nav_mode_", 1, &VelocitySmoother::liftStateCB, this);
    charge_state_sub  = nh.subscribe("/up_and_down_pile_status", 1, &VelocitySmoother::chargeStateCB, this);
    go_back_state_sub = nh.subscribe("/switch_side_lidar_back_detection", 1, &VelocitySmoother::goBackStateCB, this);
    rrt_state_sub     = nh.subscribe("/rrt_state", 1, &VelocitySmoother::rrtStateCB, this);

    // 异常服务急停 
    urgency_stop_srv = nh.advertiseService("/FaultDiagnosis/unrecoverable_fault/lock_robot", &VelocitySmoother::UrgencyStopSrv, this);
    fault_diagnostic_srv = nh.advertiseService("/FaultDiagnosis/recoverable_fault/lock_robot", &VelocitySmoother::FaultDiagnosticSrv, this);

    // 状态监控初始化
    Error_state = State_controlPtr(new state_control(odom_delay_s, dev_num_max, observe_time_s, 1 / frequency, 
                                                     vel_dev, rot_dev, sum_x_dev, sum_y_dev, sum_w_dev, 
                                                     is_debug));
    // 动态调参
    if (is_debug) {
      dynamic_reconfigure_callback = boost::bind(&VelocitySmoother::reconfigCB, this, _1);           //绑定回调函数
      dynamic_reconfigure_server.setCallback(dynamic_reconfigure_callback);  //为服务器设置回调函数， 节点程序运行时会调用一次回调函数来输出当前的参数配置情况
    }

    ROS_INFO_COND(1, "\n\n[/velocity_smoother] Get param:\n frequency: %f, decel_factor: %f, feedback: %d \n Vx: %f, Vw: %f, ax: %f, aw: %f \n use_limit_back: %d \n use_limit_speedup: %d \n is_debug: %d \n",
             frequency, decel_factor, feedback, speed_lim_v, speed_lim_w, accel_lim_v, accel_lim_w, use_limit_back, use_limit_speedup, is_debug);

  }

  void VelocitySmoother::velocityCB(const geometry_msgs::Twist::ConstPtr& msg)
  {
    //ROS_INFO("Velocity callback ...");
    // Estimate commands frequency; we do continuously as it can be very different depending on the
    // publisher type, and we don't want to impose extra constraints to keep this package flexible
    if (period_record.size() < PERIOD_RECORD_SIZE) {
      period_record.push_back((ros::Time::now() - last_cb_time).toSec());
    }else{
      period_record[pr_next] = (ros::Time::now() - last_cb_time).toSec();
    }

    pr_next++;
    pr_next %= period_record.size();
    last_cb_time = ros::Time::now();

    if (period_record.size() <= PERIOD_RECORD_SIZE/2) {
      // wait until we have some values; make a reasonable assumption (10 Hz) meanwhile
      cb_avg_time = 0.1;
    }else{
      // enough; recalculate with the latest input
      cb_avg_time = median(period_record);
    }

    input_active = true;
    cmd_last_time = ros::Time::now();

    // Bound speed with the maximum values
    target_vel.linear.x  =
        msg->linear.x  > 0.0 ? std::min(msg->linear.x,  speed_lim_v) : std::max(msg->linear.x,  -speed_lim_v);
    target_vel.angular.z =
        msg->angular.z > 0.0 ? std::min(msg->angular.z, speed_lim_w) : std::max(msg->angular.z, -speed_lim_w);
    target_vel.linear.y = msg->linear.y;
  }

  void VelocitySmoother::odometryCB(const nav_msgs::Odometry::ConstPtr& msg)
  {
    //ROS_INFO("Odometry callback ...");
      current_vel = msg->twist.twist;
      odom = *msg;
  }

  void VelocitySmoother::robotVelCB(const geometry_msgs::Twist::ConstPtr& msg)
  {
    //ROS_INFO("Robot velocity callback ...");
    if (robot_feedback == COMMANDS)
      current_vel = *msg;
  }

  void VelocitySmoother::reconfigCB(velocity_smoother::VelocitySmoothConfig &config)
  {
    speed_lim_v = config.speed_lim_v; 
    speed_lim_w = config.speed_lim_w;   
    accel_lim_v = config.accel_lim_v; 
    accel_lim_w = config.accel_lim_w;  
    frequency = config.frequency;   
    decel_factor = config.decel_factor;  
    Q_noise = config.Q_noise; 
    R_noise = config.R_noise; 
    sample_n_filter = config.move_sample_N;
    odom_delay_s = config.odom_delay_s; 
    dev_num_max = config.dev_num_max;   
    observe_time_s = config.observe_time_s;
    vel_dev = config.vel_dev;
    rot_dev = config.rot_dev; 
    sum_x_dev = config.sum_x_dev;  
    sum_y_dev = config.sum_y_dev;   
    sum_w_dev = config.sum_w_dev;  
    use_calmanFilter = config.use_calmanFilter;
    use_moving_filter = config.use_movingFilter;
    use_limit_back = config.use_limit_back;
    use_limit_speedup = config.use_limit_speedup;
    use_fault_checked_ = config.use_fault_checked;
    is_debug = config.is_debug;
    
  }

  void VelocitySmoother::liftStateCB(const std_msgs::Int32& msg)
  {
    int temp_state = msg.data; 
    if (temp_state == 3 ){
      lift_back_state = true;
    } else {
      lift_back_state = false;
    }
    ROS_INFO_COND(is_debug,"[VelocitySmoother]<liftStateCB> lift_state: %d", lift_back_state);
  
  }

  void VelocitySmoother::chargeStateCB(const std_msgs::Int8& msg)
  {
    int temp_state = msg.data; 
    if (temp_state == 5 || temp_state == 4 || temp_state == 8){
      charge_back_state = true;
    } else {
      charge_back_state = false;
    }

    if (temp_state == 6) {
      Error_state->vel_dev = 0.2;
      Error_state->rot_dev = 0.2;
    } else if (temp_state == 7) {
      Error_state->vel_dev = vel_dev;
      Error_state->rot_dev = rot_dev;
    } else {
      Error_state->vel_dev = vel_dev;
      Error_state->rot_dev = rot_dev;
    }
    
    ROS_INFO_COND(is_debug,"[VelocitySmoother]<liftStateCB> charge_back_state: %d", charge_back_state);
  
  }

  void VelocitySmoother::goBackStateCB(const std_msgs::Bool& msg) {
    go_back_state = msg.data;
  }

  void VelocitySmoother::rrtStateCB(const std_msgs::Bool& msg){
    rrt_state_ = msg.data;
  }

  bool VelocitySmoother::UrgencyStopSrv(custom_msgs_srvs::SetBool::Request &req, custom_msgs_srvs::SetBool::Response &res)
  {
    ROS_INFO_COND(is_debug,"[%s][%d] VelocitySmoother(srv): <UrgencyStop: %d>",__func__,__LINE__,req.data);
    
    urgency_stop_Flag = req.data;
    res.success = true;
    return true;
  }

  bool VelocitySmoother::FaultDiagnosticSrv(custom_msgs_srvs::SetBool::Request &req, custom_msgs_srvs::SetBool::Response &res)
  {
    ROS_INFO_COND(is_debug,"[%s][%d] VelocitySmoother(srv): <FaultDiagnostic: %d>",__func__,__LINE__,req.data);

    fault_diagnostic_Flag = req.data;
    res.success = true;
    return true;
  }

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  void VelocitySmoother::spin() {
    double period = 1.0 / frequency;
    ros::Rate spin_rate(frequency);

    while (ros::ok()) {
      if ((robot_feedback != NONE) && input_active && (cb_avg_time > 0.0) &&
          (((ros::Time::now() - last_cb_time).toSec() > 5.0*cb_avg_time)     || // 5 missing msgs
            (std::abs(current_vel.linear.x  - last_cmd_vel.linear.x)  > 0.2) ||
            (std::abs(current_vel.angular.z - last_cmd_vel.angular.z) > 2.0))) {
        
        // If the publisher has been inactive for a while, or if our current commanding differs a lot
        // from robot velocity feedback, we cannot trust the former; relay on robot's feedback instead
        // This can happen mainly due to preemption of current controller on velocity multiplexer.
        // TODO: current command/feedback difference thresholds are 진짜 arbitrary; they should somehow
        // be proportional to max v and w...
        // The one for angular velocity is very big because is it's less necessary (for example the
        // reactive controller will never make the robot spin) and because the gyro has a 15 ms delay
        ROS_WARN("Using robot velocity feedback (%s) instead of last command: %f, %f, %f",
                  robot_feedback == ODOMETRY ? "odometry" : "end commands",
                 (ros::Time::now()      - last_cb_time).toSec(),
                  current_vel.linear.x  - last_cmd_vel.linear.x,
                  current_vel.angular.z - last_cmd_vel.angular.z);
        last_cmd_vel = current_vel;
      }

      geometry_msgs::TwistPtr cmd_vel;
      geometry_msgs::TwistStamped cmd_vel_stamp;
      cmd_vel_stamp.header.frame_id = "odom";
      cmd_vel_stamp.header.stamp = ros::Time::now();
      double cmd_time = (ros::Time::now() - cmd_last_time).toSec();

      // 监控控制延时,失控 1.0 秒之后停下来
      if (cmd_time > 0.5) {          //1.0, 0.5      
        cmd_timeout_Flag = true;
      } else {
        cmd_timeout_Flag = false;
      }

      // 直接制动为 0速
      if ((fabs(target_vel.linear.x) < 1e-3) && (fabs(target_vel.linear.y + 1.0) < 1e-3)) {
        break_vel_flag_ = true;
      } else {
        break_vel_flag_ = false;
      }

      if ((target_vel.linear.x  != last_cmd_vel.linear.x) ||
          (target_vel.angular.z != last_cmd_vel.angular.z)) {
        // Try to reach target velocity ensuring that we don't exceed the acceleration limits
        cmd_vel.reset(new geometry_msgs::Twist(target_vel));
        double v_inc, w_inc, max_v_inc, max_w_inc;

        v_inc = target_vel.linear.x - last_cmd_vel.linear.x;
        if ((robot_feedback == ODOMETRY) && (current_vel.linear.x*target_vel.linear.x < 0.0)) {
          // countermarch (on robots with significant inertia; requires odometry feedback to be detected)
          max_v_inc = decel_lim_v*period;
        } else {
          max_v_inc = ((v_inc*target_vel.linear.x > 0.0)?accel_lim_v:decel_lim_v)*period;
        }

        w_inc = target_vel.angular.z - last_cmd_vel.angular.z;
        if ((robot_feedback == ODOMETRY) && (current_vel.angular.z*target_vel.angular.z < 0.0)) {
          // countermarch (on robots with significant inertia; requires odometry feedback to be detected)
          max_w_inc = decel_lim_w*period;
        } else {
          max_w_inc = ((w_inc*target_vel.angular.z > 0.0)?accel_lim_w:decel_lim_w)*period;
        }

        // Calculate and normalise vectors A (desired velocity increment) and B (maximum velocity increment),
        // where v acts as coordinate x and w as coordinate y; the sign of the angle from A to B determines
        // which velocity (v or w) must be overconstrained to keep the direction provided as command
        double MA = sqrt(    v_inc *     v_inc +     w_inc *     w_inc);
        double MB = sqrt(max_v_inc * max_v_inc + max_w_inc * max_w_inc);

        double Av = std::abs(v_inc) / MA;
        double Aw = std::abs(w_inc) / MA;
        double Bv = max_v_inc / MB;
        double Bw = max_w_inc / MB;
        double theta = atan2(Bw, Bv) - atan2(Aw, Av);

        if (theta < 0) {
          // overconstrain linear velocity
          max_v_inc = (max_w_inc*std::abs(v_inc))/std::abs(w_inc);
        } else {
          // overconstrain angular velocity
          max_w_inc = (max_v_inc*std::abs(w_inc))/std::abs(v_inc);
        }

        if (std::abs(v_inc) > max_v_inc) {
          // we must limit linear velocity
          cmd_vel->linear.x  = last_cmd_vel.linear.x  + sign(v_inc)*max_v_inc;
        }

        if (std::abs(w_inc) > max_w_inc) {
          // we must limit angular velocity
          cmd_vel->angular.z = last_cmd_vel.angular.z + sign(w_inc)*max_w_inc;
        }

        cmd_vel_stamp.twist = *cmd_vel;
        
        // 卡尔曼滤波
        if (use_calmanFilter) {
          vel_v.x_last = cmd_vel_stamp.twist.linear.x;
          vel_w.x_last = cmd_vel_stamp.twist.angular.z;
          cmd_vel_stamp.twist.linear.x  = vel_v.kalman_filter(Q_noise, R_noise, cmd_vel_stamp.twist.linear.x);
          cmd_vel_stamp.twist.angular.z = vel_w.kalman_filter(Q_noise, R_noise, cmd_vel_stamp.twist.angular.z);
        }
        // 滑动滤波 
        if (use_moving_filter) {
         cmd_vel_stamp.twist = moveing_filter_smoothing(cmd_vel_stamp.twist, sample_n_filter);
        }

        // 后退功能限制及监测
        if (use_limit_back) {
          if (!isBack(lift_back_state, charge_back_state, go_back_state, rrt_state_)) {
            // 如果不允许后退，后退异常上报
            if (cmd_vel_stamp.twist.linear.x < -0.005) {
              ROS_WARN_THROTTLE(2.0, "[velocity_smoother] (limit_back)  topic: /cmd_vel is problem -> %f ", cmd_vel_stamp.twist.linear.x);
              // 如果不允许后退，后退异常，速度强行置零
              cmd_vel_stamp.twist.linear.x = 0.00;
            }
            if (current_vel.linear.x < -0.005) {
              ROS_WARN_THROTTLE(2.0, "[velocity_smoother] (limit_back)  topic: /odom is problem -> %f ", current_vel.linear.x);
              // 如果不允许后退，后退异常，速度强行置零
              cmd_vel_stamp.twist.linear.x = 0.00;
            }
          } 
        }

        // 低速消抖
        if (use_limit_speedup) {
          limit_speedup(&cmd_vel_stamp, current_vel);
        }

        if(isForceToStop()) {
          smooth_vel_pub.publish(plt_vel);
          last_cmd_vel = plt_vel.twist;
        }else{
          smooth_vel_pub.publish(cmd_vel_stamp);
          last_cmd_vel = *cmd_vel;
          plt_vel = cmd_vel_stamp;
        }
        if (use_fault_checked_) {
          FaultStatePub();
        }

      } else if (input_active) {

        // We already reached target velocity; just keep resending last command while input is active
        cmd_vel.reset(new geometry_msgs::Twist(last_cmd_vel));
        cmd_vel_stamp.twist = *cmd_vel;
        
        // 卡尔曼滤波
        if (use_calmanFilter) {
          vel_v.x_last = cmd_vel_stamp.twist.linear.x;
          vel_w.x_last = cmd_vel_stamp.twist.angular.z;
          cmd_vel_stamp.twist.linear.x  = vel_v.kalman_filter(Q_noise, R_noise, cmd_vel_stamp.twist.linear.x);
          cmd_vel_stamp.twist.angular.z = vel_w.kalman_filter(Q_noise, R_noise, cmd_vel_stamp.twist.angular.z);
        }
        
        // 滑动滤波 
        if (use_moving_filter) {
         cmd_vel_stamp.twist = moveing_filter_smoothing(cmd_vel_stamp.twist, sample_n_filter);
        }

        // 后退功能限制及监测
        if (use_limit_back) {
          if (!isBack(lift_back_state, charge_back_state, go_back_state, rrt_state_)) {
            // 如果不允许后退，后退异常上报
            if (cmd_vel_stamp.twist.linear.x < -0.005) {
              ROS_WARN_THROTTLE(0.5, "[velocity_smoother] (limit_back)  topic: /cmd_vel is problem -> %f ", cmd_vel_stamp.twist.linear.x);
              // 如果不允许后退，后退异常，速度强行置零
              cmd_vel_stamp.twist.linear.x = 0.00;
            }
            if (current_vel.linear.x < -0.005) {
              ROS_WARN_THROTTLE(0.5, "[velocity_smoother] (limit_back)  topic: /odom is problem -> %f ", current_vel.linear.x);
              // 如果不允许后退，后退异常，速度强行置零
              cmd_vel_stamp.twist.linear.x = 0.00;
            }
          } 
        }
        
        // 低速消抖
        if (use_limit_speedup) {
          limit_speedup(&cmd_vel_stamp, current_vel);
        }
        
        if(isForceToStop()) {
          smooth_vel_pub.publish(plt_vel);
          last_cmd_vel = plt_vel.twist;
        }else{
          smooth_vel_pub.publish(cmd_vel_stamp);
          plt_vel = cmd_vel_stamp;
        }
        if (use_fault_checked_) {
          FaultStatePub();
        }
        //input_active = false;

      } else {
        plt_vel.header.stamp = ros::Time::now();
        plt_vel.twist.linear.x = 0.0;
        plt_vel.twist.angular.z = 0.0;
        smooth_vel_pub.publish(plt_vel);
        last_cmd_vel = plt_vel.twist; 
 
      }

      ros::spinOnce();
      spin_rate.sleep();
    }
  }
  /////////////////////////////////////////////////////////////////////////////////////////////

  /*>卡尔曼滤波函数。---> Gareth <--- */
  double KalmanFilter::kalman_filter(double Q, double R, double ActualValue) {
    /* 预测 */
    x_raw = A*x_last;
    x_raw = A*x;                //根据估计值计算预测值
    //p_raw = A*p_last*A + Q;
    p_raw=A*p*A + Q;            //计算预测值与真实值之间误差的协方差矩阵
    // cout<<"Q is :"<<q<<endl;
    // cout<<"R is :"<<r<<endl;
    /* 评估 */
    K = p_raw*H / (H*p_raw*H+R);            //计算卡尔曼增益
    x = x_raw + K*(ActualValue - H*x_raw);  //计算估计值
    p = (1 - K*H) * p_raw;                  //计算估计值与真实值之间误差的协方差矩阵
    /* 更新 */
    x_last = x;
    //p_last = p;
    /* 返回估计值 */
    return x;
  }
  
  /**<对数据源进行滑动滤波处理。二级平滑，可能会影响时不时性，但不影响真实性。 --> Gareth <-- */
  geometry_msgs::Twist VelocitySmoother::moveing_filter_smoothing(geometry_msgs::Twist current_target_vel, unsigned int sample_n_filter) {

    geometry_msgs::Twist target_vel_sum;          //样本和
    std::vector<geometry_msgs::Twist> buff_target_vel_temp;   //临时量

    buff_target_vel.push_back(current_target_vel);

    if (buff_target_vel.size() < sample_n_filter - 1) {

      current_target_vel.linear.x  = buff_target_vel[0].linear.x;
      current_target_vel.angular.z = buff_target_vel[0].angular.z;
    }
    else if (buff_target_vel.size() == sample_n_filter - 1) {

      for (int i = 0; i < buff_target_vel.size(); i++) {
        target_vel_sum.linear.x  += buff_target_vel[i].linear.x;
        target_vel_sum.angular.z += buff_target_vel[i].angular.z;
      }
      current_target_vel.linear.x  = target_vel_sum.linear.x  / (double)buff_target_vel.size();
      current_target_vel.angular.z = target_vel_sum.angular.z / (double)buff_target_vel.size();

      buff_target_vel_temp = buff_target_vel;
      buff_target_vel.clear();

      for (int i = 0; i < buff_target_vel_temp.size() - 1; i++) {
        buff_target_vel.push_back(buff_target_vel_temp[i+1]);
      }
    }
    return current_target_vel;
  }

  void VelocitySmoother::FaultStatePub()
  {
    // Nano 暂不检测
//    std::string ros_distro(std::getenv("ROS_DISTRO"));
//    if (!ros_distro.compare(std::string("melodic"))) {
//      ROS_INFO_ONCE("ROS is melodic, disable odom check.");
//      return;
//    }
  
    // 状态检测及上报
    Error_state->is_debug = is_debug;

    bool Twist_fault = 0;
    bool Pose_fault  = 0;
    Twist_fault = !Error_state->twist_return_ok(odom, plt_vel.twist);
    Pose_fault  = !Error_state->pose_return_ok(odom, plt_vel.twist);
    
    // Twist state Pub
    if (Twist_fault_flag && Twist_fault) {
      Twist_fault_flag = 0;
      Twist_true_flag = 1;
      system_msgs::DiagnosticStatus fault_diagnostic_msg;
      fault_diagnostic_msg.header.stamp = ros::Time::now();
      fault_diagnostic_msg.fault_id = 11028;
      fault_diagnostic_msg.fault_description = "Twist is Error";
      fault_diagnostic_msg.fault_level = fault_diagnostic_msg.ERROR;
      fault_state_pub.publish(fault_diagnostic_msg);
      ROS_WARN("Twist state error Pub !!! vel[%f] rot[%f]\n", Error_state->vel_dev, Error_state->rot_dev);  
    } else if (Twist_true_flag && (!Twist_fault)) {
      Twist_true_flag = 0;
      Twist_fault_flag = 1;
      system_msgs::DiagnosticStatus fault_diagnostic_msg;
      fault_diagnostic_msg.header.stamp = ros::Time::now();
      fault_diagnostic_msg.fault_id = 11028;
      fault_diagnostic_msg.fault_description = "Twist is OK";
      fault_diagnostic_msg.fault_level = fault_diagnostic_msg.OK;
      // ROS_INFO("[Twist_true_flag]  Twist_true_flag: %d , Twist_fault: %d ", Twist_true_flag, Twist_fault);
      fault_state_pub.publish(fault_diagnostic_msg);
    } else {
      /* code */
    }

    // Pose state Pub
    if (Pose_fault_flag && Pose_fault) {
      Pose_fault_flag = 0;
      Pose_true_flag = 1;
      system_msgs::DiagnosticStatus fault_diagnostic_msg;
      fault_diagnostic_msg.header.stamp = ros::Time::now();
      fault_diagnostic_msg.fault_id = 11029;
      fault_diagnostic_msg.fault_description = "Pose and twist is Error";
      fault_diagnostic_msg.fault_level = fault_diagnostic_msg.ERROR;
      fault_state_pub.publish(fault_diagnostic_msg);
      ROS_WARN("Pose state error Pub !!! \n");
    } else if (Pose_true_flag && (!Pose_fault)) {
      Pose_true_flag = 0;
      Pose_fault_flag = 1;
      system_msgs::DiagnosticStatus fault_diagnostic_msg;
      fault_diagnostic_msg.header.stamp = ros::Time::now();
      fault_diagnostic_msg.fault_id = 11029;
      fault_diagnostic_msg.fault_description = "Pose and twist is OK";
      fault_diagnostic_msg.fault_level = fault_diagnostic_msg.OK;
      fault_state_pub.publish(fault_diagnostic_msg);
    } else {
      /* code */
    }

    if (is_debug) {
      if (Twist_fault) {
        ROS_ERROR_COND(is_debug,"twist  return not ok !!!  [131 A] ");
        printf("------>[twist_return_ok]  twist_linear_x_ok: %d , twist_angular_z_ok: %d \n", 
        Error_state->twist_linear_x_ok, Error_state->twist_angular_z_ok);
        printf(" [debug] Twist_fault_flag: %d, Twist_fault: %d \n", Twist_fault_flag, Twist_fault);
      }
      if (Pose_fault) {
        ROS_ERROR_COND(is_debug,"pose  return not ok !!!  [132 A] ");
        printf("------>[pose_return_ok]  position_ok: %d , orientation_ok: %d \n", 
        Error_state->position_ok, Error_state->orientation_ok);
        printf(" [debug] Pose_fault_flag: %d, Pose_fault: %d \n", Pose_fault_flag, Pose_fault);
      }
    }
  }

  bool VelocitySmoother::isForceToStop()
  {
    if ( urgency_stop_Flag || fault_diagnostic_Flag || cmd_timeout_Flag || break_vel_flag_) {
      if (is_debug)  ROS_ERROR("  [isForceToStop] !!! urgency_Flag: %d, fault_Flag: %d, timeout_Flag: %d",
                               urgency_stop_Flag, fault_diagnostic_Flag, cmd_timeout_Flag);

      plt_vel.header.stamp = ros::Time::now();
      plt_vel.twist.linear.x = 0;
      plt_vel.twist.linear.y = 0;
      plt_vel.twist.linear.z = 0;
      plt_vel.twist.angular.x = 0;
      plt_vel.twist.angular.y = 0;
      plt_vel.twist.angular.z = 0;

      return true;
    }
    return false;
  }

  bool VelocitySmoother::isBack(bool lift_back_state, bool charge_back_state, bool go_back_state, bool rrt_state) 
  {
    if (lift_back_state || charge_back_state || go_back_state || rrt_state) {
      return true;
    } else {
      return false;
    }
  }

  bool VelocitySmoother::VelocitySmootherInit()
  {

    is_Smoother_Init = true;

    lift_back_state = false;
    charge_back_state = false;
    go_back_state = false;
    rrt_state_ = false;
    urgency_stop_Flag = false;
    fault_diagnostic_Flag = false;
    cmd_timeout_Flag = false;
    cmd_last_time = ros::Time::now();

    Twist_true_flag = 0;
    Twist_fault_flag = 1;
    Pose_true_flag  = 0;
    Pose_fault_flag  = 1;

  }

  void VelocitySmoother::limit_speedup(geometry_msgs::TwistStamped* cmd_vel_stamp, geometry_msgs::Twist current_vel)
  {
    // vel 速度平移控制
    // 向前
    if (cmd_vel_stamp->twist.linear.x > 0.005 &&               // 控制运动方向正确
        cmd_vel_stamp->twist.linear.x < speed_lim_v &&         // 控制速度没有越界
        current_vel.linear.x >= -0.02) {                       // 里程计平移返回正常范围 
      // front start　向前起步控制
      if (cmd_vel_stamp->twist.linear.x - 0.02 > current_vel.linear.x) {            // 阈值范围内，发送速度大于里程计速度，判断为向后起步加速
        current_vel.linear.x < 0.06 && cmd_vel_stamp->twist.linear.x > 0.001 ?
             ( cmd_vel_stamp->twist.linear.x = current_vel.linear.x + 0.012) :
       (current_vel.linear.x < 0.12 && cmd_vel_stamp->twist.linear.x > 0.001 ?
             ( cmd_vel_stamp->twist.linear.x = current_vel.linear.x + 0.017) :
       (current_vel.linear.x < 0.20 && cmd_vel_stamp->twist.linear.x > 0.001 ?
             ( cmd_vel_stamp->twist.linear.x = current_vel.linear.x + 0.021) :
                1));
        //  printf("s1--vel---> odom_vx: %f, cmd_vx: %f, time_count: %f\n",
        //  current_vel.linear.x,cmd_vel_stamp->twist.linear.x, time_count);
      }
      // front end　向前到点控制
      else if (cmd_vel_stamp->twist.linear.x + 0.02 < current_vel.linear.x &&       // 阈值范围内，发送速度小于里程计速度，判断为减速停下
               cmd_vel_stamp->twist.linear.x + 0.02 < last_cmd_vel.linear.x) {      // 进一步的，通过两次运动趋势，更加确定为减速停下状态
        current_vel.linear.x < 0.05 && cmd_vel_stamp->twist.linear.x > 0.001 ?
              ( cmd_vel_stamp->twist.linear.x = current_vel.linear.x - 0.005) :
        (current_vel.linear.x < 0.07 && cmd_vel_stamp->twist.linear.x > 0.001 ?
              ( cmd_vel_stamp->twist.linear.x = current_vel.linear.x - 0.010) :
        (current_vel.linear.x < 0.12 && cmd_vel_stamp->twist.linear.x > 0.001 ?
              ( cmd_vel_stamp->twist.linear.x = current_vel.linear.x - 0.015) :
                1));
        // printf("s2--vel---> odom_vx: %f, cmd_vx: %f, time_count: %f\n",
        // current_vel.linear.x,cmd_vel_stamp->twist.linear.x, time_count);
      }
    }
    // 向后
    else if (cmd_vel_stamp->twist.linear.x < -0.005 &&          // 控制运动方向正确
             cmd_vel_stamp->twist.linear.x > -0.3   &&          // 控制速度没有越界
             current_vel.linear.x < 0.02) {                     // 里程计平移返回正常范围    
      // back start  向后起步控制
      if (cmd_vel_stamp->twist.linear.x + 0.02 < current_vel.linear.x) {          // 阈值范围内，发送速度小于里程计速度，判断为向后起步加速
        current_vel.linear.x >- 0.06 && cmd_vel_stamp->twist.linear.x < -0.001 ?
               ( cmd_vel_stamp->twist.linear.x = current_vel.linear.x - 0.012) :
       (current_vel.linear.x >- 0.12 && cmd_vel_stamp->twist.linear.x < -0.001 ?
               ( cmd_vel_stamp->twist.linear.x = current_vel.linear.x - 0.017) :
       (current_vel.linear.x >- 0.20 && cmd_vel_stamp->twist.linear.x < -0.001 ?
               ( cmd_vel_stamp->twist.linear.x = current_vel.linear.x - 0.021) :
                 1));
        //  printf("s1--vel---> odom_vx: %f, cmd_vx: %f, time_count: %f\n",
        //  current_vel.linear.x,cmd_vel_stamp->twist.linear.x, time_count);
      }
      // back end  向后到点控制
      else if (cmd_vel_stamp->twist.linear.x - 0.02 > current_vel.linear.x &&      // 阈值范围内，发送速度大于里程计速度，判断为向后减速停下
               cmd_vel_stamp->twist.linear.x - 0.02 > last_cmd_vel.linear.x) {     // 进一步的，通过两次运动趋势，更加确定为向后减速停下状态
        current_vel.linear.x > -0.05 && cmd_vel_stamp->twist.linear.x < -0.001 ?
               ( cmd_vel_stamp->twist.linear.x = current_vel.linear.x + 0.005) :
       (current_vel.linear.x > -0.07 && cmd_vel_stamp->twist.linear.x < -0.001 ?
               ( cmd_vel_stamp->twist.linear.x = current_vel.linear.x + 0.010) :
       (current_vel.linear.x > -0.12 && cmd_vel_stamp->twist.linear.x < -0.001 ?
               ( cmd_vel_stamp->twist.linear.x = current_vel.linear.x + 0.015) :
                 1));
        // printf("s2--vel---> odom_vx: %f, cmd_vx: %f, time_count: %f\n",
        // current_vel.linear.x,cmd_vel_stamp->twist.linear.x, time_count);
      }
    }

    // rot 旋转控制
    // end lift 左转停下控制
    if (cmd_vel_stamp->twist.angular.z > 0.005 &&                              // 控制方向正确
        //  cmd_vel_stamp->twist.angular.z < last_cmd_vel.angular.z &&      
        current_vel.linear.x < 0.005 && current_vel.linear.x > -0.005 &&       // 确保平移状态为零
        current_vel.angular.z > -0.005 &&                                      // 里程计旋转返回范围正常
        cmd_vel_stamp->twist.angular.z + 0.05 < current_vel.angular.z) {       // 阈值范围内，发送速度小于里程计速度，判断为左转减速停下
       current_vel.angular.z < 0.10 ? ( cmd_vel_stamp->twist.angular.z = current_vel.angular.z - 0.006) :
      (current_vel.angular.z < 0.20 ? ( cmd_vel_stamp->twist.angular.z = current_vel.angular.z - 0.012) :
      (current_vel.angular.z < 0.40 ? ( cmd_vel_stamp->twist.angular.z = current_vel.angular.z - 0.024) :
      (current_vel.angular.z < 0.60 ? ( cmd_vel_stamp->twist.angular.z = current_vel.angular.z - 0.040) : 1)));

        cmd_vel_stamp->twist.angular.z < 0.40 && cmd_vel_stamp->twist.angular.z > 0.20 ?
        cmd_vel_stamp->twist.angular.z = 0.20 : 1;
        cmd_vel_stamp->twist.angular.z < 0.20 && cmd_vel_stamp->twist.angular.z > 0.01 ?
        cmd_vel_stamp->twist.angular.z = 0.10 : 1;

        // printf("s1_L--rot---> odom_vw: %f, cmd_vw: %f, time_count: %f\n",
        // current_vel.angular.z,cmd_vel_stamp->twist.angular.z, time_count);
    }
    // end right　右转停下控制
    else if (cmd_vel_stamp->twist.angular.z < -0.005 &&                         // 控制方向正确
             //  cmd_vel_stamp->twist.angular.z > last_cmd_vel.angular.z &&
             current_vel.linear.x < 0.005 && current_vel.linear.x > -0.005 &&   // 确保平移状态为零
             current_vel.angular.z < 0.005 &&                                   // 里程计旋转返回范围正常
             cmd_vel_stamp->twist.angular.z - 0.05 > current_vel.angular.z) {   // 阈值范围内，发送速度大于里程计速度，判断为右转减速停下
       current_vel.angular.z > -0.10 ? ( cmd_vel_stamp->twist.angular.z = current_vel.angular.z + 0.006) :
      (current_vel.angular.z > -0.20 ? ( cmd_vel_stamp->twist.angular.z = current_vel.angular.z + 0.012) :
      (current_vel.angular.z > -0.40 ? ( cmd_vel_stamp->twist.angular.z = current_vel.angular.z + 0.024) :
      (current_vel.angular.z > -0.60 ? ( cmd_vel_stamp->twist.angular.z = current_vel.angular.z + 0.040) : 1)));

        cmd_vel_stamp->twist.angular.z > -0.40 && cmd_vel_stamp->twist.angular.z < -0.20 ?
        cmd_vel_stamp->twist.angular.z = -0.20 : 1;
        cmd_vel_stamp->twist.angular.z > -0.20 && cmd_vel_stamp->twist.angular.z < -0.01 ?
        cmd_vel_stamp->twist.angular.z = -0.01 : 1;

        // printf("s2_R--rot---> odom_vw: %f, cmd_vw: %f, time_count: %f\n",
        // current_vel.angular.z,cmd_vel_stamp->twist.angular.z, time_count);
    }

    // // start lift
    // else if (
    //     cmd_vel_stamp->twist.angular.z > 0.005 &&
    // // cmd_vel_stamp->twist.angular.z < last_cmd_vel.angular.z &&
    //     current_vel.linear.x < 0.005 && current_vel.linear.x > -0.005 &&
    //     cmd_vel_stamp->twist.angular.z - 0.05 > current_vel.angular.z
    //     )
    // {
    //   current_vel.angular.z < 0.10 ? ( cmd_vel_stamp->twist.angular.z = current_vel.angular.z + 0.06) :
    //   (current_vel.angular.z < 0.20 ? ( cmd_vel_stamp->twist.angular.z = current_vel.angular.z + 0.12) :
    //   (current_vel.angular.z < 0.40 ? ( cmd_vel_stamp->twist.angular.z = current_vel.angular.z + 0.24) :
    //   (current_vel.angular.z < 0.60 ? ( cmd_vel_stamp->twist.angular.z = current_vel.angular.z + 0.40) :
    //                                   1)));

    //     // cmd_vel_stamp->twist.angular.z < 0.40 && cmd_vel_stamp->twist.angular.z > 0.20 ?
    //     // cmd_vel_stamp->twist.angular.z = 0.20 : 1;
    //     // cmd_vel_stamp->twist.angular.z < 0.20 && cmd_vel_stamp->twist.angular.z > 0.01 ?
    //     // cmd_vel_stamp->twist.angular.z = 0.10 : 1;


    //   printf("s1_L--rot---> odom_vw: %f, cmd_vw: %f, time_count: %f\n",
    //   current_vel.angular.z,cmd_vel_stamp->twist.angular.z, time_count);
    // }
    // // start right
    // else if (
    //         cmd_vel_stamp->twist.angular.z < -0.005 &&
    //       // cmd_vel_stamp->twist.angular.z > last_cmd_vel.angular.z &&
    //         current_vel.linear.x < 0.005 && current_vel.linear.x > -0.005 &&
    //         cmd_vel_stamp->twist.angular.z + 0.05 < current_vel.angular.z
    //         )
    // {
    //   current_vel.angular.z > -0.10 ? ( cmd_vel_stamp->twist.angular.z = current_vel.angular.z - 0.06) :
    //   (current_vel.angular.z > -0.20 ? ( cmd_vel_stamp->twist.angular.z = current_vel.angular.z - 0.12) :
    //   (current_vel.angular.z > -0.40 ? ( cmd_vel_stamp->twist.angular.z = current_vel.angular.z - 0.24) :
    //   (current_vel.angular.z > -0.60 ? ( cmd_vel_stamp->twist.angular.z = current_vel.angular.z - 0.40) :
    //                                     1)));

    // // cmd_vel_stamp->twist.angular.z > -0.40 && cmd_vel_stamp->twist.angular.z < -0.20 ?
    // // cmd_vel_stamp->twist.angular.z = -0.20 : 1;
    // // cmd_vel_stamp->twist.angular.z > -0.20 && cmd_vel_stamp->twist.angular.z < -0.01 ?
    // // cmd_vel_stamp->twist.angular.z = -0.01 : 1;

    //   printf("s2_R--rot---> odom_vw: %f, cmd_vw: %f, time_count: %f\n",
    //   current_vel.angular.z,cmd_vel_stamp->twist.angular.z, time_count);
    // }
  }

}
////////////////////////////////////////////////////////////////////////////////////////////////////  
// Main
int main(int argc, char** argv) {
  ros::init(argc, argv, "velocity_smoother");
  velocity_smoother::VelocitySmoother vs;
  //vs.init();


  vs.spin();

  return 0;
}
//PLUGINLIB_EXPORT_CLASS(yocs_velocity_smoother::VelocitySmootherNodelet, nodelet::Nodelet);
