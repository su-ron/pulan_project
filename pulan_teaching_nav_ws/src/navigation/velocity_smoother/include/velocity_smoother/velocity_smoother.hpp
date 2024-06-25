/**
 * @file /include/yocs_velocity_smoother/velocity_smoother_nodelet.hpp
 *
 * @brief Velocity smoother interface
 *
 * License: BSD
 *   https://raw.github.com/yujinrobot/yujin_ocs/hydro/yocs_velocity_smoother/LICENSE
 **/
/*****************************************************************************
 * Ifdefs
 *****************************************************************************/

#ifndef YUJIN_OCS_VELOCITY_SMOOTHER_HPP_
#define YUJIN_OCS_VELOCITY_SMOOTHER_HPP_

/*****************************************************************************
 * Includes
 *****************************************************************************/

// ros
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Bool.h>

// state monitor
#include "velocity_smoother/state_control.h"

// dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include "velocity_smoother/VelocitySmoothConfig.h"

// custom_msgs_srvs
#include "system_msgs/DiagnosticStatus.h"
#include "custom_msgs_srvs/SetBool.h"
#include <cstdlib>
#include <string>

/*****************************************************************************
* Namespaces
*****************************************************************************/

namespace velocity_smoother {

/*****************************************************************************
* VelocitySmoother
*****************************************************************************/
class KalmanFilter
{
public:
  double A = 1.0;
  double H = 1.0;
  double x_last;
  double p_last;
  /*>卡尔曼滤波函数。---> Gareth <--- */
  double kalman_filter(double q, double r, double ActualValue);
private:
  /*> 卡尔曼滤波平滑  -- > Gareth <--- */
  double x;
  double x_raw;
  double p;
  double p_raw;
  double R;
  double Q;
  double K;
};

class VelocitySmoother
{
public:
  //VelocitySmoother(const std::string &name)
  // : name(name), shutdown_req(false), input_active(false), pr_next(0), dynamic_reconfigure_server(nullptr) { };
  VelocitySmoother();

  ~VelocitySmoother()
  {
    //if (dynamic_reconfigure_server != NULL)
    //  delete dynamic_reconfigure_server;
  }

  //bool init(ros::NodeHandle& nh);
  void spin();
  //void shutdown() { shutdown_req = true; };

private:
  enum RobotFeedbackType
  {
    NONE,
    ODOMETRY,
    COMMANDS
  } robot_feedback;  /**< What source to use as robot velocity feedback */

  std::string name;
  double speed_lim_v, accel_lim_v, decel_lim_v;
  double speed_lim_w, accel_lim_w, decel_lim_w;
  double decel_factor;
  double frequency;

  /**> kalman params ---> Gareth <--- */
  bool use_calmanFilter;
  double Q_noise, R_noise;
  KalmanFilter vel_v, vel_w;

  /**> moveing_filter params ---> Gareth <---*/
  bool use_moving_filter;                             /**<是否用滑动滤波       --> Gareth <-- */
  int  sample_n_filter;                               /**< 滑动滤波样本数: n.  --> Gareth <-- */
  std::vector<geometry_msgs::Twist> buff_target_vel;  /**< 缓存多个速度.       --> Gareth <-- */

  /**> 后退模式监管 */
  bool use_limit_back;                                /**<是否用限制后退       --> Gareth <-- */
  bool lift_back_state;                               /**<电梯状态监控      　 --> Gareth <-- */
  bool charge_back_state;                             /**<充电状态监控　       --> Gareth <-- */            
  bool go_back_state;                                 /**<后退状态监控　       --> Gareth <-- */            
  bool rrt_state_;                                     /**<rrt状态监控　       --> Zack   <-- */     
  bool use_fault_checked_;                             /**<故障诊断监控                   <-- */

  /**> 后退模式监管及处理 */
  ros::Subscriber lift_state_sub;
  ros::Subscriber charge_state_sub;
  ros::Subscriber go_back_state_sub;
  ros::Subscriber rrt_state_sub;
  /**
   * @brief 电梯消息订阅
   * @param msg ->   
   * 消息内容： 
   * 0  - 正常模式;  
   * 1  - 进入电梯模式;  
   * 2  - 出电梯模式;   
   * 3  - 后退模式
   */
  void liftStateCB(const std_msgs::Int32& msg);
  /**
   * @brief 电梯消息订阅
   * @param msg ->   
   * 消息内容： 
   * 0 - 非充电情况
   * 1 - 前往充电Goal
   * 2 - 扫桩
   * 3 - 失败重试
   * 4 - 成功校准对位
   * 5 - 后退上桩
   * 6 - 前进下桩
   */
  void chargeStateCB(const std_msgs::Int8& msg);
  /**
   * @brief 后退模式函数
   * @param msg  ->  true 允许后退， false不允许后退
   */
  void goBackStateCB(const std_msgs::Bool& msg);
  /**
   * @brief rrt状态函数
   * @param msg  ->  true 允许后退， false不允许后退
   */
  void rrtStateCB(const std_msgs::Bool& msg);
  /**
   * @brief 后退模式处理函数
   * @param lift_back_state   ->  电梯状态监控 
   * @param charge_back_state ->  充电状态监控 
   */
  bool isBack(bool lift_back_state, bool charge_back_state, bool go_back_state, bool rrt_state);

  /**> 错误状态监测 */  
  bool Twist_true_flag;          /*!> 速度状态正常，发布 */
  bool Twist_fault_flag;         /*!> 速度状态异常，发布 */
  bool Pose_true_flag;           /*!> 里程状态正常，发布 */
  bool Pose_fault_flag;          /*!> 里程状态异常，发布 */
  double odom_delay_s;           /*!> odom数据延迟时间：ms */ 
  double dev_num_max;            /*!> 偏差次数上限 */ 
  double observe_time_s;         /*!> 里程监测时间 */
  double vel_dev;                /*!> 前进 Vx 速度偏差量 */
  double rot_dev;                /*!> 旋转 Vw 速度偏差量 */
  double sum_x_dev;              /*!> x方向里程偏差 */
  double sum_y_dev;              /*!> y方向里程偏差 */
  double sum_w_dev;              /*!> w方向里程偏差 */
  State_controlPtr Error_state;  /*!> 构建，错误状态检测 */

  /**> 错误状态上报及恢复 */
  nav_msgs::Odometry                          odom;
  geometry_msgs::TwistStamped              plt_vel;
  ros::Publisher  fault_state_pub;                      /*!> 发布错误信息 */
  ros::Subscriber fault_state_sub;                      /*!> 订阅信息反馈 */
  void FaultStatePub();

  /**> 控制超时急停 */
  bool cmd_timeout_Flag;                            /*!> 控制超时，标志 */
  bool break_vel_flag_;                              /*!> 直接刹车，标志 */
  ros::Time cmd_last_time;              

  /**> 异常服务急停 */
  bool is_Smoother_Init;                            /*!> 全局变量初始化，标志位 */
  bool urgency_stop_Flag;                           /*!> 紧急事件，停车 */
  bool fault_diagnostic_Flag;                       /*!> 特殊事件，停车 */
  ros::ServiceServer urgency_stop_srv;
  ros::ServiceServer fault_diagnostic_srv;
  bool UrgencyStopSrv(custom_msgs_srvs::SetBool::Request &req, custom_msgs_srvs::SetBool::Response &res);
  bool FaultDiagnosticSrv(custom_msgs_srvs::SetBool::Request &req, custom_msgs_srvs::SetBool::Response &res);
  bool isForceToStop();                             /*!> 强制停车处理 */
  bool VelocitySmootherInit();                      /*!> 全局变量初始化函数　*/

  /**> 轮毂电机起步停车消抖 */
  bool use_limit_speedup; /**< limit the start speed */
  /**
   * @param cmd_vel_stamp -> plt 下发速度
   * @param current_vel   -> odom 返回速度
   */
  void limit_speedup(geometry_msgs::TwistStamped* cmd_vel_stamp, geometry_msgs::Twist current_vel);

  geometry_msgs::Twist last_cmd_vel;
  geometry_msgs::Twist  current_vel;
  geometry_msgs::Twist   target_vel;
  

  bool                     is_debug;
  bool                 shutdown_req; /**< Shutdown requested by nodelet; kill worker thread */
  bool                 input_active;

  double                cb_avg_time;
  ros::Time            last_cb_time;
  std::vector<double> period_record; /**< Historic of latest periods between velocity commands */
  unsigned int              pr_next; /**< Next position to fill in the periods record buffer */

  ros::Subscriber odometry_sub;    /**< Current velocity from odometry */
  ros::Subscriber current_vel_sub; /**< Current velocity from commands sent to the robot, not necessarily by this node */
  ros::Subscriber raw_in_vel_sub;  /**< Incoming raw velocity commands */
  ros::Publisher  smooth_vel_pub;  /**< Outgoing smoothed velocity commands */
  

  void velocityCB(const geometry_msgs::Twist::ConstPtr& msg);
  void robotVelCB(const geometry_msgs::Twist::ConstPtr& msg);
  void odometryCB(const nav_msgs::Odometry::ConstPtr&   msg);

  double sign(double x)  { return x < 0.0 ? -1.0 : +1.0; };

  double median(std::vector<double> values) {
    // Return the median element of an doubles vector
    nth_element(values.begin(), values.begin() + values.size()/2, values.end());
    return values[values.size()/2];
  };
    
  /**<对数据源进行滑动滤波处理。二级平滑，可能会影响时不时性，但不影响真实性。 --> Gareth <-- */
  geometry_msgs::Twist moveing_filter_smoothing(geometry_msgs::Twist current_target_vel, unsigned int sample_n_filter);


  /*********************
  ** Dynamic Reconfigure
  **********************/
  dynamic_reconfigure::Server<VelocitySmoothConfig>                dynamic_reconfigure_server  ;
  dynamic_reconfigure::Server<VelocitySmoothConfig>::CallbackType  dynamic_reconfigure_callback;
  void reconfigCB(velocity_smoother::VelocitySmoothConfig &config);
};




} // yocs_namespace velocity_smoother

#endif /* YUJIN_OCS_VELOCITY_SMOOTHER_HPP_ */
