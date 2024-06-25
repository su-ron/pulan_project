#include "velocity_smoother/state_control.h"

namespace velocity_smoother {
  state_control::state_control(double odom_delay_s_, double dev_num_max_, double observe_time_s_, double dt_, 
                               double vel_dev_, double rot_dev_, double sum_x_dev_, double sum_y_dev_, double sum_w_dev_, 
                               bool is_debug_) :
                               vel_dev(0.01), rot_dev(0.01), sum_x_dev(1.0), sum_y_dev(1.0), sum_w_dev(1.5),
                               vel_dev_num(0), rot_dev_num(0), x_dev_num(0), y_dev_num(0), w_dev_num(0)
  {
    dt = dt_;
    is_debug = is_debug_;
    odom_delay_s = odom_delay_s_;
    dev_num_max  = dev_num_max_;
    observe_time_s = observe_time_s_;
    vel_dev = vel_dev_;
    rot_dev = rot_dev_;
    sum_x_dev = sum_x_dev_; 
    sum_y_dev = sum_y_dev_; 
    sum_w_dev = sum_w_dev_; 
  }
  state_control::~state_control() {
  }

  int state_control::period_return_ok(const nav_msgs::Odometry odom, const geometry_msgs::Twist plt)
  {
    static ros::Time period_last_time = ros::Time::now();
    double odom_linear_x  = odom.twist.twist.linear.x;
    double odom_angular_z = odom.twist.twist.angular.z;

    plt_period_x.push_back(plt.linear.x);
    plt_period_w.push_back(plt.angular.z);

    if ((plt_period_x.at(0) * odom_linear_x > 0 && fabs(plt_period_x.at(0) - odom_linear_x) < vel_dev)) {
      if( (ros::Time::now() - period_last_time).toSec() > odom_delay_s*0.5 && 
          (ros::Time::now() - period_last_time).toSec() < odom_delay_s*1.5  ) {
        
      }
      plt_period_x.erase(plt_period_x.begin());
      plt_period_w.erase(plt_period_w.begin());
    }

  }

  bool state_control::twist_return_ok(const nav_msgs::Odometry odom, const geometry_msgs::Twist plt)
  {
    static bool update_vel = 0;

    if (fabs(plt.linear.x) + fabs(plt.angular.z) <= 0.0001) {
      update_vel = 0;
      return true;
    }

    // printf("[debug 1 ]: odom_x:%lf, odom_w:%lf, plt_x:%lf, plt_w:%lf \n", odom.twist.twist.linear.x,odom.twist.twist.angular.z
    //                                                                , plt.linear.x, plt.angular.z);
    static ros::Time vel_last_time = ros::Time::now();
    static double plt_linear_x;
    static double plt_angular_z;
    double odom_linear_x  = odom.twist.twist.linear.x;
    double odom_angular_z = odom.twist.twist.angular.z;
    double temp_vx_error = 0;
    double temp_vw_error = 0;

    if (!update_vel) {
      update_vel = 1;

      plt_linear_x = plt.linear.x;
      plt_angular_z = plt.angular.z;
    }

    if ((ros::Time::now() - vel_last_time).toSec() >= odom_delay_s) {   // 5.0
      update_vel = 0;

      temp_vx_error = fabs(plt_linear_x - odom_linear_x);

      if ( temp_vx_error > vel_dev) {
        ((++ vel_dev_num) > dev_num_max ) ? (vel_dev_num = dev_num_max) : 1;
      } else {
        ((-- vel_dev_num) < 0) ? (vel_dev_num = 0) : 1;
      }
    
      temp_vw_error = fabs(plt_angular_z - odom_angular_z);
      if ( temp_vw_error > rot_dev ) {
        ((++ rot_dev_num) > dev_num_max ) ? (rot_dev_num = dev_num_max ) : 1;
      } else {
        ((-- rot_dev_num) < 0) ? (rot_dev_num = 0) : 1;
      }
      vel_last_time = ros::Time::now();
      
      // ROS_INFO("vel_dev_num: %d, rot_dev_num: %d \n", vel_dev_num, rot_dev_num);

      // if (is_debug)
      // {
      //   printf ("------>[debug] vel_dev_num:   %f, rot_dev_num:    %f  \n",              vel_dev_num,   rot_dev_num);
      //   printf ("------>[debug] plt_linear_x:  %f, odom_linear_x:  %f, vx_error: %f \n", plt_linear_x,  odom_linear_x,  temp_vx_error);
      //   printf ("------>[debug] plt_angular_z: %f, odom_angular_z: %f, vw_error: %f \n", plt_angular_z, odom_angular_z, temp_vw_error);
      // }
    }

    (vel_dev_num >= dev_num_max) ? (twist_linear_x_ok  = 0) : (twist_linear_x_ok  = 1);
    (rot_dev_num >= dev_num_max) ? (twist_angular_z_ok = 0) : (twist_angular_z_ok = 1);
    
    if (!twist_linear_x_ok || !twist_angular_z_ok) {
      if (is_debug) {
        printf ("------>[debug][twi] vel_dev_num:   %f, rot_dev_num:    %f  \n",              vel_dev_num,   rot_dev_num);
        printf ("------>[debug][twi] plt_linear_x:  %f, odom_linear_x:  %f, vx_error: %f \n", plt_linear_x,  odom_linear_x,  temp_vx_error);
        printf ("------>[debug][twi] plt_angular_z: %f, odom_angular_z: %f, vw_error: %f \n", plt_angular_z, odom_angular_z, temp_vw_error);
      }

      // 保证采样间隔
      vel_dev_num = 0;
      rot_dev_num = 0;
      return false;
    }
    return true;
  }

  bool state_control::pose_return_ok(const nav_msgs::Odometry odom, const geometry_msgs::Twist plt)
  {
    if (fabs(plt.linear.x) + fabs(plt.angular.z) <= 0.0001) {
      plt_period_x.clear(), plt_period_w.clear();                   /*!> plt 单位速度容器 */
      plt_dt_x.clear(), plt_dt_y.clear(), plt_dt_w.clear();         /*!> plt 单位里程容器 */
      plt_sum_x.clear(), plt_sum_y.clear(), plt_sum_w.clear();      /*!> plt 累计里程容器 */
      odom_dt_x.clear(), odom_dt_y.clear(), odom_dt_w.clear();      /*!> odom单位里程容器 */

      return true;
    }
        

    // printf("[debug 2 ]: odom_x:%lf, odom_w:%lf, plt_x:%lf, plt_w:%lf \n", odom.twist.twist.linear.x,odom.twist.twist.angular.z
    //                                                                     , plt.linear.x, plt.angular.z);
    static ros::Time pose_last_time = ros::Time::now();
    double theta = plt.angular.z * dt;
    double odom_theta = odom.twist.twist.angular.z * dt;
    double pose_x = plt.linear.x * cos(theta) * dt;
    double pose_y = plt.linear.x * sin(theta) * dt;
    double odom_pose_x = odom.twist.twist.linear.x * cos(odom_theta) * dt;
    double odom_pose_y = odom.twist.twist.linear.x * sin(odom_theta) * dt;
    double odom_sum_x, odom_sum_y, odom_sum_w;

    plt_dt_x.push_back(pose_x);
    plt_dt_y.push_back(pose_y);
    plt_dt_w.push_back(theta );
    odom_dt_x.push_back(odom_pose_x);
    odom_dt_y.push_back(odom_pose_y);
    odom_dt_w.push_back(odom_theta );

    if (plt_dt_x.size() > observe_time_s / dt) {
      for (int i=0; i < plt_dt_x.size(); i++) {
        pose_x += plt_dt_x.at(i);
        pose_y += plt_dt_y.at(i);
        theta  += plt_dt_w.at(i);
      }

      plt_sum_x.push_back(pose_x);
      plt_sum_y.push_back(pose_y);
      plt_sum_w.push_back(theta );

      plt_dt_x.erase(plt_dt_x.begin());
      plt_dt_y.erase(plt_dt_y.begin());
      plt_dt_w.erase(plt_dt_w.begin());

    } else {
      if(plt_sum_x.size() <= 0 || plt_sum_y.size() <= 0 || plt_sum_w.size() <= 0)
        return true;
    }
    

    if (odom_dt_x.size() > observe_time_s / dt) {
      for (int i=0; i < odom_dt_x.size(); i++) {
        odom_sum_x += odom_dt_x.at(i);
        odom_sum_y += odom_dt_y.at(i);
        odom_sum_w += odom_dt_w.at(i);
      }

      odom_dt_x.erase(odom_dt_x.begin());
      odom_dt_y.erase(odom_dt_y.begin());
      odom_dt_w.erase(odom_dt_w.begin());

    }

    if ((ros::Time::now() - pose_last_time).toSec() >= odom_delay_s) {  // 5.0

      if (fabs(plt_sum_x.at(0) - odom_sum_x) > (sum_x_dev * fabs(plt_sum_x.at(0)) + sum_x_dev)) {
        ((++ x_dev_num) > dev_num_max ) ? (x_dev_num = dev_num_max ) : 1;
      } else {
        ((-- x_dev_num) < 0) ? (x_dev_num = 0) : 1;
      }

      if (fabs(plt_sum_y.at(0) - odom_sum_y) > (sum_y_dev * fabs(plt_sum_y.at(0)) + sum_y_dev)) {
        ((++ y_dev_num) > dev_num_max ) ? (y_dev_num = dev_num_max ) : 1;
      } else {
        ((-- y_dev_num) < 0) ? (y_dev_num = 0) : 1;
      }

      if (fabs(plt_sum_w.at(0) - odom_sum_w) > (sum_w_dev * fabs(plt_sum_w.at(0)) + sum_w_dev)) {
        ((++ w_dev_num) > dev_num_max ) ? (w_dev_num = dev_num_max ) : 1;
      } else {
        ((-- w_dev_num) < 0) ? (w_dev_num = 0) : 1;
      }

      // if (is_debug)
      // if(plt_sum_x.size() > 0 && plt_sum_y.size() > 0 && plt_sum_w.size() > 0)
      // {
      //   printf ("------>[debug][pos] plt_sum_x: %f, plt_sum_y: %f, plt_sum_w: %f, \n", plt_sum_x.at(0), plt_sum_y.at(0), plt_sum_w.at(0));
      // }
      // else
      // {
      //   printf ("------>[debug][pos] plt_sum_x: %f, plt_sum_y: %f, plt_sum_w: %f, \n", -1.0, -1.0, -1.0);
      // }

      plt_sum_x.erase(plt_sum_x.begin());
      plt_sum_y.erase(plt_sum_y.begin());
      plt_sum_w.erase(plt_sum_w.begin());
    }

    ((x_dev_num >= dev_num_max) || (y_dev_num >= dev_num_max)) ? (position_ok = 0) : (position_ok = 1);
    (w_dev_num >= dev_num_max) ? (orientation_ok = 0) : (orientation_ok = 1);

    if (!position_ok || !position_ok) {
      
      if (is_debug) {
        printf ("------>[debug][pos] x_dev_num: %f, y_dev_num: %f, w_dev_num: %f, \n", x_dev_num, y_dev_num, w_dev_num);
        printf ("------>[debug][pos] odom_sum_x: %f, odom_sum_y: %f, odom_sum_w: %f, \n", odom_sum_x, odom_sum_y, odom_sum_w);
        
      }

      // 保证采样间隔
      x_dev_num = 0;
      y_dev_num = 0;
      w_dev_num = 0;
      return false;
    }

    return true;
  }
}
