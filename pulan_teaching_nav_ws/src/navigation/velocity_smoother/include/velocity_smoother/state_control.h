/******************************************************************************
 * 简介：速度异常监测及控制
 * 内容：
 *      1.odom 数据异常监测：速度大小的合理性、里程计变化趋势合理性
 *      2.强制锁住
 * 
 * 作者：Gareth
 *****************************************************************************/

#ifndef ERROR_STATE_CONTROL_H
#define ERROR_STATE_CONTROL_H

/*****************************************************************************
 * Includes
 *****************************************************************************/
#include <ros/ros.h>
#include <tf/tf.h>
#include <vector>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistStamped.h>
// boost classes
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>

/*****************************************************************************
* Namespaces
*****************************************************************************/
namespace velocity_smoother {

class state_control
{
public:
    state_control(double odom_delay_s_, double dev_num_max_, double observe_time_s_, double dt_, 
                  double vel_dev_, double rot_dev_, double sum_x_dev_, double sum_y_dev_, double sum_w_dev_, 
                  bool is_debug_ = false);
    ~state_control();

    int  period_return_ok (const nav_msgs::Odometry odom, const geometry_msgs::Twist plt);
    bool twist_return_ok  (const nav_msgs::Odometry odom, const geometry_msgs::Twist plt);
    bool pose_return_ok   (const nav_msgs::Odometry odom, const geometry_msgs::Twist plt);

    bool control_period_state;       
    bool twist_linear_x_ok;
    bool twist_angular_z_ok;
    bool position_ok;
    bool orientation_ok;
    bool is_debug;

    double odom_delay_s;         /*!> odom数据延迟时间：ms */ 
    double dev_num_max;          /*!> 偏差次数上限 */ 
    double observe_time_s;       /*!> 里程监测时间 */
    double dt;                   /*!> 下发控制周期 */
    double vel_dev;              /*!> 前进 Vx 速度偏差量 */
    double rot_dev;              /*!> 旋转 Vw 速度偏差量 */
    double vel_dev_num;          /*!> 前进 Vx 错误累计次数统计 */
    double rot_dev_num;          /*!> 旋转 Vw 错误累计次数统计 */
    double sum_x_dev;            /*!> x方向里程偏差 */
    double sum_y_dev;            /*!> y方向里程偏差 */
    double sum_w_dev;            /*!> w方向里程偏差 */
    double x_dev_num;            /*!> x方向里程误差计数 */
    double y_dev_num;            /*!> y方向里程误差计数 */
    double w_dev_num;            /*!> w方向里程误差计数 */

private:
    std::vector<double> plt_period_x, plt_period_w;           /*!> plt 单位速度容器 */
    std::vector<double> plt_dt_x, plt_dt_y, plt_dt_w;         /*!> plt 单位里程容器 */
    std::vector<double> plt_sum_x, plt_sum_y, plt_sum_w;      /*!> plt 累计里程容器 */
    std::vector<double> odom_dt_x, odom_dt_y, odom_dt_w;      /*!> odom单位里程容器 */

};

typedef boost::shared_ptr<state_control> State_controlPtr;

}

#endif /* YUJIN_OCS_VELOCITY_SMOOTHER_HPP_ */