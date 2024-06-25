#ifndef __WHEELTEC_ROBOT_H_
#define __WHEELTEC_ROBOT_H_

#include "custom_msgs_srvs/ChargingSwitch.h"
#include "custom_msgs_srvs/EmergencySwitch.h"
#include "custom_msgs_srvs/LidarSwitch.h"
#include "custom_msgs_srvs/LightSwitch.h"
#include "custom_msgs_srvs/UltrasoundSwitch.h"
#include "custom_msgs_srvs/robotChassisStatus.h"
#include "ros/ros.h"
#include <fcntl.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Vector3.h>
#include <iostream>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <serial/serial.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <string>
#include <sys/stat.h>
#include <sys/types.h>
#include <tf/transform_broadcaster.h>
#include <thread>
#include <unistd.h>
#include <geometry_msgs/TwistStamped.h>
using namespace std;
#define SEND_DATA_CHECK 1
#define READ_DATA_CHECK 0
#define FRAME_HEADER 0X7B
#define FRAME_TAIL 0X7D
#define RECEIVE_DATA_SIZE 52
#define SEND_DATA_SIZE 17
#define PI 3.1415926f

const double odom_pose_covariance[36] = {
    1e-3, 0, 0, 0,   0, 0, 0, 1e-3, 0, 0, 0,   0, 0, 0, 1e6, 0, 0, 0,
    0,    0, 0, 1e6, 0, 0, 0, 0,    0, 0, 1e6, 0, 0, 0, 0,   0, 0, 1e3 };
const double odom_pose_covariance2[36] = {
    1e-9, 0, 0, 0,   0, 0, 0, 1e-3, 1e-9, 0, 0,   0, 0, 0, 1e6, 0, 0, 0,
    0,    0, 0, 1e6, 0, 0, 0, 0,    0,    0, 1e6, 0, 0, 0, 0,   0, 0, 1e-9 };

const double odom_twist_covariance[36] = {
    1e-3, 0, 0, 0,   0, 0, 0, 1e-3, 0, 0, 0,   0, 0, 0, 1e6, 0, 0, 0,
    0,    0, 0, 1e6, 0, 0, 0, 0,    0, 0, 1e6, 0, 0, 0, 0,   0, 0, 1e3 };
const double odom_twist_covariance2[36] = {
    1e-9, 0, 0, 0,   0, 0, 0, 1e-3, 1e-9, 0, 0,   0, 0, 0, 1e6, 0, 0, 0,
    0,    0, 0, 1e6, 0, 0, 0, 0,    0,    0, 1e6, 0, 0, 0, 0,   0, 0, 1e-9 };

typedef struct __Vel_Pos_Data_ {
  float X;
  float Y;
  float Z;
} Vel_Pos_Data;

typedef struct BATTERY_INFO_ {
  float power;
  float voltage;
  float current;
  float temperature;
} BATTERY_INFO;

typedef struct MOTOR_INFO_ {
  float left_motor_current;
  float right_motor_current;
} MOTOR_INFO;

typedef struct POWER_CURRENT_ {
  float v_19;
  float v_12;
  float v_5;
} POWER_CURRENT;

typedef struct _SEND_DATA_ {
  uint8_t tx[SEND_DATA_SIZE];
  float X_speed;
  float Y_speed;
  float Z_speed;
  unsigned char Frame_Tail;

} SEND_DATA;

typedef struct _RECEIVE_DATA_ {
  uint8_t rx[RECEIVE_DATA_SIZE];
  uint8_t Flag_Stop;
  unsigned char Frame_Header;
  float X_speed;
  float Y_speed;
  float Z_speed;
  float Power_Voltage;
  unsigned char Frame_Tail;
} RECEIVE_DATA;

typedef struct SwitchCommand_ {
  uint8_t left_light_enabled;
  uint8_t right_light_enabled;
  uint8_t charge_control_enabled;
  uint8_t emergency_enabled;
  uint8_t lidar_enabled;
  uint8_t ultrasound_enabled;
} SwitchCommand;

SwitchCommand switch_command_;
enum class CHARGE_STATE { IDLE = 0, CHARGING, ERROR, FULL, UNKNOWN };

CHARGE_STATE convertChargeState(uint8_t value) {
  switch (value) {
  case 0:
    return CHARGE_STATE::IDLE;
  case 1:
    return CHARGE_STATE::CHARGING;
  case 2:
    return CHARGE_STATE::ERROR;
  case 3:
    return CHARGE_STATE::FULL;
  default:
    // 处理不正确的值，这里返回默认状态
    return CHARGE_STATE::UNKNOWN;
  }
}

enum class LIGHT_STATE { OFF = 0, ON, FLASH, UNKNOWN };

LIGHT_STATE convertLightState(uint8_t value) {
  switch (value) {
  case 0:
    return LIGHT_STATE::OFF;
  case 1:
    return LIGHT_STATE::ON;
  case 2:
    return LIGHT_STATE::FLASH;
  default:
    return LIGHT_STATE::UNKNOWN;
  }
}

class turn_on_robot {
public:
  turn_on_robot();
  ~turn_on_robot();
  void Control();
  bool Get_Sensor_Data();
  void Publish_Odom();
  void publishRobotChassisStatus();
  serial::Serial Stm32_Serial;
  bool status_flag_ = false;
  float Sampling_Time;
  ros::Time _Now, _Last_Time;

private:
  void Cmd_Vel_Callback(const geometry_msgs::TwistStamped& twist_aux);
  void light_state_callback(const custom_msgs_srvs::LightSwitch& light_state_msg);
  void charge_switch_callback(
    const custom_msgs_srvs::ChargingSwitch& charge_switch_msg);
  void ultrasound_switch_callback(
    const custom_msgs_srvs::UltrasoundSwitch& ultrasound_switch_msg);
  void
    lidar_switch_callback(const custom_msgs_srvs::LidarSwitch& lidar_switch_msg);
  void emergency_switch_callback(
    const custom_msgs_srvs::EmergencySwitch& emergency_switch_msg);


  float Odom_Trans(uint8_t Data_High, uint8_t Data_Low);
  unsigned char Check_Sum(unsigned char Count_Number, unsigned char mode);
  int serial_baud_rate;
  string usart_port_name, robot_frame_id, smoother_cmd_vel;
  ros::NodeHandle n;
  ros::NodeHandle private_nh;

  ros::Subscriber Cmd_Vel_Sub, light_switch_sub_, charge_switch_sub_,
    ultrasound_switch_sub_, lidar_switch_sub_, emergency_switch_sub_;
  ros::Publisher odom_publisher;
  tf::TransformBroadcaster odom_broadcaster;
  RECEIVE_DATA Receive_Data;
  SEND_DATA Send_Data;
  ros::Publisher robotChassisStatus_publisher;

  Vel_Pos_Data Robot_Pos;

  Vel_Pos_Data Robot_Vel;
  BATTERY_INFO battery_info_;
  MOTOR_INFO motor_info_;
  POWER_CURRENT power_current_;
  int error_code_;
  CHARGE_STATE charge_state_;
  bool emergency_switch_status_;
  float ultrasound_1_, ultrasound_2_;
  float left_odometer_, right_odometer_;
  LIGHT_STATE left_light_state_, right_light_state_;
  bool IR_alignment_status_;
  bool loose_shaft_switch_state_;
  string chassis_info_ = "pulan_robot";
  vector<uint8_t> rxDataBuf;
};

float u8_to_u16(const uint8_t Data_High, const uint8_t Data_Low) {
  float data_return;
  short transition_16 = 0;
  transition_16 |= Data_High << 8;
  transition_16 |= Data_Low & 0xff;

  data_return =
    (transition_16 / 1000) +
    (transition_16 % 1000) *
    0.001; 
  return data_return;
}

float u8_to_u32(const uint8_t Byte_1, const uint8_t Byte_2,
  const uint8_t Byte_3, const uint8_t Byte_4) {
  long temp_32 = 0;
  temp_32 |= Byte_1 << 24;
  temp_32 |= Byte_2 << 16;
  temp_32 |= Byte_3 << 8;
  temp_32 |= Byte_4 & 0xff;
  return static_cast<float>(temp_32) / 1000.0;
}

#endif
