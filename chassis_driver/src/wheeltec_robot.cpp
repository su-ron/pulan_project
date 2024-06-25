#include "wheeltec_robot.h"

turn_on_robot::turn_on_robot() : Sampling_Time(0), private_nh("~") {
  // TODO有没有更方便的赋0方式呢,或者数据结构的选择上
  memset(&Robot_Pos, 0, sizeof(Robot_Pos));
  memset(&Robot_Vel, 0, sizeof(Robot_Vel));
  memset(&Receive_Data, 0, sizeof(Receive_Data));
  memset(&Send_Data, 0, sizeof(Send_Data));

  private_nh.param<std::string>("usart_port_name", usart_port_name,
    "/dev/chassis_controller");
  private_nh.param<int>("serial_baud_rate", serial_baud_rate, 115200);
  private_nh.param<std::string>("smoother_cmd_vel", smoother_cmd_vel,
    "/smoother_cmd_vel");
  private_nh.param<std::string>("robot_frame_id", robot_frame_id, "base_link");

  odom_publisher = n.advertise<nav_msgs::Odometry>("odom", 10);
  robotChassisStatus_publisher =
    n.advertise<custom_msgs_srvs::robotChassisStatus>("robotChassisStatus",
      20);

  Cmd_Vel_Sub =
    n.subscribe(smoother_cmd_vel, 1, &turn_on_robot::Cmd_Vel_Callback, this);
  light_switch_sub_ = n.subscribe("/light_switch", 1,
    &turn_on_robot::light_state_callback, this);
  charge_switch_sub_ = n.subscribe(
    "/charge_switch", 1, &turn_on_robot::charge_switch_callback, this);
  ultrasound_switch_sub_ =
    n.subscribe("/ultrasound_switch", 1,
      &turn_on_robot::ultrasound_switch_callback, this);
  lidar_switch_sub_ = n.subscribe("/lidar_switch", 1,
    &turn_on_robot::lidar_switch_callback, this);
  emergency_switch_sub_ = n.subscribe(
    "/emergency_switch", 1, &turn_on_robot::emergency_switch_callback, this);


  try {
    Stm32_Serial.setPort(usart_port_name);
    Stm32_Serial.setBaudrate(serial_baud_rate);
    serial::Timeout _time = serial::Timeout::simpleTimeout(2000);
    Stm32_Serial.setTimeout(_time);
    Stm32_Serial.open();
  }
  catch (serial::IOException& e) {
    ROS_ERROR_STREAM(
      "pulan_robot can not open serial port,Please check the "
      "serial port cable! ");
  }
  if (Stm32_Serial.isOpen()) {
    ROS_INFO_STREAM("pulan_robot serial port opened");
    status_flag_ = true;
  }
}

void receiveThread(turn_on_robot* robot) {
  robot->_Last_Time = ros::Time::now();
  while (ros::ok()) {
    robot->_Now = ros::Time::now();
    robot->Sampling_Time = (robot->_Now - robot->_Last_Time).toSec();
    if (robot->Get_Sensor_Data()) {
      robot->Publish_Odom();
      robot->publishRobotChassisStatus();
    }
    robot->_Last_Time = robot->_Now;
  }
}
void sendThread(turn_on_robot* robot) {
  while (ros::ok()) {
    ros::spin();
  }
}


void turn_on_robot::light_state_callback(
  const custom_msgs_srvs::LightSwitch& light_state_msg) {
  switch_command_.left_light_enabled = light_state_msg.left_light_switch;
  switch_command_.right_light_enabled = light_state_msg.right_light_switch;
}
void turn_on_robot::charge_switch_callback(
  const custom_msgs_srvs::ChargingSwitch& charge_switch_msg) {
  switch_command_.charge_control_enabled =
    charge_switch_msg.charging_switch_cmd;
}
void turn_on_robot::ultrasound_switch_callback(
  const custom_msgs_srvs::UltrasoundSwitch& ultrasound_switch_msg) {
  switch_command_.ultrasound_enabled =
    ultrasound_switch_msg.ultrasound_switch_cmd;
}
void turn_on_robot::lidar_switch_callback(
  const custom_msgs_srvs::LidarSwitch& lidar_switch_msg) {
  switch_command_.lidar_enabled = lidar_switch_msg.lidar_switch_cmd;
}
void turn_on_robot::emergency_switch_callback(
  const custom_msgs_srvs::EmergencySwitch& emergency_switch_msg) {
  switch_command_.emergency_enabled = emergency_switch_msg.emergency_switch_cmd;
}
void turn_on_robot::Cmd_Vel_Callback(
  const geometry_msgs::TwistStamped& twist_aux) {
  int16_t transition = 0;
  Send_Data.tx[0] = FRAME_HEADER;
  Send_Data.tx[1] = 0x03;//TODO使能电机和车灯
if(twist_aux.twist.angular.z>0.3)	{Send_Data.tx[2]=0x20;}
else if(twist_aux.twist.angular.z<-0.3) {Send_Data.tx[2]=0x40;}
else {Send_Data.tx[2]=0xff;}

  transition = twist_aux.twist.linear.x * 1000;
  Send_Data.tx[4] = transition & 0xff;
  Send_Data.tx[3] = transition >> 8;

  transition = 0;
  transition = twist_aux.twist.linear.y * 1000;
  Send_Data.tx[6] = transition & 0xff;
  Send_Data.tx[5] = transition >> 8;

  transition = 0;
  transition = twist_aux.twist.angular.z * 1000;
  Send_Data.tx[8] = transition & 0xff;
  Send_Data.tx[7] = transition >> 8;

  Send_Data.tx[9] = 0;
  Send_Data.tx[10] = switch_command_.charge_control_enabled;
  Send_Data.tx[11] = switch_command_.ultrasound_enabled;
  Send_Data.tx[12] = 0;
  Send_Data.tx[13] = switch_command_.lidar_enabled;
  Send_Data.tx[14] = switch_command_.emergency_enabled;

  Send_Data.tx[SEND_DATA_SIZE - 2] =
    Check_Sum(SEND_DATA_SIZE - 2, SEND_DATA_CHECK);
  Send_Data.tx[SEND_DATA_SIZE - 1] = FRAME_TAIL;
  try {
    Stm32_Serial.write(Send_Data.tx, sizeof(Send_Data.tx));
  }
  catch (serial::IOException& e) {
    ROS_ERROR_STREAM("Unable to send data through serial port");
  }
}

void turn_on_robot::Publish_Odom() {
  geometry_msgs::Quaternion odom_quat =
    tf::createQuaternionMsgFromYaw(Robot_Pos.Z);
  nav_msgs::Odometry odom;

  Robot_Pos.X +=
    (Robot_Vel.X * cos(Robot_Pos.Z) - Robot_Vel.Y * sin(Robot_Pos.Z)) *
    Sampling_Time;
  Robot_Pos.Y +=
    (Robot_Vel.X * sin(Robot_Pos.Z) + Robot_Vel.Y * cos(Robot_Pos.Z)) *
    Sampling_Time;
  Robot_Pos.Z += Robot_Vel.Z * Sampling_Time;

  odom.header.stamp = ros::Time::now();
  odom.header.frame_id = "odom";
  odom.pose.pose.position.x = Robot_Pos.X;
  odom.pose.pose.position.y = Robot_Pos.Y; 
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = odom_quat;

  odom.child_frame_id = robot_frame_id;
  odom.twist.twist.linear.x = Robot_Vel.X;
  odom.twist.twist.linear.y = 0.0;
  odom.twist.twist.angular.z = Robot_Vel.Z;

  if (fabs(Robot_Vel.X) < 1e-6 &&
    fabs(Robot_Vel.Z) <
    1e-6)  //  如果velocity是零，说明编码器的误差会比较小，认为编码器数据更可靠
  {
    memcpy(&odom.pose.covariance, odom_pose_covariance2,
      sizeof(odom_pose_covariance2)),
      memcpy(&odom.twist.covariance, odom_twist_covariance2,
        sizeof(odom_twist_covariance2));
  }
  else  //  如果小车velocity非零，考虑到运动中编码器可能带来的滑动误差，认为imu的数据更可靠
  {
    memcpy(&odom.pose.covariance, odom_pose_covariance,
      sizeof(odom_pose_covariance)),
      memcpy(&odom.twist.covariance, odom_twist_covariance,
        sizeof(odom_twist_covariance));
  }
  odom_publisher.publish(odom);
}

void turn_on_robot::publishRobotChassisStatus() {
  custom_msgs_srvs::robotChassisStatus robotChassisStatus_msg;

  robotChassisStatus_msg.chassis_motor_status = Receive_Data.Flag_Stop;
  robotChassisStatus_msg.chassis_batter_status.capacity = battery_info_.power;
  robotChassisStatus_msg.chassis_batter_status.voltage = battery_info_.voltage;
  robotChassisStatus_msg.chassis_batter_status.current = battery_info_.current;
  robotChassisStatus_msg.chassis_batter_status.temperature =
    battery_info_.temperature;
  robotChassisStatus_msg.chassis_motor_current.push_back(
    motor_info_.left_motor_current);
  robotChassisStatus_msg.chassis_motor_current.push_back(
    motor_info_.right_motor_current);
  robotChassisStatus_msg.chassis_current.current_19V = power_current_.v_19;
  robotChassisStatus_msg.chassis_current.current_12V = power_current_.v_12;
  robotChassisStatus_msg.chassis_current.current_5V = power_current_.v_5;
  robotChassisStatus_msg.chassis_fault_code_info = error_code_;

  robotChassisStatus_msg.chassis_charging_status =
    static_cast<int8_t>(charge_state_);
  robotChassisStatus_msg.chassis_emergency_switch_status =
    emergency_switch_status_;
  robotChassisStatus_msg.ultrasonic_sensors.push_back(ultrasound_1_);
  robotChassisStatus_msg.ultrasonic_sensors.push_back(ultrasound_2_);
  robotChassisStatus_msg.chassis_odom.push_back(left_odometer_);
  robotChassisStatus_msg.chassis_odom.push_back(right_odometer_);
  robotChassisStatus_msg.chassis_lamp_status.push_back(
    static_cast<int8_t>(left_light_state_));
  robotChassisStatus_msg.chassis_lamp_status.push_back(
    static_cast<int8_t>(right_light_state_));
  robotChassisStatus_msg.IR_alignment_status = IR_alignment_status_;
  robotChassisStatus_msg.loose_shaft_switch_status_info =
    loose_shaft_switch_state_;
  robotChassisStatus_msg.chassis_info = chassis_info_;

  robotChassisStatus_publisher.publish(robotChassisStatus_msg);
}

unsigned char turn_on_robot::Check_Sum(unsigned char Count_Number,
  unsigned char mode) {
  unsigned char check_sum = 0, k;

  if (mode == 0) {
    for (k = 0; k < Count_Number; k++) {
      check_sum = check_sum ^ Receive_Data.rx[k];
    }
  }
  if (mode == 1) {
    for (k = 0; k < Count_Number; k++) {
      check_sum = check_sum ^ Send_Data.tx[k];
    }
  }
  return check_sum;
}

bool turn_on_robot::Get_Sensor_Data() {
  std::vector<uint8_t> dataVector;
  size_t size;
  uint8_t Receive_Data_temp[200] = { 0 };
  Receive_Data.rx[RECEIVE_DATA_SIZE] = { 0 };
  size = Stm32_Serial.read(Receive_Data_temp, 200);
  for (int i = 0; i < size; i++) {
    rxDataBuf.push_back(Receive_Data_temp[i]);
  }

  //确保一定包含一个完整的帧
  if (rxDataBuf.size() < RECEIVE_DATA_SIZE * 2) {
    return false;
  }
  int i = 0;
  int pos_s = 0;
  int pos_e = 0;
  int pos_el = 0;
  std::vector<std::vector<int>> pos_list;
  std::vector<int> pos;
  while (i < rxDataBuf.size()) {
    //检验帧头
    if (rxDataBuf[i] == FRAME_HEADER && pos_e != pos_el) {
      pos_s = i;
      pos_el = pos_e;
    }
    if (rxDataBuf[i] == FRAME_TAIL) {
      pos_e = i;
    }
    if (pos_e - pos_s == RECEIVE_DATA_SIZE - 1) {
      pos.push_back(pos_s);
      pos.push_back(pos_e);
      pos_list.push_back(pos);
      pos.clear();
    }
    i++;
  }
  int num = pos_list.size();
  if (num <= 0) return false;
  int index = pos_list[num - 1][0];

  Receive_Data.Flag_Stop = rxDataBuf[index + 1];
  Robot_Vel.X = u8_to_u16(rxDataBuf[index + 2], rxDataBuf[index + 3])*0.1;
//  Robot_Vel.Y = u8_to_u16(rxDataBuf[index + 4], rxDataBuf[index + 5]);
Robot_Vel.Y=0;
  Robot_Vel.Z = u8_to_u16(rxDataBuf[index + 6], rxDataBuf[index + 7])*0.1;
//ROS_INFO("%f---%f---%f",Robot_Vel.X,Robot_Vel.Y,Robot_Vel.Z);

  battery_info_.power = u8_to_u16(rxDataBuf[index + 8], rxDataBuf[index + 9]);
  battery_info_.voltage =
    u8_to_u16(rxDataBuf[index + 10], rxDataBuf[index + 11]);
  battery_info_.current =
    u8_to_u16(rxDataBuf[index + 12], rxDataBuf[index + 13]);
  battery_info_.temperature =
    u8_to_u16(rxDataBuf[index + 14], rxDataBuf[index + 15]);
  motor_info_.left_motor_current =
    u8_to_u16(rxDataBuf[index + 16], rxDataBuf[index + 17]);
  motor_info_.right_motor_current =
    u8_to_u16(rxDataBuf[index + 18], rxDataBuf[index + 19]);
  power_current_.v_19 = u8_to_u16(rxDataBuf[index + 20], rxDataBuf[index + 21]);
  power_current_.v_12 = u8_to_u16(rxDataBuf[index + 22], rxDataBuf[index + 23]);
  power_current_.v_5 = u8_to_u16(rxDataBuf[index + 24], rxDataBuf[index + 25]);
  error_code_ = u8_to_u16(rxDataBuf[index + 26], rxDataBuf[index + 27]);
  charge_state_ = convertChargeState(
    u8_to_u16(rxDataBuf[index + 28], rxDataBuf[index + 29]));
  emergency_switch_status_ = rxDataBuf[index + 30];
  ultrasound_1_ = u8_to_u32(rxDataBuf[index + 31], rxDataBuf[index + 32],
    rxDataBuf[index + 33], rxDataBuf[index + 34]);
  ultrasound_2_ = u8_to_u32(rxDataBuf[index + 35], rxDataBuf[index + 36],
    rxDataBuf[index + 37], rxDataBuf[index + 38]);
  left_odometer_ = u8_to_u16(rxDataBuf[index + 39], rxDataBuf[index + 40]);
  right_odometer_ = u8_to_u16(rxDataBuf[index + 41], rxDataBuf[index + 42]);
  left_light_state_ = convertLightState(rxDataBuf[43]);
  right_light_state_ = convertLightState(rxDataBuf[44]);
  IR_alignment_status_ = rxDataBuf[45];
  loose_shaft_switch_state_ =
    u8_to_u16(rxDataBuf[index + 46], rxDataBuf[index + 47]);
  chassis_info_ = u8_to_u16(rxDataBuf[index + 48], rxDataBuf[index + 49]);

  rxDataBuf.clear();
  return true;
}

turn_on_robot::~turn_on_robot() {
  Send_Data.tx[0] = FRAME_HEADER;
  for (int i = 1; i < SEND_DATA_SIZE - 2; ++i) {
    Send_Data.tx[i] = 0;
  }

  Send_Data.tx[SEND_DATA_SIZE - 2] =
    Check_Sum(SEND_DATA_SIZE - 2, SEND_DATA_CHECK);
  Send_Data.tx[SEND_DATA_SIZE - 1] = FRAME_TAIL;
  try {
    Stm32_Serial.write(Send_Data.tx, sizeof(Send_Data.tx));
  }
  catch (serial::IOException& e) {
    ROS_ERROR_STREAM("Unable to send data through serial port");
  }
  Stm32_Serial.close();
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "pulan_robot");
  turn_on_robot Robot_Control;

  std::thread receive(receiveThread, &Robot_Control);
  std::thread send(sendThread, &Robot_Control);

  receive.join();
  send.join();

  return 0;
}
