# 1.概述

## 1.1 基本原理

阿克曼几何的简化版 – 车辆单轨模型（自行车模型）

<img src="https://cdn.jsdelivr.net/gh/su-ron/image/imgimage-20240527204247092.png" alt="image-20240527204247092" style="zoom:67%;" />

好处就在于它简化了前轮转向角与后轴将遵循的曲率之间的几何关系，其关系如下式所示：
$$
t a n ( δ ) =\frac{L}{R} \\所以，δ=arctan\frac{L}{R}
$$
其中 δ表示前轮的转角，L为轴距，R则为在给定的转向角下后轴遵循着的圆的半径。通过该式可以知道，**当计算得到曲率的时候，就能得到前轮摆角**，

<img src="https://cdn.jsdelivr.net/gh/su-ron/image/imgimage-20240527204109439.png" alt="image-20240527204109439" style="zoom:67%;" />

注意：上面公式中的x0,y0就是x<sub>0</sub>‘，y<sub>0</sub>';

**L0指的是预瞄距离,(x<sub>0</sub>‘，y<sub>0</sub>')表示预瞄点**,**所以预瞄距离与前轮摆角成反比**

可得斜率计算公式：

<img src="https://cdn.jsdelivr.net/gh/su-ron/image/imgimage-20240527204452194.png" alt="image-20240527204452194" style="zoom:67%;" />

**//注意，这里的表示与一般车辆方向有所不同，一般车辆坐标系是：以车体建立坐标系，x正方向朝前，y正方向朝左**

前轮摆角和角速度的关系
$$
1/R=w/v \\
δ=arctan(l/R)=arctan(Lw/v)
$$



# 2.代码解读

## 2.1 主函数架构

```c++
int main(int argc, char** argv)
{
  ros::init(argc, argv, "pure_pursuit");
  waypoint_follower::PurePursuitNode ppn;
  ppn.run();

  return 0;
}
```

先对PurePursuitNode类进行构造，再执行run函数

## 2.1 关注其发布与订阅函数

在其构造函数中主要关注其发布与订阅

订阅

```c++
  // setup subscriber
  //得到当前轨迹
  sub1_ = nh_.subscribe("final_waypoints", 10,
    &PurePursuitNode::callbackFromWayPoints, this);
  //得到当前位姿
  sub2_ = nh_.subscribe("current_pose", 10,
    &PurePursuitNode::callbackFromCurrentPose, this);
  //获取参数
  sub3_ = nh_.subscribe("config/waypoint_follower", 10,
    &PurePursuitNode::callbackFromConfig, this);
  //得到当前速度
  sub4_ = nh_.subscribe("current_velocity", 10,
    &PurePursuitNode::callbackFromCurrentVelocity, this);
```

发布

```c++
  // setup publisher
  pub1_ = nh_.advertise<geometry_msgs::TwistStamped>("twist_raw", 10);
  pub2_ = nh_.advertise<autoware_msgs::ControlCommandStamped>("ctrl_raw", 10);
```



## 2.2 run函数

 **主体架构**

<img src="https://cdn.jsdelivr.net/gh/su-ron/image/imgimage-20240529152241450.png" alt="image-20240529152241450" style="zoom:67%;" />





## 2.3 判断获取曲率

框架：

![image-20240529155602360](https://cdn.jsdelivr.net/gh/su-ron/image/imgimage-20240529155602360.png)



### 2.3.1 寻找到下一个路径点

主要目的：找到大于lookahead_distance_的路径点下表，并赋给next_waypoint_number_

```c++
//基于前瞻距离，得到下一个路径点
void PurePursuit::getNextWaypoint()
{
  // note-tianyu 注意这里的current_waypoints_就是订阅到的final_waypoints(从closed waypoints往后延一定size)
  int path_size = static_cast<int>(current_waypoints_.size());

  // if waypoints are not given, do nothing.
  if (path_size == 0)
  {
    next_waypoint_number_ = -1;
    return;
  }

  // look for the next waypoint.
  for (int i = 0; i < path_size; i++)
  {
    // if search waypoint is the last
    if (i == (path_size - 1))
    {
      ROS_INFO("search waypoint is the last");
      next_waypoint_number_ = i;
      return;
    }

    // if there exists an effective waypoint
    // note-tianyu 计算当前position和current_waypoints_中的每一个waypoint的平面距离，判断是否存在可观测到的路标点
    //找到大于lookahead_distance_的路径点下表，并赋给next_waypoint_number_
    if (getPlaneDistance(
      current_waypoints_.at(i).pose.pose.position, current_pose_.position)
      > lookahead_distance_)
    {
      next_waypoint_number_ = i;
      return;
    }
  }

  // if this program reaches here , it means we lost the waypoint!
  next_waypoint_number_ = -1;
  return;
}
```



### 2.3.2 线性插值寻找前瞻点

**主要目的：lookahead_distance_来对目标waypoint的位置来进行插值运算，对位于当前车辆前方的目标路径点进行线性插值，以确定一个lookahead（前瞻）点。**

```c++
  // linear interpolation and calculate angular velocity
  //线性插值并计算角速度，基于next_waypoint_number_得到next_target_position_
  bool interpolation =interpolateNextTarget(next_waypoint_number_, &next_target_position_);
```

**大体思路**

<img src="https://cdn.jsdelivr.net/gh/su-ron/image/imgimage-20240529162333042.png" alt="image-20240529162333042" style="zoom: 80%;" />

#### **1.寻找到start与end点**

```c++
  geometry_msgs::Point end = current_waypoints_.at(next_waypoint).pose.pose.position;
  geometry_msgs::Point start = current_waypoints_.at(next_waypoint - 1).pose.pose.position;
```



#### **2.基于start与end点构建一次函数，并得到当前位置到直线上的距离**

```c++
 // let the linear equation be "ax + by + c = 0"
  // if there are two points (x1,y1) , (x2,y2),
  // a = "y2-y1, b = "(-1) * x2 - x1" ,c = "(-1) * (y2-y1)x1 + (x2-x1)y1"
  // note-tianyu 用于线性插值的公式为一次函数,start和end为接下来两个相邻waypoints
  double a = 0;
  double b = 0;
  double c = 0;
  double get_linear_flag = getLinearEquation(start, end, &a, &b, &c);
  if (!get_linear_flag)
    return false;

  // let the center of circle be "(x0,y0)", in my code ,
  // the center of circle is vehicle position
  // the distance  "d" between the foot of
  // a perpendicular line and the center of circle is ...
  //    | a * x0 + b * y0 + c |
  // d = -------------------------------
  //          √( a~2 + b~2)
  // note-tianyu 计算point到直线的距离
  double d = getDistanceBetweenLineAndPoint(current_pose_.position, a, b, c);
```



**基于两点构造一次函数**

```c++
// let the linear equation be "ax + by + c = 0"
// if there are two points (x1,y1) , (x2,y2), a = "y2-y1, b = "(-1) * x2 - x1" ,c = "(-1) * (y2-y1)x1 + (x2-x1)y1"
bool getLinearEquation(geometry_msgs::Point start, geometry_msgs::Point end, double *a, double *b, double *c)
{
  // (x1, y1) = (start.x, star.y), (x2, y2) = (end.x, end.y)
  double sub_x = std::fabs(start.x - end.x);
  double sub_y = std::fabs(start.y - end.y);
  double error = std::pow(10, -5);  // 0.00001

  if (sub_x < error && sub_y < error)
  {
    ROS_INFO("two points are the same point!!");
    return false;
  }

  *a = end.y - start.y;
  *b = (-1) * (end.x - start.x);
  *c = (-1) * (end.y - start.y) * start.x + (end.x - start.x) * start.y;

  return true;
}
```

**计算点到直线的距离**

```c++
double getDistanceBetweenLineAndPoint(geometry_msgs::Point point, double a, double b, double c)
{
  double d = std::fabs(a * point.x + b * point.y + c) / std::sqrt(std::pow(a, 2) + std::pow(b, 2));

  return d;
}
```



#### **3.获取单位向量unit_v与法向量**

```c++
  // unit vector of point 'start' to point 'end'
  tf::Vector3 v((end.x - start.x), (end.y - start.y), 0);
  tf::Vector3 unit_v = v.normalize();

  // normal unit vectors of v  获取unit_v的法向量
  // rotate to counter clockwise 90 degree
  tf::Vector3 unit_w1 = rotateUnitVector(unit_v, 90);
  // rotate to counter clockwise 90 degree
  tf::Vector3 unit_w2 = rotateUnitVector(unit_v, -90);
```



#### 4.计算两个垂足

```c++
  // 计算垂足，得到两个垂足，然后判断哪个垂足在直线上
  geometry_msgs::Point h1;
  h1.x = current_pose_.position.x + d * unit_w1.getX();
  h1.y = current_pose_.position.y + d * unit_w1.getY();
  h1.z = current_pose_.position.z;

  geometry_msgs::Point h2;
  h2.x = current_pose_.position.x + d * unit_w2.getX();
  h2.y = current_pose_.position.y + d * unit_w2.getY();
  h2.z = current_pose_.position.z;
  geometry_msgs::Point h;
  //判断垂足是否在直线上
  if (fabs(a * h1.x + b * h1.y + c) < ERROR)
  {
    h = h1;
    //   ROS_INFO("use h1");
  }
  else if (fabs(a * h2.x + b * h2.y + c) < ERROR)
  {
    //   ROS_INFO("use h2");
    h = h2;
  }
  else
  {
    return false;
  }
```

#### 5.判断垂足

```c++
// 相切情况
  if (d == search_radius)
  {
    *next_target = h;
    return true;
  }
  else
  {
    // if there are two intersection
    // 相交情况
    // get intersection in front of vehicle
    double s = sqrt(pow(search_radius, 2) - pow(d, 2));
    geometry_msgs::Point target1;
    target1.x = h.x + s * unit_v.getX();
    target1.y = h.y + s * unit_v.getY();
    target1.z = current_pose_.position.z;

    geometry_msgs::Point target2;
    target2.x = h.x - s * unit_v.getX();
    target2.y = h.y - s * unit_v.getY();
    target2.z = current_pose_.position.z;

    // ROS_INFO("target1 : ( %lf , %lf , %lf)", target1.x, target1.y, target1.z);
    // ROS_INFO("target2 : ( %lf , %lf , %lf)", target2.x, target2.y, target2.z);
    // displayLinePoint(a, b, c, target1, target2, h);  // debug tool

    // check intersection is between end and start
    //垂足点是在start与end两点之间的，所以通过判断距离可以获取到
    // 看下哪个交点是在车辆朝前的
    double interval = getPlaneDistance(end, start);
    if (getPlaneDistance(target1, end) < interval)
    {
      // ROS_INFO("result : target1");
      *next_target = target1;
      return true;
    }
    else if (getPlaneDistance(target2, end) < interval)
    {
      // ROS_INFO("result : target2");
      *next_target = target2;
      return true;
    }
    else
    {
      // ROS_INFO("result : false ");
      return false;
    }
  }
```



### 2.3.3 获取曲率

```c++
  // 基于车辆运动模型来进行曲率计算
  *output_kappa = calcCurvature(next_target_position_);
```

主要基于概述中的公式来推导获得曲率

```c++
double PurePursuit::calcCurvature(geometry_msgs::Point target) const
{
  double kappa;
  // note-tianyu 先计算target到当前position的平面距离
  double denominator = pow(getPlaneDistance(target, current_pose_.position), 2);
  //calcRelativeCoordinate(target, current_pose_)将点target转换到current_pose_坐标系上
  double numerator = 2 * calcRelativeCoordinate(target, current_pose_).y;

  if (denominator != 0)
  {
    kappa = numerator / denominator; //note-tianyu 参考阿克曼转向模型page10 公式(2)，以车体建立坐标系，x正方向朝前，y正方向朝左
  }
  else
  {
    if (numerator > 0)
    {
      kappa = KAPPA_MIN_;
    }
    else
    {
      kappa = -KAPPA_MIN_;
    }
  }
  ROS_INFO("kappa : %lf", kappa);
  return kappa;
}
```



## 2.4 发布控制命令

```c++
    publishTwistStamped(can_get_curvature, kappa);
    publishControlCommandStamped(can_get_curvature, kappa);
```

发布阿克曼的控制命令

```c++
void PurePursuitNode::publishControlCommandStamped(
  const bool& can_get_curvature, const double& kappa) const
{
  if (!publishes_for_steering_robot_)
  {
    return;
  }

  autoware_msgs::ControlCommandStamped ccs;
  ccs.header.stamp = ros::Time::now();
    //速度控制是没有做的
  ccs.cmd.linear_velocity = can_get_curvature ? computeCommandVelocity() : 0;
    //
  ccs.cmd.linear_acceleration = can_get_curvature ? computeCommandAccel() : 0;
  ccs.cmd.steering_angle =
    can_get_curvature ? convertCurvatureToSteeringAngle(wheel_base_, kappa) : 0;

  pub2_.publish(ccs);
}
```

速度控制是基于得到的轨迹点速度来获取的

```c++
double PurePursuitNode::computeCommandVelocity() const
{
  if (velocity_source_ == enumToInteger(Mode::dialog))
  {
    return getSgn() * kmph2mps(const_velocity_);
  }

  return command_linear_velocity_;
}
```

加速度控制是基于当前速度和下一个轨迹点速度来获取的

```c++
double PurePursuitNode::computeCommandAccel() const
{
  const geometry_msgs::Pose current_pose = pp_.getCurrentPose();
  const geometry_msgs::Pose target_pose =
    pp_.getCurrentWaypoints().at(1).pose.pose;

  // v^2 - v0^2 = 2ax
  const double x =
      std::hypot(current_pose.position.x - target_pose.position.x,
        current_pose.position.y - target_pose.position.y);
  const double v0 = current_linear_velocity_;
  const double v = computeCommandVelocity();
  const double a = getSgn() * (v * v - v0 * v0) / (2 * x);
  return a;
}
```

前轮转角控制

```c++

double convertCurvatureToSteeringAngle(
  const double& wheel_base, const double& kappa)
{
  return atan(wheel_base * kappa);//参考转向模型，page10-(3)
}
```

