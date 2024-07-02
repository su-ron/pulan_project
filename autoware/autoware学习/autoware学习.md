# 0.demo演示

![image-20240507194912282](https://cdn.jsdelivr.net/gh/su-ron/image/imgimgimage-20240507194912282.png)

最后的实现效果：

巡检小车astar和op planner实现避障规划



# 1.Autoware概述

<img src="https://cdn.jsdelivr.net/gh/su-ron/image/imgimgimage-20240507201927550.png" alt="image-20240507201927550" style="zoom: 50%;" />

autoware

<img src="https://cdn.jsdelivr.net/gh/su-ron/image/imgimgimage-20240507202113894.png" alt="image-20240507202113894" style="zoom:50%;" />

关键网站

<img src="https://cdn.jsdelivr.net/gh/su-ron/image/imgimage-20240507202232099.png" alt="image-20240507202232099" style="zoom:33%;" />

课程网站

<img src="https://cdn.jsdelivr.net/gh/su-ron/image/imgimgimage-20240507202440400.png" alt="image-20240507202440400" style="zoom:33%;" />



单独编译一个包

```
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select 包名
```





# 5.仿真环境搭建

## 5.1 仿真的必要性及常见仿真工具介绍

仿真工具

![image-20240507204630972](https://cdn.jsdelivr.net/gh/su-ron/image/imgimage-20240507204630972-1715516709403-13.png)



网址：https://gitee.com/ecoxiaoyongguo/autoware.test?skip_mobile=true

https://gitee.com/ren_sixu/autoware.project



# 5.2 仿真模块介绍和源码解析

<img src="https://cdn.jsdelivr.net/gh/su-ron/image/imgimgimage-20240507205848075.png" alt="image-20240507205848075" style="zoom:50%;" />

# 6.决策规划模块

## 6.1 概述

![image-20240508172800553](https://cdn.jsdelivr.net/gh/su-ron/image/imgimgimage-20240508172800553.png)

## 6.2 规划决策方案

![image-20240508173200649](https://cdn.jsdelivr.net/gh/su-ron/image/imgimgimage-20240508173200649.png)

## 6.3 lane_planner模块源码解析和仿真

### 6.3.1概述

lane_planner实际上是为了输出一条基本的轨迹

![image-20240508173645189](https://cdn.jsdelivr.net/gh/su-ron/image/imgimgimage-20240508173645189.png)

lane stop根据交通灯信息，输出一条满足交规的轨迹;
lane select根据上层指令确认是否需要切换lane，并监控当前车辆在轨迹中的行驶状况

**方式一、最基本的决策规划方案**

![image-20240508174136256](https://cdn.jsdelivr.net/gh/su-ron/image/imgimgimage-20240508174136256.png)

new_mission_planning.launch可以看到决策规划模块是由四个部分组成

```makefile
  <!-- lane_navi start-->
  <!-- -->
  <include file="$(find lane_planner)/launch/lane_navi.launch" />
  <!-- lane_navi end-->

  <!-- lane_rule -->
  <node pkg="lane_planner" type="lane_rule" name="lane_rule" output="screen" />

  <!-- lane_stop -->
  <node pkg="lane_planner" type="lane_stop" name="lane_stop" output="screen" />

  <!-- lane_select -->
  <node pkg="lane_planner" type="lane_select" name="lane_select" output="screen" />
```

### **6.3.2 lane_navi模块**

lane_navi节点的主要作用：根据路由请求在矢量地图中寻找通往目的地的各条可行路径，并发布到话题“lane_waypoints_array"

```c++
  waypoint_pub = n.advertise<autoware_msgs::LaneArray>("/lane_waypoints_array", pub_waypoint_queue_size,
                 pub_waypoint_latch);
  //矢量地图的点，车道，节点
  //point包含经纬高和x,y坐标
  ros::Subscriber point_sub = n.subscribe("/vector_map_info/point", sub_vmap_queue_size, cache_point);
  //由两个Node组成，起点Node和终点Node
  ros::Subscriber lane_sub = n.subscribe("/vector_map_info/lane", sub_vmap_queue_size, cache_lane);
  //
  ros::Subscriber node_sub = n.subscribe("/vector_map_info/node", sub_vmap_queue_size, cache_node);
```

回调函数cache_point的作用：**主要工作是把vector_map的信息解析出来，转成autoware_msgs::LaneArray进行发布**，得到一条基本的轨迹

```c++
void cache_point(const vector_map::PointArray& msg)
{
  all_vmap.points = msg.data;
  update_values();
}
```



### **6.3.3 lane_rule模块**

**作用：对节点lane_navi发布在”/lane_waypoints_array"上的规划路径在轨迹点速度方面进一步修正，为红灯时在停车线内减速停车等场景提供支持**

```c++
  //订阅lane_navi模块发出的信息
  ros::Subscriber waypoint_sub = n.subscribe("/lane_waypoints_array", sub_waypoint_queue_size, create_waypoint); 
  //停止线信息
  ros::Subscriber stopline_sub = n.subscribe("/vector_map_info/stop_line", sub_vmap_queue_size, cache_stopline);
  //
  traffic_pub =
      n.advertise<autoware_msgs::LaneArray>("/traffic_waypoints_array", pub_waypoint_queue_size, pub_waypoint_latch);
 //红灯时要走的路径轨迹
  red_pub = n.advertise<autoware_msgs::LaneArray>("/red_waypoints_array", pub_waypoint_queue_size, pub_waypoint_latch);
 //绿灯时要走的路径轨迹
  green_pub =
      n.advertise<autoware_msgs::LaneArray>("/green_waypoints_array", pub_waypoint_queue_size, pub_waypoint_latch);
```



### 6.3.4 lane_stop模块

**作用：根据信号灯相位选择之前节点lane_rule发布的红/绿灯时导航路径("/red_waypoints_array"和"/green_waypoints_array"），并将其发布到话题"/traffic_waypoints_array"**



### 6.3.5 lane_select模块

作用：判断当前车道，同时规划从当前车道切换至其它车道的轨迹，接着根据话题“state"的驾驶状态（是否需要换道）发布当前车道数据/换道轨迹数据至话题"base_waypoints"供其它节点继续规划

```c++
  // setup subscriber
  sub1_ = nh_.subscribe("traffic_waypoints_array", 1, &LaneSelectNode::callbackFromLaneArray, this);
  sub2_ = nh_.subscribe("current_pose", 1, &LaneSelectNode::callbackFromPoseStamped, this);
  sub3_ = nh_.subscribe("current_velocity", 1, &LaneSelectNode::callbackFromTwistStamped, this);
  //note-tianyu 下面三个在仿真场景下不订阅
  sub4_ = nh_.subscribe("state", 1, &LaneSelectNode::callbackFromState, this);
  sub5_ = nh_.subscribe("/config/lane_select", 1, &LaneSelectNode::callbackFromConfig, this);
  sub6_ = nh_.subscribe("/decision_maker/state", 1, &LaneSelectNode::callbackFromDecisionMakerState, this);

  // setup publisher

  pub1_ = nh_.advertise<autoware_msgs::Lane>("base_waypoints", 1);
  pub2_ = nh_.advertise<std_msgs::Int32>("closest_waypoint", 1);
  pub3_ = nh_.advertise<std_msgs::Int32>("change_flag", 1);
  pub4_ = nh_.advertise<std_msgs::Int32>("/current_lane_id", 1);
  pub5_ = nh_.advertise<autoware_msgs::VehicleLocation>("vehicle_location", 1);
```

发布的closet_waypoint的计算



## 6.4 astar规划模块源码解析

### 6.4.1 方案介绍

**方式2：基于局部静态规划方案**

<img src="C:/Users/su/Desktop/%E8%87%AA%E5%8A%A8%E9%A9%BE%E9%A9%B6/image/image-20240510113342118.png" alt="image-20240510113342118" style="zoom:67%;" />

**方式3：基于手动指定终点的全局路径规划**

<img src="https://cdn.jsdelivr.net/gh/su-ron/image/imgimgimage-20240510113642894.png" alt="image-20240510113642894" style="zoom:67%;" />

### 6.4.2 A*算法介绍



**代码解析**

实现效果：

<img src="C:/Users/su/Desktop/%E8%87%AA%E5%8A%A8%E9%A9%BE%E9%A9%B6/image/image-20240510121751046.png" alt="image-20240510121751046" style="zoom:67%;" />











# 7.控制模块

## 7.1概述

![image-20240508155521565](https://cdn.jsdelivr.net/gh/su-ron/image/imgimgimage-20240508155521565.png)
