# pulan_project

这里主要记录关于在普蓝机器人六轮双驱差速机器人的开发过程



# 1.pulan_teaching_nav_ws

## 1.1 方案介绍

主要记录了关于六轮双驱差速机器人教研版方案的开发

基于这个方案，能够使得初学者能够快速得搭建起来整个机器人系统，从而使得学习者能够通过使用这个软件，快速地对整个机器人系统有系统的认识。

项目框架：

![image-20241126213051496](https://cdn.jsdelivr.net/gh/su-ron/image/imgimage-20241126213051496.png)

机器人;

<img src="https://cdn.jsdelivr.net/gh/su-ron/image/imgimage-20241126213252576.png" alt="image-20241126213252576" style="zoom:50%;" />

最终的实现效果：

控制界面：

![image-20240625194137048](https://cdn.jsdelivr.net/gh/su-ron/image/imgimage-20240625194137048.png)

项目实现效果

<video src="README.assets/pulan_robot.mp4"></video>

## 1.2  方案阐述

在教研方案中，主要是基于ROS开源方案来进行开发的，使用A*算法进行全局路径规划生成全局路径，使用DWA算法实现局部路径规划，并下发控制命令到底层进行控制



# 2.auto_container_manager

主要记录了六轮双驱差速机器人充电上桩和下桩的算法

详细设计方案与参数调节可以查看auto_container_manager文件夹



# 3.chassic_driver

这个记录的是对接的上下位机的驱动程序，主要功能就是，通过接收下层上发的数据，然后通过相关话题来发布出来

具体的通信机制可以查看上下位机通信文档
