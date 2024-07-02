# 请进代码仓库的同学尽快clone代码,仓库人员已满,需要人员流动管理

# autoware.auto carla Tier IV

欢迎大家关注我的抖音账号“Tech天宇”，里面会经常更新自动驾驶知识点（包括但不限制于autoware.auto/universe, carla模拟器，Tier IV公司最新消息等等，出于对我们这门课程的保护，抖音内容不会出现本门课程视频的内容，所以属于额外的知识点，有精力的同学可以关注下）

# autoware.project

autoware.project专门为参加“自动驾驶框架Autoware源码解析与项目实战”的同学们日常学习而建立。

各分支介绍如下:

master 源代码分支，同课程内容中的源代码讲解一致，随着课程的推进持续更新中;

demo_dataset 数据集分支，正式跑demo前请先将其配置好，其中包括点云地图，矢量地图，配置文件等相应对应不同模拟场景的demo数据集,(docker镜像中包含);

homework 分支，包含每个章节项目练习的原始数据，参考代码等等;

其他分支随着课程推进陆续解索。。。

# 仿真环境部署(如果使用课程提供的docker镜像，请忽略这个步骤)

准备工作：

1、将网盘中课程资料里的gazebo模型库中"models.zip"下载下来并解压；

2、将解压得到的"models"文件夹放在"/home/用户名/.gazebo"下,replace原有；

3、"./gazebo"为一个隐藏文件夹，如果没有说明没有运行过gazebo，运行一次后会自动生成；

4、将网盘中课程资料里的gazebo模型库中"actor_collisions.zip"下载并解压；

5、cd actor_collisions;mkdir build;cd build;cmake ..;make;

6、将生成的"libActorCollisionsPlugin.so"放入/usr/lib/x86_64-linux-gnu/gazebo-9/plugins/

7、pull本repo下的demo_dataset分支最新版，并更新到".autoware"文件夹；

# 仿真1:Lesson2 建图

1、roslaunch autoware_quickstart_examples my_mapping.launch

2、rviz -d src/autoware/documentation/autoware_quickstart_examples/config/default.rviz;

3、rosbag play sample_msimcity_lidar_imu.bag（从课程平台下载bag）

# 仿真2：简化版的仿真启动

仿真2为简易的仿真环境，车辆静止，主要便于大家进行感知模块的仿真,操作步骤如下：

依次启动如下文件：

1、roslaunch autoware_quickstart_examples mini_map.launch；

2、roslaunch autoware_quickstart_examples mini_localization.launch;

3、rviz -d src/autoware/documentation/autoware_quickstart_examples/config/default.rviz;

4、手动给定一个初始位姿(根据车辆位置来选)；

5、roslaunch autoware_quickstart_examples mini_sil_env.launch（多等一会）;

6、roslaunch autoware_quickstart_examples mini_detection.launch（多等一会）;

# 仿真3：完整版的仿真启动，标准的启动

仿真3为完整的仿真环境，后面讲解的规划控制模块都是基于它,操作步骤如下：

依次启动如下文件：

1、roslaunch autoware_quickstart_examples new_map.launch；

2、roslaunch autoware_quickstart_examples new_localization.launch;

3、rviz -d src/autoware/documentation/autoware_quickstart_examples/config/default.rviz;

4、手动给定一个初始位姿；

5、roslaunch vehicle_gazebo_simulation_launcher world_test_citysim_a.launch(多等一会，3-5mins都有可能)

6、roslaunch vehicle_gazebo_simulation_launcher world_test_citysim_b.launch(等到前面的gazebo world启动成功且定位成功后再启动)

7、roslaunch autoware_quickstart_examples new_detection.launch

8、roslaunch autoware_quickstart_examples new_mission_planning.launch

9、roslaunch autoware_quickstart_examples new_motion_planning.launch

# 仿真4：基于op_global_planner的全局路径规划，手动给定终点，遇到障碍物停止

依次启动如下文件：

1、roslaunch autoware_quickstart_examples new_global_plan_map.launch；

2、roslaunch autoware_quickstart_examples new_localization.launch;

3、roslaunch autoware_quickstart_examples new_detection.launch

4、rviz -d src/autoware/documentation/autoware_quickstart_examples/config/default.rviz;

5、手动给定一个初始位姿；

6、roslaunch vehicle_gazebo_simulation_launcher world_test_citysim_a.launch(多等一会，3-5mins都有可能)

7、roslaunch vehicle_gazebo_simulation_launcher world_test_citysim_b.launch(等到前面的gazebo world启动成功且定位成功后再启动)

8、roslaunch autoware_quickstart_examples new_op_global_planning.launch

9、roslaunch autoware_quickstart_examples new_motion_planning.launch

10、沿着车道方向给一个goal

11、rostopic pub /light_color_managed autoware_msgs/TrafficLight "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
traffic_light: 0" 模拟红灯信号，汽车停在路口前

12、rostopic pub /light_color_managed autoware_msgs/TrafficLight "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
traffic_light: 1" 模拟绿灯信号，汽车路口起步

# 仿真5：基于op_local_planner的全局路径规划，计算每条轨迹权重

依次启动如下文件：

1、roslaunch autoware_quickstart_examples new_map.launch；

2、roslaunch autoware_quickstart_examples new_localization.launch;

3、roslaunch autoware_quickstart_examples new_detection.launch

4、rviz -d src/autoware/documentation/autoware_quickstart_examples/config/default.rviz;

5、手动给定一个初始位姿；

6、roslaunch vehicle_gazebo_simulation_launcher world_test_citysim_a.launch(多等一会，3-5mins都有可能)

7、roslaunch vehicle_gazebo_simulation_launcher world_test_citysim_b.launch(等到前面的gazebo world启动成功且定位成功后再启动)

8、roslaunch autoware_quickstart_examples new_op_local_planner.launch

9、roslaunch autoware_quickstart_examples new_motion_planning.launch

10、在rviz中选择好相应的topic，以便规划轨迹可以显示出来,手动添加障碍物，可以看到轨迹颜色变化

# 仿真6：基于astar的路径规划，手动给定终点，遇到障碍停止

依次启动如下文件：

1、roslaunch autoware_quickstart_examples new_map.launch；

2、roslaunch autoware_quickstart_examples new_localization.launch;

3、roslaunch autoware_quickstart_examples new_detection.launch

4、rviz -d src/autoware/documentation/autoware_quickstart_examples/config/default.rviz;

5、手动给定一个初始位姿；

6、roslaunch vehicle_gazebo_simulation_launcher world_test_citysim_a.launch(多等一会，3-5mins都有可能)

7、roslaunch vehicle_gazebo_simulation_launcher world_test_citysim_b.launch(等到前面的gazebo world启动成功且定位成功后再启动)

8、roslaunch autoware_quickstart_examples new_manual_astar_planner.launch

9、roslaunch autoware_quickstart_examples new_motion_planning.launch

10、在costmap上给定一个goal（注意rviz需要更换下显示的topic）

# 仿真7：基于astar的避障路线规划，前方出现障碍物，规划路线避开

依次启动如下文件：

1、roslaunch autoware_quickstart_examples new_map.launch；

2、roslaunch autoware_quickstart_examples new_localization.launch;

3、roslaunch autoware_quickstart_examples new_detection.launch

4、rviz -d src/autoware/documentation/autoware_quickstart_examples/config/default.rviz;

5、手动给定一个初始位姿；

6、roslaunch vehicle_gazebo_simulation_launcher world_test_citysim_a.launch(多等一会，3-5mins都有可能)

7、roslaunch vehicle_gazebo_simulation_launcher world_test_citysim_b.launch(等到前面的gazebo world启动成功且定位成功后再启动)

8、roslaunch autoware_quickstart_examples new_manual_astar_planning.launch

9、roslaunch costmap_generator costmap_generator.launch

10、roslaunch autoware_quickstart_examples new_avoid_motion_planning.launch

# autoware.ai环境配置:

源码和docker二选一

# 源码编译安装：

安装Ubuntu 18.04（建议使用双系统，虚拟机会很卡）;

安装ROS Melodic（可以使用鱼香ROS，一键自动安装）;

wget http://fishros.com/install -O fishros && . fishros

安装Ubuntu/ROS系统依赖;

sudo apt update

sudo apt install python3-pip

sudo apt install -y python-catkin-pkg python-rosdep ros-$ROS_DISTRO-catkin

sudo apt install -y python3-pip python3-colcon-common-extensions python3-setuptools python3-vcstool

pip3 install -U setuptools

rosdep install -y --from-paths src --ignore-src --rosdistro melodic

创建工作空间;

mkdir -p autoware.ai

cd到安装目录;

cd autoware.ai

clone代码repo;

git clone -b master https://gitee.com/ren_sixu/autoware.project.git

clone地图配置等辅助文件;

cd到/home/user下;

mkdir -p .autoware

cd .autoware

git clone -b demo_dataset https://gitee.com/ren_sixu/autoware.project.git

编译指令;

colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

注意：编译过程中会提示确认package，一般都是ros的thirdlib，缺少什么install什么就行，然后重新编译;

# docker镜像安装：

安装docker软件；

从课程资料中下载镜像文件ai.tar;

systemctl restart docker(一般需要执行一下这句);

docker load -i ai.tar

docker images查看导入镜像的image id

docker tag IMAGEID autoware/autoware:latest

cd /project/root/path

xhost +

chmod +x ai_docker.sh

./ai_docker.sh

cd /project/root/path/in/docker

colcon build(If the docker loading success, it will compile successful)

进行更改以后别忘了保存当前容器：docker commit -m="描述信息" -a="作者" 容器id 目标镜像名： [TAG]

# 官方demo启动指令：（别忘了source，参考1.5节课程或者直播的操作过程）

roslaunch autoware_quickstart_examples my_map.launch

roslaunch autoware_quickstart_examples my_localization.launch

rviz -d src/autoware/documentation/autoware_quickstart_examples/config/default.rviz 

rosbag play sample_moriyama_150324.bag（从课程平台下载bag）

roslaunch autoware_quickstart_examples my_detection.launch

roslaunch autoware_quickstart_examples my_mission_planning.launch

roslaunch autoware_quickstart_examples my_motion_planning.launch

TODO。。。。 持续更新中。。。。。。。