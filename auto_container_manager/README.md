# 1.自动充电流程

## 1.1 简介

auto_container_manager功能包是用于对接自动货柜业务的功能包，通过接收上位机指令触发机器人进行货柜特征识别及完成对位控制操作。



## 1.2 整体架构及流程

流程：

![img](https://cdn.jsdelivr.net/gh/su-ron/image/imgwps1.png)

​                                                                                          **图1  自动充电流程图**

自动对桩充电业务流程较为简单，由上位机控制下发指令，机器人进入货柜标志识别及对桩控制流程，并实时反馈识别及对桩的过程及状态，由上位机判断是否继续下一步操作。



主要代码架构如下：

![image-20240625192610010](F:/%E8%87%AA%E5%8A%A8%E9%A9%BE%E9%A9%B6/image/image-20240625192610010.png)

​                                                                                             图1 自动充电代码构成图

 代码框架中主要由三个类组成：

1. AutoContainerManager类，主函数，构建回调scancb()、getLoadTaskSrv()以及loop()的while()循环执行程序内容，内容如下：

- ​      **getLoadTaskSrv()**回调获取货柜对位任务执行或关闭指令，相关指令内容可查看《机器人自动货柜接口及信息表V0.1.1》文档。
- ​      **autoContainerFlow()**启动scanCB()的特征识别；
- ​      **autoContainerFlow()**基于识别信息，进行对位控制；
- ​      **determineRelativePose()** 过程中实时判断，位姿偏差是否过大（存在人为推动等）并发布故障码信息；
- ​      **publishAutoContainerStatus()** 实时发布状态信息给上位机，以便上位机知晓完成对位指令进行后续流程或知晓故障信息进行相关重试或报障处理



2. CalibrateMotionControl类，基于激光雷达数据和机器人轮廓信息，检测当前状态是否存在障碍物影响机器人前进及旋转，功能与自动充电桩对位控制一致；



3. LabelRecognizer类，基于激光雷达数据进行三角特征识别及检测，该功能与自动充电桩识别充电桩检测功能一致，



# 2.功能模块描述

​      在上述代码架构流程中，autoContainerFlow()的处理流程及相关状态是其关键模块，以下将主要介绍其中各个状态及相关处理流程；



## 2.1 整理流程状态标志说明

​     在循环执行autoContainerFlow()函数中，通过对auto_container_state_变量进行切换，完成整体功能实现，该变量初始状态为**AutoContainerState::**STATE_NOT_TASKING。详细流程说明如下Table1：

| 状态                                              | 描述                                                         | 切换条件                                                     |
| ------------------------------------------------- | ------------------------------------------------------------ | ------------------------------------------------------------ |
| **AutoContainerState::**STATE_NOT_TASKING         | 无任务状态：1. scan回调功能挂起2. loop()循环处理挂起         | 获取到上位机关闭指令                                         |
| **AutoContainerState::**STATE_GET_TASKING_CMD     | 执行命令状态：1. loop()循环执行任务2. 初始化scan回调的topic，并开始允许特征检测3. 切换状态STATE_REACH_TASKING_POINT | 获取到上位机启动指令                                         |
| **AutoContainerState::**STATE_REACH_TASKING_POINT | 到点检测状态：1. Loop()循环执行任务2. 检测特征检测结果，若失败则进行原地摆动控制3. 若超过一定次数检测失败则上报177故障码4. 若检测成功，切换状态STATE_ALIGN | 因为上位机只有到点才下发执行指令，故而此处获取到上位机启动指令，在状态STATE_GET_TASKING_CMD下，下一步就直接切换该状态 |
| **AutoContainerState::**STATE_ALIGN               | 对位控制状态：1. Loop()循环执行任务2. 执行校准对位控制3. 若对位成功，切换状态STATE_FINISH_TASKING | 在STATE_REACH_TASKING_POINT状态下，特征检测成功后切换该状态  |
| **AutoContainerState::**STATE_FINISH_TASKING      | 任务完成状态：1. Loop()循环执行任务2. 完成任务无操作         | 在STATE_ALIGN状态下，校准对位成功后切换该状态                |

​                                                                                               **Table 1 整体运行流程状态说明**



## 2.2 控制状态流程标志说明

​        在**AutoContainerState::**STATE_ALIGN状态下，进入AutoContainerManager::**alignment**()函数会使机器人进行水平及前后的对位校准，若有其他的对位算法，可直接更替该函数即可，当前进行校准对位处理，存在下述相关状态变量（alignment_state_），详细说明如下Table2：

​                                                                  

| 状态                                               | 描述                                                         | 切换条件                                                     |
| -------------------------------------------------- | ------------------------------------------------------------ | ------------------------------------------------------------ |
| **ControlState::**STATE_READY_CALIBRATION          | 准备运行状态：1. 获得偏转90°水平的朝向误差；2. 进入STATE_POSITION_CALIBRATION状态 | 整体流程进入到**AutoContainerState::**STATE_ALIGN时，进行切换到该状态 |
| **ControlState::**STATE_POSITION_CALIBRATION       | 位姿误差判定状态：1.对比当前位姿与特征位姿态的横向Y轴误差；2.若误差较大，进入STATE_ROTATE_CALIBRATION状态，实现水平校准3.若误差较小，进入STATE_FROW_BACK_ORI_CONFILM状态，直接进行前后校准 | STATE_READY_CALIBRATIO状态后直接进行切换到该状态             |
| **ControlState::**STATE_ROTATE_CALIBRATION         | 旋转角度控制状态：1. 依据旋转角度偏差，进行原地旋转校准2. 若无阻挡，则依据角度误差大小，线性的调整下发角速度3. 达到误差精度后，下发零速，切换状态到STATE_LATERAL_CALIBRATION | 1.STATE_FROW_BACK_ORI_CONFILM状态后若误差较大（详情见参数说明），则切换该状态进行原地旋转到与特征物水平。 |
| **ControlState::**STATE_FROW_BACK_CALIBRATION      | 前后调整状态：1. 依据前后误差进行前后移动，校准X轴偏差2. 校准完成后，切换STATE_FINISH_CALIBRATION状态 | 1.STATE_FROW_BACK_ORI_CONFILM状态后，则切换该状态进行前后误差校准 |
| **ControlState::**STATE_LATERAL_CALIBRATION        | 水平前后调整状态：1. 机器人在旋转90度后，相对充电桩水平，开始前后移动，校准Y轴偏差2.旋转校准后切换STATE_LATERAL_ORI_CONFILM状态 | STATE_ROTATE_CALIBRATION状态后切换该状态                     |
| **ControlState::**STATE_LATERAL_ORI_CONFILM        | 水平旋转到正面角度误差确认状态：1. 确认当前位姿，需要旋转多少角度才能将机器旋转回正对特征物中心2.计算完成后，继续切换STATE_LATERAL_ROTATE_CALIBRATION状态 | STATE_LATERAL_CALIBRATION状态后切换该状态                    |
| **ControlState::**STATE_LATERAL_ROTATE_CALIBRATION | 水平旋转到正面校准控制状态：1. 基于旋转偏差，控制机器人旋转回正对特征物中心2.旋转校准完后，切换STATE_FROW_BACK_ORI_CONFILM状态 | STATE_LATERAL_ORI_CONFILM状态后，水平校准完毕后，则切换该状态进行原地旋转到与特征物正对 |
| **ControlState::**STATE_FROW_BACK_ORI_CONFILM      | 前后移动误差确认状态：1.确认当前位姿，需要前后移动多少才能将机器人往前实现对位2.计算完成后，继续切换STATE_FROW_BACK_CALIBRATION状态 | 1.STATE_LATERAL_ROTATE_CALIBRATION状态后，则切换该状态进行前后误差确定2.STATE_POSITION_CALIBRATION状态后若误差较小（详情见参数说明），则切换该状态 |
| **ControlState::**STATE_FINISH_CALIBRATION         | 校准完成状态：1. 校准完成，不做相关处理2. 切换整体流程状态到**AutoContainerState::**STATE_FINISH_TASKING | STATE_FROW_BACK_CALIBRATION状态完成后，则切换该状态          |

​                                                                                                        **Table 2 对位控制流程状态说明** 



## 2.3 参数说明

| ***\*参数\****                      | ***\*数值\**** | ***\*说明\****                                               |
| ----------------------------------- | -------------- | ------------------------------------------------------------ |
| obstacle_check_area_length          | 0.3            | 若为方形机器人，障碍物点x轴方向x值小于此值，则认为不可前进，单位m |
| obstacle_check_area_width           | 0.5            | 若为方形机器人，障碍物点y轴方向y值小于此值的1/2，则认为不可前进，单位m |
| circumscribed_radius                | 0.45           | 若为方形机器人，障碍物点距离小于此值，则认为存在不可旋转，单位m |
| circular_obstacle_check_area_length | 0.3            | 若为圆形机器人，障碍物点x轴方向x值小于此值，则认为不可前进，单位m |
| circular_obstacle_check_area_width  | 0.5            | 若为圆形机器人，障碍物点y轴方向y值小于此值的1/2，则认为不可前进，单位m |
| circular_circumscribed_radius       | 0.275          | 若为圆形机器人，障碍物点距离小于此值，则认为存在不可旋转，单位m |
| test_mode                           | 0              | 测试模式，1为测试可直接识别特征，0位关闭不识别，需要通过正规流程才会进行识别 |
| filter_range                        | 1.5            | 识别标志物的有效距离，超过此距离的数据不纳入识别检测，单位m  |
| angle_tolerance                     | 0.01           | 原地旋转，校准角度精度，单位rad                              |
| distance_tolerance                  | 0.01           | 前后移动，校准位置距离精度，单位m                            |
| relative_distance_tolerance         | 0.53           | 机器人中心离标志物中心保持的纵向距离，单位m                  |
| robot_distance_tolerance            | -0.01          | 机器人横向偏差小于此值可不做横向水平移动校准，此处设置为负数，则默认都需要横向水平校准，单位m |
| check_distance_error_tolerance      | 2.0            | 对位及装载过程中，检测到机器人离标志物最大距离则报错178，单位m |
| check_angle_error_tolerance         | 30.0           | 对位及装载过程中，检测到机器人离标志物最大偏转角则报错179，单位° |
| angle_kp                            | 0.6            | 原地旋转，基于旋转角度误差调整旋转角速度的比例系数           |
| distance_kp                         | 0.4            | 前后移动，基于移动位置误差调整线速度的比例系数               |

​                                                                                                          **Table 3 配置参数说明**



# 3. 充电桩下桩流程说明

充电桩下桩功能还是在auto_container_manager功能包中实现的。机器人底盘充电时通过接收上位机指令来判断是否需要进行下桩任务。





# 4.目前存在问题

4.1 运动校准方案：与自动充电控制校准一致，效率不高，无法一步到位。

考虑：可以使用五次多项式生成曲线和MPC控制的方法来对这个自动充电方案去进行优化







