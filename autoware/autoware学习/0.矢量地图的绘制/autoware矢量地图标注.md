# 1.使用vetor map builder在线标注方法







# 2.使用autoware Maptools插件进行矢量地图的绘制

参考网址：**https://blog.csdn.net/fearloo/article/details/134312633**

## 2.1 Unity安装教程

https://blog.csdn.net/Dugege007/article/details/128472571?ops_request_misc=%257B%2522request%255Fid%2522%253A%2522171522822716800182723515%2522%252C%2522scm%2522%253A%252220140713.130102334..%2522%257D&request_id=171522822716800182723515&biz_id=0&utm_medium=distribute.pc_search_result.none-task-blog-2~all~top_positive~default-1-128472571-null-null.142^v100^pc_search_result_base7&utm_term=unity%E5%AE%89%E8%A3%85&spm=1018.2226.3001.4187

在unity添加MapToolbox报错

参考：https://blog.csdn.net/hubiao1235/article/details/126369289?ops_request_misc=%257B%2522request%255Fid%2522%253A%2522171523620416777224420468%2522%252C%2522scm%2522%253A%252220140713.130102334.pc%255Fall.%2522%257D&request_id=171523620416777224420468&biz_id=0&utm_medium=distribute.pc_search_result.none-task-blog-2~all~first_rank_ecpm_v1~rank_v31_ecpm-2-126369289-null-null.142^v100^pc_search_result_base7&utm_term=Library%5CPackageCache%5Ccom.unity.jobs%400.8.0-preview.23%5CUnity.Jobs%5CIJobParallelForDefer.cs%2877%2C85%29%3A%20error%20CS8377%3A%20The%20type%20U%20must%20be%20a%20non-nullable%20value%20type%2C%20along%20with%20all%20fields%20at%20any%20level%20of%20nest&spm=1018.2226.3001.4187

<img src="C:/Users/su/Desktop/%E8%87%AA%E5%8A%A8%E9%A9%BE%E9%A9%B6/image/image-20240509144204897.png" alt="image-20240509144204897" style="zoom:50%;" />

2.绘制曲线

https://github.com/autocore-ai/MapToolbox/issues/48



## 2.2 矢量地图制作



![image-20240510181053093](https://cdn.jsdelivr.net/gh/su-ron/image/imgimage-20240510181053093.png)

![image-20240510181201243](https://cdn.jsdelivr.net/gh/su-ron/image/imgimage-20240510181201243.png)

导入MapToolBox包

![image-20240510182245726](https://cdn.jsdelivr.net/gh/su-ron/image/imgimage-20240510182245726.png)



![image-20240510182333756](https://cdn.jsdelivr.net/gh/su-ron/image/imgimage-20240510182333756.png)



网址：https://github.com/autocore-ai/MapToolbox.git

注意选择vector_map分支，下载下来

![image-20240510182529367](https://cdn.jsdelivr.net/gh/su-ron/image/imgimage-20240510182529367.png)



添加来自磁盘的包，解压后，选择package.json文件

![image-20240510182707372](https://cdn.jsdelivr.net/gh/su-ron/image/imgimage-20240510182707372.png)

此时会出现错误

![image-20240510182824659](https://cdn.jsdelivr.net/gh/su-ron/image/imgimage-20240510182824659.png)

解决方法： 解决办法：

打开Package Manager包管理器
在Unity的界面，点击Window选项中的Package Manager，并且点击加号中的 "Add package from git URL..." ，add packages  “com.unity.entities”



此时MapToolBox包导入成功，

<img src="https://cdn.jsdelivr.net/gh/su-ron/image/imgimage-20240510183329949.png" alt="image-20240510183329949" style="zoom:50%;" />

1.将点云地图导入

将点云地图导入你的assets文件夹中

![image-20240510183505301](https://cdn.jsdelivr.net/gh/su-ron/image/imgimage-20240510183505301.png)

<img src="https://cdn.jsdelivr.net/gh/su-ron/image/imgimage-20240512094520302.png" alt="image-20240512094520302" style="zoom:50%;" />

2.新建游戏对象->3D对象->平面

并且调整plane的高度和倾斜角度，使plane的高度刚好贴合导入的点云地图的地面

<img src="https://cdn.jsdelivr.net/gh/su-ron/image/imgimage-20240512094913537.png" alt="image-20240512094913537" style="zoom: 67%;" />

调整位置

![image-20240512095825849](https://cdn.jsdelivr.net/gh/su-ron/image/imgimage-20240512095825849.png)



3.新建 autoware adasmap

<img src="https://cdn.jsdelivr.net/gh/su-ron/image/imgimage-20240512100322972.png" alt="image-20240512100322972" style="zoom:67%;" />

![image-20240512100441565](https://cdn.jsdelivr.net/gh/su-ron/image/imgimage-20240512100441565.png)



4.标注RoadEdge

RoadEdge是道路边缘

点击右面的add RoadEdge,会出现一个小坐标系，拉动坐标系，就会出现一条线
俯视图：

![image-20240512100953130](https://cdn.jsdelivr.net/gh/su-ron/image/imgimage-20240512100953130.png)

在俯视图画好的时候，调整它的高度

![image-20240512102109229](https://cdn.jsdelivr.net/gh/su-ron/image/imgimage-20240512102109229.png)



5.画whiteline

(步骤跟之前一样，不过白线就相当于实际路况下的划分车道的白线)

![image-20240512103026731](https://cdn.jsdelivr.net/gh/su-ron/image/imgimage-20240512103026731.png)



6.绘制lane（汽车能走的路线）
(画法跟之前一样，区别就是自身的含义)

![image-20240512104237672](https://cdn.jsdelivr.net/gh/su-ron/image/imgimage-20240512104237672.png)

拐角处的弧度绘制：
一般实际场景下的道路在拐弯处都是有一个弧度的，贴合实际的绘制方法就是，在拐弯处先画一条斜线放在那儿
调整完全部线的高度之后，再回来选择这条小斜线(可能有多条，一条一条的来)，点击Subdivision

![image-20240512104813553](https://cdn.jsdelivr.net/gh/su-ron/image/imgimage-20240512104813553.png)

会在线的中间出现两个小坐标系

![image-20240512104503761](https://cdn.jsdelivr.net/gh/su-ron/image/imgimage-20240512104503761.png)

拖动形成弧线后点击Normal Way：

![image-20240512104910462](https://cdn.jsdelivr.net/gh/su-ron/image/imgimage-20240512104910462.png)





切分：

- ​     拐弯处同RoadEdge一样切分，不同的是lane要切成一个一个的小段，因为lane是代表了汽车可以行驶的规则，红绿灯、停车线、信号牌等交通标志都要与lane进行链接

小tips:

- 如果你的地图是环状的封闭的形式(例如围绕某个楼一圈)，此时也不能将一圈的lane画为一整条，因为autoware在加载矢量地图的时候会寻找lane之间的来连接关系，自己跟自己的链接在进行路径规划的时候会使得接口处不可到达。



### 7. stopline

停止线
选择位置，画完之后可以将其与lane,路标/信号灯相连，汽车就知道在这儿有交通标志

![image-20240512105235608](https://cdn.jsdelivr.net/gh/su-ron/image/imgimage-20240512105235608.png)

![image-20240512105847910](https://cdn.jsdelivr.net/gh/su-ron/image/imgimage-20240512105847910.png)

**8.信号灯**

![image-20240512105633666](https://cdn.jsdelivr.net/gh/su-ron/image/imgimage-20240512105633666.png)



**9.保存vector map**

![image-20240512110337920](C:/Users/su/Desktop/%E8%87%AA%E5%8A%A8%E9%A9%BE%E9%A9%B6/image/image-20240512110337920.png)

保存后的文件为：

![image-20240512110647294](C:/Users/su/Desktop/%E8%87%AA%E5%8A%A8%E9%A9%BE%E9%A9%B6/image/image-20240512110647294.png)