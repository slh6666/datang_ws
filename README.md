# 西安交通大学智能车团队&大唐移动合作开发无人车项目
用途为高校无人车教育，功能包含定位规划控制、动态避障、双目相机探测、超声波预警急停。  
![image](https://github.com/slh6666/datang_ws/blob/main/wheeltec_robot.jpg)  
![image](https://github.com/slh6666/datang_ws/blob/main/mymap_test4.pgm)  
![image](https://github.com/slh6666/datang_ws/blob/main/puhuo.PNG)
## 0.底盘软件功能包
&emsp;&emsp;功能包`turn_on_wheeltec_robot`是轮趣无人车底盘与上层ROS主控交互的软件包，主要负责串口读取底层主控stm32的底盘相关数据，
其源码文件位于`/src/turn_on_wheeltec_robot/src/wheeltec_robot.cpp`。在其已有功能基础上，我们开发了超声波数据读取、
底盘速度数据读取、行人及红绿灯数据传输并实现了利用超声波的急停避障功能。
我们采用以下命令启动底盘程序：  
```
roslaunch turn_on_wheeltec_robot base_serial.launch
```
## 1.里程计功能包
&emsp;&emsp;轮趣无人车原本是将里程计集成到底盘功能包中，采用IMU和自行车里程计融合定位，但是实际使用发现IMU数据有误，
造成了里程计累积时漂移，故将里程计独立为一个功能包，注意此时使用的只是单纯的自行车里程计。功能包`imu_wheeltec_odometry`
是负责里程计计算的功能包，启动方式为：  
```
roslaunch imu_wheeltec_odometry imu_odometry.launch
```
但为了方便起见，我们将该条命令与定位的slam命令一起启动，于是只需要启动如下命令即可：
```
roslaunch ebot_bring slam.launch
```
## 3.自主导航
&emsp;&emsp;其他上层的定位规划控制算法均与小黄车一致，启动流程参见小黄车项目：https://github.com/slh6666/datang_ws
