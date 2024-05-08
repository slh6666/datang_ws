# costmap_generation

## 介绍:

本package用于生成代价地图,其输入包括传感器数据和静态地图.

## 依赖:

- map_server

- turtlebot3_gazebo

- gazebo7.0.0

## 使用:

```
git clone git@47.108.82.182:flztiii/costmap_generation.git
cd workspace
catkin_make
source devel/setup.bash
roslaunch costmap_generation test.launch
```

## 拓展项:

可以修改sensor.yaml文件对输入传感器接口进行调整.sensor.yaml文件示例如下

```
observation_sources: base_scan
base_scan: {topic_name: scan, data_type: LaserScan}
```
