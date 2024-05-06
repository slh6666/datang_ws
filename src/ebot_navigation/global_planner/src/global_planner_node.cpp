#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>

int main(int argc, char ** argv){

    ros::init(argc, argv, "global_planner_node");
    ros::NodeHandle nh;
    ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("global_path", 10);

    

    std::vector<float> x_set;
    std::vector<float> y_set;

    nh.param("/global_planner_node/x_set", x_set, {0.0, 10.0, 20.5});
    nh.param("/global_planner_node/y_set", y_set, {0.0, -6.0, 5.0});

    ros::Rate loop_rate(10);
    while(ros::ok()){
        ros::spinOnce();

        // 输出可视化
        nav_msgs::Path global_path;
        global_path.header.frame_id = "map";
        global_path.header.stamp = ros::Time::now();
        for (size_t i = 0; i < x_set.size(); i++) {
            geometry_msgs::PoseStamped pose;
            pose.header = global_path.header;
            pose.pose.position.x = x_set[i];
            pose.pose.position.y = y_set[i];
            global_path.poses.push_back(pose);
        }
        path_pub.publish(global_path);

        loop_rate.sleep();
    }


    return 0;
}