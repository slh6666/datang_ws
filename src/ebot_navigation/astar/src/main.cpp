#include <iostream>
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <opencv2/opencv.hpp>
#include "Astar.h"
#include "OccMapTransform.h"
#include <std_msgs/Int8.h>
#include <nav_msgs/Odometry.h>


using namespace cv;
using namespace std;


//-------------------------------- Global variables ---------------------------------//
// Subscriber
ros::Subscriber map_sub;
ros::Subscriber startPoint_sub;
ros::Subscriber targetPoint_sub;
// Publisher
ros::Publisher mask_pub;
ros::Publisher path_pub;
ros::Publisher trap_pub;

// Object
nav_msgs::OccupancyGrid OccGridMask;
nav_msgs::Path path;
pathplanning::AstarConfig config;
pathplanning::Astar astar;
OccupancyGridParam OccGridParam;
Point startPoint, targetPoint;

// Parameter
double InflateRadius;
bool map_flag;
bool startpoint_flag;
bool targetpoint_flag;
bool start_flag;
int rate;

//-------------------------------- Callback function ---------------------------------//
void MapCallback(const nav_msgs::OccupancyGrid& msg)
{
    // Get parameter
    OccGridParam.GetOccupancyGridParam(msg);

    // Get map
    int height = OccGridParam.height;
    int width = OccGridParam.width;
    int OccProb;
    Mat Map(height, width, CV_8UC1);
    for(int i=0;i<height;i++)
    {
        for(int j=0;j<width;j++)
        {
            OccProb = msg.data[i * width + j];
            OccProb = (OccProb < 0) ? 100 : OccProb; // set Unknown to 0
            // The origin of the OccGrid is on the bottom left corner of the map
            Map.at<uchar>(height-i-1, j) = 255 - round(OccProb * 255.0 / 100.0);
        }
    }

    // Initial Astar
    Mat Mask;
    config.InflateRadius = round(InflateRadius / OccGridParam.resolution);
    astar.InitAstar(Map, Mask, config);

    // Publish Mask
    OccGridMask.header.stamp = ros::Time::now();
    OccGridMask.header.frame_id = "map";
    OccGridMask.info = msg.info;
    OccGridMask.data.clear();
    for(int i=0;i<height;i++)
    {
        for(int j=0;j<width;j++)
        {
            OccProb = Mask.at<uchar>(height-i-1, j) * 255;
            OccGridMask.data.push_back(OccProb);
        }
    }

    // Set flag
    map_flag = true;
    startpoint_flag = false;
    targetpoint_flag = false;
}

void StartPointCallback(const geometry_msgs::PoseWithCovarianceStamped& msg)
{
    if(map_flag != true){
        std::cout << "wait for map data" << std::endl;
        return;
    }
    //cout<<"1"<<endl;
    Point2d src_point = Point2d(msg.pose.pose.position.x, msg.pose.pose.position.y);
    //cout<<"2"<<endl;
    OccGridParam.Map2ImageTransform(src_point, startPoint);

    // Set flag
    startpoint_flag = true;
    //printf("true1");
    //cout<<"3"<<endl;
    if(map_flag && startpoint_flag && targetpoint_flag)
    {
        start_flag = true;
    }

//    ROS_INFO("startPoint: %f %f %d %d", msg.pose.pose.position.x, msg.pose.pose.position.y,
//             startPoint.x, startPoint.y);
}

// void StartPointCallback(const nav_msgs::OdometryConstPtr& msg)
// {
//     Point2d src_point = Point2d(msg->pose.pose.position.x, msg->pose.pose.position.y);
//     OccGridParam.Map2ImageTransform(src_point, startPoint);

//     // Set flag
//     startpoint_flag = true;
//     if(map_flag && startpoint_flag && targetpoint_flag)
//     {
//         start_flag = true;
//     }
// }

// void OdomCallback(const nav_msgs::OdometryConstPtr& msg)
// {
//     current_velocity = msg->twist.twist;
//     odom_updated = true;
// }

void TargetPointtCallback(const geometry_msgs::PoseStamped& msg)
{
    //cout<<"1'"<<endl;	
    Point2d src_point = Point2d(msg.pose.position.x, msg.pose.position.y);
    OccGridParam.Map2ImageTransform(src_point, targetPoint);

    // Set flag
    targetpoint_flag = true;
    //cout<<"2'"<<endl;
    // if(map_flag && startpoint_flag && targetpoint_flag)
    // {
    //     start_flag = true;
    // }
        //cout<<"1''"<<endl;

    // if(start_flag)
    // {
    //     double start_time = ros::Time::now().toSec();
    //     // Start planning path
    //     vector<Point> PathList;
    //     astar.PathPlanning(startPoint, targetPoint, PathList);
    //     if(!PathList.empty())
    //     {
    //         path.header.stamp = ros::Time::now();
    //         path.header.frame_id = "map";
    //         path.poses.clear();
	//     int i = 0;
    //         while(i<PathList.size())
    //         {
    //             Point2d dst_point;
    //             OccGridParam.Image2MapTransform(PathList[i], dst_point);
    //             geometry_msgs::PoseStamped pose_stamped;
    //             pose_stamped.header.stamp = ros::Time::now();
    //             pose_stamped.header.frame_id = "map";
    //             pose_stamped.pose.position.x = dst_point.x;
    //             pose_stamped.pose.position.y = dst_point.y;
    //             pose_stamped.pose.position.z = 0;
    //             path.poses.push_back(pose_stamped);
	// 	i++;
			
    //         }
    //         path_pub.publish(path);
    //         double end_time = ros::Time::now().toSec();

    //         ROS_INFO("Find a valid path successfully! Use %f s", end_time - start_time);
    //     }
    //     else
    //     {
    //         ROS_ERROR("Can not find a valid path");
    //     }

    //         // Set flag
    //     start_flag = false;
    // }

    //     if(map_flag)
    //     {
    //         mask_pub.publish(OccGridMask);
    //     }
//    ROS_INFO("targetPoint: %f %f %d %d", msg.pose.position.x, msg.pose.position.y,
//             targetPoint.x, targetPoint.y);
}

//-------------------------------- Main function ---------------------------------//
int main(int argc, char * argv[])
{
    //  Initial node
    ros::init(argc, argv, "astar");
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");
    ROS_INFO("Start astar node!\n");

    // Initial variables
    map_flag = false;
    startpoint_flag = false;
    targetpoint_flag = false;
    start_flag = false;

    // Parameter
    nh_priv.param<bool>("Euclidean", config.Euclidean, true);
    nh_priv.param<int>("OccupyThresh", config.OccupyThresh, -1);
    nh_priv.param<double>("InflateRadius", InflateRadius, -1);
    nh_priv.param<int>("rate", rate, 10);

    // Subscribe topics
    map_sub = nh.subscribe("map", 10, MapCallback);
    // startPoint_sub = nh.subscribe("/odom", 10, StartPointCallback);
    startPoint_sub = nh.subscribe("/amcl_pose", 10, StartPointCallback);
    targetPoint_sub = nh.subscribe("move_base_simple/goal", 10, TargetPointtCallback);
    //ros::spinOnce();
    //ros::spin();
    // Advertise topics
    mask_pub = nh.advertise<nav_msgs::OccupancyGrid>("mask", 1);
    path_pub = nh.advertise<nav_msgs::Path>("/global_path", 10);
    trap_pub = nh.advertise<std_msgs::Int8>("/is_trapping", 10);

    // Loop and wait for callback
    ros::Rate loop_rate(rate);
    while(ros::ok())
    {
        ros::spinOnce();
        // static int count = 0;
        if(map_flag && startpoint_flag && targetpoint_flag)
        {
            start_flag = true;
            startpoint_flag = false;
            targetpoint_flag = false;
        }
	// cout<<"1''"<<endl;
        if(start_flag)
        {
            double start_time = ros::Time::now().toSec();
            // Start planning path
            // static vector<Point> PathList;

            vector<Point> PathList;
            
            // static Point tem_point;

            // if(count == 0){
            // astar.PathPlanning(startPoint, targetPoint, PathList);
            // tem_point = PathList.back();
            // count++;
            // }else if (targetPoint != tem_point){
            // vector<Point> PathAdd;
            // astar.PathPlanning(tem_point, targetPoint, PathAdd);
            // for(auto it = PathAdd.begin();it != PathAdd.end(); it++){
            //     PathList.push_back(*it);
            // }
            // tem_point = PathList.back();
            // }

            if(!PathList.empty())
            {
                path.header.stamp = ros::Time::now();
                path.header.frame_id = "map";
                path.poses.clear();
                std::cout << PathList.size() <<std::endl;
                for(int i=0;i<PathList.size();i++)
                {
                    Point2d dst_point;
                    OccGridParam.Image2MapTransform(PathList[i], dst_point);

                    geometry_msgs::PoseStamped pose_stamped;
                    pose_stamped.header.stamp = ros::Time::now();
                    pose_stamped.header.frame_id = "map";
                    pose_stamped.pose.position.x = dst_point.x;
                    pose_stamped.pose.position.y = dst_point.y;
                    pose_stamped.pose.position.z = 0;
                    path.poses.push_back(pose_stamped);
                }
                path_pub.publish(path);
                double end_time = ros::Time::now().toSec();
                std_msgs::Int8 trap_flag;
                trap_flag.data = 0;
                trap_pub.publish(trap_flag);
                ROS_INFO("Find a valid path successfully! Use %f s", end_time - start_time);
                // std::cout << PathList.back().x <<" "<<PathList.back().y<<std::endl;
                // std::cout << tem_point.x << " " << tem_point.y << std::endl;
                // count ++;
            }
            else
            {
                std_msgs::Int8 trap_flag;
                trap_flag.data = 1;
                trap_pub.publish(trap_flag);
                ROS_ERROR("Can not find a valid path");
            }

            // Set flag
            start_flag = false;  //待定
        }

        if(map_flag)
        {
            mask_pub.publish(OccGridMask);
        }

        loop_rate.sleep();
        // ros::spin();
    }


    return 0;
}
