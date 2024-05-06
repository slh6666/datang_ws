#include "PidControl.h"
#include <ros/ros.h>
#include <string>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
// #include <styx_msgs/Lane.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
namespace ebot_controller {
class ebotControllerNode{ 
public:
    ebotControllerNode(ros::NodeHandle &n);

private:
    void ControlCallback(const ros::TimerEvent& event);
    void pose_cb(const geometry_msgs::PoseStamped msg);
    void lane_cb(const nav_msgs::Path::ConstPtr &msg);  //enable this to use global path 
    void reference_point(const nav_msgs::Path msg);
    // void vel_cb(const geometry_msgs::Twist msg);
    void nearest_point_index(const nav_msgs::Path &msg);
    double getdistance(geometry_msgs::Point A,geometry_msgs::Point B);
    double point_distance(double x1,double y1,double x2, double y2);
    void get_nearest_dis();
    void pose_cb_test(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr msg);
    void tf_baseline(const geometry_msgs::PointStamped base_point);
    void waypoints_cb(const trajectory_msgs::JointTrajectory::ConstPtr &msg);

    ros::Publisher  pub_twist_cmd;
    ros::Publisher  marker_pub;        
    ros::Subscriber sub_rear_pose;
    ros::Subscriber sub_velocity;
    ros::Subscriber sub_final_waypoints;
    ros::Subscriber sub_twist_cmd;
    ros::Subscriber sub_waypoints;
    ros::Timer control_timer_;
    ros::Subscriber sub_rear_pose_test;
    double add_distance;
    double set_distance;
    double lad;
    double intercept;
    double steering_theta_;
    double control_period_;
    // double velocity_;
    double steering_theta;
    int target_point_index;
    int nearest_point;
    double max_steering_theta;
    double min_steering_theta;
    double array[100][100];
    double currentX;
    double currentY;
    double targetX;
    double targetY;
    int len;
    double P;
    double I;
    double D;
    PidControl PID;
    geometry_msgs::Twist velocity;
    geometry_msgs::PoseStamped rear_pose;
    nav_msgs::Path final_waypoints;
    geometry_msgs::Point current_pose_;
    geometry_msgs::PoseWithCovarianceStamped my_odom;
    nav_msgs::Path g_curve_;
    tf::TransformListener* tf_listener_;
    // styx_msgs::Lane velocity_;

    };
}
