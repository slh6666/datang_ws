#include<ros/ros.h>
#include<geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include<iostream>
using namespace std;
 
class Teleop
{
public:
    Teleop();
 
private:
    /* data */
    void callback(const sensor_msgs::Joy::ConstPtr& Joy);
    ros::NodeHandle n;
    ros::Subscriber sub ;
    ros::Publisher pub ;
    double vlinear,vangular;
    int axis_ang,axis_lin,ton;
};
 
Teleop::Teleop()
{
    n.param<int>("axis_linear",axis_lin,1);
    n.param<int>("axis_angular",axis_ang,0);
    n.param<double>("vel_linear",vlinear,0.5);
    n.param<double>("vel_angular",vangular,30);
    n.param<int>("button",ton,5);
    pub = n.advertise<geometry_msgs::Twist>("/cmd_vel",1);  //_mux/input/teleop
    sub = n.subscribe<sensor_msgs::Joy>("joy",10,&Teleop::callback,this);
}
 
void Teleop::callback(const sensor_msgs::Joy::ConstPtr& Joy)
{
    geometry_msgs::Twist v;
    //v.linear.x =Joy->axes[axis_lin]*vlinear;
   // v.angular.z =Joy->axes[axis_ang]*vangular;
  // if(Joy->buttons[ton])
        v.linear.x =(0.5+Joy->axes[axis_lin]+Joy->axes[1])*vlinear;
        v.angular.z =(Joy->axes[axis_ang]+Joy->axes[0])*vangular;
        ROS_INFO("linear:%.3lf   angular:%.3lf",v.linear.x,v.angular.z);
        pub.publish(v);
    
}
 
 
int main(int argc,char** argv)
{
    ros::init(argc, argv, "log");
    Teleop telelog;
    ros::spin();
    return 0;
}
