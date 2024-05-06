#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/TwistStamped.h>
ros::Publisher control_pub;
geometry_msgs::TwistStamped control_msg;
void joy_callback(sensor_msgs::JoyConstPtr joy_msgs)
{
    float motor,steering;
    motor = (float)(-joy_msgs->axes[1]);
    steering =(float) (-joy_msgs->axes[3]*50)*100*0.009657;
    control_msg.twist.linear.x=motor;
    control_msg.twist.angular.x=steering;
    control_pub.publish(control_msg);
}

int main(int argc,char **argv)
{
    ros::init(argc,argv,"joy2control");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    std::string joy_topic;
    std::string control_topic;
    nh_private.param<std::string>("joy_topic",joy_topic,"/joy");
    nh_private.param<std::string>("control_topic",control_topic,"/robot/control");

    //订阅手柄数据
    ros::Subscriber joy_sub = nh.subscribe<sensor_msgs::Joy>(joy_topic,100,joy_callback);
    //发送解析后对车辆速度朝向的控制指令
    control_pub = nh.advertise<geometry_msgs::TwistStamped>(control_topic,100);

    ros::spin();
    return 0;
}
