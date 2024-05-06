#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Vector3.h>


class SubscribeAndPublish
{
public:
  SubscribeAndPublish()
  {
     x_ = 0.0;
     y_ = 0.0;
     th_ = 0.0;
 
     vx_ = 0.0;
     vy_ = 0.0;
     vth_ = 0.0;
     current_time_ = ros::Time::now();
     last_time_ = ros::Time::now();
    //Topic you want to publish
    pub_ = n_.advertise<nav_msgs::Odometry>("/bicycle_wheel_odometry/odom", 5);
 
    //Topic you want to subscribe
    sub_ = n_.subscribe("/robot/chassis_status", 1, &SubscribeAndPublish::callback, this);
  }
 
  void callback(const geometry_msgs::TwistStamped::ConstPtr& msg)
  {

    //nav_msgs::Odometry output;
    //.... do something with the input and generate the output...
    current_time_ = ros::Time::now();
    vx_ = msg->twist.linear.x;
    float yaw = msg->twist.angular.z;
    yaw = yaw / 180 * 3.1415926*0.75;
    float L = 0.313;
    float r =  L / tan(yaw);
    // r = r + 0.095;
    
    // double dt = 0.02;
    vth_ = vx_ / r;
    vy_= 0.0;   

    double dt = (current_time_ - last_time_).toSec();
    
    double delta_x = (vx_ * cos(th_) - vy_ * sin(th_)) * dt;
    double delta_y = (vx_ * sin(th_) + vy_ * cos(th_)) * dt;
    double delta_th = vth_ * dt;
 
    x_ += delta_x;
    y_ += delta_y;
    th_ += delta_th;
 
    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th_);



 
    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time_;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_footprint";
 
    odom_trans.transform.translation.x = x_;
    odom_trans.transform.translation.y = y_;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;
 
    //send the transform
    odom_broadcaster_.sendTransform(odom_trans);

 

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time_;
    odom.header.frame_id = "odom";
 
    //set the position
    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;
 
    //set the velocity
    odom.child_frame_id = "base_footprint";
    odom.twist.twist.linear.x = vx_;
    odom.twist.twist.linear.y = vy_;
    odom.twist.twist.angular.z = vth_; //jiasudu

    odom.pose.covariance[0] = 0.1;
    odom.pose.covariance[7] = 0.1;
    odom.pose.covariance[14] = 0.1;
    odom.pose.covariance[21] = 0.0025;
    odom.pose.covariance[28] = 0.0025;
    odom.pose.covariance[25] = 0.0025;

    odom.twist.covariance[0] = 0.25;
    odom.twist.covariance[7] = 0.25;
    odom.twist.covariance[14] = 0.1;
    odom.twist.covariance[21] = 0.02;
    odom.twist.covariance[28] = 0.02;
    odom.twist.covariance[25] = 0.02;
 
    //publish the message
    pub_.publish(odom);
 
    last_time_ = current_time_;
 
  }
 
private:
// 
  ros::NodeHandle n_; 
  ros::Publisher pub_;
  ros::Subscriber sub_;
  ros::Time current_time_, last_time_;
  tf::TransformBroadcaster odom_broadcaster_;


  double x_ ;
  double y_ ;
  double th_ ;
 
  double vx_;
  double vy_ ;
  double vth_ ;
 
};//End of class SubscribeAndPublish
 


int main(int argc, char **argv)
{

  //Initiate ROS
  ros::init(argc, argv, "bicycle_wheel_odometry");
 
  //Create an object of class SubscribeAndPublish that will take care of everything
  SubscribeAndPublish SAPObject;

  ros::Rate loop_rate(50);

  while(ros::ok()){

        loop_rate.sleep();

        ros::spinOnce();  
  }

  return 0;
}

