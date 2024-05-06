#include "ebotController.h"
#include <ros/ros.h>
#include <signal.h>




ros::Publisher  pub_stop_twist_cmd;

geometry_msgs::Twist cmd_twist;

ebot_controller::ebotControllerNode* node;


void mySigintHandler(int sig)
   {
   // Do some custom action.

    // For example, publish a stop message to some other nodes.
      
    cmd_twist.linear.x = 0;
    // cmd_twist.linear.x = 0.5;
    cmd_twist.angular.z = 0;
    pub_stop_twist_cmd.publish(cmd_twist);

    std::cout<<"ros shut down!"<<std::endl;  
   // All the default sigint handler does is call shutdown()
    ros::shutdown();
  }

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ebot_controller");
  ros::NodeHandle n;
  signal(SIGINT, mySigintHandler);

  pub_stop_twist_cmd = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

  node = new ebot_controller::ebotControllerNode(n);

  while(ros::ok()){

    ros::spin();
    
  }
  return 0;
}