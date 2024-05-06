#include "local_planner/frenet_trajectory.hpp"
#include <signal.h>

void mySigintHandler(int sig)
   {
   // Do some custom action.
    // For example, publish a stop message to some other nodes.
    std::cout<<"ros shut down!"<<std::endl;  
   // All the default sigint handler does is call shutdown()
    ros::shutdown();
  }

int main(int argc, char **argv){

    ros::init(argc, argv, "local_planner_node");

    ros::NodeHandle nh("~");
    //signal(SIGINT, mySigintHandler);

    FrenetPathPlanner frenet_path_planner(nh);

    ros::Rate loop_rate(10);

    std::cout<<"wait for global path"<<std::endl;
        
    ros::topic::waitForMessage<nav_msgs::Path>("/global_path");

    std::cout<<"wait for costmap"<<std::endl;
        
    // ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("/costmap");
    
    while(ros::ok()){

        loop_rate.sleep();
        
        ros::Time begin = ros::Time::now();

        frenet_path_planner.initPlanner();

        ros::spinOnce();

        frenet_path_planner.referenceCurvePlanning();

        frenet_path_planner.ReferenceCurvePub();
        //开始进行规划
        frenet_path_planner.Planning();

        frenet_path_planner.TrajectoryPub();

        ros::Time end = ros::Time::now();

        std::cout<<"node planning time = "<<(end - begin).toSec()<<"s"<<std::endl;  //输出时间（单位：ｓ）


    }

    return 0;
}