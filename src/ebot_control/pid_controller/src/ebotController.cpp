#include "ebotController.h"
#include "PidControl.h"
#include <tf/transform_datatypes.h>
#include <iostream>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

namespace ebot_controller{

ebotControllerNode::ebotControllerNode(ros::NodeHandle &n){

    tf::TransformBroadcaster br;
    tf::TransformListener listener;
    tf_listener_ = new tf::TransformListener();
    n.param("/ebotController/P", P, 0.5);
    n.param("/ebotController/I", I, 0.4);
    n.param("/ebotController/D", D, 0.0);
    n.param("/ebotController/set_distance",set_distance,0.8);
    target_point_index = 0;
    //sub
    // sub_rear_pose = n.subscribe("/vehicle/rear_pose",1,&ebotControllerNode::pose_cb,this);

    // sub_final_waypoints = n.subscribe("/base_waypoints",1,&ebotControllerNode::lane_cb,this);
    sub_waypoints = n.subscribe("/local_trajectory",1,&ebotControllerNode::waypoints_cb,this);
    // sub_twist_cmd = n.subscribe("/vehicle/cmd",1,&ebotControllerNode::vel_cb,this);

    //test
    sub_rear_pose_test = n.subscribe("/amcl_pose",1,&ebotControllerNode::pose_cb_test,this);

    //pub
    pub_twist_cmd = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    marker_pub =n.advertise<visualization_msgs::Marker>("/target_point",1);
    
    control_timer_ = n.createTimer(ros::Duration(0.02), &ebotControllerNode::ControlCallback,this);
}   

double ebotControllerNode::getdistance(geometry_msgs::Point A,geometry_msgs::Point B)
{   
    double d = sqrt((A.x-B.x)*(A.x-B.x)+(A.y-B.y)*(A.y-B.y));
    return d;
}   
double ebotControllerNode::point_distance(double x1,double y1,double x2, double y2)
{   
    double d =sqrt(pow((x2-x1),2)+pow((y2-y1),2));
    return d;
}

// void ebotControllerNode::pose_cb(const geometry_msgs::PoseStamped& msg)
// {
//     geometry_msgs::PoseStamped rear_pose = msg;
// }

//test
void ebotControllerNode::pose_cb_test(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr msg)
{

    my_odom.pose.position.x = msg->pose.pose.position.x;
    my_odom.pose.position.y = msg->pose.pose.position.y;
    my_odom.pose.orientation = msg->pose.pose.orientation;
                //tf广播
   // tf::TransformBroadcaster br;
   // tf::Transform transform;
   // transform.setOrigin( tf::Vector3(my_odom.pose.pose.position.x, my_odom.pose.pose.position.y, 0) );
   // transform.setRotation( tf::Quaternion(msg->pose.pose.orientation.x,msg->pose.pose.orientation.y,msg->pose.pose.orientation.z,msg->pose.pose.orientation.w) );
   // br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/odom","/base_link"));
   // std::cout<<"CurrentX:"<<my_odom.pose.pose.position.x<<"CurrentY:"<<my_odom.pose.pose.position.y<<std::endl;
//   std::cout <<"expect velocity twist_aux.linear.x:"<<twist_aux.linear.x << std::endl;
//   if(radar_data < 65 && radar_data >= 0 )//如果不符合启动条件，即接受到超声数据且距离障碍在阈值距离以上。注意：由于车有惯性，真实的阈值=阈值（这里为60）- 惯性刹车距离
//   {
//     transition=(twist_aux.linear.x>0) ? 0 : (twist_aux.linear.x*1000);//设置被障碍物阻挡时，可以后退但不能前进
//     std::cout <<"actual velocity chassis_data:"<<chassis_data << std::endl;
//   }
//   else
//   {
//     transition = twist_aux.linear.x*1000; //将浮点数放大一千倍，简化传输
//   }
}
void ebotControllerNode::waypoints_cb(const trajectory_msgs::JointTrajectory::ConstPtr &recieved_curve) //读取路径
{   
    final_waypoints.header.frame_id = recieved_curve->header.frame_id;
    final_waypoints.header.stamp = ros::Time::now();
    final_waypoints.poses.resize(recieved_curve->points.size());
    // velocity_.waypoints.resize(recieved_curve->points.size());
    for (size_t i = 0;i < recieved_curve->points.size(); i++)
    {
        
        final_waypoints.poses[i].pose.position.x = recieved_curve->points[i].positions[0];
        final_waypoints.poses[i].pose.position.y = recieved_curve->points[i].positions[1];
        final_waypoints.poses[i].pose.position.z = recieved_curve->points[i].positions[2];//chao xiang jiao
        
        //加上xy的分速度算出来的速度先占用四元数数组
        final_waypoints.poses[i].pose.orientation.x = sqrt(pow(recieved_curve->points[i].velocities[0],2)+pow(recieved_curve->points[i].velocities[1],2));



        // if (final_waypoints.poses[i].pose.position.z < 0)
        // {
        //     final_waypoints.poses[i].pose.position.z = final_waypoints.poses[i].pose.position.z + 2*3.1415926;
        // }
        // velocity_.waypoints[i].twist.twist.linear.x = recieved_curve->points[i].velocities[0];
        // velocity_.waypoints[i].twist.twist.linear.y = recieved_curve->points[i].velocities[1];
        // velocity_.waypoints[i].twist.twist.linear.x = cos(final_waypoints.poses[i].pose.position.z)*velocity_.waypoints[i].twist.twist.linear.x +sin(final_waypoints.poses[i].pose.position.z)*velocity_.waypoints[i].twist.twist.linear.y;
        
    }

    //下面的速度计算并没有被使用，路径中并没有速度信息，cmdpublish之前同意linear.x置0.5
    // velocity.linear.x = cos(final_waypoints.poses[target_point_index].pose.position.z)*recieved_curve->points[target_point_index].velocities[0] 
    //                     + sin(final_waypoints.poses[target_point_index].pose.position.z)*recieved_curve->points[target_point_index].positions[1];
    // // velocity.linear.x = velocity_.waypoints[target_point_index].twist.twist.linear.x;
    // if (velocity.linear.x > 0.5)
    // {
    //     velocity.linear.x = 0.5;
    // }
    // if (velocity.linear.x < -0.5)
    // {
    //     velocity.linear.x = -0.5;
    // }
    // // velocity.linear.x = velocity.linear.x; //小车实际速度负值为前进方向
    // std::cout<< "velocity.linear.x:" << velocity.linear.x << std::endl;

}

void ebotControllerNode::nearest_point_index(const nav_msgs::Path &msg)  //求取目标参考点索引
{

    currentX = my_odom.pose.position.x;//当前车辆位置坐标
    currentY = my_odom.pose.position.y;
    geometry_msgs::Point current_pose_;
    current_pose_.x = currentX;
    current_pose_.y = currentY;

    //double r = r_distance*r_distance;
    len = final_waypoints.poses.size(); //坐标点数量
    // nearest_point = 0;
    // std::cout<< "len:" << len << std::endl;
    //std::cout<< final_waypoints.poses[0].pose.position.x << std::endl;
    // double min_distance = 0.1;
   // int i = 0;

    if (final_waypoints.poses.size()>0)
    {

        double min_distance = sqrt(pow((currentX - final_waypoints.poses[0].pose.position.x),2)+pow((currentY-final_waypoints.poses[0].pose.position.y),2));
        
        
        for (int num = 0; num<len-1; num++) 
        {
            geometry_msgs::Point select_pose;
            select_pose.x = final_waypoints.poses[num].pose.position.x;
            select_pose.y = final_waypoints.poses[num].pose.position.y;
            double current_dis = getdistance(current_pose_,select_pose);//车辆位置和选取点距离
           // std::cout<< "××××××××××××current_pose_:" << current_pose_.x <<"," << current_pose_.y << std::endl;
            // std::cout<< "select_pose:" << select_pose.x << "," <<select_pose.y << std::endl;
            // std::cout<< "#############min_distance:" << min_distance << std::endl;
            // std::cout<< "current_dis:" << current_dis << std::endl;        
            if(current_dis <  min_distance)
            {
                min_distance = current_dis;

                nearest_point = num;
                //i = num;
     
            }    
            // std::cout<< "nearest_point:" << final_waypoints.poses[nearest_point].pose.position.x << ","<< final_waypoints.poses[nearest_point].pose.position.y << std::endl;
             
        }
        // target_point_index = nearest_point;

        int  search_index = nearest_point;
        // std::cout << "initail_search_index:" << search_index << std::endl;
 
        for (double lad = 0; search_index < len-1; search_index++)
        {

            add_distance = point_distance(final_waypoints.poses[search_index].pose.position.x,final_waypoints.poses[search_index].pose.position.y,
                                            final_waypoints.poses[search_index+1].pose.position.x,final_waypoints.poses[search_index+1].pose.position.y );
            lad += add_distance;
            // std::cout << lad << std::endl;

            if (lad > set_distance)
            {
                target_point_index = search_index;
                break; 
            }

            if(search_index == len - 2) target_point_index = len - 1; 
        }
        // target_point_index += 30;

        // std::cout<< "target_point_index:" <<target_point_index << std::endl;
        // std::cout <<  search_index << " "<< len << "" <<std::endl;
        // std::cout<< "target_point_index:" << target_point_index <<":"<< final_waypoints.poses[target_point_index].pose.position.x << ","<< final_waypoints.poses[target_point_index].pose.position.y << std::endl;
        // std::cout<< "nearest_point_index:" << nearest_point <<":"<< final_waypoints.poses[nearest_point].pose.position.x << ","<< final_waypoints.poses[nearest_point].pose.position.y << std::endl;
        // std::cout<< "target_point_index_yaw" << final_waypoints.poses[target_point_index].pose.position.z << std::endl;
        std::cout << "my_odom.yaw:" << QuaternionToYaw_2D(my_odom.pose.orientation) <<std::endl;
 

    }

}


// speed pid control
void ebotControllerNode::ControlCallback(const ros::TimerEvent& event){
    if (final_waypoints.poses.size()>0)
    {
        control_period_ = 0.02;
        //1为弧度制，以前为65是角度制-hejintao
        max_steering_theta =  1;
        min_steering_theta = -1;

        static PidControl PID(P,I,D,min_steering_theta,max_steering_theta);//为使误差累积，实例化对象为静态对象
        std::cout <<"pid_last_err:"<<PID.get_last_error() << std::endl;
        
        double control_period = control_period_;
        double steering_theta = steering_theta_;
        
        // PID.setParams(P,I,D,min_steering_theta,max_steering_theta);

        nearest_point_index(final_waypoints);

        //目标参考点位置
        targetX = final_waypoints.poses[target_point_index].pose.position.x;
        targetY = final_waypoints.poses[target_point_index].pose.position.y;

        

        // get_nearest_dis();
        // steering_theta = P.step(intercept,control_period);

        //监听
        tf::TransformListener listener;
        geometry_msgs::PointStamped base_link_point;
        base_link_point.header.frame_id = "/map";
        base_link_point.header.stamp = ros::Time();
            
        //base_link_point检测点获取，即目标点对车辆坐标系的转换
        // std::cout<<"targetx:"<<targetX<<"targetY:"<<targetY<<std::endl;
        base_link_point.point.x = targetX;  
        base_link_point.point.y = targetY;
        base_link_point.point.z = 0.0;
        geometry_msgs::PointStamped vehicle_frame_point;
        try{
            // 等待获取监听信息并获取转换坐标信息
            listener.waitForTransform("/base_link", "/map", ros::Time(0), ros::Duration(3.0));
            listener.transformPoint("/base_link", base_link_point, vehicle_frame_point);

            // intercept = fabs(.point.y);  //转换后的相对y偏差
            intercept = fabs((0.7*final_waypoints.poses[target_point_index].pose.position.z +
             0.1*final_waypoints.poses[nearest_point].pose.position.z +
              0.2*final_waypoints.poses[(nearest_point + target_point_index) / 2].pose.position.z)- QuaternionToYaw_2D(my_odom.pose.orientation));

            std::cout << "inal_waypoints.poses[target_point_index].pose.position.z:" << final_waypoints.poses[target_point_index].pose.position.z <<std::endl;
            std::cout << "final_waypoints.poses[nearest_point].pose.position.z:" << final_waypoints.poses[nearest_point].pose.position.z <<std::endl;
            std::cout << "final_waypoints.poses[(nearest_point + target_point_index) / 2].pose.position.z:" << final_waypoints.poses[(nearest_point + target_point_index) / 2].pose.position.z <<std::endl;
            std::cout << "intercept" << intercept <<std::endl;

            // ROS_INFO("map: (%.2f, %.2f. %.2f) -----> base_link: (%.2f, %.2f, %.2f) at time %.2f",
            //     base_link_point.point.x, base_link_point.point.y, base_link_point.point.z,
            //     vehicle_frame_point.point.x, vehicle_frame_point.point.y, vehicle_frame_point.point.z, vehicle_frame_point.header.stamp.toSec());
            // std::cout<<"vehicle_frame_point.point.y:"<< vehicle_frame_point.point.y << std::endl;
            // std::cout<<"intercept:"<<intercept<<std::endl;
            // std::cout<<"vehicle_frame_point.point.y:"<<vehicle_frame_point.point.y<<std::endl;
        }
        catch(tf::TransformException& ex){
            ROS_ERROR("Received an exception trying to transform a point from \"base_laser\" to \"base_link\": %s", ex.what());
        }


        // steering_theta = PID.step(intercept,control_period) - QuaternionToYaw_2D(my_odom.pose.orientation);
   
        steering_theta = PID.step(intercept,control_period);


        // 判断车辆及轨迹的位置关系
        if (vehicle_frame_point.point.y > 0)
        {
            steering_theta = steering_theta;
        
        }
        else
        {
            steering_theta = -steering_theta; 
        }

		//1.74为弧度制，以前为100是角度制-hejintao
        if (steering_theta > 1.74)
        {
                steering_theta = 1.74;
        }
            
        else if(steering_theta < -1.74)
        {
                steering_theta = -1.74;
        }


        std::cout<<"steering_theta:"<<steering_theta<<std::endl;
        // show target point 
        visualization_msgs::Marker point;
        point.header.frame_id = "map";
        point.header.stamp = ros::Time();
        point.pose.position.x = targetX;
        point.pose.position.y = targetY;
        // point.pose.position.x = currentX;
        // point.pose.position.y = currentY;
        point.color.g =1.0f;
        point.color.a =1.0;
        point.scale.x = 0.2;
        point.scale.y = 0.2;
        point.scale.z = 0.2;
        marker_pub.publish(point);

        geometry_msgs::Twist cmd_twist;

        // cmd_twist.linear.x = 0.5;
        cmd_twist.linear.x = 0.5*0.5 + 0.5 * (0.6*final_waypoints.poses[nearest_point].pose.orientation.x +
                            0.3*final_waypoints.poses[target_point_index].pose.orientation.x+
                            0.1*final_waypoints.poses[len - 1].pose.orientation.x);
        //speed --
        
        double set_stop_distance = 1;
        double stop_distance = point_distance(currentX,currentY,final_waypoints.poses.back().pose.position.x,final_waypoints.poses.back().pose.position.y);
        // ROS_INFO("to jubu back dis is %f",stop_distance);
        if (stop_distance < set_stop_distance)
        {
            cmd_twist.linear.x = 0.3*cmd_twist.linear.x; 

            if (stop_distance  <= 0.3)
            {
                cmd_twist.linear.x = 0;
            }
        }
        cmd_twist.angular.z = stop_distance  <= 0.3 ? 0 : steering_theta;
        pub_twist_cmd.publish(cmd_twist);
    }
    else{
        ROS_INFO("final_waypoints.poses.size()<=0");
        geometry_msgs::Twist cmd_twist;
        cmd_twist.linear.x = 0;
        cmd_twist.angular.z = 0;
        pub_twist_cmd.publish(cmd_twist);
    }
}

}
