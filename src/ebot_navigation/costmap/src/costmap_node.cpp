#include "costmap/costmap_node.hpp"

int main(int argc, char **argv) {
    ros::init(argc, argv, "costmap_generation_node");
    // 创建代价地图生成类
    std::shared_ptr<CostmapGeneration> costmap_generator(new CostmapGeneration());
    // 发布输出地图
    ros::NodeHandle nh("~");
    std::string total_cloud_vis_topic;
    nh.param("total_cloud_vis_topic", total_cloud_vis_topic, std::string("/costmap_generation/total_cloud_vis"));
    ros::Publisher total_cloud_vis = nh.advertise<sensor_msgs::PointCloud2>(total_cloud_vis_topic, 10);
    ros::Rate loop_rate(10);
    while(ros::ok()) {
        ros::spinOnce();
        // 发布点云可视化
        pcl::PointCloud<pcl::PointXYZ> pcl_cloud = costmap_generator->getData();
        sensor_msgs::PointCloud2 sensor_cloud;
        pcl::toROSMsg(pcl_cloud, sensor_cloud);
        sensor_cloud.header.frame_id = "world";
        sensor_cloud.header.stamp = ros::Time::now();
        total_cloud_vis.publish(sensor_cloud);
        // 发布代价地图
        costmap_generator->publishCostMap();
        loop_rate.sleep();
    }
    return 0;
}
