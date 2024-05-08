#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <laser_geometry/laser_geometry.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/io/pcd_io.h>
#include <pcl_ros/transforms.h>
#include <nav_msgs/OccupancyGrid.h>
#include <string>
#include <vector>
#include <mutex>

struct Point2f
{
    double x_;
    double y_;
};

struct CurvePoint
{
    Point2f position_;		///< Position of point on curve, Unit = meter/centimeter/...
    double theta_;			///< Tengential orientation of point, rad
    double kappa_;			///< Curvature of point, rad/Unit,positive for anticlockwise & negative for clockwise
};

// 栅格地图类
class GridMap {
 public:
    // 构造函数
    GridMap(const nav_msgs::OccupancyGrid &occupancy_grid){
        for (auto meta_data: occupancy_grid.data) {
            if (meta_data > 50) {
                this->data_.push_back(true);
            } else {
                this->data_.push_back(false);
            }
        }
        this->width_ = occupancy_grid.info.width;
        this->height_ = occupancy_grid.info.height;
        this->resolution_ = occupancy_grid.info.resolution;
        this->root_x_ = occupancy_grid.info.origin.position.x;
        this->root_y_ = occupancy_grid.info.origin.position.y;
        this->root_theta_ = 2.0 * atan2(occupancy_grid.info.origin.orientation.z, occupancy_grid.info.origin.orientation.w);
    };

    GridMap() {};

    // 析构函数
    ~GridMap() {};

    // 修改分辨率
    void changeResolution(double new_resolution) {
        // 计算比例系数
        double ratio = new_resolution / this->resolution_;
        // 计算新的宽和高
        int new_width = static_cast<int>(this->width_ / ratio);
        int new_height = static_cast<int>(this->height_ / ratio);
        // 得到新的栅格数据
        std::vector<bool> new_data(new_width * new_height, false);
        for (int i = 0; i < this->width_; i++) {
            for (int j = 0; j < this->height_; j++) {
                // 计算是否被占据
                int index = this->getIndex(i, j);
                if (this->isOccupied(index)) {
                    // 如果被占据,判断它对应新栅格的下标
                    int new_index = static_cast<int>(i / ratio) + static_cast<int>(j / ratio) * new_width;
                    new_data[new_index] = true;
                }
            }
        }
        // 更新数据
        this->resolution_ = new_resolution;
        this->width_ = new_width;
        this->height_ = new_height;
        this->data_ = new_data;
        std::cout << "new width: " << this->width_ << ", new height: " << this->height_ << std::endl;
        std::cout << "data size: " << this->data_.size() << std::endl;
    }

    // 获取对应栅格是否被占据
    bool isOccupied(int index) const {
        return this->data_[index];
    };

    // 修改对应栅格为被占据
    void setOccupied(int index) {
        this->data_[index] = true;
    }

    // 求对应点的栅格下标
    int getIndex(int x, int y) const {
        return x + y * this->width_;
    };

    // 计算栅格坐标
    std::pair<int, int> getXY(int index) const {
        int x = index % this->width_;
        int y = index / this->width_;
        return std::make_pair(x, y);
    };

    // 判断是否抄错边界
    bool isVerify(int x, int y) const {
        if (x >= 0 && x <= this->width_ and y >= 0 and y <= height_) {
            return true;
        } else {
            return false;
        }
    }

    // 计算对应的栅格坐标
    std::pair<int, int> getGridMapCoordinate(double px, double py) const {
        CurvePoint curve_point;
        curve_point.position_.x_ = this->root_x_;
        curve_point.position_.y_ = this->root_y_;
        curve_point.theta_ = this->root_theta_;
        Point2f point;
        point.x_ = px;
        point.y_ = py;
        Point2f new_point = this->calcNewCoordinationPosition(curve_point, point);
        int x = new_point.x_ / this->resolution_;
        int y = new_point.y_ / this->resolution_;
        return std::pair<int, int>(x, y);
    };

    // 计算真实坐标
    Point2f getCartesianCoordinate(int x, int y) const {
        CurvePoint curve_point;
        curve_point.position_.x_ = this->root_x_;
        curve_point.position_.y_ = this->root_y_;
        curve_point.theta_ = this->root_theta_;
        Point2f point;
        point.x_ = static_cast<double>(x * this->resolution_);
        point.y_ = static_cast<double>(y * this->resolution_);
        Point2f raw_point = this->calcOldCoordinationPosition(curve_point, point);
        return raw_point;
    };

    // 转化成ros消息
    nav_msgs::OccupancyGrid toRosMessage() const{
        nav_msgs::OccupancyGrid occupancy_grid;
        geometry_msgs::Pose pose;
        pose.position.x = this->root_x_;
        pose.position.y = this->root_y_;
        pose.orientation.z = sin(this->root_theta_ * 0.5);
        pose.orientation.w = cos(this->root_theta_ * 0.5);
        std::vector<int8_t> out_data;
        for (auto meta_data: this->data_) {
            if (meta_data) {
                out_data.push_back(100);
            } else {
                out_data.push_back(0);
            }
        }
        occupancy_grid.data = out_data;
        occupancy_grid.info.resolution = this->resolution_;
        occupancy_grid.info.width = this->width_;
        occupancy_grid.info.height = this->height_;
        occupancy_grid.info.origin = pose;
        occupancy_grid.info.map_load_time = ros::Time::now();
        occupancy_grid.header.frame_id = "map";
        occupancy_grid.header.stamp = ros::Time::now();
        return occupancy_grid;
    };

    // 得到栅格地图的宽度
    int getWidth() const {
        return this->width_;
    };

    // 得到栅格地图的高度
    int getHeight() const {
        return this->height_;
    };

    // 得到栅格地图的分辨率；
    int getResolution() const {
        return this->resolution_;
    };



    // 计算坐标系转换
    Point2f calcNewCoordinationPosition(const CurvePoint &new_coordination_origin, const Point2f &position) const {
        Point2f new_position;
        new_position.x_ = (position.x_ - new_coordination_origin.position_.x_) * cos(new_coordination_origin.theta_) + (position.y_ - new_coordination_origin.position_.y_) * sin(new_coordination_origin.theta_);
        new_position.y_ = -(position.x_ - new_coordination_origin.position_.x_) * sin(new_coordination_origin.theta_) + (position.y_ - new_coordination_origin.position_.y_) * cos(new_coordination_origin.theta_);
        return new_position;
    }

    // 计算坐标系反转换
    Point2f calcOldCoordinationPosition(const CurvePoint &new_coordination_origin, const Point2f &position) const {
        Point2f old_position;
        old_position.x_ = new_coordination_origin.position_.x_ + position.x_ * cos(new_coordination_origin.theta_) - position.y_ * sin(new_coordination_origin.theta_);
        old_position.y_ = new_coordination_origin.position_.y_ + position.x_ * sin(new_coordination_origin.theta_) + position.y_ * cos(new_coordination_origin.theta_);
        return old_position;
    }

 private:
    int width_;  // 栅格地图的宽度(格数)
    int height_;  // 栅格地图的高度(格数)
    double resolution_;  // 栅格地图的分辨率
    double root_x_;  // 栅格地图根节点x坐标
    double root_y_;  // 栅格地图根节点y坐标
    double root_theta_;  // 栅格地图根节点朝向
    std::vector<bool> data_;  // 栅格地图数据
};

// 消息接收类
class MessageReceiver {
 public:
    // 构造函数
    MessageReceiver(const ros::NodeHandle &nh, const std::string &topic_name, const std::string &data_type) {
        // 初始化变量
        this->nh_ = nh;
        this->tf_listener_ptr_.reset(new tf::TransformListener());
        this->projector_.reset(new laser_geometry::LaserProjection());
        // 判断类型
        if (!(data_type == "PointCloud2" || data_type == "PointCloud" || data_type == "LaserScan" || data_type == "Ultrasonic")) {
            ROS_FATAL("Only topics that use point clouds or laser scans are currently supported");
            throw std::runtime_error("Only topics that use point clouds or laser scans are currently supported");
        } else if (data_type == "PointCloud2") {
            sub_ = this->nh_.subscribe(topic_name, 1, &MessageReceiver::pointCloud2CallBack, this);
        } else if (data_type == "PointCloud") {
            sub_ = this->nh_.subscribe(topic_name, 1, &MessageReceiver::pointCloudCallBack, this);
        } else if (data_type == "LaserScan") {
            //std::cout << "subscribe laserscan" << std::endl;
            sub_ = this->nh_.subscribe(topic_name, 1, &MessageReceiver::laserScanCallBack, this);
        } else if (data_type == "Ultrasonic") {
            sub_ = this->nh_.subscribe(topic_name, 1, &MessageReceiver::UltrasonicCallBack, this);
        }
    }

    // 析构函数
    ~MessageReceiver() {

    }

    // 获取消息中的数据
    pcl::PointCloud<pcl::PointXYZ> getData() {
        this->mutex_.lock();
        pcl::PointCloud<pcl::PointXYZ> data = this->record_data_;
        this->mutex_.unlock();
        return data;
    }

 private:
    // 消息回调函数
    void pointCloud2CallBack(sensor_msgs::PointCloud2::ConstPtr message) {
        this->bufferCloud(*message);
    }

    // 消息回调函数
    void pointCloudCallBack(sensor_msgs::PointCloud::ConstPtr message) {
        sensor_msgs::PointCloud2 cloud2;
        if (!sensor_msgs::convertPointCloudToPointCloud2(*message, cloud2)) {
            ROS_ERROR("Failed to convert a PointCloud to a PointCloud2, dropping message");
            return;
        }
        this->bufferCloud(cloud2);
    }

    // 消息回调函数
    void laserScanCallBack(sensor_msgs::LaserScan::ConstPtr message) {
        sensor_msgs::PointCloud2 cloud;
        cloud.header = message->header;
        try{
            this->projector_->transformLaserScanToPointCloud(message->header.frame_id, *message, cloud, *tf_listener_ptr_);
        }
        catch (tf::TransformException &ex)
        {
            this->projector_->projectLaser(*message, cloud);
        }
        this->bufferCloud(cloud);
    }

    // 消息回调函数
    void UltrasonicCallBack(sensor_msgs::Range::ConstPtr message) {
        sensor_msgs::LaserScan scan;
        scan.header = message->header;
        scan.angle_min = -message->field_of_view / 2;
        scan.angle_max = message->field_of_view / 2;
        scan.angle_increment = 0.005;
        scan.range_min =message->min_range;
        scan.range_max =message->max_range;
        if (message->range < message->max_range) {
            scan.ranges.resize(message->field_of_view / scan.angle_increment + 1);
            for (size_t i = 0; i < scan.ranges.size(); i++) {
                scan.ranges[i] = message->range;
            }
        }

        sensor_msgs::PointCloud2 cloud;
        cloud.header = message->header;
        try{
            this->projector_->transformLaserScanToPointCloud(message->header.frame_id, scan, cloud, *tf_listener_ptr_);
        }
        catch (tf::TransformException &ex)
        {
            this->projector_->projectLaser(scan, cloud);
        }
        this->bufferCloud(cloud);
    }

    // 进行点云处理
    void bufferCloud(const sensor_msgs::PointCloud2& cloud) {
        // Actually convert the PointCloud2 message into a type we can reason about
        pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
        try {
            pcl::PCLPointCloud2 pcl_pc2;
            pcl_conversions::toPCL(cloud, pcl_pc2);

            pcl::fromPCLPointCloud2(pcl_pc2, pcl_cloud);
        }
        catch (pcl::PCLException& ex) {
            ROS_ERROR("Failed to convert a message to a pcl type, dropping observation: %s", ex.what());
            return;
        }
        // 获取tf
        tf::StampedTransform tf_transformer;  // tf变换矩阵
        tf_listener_ptr_->waitForTransform("world", cloud.header.frame_id, ros::Time(0), ros::Duration(1));
        try {
            tf_listener_ptr_->lookupTransform("world", cloud.header.frame_id, ros::Time(0), tf_transformer);
        }
        catch(const std::exception& e) {
            ROS_ERROR_STREAM("[TF NOT FIND ERROR] " << cloud.header.frame_id);
        }
        // 进行转化
        pcl::PointCloud<pcl::PointXYZ> transformed_pcl_cloud;
        pcl_ros::transformPointCloud(pcl_cloud, transformed_pcl_cloud, tf_transformer);
        // 保存输出
        this->mutex_.lock();
        this->record_data_ = transformed_pcl_cloud;
        this->mutex_.unlock();
    }

    ros::NodeHandle nh_;  // ros句柄
    std::shared_ptr<tf::TransformListener> tf_listener_ptr_;  // tf监听器
    ros::Subscriber sub_;  // 消息订阅
    std::shared_ptr<laser_geometry::LaserProjection> projector_;  // laserscan消息转化
    pcl::PointCloud<pcl::PointXYZ> record_data_;  // 记录的数据
    std::mutex mutex_;  // 线程锁
};

class CostmapGeneration {
 public:
    // 构造函数
    CostmapGeneration() {
        // 进行ros相关初始化
        ros::NodeHandle nh("~"), g_nh;
        // 获得感知参数列表
        std::string topics_string;
        nh.getParam("observation_sources", topics_string);
        std::stringstream ss(topics_string);
        std::cout << "topics_string " << topics_string << std::endl;
        // 初始化感知对接
        std::string source;
        while (ss >> source) {
            ros::NodeHandle source_node(nh, source);
            // std::cout << "source_node.getNamespace()" << source_node.getNamespace() << std::endl;
            // ros::V_string keys;
            // source_node.getParamNames(keys);
            // for (auto key: keys) {
            //     std::cout << "key " << key << std::endl;
            // }
            // 订阅节点名称
            std::string topic_name;
            source_node.getParam("topic_name", topic_name);
            std::cout << "topic_name " << topic_name << std::endl;
            // 订阅节点类型
            std::string data_type;
            source_node.getParam("data_type", data_type);
            std::cout << "data_type " << data_type << std::endl;
            // 判断输入是否正确
            if (!(data_type == "PointCloud2" || data_type == "PointCloud" || data_type == "LaserScan" || data_type == "Ultrasonic")) {
                ROS_FATAL("Only topics that use point clouds or laser scans are currently supported");
                throw std::runtime_error("Only topics that use point clouds or laser scans are currently supported");
            }
            // 创建回调对象
            std::shared_ptr<MessageReceiver> message_receiver_ptr(new MessageReceiver(g_nh, topic_name, data_type));
            message_receiver_ptrs_.push_back(message_receiver_ptr);
        }
        // 初始化地图对接
        std::string static_map_topic;
        nh.getParam("static_map_topic", static_map_topic);
        std::cout << "static_map_topic " << static_map_topic << std::endl;
        this->map_sub_ = nh.subscribe(static_map_topic, 1, &CostmapGeneration::mapUpdate, this);
        // 初始化代价地图发布
        std::string costmap_pub_topic;
        nh.getParam("costmap_pub_topic", costmap_pub_topic);
        std::cout << "costmap_pub_topic " << costmap_pub_topic << std::endl;
        this->costmap_pub_ = nh.advertise<nav_msgs::OccupancyGrid>(costmap_pub_topic, 10);
    }

    // 析构函数
    ~CostmapGeneration() {};

    // 获取信息
    pcl::PointCloud<pcl::PointXYZ> getData() {
        pcl::PointCloud<pcl::PointXYZ> total_cloud;
        for (auto message_receiver_ptr: message_receiver_ptrs_) {
            pcl::PointCloud<pcl::PointXYZ> cloud = message_receiver_ptr->getData();
            total_cloud += cloud;
        }
        return total_cloud;
    };

    // 生成代价地图
    void publishCostMap() {
        // 判断是否接收到地图
        this->static_map_received_flag_mutex_.lock();
        bool static_map_received_flag = this->static_map_received_flag_;
        this->static_map_received_flag_mutex_.unlock();
        if (!static_map_received_flag) {
            std::cout << "未接收到地图" << std::endl;
            return;
        }
        // 获取初始值
        this->static_map_mutex_.lock();
        nav_msgs::OccupancyGrid costmap = this->static_map_;
        this->static_map_mutex_.unlock();
        // 进行格式转化
        GridMap grid_map = GridMap(costmap);
        // 得到感知数据
        pcl::PointCloud<pcl::PointXYZ> cloud = this->getData();
        // 遍历感知数据,将其投影到地图中
        for (auto point: cloud.points) {
            // 计算点云对应的栅格
            std::pair<int, int> grid_point = grid_map.getGridMapCoordinate(point.x, point.y);
            // 判断是否在地图内
            if (grid_map.isVerify(grid_point.first, grid_point.second)) {
                // 设置该栅格为被占据
                grid_map.setOccupied(grid_map.getIndex(grid_point.first, grid_point.second));
            }
        }
        costmap = grid_map.toRosMessage();
        this->costmap_pub_.publish(costmap);
    }

 private:
    // 更新地图
    void mapUpdate(nav_msgs::OccupancyGrid::ConstPtr msg) {
        this->static_map_mutex_.lock();
        this->static_map_ = *msg;
        this->static_map_mutex_.unlock();
        this->static_map_received_flag_mutex_.lock();
        if (!this->static_map_received_flag_) {
            this->static_map_received_flag_ = true;
        }
        this->static_map_received_flag_mutex_.unlock();
    }


    std::vector<std::shared_ptr<MessageReceiver>> message_receiver_ptrs_;  // 障碍物记录器
    nav_msgs::OccupancyGrid static_map_;  // 静态地图
    std::mutex static_map_mutex_;  // 静态地图锁
    ros::Publisher costmap_pub_;  // 代价地图发布
    ros::Subscriber map_sub_;  // 静态地图接收
    bool static_map_received_flag_ = false;  // 是否接收到静态地图
    std::mutex static_map_received_flag_mutex_;  // 是否接收到静态地图锁
};
