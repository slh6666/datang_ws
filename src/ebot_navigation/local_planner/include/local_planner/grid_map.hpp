#ifndef GRID_MAP_HPP
#define GRID_MAP_HPP



#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <time.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>

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
    double getResolution() const {
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

#endif