/*
    Copyright [2021] Pang Jinlong

*/

#ifndef FRENET_TRAJECTORY_H_
#define FRENET_TRAJECTORY_H_

#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <time.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "grid_map.hpp"



//五次多项式
class QuinticPolynomial{
    public:
        QuinticPolynomial(float d0, float v0, float a0, float df, float vf, float af, float T){
            a0_ = d0;
            a1_ = v0;
            a2_ = a0 * 0.5;
            T_ = T;
            coefficientGeneration(df,vf,af);
        }
        ~QuinticPolynomial(){}
        
        void coefficientGeneration(float df, float vf, float af);
        float calcValue(float t);
        float calcDerivation(float t);
        float calc2Derivation(float t);
        float calc3Derivation(float t);

    private:
        float a0_, a1_, a2_, a3_, a4_, a5_;
        Eigen::VectorXf coefficient_;
        float T_; 
};

//四次多项式
class QuarticPolynomial{

    public:
        QuarticPolynomial(float s0, float v0, float a0, float vf, float af, float T){
            a0_ = s0;
            a1_ = v0;
            a2_ = a0 * 0.5;
            T_ = T;
            coefficientGeneration(vf,af);
       }
        ~QuarticPolynomial(){};

        void coefficientGeneration(float vf, float af);
        float calcValue(float t);
        float calcDerivation(float t);
        float calc2Derivation(float t);
        float calc3Derivation(float t);

    private:
        float a0_, a1_, a2_, a3_, a4_;
        Eigen::VectorXf coefficient_;
        float T_;
};


//构建三次样条曲线
class Spline{
    public:
        Spline(std::vector<float> &x, std::vector<float> &y){
            x_ = x;
            y_ = y;
            number_ = x.size();
            n_ = int(x.size())-1;
            //中间参数
            for(int i=0; i<int(x_.size())-1; i++){
                h_.push_back(x_[i+1]-x[i]);
            }
            a_ = Eigen::VectorXf::Zero(y_.size());
            //开始计算参数a
            for(int i=0; i<y_.size(); i++){
                a_[i]= y_[i];
            }
            coefficientGeneration(x_, y_);
        }

        Spline(){}
        ~Spline(){}

        void coefficientGeneration(std::vector<float> &x, std::vector<float> &y);
        //获得矩阵A
        Eigen::MatrixXf getA();
        //获得向量B，自由边界条件
        Eigen::VectorXf getB();
        //计算对应函数段
        int getSegment(float &s){
            auto p = std::upper_bound(x_.begin(),x_.end(),s); 
            //找到第一个大于 value 值的位置并返回，如果找不到同样返回 end() 位置
            if(p == x_.end()){
                std::cout<<"超过限制的参考曲线段"<<std::endl;
                return -1;
            } else{
                return std::distance(x_.begin(),p)-1;
            }
        }
        //计算位置
        float calc(float &s){
            //首先得到对应函数段
            int index = getSegment(s);

                float dis = s - x_[index];
                // std::cout<<"index "<< index <<std::endl;
                // std::cout<<"a "<< a_[index] <<std::endl;
                // std::cout<<"b "<< b_[index] <<std::endl;
                // std::cout<<"c "<< c_[index] <<std::endl;
                // std::cout<<"d "<< d_[index] <<std::endl;
                return a_[index] + b_[index] * dis + c_[index] * dis * dis + d_[index]* dis * dis * dis;
        }
        //计算一阶导数
        float calcd(float &s){
            //首先得到对应函数段
            int index = getSegment(s);
                float dis = s - x_[index];
                return b_[index] + 2.0 * c_[index] * dis + 3.0 * d_[index] * dis * dis;
        }
        //计算二阶导数
        float calcdd(float &s){
            int index = getSegment(s);
                float dis = s - x_[index];
                return 2.0 * c_[index] + 6.0 * d_[index] * dis;
        }


    private:
        //输入散点
        std::vector<float> x_;
        std::vector<float> y_;
        //散点数量
        int number_;
        //最大下标(也是方程式的个数，参数个数)
        int n_;
        //中间参数
        std::vector<float> h_;
        //未知参数
        Eigen::VectorXf a_, b_, c_, d_;

};


//2D三次样条曲线  x y 方向都是三次样条曲线！！！ 这里用于表示Frenet坐标系
class CubicSpline2D{
    public:
        CubicSpline2D(std::vector<float> &x, std::vector<float> &y){
            x_ = x;
            y_ = y;
            //计算路程
            calcPathDistence();
            //开始构建曲线
            genarateSpline();
        }
        CubicSpline2D(){}
        ~CubicSpline2D(){}


        //计算路程
        void calcPathDistence(){

            std::vector<float> ds;
            // std::cout<<"x size"<< x_.size()<<std::endl;
            // std::cout<<"x-1 size"<< (int(x_.size())-1)<<std::endl;
            for(int i=0; i<(int(x_.size())-1); i++){
                ds.push_back(sqrt(pow(x_[i+1]-x_[i], 2)+ pow(y_[i+1]- y_[i], 2)));
            }
            s_.push_back(0.0);
            float sum_dis = 0.0;
            for(int i=0; i<ds.size(); i++){
                sum_dis = sum_dis+ ds[i];
                s_.push_back(sum_dis);
            }
        }
        //开始构建曲线
        void genarateSpline(){
            Spline_X_ = Spline(s_,x_);
            Spline_Y_ = Spline(s_,y_);
        }

        //由s计算在xy坐标系下的位置
        void calcPosition(std::vector<float> &s_set, std::vector<float> &x_set, std::vector<float> &y_set){
            for (auto s: s_set){
                if(s<s_.front()||s>s_.back()){
                    //std::cout<<"规划的点越过终点，去掉"<<std::endl;
                    continue;
                }
                x_set.push_back(Spline_X_.calc(s));
                y_set.push_back(Spline_Y_.calc(s));
            }
        }

        //计算朝向
        void calcYaw(std::vector<float> &s_set, std::vector<float> &yaw_set){
            for (auto s: s_set){
                if(s<s_.front()||s>s_.back()){
                    //std::cout<<"规划的点越过终点，去掉"<<std::endl;
                    continue;
                }
                float dx = Spline_X_.calcd(s);
                float dy = Spline_Y_.calcd(s);
                yaw_set.push_back(atan2(dy,dx));
            }
        }

        //计算曲率
        void calcKappa(std::vector<float> &s_set, std::vector<float> &kappa_set){
            for(auto s: s_set){

                if(s<s_.front()||s>s_.back()){
                    //std::cout<<"规划的点越过终点，去掉"<<std::endl;
                    continue;
                }

                float dx = Spline_X_.calcd(s);
                float dy = Spline_Y_.calcd(s);
                float ddx = Spline_X_.calcdd(s);
                float ddy = Spline_Y_.calcdd(s);
                kappa_set.push_back((ddy * dx - ddx * dy) / pow((dx * dx + dy * dy),(3 / 2)));
            }
        }

        std::vector<float> get_x_(){ return x_;}
        std::vector<float> get_y_(){ return y_;}
        std::vector<float> get_s_(){ return s_;}

    private:
        //输入散点
        std::vector<float> x_;
        std::vector<float> y_;

        std::vector<float> s_;

        Spline Spline_X_;
        Spline Spline_Y_;
};

//frenet轨迹对象
class FrenetPath{
    public:
        FrenetPath(){}
        ~FrenetPath(){}
        //时间采样
        std::vector<float> t_;

        //frenet系数据
        //横向偏移 横向偏移一阶导数 横向偏移二阶导数 横向偏移三阶导数
        std::vector<float> d_, d_derivative_, d_2derivative_, d_3derivative_;

        //纵向偏移 纵向偏移一阶导数 纵向偏移二阶导数 纵向偏移三阶导数
        std::vector<float> s_, s_derivative_, s_2derivative_, s_3derivative_;

        //评分
        float cost_;

        //世界系数据 x坐标 y坐标 x速度 y速度 朝向角 朝向角速度 曲率 两点之间的距离
        std::vector<float> x_, y_, yaw_, x_derivative, y_derivative, yaw_derivative, kappa_, dis_;
};

class FrenetPathPlanner{


    public:
        FrenetPathPlanner(ros::NodeHandle &nh_){

            std::string global_path;
            std::string costmap;    
            std::string robot_pose;
            std::string local_trajectory;
            std::string local_trajectory_path;
            std::string renference_curve;
            std::string frenet_path_set_one;
            
            //ros::NodeHandle nh_("~");
            nh_.param("global_path", global_path, std::string("global_path"));
            nh_.param("costmap", costmap, std::string("costmap"));
            nh_.param("robot_pose", robot_pose, std::string("robot_pose"));
            nh_.param("local_trajectory", local_trajectory, std::string("local_trajectory"));
            nh_.param("local_trajectory_path", local_trajectory_path, std::string("local_trajectory_path"));
            nh_.param("renference_curve", renference_curve, std::string("renference_curve"));
            // nh_.param("frenet_path_set_one", frenet_path_set_one, std::string("frenet_path_set_one"));
            
            nh_.param("ROAD_WIDTH", ROAD_WIDTH, 7.0);
            nh_.param("WIDTH_GAP", WIDTH_GAP, 1.0);
            nh_.param("MIN_T", MIN_T, 4.0);
            nh_.param("MAX_T", MAX_T, 5.0);
            nh_.param("T_GAP", T_GAP, 0.1);
            nh_.param("ESP", ESP, 1e-6);
            nh_.param("MAX_VELOCITY", MAX_VELOCITY, 50.0/3.6);
            nh_.param("TARGET_VELOCITY", TARGET_VELOCITY, 30.0/3.6);
            nh_.param("VELOCITY_GAP", VELOCITY_GAP, 5.0/3.6);
            nh_.param("VELOCITY_SAMPLE_N", VELOCITY_SAMPLE_N, 1);
            nh_.param("MAX_KAPPA", MAX_KAPPA, 1.0);
            nh_.param("MAX_ACCELERATION", MAX_ACCELERATION, 2.0);
            nh_.param("ROBOT_SIZE", ROBOT_SIZE, 1.5);
            nh_.param("GOAL_SIZE", GOAL_SIZE, 0.5);
            nh_.param("PATH_GAP", PATH_GAP, 50);
            nh_.param("AREA", AREA, 20.0);

            path_sub = nh_.subscribe(global_path,1, &FrenetPathPlanner::PathCallback, this);
            costmap_sub = nh_.subscribe(costmap, 1, &FrenetPathPlanner::MapCallback, this);
            pose_sub = nh_.subscribe(robot_pose, 1, &FrenetPathPlanner::PoseCallback, this);
            
            trajectory_path_pub = nh_.advertise<nav_msgs::Path>(local_trajectory_path,10);
            trajectory_pub = nh_.advertise<trajectory_msgs::JointTrajectory>(local_trajectory,10);
            reference_curve_pub = nh_.advertise<nav_msgs::Path>(renference_curve,10);
            // frenet_path_set_one_pub = nh_.advertise<nav_msgs::Path>(frenet_path_set_one,10);
            candidates_viz_pub = nh_.advertise<visualization_msgs::MarkerArray>("/candidates_viz", 10, true);

            
            current_s = 0.0; //frenet坐标系下的s轴
            // current_s_derivative = 0.2;
            current_s_derivative = 0.0;
            current_s_2derivative = 0.0;

            current_d = 0.0; //frenet坐标系下的d轴
            current_d_derivative = 0.0;
            current_d_2derivative = 0.0;

            expected_path_x = 0.0;
            expected_path_y = 0.0;

            current_x = 0;
            current_y = 0;

        };
        FrenetPathPlanner(){};
        ~FrenetPathPlanner(){};

        void Planning();

        void referenceCurvePlanning();

        void frenetOptimalTrajectoryPlanning(std::vector<FrenetPath>& frenet_path_set, 
                                        float & current_s,float & current_s_d, float & current_s_2d,
                                        float & current_d, float & current_d_d, float & current_d_2d);

        void frenetToGloabalTrajectory(std::vector<FrenetPath>& frenet_path_set, 
                               std::vector<FrenetPath>& global_path_set, 
                               CubicSpline2D &reference_curve);


        void checkPath(std::vector<FrenetPath>& checked_path_set, 
                        std::vector<FrenetPath>& oringin_path_set);

        
        // void checkPath(std::vector<FrenetPath>& checked_path_set, 
        //                std::vector<FrenetPath>& oringin_path_set, 
        //                std::vector<std::vector<float> > obstacles);

        // bool isCollision(FrenetPath &path, std::vector<std::vector<float> > obstacles); 
              

        bool isCollision(FrenetPath &path);


        //全局路径回调函数
        void PathCallback(const nav_msgs::Path::ConstPtr path);

        //代价地图回调函数
        void MapCallback(const nav_msgs::OccupancyGrid::ConstPtr map);

        //位姿回调函数
        void PoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr pose);

        //速度回调函数
        void TwistCallback(const geometry_msgs::Twist::ConstPtr twist);

        //frenet轨迹发布函数
        void TrajectoryPub();

        //frenet参考曲线发布函数
        void ReferenceCurvePub();

        void initPlanner();
        
    private:
        
        ros::Publisher reference_curve_pub;//参考曲线发布
        ros::Publisher trajectory_path_pub;//轨迹的路径发布
        ros::Publisher trajectory_pub;//轨迹发布
        // ros::Publisher frenet_path_set_one_pub;//debug use
        ros::Publisher candidates_viz_pub;
        ros::Subscriber path_sub;//订阅全局路径
        ros::Subscriber costmap_sub;//订阅代价地图
        ros::Subscriber pose_sub;//订阅当前位姿

        double ROAD_WIDTH;  // 道路的最大宽度
        double WIDTH_GAP;  // 道路的横向采样间隔
        double MIN_T;  // 最短时间开销
        double MAX_T;  // 最长时间开销
        double T_GAP ;  // 时间采样kdtree
        double ESP;  // 精度要求
        double MAX_VELOCITY;  // 最大速度
        double TARGET_VELOCITY;  // 期望速度
        double VELOCITY_GAP;  // 速度采样间隔
        int VELOCITY_SAMPLE_N;  // 速度采样个数
        double MAX_KAPPA;  // 最大曲率
        double MAX_ACCELERATION;  // 最大加速度
        double ROBOT_SIZE;  // 机器人的半径
        double GOAL_SIZE;  // 机器人的半径
        int    PATH_GAP;
        double AREA;  // 可视化窗口大小

        // 评分加权参数
        float KJ = 0.1;
        float KT = 0.1;
        float KD = 1.0;
        float KLON = 1.0;
        float KLAT = 1.0;

        //初始化参考路点
        std::vector<float> wx;
        std::vector<float> wy; 

        //初始化障碍物所在位置
        std::vector<std::vector<float> > obstacles = {{20.0, 10.0},
                                                    {30.0, 6.0},
                                                    {30.0, 8.0},
                                                    {35.0, 8.0},
                                                    {50.0, 3.0}};

        // 初始化车辆信息，车辆在frenet坐标下规划出来轨迹的期望位置、速度、加速度等。
        float current_s; //frenet坐标系下的s轴
        float current_s_derivative;
        float current_s_2derivative;

        float current_d; //frenet坐标系下的d轴
        float current_d_derivative;
        float current_d_2derivative ;

        float expected_path_x;
        float expected_path_y;

        //当前车辆在全局坐标系下的位置，订阅定位模块得到。作为机器人的真实位置
        float current_x;
        float current_y;

        //占用栅格地图地图
        nav_msgs::OccupancyGrid occupancy_map;

        //代价地图类
        //GridMap grid_map(occupancy_map);
        GridMap grid_map;

        //参考曲线
        CubicSpline2D reference_curve;

        std::vector<float> reference_waypoints_x;
        std::vector<float> reference_waypoints_y;
        int reference_curve_num = 0;

        //最终规划出来的局部路径
        FrenetPath final_path;


};




#endif