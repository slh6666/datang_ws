#include "local_planner/frenet_trajectory.hpp"


//三次样条曲线
void Spline::coefficientGeneration(std::vector<float> &x, std::vector<float> &y){
    Eigen::MatrixXf A = getA();
    Eigen::VectorXf B = getB();

    //Eigen::VectorXf m = A.colPivHouseholderQr().solve(B);
    //求解中间参量m
    Eigen::MatrixXf A_inv = A.inverse();
    Eigen::VectorXf m =  A_inv * B;
    //std::cout<<"A "<< A<<std::endl;
    //std::cout<<"B "<< B<<std::endl;
    //std::cout<<"m "<< m<<std::endl;
    //计算参数c
    c_ = 0.5*m;
    d_ = Eigen::VectorXf::Zero(n_);
    b_ = Eigen::VectorXf::Zero(n_);
    for(int i= 0; i<d_.size(); i++){
        d_[i] = (m[i + 1] - m[i]) / (6.0 * h_[i]);
        b_[i] = (y_[i + 1] - y_[i]) / h_[i] - 0.5 * h_[i] * m[i] - h_[i] * (m[i + 1] - m[i]) / 6.0;
    }
    // std::cout<<"a "<< a_<<std::endl;
    // std::cout<<"b "<< b_<<std::endl;
    // std::cout<<"c "<< c_<<std::endl;
    // std::cout<<"d "<< d_<<std::endl;
}

Eigen::MatrixXf Spline::getA(){
    Eigen::MatrixXf A =  Eigen::MatrixXf::Zero(number_,number_); 
    A(0,0) = 1.0;
    for(int i=1; i<n_; i++){
        A(i,i) = 2*(h_[i-1]+h_[i]);
        A(i-1,i) = h_[i-1];
        A(i,i-1) = h_[i-1];
    }
    A(0,1) = 0.0;
    A(n_-1,n_) = h_[n_-1];
    A(n_,n_) = 1.0;
    return A;
}

Eigen::VectorXf Spline::getB(){
    Eigen::VectorXf B = Eigen::VectorXf::Zero(number_);
    for(int i=1; i<n_; i++){
        B(i) = 6.0 * ((y_[i + 1] - y_[i]) / h_[i] - (y_[i] - y_[i - 1]) / h_[i - 1]);
    }
    return B;
}


void QuinticPolynomial::coefficientGeneration(float df, float vf, float af){

    Eigen::Matrix<float,3,3> A;
    A <<    pow(T_, 3),     pow(T_, 4),      pow(T_, 5),
            3.0*pow(T_, 2), 4.0*pow(T_, 3),  5.0*pow(T_, 4),
            6.0*pow(T_, 1), 12.0*pow(T_, 2), 20.0*pow(T_, 3);
    
    Eigen::Vector3f b;
    b << df- a0_ - a1_*T_ - a2_*pow(T_, 2), 
         vf - a1_ - 2.0*a2_*T_, 
         af - 2.0*a2_;

    Eigen::Vector3f X;
    
    clock_t start, end;

    // start = clock();

    // X = A.householderQr().solve(b);

    // end = clock();

    // std::cout<<"QR time = "<<double(end-start)/CLOCKS_PER_SEC<<"s"<<std::endl;  //输出时间（单位：ｓ）

    //start = clock();
    Eigen::MatrixXf A_inv = A.inverse();
    X =  A_inv * b;
    //end = clock();
    //std::cout<<"inverse time = "<<double(end-start)/CLOCKS_PER_SEC<<"s"<<std::endl;  //输出时间（单位：ｓ）

    //X = A.llt().solve(b);
    a3_ = X(0);
    a4_ = X(1);
    a5_ = X(2);
    coefficient_ = Eigen::VectorXf::Zero(6);
    coefficient_ << a0_, a1_, a2_, a3_, a4_, a5_;
}
//给定输入时间，计算对应值
float QuinticPolynomial::calcValue(float t){
    Eigen::VectorXf t6(6);
    t6 << 1.0, t, pow(t, 2), pow(t, 3), pow(t, 4), pow(t, 5);
    return coefficient_.dot(t6);
}
//给定输入时间，计算导数
float QuinticPolynomial::calcDerivation(float t){
    Eigen::VectorXf t5(5);
    t5 << 1.0, 2.0*t, 3.0*pow(t, 2), 4.0*pow(t, 3), 5.0*pow(t, 4);
    return coefficient_.tail(5).dot(t5);
}
//给定输入时间，计算二阶导数
float QuinticPolynomial::calc2Derivation(float t){
    Eigen::VectorXf t4(4);
    t4 << 2.0, 6.0*t, 12.0*pow(t, 2), 20.0*pow(t, 3);
    return coefficient_.tail(4).dot(t4);
}
//给定输入时间，计算三阶导数
float QuinticPolynomial::calc3Derivation(float t){
    Eigen::VectorXf t3(3);
    t3 << 6.0, 24.0*t, 60.0*pow(t, 2);
    return coefficient_.tail(3).dot(t3);
}



// 生成4次多项式系数计算
void QuarticPolynomial::coefficientGeneration(float vf, float af){

    Eigen::Matrix<float,2,2> A;
    A << 3.0*pow(T_, 2), 4.0*pow(T_, 3),
         6.0*pow(T_, 1), 12.0*pow(T_, 2);
    
    Eigen::Vector2f b;
    b << vf - a1_ - 2.0*a2_*T_, 
         af - 2.0*a2_;

    Eigen::VectorXf X(2);
    //X = A.colPivHouseholderQr().solve(b);
    Eigen::MatrixXf A_inv = A.inverse();
    X =  A_inv * b;
    //X = A.llt().solve(b);
    a3_ = X(0);
    a4_ = X(1);
    coefficient_ = Eigen::VectorXf::Zero(5);
    coefficient_ << a0_, a1_, a2_, a3_, a4_;
}
//给定输入时间，计算对应值
float QuarticPolynomial::calcValue(float t){
    Eigen::VectorXf t5(5);
    t5 << 1.0, t, pow(t, 2), pow(t, 3), pow(t, 4);
    return coefficient_.dot(t5);
}
//给定输入时间，计算导数
float QuarticPolynomial::calcDerivation(float t){
    Eigen::VectorXf t4(4);
    t4 << 1.0, 2.0*t, 3.0*pow(t, 2), 4.0*pow(t, 3);
    return coefficient_.tail(4).dot(t4);
}
//给定输入时间，计算二阶导数
float QuarticPolynomial::calc2Derivation(float t){
    Eigen::VectorXf t3(3);
    t3 << 2.0, 6.0*t, 12.0*pow(t, 2);
    return coefficient_.tail(3).dot(t3);
}
//给定输入时间，计算三阶导数
float QuarticPolynomial::calc3Derivation(float t){
    Eigen::VectorXf t2(2);
    t2 << 6.0, 24.0*t;
    return coefficient_.tail(2).dot(t2);
}




void FrenetPathPlanner::Planning(){

    
    
    //接下来是规划的主体内容
    //第一步生成参考路点路径曲线
    //利用三次样条差值生成参考曲线
    if(wx.size()>0 && reference_waypoints_x.size()>0 && reference_waypoints_y.size()>0){
        //判断是否已经到达终点范围

        // std::cout << "current_x " << current_x << std::endl;
        // std::cout << "wx.back() " << wx.back() << std::endl;
        
        // std::cout << "current_y " << current_y << std::endl;
        // std::cout << "wy.back() " << wy.back() << std::endl;

        //std::cout << "ROBOT_SIZE " << ROBOT_SIZE << std::endl;

        if(sqrt(pow(current_x - wx.back(), 2) + 
                pow(current_y-wy.back(), 2)) < GOAL_SIZE){
                    std::cout<<"goal reached"<<std::endl;
                }
        else{

            //std::cout << "current_x " << current_x << "current_y " << current_y << std::endl;
            //std::cout << "expected_path_x " << expected_path_x << "expected_path_y " << expected_path_y<< std::endl;
            
            clock_t start, end , end1, end2;
            
            start = clock();

            //记录当前点的朝向和曲率
            std::vector<float> yaw_set, kappa_set, s_set;

            //每一次更新所花费时间
            int  gap = 8;

            //原算法以10hz频率不断进行规划，此处以到达下一个点进行替代，每到规划的下一个点就进行重新规划
            //进行frenet坐标架下的规划
            std::vector<FrenetPath> frenet_path_set;
            frenetOptimalTrajectoryPlanning(frenet_path_set,
                                            current_s, current_s_derivative, current_s_2derivative, 
                                            current_d, current_d_derivative, current_d_2derivative);
            std::cout<< "frenet_path_set number "<<frenet_path_set.size() << std::endl;
            
            end = clock();

            //std::cout<<"frenet planning time = "<<double(end-start)/CLOCKS_PER_SEC<<"s"<<std::endl;  //输出时间（单位：ｓ）
            
            //将frenet坐标系转换到世界坐标系下
            std::vector<FrenetPath> global_path_set;
            frenetToGloabalTrajectory(frenet_path_set, global_path_set, reference_curve);
            std::cout<< "global_path_set number "<<global_path_set.size() << std::endl;
            
            end1 = clock();

            //std::cout<<"frame time = "<<double(end1-end)/CLOCKS_PER_SEC<<"s"<<std::endl;  //输出时间（单位：ｓ）

            //  for(size_t j = 0; j < global_path_set.size(); j++){

            //      nav_msgs::Path frenet_path_set_one;
            //      frenet_path_set_one.header.frame_id = "map";
            //      frenet_path_set_one.header.stamp = ros::Time::now();
            //      for (size_t i = 0; i < global_path_set[j].x_.size(); i++) {
            //          geometry_msgs::PoseStamped pose;
            //          pose.header = frenet_path_set_one.header;
            //          pose.pose.position.x = global_path_set[j].x_[i];
            //          pose.pose.position.y = global_path_set[j].y_[i];
            //          //std::cout<<"reference_waypoints_x" << reference_waypoints_x[i] <<std::endl;
            //          //std::cout<<"reference_waypoints_y" << reference_waypoints_y[i] <<std::endl;
            //          frenet_path_set_one.poses.push_back(pose);
            //      }
            //      frenet_path_set_one_pub.publish(frenet_path_set_one);
            //  }
            
            visualization_msgs::MarkerArray delete_marker_array;
            visualization_msgs::Marker delete_marker;
            delete_marker.header.frame_id = "map";
            delete_marker.header.stamp = ros::Time::now(); //!!!
            delete_marker.action = visualization_msgs::Marker::DELETEALL;
            delete_marker.id = 0;
            delete_marker_array.markers.push_back(delete_marker);
            this->candidates_viz_pub.publish(delete_marker_array);

            visualization_msgs::MarkerArray paths_array;
            for(size_t j = 0; j < global_path_set.size(); j++){

                visualization_msgs::Marker path_marker;
                path_marker.header.frame_id = "map";
                path_marker.header.stamp = ros::Time::now();
                path_marker.id = j;
                path_marker.type = visualization_msgs::Marker::LINE_STRIP;
                path_marker.action = visualization_msgs::Marker::ADD;
                path_marker.scale.x = 0.02;
                path_marker.color.r = 0.0;
                path_marker.color.g = 0.0;
                path_marker.color.b = 1.0;
                path_marker.color.a = 0.8;
                for (size_t i = 0; i < global_path_set[j].x_.size(); i++) {
                        geometry_msgs::Point point;
                        point.x = global_path_set[j].x_[i];
                        point.y = global_path_set[j].y_[i];
                        path_marker.points.push_back(point);
                }
                paths_array.markers.push_back(path_marker);
            }
            this->candidates_viz_pub.publish(paths_array);
            
        

            //判断路径组中是否可行
            std::vector<FrenetPath> checked_path_set;
            checkPath(checked_path_set, global_path_set);
            //std::cout<< "checked_path_set number "<<checked_path_set.size() << std::endl;

            if(checked_path_set.size()<=0){
                std::cout<< "checked_path_set number <= 0"<<std::endl;
                return;
            }

            // std::vector<FrenetPath> checked_path_set = global_path_set;


            //从其中选取一条代价最低的作为最终路径
            std::vector<float> cost_set;
            for(auto checked_path: checked_path_set){
                cost_set.push_back(checked_path.cost_);
                //std::cout<< "cost_set "<<checked_path.cost_ << std::endl;
            }
            auto min_cost = std::min_element(cost_set.begin(),cost_set.end());
            //std::cout<< "distance "<<std::distance(cost_set.begin(),min_cost) << std::endl;
            final_path = checked_path_set[std::distance(cost_set.begin(),min_cost)];

            //std::cout<<"local planning finised next pub the trajectory"<<std::endl;

            //新的下标
            int current_index = std::min(PATH_GAP, int(final_path.x_.size()) - 1);
            //记录当前点的朝向和曲率
            for(int i=0; i<current_index; i++){
                yaw_set.push_back(final_path.yaw_[i]);
                kappa_set.push_back(final_path.kappa_[i]);
                s_set.push_back(final_path.s_[i]);
            }

            expected_path_x = final_path.x_.back();
            expected_path_y = final_path.y_.back();

            if(sqrt(pow(current_x - expected_path_x, 2) + 
            pow(current_y-expected_path_y, 2)) < GOAL_SIZE){

                std::cout<<"到达局部轨迹的期望终点，进行下一段轨迹规划"<<std::endl;
                //得到新的起点
                current_s = final_path.s_[current_index];
                current_s_derivative = final_path.s_derivative_[current_index];
                current_s_2derivative = 0.0;
                current_d = final_path.d_[current_index];
                current_d_derivative= final_path.d_derivative_[current_index];
                current_d_2derivative = final_path.d_2derivative_[current_index];
            }

        }
    }
}

void FrenetPathPlanner::referenceCurvePlanning(){

  
    if(wx.size()>0 && reference_curve_num<1){
    
            clock_t curve_start, curve_end;
            curve_start = clock();
            
            reference_curve = CubicSpline2D(wx,wy);

            //参考曲线采样
            std::vector<float> reference_samples;
            
            float sample = reference_curve.get_s_().front();
            for(int i=0; sample < reference_curve.get_s_().back(); i++){
                reference_samples.push_back(sample);
                sample += 0.1;
            }
            reference_curve.calcPosition(reference_samples,reference_waypoints_x,reference_waypoints_y);

            if (reference_waypoints_x.size()>0 && reference_waypoints_y.size()>0)
                reference_curve_num = 1;
            
            curve_end = clock();
            //std::cout<<"reference_curve time = "<<double(curve_end-curve_start)/CLOCKS_PER_SEC<<"s"<<std::endl;  //输出时间（单位：ｓ）
    }

};

void FrenetPathPlanner::frenetOptimalTrajectoryPlanning(std::vector<FrenetPath>& frenet_path_set, 
                                        float & current_s,float & current_s_d, float & current_s_2d,
                                        float & current_d, float & current_d_d, float & current_d_2d){
    double T = MIN_T;
    for(int j=0; T<MAX_T + ESP; j++){
        

        //首先是生横向偏移随时间变化，变化量为最终横向偏移和时间长度
        float df_offset = -ROAD_WIDTH;
        for(int i=0; df_offset < ROAD_WIDTH; i++){
        

            clock_t start, end , end1, end2;
                
            start = clock();
            //根据输入参数进行五次多项式拟合，输入参数[d0, d0_d, d0_dd, df, df_d, df_dd] 加加速度最小
            QuinticPolynomial quintic_polynomial(current_d, current_d_d, current_d_2d, df_offset, 0.0, 0.0, T);

            

            //初始化一个轨迹对象；
            FrenetPath frenet_path;
            float sample_t = 0.0;

            for(int k=0; sample_t<T + ESP; k++){
                    
                    
                    frenet_path.t_.push_back(sample_t);
                    //横向偏移采样
                    frenet_path.d_.push_back(quintic_polynomial.calcValue(sample_t));
                    frenet_path.d_derivative_.push_back(quintic_polynomial.calcDerivation(sample_t));
                    frenet_path.d_2derivative_.push_back(quintic_polynomial.calc2Derivation(sample_t));
                    frenet_path.d_3derivative_.push_back(quintic_polynomial.calc3Derivation(sample_t));

                    sample_t = sample_t + T_GAP;
            }

            end = clock();
            //std::cout<<"quintic_polynomial planning time = "<<double(end-start)/CLOCKS_PER_SEC<<"s"<<std::endl;  //输出时间（单位：ｓ）

            //开始生成纵向偏移随时间变化，采用的是速度保持模式，因此只需要用四次多项式
            float sf_derivative = TARGET_VELOCITY - VELOCITY_GAP * VELOCITY_SAMPLE_N;
            for(int m=0; sf_derivative < TARGET_VELOCITY + VELOCITY_GAP * VELOCITY_SAMPLE_N + ESP; m++){
                

                start = clock();
                //根据输入参数进行四次多项式拟合（四次多项式不唯一，这里使用的一阶导数和二阶导数参数）
                QuarticPolynomial quartic_polynomial(current_s, current_s_d, current_s_2d, sf_derivative, 0.0, T);

                
                //std::cout<<"quartic planning time = "<<double(end-start)/CLOCKS_PER_SEC<<"s"<<std::endl;  //输出时间（单位：ｓ）
                //对曲线进行采样

                frenet_path.s_.clear();
                frenet_path.s_derivative_.clear();
                frenet_path.s_2derivative_.clear();
                frenet_path.s_3derivative_.clear();
                

                for(auto t: frenet_path.t_){
                    //纵向偏移采样
                    frenet_path.s_.push_back(quartic_polynomial.calcValue(t));
                    frenet_path.s_derivative_.push_back(quartic_polynomial.calcDerivation(t));
                    frenet_path.s_2derivative_.push_back(quartic_polynomial.calc2Derivation(t));
                    frenet_path.s_3derivative_.push_back(quartic_polynomial.calc3Derivation(t));
                }

                end = clock();
                //std::cout<<"QuarticPolynomial time = "<<double(end-start)/CLOCKS_PER_SEC<<"s"<<std::endl;  //输出时间（单位：ｓ）

                //得到轨迹在frenet系的全部信息
                //开始给轨迹进行评分
                //首先是横向评分，横向评分包括三部分
                //第一部分是jerk
                float lateral_jerk_cost = 0.0;
                for(auto d_3derivative :frenet_path.d_3derivative_){
                    lateral_jerk_cost += pow(d_3derivative, 2);
                }
                //第二部分是时间开销
                float lateral_time_cost = T;
                //第三部分是最终横向偏移的平方
                float lateral_offset_cost = pow(frenet_path.d_.back(),2);
                //对以上三部分进行加权求和
                float lateral_cost = KJ * lateral_jerk_cost + KT * lateral_time_cost + KD * lateral_offset_cost;
                //接下来是纵向评分
                //纵向评分有三个部分
                //第一部分是jerk
                float longitudinal_jerk_cost = 0.0;
                for(auto s_3derivative :frenet_path.s_3derivative_){
                    longitudinal_jerk_cost += pow(s_3derivative, 2);
                }
                //第二部分是时间开销
                float longitudinal_time_cost = T;
                //第三部分是最终速度偏差
                float longitudinal_sderivative_offset = pow(frenet_path.s_derivative_.back()-TARGET_VELOCITY, 2);
                //对以上部分进行加权求和
                float longitudinal_cost = KJ * longitudinal_jerk_cost + KT * longitudinal_time_cost + KD * longitudinal_sderivative_offset;
                //求解最终评分
                frenet_path.cost_ = KLAT * lateral_cost + KLON * longitudinal_cost;
                //std::cout<<"cost_" << frenet_path.cost_ << std::endl;
                // 保存曲线到列表
                frenet_path_set.push_back(frenet_path);

                sf_derivative = sf_derivative + VELOCITY_GAP;
            }

            df_offset = df_offset + WIDTH_GAP;
        }
        T = T + T_GAP;
    }

}

// frenet坐标系到世界坐标系的转化
void FrenetPathPlanner::frenetToGloabalTrajectory(std::vector<FrenetPath>& frenet_path_set, 
                               std::vector<FrenetPath>& global_path_set, 
                               CubicSpline2D &reference_curve){
    
    for(auto frenet_path : frenet_path_set){
        FrenetPath global_path;
        //类对象赋值
        global_path = frenet_path;
        //第一步补全位置信息x和y
        //clock_t start, end , end1, end2;
        //start = clock();
        std::vector<float> rfc_x, rfc_y, rfc_yaw;
        //得到的是参考线的x y yaw
        reference_curve.calcPosition(global_path.s_, rfc_x, rfc_y);
        reference_curve.calcYaw(global_path.s_, rfc_yaw);

        //end = clock();
        //std::cout<<"frenetToGloabalTrajectory time = "<<double(end-start)/CLOCKS_PER_SEC<<"s"<<std::endl;  //输出时间（单位：ｓ）

        for(int i=0; i<rfc_x.size(); i++){
            float x = rfc_x[i] - sin(rfc_yaw[i]) * frenet_path.d_[i];
            float y = rfc_y[i] + cos(rfc_yaw[i]) * frenet_path.d_[i];
            global_path.x_.push_back(x);
            global_path.y_.push_back(y);

            float x_vel = global_path.s_derivative_[i] * cos(rfc_yaw[i]) - global_path.d_derivative_[i] * sin(rfc_yaw[i]);
            float y_vel = global_path.s_derivative_[i] * sin(rfc_yaw[i]) + global_path.d_derivative_[i] * cos(rfc_yaw[i]);
            global_path.x_derivative.push_back(x_vel);
            global_path.y_derivative.push_back(y_vel);

        }
        //完成x,y信息的补全后通过计算的方法得到yaw和ds，计算方法为yaw=atan2(dy, dx)
        for(int i=0; i<global_path.x_.size() - 1; i++ ){
            float yaw = atan2(global_path.y_[i + 1] - global_path.y_[i], global_path.x_[i + 1] - global_path.x_[i]);
            global_path.yaw_.push_back(yaw);
            float dis = sqrt(pow((global_path.y_[i + 1] - global_path.y_[i]), 2) + pow((global_path.x_[i + 1] - global_path.x_[i]), 2) );
            global_path.dis_.push_back(dis);
        }
        global_path.yaw_.push_back(global_path.yaw_.back());
        // 完成yaw的补全后通过计算得到kappa，计算方法为kappa=dyaw/ds
        for(int i=0; i<global_path.yaw_.size()-1; i++){
            float kappa = (global_path.yaw_[i + 1] - global_path.yaw_[i]) / global_path.dis_[i];
            global_path.kappa_.push_back(kappa);
        }
        global_path.kappa_.push_back(global_path.kappa_.back());
        global_path_set.push_back(global_path);
    }
}

// bool FrenetPathPlanner::isCollision(FrenetPath &path, std::vector<std::vector<float> > obstacles){
//     for(int i=0; i<path.x_.size(); i++){
//         for(auto obstacle: obstacles){
//             float dis = std::sqrt(pow(path.x_[i]-obstacle[0],2) + pow(path.y_[i]-obstacle[1],2));
//             if (dis <= ROBOT_SIZE)
//                 return true;
//         }
//     }
//     return false;
// }

bool FrenetPathPlanner::isCollision(FrenetPath &path){
    for(int i=0; i<path.x_.size(); i++){

        // 计算对应的栅格坐标
        std::pair<int,int> grid_point;
        grid_point = grid_map.getGridMapCoordinate(path.x_[i],path.y_[i]);
        int point_x; 
        int point_y;

        int range = std::round(ROBOT_SIZE/grid_map.getResolution());//避障的范围
        for(int j=-range; j<= range; j++){
            
            for(int k= -range; k<= range; k++){
                point_x = grid_point.first + j;
                point_y = grid_point.second + k;

                //是否越界
                if(!grid_map.isVerify(point_x, point_y)){
                    return true;
                }  
                //是否和障碍物碰撞
                if(grid_map.isOccupied(grid_map.getIndex(point_x, point_y))){
                    return true;
                }    
            }

        }
    }
    return false;
}

//判断路径组中路径是否可行
void FrenetPathPlanner::checkPath(std::vector<FrenetPath>& checked_path_set, std::vector<FrenetPath>& oringin_path_set){
    //判断路径是否可行存在四个方面
    for(int i=0; i<oringin_path_set.size(); i++){
        //第一个方面判断是否有大于最大速度的速度
        float max_velocity = *std::max_element(oringin_path_set[i].s_derivative_.begin(),oringin_path_set[i].s_derivative_.end());
        float min_velocity = *std::min_element(oringin_path_set[i].s_derivative_.begin(),oringin_path_set[i].s_derivative_.end());
        if(max_velocity > MAX_VELOCITY || min_velocity < -MAX_VELOCITY)
            continue;
        //第二个方面判断是否有大于最大曲率的曲率
        float max_kappa = *std::max_element(oringin_path_set[i].kappa_.begin(),oringin_path_set[i].kappa_.end());
        float min_kappa = *std::min_element(oringin_path_set[i].kappa_.begin(),oringin_path_set[i].kappa_.end());
        if(max_kappa > MAX_KAPPA || min_kappa < -MAX_KAPPA)
            continue;
        //第三个方面判断是否有大于最大加速度的加速度
        float max_acceleration = *std::max_element(oringin_path_set[i].s_2derivative_.begin(),oringin_path_set[i].s_2derivative_.end());
        float min_acceleration = *std::min_element(oringin_path_set[i].s_2derivative_.begin(),oringin_path_set[i].s_2derivative_.end());
        if(max_acceleration > MAX_ACCELERATION || min_acceleration < -MAX_ACCELERATION)
            continue;
        //第四个方面，判断是否与障碍物存在碰撞
        if(isCollision(oringin_path_set[i]))
            continue;
        checked_path_set.push_back(oringin_path_set[i]);
    }
}

//全局路径回调函数
void FrenetPathPlanner::PathCallback(const nav_msgs::Path::ConstPtr path){

    wx.clear();
    wy.clear();
    for(auto point_pose: path->poses){
        wx.push_back(point_pose.pose.position.x);
        wy.push_back(point_pose.pose.position.y);  
    }

}
//代价地图回调函数
void FrenetPathPlanner::MapCallback(const nav_msgs::OccupancyGridConstPtr map){
     
     occupancy_map = *map;
     grid_map = GridMap(occupancy_map);

}

//位姿回调函数
void FrenetPathPlanner::PoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr pose){

    current_x = pose->pose.pose.position.x;
    current_y = pose->pose.pose.position.y;
    // current_x = pose->pose.pose.position.x;
    // current_y = pose->pose.pose.position.y;
};

//速度回调函数
void FrenetPathPlanner::TwistCallback(const geometry_msgs::Twist::ConstPtr twist){

    //current_s_derivative = twist->linear.x;
    //current_d_derivative = twist->linear.y;
};

void FrenetPathPlanner::TrajectoryPub(){
    
        // 轨迹的路径输出可视化
        nav_msgs::Path trajectory_path;
        trajectory_path.header.frame_id = "map";
        trajectory_path.header.stamp = ros::Time::now();
        for (size_t i = 0; i < final_path.x_.size(); i++) {
            geometry_msgs::PoseStamped pose;
            pose.header = trajectory_path.header;
            pose.pose.position.x = final_path.x_[i];
            pose.pose.position.y = final_path.y_[i];
            trajectory_path.poses.push_back(pose);
        }
        trajectory_path_pub.publish(trajectory_path);

        //轨迹输出
        trajectory_msgs::JointTrajectory trajectory;
        trajectory.header.frame_id = "map";
        trajectory.header.stamp = ros::Time::now();
        for(size_t i=0; i< final_path.x_.size(); i++){
            trajectory_msgs::JointTrajectoryPoint trajectory_point;
            trajectory_point.positions.push_back(final_path.x_[i]);
            trajectory_point.positions.push_back(final_path.y_[i]);
            trajectory_point.positions.push_back(final_path.yaw_[i]);
            trajectory_point.positions.push_back(final_path.kappa_[i]);
            

            trajectory_point.velocities.push_back(final_path.x_derivative[i]);
            trajectory_point.velocities.push_back(final_path.y_derivative[i]);
            //trajectory_point.velocities.push_back(final_path.yaw_derivative[i]);
            
            trajectory_point.time_from_start = ros::Duration(final_path.t_[i]);
            trajectory.points.push_back(trajectory_point);
        }
         trajectory_pub.publish(trajectory);
};

void FrenetPathPlanner::ReferenceCurvePub(){

    // 输出可视化
    nav_msgs::Path reference_path;
    reference_path.header.frame_id = "map";
    reference_path.header.stamp = ros::Time::now();
    for (size_t i = 0; i < reference_waypoints_x.size(); i++) {
        geometry_msgs::PoseStamped pose;
        pose.header = reference_path.header;
        pose.pose.position.x = reference_waypoints_x[i];
        pose.pose.position.y = reference_waypoints_y[i];
        //std::cout<<"reference_waypoints_x" << reference_waypoints_x[i] <<std::endl;
        //std::cout<<"reference_waypoints_y" << reference_waypoints_y[i] <<std::endl;
        reference_path.poses.push_back(pose);
    }
    reference_curve_pub.publish(reference_path);

};

void FrenetPathPlanner::initPlanner(){
    // this->wx.clear();
    // this->wy.clear();
    // current_s = current_s_derivative = current_s_2derivative = 0;  //current_s_derivative不一定
    // current_d = current_d_derivative = current_d_2derivative = 0;
    // reference_waypoints_x.clear();
    // reference_waypoints_y.clear();
    reference_curve_num = 0;
}

//change 原来的版本是只进行一次全局规划 全局规划三次样条后作为参考线（Frenet坐标系） 在整个导航过程中Frenet坐标系不变 
//小车到达规划出的局部路径的终点后在将终点作为下一段规划的起点继续规划
//这样做需要控制误差较小（能够保证精确控制到局部目标点附近） 并且在到达局部路径的终点之前不会重新规划 一直都是输出同一条局部路径 小车并不在路径的起点上

// 随着全局规划更新 参考线也更新
// 控制频率 > 局部频率 > 全局频率