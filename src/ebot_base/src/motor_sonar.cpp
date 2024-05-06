#include <ros/ros.h>
#include <serial/serial.h>
#include <iostream>
#include "control/crc_m.h"
#include "control/motor.h"
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Range.h>
#include <geometry_msgs/TwistStamped.h>
#include <boost/lexical_cast.hpp>


using namespace std;

float speed = 0;
float angle=0;
bool msg_flag = false;
float  radar_result=-1;
float  chassis_speed=-1;
float  chassis_orientation=-1;
serial::Serial sp;
ros::Publisher radar_result_pub;
ros::Publisher sonar_result_pub;
ros::Publisher chassis_status_pub;
ros::Publisher battery_voltage_pub;
bool rec_radar_flag = false;
bool vec_break_flag = 0;
bool BREAK_ALLOWED = 1;
u_int8_t code[14];
geometry_msgs::TwistStamped chassis;

//超声波数据订阅
float radar_data = -1;

void radar_callback(const sensor_msgs::Range::ConstPtr &msg) {
    radar_data = msg->range*100;
}
///////////////////////////////////////////////////////////


// 接收控制指令
void control_callback(const geometry_msgs::Twist::ConstPtr &msg)
{
    msg_flag = true;
    // speed =  msg->twist.linear.x;
    // angle =  msg->twist.angular.x;
    speed =    -(msg->linear.x);
    angle =   -(msg->angular.z) + 3;
    std::cout<<"speed "<< speed <<std::endl;
}

// 打开串口
int open_port(const string &port_name)
{
    serial::Timeout to = serial::Timeout::simpleTimeout(100);
    sp.setPort(port_name);
    sp.setBaudrate(115200);
    sp.setTimeout(to);
    try
    {
        sp.open();
    }
    catch (serial::IOException &e)
    {
        ROS_ERROR_STREAM("Unable to open port: " << port_name);
        return -1;
    }
    if (sp.isOpen())
    {
        ROS_INFO_STREAM(port_name << " is Opened.");
    }
    else
    {
        return -1;
    }
}

// 处理串口接收到的底盘信息
void frameDataParsing(std::vector<uint8_t> frame) {
    uint8_t cmd_id = frame[3];

    switch (cmd_id) {
        case 0x21 : {
            std::array<uint8_t, 4> data;
            memcpy(&data[0], &frame[4], sizeof(data));
            memcpy(&radar_result, &data[0], sizeof(radar_result));
            rec_radar_flag = true;
            break;
        }
        case 0x22 : {
            std::array<uint8_t, 8> data;
            memcpy(&data[0], &frame[4], sizeof(data));
            memcpy(&chassis_speed, &data[0], sizeof(chassis_speed));
            memcpy(&chassis_orientation, &data[4], sizeof(chassis_orientation));
            chassis.header.stamp=ros::Time::now();
            chassis.twist.angular.z= -(chassis_orientation + 8);
            chassis.twist.linear.x=chassis_speed;
            chassis_status_pub.publish(chassis);
	    std::cout<< "chassis_speed " << chassis_speed<<' '<<"chassis_orientation "<< chassis_orientation<<std::endl;
            break;
        }
        case 0x13 : {
            std::array<uint8_t, 4> data;
            std_msgs::Float32 battery_voltage;
            memcpy(&data[0], &frame[4], sizeof(data));
            // parseBatteryVoltage(data, &battery_voltage);
            memcpy(&battery_voltage, &data[0], sizeof(battery_voltage));    // 电池电压采样值
            // std::cout<< battery_voltage<<std::endl;
            battery_voltage_pub.publish(battery_voltage);
            break;
        }
        default:
            rec_radar_flag = false;
            break;
    }

    /*原程序
    if(radar_result < 30 ){
         vec_break_flag =1;
         }else{
             vec_break_flag = 0;
         }
    */

   //rostopic读取
   if(radar_data < 30 ){
         vec_break_flag =1;
         }else{
             vec_break_flag = 0;
         }
}
// 校验串口接收到的数据
void unpackReceivedData(std::vector<uint8_t>* buffer) {
    int length = 0;
    std::vector<uint8_t> frame;
    UnpackStep unpack_step = UnpackStep::HEADER_SOF;

    // 确保buffer中存在完整的一帧
    while (buffer->size() >=2 && buffer->size() >= length) {
        switch (unpack_step) {
            case UnpackStep::HEADER_SOF:
                if ((*buffer)[0] == FRAME_HEADER_SOF) {
                    unpack_step = UnpackStep::HEADER_LEN;
                } else {
                    buffer->erase(buffer->begin());
                    frame.clear();
                    length = 0;
                }
                break;

            case UnpackStep::HEADER_LEN:
                length = (*buffer)[1] + FRAME_HEADER_LEN + FRAME_TAIL_LEN + FRAME_CMD_LEN;
                frame.insert(frame.end(), buffer->begin(), buffer->begin() + length);
                unpack_step = UnpackStep::HEADER_CRC8;
                break;

            case UnpackStep::HEADER_CRC8:
                if (verifyCRC8Checksum(frame, FRAME_HEADER_LEN)) {
                    unpack_step = UnpackStep::TAIL_CRC16;
                } else {
                    buffer->erase(buffer->begin());
                    frame.clear();
                    length = 0;
                    unpack_step = UnpackStep::HEADER_SOF;
                    ROS_WARN_STREAM("CRC8 check failed");
                }
                break;

            case UnpackStep::TAIL_CRC16:
                if (verifyCRC16Checksum(frame, length)) {
                    frameDataParsing(frame);
                } else {
                    ROS_WARN_STREAM("CRC16 check failed");
                }
                buffer->erase(buffer->begin(), buffer->begin()+length);
                frame.clear();
                length = 0;
                unpack_step = UnpackStep::HEADER_SOF;
                break;

            default:
                frame.clear();
                length = 0;
                unpack_step = UnpackStep::HEADER_SOF;
                break;
        }
    }
}
// 读取串口数据
void read_serial(void)
{
    std::vector<uint8_t> unpack_buffer;
    size_t read_size=sp.available();
    if(read_size){
        std::vector<uint8_t> recv_buffer;
        read_size = sp.read(recv_buffer, read_size);
        unpack_buffer.insert(unpack_buffer.end(), recv_buffer.begin(), recv_buffer.end());
        unpackReceivedData(&unpack_buffer);
    }
}
// 编码发送给底盘的控制指令
void control_coding(const float Speed, const float Angle){
    // /* code */
        uint8_t data[8];
        uint8_t header[3];
        uint8_t cmd = 0x02;  //标志位
        unsigned long bin = ((unsigned long *)&Speed)[0];
        unsigned long bin1 = ((unsigned long *)&Angle)[0];

        data[0] = bin & 0x000000ff;
        data[1] = (bin & 0x0000ff00) >> 8;
        data[2] = (bin & 0x00ff0000) >> 16;
        data[3] = (bin & 0xff000000) >> 24;
        data[4] = bin1 & 0x000000ff;
        data[5] = (bin1 & 0x0000ff00) >> 8;
        data[6] = (bin1 & 0x00ff0000) >> 16;
        data[7] = (bin1 & 0xff000000) >> 24;

        header[0] = 0xa5;
        header[1] = 0x08;
        appendCRC8Checksum(header, 3);

        for (int i = 0; i < 3; i++)
            code[i] = header[i];    //header

        code[3] = cmd;    //cmd

        for (int i = 0; i < 8; i++)
            code[i + 4] = data[i];   //data
        appendCRC16Checksum(code, 14);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "control_serial_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    std::string port_name;
    std::string control_topic;
    std::string radar_result_topic;
    std::string chassis_status_topic;
    std::string battery_voltage_topic;
    std::string sonar_result_topic;
    nh_private.param<std::string>("PortName", port_name, "/dev/ttyUSB0");
    nh_private.param<std::string>("control_topic", control_topic, "/robot/control");
    nh_private.param<std::string>("radar_result_topic", radar_result_topic, "/robot/radar_result");
    nh_private.param<std::string>("chassis_status", chassis_status_topic, "/robot/chassis_status");
    nh_private.param<std::string>("battery_voltage", battery_voltage_topic, "/robot/battery_voltage");
    nh_private.param<std::string>("sonar_result", sonar_result_topic, "/robot/sonar");

    // ros::Subscriber motor_control_sub = nh.subscribe<geometry_msgs::TwistStamped>(control_topic, 100, control_callback);//订阅控制指令
    ros::Subscriber motor_control_sub = nh.subscribe<geometry_msgs::Twist>(control_topic, 100, control_callback);//订阅控制指令
    radar_result_pub = nh.advertise<std_msgs::Float32>(radar_result_topic,100);//发布超声波测距结果
    sonar_result_pub = nh.advertise<sensor_msgs::Range>(sonar_result_topic,100);//发布超声波测距结果
    chassis_status_pub = nh.advertise<geometry_msgs::TwistStamped>(chassis_status_topic,100);// 发布车辆速度及角度
    battery_voltage_pub = nh.advertise<std_msgs::Float32>(battery_voltage_topic,100);// 发布车辆状态

    ros::Subscriber radar_sub = nh.subscribe<sensor_msgs::Range>("/ultrasonic_sensors/out/front_left", 100, radar_callback);

    // 打开串口
    open_port(port_name);

    ros::Rate loop_rate(20);
    int cnt = 0;
    float SPEED=0,ANGLE=0;
    // deprecated
    std_msgs::Float32 radar_result_msg;
    sensor_msgs::Range radar_range_msg;
    radar_range_msg.header.frame_id = "sonar";
    radar_range_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
    radar_range_msg.min_range = 0;
    radar_range_msg.max_range = 0.84;
    radar_range_msg.field_of_view = M_PI / 1.8;

    while (ros::ok())
    {
        //std::cout<< vec_break_flag <<std::endl;
        // 紧急制动 若紧急制动标志位不为1或不允许紧急制动，则数十位数后启动车辆
        if (!vec_break_flag | !BREAK_ALLOWED)
        {
            if (cnt > 10)
            {
                cnt = 10;
                SPEED = speed;
                ANGLE = angle;
            }
            else{ANGLE = angle;}
        }
        else
        {
            SPEED = (speed < 0) ? 0 : speed; // 前方有障碍物，禁止前进
            ANGLE = angle;
            cnt = 0;
        }
        //std::cout<< SPEED<<" "<<std::endl;
        // 编码控制指令
        control_coding(SPEED, ANGLE);
        // 将编码后的指令写入串口
        sp.write(code, 14);

        //发布接收到的超声波测距结果，若未接收到则值未-1
        // deprecated
        if(rec_radar_flag == false){ radar_result = -1; }
        radar_result_msg.data=radar_result;
        radar_result_pub.publish(radar_result_msg);

        if (rec_radar_flag) {
            radar_range_msg.header.stamp = ros::Time::now();
            radar_range_msg.range = radar_result / 100;
            sonar_result_pub.publish(radar_range_msg);
        }

        msg_flag = false;
        ros::spinOnce();
        loop_rate.sleep();

        read_serial();
        cnt++;
    }

    sp.close();

    return 0;
}
