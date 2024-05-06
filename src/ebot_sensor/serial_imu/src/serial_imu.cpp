//serial_imu.cpp
#include <ros/ros.h>
#include <serial/serial.h>
#include <iostream>
#include <stdio.h>
#include <sys/select.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <pthread.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>

#ifdef __cplusplus
extern "C"{
#endif
#include <stdint.h>
#include <stdbool.h>
#include "packet.h"
#include "imu_data_decode.h"
int imu_data_decode_init(void);
typedef void (*on_data_received_event)(packet_t *ptr);
void packet_decode_init(packet_t *pkt, on_data_received_event rx_handler);
uint32_t packet_decode(uint8_t);

#ifdef __cplusplus
}
#endif

void dump_data_packet(receive_imusol_packet_t *data);

static int frame_rate;

static uint8_t buf[2048];

static sensor_msgs::Imu imu_msg;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "serial_imu");
    //创建句柄（虽然后面没用到这个句柄，但如果不创建，运行时进程会出错）
    ros::NodeHandle n;
	ros::NodeHandle nh_private("~");

	std::string port_name;
	std::string frame_id;
	int count = 0;

	nh_private.param<std::string>("PortName",port_name,"/dev/ttyUSB0");
	nh_private.param<std::string>("frame_id",frame_id,"/imu");

	ros::Publisher imu_pub = n.advertise<sensor_msgs::Imu>("imu", 1000);

    //创建一个serial类
    serial::Serial sp;
    //创建timeout
    serial::Timeout to = serial::Timeout::simpleTimeout(100);
    //设置要打开的串口名称
    sp.setPort(port_name);
    //设置串口通信的波特率
    sp.setBaudrate(115200);
    //串口设置timeout
    sp.setTimeout(to);
	
	imu_data_decode_init();
 
    try
    {
        //打开串口
        sp.open();
    }
    catch(serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port.");
        return -1;
    }
    
    //判断串口是否打开成功
    if(sp.isOpen())
    {
        ROS_INFO_STREAM(port_name+" is opened.");
    }
    else
    {
        return -1;
    }
	
    ros::Rate loop_rate(1000);

    while(ros::ok())
    {
		imu_msg.header.stamp = ros::Time::now();
		imu_msg.header.seq = count;
		imu_msg.header.frame_id =  frame_id;

        //获取缓冲区内的字节数r
		size_t n = sp.available();
        if(n!=0)
        {
            uint8_t buffer[1024];
            //读出数据
            n = sp.read(buffer, n);
            if(n > 0)
			{
				for(int i = 0; i < n; i++)
					packet_decode(buffer[i]);
				printf("------------------------\n");
				if(receive_gwsol.tag != KItemGWSOL)
				{
					dump_data_packet(&receive_imusol);
				}
				else
				{
					printf("       GW ID: %4d\n",receive_gwsol.gw_id);
					for(int i = 0; i < receive_gwsol.n; i++)
					{
						dump_data_packet(&receive_gwsol.receive_imusol[i]);
					}
				}
			}
			count++;
			imu_pub.publish(imu_msg);
        }
        loop_rate.sleep();
    }
    
	//关闭串口
	sp.close();
 
	return 0;
}

void dump_data_packet(receive_imusol_packet_t *data)
{
	if(bitmap & BIT_VALID_ID)
	{
		printf("     Devie ID:%6d\n",data->id);
	}
	if(bitmap & BIT_VALID_TIMES)
	{
		printf("    Run times:%-8d\n",data->times % 1000);
	}
	if(bitmap & BIT_VALID_ACC)
	{
		printf("       Acc(G):%8.3f %8.3f %8.3f\r\n", data->acc[0], data->acc[1], data->acc[2]);
		imu_msg.linear_acceleration.x = data->acc[0] * 9.794;
		imu_msg.linear_acceleration.y = data->acc[1] * 9.794;
		imu_msg.linear_acceleration.z = data->acc[2] * 9.794;
	}
	if(bitmap & BIT_VALID_GYR)
	{
		printf("   Gyr(deg/s):%8.2f %8.2f %8.2f\r\n", data->gyr[0], data->gyr[1], data->gyr[2]);
		imu_msg.angular_velocity.x = data->gyr[0] * M_PI /180.0;
		imu_msg.angular_velocity.y = data->gyr[1] * M_PI /180.0;
		imu_msg.angular_velocity.z = data->gyr[2] * M_PI /180.0;
	}
	if(bitmap & BIT_VALID_MAG)
	{
		printf("      Mag(uT):%8.2f %8.2f %8.2f\r\n", data->mag[0], data->mag[1], data->mag[2]);
	}
	if(bitmap & BIT_VALID_EUL)
	{
		printf("   Eul(R P Y):%8.2f %8.2f %8.2f\r\n", data->eul[0], data->eul[1], data->eul[2]);
		geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromRollPitchYaw(data->eul[0], data->eul[1], data->eul[2]);
		imu_msg.orientation = quat;
	}
	if(bitmap & BIT_VALID_QUAT)
	{
		printf("Quat(W X Y Z):%8.3f %8.3f %8.3f %8.3f\r\n", data->quat[0], data->quat[1], data->quat[2], data->quat[3]);
		imu_msg.orientation.w = data->quat[0];
		imu_msg.orientation.x = data->quat[1];
		imu_msg.orientation.y = data->quat[2];
		imu_msg.orientation.z = data->quat[3];
	}
}
